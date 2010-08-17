/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>  // Precompiled headers

#include <mrpt/poses/CNetworkOfPoses.h>
#include <mrpt/math/dijkstra.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::system;


namespace mrpt
{
	namespace poses
	{
		namespace detail
		{
			template <class POSE> void write_VERTEX_line(const TNodeID id, const POSE &p, std::ofstream &f);
			template <> void write_VERTEX_line<CPose2D>(const TNodeID id, const CPose2D &p, std::ofstream &f)
			{
				//  VERTEX2 id x y phi
				f << "VERTEX2 " << id << format(" %.04f %.04f %.04f\n",p.x(),p.y(),p.phi() );
			}
			template <> void write_VERTEX_line<CPose3D>(const TNodeID id, const CPose3D &p, std::ofstream &f)
			{
				//  VERTEX3 id x y z roll pitch yaw
				// **CAUTION** In the TORO graph format angles are in the RPY order vs. MRPT's YPR.
				f << "VERTEX3 " << id << format(" %.04f %.04f %.04f %.04f %.04f %.04f\n",p.x(),p.y(),p.z(),p.roll(),p.pitch(),p.yaw() );
			}

			template <class EDGE> void write_EDGE_line( const std::pair<TNodeID,TNodeID> &edgeIDs,const EDGE & edge, std::ofstream &f);
			template <> void write_EDGE_line<CPosePDFGaussianInf>( const std::pair<TNodeID,TNodeID> &edgeIDs,const CPosePDFGaussianInf & edge, std::ofstream &f)
			{
				//  EDGE2 to_id from_id Ax Ay Aphi inf_xx inf_xy inf_yy inf_pp inf_xp inf_yp
				f << "EDGE2 " << edgeIDs.second << " " << edgeIDs.first << " " <<
					//format(" %.06f %.06f %.06f %e %e %e %e %e %e\n",
						edge.mean.x()<<" "<<edge.mean.y()<<" "<<edge.mean.phi()<<" "<<
						edge.cov_inv(0,0)<<" "<<edge.cov_inv(0,1)<<" "<<edge.cov_inv(1,1)<<" "<<
						edge.cov_inv(2,2)<<" "<<edge.cov_inv(0,2)<<" "<<edge.cov_inv(1,2) << endl;
			}
			template <> void write_EDGE_line<CPose3DPDFGaussianInf>( const std::pair<TNodeID,TNodeID> &edgeIDs,const CPose3DPDFGaussianInf & edge, std::ofstream &f)
			{
				//  EDGE3 to_id from_id Ax Ay Az Aroll Apitch Ayaw inf_11 inf_12 .. inf_16 inf_22 .. inf_66
				// **CAUTION** In the TORO graph format angles are in the RPY order vs. MRPT's YPR.
				f << "EDGE3 " << edgeIDs.second << " " << edgeIDs.first << " " <<
					//format(" %.06f %.06f %.06f %.06f %.06f %.06f %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e\n",
						edge.mean.x()<<" "<<edge.mean.y()<<" "<<edge.mean.z()<<" "<<
						edge.mean.roll()<<" "<<edge.mean.pitch()<<" "<<edge.mean.yaw()<<" "<<
						edge.cov_inv(0,0)<<" "<<edge.cov_inv(0,1)<<" "<<edge.cov_inv(0,2)<<" "<<edge.cov_inv(0,5)<<" "<<edge.cov_inv(0,4)<<" "<<edge.cov_inv(0,3)<<" "<<
						edge.cov_inv(1,1)<<" "<<edge.cov_inv(1,2)<<" "<<edge.cov_inv(1,5)<<" "<<edge.cov_inv(1,4)<<" "<<edge.cov_inv(1,3)<<" "<<
						edge.cov_inv(2,2)<<" "<<edge.cov_inv(2,5)<<" "<<edge.cov_inv(2,4)<<" "<<edge.cov_inv(2,3)<<" "<<
						edge.cov_inv(5,5)<<" "<<edge.cov_inv(5,4)<<" "<<edge.cov_inv(5,3)<<" "<<
						edge.cov_inv(4,4)<<" "<<edge.cov_inv(4,3)<<" "<<
						edge.cov_inv(3,3) << endl;
			}
			template <> void write_EDGE_line<CPosePDFGaussian>( const std::pair<TNodeID,TNodeID> &edgeIDs,const CPosePDFGaussian & edge, std::ofstream &f)
			{
				CPosePDFGaussianInf p;
				p.copyFrom(edge);
				write_EDGE_line(edgeIDs,p,f);
			}
			template <> void write_EDGE_line<CPose3DPDFGaussian>( const std::pair<TNodeID,TNodeID> &edgeIDs,const CPose3DPDFGaussian & edge, std::ofstream &f)
			{
				CPose3DPDFGaussianInf p;
				p.copyFrom(edge);
				write_EDGE_line(edgeIDs,p,f);
			}

			// =================================================================
			//                     save_graph_of_poses_from_text_file
			// =================================================================
			template<class CPOSE>
			void save_graph_of_poses_from_text_file(const CNetworkOfPoses<CPOSE> *g, const std::string &fil)
			{
				std::ofstream  f;
				f.open(fil.c_str());
				if (!f.is_open())
					THROW_EXCEPTION_CUSTOM_MSG1("Error opening file '%s' for writing",fil.c_str());

				// 1st: Nodes
				for (typename CNetworkOfPoses<CPOSE>::global_poses_t::const_iterator itNod = g->nodes.begin();itNod!=g->nodes.end();++itNod)
					write_VERTEX_line(itNod->first, itNod->second, f);

				// 2nd: Edges:
				for (typename CNetworkOfPoses<CPOSE>::const_iterator it=g->begin();it!=g->end();++it)
					if (it->first.first!=it->first.second)	// Ignore self-edges, typically from importing files with EQUIV's
						write_EDGE_line( it->first, it->second, f);

			} // end save_graph


			// Auxiliary method to determine 2D/3D graph type:
			//  Only the specializations below are really defined.
			template <class CPOSE> bool is_3D_graph_from_edge_type();

			template <> bool is_3D_graph_from_edge_type<CPosePDFGaussian>()   { return false; }
			template <> bool is_3D_graph_from_edge_type<CPosePDFGaussianInf>()   { return false; }

			template <> bool is_3D_graph_from_edge_type<CPose3DPDFGaussian>() { return true;  }
			template <> bool is_3D_graph_from_edge_type<CPose3DPDFGaussianInf>() { return true;  }

			// =================================================================
			//                     load_graph_of_poses_from_text_file
			// =================================================================
			template<class CPOSE>
			void load_graph_of_poses_from_text_file(CNetworkOfPoses<CPOSE>*g, const std::string &fil)
			{
				set<string>  alreadyWarnedUnknowns; // for unknown line types, show a warning to cerr just once.

				// First, empty the graph:
				g->clear();

				// Determine if it's a 2D or 3D graph, just to raise an error if loading a 3D graph in a 2D one, since
				//  it would be an unintentional loss of information:
				const bool graph_is_3D = is_3D_graph_from_edge_type<CPOSE>();


				std::ifstream  f;
				f.open(fil.c_str());
				if (!f.is_open())
					THROW_EXCEPTION_CUSTOM_MSG1("Error opening file '%s' for reading",fil.c_str());

				// -------------------------------------------
				// 1st PASS: Read EQUIV entries only
				//  since processing them AFTER loading the data
				//  is much much slower.
				// -------------------------------------------
				map<TNodeID,TNodeID> lstEquivs; // List of EQUIV entries: NODEID -> NEWNODEID. NEWNODEID will be always the lowest ID number.

				// Read & process lines each at once until EOF:
				for (unsigned int lineNum = 1; !f.fail();  ++lineNum )
				{
					string lin;
					getline(f,lin);

					lin = mrpt::system::trim(lin);
					if (lin.empty()) continue; // Ignore empty lines.

					// Ignore comments lines, starting with "#" or "//".
					if ( mrpt::system::strStarts(lin,"#") || mrpt::system::strStarts(lin,"//") )
						continue;

					// Parse the line as a string stream:
					istringstream s;
					s.str(lin);

					string key;
					if ( !(s >> key) || key.empty() )
						THROW_EXCEPTION(format("Line %u: Can't read string for entry type in: '%s'", lineNum, lin.c_str() ) );

					if ( strCmpI(key,"EQUIV") )
					{
						// Process these ones at the end, for now store in a list:
						TNodeID  id1,id2;
						if (!(s>> id1 >> id2))
							THROW_EXCEPTION(format("Line %u: Can't read id1 & id2 in EQUIV line: '%s'", lineNum, lin.c_str() ) );
						lstEquivs[std::max(id1,id2)] = std::min(id1,id2);
					}
				} // end 1st pass

				// -------------------------------------------
				// 2nd PASS: Read all other entries
				// -------------------------------------------
				f.clear();
				f.seekg(0);

				// Read & process lines each at once until EOF:
				for (unsigned int lineNum = 1; !f.fail();  ++lineNum )
				{
					string lin;
					getline(f,lin);

					lin = mrpt::system::trim(lin);
					if (lin.empty()) continue; // Ignore empty lines.

					// Ignore comments lines, starting with "#" or "//".
					if ( mrpt::system::strStarts(lin,"#") || mrpt::system::strStarts(lin,"//") )
						continue;

					// Parse the line as a string stream:
					istringstream s;
					s.str(lin);

					// Recognized strings:
					//  VERTEX2 id x y phi
					//  EDGE2 to_id from_id Ax Ay Aphi inf_xx inf_xy inf_yy inf_pp inf_xp inf_yp
					//  VERTEX3 id x y z roll pitch yaw
					//  EDGE3 to_id from_id Ax Ay Az Aroll Apitch Ayaw inf_11 inf_12 .. inf_16 inf_22 .. inf_66
					//  EQUIV id1 id2
					string key;
					if ( !(s >> key) || key.empty() )
						THROW_EXCEPTION(format("Line %u: Can't read string for entry type in: '%s'", lineNum, lin.c_str() ) );

					if ( strCmpI(key,"VERTEX2") )
					{
						TNodeID  id;
						TPose2D  p2D;
						if (!(s>> id >> p2D.x >> p2D.y >> p2D.phi))
							THROW_EXCEPTION(format("Line %u: Error parsing VERTEX2 line: '%s'", lineNum, lin.c_str() ) );

						// Make sure the node is new:
						if (g->nodes.find(id)!=g->nodes.end())
							THROW_EXCEPTION(format("Line %u: Error, duplicated verted ID %u in line: '%s'", lineNum, static_cast<unsigned int>(id), lin.c_str() ) );

						// EQUIV? Replace ID by new one.
						{
							const map<TNodeID,TNodeID>::const_iterator itEq = lstEquivs.find(id);
							if (itEq!=lstEquivs.end()) id = itEq->second;
						}

						// Add to map: ID -> absolute pose:
						if (g->nodes.find(id)==g->nodes.end())
						{
							typename CNetworkOfPoses<CPOSE>::contraint_t::type_value & newNode = g->nodes[id];
							newNode = CPose2D(p2D); // Auto converted to CPose3D if needed
						}
					}
					else if ( strCmpI(key,"VERTEX3") )
					{
						if (!graph_is_3D)
							THROW_EXCEPTION(format("Line %u: Try to load VERTEX3 into a 2D graph: '%s'", lineNum, lin.c_str() ) );

						//  VERTEX3 id x y z roll pitch yaw
						TNodeID  id;
						TPose3D  p3D;
						// **CAUTION** In the TORO graph format angles are in the RPY order vs. MRPT's YPR.
						if (!(s>> id >> p3D.x >> p3D.y >> p3D.z >> p3D.roll >> p3D.pitch >> p3D.yaw ))
							THROW_EXCEPTION(format("Line %u: Error parsing VERTEX3 line: '%s'", lineNum, lin.c_str() ) );

						// Make sure the node is new:
						if (g->nodes.find(id)!=g->nodes.end())
							THROW_EXCEPTION(format("Line %u: Error, duplicated verted ID %u in line: '%s'", lineNum, static_cast<unsigned int>(id), lin.c_str() ) );

						// EQUIV? Replace ID by new one.
						{
							const map<TNodeID,TNodeID>::const_iterator itEq = lstEquivs.find(id);
							if (itEq!=lstEquivs.end()) id = itEq->second;
						}

						// Add to map: ID -> absolute pose:
						if (g->nodes.find(id)==g->nodes.end())
						{
							g->nodes[id] = typename CNetworkOfPoses<CPOSE>::contraint_t::type_value( CPose3D(p3D) ); // Auto converted to CPose2D if needed
						}
					}
					else if ( strCmpI(key,"EDGE2") )
					{
						//  EDGE2 to_id from_id Ax Ay Aphi inf_xx inf_xy inf_yy inf_pp inf_xp inf_yp
						//                                   s00   s01     s11    s22    s02    s12
						//  Read values are:
						//    [ s00 s01 s02 ]
						//    [  -  s11 s12 ]
						//    [  -   -  s22 ]
						//
						TNodeID  to_id, from_id;
						if (!(s>> to_id >> from_id))
							THROW_EXCEPTION(format("Line %u: Error parsing EDGE2 line: '%s'", lineNum, lin.c_str() ) );

						// EQUIV? Replace ID by new one.
						{
							const map<TNodeID,TNodeID>::const_iterator itEq = lstEquivs.find(to_id);
							if (itEq!=lstEquivs.end()) to_id = itEq->second;
						}
						{
							const map<TNodeID,TNodeID>::const_iterator itEq = lstEquivs.find(from_id);
							if (itEq!=lstEquivs.end()) from_id = itEq->second;
						}

						TPose2D  Ap_mean;
						CMatrixDouble33 Ap_cov_inv;
						if (!(s>>
								Ap_mean.x >> Ap_mean.y >> Ap_mean.phi >>
								Ap_cov_inv(0,0) >> Ap_cov_inv(0,1) >> Ap_cov_inv(1,1) >>
								Ap_cov_inv(2,2) >> Ap_cov_inv(0,2) >> Ap_cov_inv(1,2) ))
							THROW_EXCEPTION(format("Line %u: Error parsing EDGE2 line: '%s'", lineNum, lin.c_str() ) );

						// Complete low triangular part of inf matrix:
						Ap_cov_inv(1,0) = Ap_cov_inv(0,1);
						Ap_cov_inv(2,0) = Ap_cov_inv(0,2);
						Ap_cov_inv(2,1) = Ap_cov_inv(1,2);

						// Convert to 2D cov, 3D cov or 3D inv_cov as needed:
						typename CNetworkOfPoses<CPOSE>::edge_t  newEdge;
						newEdge.copyFrom( CPosePDFGaussianInf( CPose2D(Ap_mean), Ap_cov_inv ) );
						g->insertEdge(from_id, to_id, newEdge);
					}
					else if ( strCmpI(key,"EDGE3") )
					{
						if (!graph_is_3D)
							THROW_EXCEPTION(format("Line %u: Try to load EDGE3 into a 2D graph: '%s'", lineNum, lin.c_str() ) );

						//  EDGE3 to_id from_id Ax Ay Az Aroll Apitch Ayaw inf_11 inf_12 .. inf_16 inf_22 .. inf_66
						TNodeID  to_id, from_id;
						if (!(s>> to_id >> from_id))
							THROW_EXCEPTION(format("Line %u: Error parsing EDGE3 line: '%s'", lineNum, lin.c_str() ) );

						// EQUIV? Replace ID by new one.
						{
							const map<TNodeID,TNodeID>::const_iterator itEq = lstEquivs.find(to_id);
							if (itEq!=lstEquivs.end()) to_id = itEq->second;
						}
						{
							const map<TNodeID,TNodeID>::const_iterator itEq = lstEquivs.find(from_id);
							if (itEq!=lstEquivs.end()) from_id = itEq->second;
						}

						TPose3D  Ap_mean;
						CMatrixDouble66 Ap_cov_inv;
						// **CAUTION** In the TORO graph format angles are in the RPY order vs. MRPT's YPR.
						if (!(s>> Ap_mean.x >> Ap_mean.y >> Ap_mean.z >> Ap_mean.roll >> Ap_mean.pitch >> Ap_mean.yaw ))
							THROW_EXCEPTION(format("Line %u: Error parsing EDGE3 line: '%s'", lineNum, lin.c_str() ) );

						// **CAUTION** Indices are shuffled to the change YAW(3) <-> ROLL(5) in the order of the data.
						if (!(s>>
								Ap_cov_inv(0,0) >> Ap_cov_inv(0,1) >> Ap_cov_inv(0,2) >> Ap_cov_inv(0,5) >> Ap_cov_inv(0,4) >> Ap_cov_inv(0,3) >>
								Ap_cov_inv(1,1) >> Ap_cov_inv(1,2) >> Ap_cov_inv(1,5) >> Ap_cov_inv(1,4) >> Ap_cov_inv(1,3) >>
								Ap_cov_inv(2,2) >> Ap_cov_inv(2,5) >> Ap_cov_inv(2,4) >> Ap_cov_inv(2,3) >>
								Ap_cov_inv(5,5) >> Ap_cov_inv(5,4) >> Ap_cov_inv(5,3) >>
								Ap_cov_inv(4,4) >> Ap_cov_inv(4,3) >>
								Ap_cov_inv(3,3) ))
						{
							// Cov may be omitted in the file:
							Ap_cov_inv.unit();

							if (alreadyWarnedUnknowns.find("MISSING_3D")==alreadyWarnedUnknowns.end())
							{
								alreadyWarnedUnknowns.insert("MISSING_3D");
								cerr << "[CNetworkOfPoses::loadFromTextFile] " << fil << ":" << lineNum << ": Warning: Information matrix missing, assuming unity.\n";
							}
						}
						else
						{
							// Complete low triangular part of inf matrix:
							for (size_t r=1;r<6;r++)
								for (size_t c=0;c<r;c++)
									Ap_cov_inv(r,c) = Ap_cov_inv(c,r);
						}

						// Convert as needed:
						typename CNetworkOfPoses<CPOSE>::edge_t  newEdge;
						newEdge.copyFrom( CPose3DPDFGaussianInf( CPose3D(Ap_mean), Ap_cov_inv ) );
						g->insertEdge(from_id, to_id, newEdge);
					}
					else if ( strCmpI(key,"EQUIV") )
					{
						// Already read in the 1st pass.
					}
					else
					{	// Unknown entry: Warn the user just once:
						if (alreadyWarnedUnknowns.find(key)==alreadyWarnedUnknowns.end())
						{
							alreadyWarnedUnknowns.insert(key);
							cerr << "[CNetworkOfPoses::loadFromTextFile] " << fil << ":" << lineNum << ": Warning: unknown entry type: " << key << endl;
						}
					}
				} // end while

			} // end load_graph

		}
	}
}

// Explicit instantations:
template void BASE_IMPEXP mrpt::poses::detail::save_graph_of_poses_from_text_file<CPosePDFGaussian>(const CNetworkOfPoses<CPosePDFGaussian> *g, const std::string &fil);
template void BASE_IMPEXP mrpt::poses::detail::save_graph_of_poses_from_text_file<CPose3DPDFGaussian>(const CNetworkOfPoses<CPose3DPDFGaussian> *g, const std::string &fil);
template void BASE_IMPEXP mrpt::poses::detail::save_graph_of_poses_from_text_file<CPosePDFGaussianInf>(const CNetworkOfPoses<CPosePDFGaussianInf> *g, const std::string &fil);
template void BASE_IMPEXP mrpt::poses::detail::save_graph_of_poses_from_text_file<CPose3DPDFGaussianInf>(const CNetworkOfPoses<CPose3DPDFGaussianInf> *g, const std::string &fil);

template void BASE_IMPEXP mrpt::poses::detail::load_graph_of_poses_from_text_file<CPosePDFGaussian>(CNetworkOfPoses<CPosePDFGaussian> *g, const std::string &fil);
template void BASE_IMPEXP mrpt::poses::detail::load_graph_of_poses_from_text_file<CPose3DPDFGaussian>(CNetworkOfPoses<CPose3DPDFGaussian> *g, const std::string &fil);
template void BASE_IMPEXP mrpt::poses::detail::load_graph_of_poses_from_text_file<CPosePDFGaussianInf>(CNetworkOfPoses<CPosePDFGaussianInf> *g, const std::string &fil);
template void BASE_IMPEXP mrpt::poses::detail::load_graph_of_poses_from_text_file<CPose3DPDFGaussianInf>(CNetworkOfPoses<CPose3DPDFGaussianInf> *g, const std::string &fil);


void  dijks_on_progress(size_t visitedCount)
{
	//if (visitedCount & 0x20 == 0 )
		cout << "dijkstra: " << visitedCount << endl;
}

// --------------------------------------------------------------------------------
//               dijkstra_nodes_estimate
// --------------------------------------------------------------------------------
template <class CPOSE>
void CNetworkOfPoses<CPOSE>::dijkstra_nodes_estimate()
{
	MRPT_TRY_START

	// Do Dijkstra shortest path from "root" to all other nodes:
	mrpt::math::CDijkstra<CPOSE>  dijkstra(*this, this->root, NULL, &dijks_on_progress);



	MRPT_TRY_END
}

// Explicit instantations:
template class CNetworkOfPoses<CPosePDFGaussian>;
template class CNetworkOfPoses<CPose3DPDFGaussian>;
template class CNetworkOfPoses<CPosePDFGaussianInf>;
template class CNetworkOfPoses<CPose3DPDFGaussianInf>;


//   Implementation of serialization stuff
// --------------------------------------------------------------------------------
IMPLEMENTS_SERIALIZABLE( CNetworkOfPoses2D, CSerializable, mrpt::poses )
IMPLEMENTS_SERIALIZABLE( CNetworkOfPoses3D, CSerializable, mrpt::poses )
IMPLEMENTS_SERIALIZABLE( CNetworkOfPoses2DInf, CSerializable, mrpt::poses )
IMPLEMENTS_SERIALIZABLE( CNetworkOfPoses3DInf, CSerializable, mrpt::poses )

// Use MRPT's STL-metaprogramming automatic serialization for the field "edges" & "nodes":
#define DO_IMPLEMENT_READ_WRITE(_CLASS) \
	void  _CLASS::writeToStream(CStream &out, int *version) const \
	{ \
		if (version) \
			*version = 0; \
		else \
		{ \
			out << nodes << edges << root;  \
		} \
	} \
	void  _CLASS::readFromStream(CStream &in, int version) \
	{ \
		switch(version) \
		{ \
		case 0: \
			{ \
				in >> nodes >> edges >> root; \
			} break; \
		default: \
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version) \
		}; \
	}


DO_IMPLEMENT_READ_WRITE(CNetworkOfPoses2D)
DO_IMPLEMENT_READ_WRITE(CNetworkOfPoses3D)
DO_IMPLEMENT_READ_WRITE(CNetworkOfPoses2DInf)
DO_IMPLEMENT_READ_WRITE(CNetworkOfPoses3DInf)

