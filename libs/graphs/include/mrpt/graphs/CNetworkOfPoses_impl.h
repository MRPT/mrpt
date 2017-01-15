/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CONSTRAINED_POSE_NETWORK_IMPL_H
#define CONSTRAINED_POSE_NETWORK_IMPL_H

#include <mrpt/graphs/dijkstra.h>
#include <mrpt/utils/CTextFileLinesParser.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/math/CArrayNumeric.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/math/ops_matrices.h> // multiply_*()
#include <mrpt/math/matrix_serialization.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussianInf.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>

namespace mrpt
{
	namespace graphs
	{
		namespace detail
		{
			using namespace std;
			using namespace mrpt;
			using namespace mrpt::utils;
			using namespace mrpt::poses;
			using namespace mrpt::graphs;

			template <class POSE_PDF> struct TPosePDFHelper
			{
				static inline void copyFrom2D(POSE_PDF &p, const CPosePDFGaussianInf &pdf ) { p.copyFrom( pdf ); }
				static inline void copyFrom3D(POSE_PDF &p, const CPose3DPDFGaussianInf &pdf ) { p.copyFrom( pdf ); }
			};
			template <> struct TPosePDFHelper<CPose2D>
			{
				static inline void copyFrom2D(CPose2D &p, const CPosePDFGaussianInf &pdf ) { p = pdf.mean; }
				static inline void copyFrom3D(CPose2D &p, const CPose3DPDFGaussianInf &pdf ) { p = CPose2D(pdf.mean); }
			};
			template <> struct TPosePDFHelper<CPose3D>
			{
				static inline void copyFrom2D(CPose3D &p, const CPosePDFGaussianInf &pdf ) { p = pdf.mean; }
				static inline void copyFrom3D(CPose3D &p, const CPose3DPDFGaussianInf &pdf ) { p = pdf.mean; }
			};

			/// a helper struct with static template functions \sa CNetworkOfPoses
			template <class graph_t>
			struct graph_ops
			{
				static void write_VERTEX_line(const TNodeID id, const mrpt::poses::CPose2D &p, std::ofstream &f)
				{
					//  VERTEX2 id x y phi
					f << "VERTEX2 " << id << " " << p.x() << " " << p.y() << " " << p.phi() << endl;
				}
				static void write_VERTEX_line(const TNodeID id, const mrpt::poses::CPose3D &p, std::ofstream &f)
				{
					//  VERTEX3 id x y z roll pitch yaw
					// **CAUTION** In the TORO graph format angles are in the RPY order vs. MRPT's YPR.
					f << "VERTEX3 " << id << " " << p.x() << " " << p.y() << " " << p.z()<< " " << p.roll()<< " " << p.pitch()<< " " << p.yaw() << endl;
				}


				static void write_EDGE_line( const TPairNodeIDs &edgeIDs,const CPosePDFGaussianInf & edge, std::ofstream &f)
				{
					//  EDGE2 from_id to_id Ax Ay Aphi inf_xx inf_xy inf_yy inf_pp inf_xp inf_yp
					// **CAUTION** TORO docs say "from_id" "to_id" in the opposite order, but it seems from the data that this is the correct expected format.
					f << "EDGE2 " << edgeIDs.first << " " << edgeIDs.second << " " <<
							edge.mean.x()<<" "<<edge.mean.y()<<" "<<edge.mean.phi()<<" "<<
							edge.cov_inv(0,0)<<" "<<edge.cov_inv(0,1)<<" "<<edge.cov_inv(1,1)<<" "<<
							edge.cov_inv(2,2)<<" "<<edge.cov_inv(0,2)<<" "<<edge.cov_inv(1,2) << endl;
				}
				static void write_EDGE_line( const TPairNodeIDs &edgeIDs,const CPose3DPDFGaussianInf & edge, std::ofstream &f)
				{
					//  EDGE3 from_id to_id Ax Ay Az Aroll Apitch Ayaw inf_11 inf_12 .. inf_16 inf_22 .. inf_66
					// **CAUTION** In the TORO graph format angles are in the RPY order vs. MRPT's YPR.
					// **CAUTION** TORO docs say "from_id" "to_id" in the opposite order, but it seems from the data that this is the correct expected format.
					f << "EDGE3 " << edgeIDs.first << " " << edgeIDs.second << " " <<
							edge.mean.x()<<" "<<edge.mean.y()<<" "<<edge.mean.z()<<" "<<
							edge.mean.roll()<<" "<<edge.mean.pitch()<<" "<<edge.mean.yaw()<<" "<<
							edge.cov_inv(0,0)<<" "<<edge.cov_inv(0,1)<<" "<<edge.cov_inv(0,2)<<" "<<edge.cov_inv(0,5)<<" "<<edge.cov_inv(0,4)<<" "<<edge.cov_inv(0,3)<<" "<<
							edge.cov_inv(1,1)<<" "<<edge.cov_inv(1,2)<<" "<<edge.cov_inv(1,5)<<" "<<edge.cov_inv(1,4)<<" "<<edge.cov_inv(1,3)<<" "<<
							edge.cov_inv(2,2)<<" "<<edge.cov_inv(2,5)<<" "<<edge.cov_inv(2,4)<<" "<<edge.cov_inv(2,3)<<" "<<
							edge.cov_inv(5,5)<<" "<<edge.cov_inv(5,4)<<" "<<edge.cov_inv(5,3)<<" "<<
							edge.cov_inv(4,4)<<" "<<edge.cov_inv(4,3)<<" "<<
							edge.cov_inv(3,3) << endl;
				}
				static void write_EDGE_line( const TPairNodeIDs &edgeIDs,const CPosePDFGaussian & edge, std::ofstream &f)
				{
					CPosePDFGaussianInf p;
					p.copyFrom(edge);
					write_EDGE_line(edgeIDs,p,f);
				}
				static void write_EDGE_line( const TPairNodeIDs &edgeIDs,const CPose3DPDFGaussian & edge, std::ofstream &f)
				{
					CPose3DPDFGaussianInf p;
					p.copyFrom(edge);
					write_EDGE_line(edgeIDs,p,f);
				}
				static void write_EDGE_line( const TPairNodeIDs &edgeIDs,const mrpt::poses::CPose2D & edge, std::ofstream &f)
				{
					CPosePDFGaussianInf p;
					p.mean = edge;
					p.cov_inv.unit(3,1.0);
					write_EDGE_line(edgeIDs,p,f);
				}
				static void write_EDGE_line( const TPairNodeIDs &edgeIDs,const mrpt::poses::CPose3D & edge, std::ofstream &f)
				{
					CPose3DPDFGaussianInf p;
					p.mean = edge;
					p.cov_inv.unit(6,1.0);
					write_EDGE_line(edgeIDs,p,f);
				}


				// =================================================================
				//                     save_graph_of_poses_to_text_file
				// =================================================================
				static void save_graph_of_poses_to_text_file(const graph_t *g, const std::string &fil)
				{
					std::ofstream  f;
					f.open(fil.c_str());
					if (!f.is_open())
						THROW_EXCEPTION_CUSTOM_MSG1("Error opening file '%s' for writing",fil.c_str());

					// 1st: Nodes
					for (typename graph_t::global_poses_t::const_iterator itNod = g->nodes.begin();itNod!=g->nodes.end();++itNod)
						write_VERTEX_line(itNod->first, itNod->second, f);

					// 2nd: Edges:
					for (typename graph_t::const_iterator it=g->begin();it!=g->end();++it)
						if (it->first.first!=it->first.second)	// Ignore self-edges, typically from importing files with EQUIV's
							write_EDGE_line( it->first, it->second, f);

				} // end save_graph

				// =================================================================
				//                     save_graph_of_poses_to_binary_file
				// =================================================================
				static void save_graph_of_poses_to_binary_file(const graph_t *g, mrpt::utils::CStream &out)
				{
					// Store class name:
					const std::string sClassName = TTypeName<graph_t>::get();
					out << sClassName;

					// Store serialization version & object data:
					const uint32_t version = 0;
					out << version;
					out << g->nodes << g->edges << g->root;
				}

				// =================================================================
				//                     read_graph_of_poses_from_binary_file
				// =================================================================
				static void read_graph_of_poses_from_binary_file(graph_t *g, mrpt::utils::CStream &in)
				{
					// Compare class name:
					const std::string sClassName = TTypeName<graph_t>::get();
					std::string sStoredClassName;
					in >> sStoredClassName;
					ASSERT_EQUAL_(sStoredClassName,sClassName)

					// Check serialization version:
					uint32_t stored_version;
					in >> stored_version;

					g->clear();
					switch (stored_version)
					{
					case 0:
						in >> g->nodes >> g->edges >> g->root;
						break;
					default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(stored_version)
					}

				}

				// =================================================================
				//                     load_graph_of_poses_from_text_file
				// =================================================================
				static void load_graph_of_poses_from_text_file(graph_t *g, const std::string &fil)
				{
					using mrpt::system::strCmpI;
					using namespace mrpt::math;

					typedef typename graph_t::constraint_t CPOSE;

					set<string>  alreadyWarnedUnknowns; // for unknown line types, show a warning to cerr just once.

					// First, empty the graph:
					g->clear();

					// Determine if it's a 2D or 3D graph, just to raise an error if loading a 3D graph in a 2D one, since
					//  it would be an unintentional loss of information:
					const bool graph_is_3D = CPOSE::is_3D();

					CTextFileLinesParser   filParser(fil);  // raises an exception on error

					// -------------------------------------------
					// 1st PASS: Read EQUIV entries only
					//  since processing them AFTER loading the data
					//  is much much slower.
					// -------------------------------------------
					map<TNodeID,TNodeID> lstEquivs; // List of EQUIV entries: NODEID -> NEWNODEID. NEWNODEID will be always the lowest ID number.

					// Read & process lines each at once until EOF:
					istringstream s;
					while (filParser.getNextLine(s))
					{
						const unsigned int lineNum = filParser.getCurrentLineNumber();
						const string lin = s.str();

						string key;
						if ( !(s >> key) || key.empty() )
							THROW_EXCEPTION(format("Line %u: Can't read string for entry type in: '%s'", lineNum, lin.c_str() ) );

						if ( mrpt::system::strCmpI(key,"EQUIV") )
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
					filParser.rewind();

					// Read & process lines each at once until EOF:
					while (filParser.getNextLine(s))
					{
						const unsigned int lineNum = filParser.getCurrentLineNumber();
						const string lin = s.str();

						// Recognized strings:
						//  VERTEX2 id x y phi
						//   =(VERTEX_SE2)
						//  EDGE2 from_id to_id Ax Ay Aphi inf_xx inf_xy inf_yy inf_pp inf_xp inf_yp
						//   =(EDGE or EDGE_SE2 or ODOMETRY)
						//  VERTEX3 id x y z roll pitch yaw
						//  VERTEX_SE3:QUAT id x y z qx qy qz qw
						//  EDGE3 from_id to_id Ax Ay Az Aroll Apitch Ayaw inf_11 inf_12 .. inf_16 inf_22 .. inf_66
						//  EDGE_SE3:QUAT from_id to_id Ax Ay Az qx qy qz qw inf_11 inf_12 .. inf_16 inf_22 .. inf_66
						//  EQUIV id1 id2
						string key;
						if ( !(s >> key) || key.empty() )
							THROW_EXCEPTION(format("Line %u: Can't read string for entry type in: '%s'", lineNum, lin.c_str() ) );

						if ( strCmpI(key,"VERTEX2") || strCmpI(key,"VERTEX") || strCmpI(key,"VERTEX_SE2") )
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
								typename CNetworkOfPoses<CPOSE>::constraint_t::type_value & newNode = g->nodes[id];
								newNode = CPose2D(p2D); // Auto converted to mrpt::poses::CPose3D if needed
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
								g->nodes[id] = typename CNetworkOfPoses<CPOSE>::constraint_t::type_value( CPose3D(p3D) ); // Auto converted to CPose2D if needed
							}
						}
						else if ( strCmpI(key,"VERTEX_SE3:QUAT") )
						{
							if (!graph_is_3D)
								THROW_EXCEPTION(format("Line %u: Try to load VERTEX_SE3:QUAT into a 2D graph: '%s'", lineNum, lin.c_str() ) );

							// VERTEX_SE3:QUAT id x y z qx qy qz qw
							TNodeID  id;
							TPose3DQuat  p3D;
							if (!(s>> id >> p3D.x >> p3D.y >> p3D.z >> p3D.qx >> p3D.qy >> p3D.qz >> p3D.qr ))
								THROW_EXCEPTION(format("Line %u: Error parsing VERTEX_SE3:QUAT line: '%s'", lineNum, lin.c_str() ) );

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
								g->nodes[id] = typename CNetworkOfPoses<CPOSE>::constraint_t::type_value( CPose3D(CPose3DQuat(p3D)) ); // Auto converted to CPose2D if needed
							}
						}
						else if ( strCmpI(key,"EDGE2") || strCmpI(key,"EDGE") || strCmpI(key,"ODOMETRY") || strCmpI(key,"EDGE_SE2") )
						{
							//  EDGE2 from_id to_id Ax Ay Aphi inf_xx inf_xy inf_yy inf_pp inf_xp inf_yp
							//                                   s00   s01     s11    s22    s02    s12
							//  Read values are:
							//    [ s00 s01 s02 ]
							//    [  -  s11 s12 ]
							//    [  -   -  s22 ]
							//
							TNodeID  to_id, from_id;
							if (!(s>> from_id >> to_id ))
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

							if (from_id!=to_id)	// Don't load self-edges! (probably come from an EQUIV)
							{
								TPose2D  Ap_mean;
								mrpt::math::CMatrixDouble33 Ap_cov_inv;
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
								TPosePDFHelper<CPOSE>::copyFrom2D(newEdge, CPosePDFGaussianInf( CPose2D(Ap_mean), Ap_cov_inv ) );
								g->insertEdge(from_id, to_id, newEdge);
							}
						}
						else if ( strCmpI(key,"EDGE3") )
						{
							if (!graph_is_3D)
								THROW_EXCEPTION(format("Line %u: Try to load EDGE3 into a 2D graph: '%s'", lineNum, lin.c_str() ) );

							//  EDGE3 from_id to_id Ax Ay Az Aroll Apitch Ayaw inf_11 inf_12 .. inf_16 inf_22 .. inf_66
							TNodeID  to_id, from_id;
							if (!(s>> from_id >> to_id ))
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

							if (from_id!=to_id)	// Don't load self-edges! (probably come from an EQUIV)
							{
								TPose3D  Ap_mean;
								mrpt::math::CMatrixDouble66 Ap_cov_inv;
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
									Ap_cov_inv.unit(6,1.0);

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
								TPosePDFHelper<CPOSE>::copyFrom3D(newEdge, CPose3DPDFGaussianInf( CPose3D(Ap_mean), Ap_cov_inv ) );
								g->insertEdge(from_id, to_id, newEdge);
							}
						}
						else if ( strCmpI(key,"EDGE_SE3:QUAT") )
						{
							if (!graph_is_3D)
								THROW_EXCEPTION(format("Line %u: Try to load EDGE3 into a 2D graph: '%s'", lineNum, lin.c_str() ) );

							//  EDGE_SE3:QUAT from_id to_id Ax Ay Az qx qy qz qw inf_11 inf_12 .. inf_16 inf_22 .. inf_66
							//  EDGE3 from_id to_id Ax Ay Az Aroll Apitch Ayaw inf_11 inf_12 .. inf_16 inf_22 .. inf_66
							TNodeID  to_id, from_id;
							if (!(s>> from_id >> to_id ))
								THROW_EXCEPTION(format("Line %u: Error parsing EDGE_SE3:QUAT line: '%s'", lineNum, lin.c_str() ) );

							// EQUIV? Replace ID by new one.
							{
								const map<TNodeID,TNodeID>::const_iterator itEq = lstEquivs.find(to_id);
								if (itEq!=lstEquivs.end()) to_id = itEq->second;
							}
							{
								const map<TNodeID,TNodeID>::const_iterator itEq = lstEquivs.find(from_id);
								if (itEq!=lstEquivs.end()) from_id = itEq->second;
							}

							if (from_id!=to_id)	// Don't load self-edges! (probably come from an EQUIV)
							{
								TPose3DQuat Ap_mean;
							 mrpt::math::CMatrixDouble66 Ap_cov_inv;
								if (!(s>> Ap_mean.x >> Ap_mean.y >> Ap_mean.z >> Ap_mean.qx >> Ap_mean.qy >> Ap_mean.qz >> Ap_mean.qr ))
									THROW_EXCEPTION(format("Line %u: Error parsing EDGE_SE3:QUAT line: '%s'", lineNum, lin.c_str() ) );

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
									Ap_cov_inv.unit(6,1.0);

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
								TPosePDFHelper<CPOSE>::copyFrom3D(newEdge, CPose3DPDFGaussianInf( CPose3D(CPose3DQuat(Ap_mean)), Ap_cov_inv ) );
								g->insertEdge(from_id, to_id, newEdge);
							}
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


				// --------------------------------------------------------------------------------
				//               Implements: collapseDuplicatedEdges
				//
				// Look for duplicated edges (even in opposite directions) between all pairs of nodes and fuse them.
				//  Upon return, only one edge remains between each pair of nodes with the mean
				//   & covariance (or information matrix) corresponding to the Bayesian fusion of all the Gaussians.
				// --------------------------------------------------------------------------------
				static size_t graph_of_poses_collapse_dup_edges(graph_t *g)
				{
					MRPT_START
					typedef typename graph_t::edges_map_t::iterator TEdgeIterator;

					// Data structure: (id1,id2) -> all edges between them
					//  (with id1 < id2)
					typedef map<pair<TNodeID,TNodeID>, vector<TEdgeIterator> > TListAllEdges; // For god's sake... when will ALL compilers support auto!! :'-(
					TListAllEdges lstAllEdges;

					// Clasify all edges to identify duplicated ones:
					for (TEdgeIterator itEd=g->edges.begin();itEd!=g->edges.end();++itEd)
					{
						// Build a pair <id1,id2> with id1 < id2:
						const pair<TNodeID,TNodeID> arc_id = make_pair( std::min(itEd->first.first,itEd->first.second),std::max(itEd->first.first,itEd->first.second) );
						// get (or create the first time) the list of edges between them:
						vector<TEdgeIterator> &lstEdges = lstAllEdges[arc_id];
						// And add this one:
						lstEdges.push_back(itEd);
					}

					// Now, remove all but the first edge:
					size_t  nRemoved = 0;
					for (typename TListAllEdges::const_iterator it=lstAllEdges.begin();it!=lstAllEdges.end();++it)
					{
						const size_t N = it->second.size();
						for (size_t i=1;i<N;i++)  // i=0 is NOT removed
							g->edges.erase( it->second[i] );

						if (N>=2) nRemoved+=N-1;
					}

					return nRemoved;
					MRPT_END
				} // end of graph_of_poses_collapse_dup_edges

				// --------------------------------------------------------------------------------
				//               Implements: dijkstra_nodes_estimate
				//
				//	Compute a simple estimation of the global coordinates of each node just from the information in all edges, sorted in a Dijkstra tree based on the current "root" node.
				//	Note that "global" coordinates are with respect to the node with the ID specified in \a root.
				// --------------------------------------------------------------------------------
				static void graph_of_poses_dijkstra_init(graph_t *g)
				{
					MRPT_START

					// Do Dijkstra shortest path from "root" to all other nodes:
					typedef CDijkstra<graph_t,typename graph_t::maps_implementation_t> dijkstra_t;
					typedef typename graph_t::constraint_t  constraint_t;

					dijkstra_t dijkstra(*g, g->root);

					// Get the tree representation of the graph and traverse it
					//  from its root toward the leafs:
					typename dijkstra_t::tree_graph_t  treeView;
					dijkstra.getTreeGraph(treeView);

					// This visitor class performs the real job of
					struct VisitorComputePoses : public dijkstra_t::tree_graph_t::Visitor
					{
						graph_t * m_g; // The original graph

						VisitorComputePoses(graph_t *g) : m_g(g) { }
						virtual void OnVisitNode( const TNodeID parent_id, const typename dijkstra_t::tree_graph_t::Visitor::tree_t::TEdgeInfo &edge_to_child, const size_t depth_level ) MRPT_OVERRIDE
						{
							MRPT_UNUSED_PARAM(depth_level);
							const TNodeID  child_id = edge_to_child.id;

							// Compute the pose of "child_id" as parent_pose (+) edge_delta_pose,
							//  taking into account that that edge may be in reverse order and then have to invert the delta_pose:
							if ( (!edge_to_child.reverse && !m_g->edges_store_inverse_poses) ||
								 ( edge_to_child.reverse &&  m_g->edges_store_inverse_poses)
								)
							{	// pose_child = p_parent (+) p_delta
								m_g->nodes[child_id].composeFrom( m_g->nodes[parent_id],  edge_to_child.data->getPoseMean() );
							}
							else
							{	// pose_child = p_parent (+) [(-)p_delta]
								m_g->nodes[child_id].composeFrom( m_g->nodes[parent_id], - edge_to_child.data->getPoseMean() );
							}
						}
					};

					// Remove all global poses but for the root node, which is the origin:
					g->nodes.clear();
					g->nodes[g->root] = typename constraint_t::type_value();  // Typ: CPose2D() or CPose3D()

					// Run the visit thru all nodes in the tree:
					VisitorComputePoses  myVisitor(g);
					treeView.visitBreadthFirst(treeView.root, myVisitor);

					MRPT_END
				}


				// Auxiliary funcs:
				template <class VEC> static inline double auxMaha2Dist(VEC &err,const CPosePDFGaussianInf &p) {
					math::wrapToPiInPlace(err[2]);
					return mrpt::math::multiply_HCHt_scalar(err,p.cov_inv); // err^t*cov_inv*err
				}
				template <class VEC> static inline double auxMaha2Dist(VEC &err,const CPose3DPDFGaussianInf &p) {
					math::wrapToPiInPlace(err[3]);
					math::wrapToPiInPlace(err[4]);
					math::wrapToPiInPlace(err[5]);
					return mrpt::math::multiply_HCHt_scalar(err,p.cov_inv); // err^t*cov_inv*err
				}
				template <class VEC> static inline double auxMaha2Dist(VEC &err,const CPosePDFGaussian &p) {
					math::wrapToPiInPlace(err[2]);
					mrpt::math::CMatrixDouble33  COV_INV(mrpt::math::UNINITIALIZED_MATRIX);
					p.cov.inv(COV_INV);
					return mrpt::math::multiply_HCHt_scalar(err,COV_INV); // err^t*cov_inv*err
				}
				template <class VEC> static inline double auxMaha2Dist(VEC &err,const CPose3DPDFGaussian &p) {
					math::wrapToPiInPlace(err[3]);
					math::wrapToPiInPlace(err[4]);
					math::wrapToPiInPlace(err[5]);
					mrpt::math::CMatrixDouble66 COV_INV(mrpt::math::UNINITIALIZED_MATRIX);
					p.cov.inv(COV_INV);
					return mrpt::math::multiply_HCHt_scalar(err,COV_INV); // err^t*cov_inv*err
				}
				// These two are for simulating maha2 distances for non-PDF types: fallback to squared-norm:
				template <class VEC> static inline double auxMaha2Dist(VEC &err,const mrpt::poses::CPose2D &p) {
					math::wrapToPiInPlace(err[2]);
					return square(err[0])+square(err[1])+square(err[2]);
				}
				template <class VEC> static inline double auxMaha2Dist(VEC &err,const mrpt::poses::CPose3D &p) {
					math::wrapToPiInPlace(err[3]);
					math::wrapToPiInPlace(err[4]);
					math::wrapToPiInPlace(err[5]);
					return square(err[0])+square(err[1])+square(err[2])+square(err[3])+square(err[4])+square(err[5]);
				}


				static inline double auxEuclid2Dist(const mrpt::poses::CPose2D &p1,const mrpt::poses::CPose2D &p2) {
					return
						square(p1.x()-p2.x())+
						square(p1.y()-p2.y())+
						square( mrpt::math::wrapToPi(p1.phi()-p2.phi() ) );
				}
				static inline double auxEuclid2Dist(const mrpt::poses::CPose3D &p1,const mrpt::poses::CPose3D &p2) {
					return
						square(p1.x()-p2.x())+
						square(p1.y()-p2.y())+
						square(p1.z()-p2.z())+
						square( mrpt::math::wrapToPi(p1.yaw()-p2.yaw() ) )+
						square( mrpt::math::wrapToPi(p1.pitch()-p2.pitch() ) )+
						square( mrpt::math::wrapToPi(p1.roll()-p2.roll() ) );
				}

				// --------------------------------------------------------------------------------
				//               Implements: detail::graph_edge_sqerror
				//
				//	Compute the square error of a single edge, in comparison to the nodes global poses.
				// --------------------------------------------------------------------------------
				static double graph_edge_sqerror(
					const graph_t *g,
					const typename mrpt::graphs::CDirectedGraph<typename graph_t::constraint_t>::edges_map_t::const_iterator &itEdge,
					bool ignoreCovariances )
				{
					MRPT_START

					// Get node IDs:
					const TNodeID from_id = itEdge->first.first;
					const TNodeID to_id   = itEdge->first.second;

					// And their global poses as stored in "nodes"
					typename graph_t::global_poses_t::const_iterator itPoseFrom = g->nodes.find(from_id);
					typename graph_t::global_poses_t::const_iterator itPoseTo   = g->nodes.find(to_id);
					ASSERTMSG_(itPoseFrom!=g->nodes.end(), format("Node %u doesn't have a global pose in 'nodes'.", static_cast<unsigned int>(from_id)))
					ASSERTMSG_(itPoseTo!=g->nodes.end(), format("Node %u doesn't have a global pose in 'nodes'.", static_cast<unsigned int>(to_id)))

					// The global poses:
					typedef typename graph_t::constraint_t constraint_t;

					const typename constraint_t::type_value &from_mean = itPoseFrom->second;
					const typename constraint_t::type_value &to_mean   = itPoseTo->second;

					// The delta_pose as stored in the edge:
					const constraint_t &edge_delta_pose = itEdge->second;
					const typename constraint_t::type_value &edge_delta_pose_mean = edge_delta_pose.getPoseMean();

					if (ignoreCovariances)
					{	// Square Euclidean distance: Just use the mean values, ignore covs.
						// from_plus_delta = from_mean (+) edge_delta_pose_mean
						typename constraint_t::type_value from_plus_delta(UNINITIALIZED_POSE);
						from_plus_delta.composeFrom(from_mean, edge_delta_pose_mean);

						// (auxMaha2Dist will also take into account the 2PI wrapping)
						return auxEuclid2Dist(from_plus_delta,to_mean);
					}
					else
					{
						// Square Mahalanobis distance
						// from_plus_delta = from_mean (+) edge_delta_pose (as a Gaussian)
						constraint_t from_plus_delta = edge_delta_pose;
						from_plus_delta.changeCoordinatesReference(from_mean);

						// "from_plus_delta" is now a 3D or 6D Gaussian, to be compared to "to_mean":
						//  We want to compute the squared Mahalanobis distance:
						//       err^t * INV_COV * err
						//
						mrpt::math::CArrayDouble<constraint_t::type_value::static_size> err;
						for (size_t i=0;i<constraint_t::type_value::static_size;i++)
							err[i] = from_plus_delta.getPoseMean()[i] - to_mean[i];

						// (auxMaha2Dist will also take into account the 2PI wrapping)
						return auxMaha2Dist(err,from_plus_delta);
					}
					MRPT_END
				}

			}; // end of graph_ops<graph_t>

		}// end NS
	}// end NS
} // end NS

#endif
