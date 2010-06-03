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

#include <mrpt/slam.h>  // Precompiled header

#include <mrpt/poses/CNetworkOfPoses.h>
#include <mrpt/slam/graph_slam.h>

#include <mrpt/math/dijkstra.h>
#include <mrpt/math/jacobians.h>
#include <mrpt/math/CLevenbergMarquardt.h>

#include <list>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::slam;
using namespace mrpt::math;
using namespace std;


/*---------------------------------------------------------------
					optimizePoseGraph_levmarq
  ---------------------------------------------------------------*/
namespace mrpt
{
	namespace slam
	{
		template <class CPOSE>
		struct TAuxDataFunctorOptPoseGraph
		{
			map<size_t,size_t>  *nodeid2idx;
			map<size_t,size_t>  *idx2nodeid;
			const CNetworkOfPoses<CPOSE>  *pose_graph;
			size_t				poseID_origin;
		};


		// Called from: optimizePoseGraph_levmarq
		template <class CPOSE, bool USE_MAHALANOBIS>
		void lm_functor_optimize_posegraph(
			const vector_double &x,
			const TAuxDataFunctorOptPoseGraph<CPOSE> &links_data,
			vector_double &out_error)
		{
			// "x" has N * CPOSE::state_length with the global coordinates of all the nodes in the graph
			//     The indices in "x" are related to arcs (links) in the graph thru "idx2nodeid"
			//
			// The error function consists of the Mahalanobis distances between each link and the current
			//  estimate from the global coordinates in "x"
			//
			const size_t nLinks = links_data.pose_graph->edgeCount();

			if (USE_MAHALANOBIS)
			     out_error.resize( nLinks );
			else out_error.resize( nLinks * CPOSE::state_length );

			static const double EPS = std::numeric_limits<double>::min();

			typename CNetworkOfPoses<CPOSE>::const_iterator  itLink;
			size_t j;
			size_t idx_src, idx_dst;

			for (j=0,itLink=links_data.pose_graph->begin() ; j<nLinks; ++j, ++itLink)
			{
				// Compare the link "itLink" with the current candidate global poses
				//  of both nodes, which are in "x[idx_*]"
				if ( itLink->first.first == links_data.poseID_origin )
						idx_src = static_cast<size_t>(-1);		// The origin is different:
				else	idx_src = CPOSE::state_length * (*links_data.nodeid2idx)[ itLink->first.first ];

				if ( itLink->first.second == links_data.poseID_origin )
						idx_dst = static_cast<size_t>(-1);		// The origin is different:
				else	idx_dst = CPOSE::state_length * (*links_data.nodeid2idx)[ itLink->first.second ];


				if (CPOSE::state_length==3)
				{
					// 2D case:
					CPose2D global_src, global_dst;

					if (idx_src != static_cast<size_t>(-1))
					{
						global_src.x( x[idx_src+0] );
						global_src.y( x[idx_src+1]);
						global_src.phi(x[idx_src+2]);
					}
					else
					{
						global_src.x(0);
						global_src.y(0);
						global_src.z(0);
					}

					if (idx_dst != static_cast<size_t>(-1))
					{
						global_dst.x( x[idx_dst+0]);
						global_dst.y( x[idx_dst+1]);
						global_dst.phi( x[idx_dst+2]);
					}
					else
					{
						global_dst.x(0);
						global_dst.y(0);
						global_dst.z(0);
					}

					const CPose2D cur_est_link = global_dst - global_src;

					const CPose2D edgeVal = CPose2D(itLink->second.mean);

					// Compare with the link in the graph:
					if (USE_MAHALANOBIS)
						out_error[j] = -log( max( EPS, itLink->second.evaluateNormalizedPDF( CPosePDF::type_value(cur_est_link) ) )); // -log(p) -> mahalanobis distance
					else
					{
						const size_t idx = j * CPOSE::state_length;
						const CPose2D link_val = CPose2D(itLink->second.mean);
						out_error[idx+0] = fabs( cur_est_link.x() - link_val.x() );
						out_error[idx+1] = fabs( cur_est_link.y() - link_val.y() );
						out_error[idx+2] = mrpt::math::wrapToPi( cur_est_link.phi() - link_val.phi() );
					}
				}
				else
				{
					// 3D case:
					const CPose3D global_src( x[idx_src+0],x[idx_src+1],x[idx_src+2],x[idx_src+3],x[idx_src+4],x[idx_src+5]);
					const CPose3D global_dst( x[idx_dst+0],x[idx_dst+1],x[idx_dst+2],x[idx_dst+3],x[idx_dst+4],x[idx_dst+5]);
					const CPose3D cur_est_link = global_dst - global_src;

					// Compare with the link in the graph:
					if (USE_MAHALANOBIS)
						out_error[j] = -log(  max( EPS, itLink->second.evaluateNormalizedPDF( CPosePDF::type_value(cur_est_link) ) ) );// -log(p) -> mahalanobis distance
					else
					{
						const size_t idx = j * CPOSE::state_length;
						const CPose3D link_val = CPose3D(itLink->second.mean);
						out_error[idx+0] = fabs( cur_est_link.x() - link_val.x() );
						out_error[idx+1] = fabs( cur_est_link.y() - link_val.y() );
						out_error[idx+2] = fabs( cur_est_link.z() - link_val.z() );
						out_error[idx+3] = mrpt::math::wrapToPi( cur_est_link.yaw() - link_val.yaw() );
						out_error[idx+4] = mrpt::math::wrapToPi( cur_est_link.pitch() - link_val.pitch() );
						out_error[idx+5] = mrpt::math::wrapToPi( cur_est_link.roll() - link_val.roll() );
					}
				}
			}

			//cout << "x: ";  mrpt::math::operator <<(cout,x); cout << endl;
			//mrpt::math::operator <<(cout,out_error);
			//cout << endl << "ERR: " << sum(out_error) << endl;
		}

		// Auxiliary function for the weight of Dijkstra edges:
		template<class CPOSE>
		double lm_functor_edge_weight(const CPOSE &edge)
		{
			return 1.0;
		}
	}
}


/*---------------------------------------------------------------
					optimizePoseGraph_levmarq
  ---------------------------------------------------------------*/
template <class CPOSE>
double mrpt::slam::optimizePoseGraph_levmarq(
	const CNetworkOfPoses<CPOSE>  &pose_graph,
	std::map<size_t,CPOSE>	      &optimal_poses,
	const size_t                   max_iterations,
	const size_t                   origin_pose
	)
{
	MRPT_START

	const size_t nLinks = pose_graph.edgeCount();

	// If there are no pose links, we have nothing to do here:
	if (!nLinks)
	{
		optimal_poses.clear();
		return 0;
	}

	// Make the list of all nodes appearing in pose_graph:
	map<size_t,size_t>  nodeid2idx;
	map<size_t,size_t>  idx2nodeid;


	// Select the reference pose ID in "nodeID_origin"
	size_t  nodeID_origin = origin_pose;

	for (typename CNetworkOfPoses<CPOSE>::const_iterator i=pose_graph.begin();i!=pose_graph.end();++i)
	{
		ASSERT_( i->first.first != i->first.second )

		{
			size_t id = i->first.first;

			if (nodeID_origin == static_cast<size_t>(-1))	// Pick the first node as origin
				nodeID_origin = id;


			if (nodeID_origin!=id && nodeid2idx.find( id ) == nodeid2idx.end() )
			{
				const size_t newIdx = nodeid2idx.size();
				nodeid2idx[id] = newIdx;
				idx2nodeid[newIdx] = id;
			}
		}
		{
			size_t id = i->first.second;
			if (nodeID_origin!=id && nodeid2idx.find( id ) == nodeid2idx.end() )
			{
				const size_t newIdx = nodeid2idx.size();
				nodeid2idx[id] = newIdx;
				idx2nodeid[newIdx] = id;
			}
		}

	}

	// The number of optimal poses to optimize: all of them but "nodeID_origin"
	ASSERT_(nodeid2idx.size()>0)
	ASSERT_(CPOSE::state_length==3 || CPOSE::state_length==6) // 2D or 3D poses only

	const size_t nGlobalPosesToOpt = nodeid2idx.size(); // This is all nodes - 1, since the origin is NOT in nodeid2idx

	// --------------------------------------------------------------------------
	// Prepare initial poses: Dijkstra algorithm for initial best global poses
	// --------------------------------------------------------------------------
	vector_double  poses_initial;
	poses_initial.resize(nGlobalPosesToOpt * CPOSE::state_length ,0);

	std::set<size_t> lstNode_IDs;
	pose_graph.getAllNodes( lstNode_IDs );

	{

		ASSERT_( nGlobalPosesToOpt == (lstNode_IDs.size()-1) );

		CDijkstra<CPOSE>  dijkstra( pose_graph, nodeID_origin, lm_functor_edge_weight );

		std::list<std::pair<size_t,size_t> >	pathEdges; // The optimal path for each node

		// Get the path from "nodeID_origin" to the rest of nodes:
		for ( std::set<size_t>::const_iterator n=lstNode_IDs.begin();n!=lstNode_IDs.end();++n)
		{
			// The origin does not appear in poses_initial:
			if (*n == nodeID_origin) continue;

			// Get the optimal path as the list of edges:
			dijkstra.getShortestPathTo(*n, pathEdges );

			// Transverse the edges from *n -> nodeID_origin
			//  and accumulate the pose increments:
			typename CPOSE::type_value   this_node_global;
			size_t curID = nodeID_origin;

			ASSERT_(!pathEdges.empty())

			for (std::list<std::pair<size_t,size_t> >::const_iterator itEdge = pathEdges.begin();itEdge!=pathEdges.end();++itEdge)
			{
				typename CDirectedGraph<CPOSE>::const_iterator itEd = pose_graph.edges.find( *itEdge );
				typename CPOSE::type_value incrPose = itEd->second.mean;

				// Inverse?
				if ( itEdge->first==curID )
				{
					curID = itEdge->second;
				}
				else
				{	// Reverse pose:
					curID = itEdge->first;
					incrPose = -incrPose;
				}

				this_node_global = this_node_global + incrPose;
			}

			// Put the global pose in the vector:
			vector_double v;
			this_node_global.getAsVector(v);

			ASSERT_( v.size() == CPOSE::state_length )

			const size_t base_idx = nodeid2idx[*n] * CPOSE::state_length;
			for (size_t q=0;q<CPOSE::state_length;q++)
				poses_initial[ base_idx+q ] = v[q] + 0.01;
		}
	}

	// Prepare the vector of increments:
	// --------------------------------------------------------------------------
	vector_double  poses_increments(nGlobalPosesToOpt * CPOSE::state_length );

	const double incr_xyz = 0.001;
	const double incr_ang = DEG2RAD(0.01);
	if (CPOSE::state_length==3)
	{
		// CPOSE is a CPosePDF
		for (size_t i=0;i<nGlobalPosesToOpt ;i++)
		{
			const size_t idx = i * CPOSE::state_length;
			poses_increments[idx+0] =
			poses_increments[idx+1] = incr_xyz;
			poses_increments[idx+2] = incr_ang;
		}
	}
	else
	{
		// CPOSE is a CPose3DPDF
		for (size_t i=0;i<nGlobalPosesToOpt ;i++)
		{
			const size_t idx = i * CPOSE::state_length;
			poses_increments[idx+0] =
			poses_increments[idx+1] =
			poses_increments[idx+2] = incr_xyz;
			poses_increments[idx+3] =
			poses_increments[idx+4] =
			poses_increments[idx+5] = incr_ang;
		}
	}


	// The auxiliary data passed to the functor:
	TAuxDataFunctorOptPoseGraph<CPOSE> auxData;
	auxData.nodeid2idx = &nodeid2idx;
	auxData.idx2nodeid = &idx2nodeid;
	auxData.pose_graph = &pose_graph;
	auxData.poseID_origin = nodeID_origin;

	// Run the optimization:
	// --------------------------------------------
	typedef CLevenbergMarquardtTempl<vector_double, TAuxDataFunctorOptPoseGraph<CPOSE> > CLevMarqPoseNets;

	typename CLevMarqPoseNets::TResultInfo LM_info;
	vector_double  poses_optimal;

	if (max_iterations>0)
	{
		bool verbose = false;

		CLevMarqPoseNets::execute(
			poses_optimal,
			poses_initial,
			&lm_functor_optimize_posegraph<CPOSE,true>,
			poses_increments,
			auxData,
			LM_info,
			verbose,
			max_iterations );
	}
	else
	{
		// Skip optimization, just use Dijkstra:
		poses_optimal.swap( poses_initial );
	}

	// Create the J matrix which can be used to estimate the covariance of the optimal solution:
	CMatrixDouble J;
	math::jacobians::jacob_numeric_estimate( poses_optimal, &lm_functor_optimize_posegraph<CPOSE,false>, poses_increments, auxData, J);

	CMatrixDouble H = J.pseudoInverse<CMatrixDouble>();
	cout << "LM cov: H size: " << size(H,1) << "x" << size(H,2) << endl;

	// Collect the optimal results in "poses_optimal" and save them in "optimal_poses":
	// ---------------------------------------------------------------------------------
	optimal_poses.clear();

	for (std::set<size_t>::const_iterator n=lstNode_IDs.begin();n!=lstNode_IDs.end();++n)
	{
		CPOSE &p = optimal_poses[*n];

		if (*n == nodeID_origin)
		{
			// The origin node:
			p.mean = typename CPOSE::type_value();  // all zeros
			p.cov.zeros();
		}
		else
		{
			// A normal node: Take the first index in the optimal state vector
			const size_t idxbase = CPOSE::state_length * nodeid2idx[*n];

			if (CPOSE::state_length==3)
				p.mean = typename CPOSE::type_value( CPose2D( poses_optimal[idxbase+0],poses_optimal[idxbase+1],poses_optimal[idxbase+2] ) );
			else
				p.mean = typename CPOSE::type_value( CPose3D(
					poses_optimal[idxbase+0],poses_optimal[idxbase+1],poses_optimal[idxbase+2],
					poses_optimal[idxbase+3],poses_optimal[idxbase+4],poses_optimal[idxbase+5] ) );

			// Covariance: TODO
			// ...
		}

	}

	return LM_info.final_sqr_err;

	MRPT_END
}


// Explicit instantations:
template double SLAM_IMPEXP mrpt::slam::optimizePoseGraph_levmarq(
	const CNetworkOfPoses<CPosePDFGaussian> &pose_graph,
	std::map<size_t,CPosePDFGaussian>	    &optimal_poses,
	const size_t                   max_iterations,
	const size_t  origin_pose );

template double SLAM_IMPEXP mrpt::slam::optimizePoseGraph_levmarq(
	const CNetworkOfPoses<CPose3DPDFGaussian> &pose_graph,
	std::map<size_t,CPose3DPDFGaussian>	    &optimal_poses,
	const size_t                   max_iterations,
	const size_t  origin_pose );


//MRPT_TODO("Add specializations for graphs of 3D poses with quaternions as well.")

