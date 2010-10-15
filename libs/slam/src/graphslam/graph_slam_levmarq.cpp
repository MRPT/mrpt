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

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::slam;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;

/*
   The method implemented in this file is based on this work:
     - "Efficient Sparse Pose Adjustment for 2D Mapping", Kurt Konolige et al., 2010.

   But generalized for not only 2D but 2D and 3D poses, and using on-manifold optimization.
*/

/*---------------------------------------------------------------
			optimize_graph_spa_levmarq
   See .h for docs.
  ---------------------------------------------------------------*/
template <class EDGE_TYPE, class MAPS_IMPLEMENTATION>
void mrpt::graphslam::optimize_graph_spa_levmarq(
	CNetworkOfPoses<EDGE_TYPE,MAPS_IMPLEMENTATION>  &graph,
	const set<TNodeID>         * in_nodes_to_optimize,
	const mrpt::utils::TParametersDouble  &extra_params
	)
{
	MRPT_START
#define VERBOSE_PREFIX "[optimize_graph_spa_levmarq] "

	// The size of things here (because size matters...)
	typedef typename EDGE_TYPE::type_value edge_poses_type;
//	static const unsigned int DIMS_POSE = edge_poses_type::static_size;
	typedef SE_traits<edge_poses_type::rotation_dimensions> SE_TYPE;

	// Some typedefs to make life easier:
	typedef CNetworkOfPoses<EDGE_TYPE,MAPS_IMPLEMENTATION>  graph_t;
	//typedef CArrayDouble<DIMS_POSE>            Array_O;
	//typedef CArrayDouble<DIMS_POSE>            Array_P;
	//typedef CMatrixFixedNumeric<double,DIMS_POSE,DIMS_POSE> Matrix_PxP;

	// Extra params:
	const bool verbose            = 0!=extra_params.getWithDefaultVal("verbose",0);
	const size_t max_iters        = extra_params.getWithDefaultVal("max_iterations",50);
	const bool   enable_profiler  = 0!=extra_params.getWithDefaultVal("profiler",0);

	mrpt::utils::CTimeLogger  profiler(enable_profiler);

	// Make list of node IDs to optimize, since the user may want only a subset of them to be optimized:
	profiler.enter("list nodeIDs to optimize");
	const set<TNodeID> * nodes_to_optimize;
	set<TNodeID> nodes_to_optimize_auxlist;  // Used only if in_nodes_to_optimize==NULL
	if (in_nodes_to_optimize)
	{
		nodes_to_optimize = in_nodes_to_optimize;
	}
	else
	{
		// We have to make the list of IDs:
		for (typename graph_t::global_poses_t::const_iterator it=graph.nodes.begin();it!=graph.nodes.end();++it)
			if (it->first!=graph.root) // Root node is fixed.
				nodes_to_optimize_auxlist.insert(nodes_to_optimize_auxlist.end(), it->first ); // Provide the "first guess" insert position for efficiency
		nodes_to_optimize = &nodes_to_optimize_auxlist;
	}
	profiler.leave("list nodeIDs to optimize");

	if (verbose)
	{
		cout << VERBOSE_PREFIX << nodes_to_optimize->size() << " nodes to optimize: ";
		if (nodes_to_optimize->size()<14)
		{
			ostream_iterator<TNodeID> out_it (cout,", ");
			std::copy(nodes_to_optimize->begin(), nodes_to_optimize->end(), out_it );
		}
		cout << endl;
	}

	// Compute Jacobians:
	// ----------------------------------------
	profiler.enter("compute Jacobians");

	// The list of Jacobians: for each constraint i->j,
	//  we need the pair of Jacobians: { dh(xi,xj)_dxi, dh(xi,xj)_dxj },
	//  which are "first" and "second" in each pair.
	// Index of the map are the node IDs {i,j} for each contraint.
	typedef pair<typename SE_TYPE::matrix_VxV_t,typename SE_TYPE::matrix_VxV_t>   TPairJacobs;

	map<TPairNodeIDs,TPairJacobs>   lstJacobians;

	// Note: We'll need those Jacobians{i->j} where at least one "i" or "j"
	//        is a free variable (i.e. it's in nodes_to_optimize)
	for (typename graph_t::edges_map_t::const_iterator it=graph.edges.begin();it!=graph.edges.end();++it)
	{
		const TPairNodeIDs              &ids  = it->first;
		const typename graph_t::edge_t  &edge = it->second;

		if (nodes_to_optimize->find(ids.first)==nodes_to_optimize->end() &&
			nodes_to_optimize->find(ids.second)==nodes_to_optimize->end())
				continue; // Skip this edge, none of the IDs are free variables.

		// get the current global poses of both nodes in this constraint:
		typename graph_t::global_poses_t::const_iterator itP1 = graph.nodes.find(ids.first);
		typename graph_t::global_poses_t::const_iterator itP2 = graph.nodes.find(ids.second);
		ASSERTDEBMSG_(itP1!=graph.nodes.end(),"Node1 in an edge does not have a global pose in 'graph.nodes'.")
		ASSERTDEBMSG_(itP2!=graph.nodes.end(),"Node2 in an edge does not have a global pose in 'graph.nodes'.")

		// Get a reference to both global poses (the type of these will end up being CPose2D or CPose3D):
		const typename graph_t::contraint_t::type_value &P1 = itP1->second;
		const typename graph_t::contraint_t::type_value &P2 = itP2->second;
		const typename graph_t::contraint_t::type_value &EDGE_POSE  = mrpt::poses::getPoseMean<typename graph_t::contraint_t,typename graph_t::contraint_t::type_value>(edge);

		// Compute the residual pose error of these pair of nodes + its constraint, 
		//  that is: P1DP2inv = P1 * EDGE * inv(P2)
		typename graph_t::contraint_t::type_value P1DP2inv(UNINITIALIZED_POSE);
		{
			typename graph_t::contraint_t::type_value P1D(UNINITIALIZED_POSE);
			P1D.composeFrom(P1,EDGE_POSE);
			const typename graph_t::contraint_t::type_value P2inv = -P2; // Pose inverse (NOT just switching signs!)
			P1DP2inv.composeFrom(P1D,P2inv);
		}

		// Compute the jacobians:
		std::pair<TPairNodeIDs,TPairJacobs> newMapEntry; 
		newMapEntry.first = ids;
		SE_TYPE::jacobian_dP1DP2inv_depsilon(P1DP2inv, &newMapEntry.second.first,&newMapEntry.second.second);

		// And insert into map of jacobians:
		lstJacobians.insert(lstJacobians.end(),newMapEntry );
	}

	profiler.leave("compute Jacobians");

	MRPT_TODO("Continue here!")




	MRPT_END
}

// Explicit instantations:
template SLAM_IMPEXP void mrpt::graphslam::optimize_graph_spa_levmarq<CPose2D,map_traits_stdmap>(CNetworkOfPoses<CPose2D,map_traits_stdmap >&,const set<TNodeID>*, const mrpt::utils::TParametersDouble  &extra_params);
template SLAM_IMPEXP void mrpt::graphslam::optimize_graph_spa_levmarq<CPose3D,map_traits_stdmap>(CNetworkOfPoses<CPose3D,map_traits_stdmap >&,const set<TNodeID>*, const mrpt::utils::TParametersDouble  &extra_params);



