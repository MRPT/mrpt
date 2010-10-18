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
#ifndef GRAPH_SLAM_H
#define GRAPH_SLAM_H

#include <mrpt/poses/CNetworkOfPoses.h>
#include <mrpt/poses/SE_traits.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
	/** SLAM methods related to graphs of pose constraints
          * \sa mrpt::poses::CNetworkOfPoses
          */
	namespace graphslam
	{
		/** Auxiliary traits template for use among graph-slam problems to make life easier with these complicated, long data type names */
		template <class EDGE_TYPE, class MAPS_IMPLEMENTATION>
		struct graphslam_traits
		{
			typedef mrpt::poses::CNetworkOfPoses<EDGE_TYPE,MAPS_IMPLEMENTATION>  graph_t;
			typedef typename graph_t::edges_map_t::const_iterator   edge_const_iterator;
			typedef typename EDGE_TYPE::type_value                  edge_poses_type;
			typedef mrpt::poses::SE_traits<edge_poses_type::rotation_dimensions> SE_TYPE;
			typedef typename SE_TYPE::matrix_VxV_t                  matrix_VxV_t;
			typedef typename SE_TYPE::array_t                       Array_O; // An array of the correct size for an "observation" (i.e. a relative pose in an edge)
			typedef std::pair<matrix_VxV_t,matrix_VxV_t>            TPairJacobs;

			/** Auxiliary struct used in graph-slam implementation: It holds the relevant information for each of the constraints being taking into account. */
			struct observation_info_t
			{
				typedef graphslam_traits<EDGE_TYPE,MAPS_IMPLEMENTATION> gst;
				// Data:
				typename gst::edge_const_iterator                     edge;
				const typename gst::graph_t::constraint_t::type_value *edge_mean;
				typename gst::graph_t::constraint_t::type_value       *P1,*P2;
			};

			typedef void (*TFunctorFeedback)(const typename graphslam_traits<EDGE_TYPE,MAPS_IMPLEMENTATION>::graph_t &graph, const size_t iter, const size_t max_iter, const double cur_sq_error );
		};

		/** Output information for mrpt::graphslam::optimize_graph_spa_levmarq() */
		struct TResultInfoSpaLevMarq
		{
			size_t  num_iters;             //!< The number of LM iterations executed.
			double  final_total_sq_error;  //!< The sum of all the squared errors for every constraint involved in the problem.
		};


		/** Optimize a graph of pose constraints using the Sparse Pose Adjustment (SPA) sparse representation and a Levenberg-Marquartd optimizer.
		  *  This method works for all types of graphs derived from \a CNetworkOfPoses (see its reference mrpt::poses::CNetworkOfPoses for the list).
		  *  The input data are all the pose constraints in \a graph (graph.edges), and the gross first estimations of the "global" pose coordinates (in graph.nodes).
		  *
		  *  Note that these first coordinates can be obtained with mrpt::poses::CNetworkOfPoses::dijkstra_nodes_estimate().
		  *
		  * The method implemented in this file is based on this work:
		  *  - "Efficient Sparse Pose Adjustment for 2D Mapping", Kurt Konolige et al., 2010.
		  * , but generalized for not only 2D but 2D and 3D poses, and using on-manifold optimization.
		  *
		  * \param[in,out] graph The input edges and output poses.
		  * \param[out] out_info Some basic output information on the process.
		  * \param[in] nodes_to_optimize The list of nodes whose global poses are to be optimized. If NULL (default), all the node IDs are optimized (but that marked as \a root in the graph).
		  * \param[in] extra_params Optional parameters, see below.
		  * \param[in] functor_feedback Optional: a pointer to a user function can be set here to be called on each LM loop iteration (eg to refresh the current state and error, refresh a GUI, etc.)
		  *
		  * List of optional parameters by name in "extra_params":
		  *		- "verbose": (default=0) If !=0, produce verbose ouput.
		  *		- "max_iterations": (default=100) Maximum number of Lev-Marq. iterations.
		  *
		  * \tparam EDGE_TYPE The type of the edges. Typically users won't have to write this template argument by hand, since the compiler will auto-fit it depending on the type of the graph object.
		  * \tparam MAPS_IMPLEMENTATION The implementation for the map: NodeID -> node_pose. Read more on this in mrpt::poses::CNetworkOfPoses
		  * \sa The example "graph_slam_demo"
		  */
		template <class EDGE_TYPE, class MAPS_IMPLEMENTATION>
		void SLAM_IMPEXP optimize_graph_spa_levmarq(
			mrpt::poses::CNetworkOfPoses<EDGE_TYPE,MAPS_IMPLEMENTATION > & graph,
			TResultInfoSpaLevMarq                                        & out_info,
			const std::set<mrpt::utils::TNodeID>            * nodes_to_optimize = NULL,
			const mrpt::utils::TParametersDouble            & extra_params = mrpt::utils::TParametersDouble(),
			typename graphslam_traits<EDGE_TYPE,MAPS_IMPLEMENTATION>::TFunctorFeedback  functor_feedback = NULL
			);




	} // End of namespace
} // End of namespace

#endif
