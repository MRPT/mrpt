/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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
#ifndef GRAPH_SLAM_LEVMARQ_H
#define GRAPH_SLAM_LEVMARQ_H

#include <mrpt/graphslam/types.h>
#include <mrpt/utils/TParameters.h>

namespace mrpt
{
	namespace graphslam
	{
		/** Optimize a graph of pose constraints using the Sparse Pose Adjustment (SPA) sparse representation and a Levenberg-Marquartd optimizer.
		  *  This method works for all types of graphs derived from \a CNetworkOfPoses (see its reference mrpt::graphs::CNetworkOfPoses for the list).
		  *  The input data are all the pose constraints in \a graph (graph.edges), and the gross first estimations of the "global" pose coordinates (in graph.nodes).
		  *
		  *  Note that these first coordinates can be obtained with mrpt::graphs::CNetworkOfPoses::dijkstra_nodes_estimate().
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
		  *		- "scale_hessian": (default=0.1) Multiplies the Hessian matrix by this scalar (may improve convergence speed).
		  *		- "initial_lambda": (default=0) <=0 means auto guess, otherwise, initial lambda value for the lev-marq algorithm.
		  *		- "tau": (default=1e-3) Initial tau value for the lev-marq algorithm.
		  *		- "e1": (default=1e-6) Lev-marq algorithm iteration stopping criterion #1: |gradient| < e1
		  *		- "e2": (default=1e-6) Lev-marq algorithm iteration stopping criterion #2: |delta_incr| < e2*(x_norm+e2)
		  *
		  * \note The following graph types are supported: mrpt::graphs::CNetworkOfPoses2D, mrpt::graphs::CNetworkOfPoses3D, mrpt::graphs::CNetworkOfPoses2DInf, mrpt::graphs::CNetworkOfPoses3DInf
		  *
		  * \tparam GRAPH_T Normally a mrpt::graphs::CNetworkOfPoses<EDGE_TYPE,MAPS_IMPLEMENTATION>. Users won't have to write this template argument by hand, since the compiler will auto-fit it depending on the type of the graph object.
		  * \sa The example "graph_slam_demo"
		  * \ingroup mrpt_graphslam_grp
		  * \note Implementation can be found in file \a levmarq_impl.h
		  */
		template <class GRAPH_T>
		void optimize_graph_spa_levmarq(
			GRAPH_T & graph,
			TResultInfoSpaLevMarq                                        & out_info,
			const std::set<mrpt::utils::TNodeID>            * nodes_to_optimize = NULL,
			const mrpt::utils::TParametersDouble            & extra_params = mrpt::utils::TParametersDouble(),
			typename graphslam_traits<GRAPH_T>::TFunctorFeedback  functor_feedback = NULL
			);

	/**  @} */  // end of grouping

	} // End of namespace
} // End of namespace

#include <mrpt/graphslam/levmarq_impl.h>

#endif
