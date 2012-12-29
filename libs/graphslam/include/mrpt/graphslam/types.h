/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef GRAPH_SLAM_TYPES_H
#define GRAPH_SLAM_TYPES_H

#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/poses/SE_traits.h>

namespace mrpt
{
	/** SLAM methods related to graphs of pose constraints
          * \sa mrpt::graphs::CNetworkOfPoses   \ingroup mrpt_graphslam_grp
          */
	namespace graphslam
	{
		/** \addtogroup mrpt_graphslam_grp
		  *  @{ */

		/** Auxiliary traits template for use among graph-slam problems to make life easier with these complicated, long data type names
		  * \tparam GRAPH_T This will typically be any mrpt::graphs::CNetworkOfPoses<...>
		  */
		template <class GRAPH_T>
		struct graphslam_traits
		{
			//typedef mrpt::graphs::CNetworkOfPoses<EDGE_TYPE,MAPS_IMPLEMENTATION>  graph_t;
			typedef GRAPH_T  graph_t;
			typedef typename graph_t::edges_map_t::const_iterator   edge_const_iterator;
			typedef typename graph_t::constraint_t                  edge_t;
			typedef typename edge_t::type_value                     edge_poses_type;
			typedef mrpt::poses::SE_traits<edge_poses_type::rotation_dimensions> SE_TYPE;
			typedef typename SE_TYPE::matrix_VxV_t                  matrix_VxV_t;
			typedef typename SE_TYPE::array_t                       Array_O; // An array of the correct size for an "observation" (i.e. a relative pose in an edge)
			typedef std::pair<matrix_VxV_t,matrix_VxV_t>            TPairJacobs;
			typedef typename mrpt::aligned_containers<
				mrpt::utils::TPairNodeIDs,
				TPairJacobs
				>::map_t  map_pairIDs_pairJacobs_t;

			/** Auxiliary struct used in graph-slam implementation: It holds the relevant information for each of the constraints being taking into account. */
			struct observation_info_t
			{
				typedef graphslam_traits<GRAPH_T> gst;
				// Data:
				typename gst::edge_const_iterator                     edge;
				const typename gst::graph_t::constraint_t::type_value *edge_mean;
				typename gst::graph_t::constraint_t::type_value       *P1,*P2;
			};

			typedef void (*TFunctorFeedback)(const GRAPH_T &graph, const size_t iter, const size_t max_iter, const double cur_sq_error );
		};

		/** Output information for mrpt::graphslam::optimize_graph_spa_levmarq() */
		struct TResultInfoSpaLevMarq
		{
			size_t  num_iters;             //!< The number of LM iterations executed.
			double  final_total_sq_error;  //!< The sum of all the squared errors for every constraint involved in the problem.
		};

	/**  @} */  // end of grouping

	} // End of namespace
} // End of namespace

#endif
