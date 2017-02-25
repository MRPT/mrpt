/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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
			typedef GRAPH_T  graph_t;  //!< Typ:  mrpt::graphs::CNetworkOfPoses<...>
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
				>::multimap_t  map_pairIDs_pairJacobs_t;

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
