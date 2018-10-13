/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/poses/SE_traits.h>
#include <mrpt/core/aligned_std_map.h>
#include <functional>

namespace mrpt
{
/** SLAM methods related to graphs of pose constraints
 * \sa mrpt::graphs::CNetworkOfPoses   \ingroup mrpt_graphslam_grp
 */
namespace graphslam
{
/** \addtogroup mrpt_graphslam_grp
 *  @{ */

/** Auxiliary traits template for use among graph-slam problems to make life
 * easier with these complicated, long data type names
 * \tparam GRAPH_T This will typically be any
 * mrpt::graphs::CNetworkOfPoses<...>
 */
template <class GRAPH_T>
struct graphslam_traits
{
	/** Typ:  mrpt::graphs::CNetworkOfPoses<...> */
	using graph_t = GRAPH_T;
	using edge_const_iterator = typename graph_t::edges_map_t::const_iterator;
	using edge_map_entry_t = typename graph_t::edges_map_t::value_type;
	using edge_t = typename graph_t::constraint_t;
	using edge_poses_type = typename edge_t::type_value;
	using SE_TYPE =
		mrpt::poses::SE_traits<edge_poses_type::rotation_dimensions>;
	using matrix_VxV_t = typename SE_TYPE::matrix_VxV_t;
	using Array_O = typename SE_TYPE::array_t;  // An array of the correct size
	// for an "observation" (i.e. a
	// relative pose in an edge)
	using TPairJacobs = std::pair<matrix_VxV_t, matrix_VxV_t>;
	using map_pairIDs_pairJacobs_t =
		mrpt::aligned_std_multimap<mrpt::graphs::TPairNodeIDs, TPairJacobs>;

	/** Auxiliary struct used in graph-slam implementation: It holds the
	 * relevant information for each of the constraints being taking into
	 * account. */
	struct observation_info_t
	{
		using gst = graphslam_traits<GRAPH_T>;
		// Data:
		const typename gst::edge_map_entry_t* edge{nullptr};
		const typename gst::graph_t::constraint_t::type_value* edge_mean;
		typename gst::graph_t::constraint_t::type_value *P1, *P2;
	};

	using TFunctorFeedback = std::function<void(
		const GRAPH_T& graph, const size_t iter, const size_t max_iter,
		const double cur_sq_error)>;
};

/** Output information for mrpt::graphslam::optimize_graph_spa_levmarq() */
struct TResultInfoSpaLevMarq
{
	/** The number of LM iterations executed. */
	size_t num_iters;
	/** The sum of all the squared errors for every constraint involved in the
	 * problem. */
	double final_total_sq_error;
};

/**  @} */  // end of grouping

}  // namespace graphslam
}  // namespace mrpt
