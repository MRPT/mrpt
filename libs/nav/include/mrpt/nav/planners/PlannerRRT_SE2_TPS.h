/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/nav/planners/TMoveTree.h>
#include <mrpt/nav/planners/PlannerRRT_common.h>
#include <numeric>

namespace mrpt::nav
{
/** \addtogroup nav_planners Path planning
 * \ingroup mrpt_nav_grp
 * @{ */

/** TP Space-based RRT path planning for SE(2) (planar) robots.
 *
 *  This planner algorithm is described in the paper:
 *   - M. Bellone, J.L. Blanco, A. Gimenez, "TP-Space RRT: Kinematic path
 * planning of non-holonomic any-shape vehicles", International Journal of
 * Advanced Robotic Systems, 2015.
 *
 *  Typical usage:
 * \code
 * mrpt::nav::PlannerRRT_SE2_TPS  planner;
 *
 * // Set or load planner parameters:
 * //planner.loadConfig( mrpt::config::CConfigFile("config_file.cfg") );
 * //planner.params.... // See RRTAlgorithmParams
 *
 * // Set RRT end criteria (when to stop searching for a solution)
 * //planner.end_criteria.... // See RRTEndCriteria
 *
 * planner.initialize();  // Initialize after setting the algorithm parameters
 *
 * // Set up planning problem:
 * PlannerRRT_SE2_TPS::TPlannerResult planner_result;
 * PlannerRRT_SE2_TPS::TPlannerInput planner_input;
 * // Start & goal:
 * planner_input.start_pose = mrpt::math::TPose2D(XXX,XXX,XXX);
 * planner_input.goal_pose  = mrpt::math::TPose2D(XXX,XXX,XXX);
 * // Set obtacles: (...)
 * // planner_input.obstacles_points ...
 * // Set workspace bounding box for picking random poses in the RRT algorithm:
 * planner_input.world_bbox_min = mrpt::math::TPoint2D(XX,YY);
 * planner_input.world_bbox_max = mrpt::math::TPoint2D(XX,YY);
 * // Do path planning:
 * planner.solve( planner_input, planner_result);
 * // Analyze contents of planner_result...
 * \endcode
 *
 *  - Changes history:
 *    - 06/MAR/2014: Creation (MB)
 *    - 06/JAN/2015: Refactoring (JLBC)
 *
 *  \todo Factorize into more generic path planner classes!  //template <class
 * POSE, class MOTIONS>...
 */
class PlannerRRT_SE2_TPS : public PlannerTPS_VirtualBase
{
   public:
	/** The type of poses at nodes */
	using node_pose_t = mrpt::math::TPose2D;

	struct TPlannerInput : public TPlannerInputTempl<node_pose_t, node_pose_t>
	{
		TPlannerInput()
		{
			start_pose = mrpt::math::TPose2D(0, 0, 0);
			goal_pose = mrpt::math::TPose2D(0, 0, 0);
			world_bbox_min = mrpt::math::TPose2D(-10., -10.0, -M_PI);
			world_bbox_max = mrpt::math::TPose2D(10., 10.0, M_PI);
		}
	};

	struct TPlannerResult : public TPlannerResultTempl<TMoveTreeSE2_TP>
	{
	};

	/** Constructor */
	PlannerRRT_SE2_TPS();

	/** Load all params from a config file source */
	void loadConfig(
		const mrpt::config::CConfigFileBase& cfgSource,
		const std::string& sSectionName = std::string("PTG_CONFIG"));

	/** Must be called after setting all params (see `loadConfig()`) and before
	 * calling `solve()` */
	void initialize();

	/** The main API entry point: tries to find a planned path from 'goal' to
	 * 'target' */
	void solve(const TPlannerInput& pi, TPlannerResult& result);

   protected:
	bool m_initialized{false};

};  // end class PlannerRRT_SE2_TPS

/** @} */
}  // namespace mrpt::nav
