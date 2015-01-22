/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/nav/planners/TMoveTree.h>
#include <mrpt/utils/CTimeLogger.h>

#include <mrpt/nav/link_pragmas.h>

namespace mrpt
{
	namespace nav
	{
		/** TP Space-based RRT path planning for SE(2) (planar) robots.
		*
		*  - Usage:
		*   - write me
		*
		*  - About the algorithm:
		*    - write me
		*
		*  - Changes history:
		*    - 06/MAR/2014: Creation (MB)
		*    - 06/JAN/2015: Refactoring (JLBC)
		*
		*  \ingroup mrpt_nav_grp
		*
		*  \todo Factorize into more generic path planner classes!  //template <class  POSE, class MOTIONS>...
		*/
		class NAV_IMPEXP PlannerRRT_SE2_TPS
		{
		public:
			struct NAV_IMPEXP TEndCriteria
			{
				double acceptedDistToTarget;  //!< Maximum distance from a pose to target to accept it as a valid solution
				double maxComputationTime;    //!< In seconds. 0 means no limit until a solution is found.

				TEndCriteria() : 
					acceptedDistToTarget ( 0.1 ),
					maxComputationTime   ( 0 )
				{
				}
			};
			TEndCriteria end_criteria;

			struct NAV_IMPEXP TAlgorithmParams
			{
				/** The robot shape used when computing collisions; it's loaded from the
				  *  config file/text as a single 2xN matrix in MATLAB format, first row are Xs, second are Ys, e.g:
				  *  \code
				  *   robot_shape = [-0.2 0.2 0.2 -0.2; -0.1 -0.1 0.1 0.1]
				  *  \endcode
				  */
				mrpt::math::TPolygon2D robot_shape;

				double goalBias;  //!< Probabily of picking the goal as random target (in [0,1], default=0.05)
				double maxLength; //!< Max length of each edge path (in meters, default=1.0)

				TAlgorithmParams() :
					goalBias(0.05),
					maxLength(1.0)
				{
					robot_shape.push_back( mrpt::math::TPoint2D(-0.5,-0.5) );
					robot_shape.push_back( mrpt::math::TPoint2D( 0.8,-0.4) );
					robot_shape.push_back( mrpt::math::TPoint2D( 0.8, 0.4) );
					robot_shape.push_back( mrpt::math::TPoint2D(-0.5, 0.5) );
				}
			};
			TAlgorithmParams params; //!< Parameters specific to this path solver algorithm

			struct NAV_IMPEXP TPlannerInput
			{
				mrpt::math::TPose2D  start_pose;
				mrpt::math::TPose2D  goal_pose;

				TPlannerInput() : 
					start_pose(0,0,0),
					goal_pose(0,0,0)
				{
				}
			};

			struct NAV_IMPEXP TPlannerResult
			{
				bool success;               //!< Whether the target was reached or not
				TMoveTreeSE2_TP move_tree;  //!< The generated motion tree that explores free space starting at "start"

				TPlannerResult() :
					success(false)
				{
				}
			};

			/** Constructor */
			PlannerRRT_SE2_TPS();

			/** Load all params from a config file source */
			void loadConfig(const mrpt::utils::CConfigFileBase &cfgSource, const std::string &sSectionName = std::string("PTG_CONFIG"));

			/** Must be called after setting all params (see `loadConfig()`) and before calling `solve()` */
			void initialize();

			/** The main API entry point: tries to find a planned path from 'goal' to 'target' */
			void solve( const TPlannerInput &pi, TPlannerResult & result );


		protected:
			mrpt::utils::CTimeLogger m_timelogger;
			bool  m_initialized; 
			mrpt::nav::TListPTGPtr m_PTGs;

		}; // end class PlannerRRT_SE2_TPS
	}
}
