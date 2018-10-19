/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CSerializable.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/system/TParameters.h>
#include <mrpt/core/aligned_std_vector.h>

#include <mrpt/nav/holonomic/CHolonomicLogFileRecord.h>
#include <mrpt/nav/holonomic/ClearanceDiagram.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/kinematics/CVehicleVelCmd.h>

namespace mrpt::nav
{
/** A class for storing, saving and loading a reactive navigation
 *   log record for the CReactiveNavigationSystem class.
 * \sa CReactiveNavigationSystem, CHolonomicLogFileRecord
 *  \ingroup nav_reactive
 */
class CLogFileRecord : public mrpt::serialization::CSerializable
{
	DEFINE_SERIALIZABLE(CLogFileRecord)

   public:
	/** Constructor, builds an empty record. */
	CLogFileRecord();

	/** The structure used to store all relevant information about each
	 *  transformation into TP-Space.
	 *  \ingroup nav_reactive  */
	struct TInfoPerPTG
	{
		/** A short description for the applied PTG */
		std::string PTG_desc;
		/** Distances until obstacles, in "pseudometers", first index for -PI
		 * direction, last one for PI direction. */
		mrpt::math::CVectorFloat TP_Obstacles;
		/** Target(s) location in TP-Space */
		std::vector<mrpt::math::TPoint2D> TP_Targets;
		/** Robot location in TP-Space: normally (0,0), except during "NOP cmd
		 * vel" steps */
		mrpt::math::TPoint2D TP_Robot;
		/** Time, in seconds. */
		double timeForTPObsTransformation, timeForHolonomicMethod;
		/** The results from the holonomic method. */
		double desiredDirection, desiredSpeed;
		/** Final score of this candidate */
		double evaluation;
		/** Evaluation factors */
		mrpt::system::TParametersDouble evalFactors;
		/** Other useful info about holonomic method execution. */
		CHolonomicLogFileRecord::Ptr HLFR;
		/** Only for the FIRST entry in a log file, this will contain a copy of
		 * the PTG with trajectories, suitable to render trajectories, etc. */
		mrpt::nav::CParameterizedTrajectoryGenerator::Ptr ptg;
		/** Clearance for each path */
		mrpt::nav::ClearanceDiagram clearance;
	};

	mrpt::nav::CParameterizedTrajectoryGenerator::TNavDynamicState navDynState;
	/** The number of PTGS: */
	uint32_t nPTGs{0};
	/** The info for each applied PTG: must contain "nPTGs * nSecDistances"
	 * elements */
	mrpt::aligned_std_vector<TInfoPerPTG> infoPerPTG;
	/** The selected PTG. */
	int32_t nSelectedPTG{-1};

	/** Known values:
	 *	- "executionTime": The total computation time, excluding sensing.
	 *	- "estimatedExecutionPeriod": The estimated execution period.
	 */
	std::map<std::string, double> values;
	/** Known values:
	 *	- "tim_start_iteration": Time of start of navigationStep()
	 *implementation.
	 *	- "tim_send_cmd_vel": Time of sending cmdvel to robot.
	 *	- "curPoseAndVel":  Time of querying robot pose and velocities.
	 */
	std::map<std::string, mrpt::system::TTimeStamp> timestamps;
	/** Additional debug traces */
	std::map<std::string, std::string> additional_debug_msgs;
	/** The WS-Obstacles */
	mrpt::maps::CSimplePointsMap WS_Obstacles, WS_Obstacles_original;
	/** The robot pose (from odometry and from the localization/SLAM system). */
	mrpt::math::TPose2D robotPoseLocalization, robotPoseOdometry;
	mrpt::math::TPose2D relPoseSense,
		relPoseVelCmd;  //! Relative poses (wrt to robotPoseLocalization) for
	//! extrapolated paths at two instants: time of obstacle
	//! sense, and future pose of motion comman
	/** The relative location of target(s) in Workspace. */
	std::vector<mrpt::math::TPose2D> WS_targets_relative;

	/** The final motion command sent to robot, in "m/sec" and "rad/sec". */
	mrpt::kinematics::CVehicleVelCmd::Ptr cmd_vel;
	/** Motion command as comes out from the PTG, before scaling speed limit
	 * filtering. */
	mrpt::kinematics::CVehicleVelCmd::Ptr cmd_vel_original;
	/** The actual robot velocities in global (map) coordinates, as read from
	 * sensors, in "m/sec" and "rad/sec". */
	mrpt::math::TTwist2D cur_vel;
	/** The actual robot velocities in local (robot) coordinates, as read from
	 * sensors, in "m/sec" and "rad/sec". */
	mrpt::math::TTwist2D cur_vel_local;

	/** The robot shape in WS. Used by PTGs derived from
	 * mrpt::nav::CPTG_RobotShape_Polygonal */
	mrpt::math::CVectorFloat robotShape_x, robotShape_y;
	/** The circular robot radius. Used by PTGs derived from
	 * mrpt::nav::CPTG_RobotShape_Circular */
	double robotShape_radius{.0};

	// "NOP motion command" mode variables:
	/** Negative means no NOP mode evaluation, so the rest of "NOP variables"
	 * should be ignored. */
	int16_t ptg_index_NOP{-1};
	uint16_t ptg_last_k_NOP{0};
	mrpt::math::TPose2D rel_cur_pose_wrt_last_vel_cmd_NOP,
		rel_pose_PTG_origin_wrt_sense_NOP;
	mrpt::nav::CParameterizedTrajectoryGenerator::TNavDynamicState
		ptg_last_navDynState;
};
}  // namespace mrpt::nav
