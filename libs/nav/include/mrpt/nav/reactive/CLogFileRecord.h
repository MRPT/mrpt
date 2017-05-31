/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CLogFileRecord_H
#define CLogFileRecord_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/utils/TParameters.h>

#include <mrpt/nav/holonomic/CHolonomicLogFileRecord.h>
#include <mrpt/nav/holonomic/ClearanceDiagram.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/kinematics/CVehicleVelCmd.h>

namespace mrpt
{
namespace nav
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CLogFileRecord, mrpt::utils::CSerializable, NAV_IMPEXP )

	/** A class for storing, saving and loading a reactive navigation
	 *   log record for the CReactiveNavigationSystem class.
	 * \sa CReactiveNavigationSystem, CHolonomicLogFileRecord
	 *  \ingroup nav_reactive
	 */
	class NAV_IMPEXP  CLogFileRecord : public mrpt::utils::CSerializable
	{
		DEFINE_SERIALIZABLE( CLogFileRecord )

	public:
		CLogFileRecord();  //!< Constructor, builds an empty record.

		/** The structure used to store all relevant information about each
		  *  transformation into TP-Space.
		  *  \ingroup nav_reactive  */
		struct NAV_IMPEXP TInfoPerPTG
		{
			std::string              PTG_desc;      //!< A short description for the applied PTG
			mrpt::math::CVectorFloat TP_Obstacles;  //!< Distances until obstacles, in "pseudometers", first index for -PI direction, last one for PI direction.
			std::vector<mrpt::math::TPoint2D>  TP_Targets;     //!< Target(s) location in TP-Space
			mrpt::math::TPoint2D     TP_Robot;      //!< Robot location in TP-Space: normally (0,0), except during "NOP cmd vel" steps
			double timeForTPObsTransformation,timeForHolonomicMethod;  //!< Time, in seconds.
			double desiredDirection,desiredSpeed;          //!< The results from the holonomic method.
			double evaluation;                       //!< Final score of this candidate
			mrpt::utils::TParametersDouble  evalFactors;   //!< Evaluation factors
			CHolonomicLogFileRecordPtr HLFR;          //!< Other useful info about holonomic method execution.
			mrpt::nav::CParameterizedTrajectoryGeneratorPtr ptg; //!< Only for the FIRST entry in a log file, this will contain a copy of the PTG with trajectories, suitable to render trajectories, etc.
			mrpt::nav::ClearanceDiagram  clearance;    //!< Clearance for each path
		};

		mrpt::nav::CParameterizedTrajectoryGenerator::TNavDynamicState navDynState;
		uint32_t  nPTGs;  //!< The number of PTGS:
		mrpt::aligned_containers<TInfoPerPTG>::vector_t infoPerPTG; //!< The info for each applied PTG: must contain "nPTGs * nSecDistances" elements
		int32_t   nSelectedPTG;   //!< The selected PTG.

		/** Known values: 
		 *	- "executionTime": The total computation time, excluding sensing.
		 *	- "estimatedExecutionPeriod": The estimated execution period.
		 */
		std::map<std::string, double>  values;
		/** Known values:
		*	- "tim_start_iteration": Time of start of navigationStep() implementation.
		*	- "tim_send_cmd_vel": Time of sending cmdvel to robot.
		*	- "curPoseAndVel":  Time of querying robot pose and velocities.
		 */
		std::map<std::string, mrpt::system::TTimeStamp>  timestamps;
		std::map<std::string, std::string>  additional_debug_msgs;  //!< Additional debug traces
		mrpt::maps::CSimplePointsMap  WS_Obstacles, WS_Obstacles_original;  //!< The WS-Obstacles
		mrpt::math::TPose2D           robotPoseLocalization, robotPoseOdometry; //!< The robot pose (from odometry and from the localization/SLAM system).
		mrpt::math::TPose2D           relPoseSense, relPoseVelCmd; //! Relative poses (wrt to robotPoseLocalization) for extrapolated paths at two instants: time of obstacle sense, and future pose of motion comman
		std::vector<mrpt::math::TPose2D> WS_targets_relative;  //!< The relative location of target(s) in Workspace.

		mrpt::kinematics::CVehicleVelCmdPtr    cmd_vel;  //!< The final motion command sent to robot, in "m/sec" and "rad/sec".
		mrpt::kinematics::CVehicleVelCmdPtr    cmd_vel_original;  //!< Motion command as comes out from the PTG, before scaling speed limit filtering.
		mrpt::math::TTwist2D   cur_vel; //!< The actual robot velocities in global (map) coordinates, as read from sensors, in "m/sec" and "rad/sec".
		mrpt::math::TTwist2D   cur_vel_local; //!< The actual robot velocities in local (robot) coordinates, as read from sensors, in "m/sec" and "rad/sec".

		mrpt::math::CVectorFloat robotShape_x,robotShape_y;  //!< The robot shape in WS. Used by PTGs derived from mrpt::nav::CPTG_RobotShape_Polygonal
		double robotShape_radius;  //!< The circular robot radius. Used by PTGs derived from mrpt::nav::CPTG_RobotShape_Circular

		// "NOP motion command" mode variables:
		int16_t                ptg_index_NOP;  //!< Negative means no NOP mode evaluation, so the rest of "NOP variables" should be ignored.
		uint16_t               ptg_last_k_NOP;
		mrpt::math::TPose2D    rel_cur_pose_wrt_last_vel_cmd_NOP, rel_pose_PTG_origin_wrt_sense_NOP;
		mrpt::nav::CParameterizedTrajectoryGenerator::TNavDynamicState ptg_last_navDynState;
	};
	  DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CLogFileRecord, mrpt::utils::CSerializable, NAV_IMPEXP )

}
}


#endif

