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
#include <mrpt/poses/CPose2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/utils/CMemoryStream.h>

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
		  *  \ingroup nav_reactive
		  */
		struct NAV_IMPEXP TInfoPerPTG
		{
			std::string              PTG_desc;      //!< A short description for the applied PTG
			mrpt::math::CVectorFloat TP_Obstacles;  //!< Distances until obstacles, in "pseudometers", first index for -PI direction, last one for PI direction.
			mrpt::math::TPoint2D     TP_Target;     //!< Target location in TP-Space
			mrpt::math::TPoint2D     TP_Robot;      //!< Robot location in TP-Space: normally (0,0), except during "NOP cmd vel" steps
			double  timeForTPObsTransformation,timeForHolonomicMethod;  //!< Time, in seconds.
			double  desiredDirection,desiredSpeed, evaluation;          //!< The results from the holonomic method.
			mrpt::math::CVectorFloat   evalFactors;   //!< Evaluation factors
			CHolonomicLogFileRecordPtr HLFR;          //!< Other useful info about holonomic method execution.
			mrpt::nav::CParameterizedTrajectoryGeneratorPtr ptg; //!< Only for the FIRST entry in a log file, this will contain a copy of the PTG with trajectories, suitable to render trajectories, etc.
			mrpt::nav::ClearanceDiagram  clearance;    //!< Clearance for each path
		};

		//mrpt::system::TTimeStamp   timestamp;  //!< The timestamp of when this log was processed by the reactive algorithm (It can be INVALID_TIMESTAMP for navigation logs in MRPT <0.9.5)
		uint32_t       nPTGs;  //!< The number of PTGS:

		 /** The info for each applied PTG: must contain "nPTGs * nSecDistances" elements */
		mrpt::aligned_containers<TInfoPerPTG>::vector_t infoPerPTG;

		int32_t					nSelectedPTG;   //!< The selected PTG.
		/** Known values: 
		 *	- "executionTime": The total computation time, excluding sensing.
		 *	- "estimatedExecutionPeriod": The estimated execution period.
		 */
		std::map<std::string, double>  values;
		/** Known values:
		*	- "tim_start_iteration":
		*	- "tim_send_cmd_vel":
		 */
		std::map<std::string, mrpt::system::TTimeStamp>  timestamps;
		std::map<std::string, std::string>  additional_debug_msgs;  //!< Additional debug traces
		mrpt::maps::CSimplePointsMap  WS_Obstacles;  //!< The WS-Obstacles
		mrpt::poses::CPose2D          robotOdometryPose; //!< The robot pose (from raw odometry or a localization system).
		mrpt::poses::CPose2D          relPoseSense, relPoseVelCmd; //! Relative poses (wrt to robotOdometryPose) for extrapolated paths at two instants: time of obstacle sense, and future pose of motion comman
		mrpt::math::TPoint2D          WS_target_relative;  //!< The relative location of target point in WS.

		mrpt::kinematics::CVehicleVelCmdPtr    cmd_vel;  //!< The final motion command sent to robot, in "m/sec" and "rad/sec".
		mrpt::kinematics::CVehicleVelCmdPtr    cmd_vel_original;  //!< Motion command as comes out from the PTG, before scaling speed limit filtering.
		mrpt::math::TTwist2D   cur_vel; //!< The actual robot velocities in global (map) coordinates, as read from sensors, in "m/sec" and "rad/sec".
		mrpt::math::TTwist2D   cur_vel_local; //!< The actual robot velocities in local (robot) coordinates, as read from sensors, in "m/sec" and "rad/sec".

		mrpt::math::CVectorFloat robotShape_x,robotShape_y;  //!< The robot shape in WS. Used by PTGs derived from mrpt::nav::CPTG_RobotShape_Polygonal
		double robotShape_radius;  //!< The circular robot radius. Used by PTGs derived from mrpt::nav::CPTG_RobotShape_Circular

		// "NOP motion command" mode variables:
		int16_t                ptg_index_NOP;  //!< Negative means no NOP mode evaluation, so the rest of "NOP variables" should be ignored.
		uint16_t               ptg_last_k_NOP;
		mrpt::poses::CPose2D   rel_cur_pose_wrt_last_vel_cmd_NOP, rel_pose_PTG_origin_wrt_sense_NOP;
		mrpt::math::TTwist2D   ptg_last_curRobotVelLocal;

	};
	  DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CLogFileRecord, mrpt::utils::CSerializable, NAV_IMPEXP )

}
}


#endif

