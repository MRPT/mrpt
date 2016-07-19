/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/kinematics/CVehicleSimulVirtualBase.h>

namespace mrpt
{
namespace kinematics 
{
	/** Kinematic simulator of a holonomic 2D robot capable of moving in any direction, with "blended"
	  * velocity profiles. See CVehicleSimul_Holo::sendVelCmd() for a description of the velocity commands in this kinematic model.
	 * \ingroup mrpt_kinematics_grp
	  */
	class KINEMATICS_IMPEXP CVehicleSimul_Holo : public CVehicleSimulVirtualBase
	{
	public:
		enum { VEL_CMD_LENGTH = 4 };

		CVehicleSimul_Holo();

		/** Sends a velocity cmd to the holonomic robot.
		 *   \param[in] vel Linear speed (m/s)
		 *   \param[in] dir Direction (rad) (In the odometry frame of reference)
		 *   \param[in] ramp_time Blend the cmd during this period (seconds)
		 *   \param[in] rot_speed Rotational speed while there is heading error and to this constant (rad/s)
		 */
		void sendVelRampCmd(double vel, double dir, double ramp_time, double rot_speed);

		/** Sends a velocity command to the robot:
		  * `cmd_vel=[vel dir_local ramp_time rot_speed]`:
		  * - vel: speed (m/s)
		  * - dir_local: direction, **relative** to the current robot heading (radians). 0 means forward.
		  * - ramp_time: Blending time between current and target time.
		  * - rot_speed: (rad/s) rotational speed for rotating such as the robot slowly faces forward.
		  */
		void sendVelCmd(const std::vector<double> &cmd_vel) MRPT_OVERRIDE
		{
			ASSERT_EQUAL_(cmd_vel.size(), getVelCmdLength());
			sendVelRampCmd(cmd_vel[0],cmd_vel[1]+m_pose.phi,cmd_vel[2],cmd_vel[3]);
		}

		size_t getVelCmdLength() const  MRPT_OVERRIDE {
			return VEL_CMD_LENGTH;
		}
		std::string getVelCmdDescription() const  MRPT_OVERRIDE {
			return std::string("vel dir ramp_time rot_speed");
		}

	private:
		/** @name Vel ramp cmds
		 *  @{ */
		struct TVelRampCmd
		{
			double issue_time;  //!< time when the cmd was issued. (<0: invalid, means there are no pending cmds to execute)
			double target_vel_x, target_vel_y, ramp_time, rot_speed, dir;
			mrpt::math::TTwist2D init_vel;

			TVelRampCmd(): issue_time(-1.0) {}
		};
		TVelRampCmd m_vel_ramp_cmd; //!< the last cmd received from the user.
		/** @} */
		
		void internal_simulStep(const double dt) MRPT_OVERRIDE;
		void internal_clear() MRPT_OVERRIDE;

	};
}
}