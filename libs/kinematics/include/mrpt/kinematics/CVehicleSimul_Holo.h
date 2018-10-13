/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/kinematics/CVehicleSimulVirtualBase.h>
#include <mrpt/kinematics/CVehicleVelCmd_Holo.h>

namespace mrpt::kinematics
{
/** Kinematic simulator of a holonomic 2D robot capable of moving in any
 * direction, with "blended"
 * velocity profiles. See CVehicleSimul_Holo::sendVelCmd() for a description of
 * the velocity commands in this kinematic model.
 * \ingroup mrpt_kinematics_grp
 */
class CVehicleSimul_Holo : public CVehicleSimulVirtualBase
{
   public:
	using kinematic_cmd_t = CVehicleVelCmd_Holo;

	CVehicleSimul_Holo();

	/** Sends a velocity cmd to the holonomic robot.
	 *   \param[in] vel Linear speed (m/s)
	 *   \param[in] dir Direction (rad) (In the odometry frame of reference)
	 *   \param[in] ramp_time Blend the cmd during this period (seconds)
	 *   \param[in] rot_speed Rotational speed while there is heading error and
	 * to this constant (rad/s)
	 */
	void sendVelRampCmd(
		double vel, double dir, double ramp_time, double rot_speed);

	void sendVelCmd(const CVehicleVelCmd& cmd_vel) override
	{
		const auto* cmd = dynamic_cast<const kinematic_cmd_t*>(&cmd_vel);
		ASSERTMSG_(
			cmd,
			"Wrong vehicle kinematic class, expected `CVehicleVelCmd_Holo`");
		sendVelRampCmd(
			cmd->vel,
			cmd->dir_local + m_odometry.phi /* local to odometry dir */,
			cmd->ramp_time, cmd->rot_speed);
	}
	CVehicleVelCmd::Ptr getVelCmdType() const override
	{
		return CVehicleVelCmd::Ptr(new kinematic_cmd_t());
	}

   private:
	/** @name Vel ramp cmds
	 *  @{ */
	struct TVelRampCmd
	{
		/** time when the cmd was issued. (<0: invalid, means there are no
		 * pending cmds to execute) */
		double issue_time{-1.0};
		double target_vel_x, target_vel_y, ramp_time, rot_speed, dir;
		mrpt::math::TTwist2D init_vel;

		TVelRampCmd() = default;
	};
	/** the last cmd received from the user. */
	TVelRampCmd m_vel_ramp_cmd;
	/** @} */

	void internal_simulControlStep(const double dt) override;
	void internal_clear() override;
};
}  // namespace mrpt::kinematics
