/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/kinematics/CVehicleSimulVirtualBase.h>
#include <mrpt/kinematics/CVehicleVelCmd_DiffDriven.h>

namespace mrpt
{
namespace kinematics
{
	/** Simulates the kinematics of a differential-driven planar mobile robot/vehicle, including odometry errors and dynamics limitations.
	 *
	 * \ingroup mrpt_kinematics_grp
	 */
	class KINEMATICS_IMPEXP CVehicleSimul_DiffDriven : public CVehicleSimulVirtualBase
	{
	public:
		typedef CVehicleVelCmd_DiffDriven  kinematic_cmd_t;

		CVehicleSimul_DiffDriven();
		virtual ~CVehicleSimul_DiffDriven();

		/** Change the model of delays used for the orders sent to the robot \sa movementCommand */
		void setDelayModelParams(double TAU_delay_sec=1.8, double CMD_delay_sec=0.) {
			cTAU = TAU_delay_sec;
			cDELAY = CMD_delay_sec;
		}

		void setV(double v) { m_v=v; }
		void setW(double w) { m_w=w; }

		double getV() {return m_v;}
		double getW() {return m_w;}

		/** Used to command the robot a desired movement: 
			* \param lin_vel Linar velocity (m/s)
			* \param ang_vel Angular velocity (rad/s)
			*/
		void  movementCommand(double lin_vel, double ang_vel );

		void sendVelCmd(const CVehicleVelCmd &cmd_vel) MRPT_OVERRIDE {
			const kinematic_cmd_t* cmd = dynamic_cast<const kinematic_cmd_t*>(&cmd_vel);
			ASSERTMSG_(cmd, "Wrong vehicle kinematic class, expected `CVehicleVelCmd_DiffDriven`");
			movementCommand(cmd->lin_vel, cmd->ang_vel);
		}
		CVehicleVelCmdPtr getVelCmdType() const MRPT_OVERRIDE {
			return CVehicleVelCmdPtr( new kinematic_cmd_t() );
		}

	private:
		double m_v, m_w; //!< lin & angular velocity in the robot local frame.
			
		/** Dynamic limitations of the robot.
			* Approximation to non-infinity motor forces: A first order low-pass filter, using:
			*   Command_Time: Time "t" when the last order was received.
			*   Command_v, Command_w: The user-desired velocities.
			*   Command_v0, Command_w0: Actual robot velocities at the moment of user request.
			*/
		double Command_Time,
			    Command_v, Command_w,
				Command_v0, Command_w0;

		double cTAU;  //!< The time-constants for the first order low-pass filter for the velocities changes.
		double cDELAY; //!< The delay constant for the velocities changes

		void internal_simulControlStep(const double dt) MRPT_OVERRIDE;
		void internal_clear() MRPT_OVERRIDE;
	};

	} // End of namespace
} // End of namespace

