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
	/** Simulates the kinematics of a differential-driven planar mobile robot/vehicle, including odometry errors and dynamics limitations.
	 *
	 * \ingroup mrpt_kinematics_grp
	 */
	class KINEMATICS_IMPEXP CVehicleSimul_DiffDriven : public CVehicleSimulVirtualBase
	{
	public:
		enum { VEL_CMD_LENGTH = 2 };

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

		/** Sends a velocity command to the robot. The number of components and their meaning depends on the derived class */
		void sendVelCmd(const std::vector<double> &cmd_vel) MRPT_OVERRIDE
		{
			ASSERT_EQUAL_(cmd_vel.size(), getVelCmdLength());
			movementCommand(cmd_vel[0],cmd_vel[1]);
		}

		size_t getVelCmdLength() const  MRPT_OVERRIDE {
			return VEL_CMD_LENGTH;
		}
		std::string getVelCmdDescription() const  MRPT_OVERRIDE {
			return std::string("vel omega");
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

		void internal_simulStep(const double dt) MRPT_OVERRIDE;
		void internal_clear() MRPT_OVERRIDE;
	};

	} // End of namespace
} // End of namespace

