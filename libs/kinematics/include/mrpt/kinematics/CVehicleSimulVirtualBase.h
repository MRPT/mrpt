/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/poses/CPose2D.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/system/datetime.h>
#include <mrpt/kinematics/CVehicleVelCmd.h>
#include <mrpt/kinematics/link_pragmas.h>

namespace mrpt
{
namespace kinematics
{
	/** This class can be used to simulate the kinematics and dynamics of a differential driven planar mobile robot, including odometry errors and dynamics limitations.
	 *  Main API methods are:
	 *  - movementCommand: Call this for send a command to the robot. This comamnd will be
	 *    delayed and passed throught a first order low-pass filter to simulate
	 *    robot dynamics.
	 *  - simulateInterval: Call this for run the simulator for the desired time period.
	 *
	 * \ingroup mrpt_kinematics_grp
	 */
	class KINEMATICS_IMPEXP CVehicleSimulVirtualBase
	{
	public:
		CVehicleSimulVirtualBase();
		virtual ~CVehicleSimulVirtualBase();

		/** @name Kinematic simulation and control interface
		  * @{ */
		/** Runs the simulator during "dt" seconds. It will be split into periods of "m_firmware_control_period". */
		void simulateOneTimeStep(const double dt);

		/** Returns the instantaneous, ground truth pose in world coordinates */
		const mrpt::math::TPose2D  & getCurrentGTPose() const { return m_GT_pose; }
		/** Brute-force move robot to target coordinates ("teleport") */
		void setCurrentGTPose(const mrpt::math::TPose2D  &pose);


		/** Returns the current pose according to (noisy) odometry \sa setOdometryErrors */
		const mrpt::math::TPose2D  & getCurrentOdometricPose() const { return m_odometry; }
		/** Brute-force overwrite robot odometry  */
		template<typename T>
		void setCurrentOdometricPose(const T &pose) { m_odometry = mrpt::math::TPose2D(pose); }

		/** Returns the instantaneous, ground truth velocity vector (vx,vy,omega) in world coordinates */
		const mrpt::math::TTwist2D & getCurrentGTVel() const { return m_GT_vel; }
		/** Returns the instantaneous, ground truth velocity vector (vx,vy,omega) in the robot local frame */
		mrpt::math::TTwist2D getCurrentGTVelLocal() const;

		/** Returns the instantaneous, odometric velocity vector (vx,vy,omega) in world coordinates */
		const mrpt::math::TTwist2D & getCurrentOdometricVel() const { return m_odometric_vel; }
		/** Returns the instantaneous, odometric velocity vector (vx,vy,omega) in the robot local frame */
		mrpt::math::TTwist2D getCurrentOdometricVelLocal() const;


		/** Get the current simulation time */
		double getTime() const { return m_time;}

		/** Sends a velocity command to the robot. The number of components and their meaning depends 
		  * on the vehicle-kinematics derived class */
		virtual void sendVelCmd(const CVehicleVelCmd &cmd_vel) = 0;
		/** Gets an empty velocity command object that can be queried to find out the number of velcmd components,... */
		virtual CVehicleVelCmdPtr getVelCmdType() const = 0;

		/** Enable/Disable odometry errors. Errors in odometry are 1 sigma Gaussian values per second */
		void setOdometryErrors(
			bool enabled,
			double Ax_err_bias  =  1e-3,
			double Ax_err_std   = 10e-3,
			double Ay_err_bias =  1e-3,
			double Ay_err_std  = 10e-3,
			double Aphi_err_bias =  mrpt::utils::DEG2RAD(1e-3),
			double Aphi_err_std  = mrpt::utils::DEG2RAD(10e-3)
			)
		{
			m_use_odo_error=enabled;
			m_Ax_err_bias=Ax_err_bias;
			m_Ax_err_std=Ax_err_std;
			m_Ay_err_bias=Ay_err_bias;
			m_Ay_err_std=Ay_err_std;
			m_Aphi_err_bias=Aphi_err_bias;
			m_Aphi_err_std=Aphi_err_std;
		}

		void resetStatus(); //! Reset all simulator variables to 0 (except the simulation time). \sa resetTime
		void resetTime(); //!< Reset time counter \sa resetStatus

		/** @} */

	protected:
		/** @name State vector
		 *  @{ */
		double                m_time;  //!< simulation running time
		mrpt::math::TPose2D   m_GT_pose;  //!< ground truth pose in world coordinates.
		mrpt::math::TTwist2D  m_GT_vel;   //!< Velocity in (x,y,omega)
		mrpt::math::TTwist2D  m_odometric_vel;   //!< Velocity in (x,y,omega)
		mrpt::math::TPose2D   m_odometry;
		/** @} */
		double m_firmware_control_period;  //!< The period at which the low-level controller updates velocities (Default: 0.5 ms)

		bool    m_use_odo_error; //!< Whether to corrupt odometry with noise
		double m_Ax_err_bias, m_Ax_err_std;
		double m_Ay_err_bias, m_Ay_err_std;
		double m_Aphi_err_bias, m_Aphi_err_std;

		virtual void internal_simulControlStep(const double dt) = 0;
		virtual void internal_clear() =0; //!< Resets all pending cmds
	private:

	};

	} // End of namespace
} // End of namespace
