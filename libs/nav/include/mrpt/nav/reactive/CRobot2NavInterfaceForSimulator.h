/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/nav/reactive/CRobot2NavInterface.h>
#include <mrpt/kinematics/CVehicleSimul_Holo.h>
#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>

namespace mrpt
{
  namespace nav
  {
	/** CRobot2NavInterface implemented for a simulator object based on mrpt::kinematics::CVehicleSimul_Holo.
	  * Only `senseObstacles()` remains virtual for the user to implement it.
	  *
	  * \sa CReactiveNavigationSystem, CAbstractNavigator, mrpt::kinematics::CVehicleSimulVirtualBase
	  *  \ingroup nav_reactive
	  */
	class NAV_IMPEXP CRobot2NavInterfaceForSimulator_Holo : public CRobot2NavInterface
	{
	private:
		mrpt::kinematics::CVehicleSimul_Holo & m_simul;
		double m_simul_time_start; //!< for getNavigationTime

	public:
		CRobot2NavInterfaceForSimulator_Holo(mrpt::kinematics::CVehicleSimul_Holo &simul) : m_simul(simul),m_simul_time_start(.0) {}
	
		bool getCurrentPoseAndSpeeds(mrpt::math::TPose2D &curPose, mrpt::math::TTwist2D &curVel, mrpt::system::TTimeStamp &timestamp) MRPT_OVERRIDE
		{
			curPose = m_simul.getCurrentGTPose();
			curVel  = m_simul.getCurrentGTVel();
			timestamp = mrpt::system::now();
			return true; // ok
		}

		virtual bool changeSpeeds(const mrpt::kinematics::CVehicleVelCmd &vel_cmd) MRPT_OVERRIDE
		{
			m_simul.sendVelCmd(vel_cmd);
			return true; // ok
		}

		bool stop(bool isEmergencyStop) MRPT_OVERRIDE
		{
			m_simul.sendVelRampCmd(0.0 /*vel*/, 0 /*dir*/, isEmergencyStop ? 0.1 : 1.0 /* ramp_time */, 0.0 /*rot speed */);
			return true;
		}

		mrpt::kinematics::CVehicleVelCmdPtr getEmergencyStopCmd() MRPT_OVERRIDE
		{
			return mrpt::kinematics::CVehicleVelCmdPtr(new mrpt::kinematics::CVehicleVelCmd_Holo(0.0,0.0,0.1,0.0));
		}

		mrpt::kinematics::CVehicleVelCmdPtr getStopCmd() MRPT_OVERRIDE
		{
			return mrpt::kinematics::CVehicleVelCmdPtr(new mrpt::kinematics::CVehicleVelCmd_Holo(0.0,0.0,1.0,0.0));
		}

		/** See CRobot2NavInterface::getNavigationTime(). In this class, simulation time is returned instead of wall-clock time. */
		double getNavigationTime() MRPT_OVERRIDE {
			return m_simul.getTime()-m_simul_time_start;
		}
		/** See CRobot2NavInterface::resetNavigationTimer() */
		void resetNavigationTimer() MRPT_OVERRIDE {
			m_simul_time_start = m_simul.getTime();
		}
	};


	/** CRobot2NavInterface implemented for a simulator object based on mrpt::kinematics::CVehicleSimul_DiffDriven
	  * Only `senseObstacles()` remains virtual for the user to implement it.
	  *
	  * \sa CReactiveNavigationSystem, CAbstractNavigator, mrpt::kinematics::CVehicleSimulVirtualBase
	  *  \ingroup nav_reactive
	  */
	class NAV_IMPEXP CRobot2NavInterfaceForSimulator_DiffDriven : public CRobot2NavInterface
	{
	private:
		mrpt::kinematics::CVehicleSimul_DiffDriven & m_simul;
		double m_simul_time_start; //!< for getNavigationTime

	public:
		CRobot2NavInterfaceForSimulator_DiffDriven(mrpt::kinematics::CVehicleSimul_DiffDriven &simul) : m_simul(simul),m_simul_time_start(.0) {}
	
		bool getCurrentPoseAndSpeeds(mrpt::math::TPose2D &curPose, mrpt::math::TTwist2D &curVel, mrpt::system::TTimeStamp &timestamp) MRPT_OVERRIDE
		{
			curPose = m_simul.getCurrentGTPose();
			curVel  = m_simul.getCurrentGTVel();
			timestamp = mrpt::system::now();
			return true; // ok
		}

		bool changeSpeeds(const mrpt::kinematics::CVehicleVelCmd &vel_cmd) MRPT_OVERRIDE
		{
			m_simul.sendVelCmd(vel_cmd);
			return true; // ok
		}

		bool stop(bool isEmergencyStop) MRPT_OVERRIDE
		{
			mrpt::kinematics::CVehicleVelCmd_DiffDriven cmd;
			cmd.setToStop();
			m_simul.sendVelCmd(cmd);
			return true;
		}


		mrpt::kinematics::CVehicleVelCmdPtr getStopCmd() MRPT_OVERRIDE
		{
			mrpt::kinematics::CVehicleVelCmdPtr cmd(new mrpt::kinematics::CVehicleVelCmd_DiffDriven());
			cmd->setToStop();
			return cmd;
		}
		mrpt::kinematics::CVehicleVelCmdPtr getEmergencyStopCmd() MRPT_OVERRIDE
		{
			return getStopCmd();
		}

		/** See CRobot2NavInterface::getNavigationTime(). In this class, simulation time is returned instead of wall-clock time. */
		double getNavigationTime() MRPT_OVERRIDE {
			return m_simul.getTime()-m_simul_time_start;
		}
		/** See CRobot2NavInterface::resetNavigationTimer() */
		void resetNavigationTimer() MRPT_OVERRIDE {
			m_simul_time_start = m_simul.getTime();
		}
	};

  }
}

