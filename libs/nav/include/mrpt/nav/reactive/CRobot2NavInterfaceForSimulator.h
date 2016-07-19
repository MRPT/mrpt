/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
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
	/** CRobot2NavInterface_Holo implemented for a simulator object based on mrpt::kinematics::CVehicleSimul_Holo.
	  * Only `senseObstacles()` remains virtual for the user to implement it.
	  *
	  * \sa CReactiveNavigationSystem, CAbstractNavigator, mrpt::kinematics::CVehicleSimulVirtualBase
	  *  \ingroup nav_reactive
	  */
	class NAV_IMPEXP CRobot2NavInterfaceForSimulator_Holo : public CRobot2NavInterface_Holo
	{
	private:
		mrpt::kinematics::CVehicleSimul_Holo & m_simul;
		double m_simul_time_start; //!< for getNavigationTime

	public:
		CRobot2NavInterfaceForSimulator_Holo(mrpt::kinematics::CVehicleSimul_Holo &simul) : m_simul(simul),m_simul_time_start(.0) {}
	
		bool getCurrentPoseAndSpeeds(mrpt::math::TPose2D &curPose, mrpt::math::TTwist2D &curVel) MRPT_OVERRIDE
		{
			curPose = m_simul.getCurrentGTPose();
			curVel  = m_simul.getCurrentGTVel();
			return true; // ok
		}

		bool changeSpeeds(const std::vector<double> &vel_cmd) MRPT_OVERRIDE
		{
			m_simul.sendVelCmd(vel_cmd);
			return true; // ok
		}

		bool stop() MRPT_OVERRIDE
		{
			m_simul.sendVelRampCmd(0.0 /*vel*/, 0 /*dir*/, 0.1 /* ramp_time */, 0.0 /*rot speed */);
			return true;
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


	/** CRobot2NavInterface_DiffDriven implemented for a simulator object based on mrpt::kinematics::CVehicleSimul_DiffDriven
	  * Only `senseObstacles()` remains virtual for the user to implement it.
	  *
	  * \sa CReactiveNavigationSystem, CAbstractNavigator, mrpt::kinematics::CVehicleSimulVirtualBase
	  *  \ingroup nav_reactive
	  */
	class NAV_IMPEXP CRobot2NavInterfaceForSimulator_DiffDriven : public CRobot2NavInterface_DiffDriven
	{
	private:
		mrpt::kinematics::CVehicleSimul_DiffDriven & m_simul;
		double m_simul_time_start; //!< for getNavigationTime

	public:
		CRobot2NavInterfaceForSimulator_DiffDriven(mrpt::kinematics::CVehicleSimul_DiffDriven &simul) : m_simul(simul),m_simul_time_start(.0) {}
	
		bool getCurrentPoseAndSpeeds(mrpt::math::TPose2D &curPose, mrpt::math::TTwist2D &curVel) MRPT_OVERRIDE
		{
			curPose = m_simul.getCurrentGTPose();
			curVel  = m_simul.getCurrentGTVel();
			return true; // ok
		}

		bool changeSpeeds(const std::vector<double> &vel_cmd) MRPT_OVERRIDE
		{
			m_simul.sendVelCmd(vel_cmd);
			return true; // ok
		}

		bool stop() MRPT_OVERRIDE
		{
			std::vector<double> cmd_vel(2, 0.0);
			m_simul.sendVelCmd(cmd_vel);
			return true;
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

