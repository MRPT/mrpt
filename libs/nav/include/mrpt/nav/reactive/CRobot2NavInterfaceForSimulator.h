/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/nav/reactive/CRobot2NavInterface.h>
#include <mrpt/kinematics/CVehicleSimul_Holo.h>
#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>

namespace mrpt::nav
{
/** CRobot2NavInterface implemented for a simulator object based on
 * mrpt::kinematics::CVehicleSimul_Holo.
 * Only `senseObstacles()` remains virtual for the user to implement it.
 *
 * \sa CReactiveNavigationSystem, CAbstractNavigator,
 * mrpt::kinematics::CVehicleSimulVirtualBase
 *  \ingroup nav_reactive
 */
class CRobot2NavInterfaceForSimulator_Holo : public CRobot2NavInterface
{
   private:
	mrpt::kinematics::CVehicleSimul_Holo& m_simul;
	/** for getNavigationTime */
	double m_simul_time_start;

   public:
	CRobot2NavInterfaceForSimulator_Holo(
		mrpt::kinematics::CVehicleSimul_Holo& simul)
		: m_simul(simul), m_simul_time_start(.0)
	{
	}

	bool getCurrentPoseAndSpeeds(
		mrpt::math::TPose2D& curPose, mrpt::math::TTwist2D& curVel,
		mrpt::system::TTimeStamp& timestamp, mrpt::math::TPose2D& curOdometry,
		std::string& frame_id) override
	{
		curPose = m_simul.getCurrentGTPose();
		curVel = m_simul.getCurrentGTVel();
		timestamp = mrpt::system::now();
		curOdometry = m_simul.getCurrentOdometricPose();
		return true;  // ok
	}

	bool changeSpeeds(const mrpt::kinematics::CVehicleVelCmd& vel_cmd) override
	{
		m_simul.sendVelCmd(vel_cmd);
		return true;  // ok
	}

	bool stop(bool isEmergencyStop) override
	{
		m_simul.sendVelRampCmd(
			0.0 /*vel*/, 0 /*dir*/, isEmergencyStop ? 0.1 : 1.0 /* ramp_time */,
			0.0 /*rot speed */);
		return true;
	}

	mrpt::kinematics::CVehicleVelCmd::Ptr getEmergencyStopCmd() override
	{
		return mrpt::kinematics::CVehicleVelCmd::Ptr(
			new mrpt::kinematics::CVehicleVelCmd_Holo(0.0, 0.0, 0.1, 0.0));
	}

	mrpt::kinematics::CVehicleVelCmd::Ptr getStopCmd() override
	{
		return mrpt::kinematics::CVehicleVelCmd::Ptr(
			new mrpt::kinematics::CVehicleVelCmd_Holo(0.0, 0.0, 1.0, 0.0));
	}

	mrpt::kinematics::CVehicleVelCmd::Ptr getAlignCmd(
		const double relative_heading_radians) override
	{
		return mrpt::kinematics::CVehicleVelCmd::Ptr(
			new mrpt::kinematics::CVehicleVelCmd_Holo(
				0.0,  // vel
				relative_heading_radians,  // local_dir
				0.5,  // ramp_time
				mrpt::signWithZero(relative_heading_radians) *
					mrpt::DEG2RAD(40.0)  // rotvel
				));
	}

	/** See CRobot2NavInterface::getNavigationTime(). In this class, simulation
	 * time is returned instead of wall-clock time. */
	double getNavigationTime() override
	{
		return m_simul.getTime() - m_simul_time_start;
	}
	/** See CRobot2NavInterface::resetNavigationTimer() */
	void resetNavigationTimer() override
	{
		m_simul_time_start = m_simul.getTime();
	}
};

/** CRobot2NavInterface implemented for a simulator object based on
 * mrpt::kinematics::CVehicleSimul_DiffDriven
 * Only `senseObstacles()` remains virtual for the user to implement it.
 *
 * \sa CReactiveNavigationSystem, CAbstractNavigator,
 * mrpt::kinematics::CVehicleSimulVirtualBase
 *  \ingroup nav_reactive
 */
class CRobot2NavInterfaceForSimulator_DiffDriven : public CRobot2NavInterface
{
   private:
	mrpt::kinematics::CVehicleSimul_DiffDriven& m_simul;
	/** for getNavigationTime */
	double m_simul_time_start;

   public:
	CRobot2NavInterfaceForSimulator_DiffDriven(
		mrpt::kinematics::CVehicleSimul_DiffDriven& simul)
		: m_simul(simul), m_simul_time_start(.0)
	{
	}

	bool getCurrentPoseAndSpeeds(
		mrpt::math::TPose2D& curPose, mrpt::math::TTwist2D& curVel,
		mrpt::system::TTimeStamp& timestamp, mrpt::math::TPose2D& curOdometry,
		std::string& frame_id) override
	{
		curPose = m_simul.getCurrentGTPose();
		curVel = m_simul.getCurrentGTVel();
		timestamp = mrpt::system::now();
		curOdometry = m_simul.getCurrentOdometricPose();
		return true;  // ok
	}

	bool changeSpeeds(const mrpt::kinematics::CVehicleVelCmd& vel_cmd) override
	{
		m_simul.sendVelCmd(vel_cmd);
		return true;  // ok
	}

	bool stop(bool isEmergencyStop) override
	{
		mrpt::kinematics::CVehicleVelCmd_DiffDriven cmd;
		cmd.setToStop();
		m_simul.sendVelCmd(cmd);
		return true;
	}

	mrpt::kinematics::CVehicleVelCmd::Ptr getStopCmd() override
	{
		mrpt::kinematics::CVehicleVelCmd::Ptr cmd(
			new mrpt::kinematics::CVehicleVelCmd_DiffDriven());
		cmd->setToStop();
		return cmd;
	}
	mrpt::kinematics::CVehicleVelCmd::Ptr getEmergencyStopCmd() override
	{
		return getStopCmd();
	}

	/** See CRobot2NavInterface::getNavigationTime(). In this class, simulation
	 * time is returned instead of wall-clock time. */
	double getNavigationTime() override
	{
		return m_simul.getTime() - m_simul_time_start;
	}
	/** See CRobot2NavInterface::resetNavigationTimer() */
	void resetNavigationTimer() override
	{
		m_simul_time_start = m_simul.getTime();
	}
};
}  // namespace mrpt::nav
