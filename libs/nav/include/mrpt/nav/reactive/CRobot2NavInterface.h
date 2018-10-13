/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/obs/obs_frwds.h>  // CSimplePointsMap
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/datetime.h>
#include <mrpt/kinematics/CVehicleVelCmd.h>

namespace mrpt::nav
{
/** The pure virtual interface between a real or simulated robot and any
 * `CAbstractNavigator`-derived class.
 *
 *  The user must define a new class derived from `CRobot2NavInterface` and
 * reimplement
 *   all pure virtual and the desired virtual methods according to the
 * documentation in this class.
 *
 * [New in MRPT 1.5.0] This class does not make assumptions about the kinematic
 * model of the robot, so it can work with either
 *  Ackermann, differential-driven or holonomic robots. It will depend on the
 * used PTGs, so checkout
 *  each PTG documentation for the lenght and meaning of velocity commands.
 *
 * If used for a simulator, users may prefer to inherit from one of these
 * classes, which already provide partial implementations:
 *  - mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven
 *  - mrpt::nav::CRobot2NavInterfaceForSimulator_Holo
 *
 * \sa CReactiveNavigationSystem, CAbstractNavigator
 *  \ingroup nav_reactive
 */
class CRobot2NavInterface : public mrpt::system::COutputLogger
{
   public:
	CRobot2NavInterface();
	~CRobot2NavInterface() override;

	/** Get the current pose and velocity of the robot. The implementation
	 * should not take too much time to return,
	 *   so if it might take more than ~10ms to ask the robot for the
	 * instantaneous data, it may be good enough to
	 *   return the latest values from a cache which is updated in a parallel
	 * thread.
	 * \return false on any error retrieving these values from the robot.
	 * \callergraph */
	virtual bool getCurrentPoseAndSpeeds(
		/** (output) The latest robot pose (typically from a
		   mapping/localization module), in world coordinates. (x,y: meters,
		   phi: radians) */
		mrpt::math::TPose2D& curPose,
		/** (output) The latest robot velocity vector, in world coordinates.
		   (vx,vy: m/s, omega: rad/s) */
		mrpt::math::TTwist2D& curVelGlobal,
		/** (output) The timestamp for the read pose and velocity values. Use
		   mrpt::system::now() unless you have something more accurate. */
		mrpt::system::TTimeStamp& timestamp,
		/** (output) The latest robot raw odometry pose; may have long-time
		   drift should be more locally consistent than curPose (x,y: meters,
		   phi: radians) */
		mrpt::math::TPose2D& curOdometry,
		/** (output) ID of the coordinate frame for curPose. Default is not
		   modified is "map". [Only for future support to submapping,etc.] */
		std::string& frame_id) = 0;

	/** Sends a velocity command to the robot.
	 * The number components in each command depends on children classes of
	 * mrpt::kinematics::CVehicleVelCmd.
	 * One robot may accept one or more different CVehicleVelCmd classes.
	 * This method resets the watchdog timer (that may be or may be not
	 * implemented in a particular robotic platform) started with
	 * startWatchdog()
	 * \return false on any error.
	 * \sa startWatchdog
	 * \callergraph
	 */
	virtual bool changeSpeeds(
		const mrpt::kinematics::CVehicleVelCmd& vel_cmd) = 0;

	/** Just like changeSpeeds(), but will be called when the last velocity
	 * command is still the preferred solution,
	 * so there is no need to change that past command. The unique effect of
	 * this callback would be resetting the watchdog timer.
	 * \return false on any error.
	 * \sa changeSpeeds(), startWatchdog()
	 * \callergraph */
	virtual bool changeSpeedsNOP();

	/** Stop the robot right now.
	 *  \param[in] isEmergencyStop true if stop is due to some unexpected error.
	 * false if "stop" happens as part of a normal operation (e.g. target
	 * reached).
	 * \return false on any error.
	 */
	virtual bool stop(bool isEmergencyStop = true) = 0;

	/** Gets the emergency stop command for the current robot
	 * \return the emergency stop command
	 */
	virtual mrpt::kinematics::CVehicleVelCmd::Ptr getEmergencyStopCmd() = 0;

	/** Gets the emergency stop command for the current robot
	 * \return the emergency stop command
	 */
	virtual mrpt::kinematics::CVehicleVelCmd::Ptr getStopCmd() = 0;

	/** Gets a motion command to make the robot to align with a given *relative*
	 * heading, without translating.
	 * Only for circular robots that can rotate in place; otherwise, return an
	 * empty smart pointer to indicate
	 * that the operation is not possible (this is what the default
	 * implementation does). */
	virtual mrpt::kinematics::CVehicleVelCmd::Ptr getAlignCmd(
		const double relative_heading_radians);

	/** Start the watchdog timer of the robot platform, if any, for maximum
	 * expected delay between consecutive calls to changeSpeeds().
	 * \param T_ms Period, in ms.
	 * \return false on any error. */
	virtual bool startWatchdog(float T_ms);

	/** Stop the watchdog timer.
	 * \return false on any error. \sa startWatchdog */
	virtual bool stopWatchdog();

	/** Return the current set of obstacle points, as seen from the local
	 * coordinate frame of the robot.
	 * \return false on any error.
	 * \param[out] obstacles  A representation of obstacles in robot-centric
	 * coordinates.
	 * \param[out] timestamp  The timestamp for the read obstacles. Use
	 * mrpt::system::now() unless you have something more accurate.
	 */
	virtual bool senseObstacles(
		mrpt::maps::CSimplePointsMap& obstacles,
		mrpt::system::TTimeStamp& timestamp) = 0;

	/** @name Navigation event callbacks
	 * @{ */
	/** Callback: Start of navigation command */
	virtual void sendNavigationStartEvent();

	/** Callback: End of navigation command (reach of single goal, or final
	 * waypoint of waypoint list) */
	virtual void sendNavigationEndEvent();

	/** Callback: Reached an intermediary waypoint in waypoint list navigation.
	 * reached_nSkipped will be `true` if the waypoint was physically reached;
	 * `false` if it was actually "skipped".
	 */
	virtual void sendWaypointReachedEvent(
		int waypoint_index, bool reached_nSkipped);

	/** Callback: Heading towards a new intermediary/final waypoint in waypoint
	 * list navigation */
	virtual void sendNewWaypointTargetEvent(int waypoint_index);

	/** Callback: Error asking sensory data from robot or sending motor
	 * commands. */
	virtual void sendNavigationEndDueToErrorEvent();

	/** Callback: No progression made towards target for a predefined period of
	 * time. */
	virtual void sendWaySeemsBlockedEvent();

	/** Callback: Apparent collision event (i.e. there is at least one obstacle
	 * point inside the robot shape) */
	virtual void sendApparentCollisionEvent();

	/** Callback: Target seems to be blocked by an obstacle. */
	virtual void sendCannotGetCloserToBlockedTargetEvent();

	/** @} */

	/** Returns the number of seconds ellapsed since the constructor of this
	 * class was invoked, or since
	 * the last call of resetNavigationTimer(). This will be normally
	 * wall-clock time, except in simulators where this method will return
	 * simulation time. */
	virtual double getNavigationTime();
	/** see getNavigationTime() */
	virtual void resetNavigationTimer();

   private:
	/** For getNavigationTime */
	mrpt::system::CTicTac m_navtime;
};
}  // namespace mrpt::nav
