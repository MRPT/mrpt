/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/nav/link_pragmas.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CTicTac.h>

#include <vector>

namespace mrpt
{
  namespace nav
  {
	/** The pure virtual interface between a real or simulated robot and any `CAbstractNavigator`-derived class.
	  *
	  *  The user must define a new class derived from `CRobot2NavInterface` and reimplement
	  *   all pure virtual and the desired virtual methods according to the documentation in this class.
	  * 
	  * [New in MRPT 1.5.0] This class does not make assumptions about the kinematic model of the robot, so it can work with either 
	  *  Ackermann, differential-driven or holonomic robots. It will depend on the used PTGs, so checkout 
	  *  each PTG documentation for the lenght and meaning of velocity commands.
	  *
	  * Users may prefer to inherit from one of these classes, which already provide implementations for the kinematic-specific methods:
	  *  - CReactiveInterfaceImplementation_DiffDriven 
	  *  - CReactiveInterfaceImplementation_Holo
	  *
	  * \sa CReactiveNavigationSystem, CAbstractNavigator
	  *  \ingroup nav_reactive
	  */
	class NAV_IMPEXP CRobot2NavInterface
	{
	public:
		CRobot2NavInterface() {}
		virtual ~CRobot2NavInterface() {}

		/** Get the current pose and speeds of the robot. The implementation should not take too much time to return,
		*   so if it might take more than ~10ms to ask the robot for the instantaneous data, it may be good enough to
		*   return the latest values from a cache which is updated in a parallel thread.
		*
		* \param[out] curPose The latest robot pose, in world coordinates. (x,y: meters, phi: radians)
		* \param[out] curVel  The latest robot velocity vector, in world coordinates. (vx,vy: m/s, omega: rad/s)
		* \return false on any error retrieving these values from the robot.
		*/
		virtual bool getCurrentPoseAndSpeeds(mrpt::math::TPose2D &curPose, mrpt::math::TTwist2D &curVel) = 0;

		/** Sends a velocity command to the robot.
		 * Length and meaning of this vector is PTG-dependent.
		 * See children classes of mrpt::nav::CParameterizedTrajectoryGenerator, but the most common cases will be:
		 * - Differential/Ackerman vehicle: `vel_cmd=[v w]`. See mrpt::kinematics::CVehicleSimul_DiffDriven::movementCommand() for more details.
		 * - Holonomic vehicle: `vel_cmd=[vel dir_local ramp_time rot_speed]`. See mrpt::kinematics::CVehicleSimul_Holo::sendVelCmd() for more details.
		 * \return false on any error.
		 */
		virtual bool changeSpeeds(const std::vector<double> &vel_cmd) = 0;

		/** Stop the robot right now.
		 * \return false on any error.
		 */
		virtual bool stop() = 0;

		/** Start the watchdog timer of the robot platform, if any.
		 * \param T_ms Period, in ms.
		 * \return false on any error.
		 */
		virtual bool startWatchdog(float T_ms) {
			MRPT_UNUSED_PARAM(T_ms);
			return true;
		}

		/** Stop the watchdog timer.
		 * \return false on any error.
		 */
		virtual bool stopWatchdog() { return true; }

		/** Return the current set of obstacle points, as seen from the local coordinate frame of the robot.
		  * \return false on any error.
		  */
		virtual bool senseObstacles( mrpt::maps::CSimplePointsMap &obstacles ) = 0;

		/** Callback: Start of navigation command */
		virtual void sendNavigationStartEvent () { std::cout << "[sendNavigationStartEvent] Not implemented by the user." << std::endl; }
		
		/** Callback: End of navigation command (reach of single goal, or final waypoint of waypoint list) */
		virtual void sendNavigationEndEvent() {	std::cout << "[sendNavigationEndEvent] Not implemented by the user." << std::endl; }

		/** Callback: Reached an intermediary waypoint in waypoint list navigation */
		virtual void sendWaypointReachedEvent(int waypoint_index) { std::cout << "[sendWaypointReachedEvent] Not implemented by the user. Reached waypoint #" << waypoint_index << std::endl; }

		/** Callback: Heading towards a new intermediary/final waypoint in waypoint list navigation */
		virtual void sendNewWaypointTargetEvent(int waypoint_index) { std::cout << "[sendNewWaypointTargetEvent] Not implemented by the user. Navigating towards waypoint #" << waypoint_index << std::endl; }

		/** Callback: Error asking sensory data from robot or sending motor commands. */
		virtual void sendNavigationEndDueToErrorEvent() { std::cout << "[sendNavigationEndDueToErrorEvent] Not implemented by the user." << std::endl; }

		/** Callback: No progression made towards target for a predefined period of time. */
		virtual void sendWaySeemsBlockedEvent() { std::cout << "[sendWaySeemsBlockedEvent] Not implemented by the user." << std::endl; }

		/** Returns the number of seconds ellapsed since the constructor of this class was invoked, or since 
		  * the last call of resetNavigationTimer(). This will be normally wall-clock time, except in simulators where this method will return simulation time. */
		virtual double getNavigationTime() {
			return m_navtime.Tac();
		}
		/** see getNavigationTime() */
		virtual void resetNavigationTimer() {
			m_navtime.Tic();
		}

		/** @name Methods implemented in CReactiveInterfaceImplementation_DiffDriven / CReactiveInterfaceImplementation_Holo
		   @{ */

		/** Must return the number of elements in a cmd_vel. Read more in changeSpeeds() */
		virtual size_t getVelCmdLength() const = 0;

		/** Scale a velocity command (may be kinematic-dependent).
		  * \param[in,out] vel_cmd The raw motion command from the selected reactive method, with the same meaning than in changeSpeeds(). Upon return, 
		  *                        this should contain the resulting scaled-down velocity command. This will be subsequently filtered by cmdVel_limits().
		  * \param[in] vel_scale A scale within [0,1] reflecting how much should be the raw velocity command be lessen (e.g. for safety reasons,...).
		  * \param[out] out_vel_cmd 
		  * 
		  * Users can directly inherit from existing implementations instead of manually redefining this method:
		  *  - mrpt::nav::CReactiveInterfaceImplementation_DiffDriven
		  *  - mrpt::nav::CReactiveInterfaceImplementation_Holo
		  */
		virtual void cmdVel_scale(std::vector<double> &vel_cmd, double vel_scale) = 0;

		/** Should compute a blended version of `beta` (within [0,1]) of `vel_cmd` and `1-beta` of `prev_vel_cmd`, simultaneously 
		  * to honoring any user-side maximum velocities.
		  */
		virtual void cmdVel_limits(std::vector<double> &vel_cmd, const std::vector<double> &prev_vel_cmd, const double beta) = 0;

		/** Load any parameter required by a derived class. */
		virtual void loadConfigFile(const mrpt::utils::CConfigFileBase &cfg, const std::string &section) = 0;

		/** @} */
	private:
		mrpt::utils::CTicTac  m_navtime; //!< For getNavigationTime
	};

	/** Partial implementation of CRobot2NavInterface for differential-driven robots
	  *  \ingroup nav_reactive
	  */
	class NAV_IMPEXP CRobot2NavInterface_DiffDriven : public CRobot2NavInterface
	{
	public:
		// See base class docs.
		size_t getVelCmdLength() const MRPT_OVERRIDE { return 2;}

		/** See docs of method in base class. The implementation for differential-driven robots of this method 
		  * just multiplies all the components of vel_cmd times vel_scale, which is appropriate
		  *  for differential-driven kinematic models (v,w).
		  */
		void cmdVel_scale(std::vector<double> &vel_cmd, double vel_scale) MRPT_OVERRIDE
		{
			ASSERT_(vel_cmd.size()==2);
			vel_cmd[0] *= vel_scale;
			vel_cmd[1] *= vel_scale;
		}

		// See base class docs.
		void cmdVel_limits(std::vector<double> &vel_cmd, const std::vector<double> &prev_vel_cmd, const double beta)
		{
			ASSERT_(robotMax_V_mps>0);
			ASSERT_(robotMax_W_radps>0);
			double new_cmd_v=vel_cmd[0], new_cmd_w=vel_cmd[1];
			filter_max_vw(new_cmd_v, new_cmd_w);
			if (fabs(new_cmd_v) < 0.01) // i.e. new behavior is nearly a pure rotation
			{                        // thus, it's OK to blend the rotational component
				new_cmd_w = beta*new_cmd_w + (1 - beta)*prev_vel_cmd[1];
			}
			else                     // there is a non-zero translational component
			{
				// must maintain the ratio of w to v (while filtering v)
				float ratio = new_cmd_w / new_cmd_v;
				new_cmd_v = beta*new_cmd_v + (1 - beta)*prev_vel_cmd[0];   // blend new v value
				new_cmd_w = ratio * new_cmd_v;  // ensure new w implements expected path curvature

				filter_max_vw(new_cmd_v, new_cmd_w);

				vel_cmd[0] = new_cmd_v; vel_cmd[1] = new_cmd_w;
			}
		}

		/** This class recognizes these parameters: `robotMax_V_mps`, `robotMax_W_degps` */
		virtual void loadConfigFile(const mrpt::utils::CConfigFileBase &cfg, const std::string &section) MRPT_OVERRIDE 
		{
			MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(robotMax_V_mps,double,  cfg,section);
			MRPT_LOAD_HERE_CONFIG_VAR_NO_DEFAULT(robotMax_W_degps,double, robotMax_W_radps, cfg,section);
			robotMax_W_radps = mrpt::utils::DEG2RAD(robotMax_W_radps);
		}

		double  robotMax_V_mps;       //!< Max. linear speed (m/s)
		double  robotMax_W_radps;     //!< Max. angular speed (rad/s)

		CRobot2NavInterface_DiffDriven() : 
			robotMax_V_mps(-1.0),
			robotMax_W_radps(-1.0)
		{}

	private:
		void filter_max_vw(double &v, double &w)
		{
			// Ensure maximum speeds:
			if (fabs(v) > robotMax_V_mps)
			{
				// Scale:
				float F = fabs(robotMax_V_mps / v);
				v *= F;
				w *= F;
			}

			if (fabs(w) > robotMax_W_radps)
			{
				// Scale:
				float F = fabs(robotMax_W_radps / w);
				v *= F;
				w *= F;
			}
		}

	};

	/** Partial implementation of CRobot2NavInterface for holonomic robots
	  *  \ingroup nav_reactive
	  */
	class NAV_IMPEXP CRobot2NavInterface_Holo : public CRobot2NavInterface
	{
	public:
		// See base class docs.
		size_t getVelCmdLength() const MRPT_OVERRIDE { return 4;}

		/** See docs of method in base class. 
		  * For holonomic robots, `vel_cmd=[vel dir_local ramp_time rot_speed]`
		  */
		void cmdVel_scale(std::vector<double> &vel_cmd, double vel_scale) MRPT_OVERRIDE
		{
			vel_cmd[0] *= vel_scale; // |(vx,vy)|
			vel_cmd[3] *= vel_scale; // rot_speed
			// ramp_time: leave unchanged
		}

		// See base class docs.
		void cmdVel_limits(std::vector<double> &vel_cmd, const std::vector<double> &prev_vel_cmd, const double beta)
		{ // remember:  `vel_cmd=[vel dir_local ramp_time rot_speed]`
			ASSERTMSG_(robotMax_V_mps>=.0, "[CReactiveInterfaceImplementation_Holo] `robotMax_V_mps` must be set to valid values: either assign values programatically or call loadConfigFile()")
			double f=1.0;
			if (vel_cmd[0]>robotMax_V_mps) f = robotMax_V_mps/vel_cmd[0];

			vel_cmd[0] *= f; // |(vx,vy)|
			vel_cmd[3] *= f; // rot_speed
			// ramp_time: leave unchanged
			// Blending with "beta" not required, since the ramp_time already blends cmds for holo robots.
		}

		/** This class recognizes these parameters: `robotMax_V_mps` */
		virtual void loadConfigFile(const mrpt::utils::CConfigFileBase &cfg, const std::string &section) MRPT_OVERRIDE 
		{
			MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(robotMax_V_mps,double,  cfg,section);
		}

		double  robotMax_V_mps;       //!< Max. instantaneous linear speed (m/s)

		CRobot2NavInterface_Holo() :
			robotMax_V_mps(-1.0)
		{
		}
	};

  }
}

