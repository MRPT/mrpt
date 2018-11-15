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
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/typemeta/TEnumType.h>
#include <mrpt/config/CLoadableOptions.h>
#include <mrpt/poses/CPose2DInterpolator.h>
#include <mrpt/poses/FrameTransformer.h>
#include <mrpt/obs/obs_frwds.h>

#include <list>
#include <mutex>
#include <memory>  // unique_ptr
#include <functional>

namespace mrpt::nav
{
/** This is the base class for any reactive/planned navigation system. See
 * derived classes.
 *
 * How to use:
 *  - A class derived from `CRobot2NavInterface` with callbacks must be defined
 * by the user and provided to the constructor.
 *  - `navigationStep()` must be called periodically in order to effectively run
 * the navigation. This method will internally call the callbacks to gather
 * sensor data and robot positioning data.
 *
 * It implements the following state machine (see
 * CAbstractNavigator::getCurrentState() ), taking into account the extensions
 * described in CWaypointsNavigator
 *  \dot
 *  digraph CAbstractNavigator_States {
 *      IDLE; NAVIGATING; SUSPENDED; NAV_ERROR;
 *      IDLE -> NAVIGATING [ label="CAbstractNavigator::navigate()"];
 *      IDLE -> NAVIGATING [ label="CWaypointsNavigator::navigateWaypoints()" ];
 *      NAVIGATING -> IDLE [ label="Final target reached" ];
 *      NAVIGATING -> IDLE [ label="CAbstractNavigator::cancel()" ];
 *      NAVIGATING -> NAV_ERROR [ label="Upon sensor errors, timeout,..." ];
 *      NAVIGATING -> SUSPENDED [ label="CAbstractNavigator::suspend()" ];
 *      SUSPENDED -> NAVIGATING [ label="CAbstractNavigator::resume()" ];
 *      NAV_ERROR -> IDLE [ label="CAbstractNavigator::resetNavError()" ];
 *  }
 *  \enddot
 *
 * \sa CWaypointsNavigator, CReactiveNavigationSystem, CRobot2NavInterface, all
 * children classes
 *  \ingroup nav_reactive
 */
class CAbstractNavigator : public mrpt::system::COutputLogger
{
   public:
	/** ctor */
	CAbstractNavigator(CRobot2NavInterface& robot_interface_impl);
	/** dtor */
	~CAbstractNavigator() override;

	/** Individual target info in CAbstractNavigator::TNavigationParamsBase and
	 * derived classes */
	struct TargetInfo
	{
		/** Coordinates of desired target location. Heading may be ignored by
		 * some reactive implementations. */
		mrpt::math::TPose2D target_coords;
		/** (Default="map") Frame ID in which target is given. Optional, use
		 * only for submapping applications. */
		std::string target_frame_id;
		/** (Default=0.5 meters) Allowed distance to target in order to end the
		 * navigation. */
		float targetAllowedDistance{0.5};
		/** (Default=false) Whether the \a target coordinates are in global
		 * coordinates (false) or are relative to the current robot pose (true).
		 */
		bool targetIsRelative{false};
		/** (Default=.05) Desired relative speed (wrt maximum speed), in range
		 * [0,1], of the vehicle at target. Holonomic nav methods will perform
		 * "slow down" approaching target only if this is "==.0". Intermediary
		 * values will be honored only by the higher-level navigator, based on
		 * straight-line Euclidean distances. */
		double targetDesiredRelSpeed{.05};
		bool targetIsIntermediaryWaypoint{
			false};  // !< (Default=false) If true, event
		// callback
		// `sendWaypointReachedEvent()` will
		// be called instead of
		// `sendNavigationEndEvent()`

		TargetInfo();
		/** Gets navigation params as a human-readable format */
		std::string getAsText() const;
		bool operator==(const TargetInfo& o) const;
		bool operator!=(const TargetInfo& o) const { return !(*this == o); }
	};

	/** Base for all high-level navigation commands. See derived classes */
	struct TNavigationParamsBase
	{
		virtual ~TNavigationParamsBase() = default;
		/** Gets navigation params as a human-readable format */
		virtual std::string getAsText() const = 0;

	   protected:
		friend bool operator==(
			const TNavigationParamsBase&, const TNavigationParamsBase&);
		virtual bool isEqual(const TNavigationParamsBase& o) const = 0;
	};

	/** The struct for configuring navigation requests. Used in
	 * CAbstractPTGBasedReactive::navigate() */
	struct TNavigationParams : public TNavigationParamsBase
	{
		/** Navigation target */
		TargetInfo target;

		/** Gets navigation params as a human-readable format */
		std::string getAsText() const override;
		virtual std::unique_ptr<TNavigationParams> clone() const
		{
			return std::make_unique<TNavigationParams>(*this);
		}

	   protected:
		bool isEqual(const TNavigationParamsBase& o) const override;
	};

	/** \name Navigation control API
	 * @{ */

	/** Loads all params from a file. To be called before initialize().
	 * Each derived class *MUST* load its own parameters, and then call *ITS
	 * PARENT'S* overriden method to ensure all params are loaded. */
	virtual void loadConfigFile(const mrpt::config::CConfigFileBase& c);
	/** Saves all current options to a config file.
	 * Each derived class *MUST* save its own parameters, and then call *ITS
	 * PARENT'S* overriden method to ensure all params are saved. */
	virtual void saveConfigFile(mrpt::config::CConfigFileBase& c) const;

	/**  Must be called before any other navigation command */
	virtual void initialize() = 0;
	/** This method must be called periodically in order to effectively run the
	 * navigation */
	virtual void navigationStep();

	/** Navigation request to a single target location. It starts a new
	 * navigation.
	 * \param[in] params Pointer to structure with navigation info (its
	 * contents will be copied, so the original can be freely destroyed upon
	 * return if it was dynamically allocated.)
	 * \note A pointer is used so the passed object can be polymorphic with
	 * derived types.
	 */
	virtual void navigate(const TNavigationParams* params);

	/** Cancel current navegation. */
	virtual void cancel();
	/** Continues with suspended navigation. \sa suspend */
	virtual void resume();
	/** Suspend current navegation. \sa resume */
	virtual void suspend();
	/** Resets a `NAV_ERROR` state back to `IDLE` */
	virtual void resetNavError();

	/** The different states for the navigation system. */
	enum TState
	{
		IDLE = 0,
		NAVIGATING,
		SUSPENDED,
		NAV_ERROR
	};

	/** Returns the current navigator state. */
	inline TState getCurrentState() const { return m_navigationState; }
	/** Sets a user-provided frame transformer object; used only if providing
	 * targets in a frame ID
	 * different than the one in which robot odometry is given (both IDs
	 * default to `"map"`).
	 */
	void setFrameTF(
		const std::weak_ptr<mrpt::poses::FrameTransformer<2>>& frame_tf);

	/** Get the current frame tf object (defaults to nullptr) \sa setFrameTF */
	std::weak_ptr<mrpt::poses::FrameTransformer<2>> getFrameTF() const
	{
		return m_frame_tf;
	}

	/** By default, error exceptions on navigationStep() will dump an error
	 * message to the output logger interface. If rethrow is enabled
	 * (default=false), the error message will be reported as well, but
	 * exceptions will be re-thrown.
	 */
	void enableRethrowNavExceptions(const bool enable)
	{
		m_rethrow_exceptions = enable;
	}
	bool isRethrowNavExceptionsEnabled() const { return m_rethrow_exceptions; }
	/** @}*/

	struct TAbstractNavigatorParams : public mrpt::config::CLoadableOptions
	{
		/** Default value=0, means use the "targetAllowedDistance" passed by the
		 * user in the navigation request. */
		double dist_to_target_for_sending_event{0};
		/** navigator timeout (seconds) [Default=30 sec] */
		double alarm_seems_not_approaching_target_timeout{30};
		/** (Default value=0.6) When closer than this distance, check if the
		 * target is blocked to abort navigation with an error. */
		double dist_check_target_is_blocked{0.6};
		/** (Default=3) How many steps should the condition for
		 * dist_check_target_is_blocked be fulfilled to raise an event */
		int hysteresis_check_target_is_blocked{3};

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& c,
			const std::string& s) override;
		void saveToConfigFile(
			mrpt::config::CConfigFileBase& c,
			const std::string& s) const override;
		TAbstractNavigatorParams();
	};

	TAbstractNavigatorParams params_abstract_navigator;

	/** Gives access to a const-ref to the internal time logger used to estimate
	 * delays \sa getTimeLogger() in derived classes */
	const mrpt::system::CTimeLogger& getDelaysTimeLogger() const
	{
		return m_timlog_delays;
	}

   private:
	/** Last internal state of navigator: */
	TState m_lastNavigationState;
	/** Will be false until the navigation end is sent, and it is reset with
	 * each new command */
	bool m_navigationEndEventSent;
	int m_counter_check_target_is_blocked;
	bool m_rethrow_exceptions{false};

	/** Called before starting a new navigation. Internally, it calls to
	 * child-implemented onStartNewNavigation() */
	void internal_onStartNewNavigation();

   protected:
	/** Events generated during navigationStep(), enqueued to be called at the
	 * end of the method execution to avoid user code to change the navigator
	 * state. */
	std::list<std::function<void(void)>> m_pending_events;

	void dispatchPendingNavEvents();

	/** To be implemented in derived classes */
	virtual void performNavigationStep() = 0;

	/** Called whenever a new navigation has been started. Can be used to reset
	 * state variables, etc. */
	virtual void onStartNewNavigation() = 0;

	/** Called after each call to CAbstractNavigator::navigate() */
	virtual void onNavigateCommandReceived();

	/** Call to the robot getCurrentPoseAndSpeeds() and updates members
	 * m_curPoseVel accordingly.
	 * If an error is returned by the user callback, first, it calls
	 * robot.stop() ,then throws an std::runtime_error exception. */
	virtual void updateCurrentPoseAndSpeeds();

	/** Factorization of the part inside navigationStep(), for the case of state
	 * being NAVIGATING.
	 * Performs house-hold tasks like raising events in case of starting/ending
	 * navigation, timeout reaching destination, etc.
	 * `call_virtual_nav_method` can be set to false to avoid calling the
	 * virtual method performNavigationStep()
	 */
	virtual void performNavigationStepNavigating(
		bool call_virtual_nav_method = true);

	/** Does the job of navigate(), except the call to
	 * onNavigateCommandReceived() */
	virtual void processNavigateCommand(const TNavigationParams* params);

	/** Stops the robot and set navigation state to error */
	virtual void doEmergencyStop(const std::string& msg);

	/** Default: forward call to m_robot.changeSpeed(). Can be overriden. */
	virtual bool changeSpeeds(const mrpt::kinematics::CVehicleVelCmd& vel_cmd);
	/** Default: forward call to m_robot.changeSpeedsNOP(). Can be overriden. */
	virtual bool changeSpeedsNOP();
	/** Default: forward call to m_robot.stop(). Can be overriden. */
	virtual bool stop(bool isEmergencyStop);

	/** Default implementation: check if target_dist is below the accepted
	 * distance.
	 * If true is returned here, the end-of-navigation event will be sent out
	 * (only for non-intermediary targets).
	 */
	virtual bool checkHasReachedTarget(const double targetDist) const;

	/** Checks whether the robot shape, when placed at the given pose (relative
	 * to the current pose),
	 * is colliding with any of the latest known obstacles.
	 * Default implementation: always returns false. */
	virtual bool checkCollisionWithLatestObstacles(
		const mrpt::math::TPose2D& relative_robot_pose) const;

	/** Current internal state of navigator: */
	TState m_navigationState;
	/** Current navigation parameters */
	std::unique_ptr<TNavigationParams> m_navigationParams;

	/** The navigator-robot interface. */
	CRobot2NavInterface& m_robot;

	/** Optional, user-provided frame transformer. */
	std::weak_ptr<mrpt::poses::FrameTransformer<2>> m_frame_tf;

	/** mutex for all navigation methods */
	std::recursive_mutex m_nav_cs;

	struct TRobotPoseVel
	{
		mrpt::math::TPose2D pose;
		mrpt::math::TTwist2D velGlobal, velLocal;
		/** raw odometry (frame does not match to "pose", but is expected to be
		 * smoother in the short term). */
		mrpt::math::TPose2D rawOdometry;
		mrpt::system::TTimeStamp timestamp;
		/** map frame ID for `pose` */
		std::string pose_frame_id;
		TRobotPoseVel();
	};

	/** Current robot pose (updated in CAbstractNavigator::navigationStep() ) */
	TRobotPoseVel m_curPoseVel;
	double m_last_curPoseVelUpdate_robot_time;
	std::string m_last_curPoseVelUpdate_pose_frame_id;
	/** Latest robot poses (updated in CAbstractNavigator::navigationStep() ) */
	mrpt::poses::CPose2DInterpolator m_latestPoses, m_latestOdomPoses;

	/** Time logger to collect delay-related stats */
	mrpt::system::CTimeLogger m_timlog_delays;

	/** For sending an alarm (error event) when it seems that we are not
	 * approaching toward the target in a while... */
	double m_badNavAlarm_minDistTarget;
	mrpt::system::TTimeStamp m_badNavAlarm_lastMinDistTime;

   public:
	MRPT_MAKE_ALIGNED_OPERATOR_NEW
};

bool operator==(
	const CAbstractNavigator::TNavigationParamsBase&,
	const CAbstractNavigator::TNavigationParamsBase&);
}  // namespace mrpt::nav
MRPT_ENUM_TYPE_BEGIN(mrpt::nav::CAbstractNavigator::TState)
using namespace mrpt::nav;
MRPT_FILL_ENUM_MEMBER(CAbstractNavigator, IDLE);
MRPT_FILL_ENUM_MEMBER(CAbstractNavigator, NAVIGATING);
MRPT_FILL_ENUM_MEMBER(CAbstractNavigator, SUSPENDED);
MRPT_FILL_ENUM_MEMBER(CAbstractNavigator, NAV_ERROR);
MRPT_ENUM_TYPE_END()
