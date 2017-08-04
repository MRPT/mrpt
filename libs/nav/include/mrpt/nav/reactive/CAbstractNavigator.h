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
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/TEnumType.h>
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/poses/CPose2DInterpolator.h>
#include <mrpt/poses/FrameTransformer.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/obs/obs_frwds.h>

#include <mrpt/nav/link_pragmas.h>

namespace mrpt
{
  namespace nav
  {
	/** This is the base class for any reactive/planned navigation system. See derived classes.
	 *
	 * How to use:
	 *  - A class derived from `CRobot2NavInterface` with callbacks must be defined by the user and provided to the constructor.
	 *  - `navigationStep()` must be called periodically in order to effectively run the navigation. This method will internally call the callbacks to gather sensor data and robot positioning data.
	 *
	 * It implements the following state machine (see CAbstractNavigator::getCurrentState() ), taking into account the extensions described in CWaypointsNavigator
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
	 * \sa CWaypointsNavigator, CReactiveNavigationSystem, CRobot2NavInterface, all children classes
	 *  \ingroup nav_reactive
	 */
	class NAV_IMPEXP CAbstractNavigator : public mrpt::utils::COutputLogger
	{
	public:
		CAbstractNavigator( CRobot2NavInterface &robot_interface_impl );  //!< ctor
		virtual ~CAbstractNavigator(); //!< dtor

		/** Individual target info in CAbstractNavigator::TNavigationParamsBase and derived classes */
		struct NAV_IMPEXP TargetInfo
		{
			mrpt::math::TPose2D target_coords;         //!< Coordinates of desired target location. Heading may be ignored by some reactive implementations.
			std::string         target_frame_id;       //!< (Default="map") Frame ID in which target is given. Optional, use only for submapping applications.
			float               targetAllowedDistance; //!< (Default=0.5 meters) Allowed distance to target in order to end the navigation.
			bool                targetIsRelative;      //!< (Default=false) Whether the \a target coordinates are in global coordinates (false) or are relative to the current robot pose (true).
			double              targetDesiredRelSpeed; //!< (Default=.05) Desired relative speed (wrt maximum speed), in range [0,1], of the vehicle at target. Holonomic nav methods will perform "slow down" approaching target only if this is "==.0". Intermediary values will be honored only by the higher-level navigator, based on straight-line Euclidean distances.
			bool                targetIsIntermediaryWaypoint; // !< (Default=false) If true, event callback `sendWaypointReachedEvent()` will be called instead of `sendNavigationEndEvent()`

			TargetInfo();
			std::string getAsText() const; //!< Gets navigation params as a human-readable format
			bool operator==(const TargetInfo&o) const;
			bool operator!=(const TargetInfo&o) const { return !(*this==o); }
		};

		/** Base for all high-level navigation commands. See derived classes */
		struct NAV_IMPEXP TNavigationParamsBase
		{
			virtual ~TNavigationParamsBase() {}
			virtual std::string getAsText() const =0; //!< Gets navigation params as a human-readable format
			virtual TNavigationParamsBase* clone() const = 0;
		protected:
			friend bool NAV_IMPEXP operator==(const TNavigationParamsBase&, const TNavigationParamsBase&);
			virtual bool isEqual(const TNavigationParamsBase& o) const =0;
		};

		/** The struct for configuring navigation requests. Used in CAbstractPTGBasedReactive::navigate() */
		struct NAV_IMPEXP TNavigationParams : public TNavigationParamsBase
		{
			TargetInfo  target; //!< Navigation target

			virtual std::string getAsText() const MRPT_OVERRIDE; //!< Gets navigation params as a human-readable format
			virtual TNavigationParamsBase* clone() const MRPT_OVERRIDE { return new TNavigationParams(*this); }
		protected:
			virtual bool isEqual(const TNavigationParamsBase& o) const MRPT_OVERRIDE;
		};

		/** \name Navigation control API
		  * @{ */

		/** Loads all params from a file. To be called before initialize(). 
		  * Each derived class *MUST* load its own parameters, and then call *ITS PARENT'S* overriden method to ensure all params are loaded. */
		virtual void loadConfigFile(const mrpt::utils::CConfigFileBase &c);
		/** Saves all current options to a config file.
		  * Each derived class *MUST* save its own parameters, and then call *ITS PARENT'S* overriden method to ensure all params are saved. */
		virtual void saveConfigFile(mrpt::utils::CConfigFileBase &c) const;

		virtual void initialize() = 0; //!<  Must be called before any other navigation command
		virtual void navigationStep(); //!< This method must be called periodically in order to effectively run the navigation

		/** Navigation request to a single target location. It starts a new navigation.
		  * \param[in] params Pointer to structure with navigation info (its contents will be copied, so the original can be freely destroyed upon return if it was dynamically allocated.)
		  * \note A pointer is used so the passed object can be polymorphic with derived types.
		  */
		virtual void navigate( const TNavigationParams *params );

		virtual void cancel(); //!< Cancel current navegation.
		virtual void resume(); //!< Continues with suspended navigation. \sa suspend
		virtual void suspend(); //!< Suspend current navegation. \sa resume
		virtual void resetNavError(); //!< Resets a `NAV_ERROR` state back to `IDLE`

		/** The different states for the navigation system. */
		enum TState {
			IDLE=0,
			NAVIGATING,
			SUSPENDED,
			NAV_ERROR
		};

		/** Returns the current navigator state. */
		inline TState getCurrentState() const { return m_navigationState; }

		/** Sets a user-provided frame transformer object; used only if providing targets in a frame ID 
		  * different than the one in which robot odometry is given (both IDs default to `"map"`). 
		  * Ownership of the pointee object remains belonging to the user, which is responsible of deleting it 
		  * and ensuring its a valid pointer during the lifetime of this navigator object.
		  * \todo [MRPT 2.0: Make this a weak_ptr]
		  */
		void setFrameTF(mrpt::poses::FrameTransformer<2> *frame_tf);

		/** Get the current frame tf object (defaults to nullptr) \sa setFrameTF */
		const mrpt::poses::FrameTransformer<2> * getFrameTF() const { return m_frame_tf; }

		/** @}*/

		struct NAV_IMPEXP TAbstractNavigatorParams : public mrpt::utils::CLoadableOptions
		{
			double dist_to_target_for_sending_event;  //!< Default value=0, means use the "targetAllowedDistance" passed by the user in the navigation request.
			double alarm_seems_not_approaching_target_timeout; //!< navigator timeout (seconds) [Default=30 sec]
			double dist_check_target_is_blocked;     //!< (Default value=0.6) When closer than this distance, check if the target is blocked to abort navigation with an error.
			int hysteresis_check_target_is_blocked;  // (Default=3) How many steps should the condition for dist_check_target_is_blocked be fulfilled to raise an event

			virtual void loadFromConfigFile(const mrpt::utils::CConfigFileBase &c, const std::string &s) MRPT_OVERRIDE;
			virtual void saveToConfigFile(mrpt::utils::CConfigFileBase &c, const std::string &s) const MRPT_OVERRIDE;
			TAbstractNavigatorParams();
		};

		TAbstractNavigatorParams params_abstract_navigator;

		/** Gives access to a const-ref to the internal time logger used to estimate delays \sa getTimeLogger() in derived classes */
		const mrpt::utils::CTimeLogger & getDelaysTimeLogger() const { return m_timlog_delays; }

	private:
		TState  m_lastNavigationState; //!< Last internal state of navigator:
		bool    m_navigationEndEventSent; //!< Will be false until the navigation end is sent, and it is reset with each new command
		int m_counter_check_target_is_blocked;

		/** Called before starting a new navigation. Internally, it calls to child-implemented onStartNewNavigation() */
		void internal_onStartNewNavigation();

	protected:
		struct NAV_IMPEXP TPendingEvent
		{
			typedef void (CRobot2NavInterface::*functor_event_void_t)();
			functor_event_void_t event_noargs; //!< event w/o arguments

			bool event_wp_reached;
			int event_wp_reached_index;
			bool event_wp_reached_reached;

			bool event_new_wp;
			int event_new_wp_index;

			bool event_cannot_get_closer_target;

			TPendingEvent();
		};
		/** Events generated during navigationStep(), enqueued to be called
		* at the end of the method execution to avoid user code to change
		* the navigator state. */
		std::vector<TPendingEvent> m_pending_events;

		void dispatchPendingNavEvents();

		/** To be implemented in derived classes */
		virtual void  performNavigationStep( )=0;

		/** Called whenever a new navigation has been started. Can be used to reset state variables, etc. */
		virtual void onStartNewNavigation() = 0;

		/** Call to the robot getCurrentPoseAndSpeeds() and updates members m_curPoseVel accordingly.
		  * If an error is returned by the user callback, first, it calls robot.stop() ,then throws an std::runtime_error exception. */
		void updateCurrentPoseAndSpeeds();

		/** Factorization of the part inside navigationStep(), for the case of state being NAVIGATING.
		  * Performs house-hold tasks like raising events in case of starting/ending navigation, timeout reaching destination, etc.
		  * `call_virtual_nav_method` can be set to false to avoid calling the virtual method performNavigationStep()
		  */
		virtual void performNavigationStepNavigating(bool call_virtual_nav_method = true);

		/** Stops the robot and set navigation state to error */
		void doEmergencyStop( const std::string &msg );

		virtual bool changeSpeeds(const mrpt::kinematics::CVehicleVelCmd &vel_cmd); //!< Default: forward call to m_robot.changeSpeed(). Can be overriden.
		virtual bool changeSpeedsNOP(); //!< Default: forward call to m_robot.changeSpeedsNOP(). Can be overriden.
		virtual bool stop(bool isEmergencyStop); //!< Default: forward call to m_robot.stop(). Can be overriden.

		/** Default implementation: check if target_dist is below the accepted distance. 
		  * If true is returned here, the end-of-navigation event will be sent out (only for non-intermediary targets).
		  */
		virtual bool checkHasReachedTarget(const double targetDist) const;

		/** Checks whether the robot shape, when placed at the given pose (relative to the current pose), 
		* is colliding with any of the latest known obstacles.
		* Default implementation: always returns false. */
		virtual bool checkCollisionWithLatestObstacles(const mrpt::math::TPose2D &relative_robot_pose) const;

		TState             m_navigationState;  //!< Current internal state of navigator:
		TNavigationParams  *m_navigationParams;  //!< Current navigation parameters

		CRobot2NavInterface   &m_robot; //!< The navigator-robot interface.
		
		/** Optional, user-provided frame transformer.
		  * Note: We dont have ownership of the pointee object! */
		mrpt::poses::FrameTransformer<2> *m_frame_tf;

		mrpt::synch::CCriticalSectionRecursive m_nav_cs; //!< mutex for all navigation methods

		struct NAV_IMPEXP TRobotPoseVel
		{
			mrpt::math::TPose2D  pose;
			mrpt::math::TTwist2D velGlobal, velLocal;
			mrpt::math::TPose2D  rawOdometry;  //!< raw odometry (frame does not match to "pose", but is expected to be smoother in the short term).
			mrpt::system::TTimeStamp timestamp;
			std::string pose_frame_id; //!< map frame ID for `pose`
			TRobotPoseVel();
		};

		TRobotPoseVel m_curPoseVel; //!< Current robot pose (updated in CAbstractNavigator::navigationStep() )
		double  m_last_curPoseVelUpdate_robot_time;
		std::string m_last_curPoseVelUpdate_pose_frame_id;
		mrpt::poses::CPose2DInterpolator m_latestPoses, m_latestOdomPoses; //!< Latest robot poses (updated in CAbstractNavigator::navigationStep() )

		mrpt::utils::CTimeLogger m_timlog_delays; //!< Time logger to collect delay-related stats

		/** For sending an alarm (error event) when it seems that we are not approaching toward the target in a while... */
		double                   m_badNavAlarm_minDistTarget;
		mrpt::system::TTimeStamp m_badNavAlarm_lastMinDistTime;

	public:
		MRPT_MAKE_ALIGNED_OPERATOR_NEW
	};

	bool NAV_IMPEXP operator==(const CAbstractNavigator::TNavigationParamsBase&, const CAbstractNavigator::TNavigationParamsBase&);

  }

	// Specializations MUST occur at the same namespace:
	namespace utils
	{
		template <>
		struct TEnumTypeFiller<mrpt::nav::CAbstractNavigator::TState>
		{
			typedef mrpt::nav::CAbstractNavigator::TState enum_t;
			static void fill(bimap<enum_t, std::string>  &m_map)
			{
				m_map.insert(mrpt::nav::CAbstractNavigator::IDLE, "IDLE");
				m_map.insert(mrpt::nav::CAbstractNavigator::NAVIGATING, "NAVIGATING");
				m_map.insert(mrpt::nav::CAbstractNavigator::SUSPENDED, "SUSPENDED");
				m_map.insert(mrpt::nav::CAbstractNavigator::NAV_ERROR, "NAV_ERROR");
			}
		};
	} // End of namespace
}
