/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "nav-precomp.h"  // Precomp header
//
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/math/TSegment2D.h>
#include <mrpt/nav/reactive/CAbstractNavigator.h>

#include <limits>
#include <typeinfo>

using namespace mrpt::nav;
using namespace std;

const double PREVIOUS_POSES_MAX_AGE = 20;  // seconds

// Ctor: CAbstractNavigator::TargetInfo
CAbstractNavigator::TargetInfo::TargetInfo()
	: target_coords(0, 0, 0), target_frame_id("map")

{
}

// Gets navigation params as a human-readable format:
std::string CAbstractNavigator::TargetInfo::getAsText() const
{
	string s;
	s += mrpt::format(
		"target_coords = (%.03f,%.03f,%.03f deg)\n", target_coords.x,
		target_coords.y, target_coords.phi);
	s += mrpt::format("target_frame_id = \"%s\"\n", target_frame_id.c_str());
	s += mrpt::format("targetAllowedDistance = %.03f\n", targetAllowedDistance);
	s += mrpt::format(
		"targetIsRelative = %s\n", targetIsRelative ? "YES" : "NO");
	s += mrpt::format(
		"targetIsIntermediaryWaypoint = %s\n",
		targetIsIntermediaryWaypoint ? "YES" : "NO");
	s += mrpt::format("targetDesiredRelSpeed = %.02f\n", targetDesiredRelSpeed);
	return s;
}

bool mrpt::nav::CAbstractNavigator::TargetInfo::operator==(
	const TargetInfo& o) const
{
	return target_coords == o.target_coords &&
		target_frame_id == o.target_frame_id &&
		targetAllowedDistance == o.targetAllowedDistance &&
		targetIsRelative == o.targetIsRelative &&
		targetDesiredRelSpeed == o.targetDesiredRelSpeed &&
		targetIsIntermediaryWaypoint == o.targetIsIntermediaryWaypoint;
}

// Gets navigation params as a human-readable format:
std::string CAbstractNavigator::TNavigationParams::getAsText() const
{
	string s;
	s += "navparams. Single target:\n";
	s += target.getAsText();
	return s;
}

bool CAbstractNavigator::TNavigationParams::isEqual(
	const CAbstractNavigator::TNavigationParamsBase& o) const
{
	auto* rhs = dynamic_cast<const CAbstractNavigator::TNavigationParams*>(&o);
	return (rhs != nullptr) && (this->target == rhs->target);
}

CAbstractNavigator::TRobotPoseVel::TRobotPoseVel()
	: pose(0, 0, 0),
	  velGlobal(0, 0, 0),
	  velLocal(0, 0, 0),
	  rawOdometry(0, 0, 0),
	  timestamp(INVALID_TIMESTAMP),
	  pose_frame_id()
{
}

/*---------------------------------------------------------------
							Constructor
  ---------------------------------------------------------------*/
CAbstractNavigator::CAbstractNavigator(CRobot2NavInterface& react_iterf_impl)
	: mrpt::system::COutputLogger("MRPT_navigator"), m_robot(react_iterf_impl)
{
	m_latestPoses.setInterpolationMethod(mrpt::poses::imLinear2Neig);
	m_latestOdomPoses.setInterpolationMethod(mrpt::poses::imLinear2Neig);
	this->setVerbosityLevel(mrpt::system::LVL_DEBUG);
}

// Dtor:
CAbstractNavigator::~CAbstractNavigator() = default;
/** \callergraph */
void CAbstractNavigator::cancel()
{
	auto lck = mrpt::lockHelper(m_nav_cs);

	MRPT_LOG_DEBUG("CAbstractNavigator::cancel() called.");
	m_navigationState = IDLE;
	this->stop(false /*not emergency*/);
}

/** \callergraph */
void CAbstractNavigator::resume()
{
	auto lck = mrpt::lockHelper(m_nav_cs);

	MRPT_LOG_DEBUG("[CAbstractNavigator::resume() called.");
	if (m_navigationState == SUSPENDED) m_navigationState = NAVIGATING;
}

/** \callergraph */
void CAbstractNavigator::suspend()
{
	auto lck = mrpt::lockHelper(m_nav_cs);

	// Issue an "stop" if we are moving:
	// Update: do it *always*, even if the current velocity is zero, since
	// we may be in the middle of a multi-part motion command. It's safer.
	this->stop(false /*not an emergency stop*/);

	MRPT_LOG_DEBUG("CAbstractNavigator::suspend() called.");
	if (m_navigationState == NAVIGATING) m_navigationState = SUSPENDED;
}

/** \callergraph */
void CAbstractNavigator::resetNavError()
{
	auto lck = mrpt::lockHelper(m_nav_cs);

	MRPT_LOG_DEBUG("CAbstractNavigator::resetNavError() called.");
	if (m_navigationState == NAV_ERROR)
	{
		m_navigationState = IDLE;
		m_navErrorReason = TErrorReason();
	}
}

void CAbstractNavigator::setFrameTF(
	const std::weak_ptr<mrpt::poses::FrameTransformer<2>>& frame_tf)
{
	m_frame_tf = frame_tf;
}

void CAbstractNavigator::loadConfigFile(const mrpt::config::CConfigFileBase& c)
{
	MRPT_START

	params_abstract_navigator.loadFromConfigFile(c, "CAbstractNavigator");

	m_navProfiler.enable(c.read_bool(
		"CAbstractNavigator", "enable_time_profiler",
		m_navProfiler.isEnabled()));

	// At this point, all derived classes have already loaded their parameters.
	// Dump them to debug output:
	{
		mrpt::config::CConfigFileMemory cfg_mem;
		this->saveConfigFile(cfg_mem);
		MRPT_LOG_INFO(cfg_mem.getContent());
	}

	MRPT_END
}

void CAbstractNavigator::saveConfigFile(mrpt::config::CConfigFileBase& c) const
{
	params_abstract_navigator.saveToConfigFile(c, "CAbstractNavigator");
}

/** \callergraph */
void CAbstractNavigator::navigationStep()
{
	auto lck = mrpt::lockHelper(m_nav_cs);

	mrpt::system::CTimeLoggerEntry tle(
		m_navProfiler, "CAbstractNavigator::navigationStep()");

	const TState prevState = m_navigationState;
	switch (m_navigationState)
	{
		case IDLE:
		case SUSPENDED:
			try
			{
				// If we just arrived at this state, stop robot:
				if (m_lastNavigationState == NAVIGATING)
				{
					MRPT_LOG_INFO(
						"[CAbstractNavigator::navigationStep()] Navigation "
						"stopped.");
					// this->stop();  stop() is called by the method switching
					// the "state", so we have more flexibility
					m_robot.stopWatchdog();
				}
			}
			catch (...)
			{
			}
			break;

		case NAV_ERROR:
			try
			{
				// Send end-of-navigation event:
				if (m_lastNavigationState == NAVIGATING &&
					m_navigationState == NAV_ERROR)
				{
					m_pending_events.emplace_back(std::bind(
						&CRobot2NavInterface::sendNavigationEndDueToErrorEvent,
						std::ref(m_robot)));
				}

				// If we just arrived at this state, stop the robot:
				if (m_lastNavigationState == NAVIGATING)
				{
					MRPT_LOG_ERROR(
						"[CAbstractNavigator::navigationStep()] Stopping "
						"Navigation due to a NAV_ERROR state!");
					this->stop(false /*not emergency*/);
					m_robot.stopWatchdog();
				}
			}
			catch (...)
			{
			}
			break;

		case NAVIGATING:
			this->performNavigationStepNavigating(
				true /* do call virtual method nav implementation*/);
			break;	// End case NAVIGATING
	};
	m_lastNavigationState = prevState;

	dispatchPendingNavEvents();
}

/** \callergraph */
void CAbstractNavigator::dispatchPendingNavEvents()
{
	// Invoke pending events:
	for (auto& ev : m_pending_events)
	{
		ev();
	}
	m_pending_events.clear();
}

/** \callergraph */
void CAbstractNavigator::doEmergencyStop(const std::string& msg)
{
	try
	{
		this->stop(true /* emergency*/);
	}
	catch (...)
	{
	}
	m_navigationState = NAV_ERROR;
	// don't overwrite an error msg from a caller:
	if (m_navErrorReason.error_code == ERR_NONE)
	{
		m_navErrorReason.error_code = ERR_EMERGENCY_STOP;
		m_navErrorReason.error_msg =
			std::string("doEmergencyStop called for: ") + msg;
	}
	MRPT_LOG_ERROR(msg);
}

void CAbstractNavigator::onNavigateCommandReceived()
{
	auto lck = mrpt::lockHelper(m_nav_cs);

	m_navigationEndEventSent = false;
	m_navigationParams.reset();
}

void CAbstractNavigator::processNavigateCommand(const TNavigationParams* params)
{
	MRPT_START
	auto lck = mrpt::lockHelper(m_nav_cs);

	ASSERT_(params != nullptr);
	ASSERT_(
		params->target.targetDesiredRelSpeed >= .0 &&
		params->target.targetDesiredRelSpeed <= 1.0);

	// Copy data:
	m_navigationParams = params->clone();

	// Transform: relative -> absolute, if needed.
	if (m_navigationParams->target.targetIsRelative)
	{
		this->updateCurrentPoseAndSpeeds();
		m_navigationParams->target.target_coords =
			m_curPoseVel.pose + m_navigationParams->target.target_coords;
		m_navigationParams->target.targetIsRelative =
			false;	// Now it's not relative
	}

	// new state:
	m_navigationState = NAVIGATING;
	m_navErrorReason = TErrorReason();

	// Reset the bad navigation alarm:
	m_badNavAlarm_minDistTarget = std::numeric_limits<double>::max();
	m_badNavAlarm_lastMinDistTime = mrpt::Clock::now();

	MRPT_END
}

void CAbstractNavigator::navigate(
	const CAbstractNavigator::TNavigationParams* params)
{
	MRPT_START
	this->onNavigateCommandReceived();
	this->processNavigateCommand(params);
	MRPT_END
}

void CAbstractNavigator::updateCurrentPoseAndSpeeds()
{
	// Ignore calls too-close in time, e.g. from the navigationStep() methods of
	// AbstractNavigator and a derived, overriding class.
	const double robot_time_secs =
		m_robot.getNavigationTime();  // this is clockwall time for real robots,
	// simulated time in simulators.

	const double MIN_TIME_BETWEEN_POSE_UPDATES = 20e-3;
	if (m_last_curPoseVelUpdate_robot_time >= .0)
	{
		const double last_call_age =
			robot_time_secs - m_last_curPoseVelUpdate_robot_time;
		if (last_call_age < MIN_TIME_BETWEEN_POSE_UPDATES)
		{
			MRPT_LOG_THROTTLE_DEBUG_FMT(
				5.0,
				"updateCurrentPoseAndSpeeds: ignoring call, since last call "
				"was only %f ms ago.",
				last_call_age * 1e3);
			return;	 // previous data is still valid: don't query the robot
			// again
		}
	}

	{
		mrpt::system::CTimeLoggerEntry tle(
			m_navProfiler, "getCurrentPoseAndSpeeds()");
		m_curPoseVel.pose_frame_id = std::string("map");  // default
		if (!m_robot.getCurrentPoseAndSpeeds(
				m_curPoseVel.pose, m_curPoseVel.velGlobal,
				m_curPoseVel.timestamp, m_curPoseVel.rawOdometry,
				m_curPoseVel.pose_frame_id))
		{
			m_navigationState = NAV_ERROR;
			m_navErrorReason.error_code = ERR_EMERGENCY_STOP;
			m_navErrorReason.error_msg = std::string(
				"ERROR calling m_robot.getCurrentPoseAndSpeeds, stopping robot "
				"and finishing navigation");
			try
			{
				this->stop(true /*emergency*/);
			}
			catch (...)
			{
			}
			MRPT_LOG_ERROR(
				"ERROR calling m_robot.getCurrentPoseAndSpeeds, stopping robot "
				"and finishing navigation");
			throw std::runtime_error(
				"ERROR calling m_robot.getCurrentPoseAndSpeeds, stopping robot "
				"and finishing navigation");
		}
	}
	m_curPoseVel.velLocal = m_curPoseVel.velGlobal;
	m_curPoseVel.velLocal.rotate(-m_curPoseVel.pose.phi);

	m_last_curPoseVelUpdate_robot_time = robot_time_secs;
	const bool changed_frame_id =
		(m_last_curPoseVelUpdate_pose_frame_id != m_curPoseVel.pose_frame_id);
	m_last_curPoseVelUpdate_pose_frame_id = m_curPoseVel.pose_frame_id;

	if (changed_frame_id)
	{
		// If frame changed, clear past poses. This could be improved by
		// requesting
		// the transf between the two frames, but it's probably not worth.
		m_latestPoses.clear();
		m_latestOdomPoses.clear();
	}

	// Append to list of past poses:
	m_latestPoses.insert(m_curPoseVel.timestamp, m_curPoseVel.pose);
	m_latestOdomPoses.insert(m_curPoseVel.timestamp, m_curPoseVel.rawOdometry);

	// Purge old ones:
	while (m_latestPoses.size() > 1 &&
		   mrpt::system::timeDifference(
			   m_latestPoses.begin()->first, m_latestPoses.rbegin()->first) >
			   PREVIOUS_POSES_MAX_AGE)
	{
		m_latestPoses.erase(m_latestPoses.begin());
	}
	while (m_latestOdomPoses.size() > 1 &&
		   mrpt::system::timeDifference(
			   m_latestOdomPoses.begin()->first,
			   m_latestOdomPoses.rbegin()->first) > PREVIOUS_POSES_MAX_AGE)
	{
		m_latestOdomPoses.erase(m_latestOdomPoses.begin());
	}
}

/** \callergraph */
bool CAbstractNavigator::changeSpeeds(
	const mrpt::kinematics::CVehicleVelCmd& vel_cmd)
{
	return m_robot.changeSpeeds(vel_cmd);
}
/** \callergraph */
bool CAbstractNavigator::changeSpeedsNOP() { return m_robot.changeSpeedsNOP(); }
/** \callergraph */
bool CAbstractNavigator::stop(bool isEmergencyStop)
{
	return m_robot.stop(isEmergencyStop);
}

CAbstractNavigator::TAbstractNavigatorParams::TAbstractNavigatorParams()

	= default;
void CAbstractNavigator::TAbstractNavigatorParams::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& c, const std::string& s)
{
	MRPT_LOAD_CONFIG_VAR_CS(dist_to_target_for_sending_event, double);
	MRPT_LOAD_CONFIG_VAR_CS(alarm_seems_not_approaching_target_timeout, double);
	MRPT_LOAD_CONFIG_VAR_CS(dist_check_target_is_blocked, double);
	MRPT_LOAD_CONFIG_VAR_CS(hysteresis_check_target_is_blocked, int);
}
void CAbstractNavigator::TAbstractNavigatorParams::saveToConfigFile(
	mrpt::config::CConfigFileBase& c, const std::string& s) const
{
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		dist_to_target_for_sending_event,
		"Default value=0, means use the `targetAllowedDistance` passed by the "
		"user in the navigation request.");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		alarm_seems_not_approaching_target_timeout,
		"navigator timeout (seconds) [Default=30 sec]");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		dist_check_target_is_blocked,
		"When closer than this distance, check if the target is blocked to "
		"abort navigation with an error. [Default=0.6 m]");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		dist_to_target_for_sending_event,
		"Default value=0, means use the `targetAllowedDistance` passed by the"
		" user in the navigation request.");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		alarm_seems_not_approaching_target_timeout,
		"navigator timeout (seconds) [Default=30 sec]");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		dist_check_target_is_blocked,
		"When closer than this distance, check if the target is blocked to "
		"abort navigation with an error. [Default=0.6 m]");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		hysteresis_check_target_is_blocked,
		"How many steps should the condition for dist_check_target_is_blocked "
		"be fulfilled to raise an event");
}

bool mrpt::nav::operator==(
	const CAbstractNavigator::TNavigationParamsBase& a,
	const CAbstractNavigator::TNavigationParamsBase& b)
{
	return typeid(a) == typeid(b) && a.isEqual(b);
}

/** \callergraph */
bool CAbstractNavigator::checkHasReachedTarget(const double targetDist) const
{
	return (targetDist < m_navigationParams->target.targetAllowedDistance);
}

/** \callergraph */
void CAbstractNavigator::internal_onStartNewNavigation()
{
	m_robot.startWatchdog(1000);  // Watchdog = 1 seg
	m_latestPoses.clear();	// Clear cache of last poses.
	m_latestOdomPoses.clear();
	onStartNewNavigation();
}

/** \callergraph */
void CAbstractNavigator::performNavigationStepNavigating(
	bool call_virtual_nav_method)
{
	const TState prevState = m_navigationState;
	try
	{
		if (m_lastNavigationState != NAVIGATING)
		{
			MRPT_LOG_INFO(
				"[CAbstractNavigator::navigationStep()] Starting Navigation. "
				"Watchdog initiated...\n");
			if (m_navigationParams)
				MRPT_LOG_DEBUG(mrpt::format(
					"[CAbstractNavigator::navigationStep()] Navigation "
					"Params:\n%s\n",
					m_navigationParams->getAsText().c_str()));

			internal_onStartNewNavigation();
		}

		// Have we just started the navigation?
		if (m_lastNavigationState == IDLE)
		{
			m_pending_events.emplace_back(std::bind(
				&CRobot2NavInterface::sendNavigationStartEvent,
				std::ref(m_robot)));
		}

		/* ----------------------------------------------------------------
		Get current robot dyn state:
		---------------------------------------------------------------- */
		updateCurrentPoseAndSpeeds();

		/* ----------------------------------------------------------------
		Have we reached the target location?
		---------------------------------------------------------------- */
		// Build a 2D segment from the current robot pose to the previous one:
		ASSERT_(!m_latestPoses.empty());
		const mrpt::math::TSegment2D seg_robot_mov = mrpt::math::TSegment2D(
			mrpt::math::TPoint2D(m_curPoseVel.pose),
			m_latestPoses.size() > 1
				? mrpt::math::TPoint2D((++m_latestPoses.rbegin())->second)
				: mrpt::math::TPoint2D((m_latestPoses.rbegin())->second));

		if (m_navigationParams)
		{
			const double targetDist = seg_robot_mov.distance(
				mrpt::math::TPoint2D(m_navigationParams->target.target_coords));

			// Should "End of navigation" event be sent??
			if (!m_navigationParams->target.targetIsIntermediaryWaypoint &&
				!m_navigationEndEventSent &&
				targetDist <
					params_abstract_navigator.dist_to_target_for_sending_event)
			{
				m_navigationEndEventSent = true;
				m_pending_events.emplace_back(std::bind(
					&CRobot2NavInterface::sendNavigationEndEvent,
					std::ref(m_robot)));
			}

			// Have we really reached the target?
			if (checkHasReachedTarget(targetDist))
			{
				m_navigationState = IDLE;
				logFmt(
					mrpt::system::LVL_WARN,
					"Navigation target (%.03f,%.03f) was reached\n",
					m_navigationParams->target.target_coords.x,
					m_navigationParams->target.target_coords.y);

				if (!m_navigationParams->target.targetIsIntermediaryWaypoint)
				{
					this->stop(false /*not emergency*/);
					if (!m_navigationEndEventSent)
					{
						m_navigationEndEventSent = true;
						m_pending_events.emplace_back(std::bind(
							&CRobot2NavInterface::sendNavigationEndEvent,
							std::ref(m_robot)));
					}
				}
				return;
			}

			// Check the "no approaching the target"-alarm:
			// -----------------------------------------------------------
			if (targetDist < m_badNavAlarm_minDistTarget)
			{
				m_badNavAlarm_minDistTarget = targetDist;
				m_badNavAlarm_lastMinDistTime = mrpt::Clock::now();
			}
			else
			{
				// Too much time have passed?
				if (mrpt::system::timeDifference(
						m_badNavAlarm_lastMinDistTime, mrpt::Clock::now()) >
					params_abstract_navigator
						.alarm_seems_not_approaching_target_timeout)
				{
					MRPT_LOG_WARN(
						"Timeout approaching the target. Aborting navigation.");

					m_navigationState = NAV_ERROR;
					m_navErrorReason.error_code = ERR_CANNOT_REACH_TARGET;
					m_navErrorReason.error_msg = std::string(
						"Timeout approaching the target. Aborting navigation.");

					m_pending_events.emplace_back(std::bind(
						&CRobot2NavInterface::sendWaySeemsBlockedEvent,
						std::ref(m_robot)));
					return;
				}
			}

			// Check if the target seems to be at reach, but it's clearly
			// occupied by obstacles:
			if (targetDist <
				params_abstract_navigator.dist_check_target_is_blocked)
			{
				const auto rel_trg = m_navigationParams->target.target_coords -
					m_curPoseVel.pose;
				const bool is_col = checkCollisionWithLatestObstacles(rel_trg);
				if (is_col)
				{
					const bool send_event =
						(++m_counter_check_target_is_blocked >=
						 params_abstract_navigator
							 .hysteresis_check_target_is_blocked);

					if (send_event)
					{
						MRPT_LOG_THROTTLE_WARN(
							5.0,
							"Target seems to be blocked by obstacles. Invoking"
							" sendCannotGetCloserToBlockedTargetEvent().");

						m_pending_events.emplace_back(std::bind(
							&CRobot2NavInterface::
								sendCannotGetCloserToBlockedTargetEvent,
							std::ref(m_robot)));

						m_counter_check_target_is_blocked = 0;
					}
				}
				else
				{
					m_counter_check_target_is_blocked = 0;
				}
			}
		}

		// The normal execution of the navigation: Execute one step:
		if (call_virtual_nav_method) { performNavigationStep(); }
	}
	catch (const std::exception& e)
	{
		m_navigationState = NAV_ERROR;
		if (m_navErrorReason.error_code == ERR_NONE)
		{
			m_navErrorReason.error_code = ERR_OTHER;
			m_navErrorReason.error_msg =
				std::string("Exception: ") + std::string(e.what());
		}

		MRPT_LOG_ERROR_FMT(
			"[CAbstractNavigator::navigationStep] Exception:\n %s", e.what());
		if (m_rethrow_exceptions) throw;
	}
	catch (...)
	{
		m_navigationState = NAV_ERROR;
		if (m_navErrorReason.error_code == ERR_NONE)
		{
			m_navErrorReason.error_code = ERR_OTHER;
			m_navErrorReason.error_msg = "Untyped exception";
		}

		MRPT_LOG_ERROR(
			"[CAbstractNavigator::navigationStep] Untyped exception!");
		if (m_rethrow_exceptions) throw;
	}
	m_navigationState = prevState;
}

bool CAbstractNavigator::checkCollisionWithLatestObstacles(
	const mrpt::math::TPose2D& relative_robot_pose) const
{
	// Default impl:
	return false;
}
