/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/reactive/CAbstractNavigator.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <limits>

using namespace mrpt::nav;
using namespace std;

const double PREVIOUS_POSES_MAX_AGE = 20; // seconds

// Ctor: CAbstractNavigator::TNavigationParams 
CAbstractNavigator::TNavigationParams::TNavigationParams() :
	target(0,0,0), 
	targetAllowedDistance(0.5),
	targetIsRelative(false),
	targetIsIntermediaryWaypoint(false)
{
}

// Gets navigation params as a human-readable format:
std::string CAbstractNavigator::TNavigationParams::getAsText() const 
{
	string s;
	s+= mrpt::format("navparams.target = (%.03f,%.03f,%.03f deg)\n", target.x, target.y,target.phi );
	s+= mrpt::format("navparams.targetAllowedDistance = %.03f\n", targetAllowedDistance );
	s+= mrpt::format("navparams.targetIsRelative = %s\n", targetIsRelative ? "YES":"NO");
	s+= mrpt::format("navparams.targetIsIntermediaryWaypoint = %s\n", targetIsIntermediaryWaypoint ? "YES":"NO");

	return s;
}

CAbstractNavigator::TRobotPoseVel::TRobotPoseVel() :
	pose(0,0,0),
	velGlobal(0,0,0),
	velLocal(0,0,0),
	timestamp(INVALID_TIMESTAMP) 
{
}

/*---------------------------------------------------------------
							Constructor
  ---------------------------------------------------------------*/
CAbstractNavigator::CAbstractNavigator(CRobot2NavInterface &react_iterf_impl) :
	mrpt::utils::COutputLogger("MRPT_navigator"),
	m_lastNavigationState ( IDLE ),
	m_navigationEndEventSent(false),
	m_navigationState     ( IDLE ),
	m_navigationParams    ( NULL ),
	m_robot               ( react_iterf_impl ),
	m_curPoseVel          (),
	m_latestPoses         (),
	m_timlog_delays       (true, "CAbstractNavigator::m_timlog_delays"),
	m_badNavAlarm_AlarmTimeout(30.0),
	DIST_TO_TARGET_FOR_SENDING_EVENT(.0)
{
	m_latestPoses.setInterpolationMethod(mrpt::poses::CPose3DInterpolator::imLinear2Neig);
	this->setVerbosityLevel(mrpt::utils::LVL_DEBUG);
}

// Dtor:
CAbstractNavigator::~CAbstractNavigator()
{
	mrpt::utils::delete_safe( m_navigationParams );
}

/*---------------------------------------------------------------
							cancel
  ---------------------------------------------------------------*/
void CAbstractNavigator::cancel()
{
	mrpt::synch::CCriticalSectionLocker csl(&m_nav_cs);
	MRPT_LOG_DEBUG("CAbstractNavigator::cancel() called.");
	m_navigationState = IDLE;
	this->stop(false /*not emergency*/);
}


/*---------------------------------------------------------------
							resume
  ---------------------------------------------------------------*/
void CAbstractNavigator::resume()
{
	mrpt::synch::CCriticalSectionLocker csl(&m_nav_cs);

	MRPT_LOG_DEBUG("[CAbstractNavigator::resume() called.");
	if ( m_navigationState == SUSPENDED )
		m_navigationState = NAVIGATING;
}


/*---------------------------------------------------------------
							suspend
  ---------------------------------------------------------------*/
void  CAbstractNavigator::suspend()
{
	mrpt::synch::CCriticalSectionLocker csl(&m_nav_cs);

	MRPT_LOG_DEBUG("CAbstractNavigator::suspend() called.");
	if ( m_navigationState == NAVIGATING )
		m_navigationState  = SUSPENDED;
}

void CAbstractNavigator::resetNavError()
{
	mrpt::synch::CCriticalSectionLocker csl(&m_nav_cs);

	MRPT_LOG_DEBUG("CAbstractNavigator::resetNavError() called.");
	if ( m_navigationState == NAV_ERROR )
		m_navigationState  = IDLE;
}

/*---------------------------------------------------------------
					navigationStep
  ---------------------------------------------------------------*/
void CAbstractNavigator::navigationStep()
{
	mrpt::synch::CCriticalSectionLocker csl(&m_nav_cs);

	const TState prevState = m_navigationState;
	switch ( m_navigationState )
	{
	case IDLE:
	case SUSPENDED:
		try
		{
			// If we just arrived at this state, stop robot:
			if ( m_lastNavigationState == NAVIGATING )
			{
				MRPT_LOG_INFO("[CAbstractNavigator::navigationStep()] Navigation stopped.");
				// this->stop();  stop() is called by the method switching the "state", so we have more flexibility
				m_robot.stopWatchdog();
			}
		} catch (...) { }
		break;

	case NAV_ERROR:
		try
		{
			// Send end-of-navigation event:
			if ( m_lastNavigationState == NAVIGATING && m_navigationState == NAV_ERROR)
				m_robot.sendNavigationEndDueToErrorEvent();

			// If we just arrived at this state, stop the robot:
			if ( m_lastNavigationState == NAVIGATING )
			{
				MRPT_LOG_ERROR("[CAbstractNavigator::navigationStep()] Stoping Navigation due to a NAV_ERROR state!");
				this->stop(false /*not emergency*/);
				m_robot.stopWatchdog();
			}
		} catch (...) { }
		break;

	case NAVIGATING:
		try
		{
			if ( m_lastNavigationState != NAVIGATING )
			{
				MRPT_LOG_INFO("[CAbstractNavigator::navigationStep()] Starting Navigation. Watchdog initiated...\n");
				if (m_navigationParams)
					MRPT_LOG_DEBUG(mrpt::format("[CAbstractNavigator::navigationStep()] Navigation Params:\n%s\n", m_navigationParams->getAsText().c_str() ));

				m_robot.startWatchdog( 1000 );	// Watchdog = 1 seg
				m_latestPoses.clear(); // Clear cache of last poses.
				onStartNewNavigation();
			}

			// Have we just started the navigation?
			if ( m_lastNavigationState == IDLE )
				m_robot.sendNavigationStartEvent();

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
				m_latestPoses.size()>1 ?
					mrpt::math::TPoint2D( (++m_latestPoses.rbegin())->second )
					:
					mrpt::math::TPoint2D((m_latestPoses.rbegin())->second)
			);

			const double targetDist = seg_robot_mov.distance( mrpt::math::TPoint2D(m_navigationParams->target) );

			// Should "End of navigation" event be sent??
			if (!m_navigationParams->targetIsIntermediaryWaypoint && !m_navigationEndEventSent && targetDist < DIST_TO_TARGET_FOR_SENDING_EVENT)
			{
				m_navigationEndEventSent = true;
				m_robot.sendNavigationEndEvent();
			}

			// Have we really reached the target?
			if ( targetDist < m_navigationParams->targetAllowedDistance )
			{
				if (!m_navigationParams->targetIsIntermediaryWaypoint) {
					this->stop(false /*not emergency*/);
				}
				m_navigationState = IDLE;
				logFmt(mrpt::utils::LVL_WARN, "Navigation target (%.03f,%.03f) was reached\n", m_navigationParams->target.x,m_navigationParams->target.y);

				if (!m_navigationParams->targetIsIntermediaryWaypoint && !m_navigationEndEventSent)
				{
					m_navigationEndEventSent = true;
					m_robot.sendNavigationEndEvent();
				}
				break;
			}

			// Check the "no approaching the target"-alarm:
			// -----------------------------------------------------------
			if (targetDist < m_badNavAlarm_minDistTarget )
			{
				m_badNavAlarm_minDistTarget = targetDist;
				m_badNavAlarm_lastMinDistTime =  mrpt::system::getCurrentTime();
			}
			else
			{
				// Too much time have passed?
				if (mrpt::system::timeDifference( m_badNavAlarm_lastMinDistTime, mrpt::system::getCurrentTime() ) > m_badNavAlarm_AlarmTimeout)
				{
					MRPT_LOG_WARN("--------------------------------------------\nWARNING: Timeout for approaching toward the target expired!! Aborting navigation!! \n---------------------------------\n");
					m_navigationState = NAV_ERROR;
					m_robot.sendWaySeemsBlockedEvent();
					break;
				}
			}

			// ==== The normal execution of the navigation: Execute one step ==== 
			performNavigationStep();

		}
		catch (std::exception &e)
		{
			cerr << "[CAbstractNavigator::navigationStep] Exception:\n" << e.what() << endl;
		}
		catch (...)
		{
			cerr << "[CAbstractNavigator::navigationStep] Unexpected exception.\n";
		}
		break;	// End case NAVIGATING
	};
	m_lastNavigationState = prevState;
}

void CAbstractNavigator::doEmergencyStop( const std::string &msg )
{
	try {
		this->stop(true /* emergency*/);
	}
	catch (...) { }
	m_navigationState = NAV_ERROR;
	MRPT_LOG_ERROR(msg);
}


void CAbstractNavigator::navigate(const CAbstractNavigator::TNavigationParams *params )
{
	mrpt::synch::CCriticalSectionLocker csl(&m_nav_cs);

	m_navigationEndEventSent = false;

	// Copy data:
	mrpt::utils::delete_safe(m_navigationParams);
	m_navigationParams = params->clone();

	// Transform: relative -> absolute, if needed.
	if ( m_navigationParams->targetIsRelative )
	{
		this->updateCurrentPoseAndSpeeds(false /*update_seq_latest_poses*/);

		const mrpt::poses::CPose2D relTarget(m_navigationParams->target);
		mrpt::poses::CPose2D absTarget;
		absTarget.composeFrom(m_curPoseVel.pose, relTarget);

		m_navigationParams->target = mrpt::math::TPose2D(absTarget);

		m_navigationParams->targetIsRelative = false; // Now it's not relative
	}

	// new state:
	m_navigationState = NAVIGATING;

	// Reset the bad navigation alarm:
	m_badNavAlarm_minDistTarget = std::numeric_limits<double>::max();
	m_badNavAlarm_lastMinDistTime = mrpt::system::getCurrentTime();
}

void CAbstractNavigator::updateCurrentPoseAndSpeeds(bool update_seq_latest_poses)
{
	{
		mrpt::utils::CTimeLoggerEntry tle(m_timlog_delays, "getCurrentPoseAndSpeeds()");
		if (!m_robot.getCurrentPoseAndSpeeds(m_curPoseVel.pose, m_curPoseVel.velGlobal, m_curPoseVel.timestamp))
		{
			m_navigationState = NAV_ERROR;
			try { 
				this->stop(true /*emergency*/);
			}
			catch (...) {}
			MRPT_LOG_ERROR("ERROR calling m_robot.getCurrentPoseAndSpeeds, stopping robot and finishing navigation");
			throw std::runtime_error("ERROR calling m_robot.getCurrentPoseAndSpeeds, stopping robot and finishing navigation");
		}
	}
	m_curPoseVel.velLocal = m_curPoseVel.velGlobal;
	m_curPoseVel.velLocal.rotate(-m_curPoseVel.pose.phi);

	// Append to list of past poses:
	m_latestPoses.insert(m_curPoseVel.timestamp, mrpt::poses::CPose3D(mrpt::math::TPose3D(m_curPoseVel.pose)));

	// Purge old ones:
	while (!m_latestPoses.empty() &&
		mrpt::system::timeDifference(m_latestPoses.begin()->first, m_latestPoses.rbegin()->first) > PREVIOUS_POSES_MAX_AGE)
	{
		m_latestPoses.erase(m_latestPoses.begin());
	}
}

bool CAbstractNavigator::changeSpeeds(const mrpt::kinematics::CVehicleVelCmd &vel_cmd)
{
	return m_robot.changeSpeeds(vel_cmd);
}
bool CAbstractNavigator::changeSpeedsNOP()
{
	return m_robot.changeSpeedsNOP();
}
bool CAbstractNavigator::stop(bool isEmergencyStop)
{
	return m_robot.stop(isEmergencyStop);
}
