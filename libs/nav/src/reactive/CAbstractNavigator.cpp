/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
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


/*---------------------------------------------------------------
							Constructor
  ---------------------------------------------------------------*/
CAbstractNavigator::CAbstractNavigator(CRobot2NavInterface &react_iterf_impl) :
	m_lastNavigationState ( IDLE ),
	m_navigationEndEventSent(false),
	m_navigationState     ( IDLE ),
	m_navigationParams    ( NULL ),
	m_robot               ( react_iterf_impl ),
	m_curPose             (0,0,0),
	m_curVel              (0,0,0),
	m_curVelLocal         (0,0,0),
	m_badNavAlarm_AlarmTimeout(30.0),
	DIST_TO_TARGET_FOR_SENDING_EVENT(.0)
{
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
	printf_debug("\n[CAbstractNavigator::cancel()]\n");
	m_navigationState = IDLE;
	m_robot.stop();
}


/*---------------------------------------------------------------
							resume
  ---------------------------------------------------------------*/
void CAbstractNavigator::resume()
{
	mrpt::synch::CCriticalSectionLocker csl(&m_nav_cs);

	printf_debug("\n[CAbstractNavigator::resume()]\n");
	if ( m_navigationState == SUSPENDED )
		m_navigationState = NAVIGATING;
}


/*---------------------------------------------------------------
							suspend
  ---------------------------------------------------------------*/
void  CAbstractNavigator::suspend()
{
	mrpt::synch::CCriticalSectionLocker csl(&m_nav_cs);

	printf_debug("\n[CAbstractNavigator::suspend()]\n");
	if ( m_navigationState == NAVIGATING )
		m_navigationState  = SUSPENDED;
}

void CAbstractNavigator::resetNavError()
{
	mrpt::synch::CCriticalSectionLocker csl(&m_nav_cs);

	printf_debug("\n[CAbstractNavigator::resetNavError()]\n");
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
				printf_debug("\n[CAbstractNavigator::navigationStep()] Navigation stopped\n");
				// m_robot.stop();  stop() is called by the method switching the "state", so we have more flexibility
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
				printf_debug("\n[CAbstractNavigator::navigationStep()] Stoping Navigation due to a NAV_ERROR state!\n");
				m_robot.stop();
				m_robot.stopWatchdog();
			}
		} catch (...) { }
		break;

	case NAVIGATING:
		try
		{
			bool is_first_nav_step = false;
			if ( m_lastNavigationState != NAVIGATING )
			{
				printf_debug("\n[CAbstractNavigator::navigationStep()] Starting Navigation. Watchdog initiated...\n");
				if (m_navigationParams)
					printf_debug("[CAbstractNavigator::navigationStep()] Navigation Params:\n%s\n", m_navigationParams->getAsText().c_str() );

				m_robot.startWatchdog( 1000 );	// Watchdog = 1 seg
				is_first_nav_step=true;
			}

			// Have we just started the navigation?
			if ( m_lastNavigationState == IDLE )
				m_robot.sendNavigationStartEvent();

			/* ----------------------------------------------------------------
			        Get current robot dyn state:
				---------------------------------------------------------------- */
			if ( !m_robot.getCurrentPoseAndSpeeds(m_curPose, m_curVel) )
			{
				m_navigationState = NAV_ERROR;
				m_robot.stop();
				throw std::runtime_error("ERROR calling m_robot.getCurrentPoseAndSpeeds, stopping robot and finishing navigation");
			}
			m_curVelLocal = m_curVel;
			m_curVelLocal.rotate(-m_curPose.phi);

			if (is_first_nav_step) m_lastPose = m_curPose;


			/* ----------------------------------------------------------------
		 			Have we reached the target location?
				---------------------------------------------------------------- */
			const mrpt::math::TSegment2D seg_robot_mov = mrpt::math::TSegment2D( mrpt::math::TPoint2D(m_curPose), mrpt::math::TPoint2D(m_lastPose) );

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
					m_robot.stop();
				}
				m_navigationState = IDLE;
				printf_debug("Navigation target (%.03f,%.03f) was reached\n", m_navigationParams->target.x,m_navigationParams->target.y);

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
					printf_debug("\n--------------------------------------------\nWARNING: Timeout for approaching toward the target expired!! Aborting navigation!! \n---------------------------------\n");
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

void CAbstractNavigator::doEmergencyStop( const char *msg )
{
	m_navigationState = NAV_ERROR;
	m_robot.stop();
	printf_debug("%s\n",msg);
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
		mrpt::math::TPose2D  currentPose;
		mrpt::math::TTwist2D cur_vel;
		if ( !m_robot.getCurrentPoseAndSpeeds(currentPose, cur_vel) )
		{
			doEmergencyStop("\n[CAbstractNavigator] Error querying current robot pose to resolve relative coordinates\n");
			return;
		}

		const mrpt::poses::CPose2D relTarget(m_navigationParams->target);
		mrpt::poses::CPose2D absTarget;
		absTarget.composeFrom(currentPose, relTarget);

		m_navigationParams->target = mrpt::math::TPose2D(absTarget);

		m_navigationParams->targetIsRelative = false; // Now it's not relative
	}

	// new state:
	m_navigationState = NAVIGATING;

	// Reset the bad navigation alarm:
	m_badNavAlarm_minDistTarget = std::numeric_limits<double>::max();
	m_badNavAlarm_lastMinDistTime = mrpt::system::getCurrentTime();
}
