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

using namespace mrpt::nav;
using namespace std;

// Ctor: CAbstractNavigator::TNavigationParams 
CAbstractNavigator::TNavigationParams::TNavigationParams() :
	target(0,0,0), 
	targetAllowedDistance(0.5),
	targetIsRelative(false)
{
}

// Gets navigation params as a human-readable format:
std::string CAbstractNavigator::TNavigationParams::getAsText() const 
{
	string s;
	s+= mrpt::format("navparams.target = (%.03f,%.03f,%.03f deg)\n", target.x, target.y,target.phi );
	s+= mrpt::format("navparams.targetAllowedDistance = %.03f\n", targetAllowedDistance );
	s+= mrpt::format("navparams.targetIsRelative = %s\n", targetIsRelative ? "YES":"NO");

	return s;
}


/*---------------------------------------------------------------
							Constructor
  ---------------------------------------------------------------*/
CAbstractNavigator::CAbstractNavigator(CRobot2NavInterface &react_iterf_impl) :
	m_lastNavigationState ( IDLE ),
	m_navigationState     ( IDLE ),
	m_navigationParams    ( NULL ),
	m_robot               ( react_iterf_impl )
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
				m_robot.stop();
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
			if ( m_lastNavigationState != NAVIGATING )
			{
				printf_debug("\n[CAbstractNavigator::navigationStep()] Starting Navigation. Watchdog initiated...\n");
				if (m_navigationParams)
					printf_debug("[CAbstractNavigator::navigationStep()] Navigation Params:\n%s\n", m_navigationParams->getAsText().c_str() );

				m_robot.startWatchdog( 1000 );	// Watchdog = 1 seg
			}

			// Start navigation??
			if ( m_lastNavigationState == IDLE )
				m_robot.sendNavigationStartEvent();

			// The normal execution of the navigation: Execute one step
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
	printf_debug(msg);
	printf_debug("\n");
}
