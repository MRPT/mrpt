/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "reactivenav-precomp.h" // Precomp header

#include <mrpt/reactivenav/CAbstractReactiveNavigationSystem.h>

using namespace mrpt::reactivenav;
using namespace std;

// Ctor: CAbstractReactiveNavigationSystem::TNavigationParams 
CAbstractReactiveNavigationSystem::TNavigationParams::TNavigationParams() :
	target(0,0), 
	targetHeading(0),
	targetAllowedDistance(0.5),
	targetIsRelative(false)
{
}

// Gets navigation params as a human-readable format:
std::string CAbstractReactiveNavigationSystem::TNavigationParams::getAsText() const 
{
	string s;
	s+= mrpt::format("navparams.target = (%.03f,%.03f)\n", target.x, target.y );
	s+= mrpt::format("navparams.targetAllowedDistance = %.03f\n", targetAllowedDistance );
	s+= mrpt::format("navparams.targetIsRelative = %s\n", targetIsRelative ? "YES":"NO");

	return s;
}


/*---------------------------------------------------------------
							Constructor
  ---------------------------------------------------------------*/
CAbstractReactiveNavigationSystem::CAbstractReactiveNavigationSystem(CReactiveInterfaceImplementation &react_iterf_impl) :
	m_lastNavigationState ( IDLE ),
	m_navigationState     ( IDLE ),
    m_navigationParams    ( NULL ),
    m_robot               ( react_iterf_impl )
{
}

// Dtor:
CAbstractReactiveNavigationSystem::~CAbstractReactiveNavigationSystem()
{
	mrpt::utils::delete_safe( m_navigationParams );
}

/*---------------------------------------------------------------
							cancel
  ---------------------------------------------------------------*/
void CAbstractReactiveNavigationSystem::cancel()
{
	printf_debug("\n[CAbstractReactiveNavigationSystem::Cancel()]\n");
	m_navigationState = IDLE;
}


/*---------------------------------------------------------------
							resume
  ---------------------------------------------------------------*/
void CAbstractReactiveNavigationSystem::resume()
{
	printf_debug("\n[CAbstractReactiveNavigationSystem::Continue()]\n");
	if ( m_navigationState == SUSPENDED )
		m_navigationState = NAVIGATING;
}


/*---------------------------------------------------------------
							suspend
  ---------------------------------------------------------------*/
void  CAbstractReactiveNavigationSystem::suspend()
{
	printf_debug("\n[CAbstractReactiveNavigationSystem::Suspend()]\n");
	if ( m_navigationState == NAVIGATING )
		m_navigationState  = SUSPENDED;
}

/*---------------------------------------------------------------
					navigationStep
  ---------------------------------------------------------------*/
void CAbstractReactiveNavigationSystem::navigationStep()
{
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
				printf_debug("\n[CAbstractReactiveNavigationSystem::navigationStep()] Navigation stopped\n");
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
				printf_debug("\n[CAbstractReactiveNavigationSystem::navigationStep()] Stoping Navigation due to a NAV_ERROR state!\n");
				m_robot.stop();
				m_robot.stopWatchdog();
			}
		} catch (...) { }
		break;

	//------------------------------------------------------------------
	//						ALGORITMO DE NAVEGACION
	//------------------------------------------------------------------
	case NAVIGATING:
		try
		{
			if ( m_lastNavigationState != NAVIGATING )
			{
				printf_debug("\n[CAbstractReactiveNavigationSystem::navigationStep()] Starting Navigation. Watchdog initiated...\n");
				if (m_navigationParams)
					printf_debug("[CAbstractReactiveNavigationSystem::navigationStep()] Navigation Params:\n%s\n", m_navigationParams->getAsText().c_str() );

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
			cerr << "[CAbstractReactiveNavigationSystem::navigationStep] Exception:\n" << e.what() << endl;
		}
		catch (...)
		{
			cerr << "[CAbstractReactiveNavigationSystem::navigationStep] Unexpected exception.\n";
		}
		break;	// End case NAVIGATING
	};
	m_lastNavigationState = prevState;
}
