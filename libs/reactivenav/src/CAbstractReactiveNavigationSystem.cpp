/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/reactivenav.h>  // Precomp header

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
	m_robot(react_iterf_impl)
{
	m_navigationState =
	m_lastNavigationState = IDLE;
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
				printf_debug("[CAbstractReactiveNavigationSystem::navigationStep()] Navigation Params:\n%s\n", m_navigationParams.getAsText().c_str() );

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
