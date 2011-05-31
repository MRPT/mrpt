/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/reactivenav.h>  // Precomp header

using namespace mrpt::reactivenav;
using namespace std;


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
							Cancel
  ---------------------------------------------------------------*/
void CAbstractReactiveNavigationSystem::cancel()
{
	printf_debug("\n[CAbstractReactiveNavigationSystem::Cancel()]\n");
	m_navigationState = IDLE;
}


/*---------------------------------------------------------------
							Continue
  ---------------------------------------------------------------*/
void CAbstractReactiveNavigationSystem::resume()
{
	printf_debug("\n[CAbstractReactiveNavigationSystem::Continue()]\n");
	if ( m_navigationState == SUSPENDED )
		m_navigationState = NAVIGATING;
}


/*---------------------------------------------------------------
							Continue
  ---------------------------------------------------------------*/
void  CAbstractReactiveNavigationSystem::suspend()
{
	printf_debug("\n[CAbstractReactiveNavigationSystem::Suspend()]\n");
	if ( m_navigationState == NAVIGATING )
		m_navigationState  = SUSPENDED;
}

/*---------------------------------------------------------------
					NavigateStep

	  Se debe llamar continuamente, cada pocos milisegundos. Internamente
	   lleva el mismo el control del tiempo que pasa entre llamadas para
	   tener el cuenta el tiempo real.
  ---------------------------------------------------------------*/
void CAbstractReactiveNavigationSystem::navigationStep()
{
	TState	startingState = m_navigationState;
	switch ( m_navigationState )
	{
	//------------------------------------------------------
	//					PARAR ROBOT
	//------------------------------------------------------
	case IDLE:
	case SUSPENDED:
		try
		{
			// Si acabamos de llegar a este estado, parar el robot:
			if ( m_lastNavigationState == NAVIGATING )
			{
				printf_debug("\n[CAbstractReactiveNavigationSystem::navigationStep()] Stoping Navigation\n");
				m_robot.stop();
				m_robot.stopWatchdog();
			}
		} catch (...) { }
		break;

	//------------------------------------------------------
	//					FINALIZACION POR ERROR
	//------------------------------------------------------
	case NAV_ERROR:
		try
		{
			// Enviar evento de final de navegacion??
			if ( m_lastNavigationState == NAVIGATING && m_navigationState == NAV_ERROR)
				m_robot.sendNavigationEndDueToErrorEvent();

			// Si acabamos de llegar a este estado, parar el robot:
			if ( m_lastNavigationState == NAVIGATING )
			{
				printf_debug("\n[CAbstractReactiveNavigationSystem::navigationStep()] Stoping Navigation due to an error!!\n");
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
			// Si acabamos de llegar a este estado, parar el robot:
			if ( m_lastNavigationState != NAVIGATING )
			{
				printf_debug("\n[CAbstractReactiveNavigationSystem::navigationStep()] Starting Navigation. Watchdog initiated...\n");
				printf_debug(" TARGET = (%.03f,%.03f)\n", m_navigationParams.target.x, m_navigationParams.target.y );
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
			cerr << e.what() << endl;
			printf("[CAbstractReactiveNavigationSystem::navigationStep] Exceptions!!\n");
		}
		catch (...)
		{
			printf("[CAbstractReactiveNavigationSystem::navigationStep] Unexpected exception!!\n");
		}
		break;	// Fin de case NAVIGATING
	};
	m_lastNavigationState = startingState;
}
