/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/nav/reactive/CRobot2NavInterface.h>
#include <iostream>

using namespace mrpt::nav;

CRobot2NavInterface::CRobot2NavInterface()
{
}
CRobot2NavInterface::~CRobot2NavInterface()
{
}
bool CRobot2NavInterface::changeSpeedsNOP()
{
	MRPT_UNSCOPED_LOGGER_START;
	MRPT_LOG_THROTTLE_INFO(10.0, "[changeSpeedsNOP] Doing nothing : not implemented in user's derived class.");
	MRPT_UNSCOPED_LOGGER_END;
	return true;
}

mrpt::kinematics::CVehicleVelCmd::Ptr CRobot2NavInterface::getAlignCmd(const double relative_heading_radians)
{
	return mrpt::kinematics::CVehicleVelCmd::Ptr();
}

bool CRobot2NavInterface::startWatchdog(float T_ms)
{
	std::cout << "[startWatchdog] Period=" << T_ms << "ms. Doing nothing: not implemented in user's derived class." << std::endl;
	return true;
}

bool CRobot2NavInterface::stopWatchdog() 
{
	std::cout << "[stopWatchdog] Doing nothing: not implemented in user's derived class." << std::endl;
	return true;
}

void CRobot2NavInterface::sendNavigationStartEvent() 
{
	std::cout << "[sendNavigationStartEvent] Doing nothing: not implemented in user's derived class." << std::endl; 
}
void CRobot2NavInterface::sendNavigationEndEvent() 
{
	std::cout << "[sendNavigationEndEvent] Doing nothing: not implemented in user's derived class." << std::endl;
}
void CRobot2NavInterface::sendWaypointReachedEvent(int waypoint_index, bool reached_nSkipped) 
{
	std::cout << "[sendWaypointReachedEvent] Marking waypoint #" << waypoint_index << " as done. Reason: " << (reached_nSkipped ? "Physically reached" : "Skipped") << std::endl;
}
void CRobot2NavInterface::sendNewWaypointTargetEvent(int waypoint_index) 
{
	std::cout << "[sendNewWaypointTargetEvent] Navigating towards waypoint #" << waypoint_index << std::endl;
}
void CRobot2NavInterface::sendNavigationEndDueToErrorEvent()
{
	std::cout << "[sendNavigationEndDueToErrorEvent] Doing nothing: not implemented in user's derived class." << std::endl;
}
void CRobot2NavInterface::sendWaySeemsBlockedEvent() 
{
	std::cout << "[sendWaySeemsBlockedEvent] Doing nothing: not implemented in user's derived class." << std::endl;
}
double CRobot2NavInterface::getNavigationTime() {
	return m_navtime.Tac();
}
void CRobot2NavInterface::resetNavigationTimer() {
	m_navtime.Tic();
}
