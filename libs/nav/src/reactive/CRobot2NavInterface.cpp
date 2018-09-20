/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "nav-precomp.h"  // Precomp header
#include <mrpt/system/COutputLogger.h>
#include <mrpt/nav/reactive/CRobot2NavInterface.h>

using namespace mrpt::nav;

CRobot2NavInterface::CRobot2NavInterface()
	: mrpt::system::COutputLogger("CRobot2NavInterface")
{
}
CRobot2NavInterface::~CRobot2NavInterface() = default;
bool CRobot2NavInterface::changeSpeedsNOP()
{
	MRPT_LOG_THROTTLE_INFO(
		10.0,
		"[changeSpeedsNOP] Doing nothing : not implemented in user's derived "
		"class.");
	return true;
}

mrpt::kinematics::CVehicleVelCmd::Ptr CRobot2NavInterface::getAlignCmd(
	const double relative_heading_radians)
{
	return mrpt::kinematics::CVehicleVelCmd::Ptr();
}

/** \callergraph */
bool CRobot2NavInterface::startWatchdog(float T_ms)
{
	MRPT_LOG_INFO_FMT(
		"[startWatchdog] Period=%.03f ms. Doing nothing: not implemented in "
		"user's derived class.",
		T_ms);
	return true;
}

/** \callergraph */
bool CRobot2NavInterface::stopWatchdog()
{
	MRPT_LOG_INFO(
		"[stopWatchdog] Doing nothing: not implemented in user's derived "
		"class.");
	return true;
}

/** \callergraph */
void CRobot2NavInterface::sendNavigationStartEvent()
{
	MRPT_LOG_INFO(
		"[sendNavigationStartEvent] Doing nothing: not implemented in user's "
		"derived class.");
}
/** \callergraph */
void CRobot2NavInterface::sendNavigationEndEvent()
{
	MRPT_LOG_INFO(
		"[sendNavigationEndEvent] Doing nothing: not implemented in user's "
		"derived class.");
}
/** \callergraph */
void CRobot2NavInterface::sendWaypointReachedEvent(
	int waypoint_index, bool reached_nSkipped)
{
	MRPT_LOG_INFO_STREAM(
		"[sendWaypointReachedEvent] Marking waypoint #"
		<< waypoint_index << " as done. Reason: "
		<< (reached_nSkipped ? "Physically reached" : "Skipped"));
}
/** \callergraph */
void CRobot2NavInterface::sendNewWaypointTargetEvent(int waypoint_index)
{
	MRPT_LOG_INFO_STREAM(
		"[sendNewWaypointTargetEvent] Navigating towards waypoint #"
		<< waypoint_index);
}
/** \callergraph */
void CRobot2NavInterface::sendNavigationEndDueToErrorEvent()
{
	MRPT_LOG_THROTTLE_INFO(
		1.0,
		"[sendNavigationEndDueToErrorEvent] Doing nothing: not implemented in "
		"user's derived class.");
}
/** \callergraph */
void CRobot2NavInterface::sendWaySeemsBlockedEvent()
{
	MRPT_LOG_THROTTLE_INFO(
		1.0,
		"[sendWaySeemsBlockedEvent] Doing nothing: not implemented in user's "
		"derived class.");
}
/** \callergraph */
void CRobot2NavInterface::sendApparentCollisionEvent()
{
	MRPT_LOG_THROTTLE_INFO(
		1.0,
		"[sendApparentCollisionEvent] Doing nothing: not implemented in user's "
		"derived class.");
}
/** \callergraph */
void CRobot2NavInterface::sendCannotGetCloserToBlockedTargetEvent()
{
	MRPT_LOG_THROTTLE_INFO(
		1.0,
		"[sendCannotGetCloserToBlockedTargetEvent] Doing nothing: not "
		"implemented in user's derived class.");
}

/** \callergraph */
double CRobot2NavInterface::getNavigationTime() { return m_navtime.Tac(); }
/** \callergraph */
void CRobot2NavInterface::resetNavigationTimer() { m_navtime.Tic(); }
