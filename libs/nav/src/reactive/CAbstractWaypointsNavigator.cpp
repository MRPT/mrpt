/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/reactive/CAbstractWaypointsNavigator.h>

using namespace mrpt::nav;
using namespace std;

// Ctor: CAbstractNavigator::TNavigationParams 
CAbstractWaypointsNavigator::TWaypointNavigationCommand::TWaypointNavigationCommand()
{
}

// Gets navigation params as a human-readable format:
std::string CAbstractWaypointsNavigator::TWaypointNavigationCommand::getAsText() const 
{
	string s;
//	s+= mrpt::format("navparams.target = (%.03f,%.03f,%.03f deg)\n", target.x, target.y,target.phi );
//	s+= mrpt::format("navparams.targetAllowedDistance = %.03f\n", targetAllowedDistance );

	return s;
}


CAbstractWaypointsNavigator::CAbstractWaypointsNavigator(CRobot2NavInterface &robot_if) :
	CAbstractNavigator(robot_if)
{
}

CAbstractWaypointsNavigator::~CAbstractWaypointsNavigator()
{
}

