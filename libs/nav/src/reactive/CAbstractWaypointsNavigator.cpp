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

CAbstractWaypointsNavigator::CAbstractWaypointsNavigator(CRobot2NavInterface &robot_if) :
	CAbstractNavigator(robot_if)
{
}

CAbstractWaypointsNavigator::~CAbstractWaypointsNavigator()
{
}

