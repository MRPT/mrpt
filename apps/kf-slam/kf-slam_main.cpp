/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: Kalman Filter-based SLAM implementation
	FILE: kf-slam_main.cpp
	AUTHOR: Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>

	See README.txt for instructions.
 ---------------------------------------------------------------*/

#include <mrpt/apps/KFSLAMApp.h>
#include <iostream>

int main(int argc, char** argv)
{
	try
	{
		mrpt::apps::KFSLAMApp app;

		app.initialize(argc, argv);
		app.run();

		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e);
		mrpt::system::pause();
		return -1;
	}
}
