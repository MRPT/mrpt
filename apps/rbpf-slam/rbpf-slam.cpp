/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: Rao-Blackwellized Particle Filter SLAM
	FILE: rbpf-slam.cpp
	AUTHOR: Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>

	See README.txt for instructions or
		 https://www.mrpt.org/list-of-mrpt-apps/application-rbpf-slam
  ---------------------------------------------------------------*/

#include <mrpt/apps/RBPF_SLAM_App.h>

#include <iostream>

int main(int argc, char** argv)
{
	try
	{
		mrpt::apps::RBPF_SLAM_App_Rawlog app;

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
