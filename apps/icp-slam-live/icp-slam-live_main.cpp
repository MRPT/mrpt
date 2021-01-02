/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: ICP-based SLAM, live version
	FILE: icp-slam-live_main.cpp
	AUTHOR: Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>
	See example config files in
	 https://github.com/MRPT/mrpt/tree/master/share/mrpt/config_files/icp-slam-live/
	or docs in
	 https://www.mrpt.org/list-of-mrpt-apps/application-icp-slam-live/
  ---------------------------------------------------------------*/

#include <mrpt/apps/ICP_SLAM_App.h>
#include <iostream>

int main(int argc, char** argv)
{
	try
	{
		mrpt::apps::ICP_SLAM_App_Live app;

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
