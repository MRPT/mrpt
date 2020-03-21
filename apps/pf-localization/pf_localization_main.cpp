/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: Particle Filter (Global) Localization Demo
	FILE: pf_localization_main.cpp
	AUTHOR: Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>

	For instructions and more:
	 https://www.mrpt.org/list-of-mrpt-apps/application-pf-localization
  ---------------------------------------------------------------*/

#include <mrpt/apps/MonteCarloLocalization_App.h>
#include <iostream>

int main(int argc, char** argv)
{
	try
	{
		mrpt::apps::MonteCarloLocalization_Rawlog app;

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
