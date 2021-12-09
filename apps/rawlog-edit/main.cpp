/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
   APPLICATION: rawlog-edit
   For instructions and more: see manpage of rawlog-edit
  ---------------------------------------------------------------*/

#include <mrpt/apps/RawlogEditApp.h>

#include <iostream>

int main(int argc, char** argv)
{
	try
	{
		mrpt::apps::RawlogEditApp app;

		app.run(argc, argv);

		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what();
		return 1;
	}
}
