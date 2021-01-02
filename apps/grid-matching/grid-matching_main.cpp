/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/apps/CGridMapAlignerApp.h>

int main(int argc, char** argv)
{
	try
	{
		mrpt::apps::CGridMapAlignerApp app;

		app.initialize(argc, argv);
		app.run();
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e) << std::endl;
		return 1;
	}
}
