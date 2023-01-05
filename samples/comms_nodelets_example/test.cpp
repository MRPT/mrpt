/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#define NODELETS_TEST_VERBOSE
#include <mrpt/core/exceptions.h>

#include <iostream>

#include "NodeletsTest_impl.cpp"

int main()
{
	try
	{
		NodeletsTest();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
}
