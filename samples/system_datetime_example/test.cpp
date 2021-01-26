/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/**
 * times
 * Prints current time instances in UTC format
 */

#include <mrpt/system/datetime.h>

#include <iostream>

using namespace mrpt;
using namespace mrpt::system;

// ------------------------------------------------------
//				TestTypes
// ------------------------------------------------------
void TestTimes()
{
	TTimeStamp t;
	for (size_t i = 0; i < 20; i++)
	{
		t = mrpt::system::getCurrentTime();
		std::cout << mrpt::system::dateTimeToString(t) << std::endl;
	}
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestTimes();

		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}
