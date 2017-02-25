/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/**
 * times
 * Prints current time instances in UTC format 
 */

#include <mrpt/utils.h>


using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::utils;

// ------------------------------------------------------
//				TestTypes
// ------------------------------------------------------
void TestTimes()
{
	TTimeStamp   t;
	for (size_t i=0;i<20;i++)
	{
		t = mrpt::system::getCurrentTime();
		std::cout << mrpt::system::dateTimeToString( t ) << std::endl;
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
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}
