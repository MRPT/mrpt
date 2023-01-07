/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/** \example system_progress_bar/test.cpp */

//! [example-system-progress-bar]

#include <mrpt/core/exceptions.h>
#include <mrpt/system/progress.h>

#include <chrono>
#include <iostream>
#include <thread>

void TestProgressBar()
{
	const int end = 400;
	const size_t barWidth = 30;

	for (int i = 0; i <= end; i++)
	{
		double p = double(i) / end;

		std::cout << "Progress: " << mrpt::system::progress(p, barWidth)
				  << mrpt::format(" %5.02f%%", 100 * p) << "\r";
		std::cout.flush();
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	std::cout << std::endl;
}

//! [example-system-progress-bar]

int main()
{
	try
	{
		TestProgressBar();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}
