/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/core/lock_helper.h>
#include <mutex>

TEST(lock_helper, testCompilation)
{
	{
		std::mutex mtx;
		{
			auto lck = mrpt::lockHelper(mtx);
			// protected code
		}
	}
	{
		std::recursive_mutex mtx;
		{
			auto lck = mrpt::lockHelper(mtx);
			// protected code
		}
	}
}
