/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/core/WorkerThreadsPool.h>

TEST(WorkerThreadsPool, runTasks)
{
	//
	int accum = 0;

	auto f = [&accum](int x) { accum += x; };

	{
		mrpt::WorkerThreadsPool pool(1);

		pool.enqueue(f, 1);
		pool.enqueue(f, 2);
		pool.enqueue(f, 3);

		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}
	EXPECT_EQ(accum, 6);
}
