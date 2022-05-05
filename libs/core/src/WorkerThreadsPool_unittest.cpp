/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/config.h>
#include <mrpt/core/WorkerThreadsPool.h>

#if !MRPT_IN_EMSCRIPTEN	 // No multithreading
TEST(WorkerThreadsPool, runTasks)
{
	//
	int accum = 0;

	auto f = [&accum](int x) { accum += x; };

	{
		mrpt::WorkerThreadsPool pool(1);

		auto fut1 = pool.enqueue(f, 1);
		auto fut2 = pool.enqueue(f, 2);
		auto fut3 = pool.enqueue(f, 3);

		const auto n = pool.pendingTasks();
		EXPECT_GE(n, 0);
		EXPECT_LE(n, 3);

		fut1.wait();
		fut2.wait();
		fut3.wait();
	}
	EXPECT_EQ(accum, 6);
}
#endif	// !MRPT_IN_EMSCRIPTEN
