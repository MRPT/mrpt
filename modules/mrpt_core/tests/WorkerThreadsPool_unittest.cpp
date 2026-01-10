/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <gtest/gtest.h>
#include <mrpt/core/config.h>
#include <mrpt/core/WorkerThreadsPool.h>

#if !MRPT_IN_EMSCRIPTEN  // No multithreading
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
    EXPECT_GE(n, 0U);
    EXPECT_LE(n, 3U);

    fut1.wait();
    fut2.wait();
    fut3.wait();
  }
  EXPECT_EQ(accum, 6);
}
#endif  // !MRPT_IN_EMSCRIPTEN
