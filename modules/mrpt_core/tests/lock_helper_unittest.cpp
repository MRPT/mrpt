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
