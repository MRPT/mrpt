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
    t = mrpt::Clock::now();
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
}
