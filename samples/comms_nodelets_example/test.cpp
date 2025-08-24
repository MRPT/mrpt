/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

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
