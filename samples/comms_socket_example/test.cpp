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

#define SOCKET_TEST_VERBOSE
#include <iostream>

#include "SocketsTest_impl.cpp"

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
  try
  {
    SocketsTest();
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "MRPT exception caught: " << e.what() << std::endl;
    return -1;
  }
  catch (...)
  {
    std::cerr << "Untyped exception!!";
    return -1;
  }
}
