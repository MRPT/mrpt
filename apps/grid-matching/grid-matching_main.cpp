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

#include <mrpt/apps/CGridMapAlignerApp.h>

int main(int argc, char** argv)
{
  try
  {
    mrpt::apps::CGridMapAlignerApp app;

    app.initialize(argc, argv);
    app.run();
  }
  catch (const std::exception& e)
  {
    std::cerr << mrpt::exception_to_str(e) << std::endl;
    return 1;
  }
}
