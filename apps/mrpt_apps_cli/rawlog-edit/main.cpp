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

/*---------------------------------------------------------------
   APPLICATION: rawlog-edit
   For instructions and more: see manpage of rawlog-edit
  ---------------------------------------------------------------*/

#include <mrpt/apps-cli/RawlogEditApp.h>

#include <iostream>

int main(int argc, char** argv)
{
  try
  {
    mrpt::apps::RawlogEditApp app;

    app.run(argc, argv);

    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
