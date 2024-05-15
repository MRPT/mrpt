/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mrpt/hwdrivers/CJoystick.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/os.h>

#include <chrono>
#include <cstdio>
#include <iostream>
#include <thread>

using namespace std;
using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;

// ------------------------------------------------------
//				TestJoystick
// ------------------------------------------------------
void TestJoystick()
{
  // Open first joystick:
  // ---------------------------
  CTicTac tictac;

  CJoystick joy;
  CJoystick::State js;

  const int nJoystick = 0;  // The first one

  printf("Press any key to stop program...\n");

  while (!mrpt::system::os::kbhit())
  {
    tictac.Tic();
    if (joy.getJoystickPosition(nJoystick, js))
    {
      double t = tictac.Tac();

      printf("Joystick axes: ");
      for (unsigned int i = 0; i < js.axes.size(); i++) printf("[%u]=%.03f ", i, js.axes[i]);

      for (unsigned b = 0; b < js.buttons.size(); b++)
        printf("B%u:%c ", b, js.buttons[b] ? 'X' : '-');
      printf(") [Query %uus]  \r", (unsigned)(t * 1e6));

      fflush(stdout);
    }
    else
    {
      printf(
          "Error reading from joystick, please connect one to the "
          "system...\r");
    }

    std::this_thread::sleep_for(20ms);
  }
}

int main()
{
  try
  {
    TestJoystick();

    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
    return -1;
  }
}
