/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>
#include <mrpt/slam.h>
#include <mrpt/hwdrivers/CJoystick.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::hwdrivers;


// ------------------------------------------------------
//				TestJoystick
// ------------------------------------------------------
void TestJoystick()
{
	// Open first joystick:
	// ---------------------------
	float		x,y,z;
	vector_bool	buttons;
	CTicTac		tictac;
	CJoystick	joy;

	const int nJoystick = 0;	// The first one

	printf("Press any key to stop program...\n");

	while ( !mrpt::system::os::kbhit() )
	{
		tictac.Tic();
		if (joy.getJoystickPosition(nJoystick,x,y,z,buttons) )
		{
			double t = tictac.Tac();

			printf("Joystick readings: %.03f, %.03f, %.03f  (", x, y, z);
			for(unsigned b=0;b<buttons.size();b++)
				printf("B%u:%c ", b,buttons[b] ? 'X':'-');
			printf(") [Query %uus]  \r", (unsigned)(t*1e6));

			fflush(stdout);
		}
		else
		{
			printf("Error reading from joystick, please connect one to the system...\r");
		}

		mrpt::system::sleep(20);
	}
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestJoystick();

		return 0;
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}
