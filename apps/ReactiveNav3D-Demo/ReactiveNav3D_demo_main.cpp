/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/config/CConfigFile.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <fstream>
#include <iostream>
#include "ReactiveNav3D_demo.h"

using namespace std;
using namespace mrpt::config;
using namespace mrpt::math;
using namespace std::literals;

int main(int num_arg, char* argv[])
{
	try
	{
		// Read function arguments
		//----------------------------------------------------------------------
		bool enable_logfile = true;

		if ((num_arg != 2) || (num_arg == 2 && string(argv[1]) == "--help"))
		{
			printf("\n\t       Arguments of the function 'main' \n");
			printf(
				"=============================================================="
				"\n\n");
			printf(" --help: Shows this menu... \n\n");
			printf(
				" <reactive3d_config.ini>: Load configuration from the given "
				"config file \n\n");
			system::os::getch();
			return 1;
		}

		// Initial steps. Load configuration from file or default
		//------------------------------------------------------
		CMyReactInterface ReactInterface;
		CReactiveNavigationSystem3D rn3d(ReactInterface, true, enable_logfile);
		rn3d.enableTimeLog(true);

		const auto cfgFilename = std::string(argv[1]);
		std::cout << "Using config file: " << cfgFilename << "\n";

		CConfigFile configNavigation(cfgFilename);
		rn3d.loadConfigFile(configNavigation);
		ReactInterface.loadMaps(configNavigation);
		ReactInterface.loadConfiguration(configNavigation);

		ReactInterface.initializeScene();
		rn3d.initialize();

		bool stop = false;
		bool moving_target = false;
		TPoint3D last_Target_Pos(0, 0, 0);
		CTicTac reactive_period;
		reactive_period.Tic();

		MyObserver observer;
		observer.observeBegin(ReactInterface.window);
		observer.mouse_click = false;

		while (!stop)
		{
			int pushed_key = 0;
			if (ReactInterface.window.keyHit())
				pushed_key = ReactInterface.window.getPushedKey();

			switch (pushed_key)
			{
				case 'p':
				case 'P':
					// Pause navigation
					rn3d.suspend();
					break;

				case 'r':
				case 'R':
					// Resume navigation
					rn3d.resume();
					break;

				case 'm':
				case 'M':
					// Move the target
					moving_target = 1;
					break;

				case 'e':
				case 'E':
					// Exit program
					stop = 1;
					break;
			}

			// Set the target when the user clicks the mouse
			if (observer.mouse_click == 1)
			{
				observer.mouse_click = false;
				if (moving_target == 1)
				{
					moving_target = false;
					const CAbstractNavigator::TNavigationParams nav_params =
						ReactInterface.createNewTarget(
							last_Target_Pos.x, last_Target_Pos.y, 0.3f, false);
					rn3d.navigate(&nav_params);
				}
			}

			// Execute navigation
			rn3d.navigationStep();
			ReactInterface.robotSim.simulateOneTimeStep(reactive_period.Tac());
			reactive_period.Tic();

			if ((rn3d.IDLE == rn3d.getCurrentState()) ||
				(rn3d.SUSPENDED == rn3d.getCurrentState()))
			{
				CSimplePointsMap auxpoints;
				mrpt::system::TTimeStamp auxpoints_time;
				ReactInterface.senseObstacles(auxpoints, auxpoints_time);
			}
			ReactInterface.updateScene();
			std::this_thread::sleep_for(5ms);

			// Move target with the mouse
			if (moving_target == 1)
			{
				int mouse_x, mouse_y;
				if (ReactInterface.window.getLastMousePosition(
						mouse_x, mouse_y))
				{
					// Get the ray in 3D for the latest mouse (X,Y):
					math::TLine3D ray;
					ReactInterface.scene->getViewport("main")
						->get3DRayForPixelCoord(mouse_x, mouse_y, ray);

					// Create a 3D plane, e.g. Z=0
					const math::TPlane ground_plane(
						TPoint3D(0, 0, 0), TPoint3D(1, 0, 0),
						TPoint3D(0, 1, 0));

					// Intersection of the line with the plane:
					math::TObject3D inters;
					math::intersect(ray, ground_plane, inters);

					// Interpret the intersection as a point, if there is an
					// intersection:
					if (inters.getPoint(last_Target_Pos))
					{
						// Move an object to the position picked by the user:
						ReactInterface.scene->getByClass<CDisk>(0)->setLocation(
							last_Target_Pos.x, last_Target_Pos.y,
							last_Target_Pos.z);
					}
				}
			}
		}

		return 0;
	}
	catch (const std::exception& e)
	{
		std::cout << "MRPT exception caught: " << mrpt::exception_to_str(e)
				  << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}
