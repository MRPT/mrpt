/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/config/CConfigFile.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/system/CRateTimer.h>
#include <mrpt/system/os.h>
#include "DifOdometry_Camera.h"

using namespace std;
using namespace mrpt;

const char* default_cfg_txt =
	"; ---------------------------------------------------------------\n"
	"; FILE: Difodo Parameters.txt\n"
	";\n"
	";  MJT @ JANUARY-2014\n"
	"; ---------------------------------------------------------------\n\n"

	"[DIFODO_CONFIG]\n\n"

	";cam_mode: 1 - 640x480, 2 - 320x240, 4 - 160x120 \n"
	"cam_mode = 2 \n\n"

	"Set the frame rate (fps) to 30 or 60 Hz \n"
	"fps = 30 \n\n"

	";Indicate the number of rows and columns. \n"
	"rows = 240 \n"
	"cols = 320 \n"
	"ctf_levels = 5 \n\n";

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------

int main(int num_arg, char* argv[])
{
	try
	{
		//						Read function arguments
		//----------------------------------------------------------------------
		bool use_config_file = false;
		string filename;
		CDifodoCamera odo;

		if (num_arg < 2)
			;
		else if (string(argv[1]) == "--help")
		{
			printf("\n\t       Arguments of the function 'main' \n");
			printf(
				"=============================================================="
				"\n\n");
			printf(" --help: Shows this menu... \n\n");
			printf(" --config FICH.txt: Load FICH.txt as config file \n\n");
			printf(
				" --create-config FICH.txt: Save the default config parameters "
				"\n\n");
			printf(" \t\t\t   in FICH.txt and close the program \n\n");
			printf(
				" --save-logfile: Enable saving a file with results of the "
				"pose estimate \n\n");
			system::os::getch();
			return 1;
		}
		else if (string(argv[1]) == "--create-config")
		{
			filename = argv[2];
			cout << endl << "Config_file name: " << filename;
			ofstream new_file(filename.c_str());
			new_file << string(default_cfg_txt);
			new_file.close();
			cout << endl << "File saved" << endl;
			system::os::getch();
			return 1;
		}
		else
		{
			for (int i = 1; i < num_arg; i++)
			{
				if (string(argv[i]) == "--save-logfile")
				{
					odo.save_results = true;
					odo.CreateResultsFile();
				}

				if (string(argv[i]) == "--config")
				{
					use_config_file = true;
					filename = argv[i + 1];
				}
			}
		}

		// Initial steps. Load configuration from file or default, initialize
		// scene and initialize camera
		//----------------------------------------------------------------------------------------------

		if (use_config_file)
		{
			mrpt::config::CConfigFileMemory configDifodo(default_cfg_txt);
			odo.loadConfiguration(configDifodo);
		}
		else
		{
			mrpt::config::CConfigFile configDifodo(filename);
			odo.loadConfiguration(configDifodo);
		}

		odo.initializeScene();
		odo.openCamera();

		//==============================================================================
		//									Main operation
		//==============================================================================

		int pushed_key = 0;
		bool working = false, stop = false;

		odo.reset();

		mrpt::system::CRateTimer rate(odo.fps);

		while (!stop)
		{
			if (odo.window.keyHit())
				pushed_key = odo.window.getPushedKey();
			else
				pushed_key = 0;

			switch (pushed_key)
			{
				// Capture a new depth frame and calculate odometry
				case 'n':
				case 'N':
					odo.loadFrame();
					odo.odometryCalculation();
					if (odo.save_results) odo.writeTrajectoryFile();

					cout << endl
						 << "Difodo runtime(ms): " << odo.execution_time;
					odo.updateScene();
					break;

				// Start and stop continous odometry
				case 's':
				case 'S':
					working = !working;
					break;

				// Close the program
				case 'e':
				case 'E':
					stop = true;
					if (odo.f_res.is_open()) odo.f_res.close();
					break;

					// Reset estimation
				case 'R':
				case 'r':
					odo.reset();
					break;
			}

			if (working)
			{
				const bool slower_than_realtime = rate.sleep();
				if (slower_than_realtime)
					cout << endl
						 << "Not enough time to compute everything!!!\n";

				odo.loadFrame();
				odo.odometryCalculation();
				if (odo.save_results) odo.writeTrajectoryFile();

				cout << endl << "Difodo runtime(ms): " << odo.execution_time;
				odo.updateScene();
			}
		}

		odo.closeCamera();

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
