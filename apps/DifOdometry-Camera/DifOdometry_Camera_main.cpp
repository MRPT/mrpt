/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "DifOdometry_Camera.h"
#include <mrpt/system/os.h>
#include <mrpt/utils/CConfigFileMemory.h>
#include <mrpt/utils/CConfigFile.h>

using namespace std;
using namespace mrpt;

const char *default_cfg_txt =
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


int main(int num_arg, char *argv[])
{
	try
	{
		//						Read function arguments
		//----------------------------------------------------------------------
		bool use_config_file = 0;
		string filename;
		CDifodoCamera odo;

		if (num_arg < 2);
		else if ( string(argv[1]) == "--help")
		{
			printf("\n\t       Arguments of the function 'main' \n");
			printf("==============================================================\n\n");
			printf(" --help: Shows this menu... \n\n");
			printf(" --config FICH.txt: Load FICH.txt as config file \n\n");
			printf(" --create-config FICH.txt: Save the default config parameters \n\n");
			printf(" \t\t\t   in FICH.txt and close the program \n\n");
			printf(" --save-logfile: Enable saving a file with results of the pose estimate \n\n");
			system::os::getch();
			return 1;
		}
		else if ( string(argv[1]) == "--create-config")
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
			for (int i=1; i<num_arg; i++)
			{
				if ( string(argv[i]) == "--save-logfile")
				{
					odo.save_results = 1;
					odo.CreateResultsFile();
				}

				if ( string(argv[i]) == "--config")
				{
					use_config_file = 1;
					filename = argv[i+1];
				}
			}
		}

		//Initial steps. Load configuration from file or default, initialize scene and initialize camera
		//----------------------------------------------------------------------------------------------

		if (use_config_file == 0)
		{
			utils::CConfigFileMemory configDifodo(default_cfg_txt);
			odo.loadConfiguration( configDifodo );
		}
		else
		{
			utils::CConfigFile configDifodo(filename);
			odo.loadConfiguration( configDifodo );
		}

		odo.initializeScene();
		odo.openCamera();

		//==============================================================================
		//									Main operation
		//==============================================================================

		int pushed_key = 0;
		bool working = 0, stop = 0;
		utils::CTicTac	main_clock;
		main_clock.Tic();

		odo.reset();

		while (!stop)
		{

			if (odo.window.keyHit())
				pushed_key = odo.window.getPushedKey();
			else
				pushed_key = 0;

			switch (pushed_key) {

			//Capture a new depth frame and calculate odometry
			case  'n':
				odo.loadFrame();
				odo.odometryCalculation();
				if (odo.save_results == 1)
					odo.writeTrajectoryFile();

				cout << endl << "Difodo runtime(ms): " << odo.execution_time;
				odo.updateScene();
				break;

			//Start and stop continous odometry
			case 's':
				working = !working;
				break;

			//Close the program
			case 'e':
				stop = 1;
				if (odo.f_res.is_open())
					odo.f_res.close();
				break;

			//Reset estimation
			case 'r':
				odo.reset();
				break;

			}

			if (working == 1)
			{
				while(main_clock.Tac() < 1.f/odo.fps);
				if (main_clock.Tac() > 1.05f/odo.fps)
					cout << endl << "Not enough time to compute everything!!!";

				main_clock.Tic();

				odo.loadFrame();
				odo.odometryCalculation();
				if (odo.save_results == 1)
					odo.writeTrajectoryFile();

				cout << endl << "Difodo runtime(ms): " << odo.execution_time;
				odo.updateScene();
			}
		}

		odo.closeCamera();

		return 0;

	}
	catch (std::exception &e)
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

