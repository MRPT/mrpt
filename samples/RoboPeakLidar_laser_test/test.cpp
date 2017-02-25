/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers/CRoboPeakLidar.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/system/threads.h> // sleep
#include <mrpt/system/os.h>
#include <iostream>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace std;


string SERIAL_NAME;	// Name of the serial port to open

// ------------------------------------------------------
//				Test_RPLIDAR
// ------------------------------------------------------
void Test_RPLIDAR()
{
	CRoboPeakLidar  laser;
	string 			serName;

	if (SERIAL_NAME.empty())
	{
		std::cout << "Enter the serial port name (e.g. COM1, ttyS0, ttyUSB0, ttyACM0): ";
		getline(cin,serName);
	}
	else
	{
		std::cout << "Using serial port: " << SERIAL_NAME << endl;
		serName = SERIAL_NAME;
	}

	// Set the laser serial port:
	laser.setSerialPort( serName );

	// Show GUI preview:
	laser.showPreview(true);

	// Config: Use defaults + selected port ( serial or ethernet )
	printf("Turning laser ON...\n");
	if (laser.turnOn())
		printf("Initialization OK!\n");
	else {
		printf("Initialization failed!\n");
		return;
	}

	cout << "Press any key to stop capturing..." << endl;

	CTicTac     tictac;
	tictac.Tic();

	while (!mrpt::system::os::kbhit())
	{
		bool						thereIsObservation,hardError;
		CObservation2DRangeScan		obs;

		laser.doProcessSimple( thereIsObservation, obs, hardError );

		if (hardError)
			printf("[TEST] Hardware error=true!!\n");

		if (thereIsObservation)
		{
			double FPS = 1.0 / tictac.Tac();

			printf("Scan received: %u ranges, FOV: %.02fdeg, %.03fHz: mid rang=%fm\n",
				(unsigned int)obs.scan.size(),
				RAD2DEG(obs.aperture),
				FPS,
				obs.scan[obs.scan.size()/2]);

			obs.sensorPose = mrpt::poses::CPose3D(0,0,0);

			tictac.Tic();
		}

		mrpt::system::sleep(5);
	};

}

int main(int argc, char **argv)
{
	try
	{
	    if (argc>1)
            SERIAL_NAME = string(argv[1]);

		Test_RPLIDAR();
		return 0;

	} catch (std::exception &e)
	{
		std::cout << "EXCEPCION: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}
}
