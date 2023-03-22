/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/hwdrivers/CTaoboticsIMU.h>

#include <chrono>
#include <iostream>
#include <thread>

std::string serialPort;	 // Name of the serial port to open

// ------------------------------------------------------
//				Test_IMU
// ------------------------------------------------------
void TestIMU()
{
	mrpt::hwdrivers::CTaoboticsIMU imu;

	if (!serialPort.empty())
		std::cout << "Using serial port: " << serialPort << std::endl;

	imu.setSerialPort(serialPort);

	// Load config:
	// imu.loadConfig( CConfigFile( "./config.ini") ,"IMU" );

	std::cout << "Trying to initialize the sensor..." << std::endl;
	imu.initialize();  // This will raise an exception on error
	std::cout << "Initialized OK!" << std::endl;

	while (1)
	{
		imu.doProcess();
	};
}

int main(int argc, char** argv)
{
	try
	{
		if (argc > 1) { serialPort = std::string(argv[1]); }

		TestIMU();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "Error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
}
