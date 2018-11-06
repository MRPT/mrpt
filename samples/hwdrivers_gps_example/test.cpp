/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers/CGPSInterface.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <thread>
#include <iostream>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;
using namespace mrpt::config;
using namespace std;

string SERIAL_NAME;  // Name of the serial port to open

// ------------------------------------------------------
//				Test_GPS
// ------------------------------------------------------
void Test_GPS()
{
	CGPSInterface gps;

	string serName;
	cout << "GPS test application." << endl << endl;

	if (mrpt::system::fileExists("./CONFIG_gps.ini"))
	{
		cout << "Using configuration from './CONFIG_gps.ini'" << endl;
		CConfigFile iniFile("./CONFIG_gps.ini");
		gps.loadConfig(iniFile, "GPS");
	}
	else
	{
		if (SERIAL_NAME.empty())
		{
			cout << "Enter the serial port name (e.g. COM1, ttyS0, ttyUSB0, "
					"ttyACM0): ";
			getline(cin, serName);
		}
		else
		{
			cout << "Using serial port: " << SERIAL_NAME << endl;
			serName = SERIAL_NAME;
		}

		// Set the laser serial port:
		gps.setSerialPortName(serName);
	}

	FILE* f = os::fopen("gps_log.txt", "wt");
	if (!f) return;

	//	bool					thereisData;
	//	mrpt::obs::CObservationGPS	gpsData;

	CGenericSensor::TListObservations lstObs;
	CGenericSensor::TListObservations::iterator itObs;

	while (!mrpt::system::os::kbhit())
	{
		gps.doProcess();
		std::this_thread::sleep_for(500ms);

		gps.getObservations(lstObs);

		if (lstObs.empty())
		{
			printf("[Test_GPS] Waiting for data...\n");
		}
		else
		{
			for (itObs = lstObs.begin(); itObs != lstObs.end(); itObs++)
			{
				ASSERT_(
					itObs->second->GetRuntimeClass() ==
					CLASS_ID(CObservationGPS));

				CObservationGPS::Ptr gpsData =
					std::dynamic_pointer_cast<CObservationGPS>(itObs->second);
				gpsData->dumpToConsole(std::cout);
			}
			lstObs.clear();
		}
	}

	os::fclose(f);
}

int main()
{
	try
	{
		Test_GPS();
		return 0;
	}
	catch (const std::exception& e)
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
