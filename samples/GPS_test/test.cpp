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
#include <mrpt/hwdrivers/CGPSInterface.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;
using namespace std;

string SERIAL_NAME;	// Name of the serial port to open

// ------------------------------------------------------
//				Test_GPS
// ------------------------------------------------------
void Test_GPS()
{
	CGPSInterface		gps;

	string 			serName;
	cout << "GPS test application." << endl << endl;

	if (mrpt::system::fileExists("./CONFIG_gps.ini"))
	{
		cout << "Using configuration from './CONFIG_gps.ini'" << endl;
		CConfigFile			iniFile("./CONFIG_gps.ini");
		gps.loadConfig( iniFile,"GPS");
	}
	else
	{
		if (SERIAL_NAME.empty())
		{
		    cout << "Enter the serial port name (e.g. COM1, ttyS0, ttyUSB0, ttyACM0): ";
		    getline(cin,serName);
		}
		else
		{
		    cout << "Using serial port: " << SERIAL_NAME << endl;
			serName = SERIAL_NAME;
		}

		// Set the laser serial port:
		gps.setSerialPortName(serName);
	}


	FILE	*f= os::fopen("gps_log.txt","wt");
	if (!f) return;

//	bool					thereisData;
//	mrpt::slam::CObservationGPS	gpsData;

	CGenericSensor::TListObservations			lstObs;
	CGenericSensor::TListObservations::iterator 	itObs;

	while (! mrpt::system::os::kbhit())
	{
		gps.doProcess();
		mrpt::system::sleep( 500 );

		gps.getObservations( lstObs );

		if (lstObs.empty())
		{
			printf("[Test_GPS] Waiting for data...\n");
		}
		else
		{
			for (itObs=lstObs.begin();itObs!=lstObs.end();itObs++)
			{
				ASSERT_(itObs->second->GetRuntimeClass()==CLASS_ID(CObservationGPS));

				CObservationGPSPtr gpsData=CObservationGPSPtr(itObs->second);
				gpsData->dumpToConsole();
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
