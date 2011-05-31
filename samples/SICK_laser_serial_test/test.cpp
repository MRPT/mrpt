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

#include <mrpt/slam.h>
#include <mrpt/hwdrivers/CSickLaserSerial.h>
#include <mrpt/gui.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::hwdrivers;
using namespace std;


string SERIAL_NAME;	// Name of the serial port to open

// ------------------------------------------------------
//				Test_PLS
// ------------------------------------------------------
void TestPLS()
{
	CSickLaserSerial	laser;

	cout << "SICK LMS thru serial port test application." << endl << endl;

	if (SERIAL_NAME.empty())
	{
        cout << "Enter the serial port name (e.g. COM1, ttyS0, ttyUSB0, ttyACM0): ";
        getline(cin,SERIAL_NAME);
	}
	else
	{
        cout << "Using serial port: " << SERIAL_NAME << endl;
	}

	laser.setSerialPort(SERIAL_NAME);

#if 1
	laser.setBaudRate(500000);
	//laser.setBaudRate(38400);
	laser.setScanFOV(180);
	laser.setScanResolution(50);  // 25=0.25deg, 50=0.5deg, 100=1deg
	//laser.setMillimeterMode(true);
#endif


#if MRPT_HAS_WXWIDGETS
	CDisplayWindowPlots		win("Laser scans");
#endif

	// Load config:
	//laser.loadConfig( CConfigFile( "./LASER_SCAN_TEST.ini") ,"PLS#1" );

	cout << "Trying to initialize the laser..." << endl;
	laser.initialize(); // This will raise an exception on error
	cout << "Initialized OK!" << endl;

	while (!mrpt::system::os::kbhit())
	{
		bool						thereIsObservation,hardError;
		CObservation2DRangeScan		obs;

		try
		{
			laser.doProcessSimple( thereIsObservation, obs, hardError );
		}
		catch (std::exception &e)
		{
			cerr << e.what() << endl;
			hardError = true;
		}

		if (hardError)
			printf("[TEST] Hardware error=true!!\n");

		if (thereIsObservation)
		{
			printf("[TEST] Observation received (%u ranges over %.02fdeg, mid=%.03f)!!\n",
				(unsigned int)obs.scan.size(),
				RAD2DEG(obs.aperture),
				obs.scan[obs.scan.size()/2]);

			obs.sensorPose = CPose3D(0,0,0);
			mrpt::slam::CSimplePointsMap		theMap;
			theMap.insertionOptions.minDistBetweenLaserPoints	= 0;
			theMap.insertObservation( &obs );

#if MRPT_HAS_WXWIDGETS
			vector_float	xs,ys,zs;
			theMap.getAllPoints(xs,ys,zs);
			win.plot(xs,ys,".b3");
			win.axis_equal();
#endif
		}
		mrpt::system::sleep(15);
	};

	laser.turnOff();
}

int main(int argc, char **argv)
{
	try
	{
	    if (argc>1)
        {
            SERIAL_NAME = string(argv[1]);
        }

		TestPLS();
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

