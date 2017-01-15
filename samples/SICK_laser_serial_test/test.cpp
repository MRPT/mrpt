/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers/CSickLaserSerial.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/system/threads.h> // sleep()
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::hwdrivers;
using namespace mrpt::poses;
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
	//laser.setBaudRate(500000);
	laser.setBaudRate(9600);
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
			mrpt::maps::CSimplePointsMap		theMap;
			theMap.insertionOptions.minDistBetweenLaserPoints	= 0;
			theMap.insertObservation( &obs );

#if MRPT_HAS_WXWIDGETS
			std::vector<float>	xs,ys,zs;
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

