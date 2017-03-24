/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers/CHokuyoURG.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/system/threads.h> // sleep
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;


string SERIAL_NAME;	// Name of the serial port to open

// ------------------------------------------------------
//				Test_HOKUYO
// ------------------------------------------------------
void Test_HOKUYO()
{
	CHokuyoURG		laser;

	string 			serName, type;

	string			ip;

	unsigned int	port;

	cout << "Specify the type of the Hokuyo connection, usb or ethernet: ";
	getline(cin,type);

	while ( (mrpt::system::lowerCase(type) != "usb" ) && ( mrpt::system::lowerCase(type) != "ethernet" ) )
	{
		cout << "Incorrect type" << endl;
		cout << "Specify the type of the Hokuyo connection, usb or ethernet: ";
		getline(cin,type);
	}

	cout << endl << endl << "HOKUYO laser range finder test application." << endl << endl;

	if ( mrpt::system::lowerCase(type) == "usb" )
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
		laser.setSerialPort( serName );

	}
	else
	{
		cout << "Enter the ip direction: ";
		getline(cin,ip);

		cout << "Enter the port number: ";
		cin >> port;

		// Set the laser serial port:
		laser.setIPandPort( ip, port );

	}
	string intensity;
	cout << endl << endl << "Enable intensity [y/n]:";
	getline(cin,intensity);
	laser.setIntensityMode(mrpt::system::lowerCase(intensity) == "y");

	// Config: Use defaults + selected port ( serial or ethernet )

	printf("[TEST] Turning laser ON...\n");
	if (laser.turnOn())
		printf("[TEST] Initialization OK!\n");
	else
	{
		printf("[TEST] Initialization failed!\n");
		return;
	}

#if MRPT_HAS_WXWIDGETS
	CDisplayWindowPlots		win("Laser scans");
#endif

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

			if(obs.hasIntensity())
			{
				size_t i;
				std::cout << "[ ";
				for(i = 0; i < obs.intensity.size()-1; i++)
				{
					std::cout << obs.intensity[i] << ",\t";
					if(i%10==9) std::cout << std::endl;
				}
				std::cout << obs.intensity[i] << " ]" << std::endl;
			}

			obs.sensorPose = CPose3D(0,0,0);

			mrpt::maps::CSimplePointsMap		theMap;
			theMap.insertionOptions.minDistBetweenLaserPoints	= 0;
			theMap.insertObservation( &obs );
			//map.save2D_to_text_file("_out_scan.txt");

			/*
			COpenGLScene			scene3D;
			opengl::CPointCloudPtr	points = opengl::CPointCloud::Create();
			points->loadFromPointsMap(&map);
			scene3D.insert(points);
			CFileStream("_out_point_cloud.3Dscene",fomWrite) << scene3D;
			*/

#if MRPT_HAS_WXWIDGETS
			std::vector<float>	xs,ys,zs;
			theMap.getAllPoints(xs,ys,zs);
			win.plot(xs,ys,".b3");
			win.axis_equal();
#endif

            tictac.Tic();
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

		Test_HOKUYO();
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
