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

#include <mrpt/hwdrivers/CHokuyoURG.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/gui.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::slam;
using namespace mrpt::gui;
using namespace mrpt::utils;
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

	while ( (lowerCase(type) != "usb" ) && ( lowerCase(type) != "ethernet" ) )
	{
		cout << "Incorrect type" << endl;
		cout << "Specify the type of the Hokuyo connection, usb or ethernet: ";
		getline(cin,type);
	}

	cout << endl << endl << "HOKUYO laser range finder test application." << endl << endl;

	if ( lowerCase(type) == "usb" )
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

			obs.sensorPose = CPose3D(0,0,0);

			mrpt::slam::CSimplePointsMap		theMap;
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
			vector_float	xs,ys,zs;
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
