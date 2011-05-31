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
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/hwdrivers/CSickLaserUSB.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::hwdrivers;
using namespace std;

// ------------------------------------------------------
//				Test_PLS
// ------------------------------------------------------
void TestPLS()
{
	CSickLaserUSB	laser;

	// Load config:
	laser.loadConfig( CConfigFile( "./LASER_SCAN_TEST.ini") ,"PLS#1" );

	laser.setDeviceSerialNumber("LASER003");

	printf("[TEST] Turning laser ON...\n");
	if (laser.turnOn())
		printf("[TEST] Initialization OK!\n");
	else
	{
		printf("[TEST] Initialization failed!\n");
		return;
	}

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

			mrpt::slam::CSimplePointsMap		map;
			map.insertionOptions.minDistBetweenLaserPoints	= 0;
			map.insertObservation( &obs );
			map.save2D_to_text_file("_out_scan.txt");

/*			COpenGLScene			scene3D;
			opengl::CPointCloudPtr points = opengl::CPointCloud::Create();
			points->loadFromPointsMap(&map);
			scene3D.insert(points);
			CFileOutputStream("_out_point_cloud.3Dscene") << scene3D;
*/
		}

		mrpt::system::sleep(10);
	};


	laser.turnOff();

}

int main()
{
	try
	{
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

