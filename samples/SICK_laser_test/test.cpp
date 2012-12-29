/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>
#include <mrpt/maps.h>
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

