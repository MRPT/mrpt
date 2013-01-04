/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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

#include <mrpt/hwdrivers/CBoardSonars.h>
#include <mrpt/base.h>
#include <mrpt/gui.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace std;

int main()
{
	try
	{
		CBoardSonars		sonarBoard;
		CObservationRange	obs;
		std::string			firmVers;
		CTicTac				tictac;

		CDisplayWindow3D	wind("Sonar representation");
		COpenGLScenePtr		&scene = wind.get3DSceneAndLock();

		scene->insert( mrpt::opengl::CGridPlaneXY::Create( -20,20,-20,20,0,1 ) );
		scene->insert( mrpt::opengl::stock_objects::RobotPioneer() );
		//scene->insert( mrpt::opengl::CCylinder::Create(1, 1, 2.0f) );
		wind.unlockAccess3DScene();

		// Load configuration:
		ASSERT_( mrpt::system::fileExists("CONFIG_sonars.ini") );
		CConfigFile conf("CONFIG_sonars.ini");
		sonarBoard.loadConfig( conf, "BOARD_SONAR_CONFIG");

		while ( !mrpt::system::os::kbhit() )
		{
			if (!sonarBoard.queryFirmwareVersion( firmVers ) )
			{
				cout << "Cannot connect to USB device... Retrying in 1 sec" << endl;
				mrpt::system::sleep(1000);
			}
			else
			{
				cout << "FIRMWARE VERSION: " << firmVers << endl;
				break;
			}
		}

		cout << "Select operation:" << endl;
		cout << " 1. Get measures from device" << endl;
		cout << " 2. Program a new I2C address to a single sonar" << endl;
		cout << "?";

		char c = os::getch();
		if (c=='1')
		{
			while ( !mrpt::system::os::kbhit() )
			{
				tictac.Tic();
				if (sonarBoard.getObservation( obs ))
				{
					double T = tictac.Tac();
					mrpt::system::clearConsole();

					printf("RX: %u ranges in %.03fms\n",(unsigned int)obs.sensedData.size(), T*1000);
					scene = wind.get3DSceneAndLock();
					for (size_t i=0;i<obs.sensedData.size();i++)
					{
						printf("[ID:%i]=%15f   0x%04X\n",obs.sensedData[i].sensorID,obs.sensedData[i].sensedDistance, (int)(100*obs.sensedData[i].sensedDistance) );

						// Show the distances
						std::string obj = format("sonar%i",obs.sensedData[i].sensorID);
						mrpt::opengl::CCylinderPtr sonarRange;
						mrpt::opengl::CRenderizablePtr objPtr = scene->getByName( obj );
						if( !objPtr )
						{
							sonarRange = mrpt::opengl::CCylinder::Create(0.0f,0.0f,1.0f,30,10);
							sonarRange->setName( obj );
							scene->insert( sonarRange );
						}
						else
							sonarRange = CCylinderPtr( objPtr );

						sonarRange->setRadii( 0, tan( obs.sensorConeApperture )*obs.sensedData[i].sensedDistance );
						sonarRange->setPose( CPose3D(obs.sensedData[i].sensorPose)+CPose3D(0,0,0,0,DEG2RAD(90.0),0) );
						sonarRange->setHeight( obs.sensedData[i].sensedDistance );
						sonarRange->enableShowName();
						sonarRange->setColor( 0, 0, 1, 0.25 );

					}
					wind.unlockAccess3DScene();
					wind.repaint();
				}
				else
				{
					cerr << "Error rx..." << endl;
					//return -1;
				}

				mrpt::system::sleep(200);
			}
		}
		else
		if (c=='2')
		{
			int		curAddr,newAddr;
			cout << "Enter current address: (decimal, 0 to 15)" << endl;
			if (1==scanf("%i",&curAddr))
			{
				cout << "Enter new address: (decimal, 0 to 15)" << endl;
				if (1==scanf("%i",&newAddr))
				{
					ASSERT_(curAddr>=0 && curAddr<16);
					ASSERT_(newAddr>=0 && newAddr<16);
					printf("Changing address %i --> %i... ",curAddr,newAddr);

					if (sonarBoard.programI2CAddress(curAddr,newAddr) )
							printf(" DONE!\n");
					else	printf(" ERROR!\n");
				}
			}
		}
	}
	catch(std::exception &e)
	{
		cerr << e.what() << endl;
		return -1;
	}

	return 0;
}

