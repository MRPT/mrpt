/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers/CBoardSonars.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/system/os.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::poses;
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
						sonarRange->setPose( mrpt::poses::CPose3D(obs.sensedData[i].sensorPose)+CPose3D(0,0,0,0,DEG2RAD(90.0),0) );
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

