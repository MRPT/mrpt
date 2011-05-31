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

#include <mrpt/hwdrivers/CBoardENoses.h>
#include <mrpt/base.h>
#include <mrpt/slam.h>


using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace std;

int main()
{
	try
	{
		CBoardENoses			eNoses;
		std::string				firmVers;
		CObservationGasSensors	obs;
		FILE					*f_log = os::fopen("./log.txt","wt");
		TTimeStamp				timStart = mrpt::system::getCurrentTime();
		
		
		// Load configuration:	
		if (mrpt::system::fileExists("./CONFIG_eNoses.ini"))
		{
			cout << "Using configuration from './CONFIG_eNoses.ini'" << endl;
			CConfigFile		conf("./CONFIG_eNoses.ini");
			eNoses.loadConfig( conf, "eNoses" );
		}
		else
		{
			cout << "Configuration file (ini) cannot be found" << endl;
			return -1;
		}

		ASSERT_( mrpt::system::fileExists("CONFIG_eNoses.ini") );
		CConfigFile conf("./CONFIG_eNoses.ini");
		eNoses.loadConfig( conf, "eNoses" );

		
		/*if (!eNoses.queryFirmwareVersion( firmVers ) )
		{
			printf("Error!!\n");
			return -1;
		}
		else
			std::cout << "FIRMWARE VERSION: " << firmVers << std::endl;
		*/

		while ( !mrpt::system::os::kbhit() )
		{
			if (! eNoses.getObservation( obs ) )
			{
				cout << "- Could not retrieve an observation from the eNoses..." << endl;
				mrpt::system::sleep(25);
			}
			else
			{

				cout << obs.m_readings.size() << " eNoses:" << endl;

				if (f_log) fprintf(f_log,"%f ", mrpt::system::timeDifference(timStart,obs.timestamp) );

				for (size_t i=0;i<obs.m_readings.size();i++)
				{
					//E-Nose Pose
					printf("#%u (%.02f,%.02f,%.02f): ",(unsigned int)i,obs.m_readings[i].eNosePoseOnTheRobot.x,obs.m_readings[i].eNosePoseOnTheRobot.y,obs.m_readings[i].eNosePoseOnTheRobot.z);
					
					//E-Nose Sensor's Data
					for (size_t j=0;j<obs.m_readings[i].sensorTypes.size();j++)
					{
						//printf("%04X: %.03fV ", obs.m_readings[i].sensorTypes[j], obs.m_readings[i].readingsVoltage[j] );

						if ( j<(obs.m_readings[i].sensorTypes.size() -1) ){
							if( (obs.m_readings[i].sensorTypes[j]==obs.m_readings[i].sensorTypes[j+1]) ){
								printf("\n%04X: %.03fV \n ", obs.m_readings[i].sensorTypes[j], obs.m_readings[i].readingsVoltage[j]-obs.m_readings[i].readingsVoltage[j+1] );
								j++;	//skip the next sensor as it has been used already.
							}else{
								printf("\n%04X: %.03fV \n ", obs.m_readings[i].sensorTypes[j], obs.m_readings[i].readingsVoltage[j] );
							}
						}else{
								printf("\n%04X: %.03fV \n ", obs.m_readings[i].sensorTypes[j], obs.m_readings[i].readingsVoltage[j] );
						}

						if (f_log) fprintf(f_log,"%f ",obs.m_readings[i].readingsVoltage[j]);
					}

					printf("\nTemp: ");
					if (obs.m_readings[i].hasTemperature)
							printf("%.04f C",obs.m_readings[i].temperature);
					else	printf("NO");

					printf("\n");
					printf("-----------------------------------------\n");
				}
				if (f_log) fprintf(f_log,"\n");
			
				mrpt::system::sleep(5);
			}

		}

		if (f_log) os::fclose(f_log);
	}
	catch(std::exception &e)
	{
		cerr << e.what() << endl;
		return -1;
	}

	return 0;
}
