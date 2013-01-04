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

#include <mrpt/hwdrivers/CBoardENoses.h>
#include <mrpt/base.h>
#include <mrpt/obs.h>


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
