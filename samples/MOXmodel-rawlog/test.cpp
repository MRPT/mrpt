/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/obs/CObservationGasSensors.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/system/filesystem.h>

using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace std;

int main(int argc, char **argv)
{
	//Variables
	string													rawlog_file,sensorLabel;
	//float													a_rise,a_decay,; //b_decay, b_rise
	int														enoseID,sensorType,indexMonitoredSensor,delay_value; // decimate_value, winNoise_size
	mrpt::obs::CObservationGasSensors::CMOSmodel		MOSmodel;
	bool													have_estimation,apply_delay;

	// Load configuration:
	if (mrpt::system::fileExists("./CONFIG_MOXmodel.ini"))
	{
		cout << "Using configuration from './CONFIG_MOXmodel.ini'" << endl;
		CConfigFile		conf("./CONFIG_MOXmodel.ini");

		rawlog_file = conf.read_string("","rawlog_file","",true);
		sensorLabel = conf.read_string("","sensorLabel","Full_MCEnose",true);
		enoseID = conf.read_int("","enoseID",0,true);
		std::string sensorType_str = conf.read_string("","sensorType","-1",true);
		stringstream convert ( sensorType_str );
		convert>> std::hex >> sensorType;

		//Delays
		apply_delay = conf.read_bool("","apply_delay",false,true);
		delay_value = conf.read_int("","delay_value",0,true);


		//MOX model parameters
		MOSmodel.a_rise = conf.read_float("","a_rise",0.0,true);
		MOSmodel.b_rise = conf.read_float("","b_rise",0.0,true);
		MOSmodel.a_decay = conf.read_float("","a_decay",0.0,true);
		MOSmodel.b_decay = conf.read_float("","b_decay",0.0,true);
		MOSmodel.winNoise_size = conf.read_int("","winNoise_size",0,true);
		MOSmodel.decimate_value = conf.read_int("","decimate_value",0,true);
		//save_maplog = conf.read_bool("","save_maplog",false,true);

		indexMonitoredSensor = -1;

	}
	else
	{
		cout << "Configuration file (ini) cannot be found\n" << endl;
		//If you are in VisualStudio, assure that the working directory in the project properties is correctly set
		return -1;
	}


	//Open Rawlogs
	cout << "Processing Rawlog " << rawlog_file << endl;
	cout << "Obtaining MOXmodel from " << sensorLabel << "(" << enoseID << ") - sensor " << sensorType << endl;
	CFileGZInputStream                      file_input;
	CFileGZOutputStream						file_output;

	file_input.open( rawlog_file );
	file_output.open("MOX_model_output.rawlog");

	if ( !file_input.fileOpenCorrectly() || !file_output.fileOpenCorrectly())
           cout << "Error opening rawlog file" << endl;



	//Process rawlog
	bool read = true;
	while ( read )
	{
	   try
	   {
		   CObservationPtr o;
		   file_input >> o;

		   if ( o ) //ASSERT_(o);
		   {
			   if (IS_CLASS(o,CObservationGasSensors))
			   {
				   CObservationGasSensorsPtr obs = CObservationGasSensorsPtr( o );

				   //Correct delay on gas readings
				   if ( apply_delay )
					   obs->timestamp = obs->timestamp - delay_value*10000000;


				   if (obs->sensorLabel == sensorLabel)
				   {
						//------------------------------------------------------
						// Get reading from CH_i for gas distribution estimation
						//------------------------------------------------------
						float raw_reading;

						if (sensorType == 0){	//compute the mean
							raw_reading = math::mean( obs->m_readings[enoseID].readingsVoltage );
						}
						else
						{
							// Get the reading of the specified sensorID
							if (indexMonitoredSensor == -1)
							{
								//First reading, get the index according to sensorID
								for (indexMonitoredSensor=0; indexMonitoredSensor< (int)obs->m_readings[enoseID].sensorTypes.size(); indexMonitoredSensor++)
								{
									if (obs->m_readings[enoseID].sensorTypes.at(indexMonitoredSensor) == vector_int::value_type(sensorType) )
										break;
								}
							}

							if (indexMonitoredSensor< (int)obs->m_readings[enoseID].sensorTypes.size())
							{
								raw_reading = obs->m_readings[enoseID].readingsVoltage.at(indexMonitoredSensor);
							}
							else //Sensor especified not found, compute default mean value
							{
								cout << "sensorType not found. Computing the mean value" << endl;
								raw_reading = math::mean( obs->m_readings[enoseID].readingsVoltage );
							}
						}

						//Obtain MOX model output
						mrpt::poses::CPose3D MOXmodel_pose = obs->m_readings[enoseID].eNosePoseOnTheRobot;
						float MOXmodel_estimation = raw_reading;
						mrpt::system::TTimeStamp MOXmodel_timestamp = obs->timestamp;

						have_estimation =  MOSmodel.get_GasDistribution_estimation(MOXmodel_estimation, MOXmodel_timestamp);

						if (have_estimation)
						{
							//Save as new obs
							mrpt::obs::CObservationGasSensors::TObservationENose gd_est;
							gd_est.hasTemperature = false;
							gd_est.temperature = 0.0;
							gd_est.isActive = false;
							gd_est.sensorTypes.push_back(0x0001);	//indicates that is a MOXmodel output
							gd_est.readingsVoltage.push_back(MOXmodel_estimation);
							gd_est.eNosePoseOnTheRobot = MOXmodel_pose;

							mrpt::obs::CObservationGasSensorsPtr obs_GDM = CObservationGasSensors::Create();
							obs_GDM->sensorLabel = "GDM";
							// modify timestamp to deal with the delay of the model
							obs_GDM->timestamp = MOXmodel_timestamp;
							obs_GDM->m_readings.push_back( gd_est );

							file_output << obs_GDM;
						}
				   }
			   }

			   //Save current sensor obs to the new Rawlog file
			   file_output << o;

		   }


	   }
	   catch( exception &e )
	   {
		   cout << "Exception: " << e.what() << endl;
		   file_input.close();
		   file_output.close();
		   read = false;
	   }
	}

	return 0;
}
