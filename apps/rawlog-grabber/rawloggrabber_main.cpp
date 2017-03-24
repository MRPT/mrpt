/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*-----------------------------------------------------------------------------
	APPLICATION: rawlog-grabber
	FILE: rawloggrabber_main.cpp
	AUTHOR: Jose Luis Blanco Claraco <joseluisblancoc@gmail.com>

	For instructions and details, see:
	 http://www.mrpt.org/list-of-mrpt-apps/application-rawlog-grabber/
  -----------------------------------------------------------------------------*/

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/round.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/system/os.h>
#include <mrpt/system/filesystem.h>

#ifdef RAWLOGGRABBER_PLUGIN
#	include "rawloggrabber_plugin.h"
#endif

using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace std;

const bool hwdrivers_verbose = (getenv("MRPT_HWDRIVERS_VERBOSE")!=NULL) && atoi(getenv("MRPT_HWDRIVERS_VERBOSE"))!=0;

const std::string GLOBAL_SECTION_NAME = "global";

// Forward declarations:
struct TThreadParams
{
	CConfigFile		*cfgFile;
	string			sensor_label;
};

void SensorThread(TThreadParams params);



CGenericSensor::TListObservations		global_list_obs;
synch::CCriticalSection					cs_global_list_obs;

bool									allThreadsMustExit = false;

string 		rawlog_ext_imgs_dir;		// Directory where to save externally stored images, only for CCameraSensor's.

// ------------------------------------------------------
//					MAIN THREAD
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		printf(" rawlog-grabber - Part of the MRPT\n");
		printf(" MRPT C++ Library: %s - Sources timestamp: %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());
		printf("-------------------------------------------------------------------\n");

		// Process arguments:
		if (argc<2)
		{
			printf("Usage: %s <config_file.ini>\n\n",argv[0]);
			mrpt::system::pause();
			return -1;
		}

		string INI_FILENAME( argv[1] );
		ASSERT_FILE_EXISTS_(INI_FILENAME)

		CConfigFile		iniFile( INI_FILENAME );

		// ------------------------------------------
		//			Load config from file:
		// ------------------------------------------
		string			rawlog_prefix = "dataset";
		int				time_between_launches = 300;
		double			SF_max_time_span = 0.25;			// Seconds
		bool			use_sensoryframes = false;
		int				GRABBER_PERIOD_MS = 1000;
		int 			rawlog_GZ_compress_level  = 1;  // 0: No compress, 1-9: compress level

		MRPT_LOAD_CONFIG_VAR( rawlog_prefix, string, iniFile, GLOBAL_SECTION_NAME );
		MRPT_LOAD_CONFIG_VAR( time_between_launches, int, iniFile, GLOBAL_SECTION_NAME );
		MRPT_LOAD_CONFIG_VAR( SF_max_time_span, float,		iniFile, GLOBAL_SECTION_NAME );
		MRPT_LOAD_CONFIG_VAR( use_sensoryframes, bool,		iniFile, GLOBAL_SECTION_NAME );
		MRPT_LOAD_CONFIG_VAR( GRABBER_PERIOD_MS, int, iniFile, GLOBAL_SECTION_NAME );

		MRPT_LOAD_CONFIG_VAR( rawlog_GZ_compress_level, int, iniFile, GLOBAL_SECTION_NAME );

		// Build full rawlog file name:
		string	rawlog_postfix = "_";

		//rawlog_postfix += dateTimeToString( now() );
		mrpt::system::TTimeParts parts;
		mrpt::system::timestampToParts(now(), parts, true);
		rawlog_postfix += format("%04u-%02u-%02u_%02uh%02um%02us",
			(unsigned int)parts.year,
			(unsigned int)parts.month,
			(unsigned int)parts.day,
			(unsigned int)parts.hour,
			(unsigned int)parts.minute,
			(unsigned int)parts.second );

		rawlog_postfix = mrpt::system::fileNameStripInvalidChars( rawlog_postfix );

		// Only set this if we want externally stored images:
		rawlog_ext_imgs_dir = rawlog_prefix+fileNameStripInvalidChars( rawlog_postfix+string("_Images") );

		// Also, set the path in CImage to enable online visualization in a GUI window:
		CImage::IMAGES_PATH_BASE = rawlog_ext_imgs_dir;


		rawlog_postfix += string(".rawlog");
		rawlog_postfix = fileNameStripInvalidChars( rawlog_postfix );

		string			rawlog_filename = rawlog_prefix + rawlog_postfix;

		cout << endl ;
		cout << "Output rawlog filename: " << rawlog_filename << endl;
		cout << "External image storage: " << rawlog_ext_imgs_dir << endl << endl;

		// ----------------------------------------------
		// Launch threads:
		// ----------------------------------------------
		vector_string	sections;
		iniFile.getAllSections( sections );

		vector<TThreadHandle>		lstThreads;

		for (vector_string::iterator it=sections.begin();it!=sections.end();++it)
		{
			if (*it==GLOBAL_SECTION_NAME || it->empty() || iniFile.read_bool(*it,"rawlog-grabber-ignore",false,false) ) 
				continue;	// This is not a sensor:

			TThreadParams	threParms;
			threParms.cfgFile		= &iniFile;
			threParms.sensor_label	= *it;

			TThreadHandle	thre = createThread(SensorThread, threParms);

			lstThreads.push_back(thre);
			sleep(time_between_launches);
		}

		// ----------------------------------------------
		// Run:
		// ----------------------------------------------
		CFileGZOutputStream	out_file;

		out_file.open( rawlog_filename, rawlog_GZ_compress_level );

		CSensoryFrame						curSF;
		CGenericSensor::TListObservations	copy_of_global_list_obs;

		cout << endl << "Press any key to exit program" << endl;
		while (!os::kbhit() && !allThreadsMustExit)
		{
			// See if we have observations and process them:
			{
				synch::CCriticalSectionLocker	lock (&cs_global_list_obs);
				copy_of_global_list_obs.clear();

				if (!global_list_obs.empty())
				{
					CGenericSensor::TListObservations::iterator itEnd = global_list_obs.begin();
					std::advance( itEnd, global_list_obs.size() / 2 );
					copy_of_global_list_obs.insert(global_list_obs.begin(),itEnd );
					global_list_obs.erase(global_list_obs.begin(), itEnd);
				}
			}	// End of cs lock

			if (use_sensoryframes)
			{
				// -----------------------
				// USE SENSORY-FRAMES
				// -----------------------
				for (CGenericSensor::TListObservations::iterator it=copy_of_global_list_obs.begin();it!=copy_of_global_list_obs.end();++it)
				{
					// If we have an action, save the SF and start a new one:
					if (IS_DERIVED(it->second, CAction))
					{
						CActionPtr act = CActionPtr( it->second);

						out_file << curSF;
						cout << "[" << dateTimeToString(now()) << "] Saved SF with " << curSF.size() << " objects." << endl;
						curSF.clear();

						CActionCollection	acts;
						acts.insert(*act);
						act.clear_unique();

						out_file << acts;
					}
					else
					if (IS_CLASS(it->second,CObservationOdometry) )
					{
						CObservationOdometryPtr odom = CObservationOdometryPtr( it->second );

						CActionRobotMovement2DPtr act = CActionRobotMovement2D::Create();
						act->timestamp = odom->timestamp;

						// Compute the increment since the last reading:
						static CActionRobotMovement2D::TMotionModelOptions odomOpts;
						static CObservationOdometry last_odo;
						static bool last_odo_first = true;

						CPose2D  odo_incr;
						int64_t  lticks_incr, rticks_incr;

						if (last_odo_first)
						{
							last_odo_first = false;
							odo_incr = CPose2D(0,0,0);
							lticks_incr = rticks_incr = 0;
						}
						else
						{
							odo_incr = odom->odometry - last_odo.odometry;
							lticks_incr = odom->encoderLeftTicks - last_odo.encoderLeftTicks;
							rticks_incr = odom->encoderRightTicks - last_odo.encoderRightTicks;

							last_odo = *odom;
						}

						// Save as action & dump to file:
						act->computeFromOdometry( odo_incr, odomOpts );

						act->hasEncodersInfo = true;
						act->encoderLeftTicks = lticks_incr;
						act->encoderRightTicks = rticks_incr;

						act->hasVelocities = true;
						act->velocityLocal = odom->velocityLocal;

						out_file << curSF;
						cout << "[" << dateTimeToString(now()) << "] Saved SF with " << curSF.size() << " objects." << endl;
						curSF.clear();

						CActionCollection	acts;
						acts.insert(*act);
						act.clear_unique();

						out_file << acts;
					}
					else
					if (IS_DERIVED(it->second, CObservation) )
					{
						CObservationPtr obs = CObservationPtr(it->second);

						// First, check if inserting this OBS into the SF would overflow "SF_max_time_span":
						if (curSF.size()!=0 && timeDifference( curSF.getObservationByIndex(0)->timestamp, obs->timestamp ) > SF_max_time_span )
						{
							if (hwdrivers_verbose)
							{
								// Show GPS mode:
								CObservationGPSPtr gps;
								size_t idx=0;
								do
								{
									gps = curSF.getObservationByClass<CObservationGPS>(idx++ );
									if (gps && gps->has_GGA_datum) {
										cout << "  GPS mode: " << (int)gps->getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>().fields.fix_quality << " label: " << gps->sensorLabel << endl;
									}
								} while (gps);

								// Show IMU angles:
								CObservationIMUPtr imu = curSF.getObservationByClass<CObservationIMU>();
								if (imu)
								{
									cout << format("   IMU angles (degrees): (yaw,pitch,roll)=(%.06f, %.06f, %.06f)",
										RAD2DEG( imu->rawMeasurements[IMU_YAW] ),
										RAD2DEG( imu->rawMeasurements[IMU_PITCH] ),
										RAD2DEG( imu->rawMeasurements[IMU_ROLL] ) ) << endl;
								}
							}

							// Save and start a new one:
							out_file << curSF;
							cout << "[" << dateTimeToString(now()) << "] Saved SF with " << curSF.size() << " objects." << endl;
							curSF.clear();
						}

						// Now, insert the observation in the SF:
						curSF.insert( obs );
					}
					else THROW_EXCEPTION("*** ERROR *** Class is not an action or an observation");
				}
			}
			else
			{
				// ---------------------------
				//  DO NOT USE SENSORY-FRAMES
				// ---------------------------
				CObservationIMUPtr imu; // Default:NULL

				for (CGenericSensor::TListObservations::iterator it=copy_of_global_list_obs.begin();it!=copy_of_global_list_obs.end();++it)
				{
					out_file << *(it->second);

					// Show GPS mode:
					if (hwdrivers_verbose)
					{
						if ( (it->second)->GetRuntimeClass() == CLASS_ID(CObservationGPS) ) {
							CObservationGPSPtr gps = CObservationGPSPtr( it->second );
							if (gps->has_GGA_datum)
								cout << "  GPS mode: " << (int)gps->getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>().fields.fix_quality << " label: " << gps->sensorLabel << endl;
						}
						else if ( (it->second)->GetRuntimeClass() == CLASS_ID(CObservationIMU) ) {
							imu = CObservationIMUPtr( it->second );
						}
					}
				}

				if (hwdrivers_verbose)
				{
					// Show IMU angles:
					if (imu)
					{
						cout << format("   IMU angles (degrees): (yaw,pitch,roll)=(%.06f, %.06f, %.06f)",
							RAD2DEG( imu->rawMeasurements[IMU_YAW] ),
							RAD2DEG( imu->rawMeasurements[IMU_PITCH] ),
							RAD2DEG( imu->rawMeasurements[IMU_ROLL] ) ) << endl;
					}
				}

				if (!copy_of_global_list_obs.empty()) {
					cout << "[" << dateTimeToString(now()) << "] Saved " << copy_of_global_list_obs.size() << " objects." << endl;
				}
			}
			sleep(GRABBER_PERIOD_MS);
		}

		if (allThreadsMustExit) {
			cerr << "[main thread] Ended due to other thread signal to exit application." << endl;
		}

		// Flush file to disk:
		out_file.close();

		// Wait all threads:
		// ----------------------------
		allThreadsMustExit = true;
		mrpt::system::sleep(300);
		cout << endl << "Waiting for all threads to close..." << endl;
		for (vector<TThreadHandle>::iterator th=lstThreads.begin();th!=lstThreads.end();++th)
			joinThread( *th );

		return 0;
	} catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl << "Program finished for an exception!!" << std::endl;
		mrpt::system::pause();
		return -1;
	}
	catch (...)
	{
		std::cerr << "Untyped exception!!" << std::endl;
		mrpt::system::pause();
		return -1;
	}
}


// ------------------------------------------------------
//				SensorThread
// ------------------------------------------------------
void SensorThread(TThreadParams params)
{
	try
	{
		string driver_name = params.cfgFile->read_string(params.sensor_label,"driver","",true);

		CGenericSensorPtr	sensor = CGenericSensor::createSensorPtr(driver_name );
		if (!sensor)
		{
			cerr << endl << "***ERROR***: Class name not recognized: " << driver_name << endl;
			allThreadsMustExit = true;
		}

		// Load common & sensor specific parameters:
		sensor->loadConfig( *params.cfgFile, params.sensor_label );

		cout << format("[thread_%s] Starting...",params.sensor_label.c_str()) << " at " << sensor->getProcessRate() <<  " Hz" << endl;

		ASSERTMSG_(sensor->getProcessRate()>0,"process_rate must be set to a valid value (>0 Hz).");
		int		process_period_ms = round( 1000.0 / sensor->getProcessRate() );

		// For imaging sensors, set external storage directory:
		sensor->setPathForExternalImages( rawlog_ext_imgs_dir );

		// Init device:
		sensor->initialize();


		while (! allThreadsMustExit )
		{
			TTimeStamp t0= now();

			// Process
			sensor->doProcess();

			// Get new observations
			CGenericSensor::TListObservations	lstObjs;
			sensor->getObservations( lstObjs );

			{
				synch::CCriticalSectionLocker	lock (&cs_global_list_obs);
				global_list_obs.insert( lstObjs.begin(), lstObjs.end() );
			}

			lstObjs.clear();

			// wait until the process period:
			TTimeStamp t1= now();
			double	At = timeDifference(t0,t1);
			int At_rem_ms = process_period_ms - At*1000;
			if (At_rem_ms>0)
				sleep(At_rem_ms);
		}

		sensor.clear();
		cout << format("[thread_%s] Closing...",params.sensor_label.c_str()) << endl;
	}
	catch (std::exception &e)
	{
		cerr << e.what() << endl;
		allThreadsMustExit = true;
	}
	catch (...)
	{
		cerr << "Untyped exception!!" << endl;
		allThreadsMustExit = true;
	}
}
