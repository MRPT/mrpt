/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/system/threads.h>
#include <mrpt/hwdrivers/CIMUIntersense.h>
#include <mrpt/obs/CObservationIMU.h>

IMPLEMENTS_GENERIC_SENSOR(CIMUIntersense,mrpt::hwdrivers)

using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::hwdrivers;
using namespace std;

#if MRPT_HAS_INTERSENSE
	#include "isense/isense.h"
#endif

// Adaptors for the "void*" memory blocks:
#define isense_handles  (static_cast<ISD_TRACKER_HANDLE*>(m_handles_ptr))

// Include libraries in linking:
#if 0
#if MRPT_HAS_INTERSENSE
	#ifdef MRPT_OS_WINDOWS
		// WINDOWS:
		#if defined(_MSC_VER) || defined(__BORLANDC__)
			#pragma comment (lib,"isense.dll")
		#endif
	#endif	// MRPT_OS_WINDOWS
#endif // MRPT_HAS_INTERSENSE
#endif
/*-------------------------------------------------------------
					CIMUIntersense
-------------------------------------------------------------*/
CIMUIntersense::CIMUIntersense( ) :
	m_handles_ptr	(NULL),
	m_timeStartUI	(0),
	m_timeStartTT	(0),
	m_sensorPose    (),
	m_nSensors		(0),
	m_sensitivity	(10),
	m_enhancement	(2),
	m_prediction	(0),
	m_useBuffer		(false),
	m_toutCounter	(0)
{
	m_sensorLabel = "isenseIMU";

#if MRPT_HAS_INTERSENSE
	m_handles_ptr = new ISD_TRACKER_HANDLE[ISD_MAX_TRACKERS](); // initialized to zeros
#else
	THROW_EXCEPTION("MRPT has been compiled with 'BUILD_INTERSENSE'=OFF, so this class cannot be used.");
#endif

}

/*-------------------------------------------------------------
					~CIMUIntersense
-------------------------------------------------------------*/
CIMUIntersense::~CIMUIntersense()
{
#if MRPT_HAS_INTERSENSE
	ISD_CloseTracker(0);	// close all the sensors
	delete[] isense_handles; m_handles_ptr = NULL;
#endif
}

/*-------------------------------------------------------------
					doProcess
-------------------------------------------------------------*/
void CIMUIntersense::doProcess()
{
#if MRPT_HAS_INTERSENSE
	if(m_state == ssError)
	{
		mrpt::system::sleep(200);
		initialize();
	}

	if(m_state == ssError)
		return;
	
	int n_data_ok = 0;
	// add an observation for each sensor
	for(int i = 0; i < m_nSensors; ++i)
	{
		ASSERT_(isense_handles[i] > 0)

		ISD_TRACKING_DATA_TYPE data;
		Bool res_ok = ISD_GetTrackingData( isense_handles[i], &data );

		if( !res_ok )
			continue;

		// current (sensor) timestamp (in usecs)
		// uint32_t nowUI = data.Station[0].TimeStampSeconds*10e6 + data.Station[0].TimeStampMicroSec; 
		float nowUI = data.Station[0].TimeStamp; // in seconds

		CObservationIMUPtr obs = CObservationIMU::Create();

		// euler angles
		obs->rawMeasurements[IMU_YAW]	= DEG2RAD(data.Station[0].Euler[0]);
		obs->dataIsPresent[IMU_YAW]		= true;
		obs->rawMeasurements[IMU_PITCH] = DEG2RAD(data.Station[0].Euler[1]);
		obs->dataIsPresent[IMU_PITCH]	= true;
		obs->rawMeasurements[IMU_ROLL]	= DEG2RAD(data.Station[0].Euler[2]);
		obs->dataIsPresent[IMU_ROLL]	= true;

		// angular velocity		
		obs->rawMeasurements[IMU_YAW_VEL]	= data.Station[0].AngularVelBodyFrame[0];	// rad/s
		obs->dataIsPresent[IMU_YAW_VEL]		= true;
		obs->rawMeasurements[IMU_PITCH_VEL] = data.Station[0].AngularVelBodyFrame[1];	// rad/s
		obs->dataIsPresent[IMU_PITCH_VEL]	= true;
		obs->rawMeasurements[IMU_ROLL_VEL]	= data.Station[0].AngularVelBodyFrame[2];	// rad/s
		obs->dataIsPresent[IMU_ROLL_VEL]	= true;

		// angular velocity	2
		obs->rawMeasurements[IMU_YAW_VEL_GLOBAL]	= data.Station[0].AngularVelNavFrame[0];	// rad/s
		obs->dataIsPresent[IMU_YAW_VEL_GLOBAL]	= true;
		obs->rawMeasurements[IMU_PITCH_VEL_GLOBAL]	= data.Station[0].AngularVelNavFrame[1];	// rad/s
		obs->dataIsPresent[IMU_PITCH_VEL_GLOBAL]	= true;
		obs->rawMeasurements[IMU_ROLL_VEL_GLOBAL]	= data.Station[0].AngularVelNavFrame[2];	// rad/s
		obs->dataIsPresent[IMU_ROLL_VEL_GLOBAL]	= true;

		// angular velocity 3 --> x,y,z velocity
		obs->rawMeasurements[IMU_X_VEL]	= data.Station[0].VelocityNavFrame[0];	// m/s
		obs->dataIsPresent[IMU_X_VEL]	= true;
		obs->rawMeasurements[IMU_Y_VEL]	= data.Station[0].VelocityNavFrame[1];	// m/s
		obs->dataIsPresent[IMU_Y_VEL]	= true;
		obs->rawMeasurements[IMU_Z_VEL]	= data.Station[0].VelocityNavFrame[2];	// m/s
		obs->dataIsPresent[IMU_Z_VEL]	= true;

		// angular acceleration: global coords
		obs->rawMeasurements[IMU_X_ACC_GLOBAL]		= data.Station[0].AccelNavFrame[0];	// m/s w/o gravity
		obs->dataIsPresent[IMU_X_ACC_GLOBAL]		= true;
		obs->rawMeasurements[IMU_Y_ACC_GLOBAL]		= data.Station[0].AccelNavFrame[1];	// m/s w/o gravity
		obs->dataIsPresent[IMU_Y_ACC_GLOBAL]		= true;
		obs->rawMeasurements[IMU_Z_ACC_GLOBAL]		= data.Station[0].AccelNavFrame[2];	// m/s w/o gravity
		obs->dataIsPresent[IMU_Z_ACC_GLOBAL]		= true;

		// angular acceleration 2: local coords
		obs->rawMeasurements[IMU_X_ACC]		= data.Station[0].AccelBodyFrame[0];
		obs->dataIsPresent[IMU_X_ACC]		= true;
		obs->rawMeasurements[IMU_Y_ACC]		= data.Station[0].AccelBodyFrame[1];
		obs->dataIsPresent[IMU_Y_ACC]		= true;
		obs->rawMeasurements[IMU_Z_ACC]		= data.Station[0].AccelBodyFrame[2];
		obs->dataIsPresent[IMU_Z_ACC]		= true;

		// position
		//obs->rawMeasurements[IMU_X]		= DEG2RAD(data.Station[0].Position[0]);
		//obs->dataIsPresent[IMU_X]		= true;
		//obs->rawMeasurements[IMU_Y]		= DEG2RAD(data.Station[0].Position[1]);
		//obs->dataIsPresent[IMU_Y]		= true;
		//obs->rawMeasurements[IMU_Z]		= DEG2RAD(data.Station[0].Position[2]);
		//obs->dataIsPresent[IMU_Z]		= true;

		// timestamp
		// uint32_t AtUI = 0;
		float AtUI = 0;
		if( m_timeStartUI == 0 )
		{
			m_timeStartUI = nowUI;
			m_timeStartTT = mrpt::system::now();
		}
		else
			AtUI	= nowUI - m_timeStartUI;

		//mrpt::system::TTimeStamp AtDO = mrpt::system::secondsToTimestamp( AtUI * 1e-6 /* Board time is usec */ );
		mrpt::system::TTimeStamp AtDO = mrpt::system::secondsToTimestamp( AtUI /* Board time is sec */ );
		obs->timestamp		= m_timeStartTT	+ AtDO;

		// other stuff
		obs->sensorPose		= m_sensorPose;
		obs->sensorLabel	= m_sensorLabel + "_" + std::to_string(i);

		// add observation
		appendObservation(obs);
		m_toutCounter = 0;
		n_data_ok++;
	} // end-for

	if( n_data_ok == 0) // none of the sensors yielded data
		m_toutCounter++;

	if( m_toutCounter > 3 )
	{
		m_toutCounter	= 0;
		m_state			= ssError;
		
		ISD_CloseTracker(0);	// close all the sensors
	}
#else
	THROW_EXCEPTION("MRPT has been compiled with 'BUILD_INTERSENSE'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
					initialize
-------------------------------------------------------------*/
void CIMUIntersense::initialize()
{
#if MRPT_HAS_INTERSENSE
	// ISD_TRACKER_HANDLE              Trackers[ISD_MAX_TRACKERS];
	DWORD openSuccess = FALSE;
	cout << "Opening trackers... ";
	openSuccess = ISD_OpenAllTrackers( (Hwnd)NULL, isense_handles, FALSE, TRUE );
	if( openSuccess < 1 )
		cout << "ERROR" << endl;
	else
	{
		cout << "DONE" << endl;

		WORD numOpenTrackers = 0;
		ISD_NumOpenTrackers(&numOpenTrackers);
		cout << "Number of opened trackers: " << numOpenTrackers << endl;
		vector<ISD_STATION_INFO_TYPE> station_info(numOpenTrackers);
		for(int i = 0; i < ISD_MAX_TRACKERS; ++i)
		{
			if( isense_handles[i] > 0 )
			{
				cout << "Retrieving configuration from sensor " << i << "...";
				// get current configuration
				// ISD_STATION_INFO_TYPE station;
				Bool res_ok = ISD_GetStationConfig( 
					isense_handles[i], 
					& station_info[i], // & station, 
					i+1 /*from 1 to ISD_MAX_TRACKERS*/,
					FALSE );

				if( !res_ok )
				{
					cout << " ERROR" << endl;
					cout << "Sensor " << i << " is working with default configuration!" << endl;
				}
				else
				{
					cout << " DONE" << endl;
					// set custom configuration ...
					//station.Sensitivity = m_sensitivity;
					//station.Enhancement = m_enhancement;
					//station.Prediction = m_prediction;
					//station.TimeStamped = TRUE;	// this must be TRUE to get timestamped data

					station_info[i].Sensitivity = m_sensitivity;
					station_info[i].Enhancement = m_enhancement;
					station_info[i].Prediction = m_prediction;
					station_info[i].TimeStamped = TRUE;	// this must be TRUE to get timestamped data

					cout << "Setting configuration to sensor " << i << "...";
					// .. and apply sensor configuration
					res_ok = ISD_SetStationConfig( 
						isense_handles[i], 
						& station_info[i], // & station, 
						i+1 /*from 1 to ISD_MAX_TRACKERS*/, 
						FALSE );

					res_ok ? cout << " DONE" << endl : cout << " ERROR" << endl;

#if 0					// set ring buffer to avoid data loss and start it:
					// 180 samples is ~1 second at maximum data rate for wired devices 
					if( station_info[i].State == 1 /*station.State == 1*/ )
					{
						if( false && m_useBuffer )
						{
							ISD_RingBufferSetup(isense_handles[i],i,NULL,180);
							ISD_RingBufferStart(isense_handles[i],i);
						}
						
					} // end-if
#endif
					mrpt::system::sleep(500);
				} // end-else
				m_nSensors++;
			} // end-if
		} // end-for

		cout << "Found (and opened) " << m_nSensors << " sensors." << endl;
		m_state = ssInitializing;
	} // end-else
#else
	THROW_EXCEPTION("MRPT has been compiled with 'BUILD_INTERSENSE'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
					loadConfig_sensorSpecific
-------------------------------------------------------------*/
void  CIMUIntersense::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase	& configSource,
	const std::string					& iniSection )
{
	m_sensorPose.setFromValues (
        configSource.read_float( iniSection, "pose_x", 0, false ),
        configSource.read_float( iniSection, "pose_y", 0, false ),
        configSource.read_float( iniSection, "pose_z", 0, false ),
        DEG2RAD( configSource.read_float( iniSection, "pose_yaw", 0, false ) ),
        DEG2RAD( configSource.read_float( iniSection, "pose_pitch", 0, false ) ),
        DEG2RAD( configSource.read_float( iniSection, "pose_roll", 0, false ) ) 
	);

	m_sensitivity	= configSource.read_int( iniSection, "sensitivity", m_sensitivity, false );
	m_enhancement	= configSource.read_int( iniSection, "enhancement", m_enhancement, false );
	m_prediction	= configSource.read_int( iniSection, "prediction", m_prediction, false );
	m_useBuffer		= configSource.read_bool( iniSection, "useBuffer", m_useBuffer, false );

	// dump parameters to console
	cout << "---------------------------" << endl;
	cout << "Intersense IMU parameters: " << endl;
	cout << "---------------------------" << endl;
	cout << "Sensitivity:	" << m_sensitivity << endl;
	cout << "Enhancement:	" << m_enhancement << endl;
	cout << "Prediction:	" << m_prediction << endl;
	cout << "Use buffer:	" << m_useBuffer << endl;
	cout << m_sensorPose << endl;
	cout << "---------------------------" << endl << endl;
}
