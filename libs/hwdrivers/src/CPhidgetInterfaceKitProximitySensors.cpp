/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/system/os.h>
#include <mrpt/hwdrivers/CPhidgetInterfaceKitProximitySensors.h>
#include <mrpt/utils/CConfigFileBase.h>

#include <algorithm>

#if MRPT_HAS_PHIDGET
	#include <phidget21.h>
#endif

using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace std;

IMPLEMENTS_GENERIC_SENSOR(CPhidgetInterfaceKitProximitySensors,mrpt::hwdrivers)

/* -----------------------------------------------------
                Constructor
   ----------------------------------------------------- */
CPhidgetInterfaceKitProximitySensors::CPhidgetInterfaceKitProximitySensors() :
	mrpt::utils::COutputLogger("CPhidgetInterfaceKitProximitySensors"),
	m_serialNumber(-1)
{
#if MRPT_HAS_PHIDGET
	m_carteInterfaceKit = new CPhidgetInterfaceKitHandle;
	*((CPhidgetInterfaceKitHandle*)m_carteInterfaceKit) = 0;
	m_sensorLabel = "PhidgetInterfaceKit";

	m_sensorIsPlugged.assign(8,false);
	m_minRange.assign(8, 0.1f);
	m_maxRange.assign(8, 0.8f);
	m_sensorPoses.resize(8);
	m_sensorType.assign(8,UNPLUGGED);

#else
	THROW_EXCEPTION("MRPT Was compiled without the CPhidget support. Recompile MRPT to use this class")
#endif
}

/* -----------------------------------------------------
                loadConfig_sensorSpecific
   ----------------------------------------------------- */
void  CPhidgetInterfaceKitProximitySensors::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string	  &iniSection )
{
#if MRPT_HAS_PHIDGET
	if(!configSource.sectionExists(iniSection)) THROW_EXCEPTION("Can't find section in configuration file");
	// looking for the board parameters.
	// process_rate = 100 						// Hz (common to all sensors)
	// serialNumber = 12345						// The interface kit serial number.
	m_process_rate = configSource.read_int(iniSection, string("process_rate"), 50);
	m_serialNumber = configSource.read_int(iniSection, string("serialNumber"), -1);
	bool display = configSource.read_bool(iniSection, string("displayRecapitulativeInformations"), false);

	// Looking for each sensor.

	for(int i = 1 ; i <= 8 ; i++)
	{
		string sensorNKeyName = format("sensor%d", i);
		string sensorType = configSource.read_string(iniSection, sensorNKeyName, string("UNPLUGGED"));
		if(sensorType != string("UNPLUGGED"))
		{
			// the sensor is plugged :
			// // check if the sensor type is supported.
			if(sensorType == string("EZ1"))
			{
				m_sensorType[i-1] = EZ1;
				m_minRange[i-1] = 0.15;			// meters
				m_maxRange[i-1] = 6.45;			// meters
			}else if(sensorType == string("SHARP-30cm"))
			{
				m_sensorType[i-1] = SHARP_30cm;
				m_minRange[i-1] = 0.04;			// meters
				m_maxRange[i-1] = 0.3;			// meters
			}else if(sensorType == string("SHARP-80cm"))
			{
				m_sensorType[i-1] = SHARP_80cm;
				m_minRange[i-1] = 0.06;			// meters
				m_maxRange[i-1] = 0.8;			// meters
			}else
			{
				string err = format("Type of sensor %d is not supported", i);
				m_state = CGenericSensor::ssError;
				THROW_EXCEPTION(err);
			}
			m_sensorIsPlugged[i-1] = true;
			// reading the sensor pose.
			string sensorNPoseX = format("pose%d_x", i);
			string sensorNPoseY = format("pose%d_y", i);
			string sensorNPoseZ = format("pose%d_z", i);
			string sensorNPoseYaw = format("pose%d_yaw", i);
			string sensorNPosePitch = format("pose%d_pitch", i);
			string sensorNPoseRoll = format("pose%d_roll", i);

			float x = configSource.read_float(iniSection, sensorNPoseX, 0.0);
			float y = configSource.read_float(iniSection, sensorNPoseY, 0.0);
			float z = configSource.read_float(iniSection, sensorNPoseZ, 0.0);
			float yaw = configSource.read_float(iniSection, sensorNPoseYaw, 0.0);
			float pitch = configSource.read_float(iniSection, sensorNPosePitch, 0.0);
			float roll = configSource.read_float(iniSection, sensorNPoseRoll, 0.0);

			m_sensorPoses[i-1] = mrpt::poses::CPose3D(x,y,z,yaw,pitch,roll);
		}
	}
	if(display)
	{	// width = 80;
		cout.fill(' ');
		cout << "+------------------------------------------------------------------------------+" << endl;
		cout.width(79);
		cout << "|  Phidget interfaceKit board number : " << m_serialNumber;
		cout << "|" << endl;
		cout << "| Process rate : " << m_process_rate;
		cout << "|" << endl;
		cout << "+---------+---------------------+----------------------------------------------+" << endl;
		cout << "|    #    + Sensor type         | Sensor 3D pose                               |" << endl;
		cout << "+---------+---------------------+----------------------------------------------+" << endl;
		for(int i = 0 ; i < 8 ; i++)
		{
			cout << "|";
			cout.width(9);
			cout << i+1;
			cout << " |";
			cout.width(19);
			switch (m_sensorType[i])
			{
				case EZ1 :
					cout << "EZ1 |";
					break;
				case SHARP_30cm :
					cout << "SHARP_30cm |";
					break;
				case SHARP_80cm :
					cout << "SHARP_80cm |";
					break;
				case UNPLUGGED :
					cout << "UNPLUGGED |";
					break;
			}
			cout.width(43);
			cout << m_sensorPoses[i];
			cout << "|" << endl;
		}
		cout << "+------------------------------------------------------------------------------+" << endl;
	}
#else
   MRPT_UNUSED_PARAM(configSource);
   MRPT_UNUSED_PARAM(iniSection);
#endif
}

/* -----------------------------------------------------
                Initialize
   ----------------------------------------------------- */
void CPhidgetInterfaceKitProximitySensors::initialize(void)
{
#if MRPT_HAS_PHIDGET
	/*Try to connect to the interface kit board*/
	CPhidgetInterfaceKit_create((CPhidgetInterfaceKitHandle*)m_carteInterfaceKit);
	CPhidget_open(*((CPhidgetHandle*)(m_carteInterfaceKit)), m_serialNumber);
	int err = CPhidget_waitForAttachment(*((CPhidgetHandle*)(m_carteInterfaceKit)), 200);		// wait 200ms for board attachment.
	// if an error occur, "err" will be a > 0 value.
	if(err > 0 )
	{
		m_state = CGenericSensor::ssError;
		THROW_EXCEPTION("Can't find Phidget IK card, please check your serial number.");
	}
	// set frame rate
	/*int miliseconds = static_cast<int>(1000./static_cast<float>(m_process_rate));
	for(int i = 0 ; i < 8 ; i++)
	{
		if(m_sensorIsPlugged[i])
		{
			int err = CPhidgetInterfaceKit_setDataRate(*((CPhidgetInterfaceKitHandle*)(m_carteInterfaceKit)), i, miliseconds);
			if(err > 0)
			{
				string error = format("Can't set process rate to %d ms on channel %d of the Phidget IK Board.", miliseconds, i);
				m_state = CGenericSensor::ssError;
				THROW_EXCEPTION(error);
			}
		}
	}*/		// seems to be used only in the event based programming way.
	// compute (min/max) of (min/max) ranges.
	m_minOfMinRanges = *min_element(m_minRange.begin(), m_minRange.end());
	m_maxOfMaxRanges = *max_element(m_maxRange.begin(), m_maxRange.end());
	// driver is ready.
	m_state = CGenericSensor::ssWorking;
#endif
}


/* -----------------------------------------------------
                Destructor
   ----------------------------------------------------- */
CPhidgetInterfaceKitProximitySensors::~CPhidgetInterfaceKitProximitySensors()
{
#if MRPT_HAS_PHIDGET
	if(*((CPhidgetHandle*)m_carteInterfaceKit))
	{
		CPhidget_close(*((CPhidgetHandle*)(m_carteInterfaceKit)));
		CPhidget_delete(*((CPhidgetHandle*)(m_carteInterfaceKit)));
	}
#endif
}

/*-------------------------------------------------------------
					doProcess
-------------------------------------------------------------*/
void CPhidgetInterfaceKitProximitySensors::doProcess( )
{
	CObservationRangePtr obs= CObservationRange::Create();

	try
	{
		getObservation(*obs);
		m_state = ssWorking;
		// if at least one data have been sensed :
		if(obs->sensedData.size() > 0)
		{
			appendObservation( obs );
		}
	}
	catch(...)
	{
		m_state = ssError;
	    THROW_EXCEPTION("No observation received from the Phidget board!");
	}
}

/*-------------------------------------------------------------
					getObservation
-------------------------------------------------------------*/
void CPhidgetInterfaceKitProximitySensors::getObservation( mrpt::obs::CObservationRange &obs )
{
#if MRPT_HAS_PHIDGET
	obs.timestamp				= mrpt::system::getCurrentTime();
	obs.sensorLabel             = m_sensorLabel;
	obs.minSensorDistance		= m_minOfMinRanges;
	obs.maxSensorDistance		= m_maxOfMaxRanges;
	obs.sensorConeApperture		= DEG2RAD(2.0f);		// TODO : Adapt to real sensor cone apperture.
	obs.sensedData.clear();

	int sensorValue;
	for(int i = 0 ; i < 8 ; i++)
	{
		if(m_sensorIsPlugged[i])
		{
			mrpt::obs::CObservationRange::TMeasurement obsRange;
			int err = CPhidgetInterfaceKit_getSensorValue(*((CPhidgetInterfaceKitHandle*)(m_carteInterfaceKit)), i, &sensorValue);
			if(err>0)
			{
				string error("Error durring acquiering sensor value on channel : %d", i);
				THROW_EXCEPTION(error);
			}
			switch (m_sensorType[i])
			{
				case EZ1 :
					// TODO : find the conversion formula.
					obsRange.sensedDistance = 1.0;
					break;
				case SHARP_30cm :
					obsRange.sensedDistance = 2076./(static_cast<float>(sensorValue) - 11.);
					break;
				case SHARP_80cm :
					obsRange.sensedDistance = 4800./(static_cast<float>(sensorValue) - 16.92);
					break;
				default :
					obsRange.sensedDistance = -1;
					break;
			}

			obsRange.sensorID = i;
			obsRange.sensorPose = m_sensorPoses[i];		// The pose of the IR sensor on the robot
			obs.sensedData.push_back( obsRange );
		}
	}
#else
   MRPT_UNUSED_PARAM(obs);
#endif
}
