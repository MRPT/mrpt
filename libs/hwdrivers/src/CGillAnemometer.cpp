/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */



#include "hwdrivers-precomp.h"   // Precompiled headers
#include <mrpt/hwdrivers/CGillAnemometer.h>
#include <mrpt/system/threads.h>
#include <mrpt/system/datetime.h>

#include <iostream>
#include <iterator>
#include <sstream>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt::hwdrivers;


IMPLEMENTS_GENERIC_SENSOR(CGillAnemometer,mrpt::hwdrivers)

/* -----------------------------------------------------
                Constructor
   ----------------------------------------------------- */
CGillAnemometer::CGillAnemometer()
{
	m_sensorLabel = "WINDSONIC";
}


/* -----------------------------------------------------
                loadConfig_sensorSpecific
   ----------------------------------------------------- */
void  CGillAnemometer::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string			&iniSection )
{
#ifdef MRPT_OS_WINDOWS
	com_port = configSource.read_string(iniSection, "COM_port_WIN", "COM1", true ) ;
#else
	com_port = configSource.read_string(iniSection, "COM_port_LIN", "/dev/tty0", true );
#endif

	com_bauds		= configSource.read_int( iniSection, "COM_baudRate",9600, false );

	pose_x = configSource.read_float(iniSection,"pose_x",0,true);
	pose_y = configSource.read_float(iniSection,"pose_y",0,true);
	pose_z = configSource.read_float(iniSection,"pose_z",0,true);
	pose_roll = configSource.read_float(iniSection,"pose_roll",0,true);
	pose_pitch = configSource.read_float(iniSection,"pose_pitch",0,true);
	pose_yaw = configSource.read_float(iniSection,"pose_yaw",0,true);
}


/* -----------------------------------------------------
				tryToOpenTheCOM
----------------------------------------------------- */
bool  CGillAnemometer::tryToOpenTheCOM()
{
    if (COM.isOpen())
        return true;	// Already open

    if (m_verbose) cout << "[CGillAnemometer] Opening " << com_port << " @ " <<com_bauds << endl;

	try
	{
        COM.open(com_port);
        // Config:
        COM.setConfig( com_bauds, 0, 8, 1 );
        //COM.setTimeouts( 1, 0, 1, 1, 1 );
		COM.setTimeouts(50,1,100, 1,20);
		//mrpt::system::sleep(10);
		COM.purgeBuffers();
		//mrpt::system::sleep(10);

		return true; // All OK!
	}
	catch (std::exception &e)
	{
		std::cerr << "[CGillAnemometer::tryToOpenTheCOM] Error opening or configuring the serial port:" << std::endl << e.what();
        COM.close();
		return false;
	}
	catch (...)
	{
		std::cerr << "[CGillAnemometer::tryToOpenTheCOM] Error opening or configuring the serial port." << std::endl;
		COM.close();
		return false;
	}
}

/* -----------------------------------------------------
				doProcess
----------------------------------------------------- */
void CGillAnemometer::doProcess()
{
	// Is the COM open?
	if (!tryToOpenTheCOM())
	{
		m_state = ssError;
		printf("ERROR: No observation received from the Anemometer!\n");
		THROW_EXCEPTION("Cannot open the serial port");
	}

	mrpt::obs::CObservationWindSensorPtr obsPtr = mrpt::obs::CObservationWindSensor::Create();
	bool have_reading = false;
	std::string wind_reading;
	bool time_out = false;

	try
	{
		while (!have_reading)
		{
			// Read info into string: (default is Polar continuous)
			// format = <STX>Q, dir, speed, units, status, <ETX>
			// Q      -> Sensor identifier
			// dir    -> 3 digits (0-359)
			// speed  -> %3.2f
			// units  -> M (m/s), K (km/h)
			// status -> 00 = ok, else is an Error Code

			wind_reading = COM.ReadString(500,&time_out);
			if (time_out)
			{
				cout << "[CGillAnemometer] " << com_port << " @ " <<com_bauds << " - measurement Timed-Out" << endl;
				mrpt::system::sleep(10);
			}
			else
				have_reading = true;
		}

		// parse format = <STX>Q, dir, speed, units, status, <ETX>
		std::deque<std::string> list;
		mrpt::system::tokenize(wind_reading, ",", list);		
		if (list.size() == 6)
		{
			//Status
			int status = atoi(list.at(4).c_str());
			if (status == 0)
			{
				//Units
				std::string s_units = list.at(3);
				//Module
				std::string s_speed = list.at(2);
				if (s_units == "M")
					obsPtr->speed = atof(s_speed.c_str());
				else if (s_units == "K")
					obsPtr->speed = atof(s_speed.c_str())*1000/3600;
				else
				{
					printf("ERROR: WindSonic measurement units not supported: %s\n", s_units.c_str());
					obsPtr->speed = 0.0;
				}
				//angle
				std::string s_direction = list.at(1);
				obsPtr->direction = atof(s_direction.c_str());

				//Prepare observation
				obsPtr->sensorLabel = m_sensorLabel;
				obsPtr->timestamp = mrpt::system::getCurrentTime();
				obsPtr->sensorPoseOnRobot = mrpt::poses::CPose3D(pose_x, pose_y, pose_z, pose_yaw, pose_pitch, pose_roll);
				appendObservation(obsPtr);
			}
			else
				printf("ERROR: WindSonic error code %u\n", status);		
		}
		else if (list.size() == 5)	//when there is no wind, the sensor does not provide a wind direction
		{
			//Status
			int status = atoi(list.at(3).c_str());
			if (status == 0)
			{
				//Units
				std::string s_units = list.at(2);
				//module
				std::string s_speed = list.at(1);
				if (s_units == "M")
					obsPtr->speed = atof(s_speed.c_str());
				else if (s_units == "K")
					obsPtr->speed = atof(s_speed.c_str()) * 1000 / 3600;
				else
				{
					printf("ERROR: WindSonic measurement units not supported: %s\n", s_units.c_str());
					obsPtr->speed = 0.0;
				}
				//Angle
				obsPtr->direction = 0.0;

				//Prepare observation
				obsPtr->sensorLabel = m_sensorLabel;
				obsPtr->timestamp = mrpt::system::getCurrentTime();
				obsPtr->sensorPoseOnRobot = mrpt::poses::CPose3D(pose_x, pose_y, pose_z, pose_yaw, pose_pitch, pose_roll);
				appendObservation(obsPtr);
			}
			else
				printf("ERROR: WindSonic error code %u\n", status);
		}
		else
		{
			printf("ERROR: Windsonic reading incorrect format: %s [%u]\n", wind_reading.c_str(),static_cast<unsigned int>(list.size()) );
		}

	}
	catch (std::exception &e)
	{
		std::cerr << "[CGillAnemometer::doProcess] Error:" << std::endl << e.what();
	}
	catch (...)
	{
		std::cerr << "[CGillAnemometer::doProcess] Unknown Error" << std::endl;
	}
}
