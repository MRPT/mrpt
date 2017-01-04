/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */



#include "hwdrivers-precomp.h"   // Precompiled headers
#include <mrpt/hwdrivers/CRaePID.h>
#include <mrpt/system/threads.h>
#include <mrpt/system/datetime.h>

#include <iostream>
#include <iterator>
#include <sstream>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt::hwdrivers;


IMPLEMENTS_GENERIC_SENSOR(CRaePID,mrpt::hwdrivers)

/* -----------------------------------------------------
                Constructor
   ----------------------------------------------------- */
CRaePID::CRaePID()
{
	m_sensorLabel = "RAE_PID";
}


/* -----------------------------------------------------
                loadConfig_sensorSpecific
   ----------------------------------------------------- */
void  CRaePID::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string			&iniSection )
{
#ifdef MRPT_OS_WINDOWS
	com_port = configSource.read_string(iniSection, "COM_port_PID", "COM1", true ) ;
#else
	com_port = configSource.read_string(iniSection, "COM_port_PID", "/dev/tty0", true );
#endif

	com_bauds		= configSource.read_int( iniSection, "baudRate",9600, false );

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
bool  CRaePID::tryToOpenTheCOM()
{
    if (COM.isOpen())
        return true;	// Already open

    if (m_verbose) cout << "[CRaePID] Opening " << com_port << " @ " <<com_bauds << endl;

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
		std::cerr << "[CRaePID::tryToOpenTheCOM] Error opening or configuring the serial port:" << std::endl << e.what();
        COM.close();
		return false;
	}
	catch (...)
	{
		std::cerr << "[CRaePID::tryToOpenTheCOM] Error opening or configuring the serial port." << std::endl;
		COM.close();
		return false;
	}
}

/* -----------------------------------------------------
				doProcess
----------------------------------------------------- */
void CRaePID::doProcess()
{
	// Is the COM open?
	if (!tryToOpenTheCOM())
	{
		m_state = ssError;
		THROW_EXCEPTION("Cannot open the serial port");
	}

	bool have_reading = false;
	std::string power_reading;
	bool time_out = false;

	while (!have_reading)
	{
		// Send command to PID to request a measurement
		COM.purgeBuffers();
		COM.Write("R",1);

		// Read PID response
		power_reading = COM.ReadString(500,&time_out);
		if (time_out)
		{
			//cout << "[CRaePID] " << com_port << " @ " <<com_bauds << " - measurement Timed-Out" << endl;
			mrpt::system::sleep(10);
		}
		else
			have_reading = true;
	}

	//cout << "[CRaePID] " << com_port << " @ " <<com_bauds << " - measurement -> " << power_reading << endl;

	// Convert the text to a number (ppm)
	const float readnum = atof(power_reading.c_str());
	const float val_ppm = readnum/1000;

	// Fill the observation
	mrpt::obs::CObservationGasSensors::TObservationENose obs;
	obs.readingsVoltage.push_back(val_ppm);
	obs.sensorTypes.push_back(0x0001);

	mrpt::obs::CObservationGasSensors obsG;
	obsG.sensorLabel = this->getSensorLabel();
	obsG.m_readings.push_back(obs);
	obsG.timestamp = mrpt::system::now();

	appendObservation(mrpt::obs::CObservationGasSensorsPtr(new mrpt::obs::CObservationGasSensors(obsG)));

}


std::string CRaePID::getFirmware()
{
	// Send the command
	cout << "Firmware version: " << endl;
	COM.purgeBuffers();
	size_t B_written = COM.Write("F",1);
	if (!B_written) return std::string("COMMS.ERROR");

	// Read the returned text
	bool time_out = false;
	std::string s_read = COM.ReadString(2000,&time_out);
	if (time_out)
		s_read =  "Time_out";
	return s_read;
}

std::string CRaePID::getModel()
{
	// Send the command
	COM.purgeBuffers();
	COM.Write("M",1);

	// Read the returned text
	return COM.ReadString();
}


std::string CRaePID::getSerialNumber()
{
	// Send the command
	COM.purgeBuffers();
	COM.Write("S",1);

	// Read the returned text
	return COM.ReadString();
}

std::string CRaePID::getName()
{
	// Send the command
	COM.purgeBuffers();
	COM.Write("N",1);

	// Read the returned text
	return COM.ReadString();

}

bool CRaePID::switchPower()
{
	// Send the command
	COM.purgeBuffers();
	COM.Write("P",1);

	// Read the returned text
	std::string reading;
	reading = COM.ReadString();

	if (strcmp(reading.c_str(),"Sleep...")==0)
		return true;
	else
		return false;
}

mrpt::obs::CObservationGasSensors CRaePID::getFullInfo()
{
	// Send the command
	COM.purgeBuffers();
	COM.Write("C",1);

	// Read the returned text
	std::string reading;
	reading = COM.ReadString();

	// Iterate over each information component (tokenize first)
	// construct a stream from the string (as seen in http://stackoverflow.com/a/53921)
	std::stringstream readings_str(reading);

	// use stream iterators to copy the stream to the vector as whitespace separated strings
	std::istream_iterator<std::string> it(readings_str);
	std::istream_iterator<std::string> endit;
	std::vector<std::string> measurements_text(it, endit);

	// Convert the text to a number (ppm)
	mrpt::obs::CObservationGasSensors::TObservationENose obs;
	mrpt::obs::CObservationGasSensors obsG;

	for (size_t k=0; k < measurements_text.size(); k++)
	{
		const float readnum = atof(measurements_text[k].c_str());
		const float val_ppm = readnum/1000.f;

		// Fill the observation
		obs.readingsVoltage.push_back(val_ppm);
		obsG.m_readings.push_back(obs);
	}

	obsG.sensorLabel = this->getSensorLabel();
	obsG.timestamp = mrpt::system::now();

	return obsG;

}

bool CRaePID::errorStatus(std::string &errorString)
{
	// Send the command
	COM.purgeBuffers();
	COM.Write("E",1);

	// Read the returned text
	std::string reading;
	reading = COM.ReadString();

	// Tokenize to separate the two components:
	// construct a stream from the string (as seen in http://stackoverflow.com/a/53921)
	std::stringstream readings_str(reading);

	// use stream iterators to copy the stream to the vector as whitespace separated strings
	std::istream_iterator<std::string> it(readings_str);
	std::istream_iterator<std::string> endit;
	std::vector<std::string> errors_text(it, endit);

	// Take the first part and check the possible error condition
	if((strcmp(errors_text[0].c_str(),"0")==0) && (strcmp(errors_text[1].c_str(),"0")==0)) // no error
	{
		return false;
	}
	else
	{
		// By the moment, return the raw error; note that if necessary a detailed description of the error can be obtained analyzing the two error strings separately
		errorString = reading;
		return true;
	}
}

void CRaePID::getLimits(float &min, float &max)
{
	// Send the command
	COM.purgeBuffers();
	COM.Write("L",1);

	// Read the returned text
	std::string reading;
	reading = COM.ReadString();

	// Tokenize to separate the two components:
	// construct a stream from the string (as seen in http://stackoverflow.com/a/53921)
	std::stringstream readings_str(reading);

	// use stream iterators to copy the stream to the vector as whitespace separated strings
	std::istream_iterator<std::string> it(readings_str);
	std::istream_iterator<std::string> endit;
	std::vector<std::string> readings_text(it, endit);

	// read min and max
	max = atof(readings_text[0].c_str());
	min = atof(readings_text[1].c_str());


}
