/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Emil Khatib  <emilkhatib@uma.es>		                       |
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



#include <mrpt/hwdrivers.h>  // Precompiled headers
#include <mrpt/hwdrivers/CRaePID.h>

#include <iostream>
#include <sstream>

#ifdef MRPT_OS_WINDOWS
	#  if defined(__GNUC__)
		// MinGW: Nothing to do here (yet)
	# else

	# endif
//#include <windows.h> // Rly needed?
#endif

using namespace mrpt::utils;
using namespace mrpt::hwdrivers;

IMPLEMENTS_GENERIC_SENSOR(CRaePID,mrpt::hwdrivers)

CRaePID::CRaePID()
{
	m_sensorLabel = "RAE_PID";
}

void  CRaePID::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string			&iniSection )
{
		pose_x = configSource.read_float(iniSection,"pose_x",0,true);
		pose_y = configSource.read_float(iniSection,"pose_y",0,true);
		pose_z = configSource.read_float(iniSection,"pose_z",0,true);
		pose_roll = configSource.read_float(iniSection,"pose_roll",0,true);
		pose_pitch = configSource.read_float(iniSection,"pose_pitch",0,true);
		pose_yaw = configSource.read_float(iniSection,"pose_yaw",0,true);
	#ifdef MRPT_OS_WINDOWS
		com_port = configSource.read_string(iniSection, "COM_port_PID", "COM1", true ) ;
	#else
		com_port = configSource.read_string(iniSection, "COM_port_PID", "/dev/tty0", true );
	#endif
		COM.open(com_port);
		COM.setConfig(9600,0,8,1,true);
		mrpt::system::sleep(10);
		COM.purgeBuffers();
		mrpt::system::sleep(10);
		COM.setTimeouts(50,1,100, 1,20);
	//std::cout << "PASA config" << std::endl;
}

void CRaePID::doProcess()
{
	// Send the command
	COM.purgeBuffers();
	COM.Write("R",1);

	// Read the returned text
	std::string power_reading;
	power_reading = COM.ReadString();

	// Convert the text to a number (ppm)
	const float readnum = atof(power_reading.c_str());
	const float val_ppm = readnum/1000;

	// Fill the observation
	mrpt::slam::CObservationGasSensors::TObservationENose obs;
	obs.readingsVoltage.push_back(val_ppm);

	CObservationGasSensors obsG;
	obsG.sensorLabel = this->getSensorLabel();
	obsG.m_readings.push_back(obs);
	obsG.timestamp = now();

	appendObservation(CObservationGasSensorsPtr(new CObservationGasSensors(obsG)));

}


std::string CRaePID::getFirmware()
{
	// Send the command
	COM.purgeBuffers();
	COM.Write("F",1);

	// Read the returned text
	return COM.ReadString();
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

CObservationGasSensors CRaePID::getFullInfo()
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
	mrpt::slam::CObservationGasSensors::TObservationENose obs;
	CObservationGasSensors obsG;

	for (size_t k=0; k < measurements_text.size(); k++)
	{
		const float readnum = atof(measurements_text[k].c_str());
		const float val_ppm = readnum/1000.f;

		// Fill the observation
		obs.readingsVoltage.push_back(val_ppm);
		obsG.m_readings.push_back(obs);
	}

	obsG.sensorLabel = this->getSensorLabel();
	obsG.timestamp = now();

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
