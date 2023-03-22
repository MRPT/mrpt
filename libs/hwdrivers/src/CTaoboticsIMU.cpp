/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"	// Precompiled headers
//
#include <mrpt/hwdrivers/CTaoboticsIMU.h>
#include <mrpt/obs/CObservationIMU.h>

#include <chrono>
#include <iostream>
#include <thread>

IMPLEMENTS_GENERIC_SENSOR(CTaoboticsIMU, mrpt::hwdrivers)

using namespace mrpt::comms;
using namespace mrpt::obs;
using namespace mrpt::hwdrivers;

CTaoboticsIMU::CTaoboticsIMU()
{
	m_state = ssInitializing;
	m_sensorLabel = "IMU";
}

CTaoboticsIMU::~CTaoboticsIMU() { m_serialPort->close(); }

void CTaoboticsIMU::doProcess()
{
	using namespace std::chrono_literals;

	if (m_state == ssError)
	{
		std::this_thread::sleep_for(200ms);
		initialize();
	}

	if (m_state == ssError) return;

	// try to read and parse a frame from the serial port:
	MRPT_TODO("continue");

	// yes: we have a new frame:
	auto obs = std::make_shared<CObservationIMU>();
	obs->timestamp = mrpt::system::now();

	obs->sensorPose = m_sensorPose;
	obs->sensorLabel = m_sensorLabel;

	appendObservation(obs);
}

void CTaoboticsIMU::initialize()
{
	if (m_verbose)
		std::cout << "[CTaoboticsIMU] Opening port: " << m_com_port << " at "
				  << m_baudRate << " bauds.\n";

	m_serialPort = std::make_unique<CSerialPort>(m_com_port);
	if (!(m_serialPort->isOpen()))
		THROW_EXCEPTION_FMT("can't open serial port %s", m_com_port.c_str());

	m_serialPort->setConfig(m_baudRate);

	m_state = ssWorking;
}

void CTaoboticsIMU::loadConfig_sensorSpecific(
	const mrpt::config::CConfigFileBase& configSource,
	const std::string& iniSection)
{
	m_sensorPose.setFromValues(
		configSource.read_float(iniSection, "pose_x", 0, false),
		configSource.read_float(iniSection, "pose_y", 0, false),
		configSource.read_float(iniSection, "pose_z", 0, false),
		DEG2RAD(configSource.read_float(iniSection, "pose_yaw", 0, false)),
		DEG2RAD(configSource.read_float(iniSection, "pose_pitch", 0, false)),
		DEG2RAD(configSource.read_float(iniSection, "pose_roll", 0, false)));

	m_com_port =
		configSource.read_string(iniSection, "serialPort", m_com_port, false);
}
