/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"  // Precompiled headers

#include <mrpt/serialization/CMessage.h>
#include <mrpt/system/os.h>
#include <mrpt/hwdrivers/CBoardSonars.h>
#include <mrpt/serialization/CArchive.h>

#include <thread>

using namespace mrpt::hwdrivers;
using namespace mrpt::serialization;
using namespace std;

IMPLEMENTS_GENERIC_SENSOR(CBoardSonars, mrpt::hwdrivers)

/*-------------------------------------------------------------
						CBoardSonars
-------------------------------------------------------------*/
CBoardSonars::CBoardSonars()
{
	MRPT_START
	m_usbSerialNumber = "SONAR001";

	m_sensorLabel = "SONAR1";

	m_gain = 6;
	m_maxRange = 4.0f;

	MRPT_END
}

/*-------------------------------------------------------------
						loadConfig_sensorSpecific
-------------------------------------------------------------*/
void CBoardSonars::loadConfig_sensorSpecific(
	const mrpt::config::CConfigFileBase& configSource,
	const std::string& iniSection)
{
	MRPT_START

	std::vector<double> aux;  // Auxiliar vector

	// Some parameters ...
	m_usbSerialNumber = configSource.read_string(
		iniSection, "USB_serialNumber", m_usbSerialNumber, true);
	m_gain = configSource.read_int(iniSection, "gain", m_gain, true);
	m_maxRange =
		configSource.read_float(iniSection, "maxRange", m_maxRange, true);
	m_minTimeBetweenPings = configSource.read_float(
		iniSection, "minTimeBetweenPings", m_minTimeBetweenPings, true);
	// ----------------------------------------------------------------------------------------------------------------------
	ASSERT_(m_maxRange > 0 && m_maxRange <= 11);
	ASSERT_(m_gain <= 16);

	// Sonar firing order ...
	configSource.read_vector(
		iniSection, "firingOrder", m_firingOrder, m_firingOrder, true);

	// ----------------------------------------------------------------------------------------------------------------------

	// Individual sonar gains ...
	configSource.read_vector(iniSection, "sonarGains", aux, aux, true);

	std::vector<int32_t>::iterator itSonar;
	std::vector<double>::iterator itAux;
	for (itSonar = m_firingOrder.begin(), itAux = aux.begin();
		 itSonar != m_firingOrder.end(); ++itSonar, ++itAux)
		m_sonarGains[*itSonar] = *itAux;
	// ----------------------------------------------------------------------------------------------------------------------
	ASSERT_(aux.size() == m_firingOrder.size());

	// Individual sonar poses
	aux.clear();
	for (itSonar = m_firingOrder.begin(); itSonar != m_firingOrder.end();
		 ++itSonar)
	{
		configSource.read_vector(
			iniSection, format("pose%i", *itSonar), aux, aux,
			true);  // Get the sonar poses
		m_sonarPoses[*itSonar] = mrpt::math::TPose3D(
			aux[0], aux[1], aux[2], DEG2RAD((float)aux[3]),
			DEG2RAD((float)aux[4]), DEG2RAD((float)aux[5]));
	}
	// ----------------------------------------------------------------------------------------------------------------------
	ASSERT_(m_sonarGains.size() == m_firingOrder.size());

	MRPT_END
}

/*-------------------------------------------------------------
					queryFirmwareVersion
-------------------------------------------------------------*/
bool CBoardSonars::queryFirmwareVersion(string& out_firmwareVersion)
{
	try
	{
		CMessage msg, msgRx;

		// Try to connect to the device:
		if (!checkConnectionAndConnect()) return false;

		msg.type = 0x10;
		auto arch = mrpt::serialization::archiveFrom(*this);
		arch.sendMessage(msg);

		if (arch.receiveMessage(msgRx))
		{
			msgRx.getContentAsString(out_firmwareVersion);
			return true;
		}
		else
			return false;
	}
	catch (...)
	{
		Close();
		return false;
	}
}

/*-------------------------------------------------------------
					checkConnectionAndConnect
-------------------------------------------------------------*/
bool CBoardSonars::sendConfigCommands()
{
	try
	{
		if (!isOpen()) return false;
		mrpt::serialization::CMessage msg, msgRx;
		size_t i;

		// Send cmd for firing order:
		// ----------------------------
		msg.type = 0x12;
		msg.content.resize(16);
		for (i = 0; i < 16; i++)
		{
			if (i < m_firingOrder.size())
				msg.content[i] = m_firingOrder[i];
			else
				msg.content[i] = 0xFF;
		}
		auto arch = mrpt::serialization::archiveFrom(*this);
		arch.sendMessage(msg);
		if (!arch.receiveMessage(msgRx)) return false;  // Error

		// Send cmd for gain:
		// ----------------------------
		// msg.type = 0x13;
		// msg.content.resize(1);
		// msg.content[0] = m_gain;
		// sendMessage(msg);
		// if (!receiveMessage(msgRx) ) return false;	// Error

		// Send cmd for set of gains:
		// ----------------------------
		msg.type = 0x16;
		msg.content.resize(16);
		for (i = 0; i < 16; i++)
		{
			if (m_sonarGains.find(i) != m_sonarGains.end())
				msg.content[i] = m_sonarGains[i];
			else
				msg.content[i] = 0xFF;
		}
		arch.sendMessage(msg);
		if (!arch.receiveMessage(msgRx)) return false;  // Error

		// Send cmd for max range:
		// ----------------------------
		msg.type = 0x14;
		msg.content.resize(1);
		msg.content[0] = (int)((m_maxRange / 0.043f) - 1);
		arch.sendMessage(msg);
		if (!arch.receiveMessage(msgRx)) return false;  // Error

		// Send cmd for max range:
		// ----------------------------
		msg.type = 0x15;
		msg.content.resize(2);
		auto T = (uint16_t)(m_minTimeBetweenPings * 1000.0f);
		msg.content[0] = T >> 8;
		msg.content[1] = T & 0x00FF;
		arch.sendMessage(msg);
		if (!arch.receiveMessage(msgRx)) return false;  // Error

		return true;
	}
	catch (...)
	{
		// Error opening device:
		Close();
		return false;
	}
}

/*-------------------------------------------------------------
					getObservation
-------------------------------------------------------------*/
bool CBoardSonars::getObservation(mrpt::obs::CObservationRange& obs)
{
	try
	{
		obs.sensorLabel = m_sensorLabel;
		obs.timestamp = mrpt::system::getCurrentTime();
		obs.minSensorDistance = 0.04f;
		obs.maxSensorDistance = m_maxRange;
		obs.sensorConeApperture = DEG2RAD(30.0f);
		obs.sensedData.clear();
		mrpt::obs::CObservationRange::TMeasurement obsRange;

		mrpt::serialization::CMessage msg, msgRx;

		// Try to connect to the device:
		if (!checkConnectionAndConnect()) return false;

		auto arch = mrpt::serialization::archiveFrom(*this);

		msg.type = 0x11;
		arch.sendMessage(msg);

		if (arch.receiveMessage(msgRx))
		{
			if (msgRx.content.empty()) return false;

			// For each sensor:
			ASSERT_((msgRx.content.size() % 2) == 0);
			vector<uint16_t> data(msgRx.content.size() / 2);
			memcpy(&data[0], &msgRx.content[0], msgRx.content.size());

			for (size_t i = 0; i < data.size() / 2; i++)
			{
				uint16_t sonar_idx = data[2 * i + 0];
				uint16_t sonar_range_cm = data[2 * i + 1];
				if (sonar_range_cm != 0xFFFF && sonar_idx < 16)
				{
					obsRange.sensorID = sonar_idx;
					obsRange.sensorPose =
						m_sonarPoses[sonar_idx];  // mrpt::poses::CPose3D(); //
					// sonar_idx
					obsRange.sensedDistance = sonar_range_cm * 0.01f;
					obs.sensedData.push_back(obsRange);
				}
			}
			return true;
		}
		else
			return false;
	}
	catch (...)
	{
		Close();
		return false;
	}
}

/*-------------------------------------------------------------
					programI2CAddress
-------------------------------------------------------------*/
bool CBoardSonars::programI2CAddress(uint8_t currentAddress, uint8_t newAddress)
{
	try
	{
		mrpt::serialization::CMessage msg, msgRx;

		// Try to connect to the device:
		if (!checkConnectionAndConnect()) return false;
		auto arch = mrpt::serialization::archiveFrom(*this);

		msg.type = 0x20;
		msg.content.resize(2);
		msg.content[0] = currentAddress;
		msg.content[1] = newAddress;
		arch.sendMessage(msg);

		std::this_thread::sleep_for(10ms);

		return arch.receiveMessage(msgRx);
	}
	catch (...)
	{
		Close();
		return false;
	}
}

/*-------------------------------------------------------------
					checkConnectionAndConnect
-------------------------------------------------------------*/
bool CBoardSonars::checkConnectionAndConnect()
{
	if (isOpen()) return true;

	try
	{
		OpenBySerialNumber(m_usbSerialNumber);
		std::this_thread::sleep_for(10ms);
		Purge();
		std::this_thread::sleep_for(10ms);
		SetLatencyTimer(1);
		SetTimeouts(300, 100);

		return sendConfigCommands();
	}
	catch (...)
	{
		// Error opening device:
		Close();
		return false;
	}
}

/*-------------------------------------------------------------
					doProcess
-------------------------------------------------------------*/
void CBoardSonars::doProcess()
{
	mrpt::obs::CObservationRange::Ptr obs =
		mrpt::make_aligned_shared<mrpt::obs::CObservationRange>();
	if (getObservation(*obs)) appendObservation(obs);
}
