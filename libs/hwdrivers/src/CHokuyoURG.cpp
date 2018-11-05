/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"  // Precompiled headers

#include <mrpt/hwdrivers/CHokuyoURG.h>
#include <mrpt/comms/CSerialPort.h>
#include <mrpt/comms/CClientTCPSocket.h>
#include <mrpt/system/os.h>
#include <mrpt/opengl/CPlanarLaserScan.h>  // in library mrpt-maps
#include <mrpt/opengl/CAxis.h>

IMPLEMENTS_GENERIC_SENSOR(CHokuyoURG, mrpt::hwdrivers)

using namespace mrpt::obs;
using namespace mrpt::hwdrivers;
using namespace mrpt::comms;
using namespace mrpt::system;
using namespace mrpt::opengl;
using namespace std;

const int MINIMUM_PACKETS_TO_SET_TIMESTAMP_REFERENCE = 10;

CHokuyoURG::CHokuyoURG()
	: m_sensorPose(0, 0, 0),
	  m_rx_buffer(40000),

	  m_com_port(""),
	  m_ip_dir("")

{
	m_sensorLabel = "Hokuyo";
}

CHokuyoURG::~CHokuyoURG()
{
	if (m_stream)
	{
		turnOff();

		if (m_I_am_owner_serial_port) delete m_stream;
		m_stream = nullptr;
	}
	m_win.reset();
}

void CHokuyoURG::sendCmd(const char* str)
{
	MRPT_START
	ASSERT_(str != nullptr);
	ASSERT_(m_stream != nullptr);
	const size_t N = strlen(str);
	const size_t nWriten = m_stream->Write(str, N);
	ASSERT_EQUAL_(nWriten, N);

	MRPT_LOG_DEBUG_STREAM("[Hokuyo] sendCmd(): `" << str << "`");

	m_lastSentMeasCmd = std::string(str);  // for echo verification
	MRPT_END
}

void CHokuyoURG::doProcessSimple(
	bool& outThereIsObservation,
	mrpt::obs::CObservation2DRangeScan& outObservation, bool& hardwareError)
{
	outThereIsObservation = false;
	hardwareError = false;

	// Bound?
	if (!ensureStreamIsOpen())
	{
		m_timeStartUI = 0;
		m_timeStartSynchDelay = 0;
		hardwareError = true;
		return;
	}

	// Wait for a message:
	char rcv_status0, rcv_status1;
	int nRanges = m_lastRange - m_firstRange + 1;
	int expectedSize = nRanges * 3 + 4;
	if (m_intensity) expectedSize += nRanges * 3;

	m_rcv_data.clear();
	m_rcv_data.reserve(expectedSize + 1000);

	m_state = ssWorking;
	if (!receiveResponse(rcv_status0, rcv_status1))
	{
		// No new data
		return;
	}

	// DECODE:
	if (rcv_status0 != '0' && rcv_status0 != '9')
	{
		hardwareError = true;
		return;
	}

	// -----------------------------------------------
	//   Extract the observation:
	// -----------------------------------------------
	outObservation.timestamp = mrpt::system::now();

	if ((size_t)expectedSize != m_rcv_data.size())
	{
		MRPT_LOG_ERROR_STREAM(
			"[CHokuyoURG::doProcess] ERROR: Expected "
			<< expectedSize << " data bytes, received " << m_rcv_data.size()
			<< "instead!");
		hardwareError = true;
		return;
	}
	// Delay the sync of timestamps due to instability in the constant rate
	// during the first few packets.
	bool do_timestamp_sync = !m_disable_firmware_timestamp;
	if (do_timestamp_sync &&
		m_timeStartSynchDelay < MINIMUM_PACKETS_TO_SET_TIMESTAMP_REFERENCE)
	{
		do_timestamp_sync = false;
		m_timeStartSynchDelay++;
	}

	if (do_timestamp_sync)
	{
		// Extract the timestamp of the sensor:
		uint32_t nowUI = ((m_rcv_data[0] - 0x30) << 18) +
						 ((m_rcv_data[1] - 0x30) << 12) +
						 ((m_rcv_data[2] - 0x30) << 6) + (m_rcv_data[3] - 0x30);

		uint32_t AtUI = 0;
		if (m_timeStartUI == 0)
		{
			m_timeStartUI = nowUI;
			m_timeStartTT = mrpt::system::now();
		}
		else
			AtUI = nowUI - m_timeStartUI;

		auto AtDO = std::chrono::milliseconds(AtUI);
		outObservation.timestamp = m_timeStartTT + AtDO;
	}

	// And the scan ranges:
	outObservation.rightToLeft = true;

	outObservation.aperture =
		nRanges * 2 * M_PI / m_sensor_info.scans_per_360deg;

	outObservation.maxRange = m_sensor_info.d_max;
	outObservation.stdError = 0.010f;
	outObservation.sensorPose = m_sensorPose;
	outObservation.sensorLabel = m_sensorLabel;

	outObservation.resizeScan(nRanges);
	char* ptr = (char*)&m_rcv_data[4];

	if (m_intensity) outObservation.setScanHasIntensity(true);

	for (int i = 0; i < nRanges; i++)
	{
		int b1 = (*ptr++) - 0x30;
		int b2 = (*ptr++) - 0x30;
		int b3 = (*ptr++) - 0x30;

		int range_mm = ((b1 << 12) | (b2 << 6) | b3);

		outObservation.setScanRange(i, range_mm * 0.001f);
		outObservation.setScanRangeValidity(
			i, range_mm >= 20 &&
				   (outObservation.scan[i] <= outObservation.maxRange));

		if (m_intensity)
		{
			int b4 = (*ptr++) - 0x30;
			int b5 = (*ptr++) - 0x30;
			int b6 = (*ptr++) - 0x30;
			outObservation.setScanIntensity(i, ((b4 << 12) | (b5 << 6) | b6));
		}
	}

	// Do filter:
	C2DRangeFinderAbstract::filterByExclusionAreas(outObservation);
	C2DRangeFinderAbstract::filterByExclusionAngles(outObservation);
	// Do show preview:
	C2DRangeFinderAbstract::processPreview(outObservation);

	outThereIsObservation = true;
}

/*-------------------------------------------------------------
						loadConfig_sensorSpecific
-------------------------------------------------------------*/
void CHokuyoURG::loadConfig_sensorSpecific(
	const mrpt::config::CConfigFileBase& configSource,
	const std::string& iniSection)
{
	m_reduced_fov =
		DEG2RAD(configSource.read_float(iniSection, "reduced_fov", 0)),

	m_motorSpeed_rpm =
		configSource.read_int(iniSection, "HOKUYO_motorSpeed_rpm", 0);
	m_sensorPose.setFromValues(
		configSource.read_float(iniSection, "pose_x", 0),
		configSource.read_float(iniSection, "pose_y", 0),
		configSource.read_float(iniSection, "pose_z", 0),
		DEG2RAD(configSource.read_float(iniSection, "pose_yaw", 0)),
		DEG2RAD(configSource.read_float(iniSection, "pose_pitch", 0)),
		DEG2RAD(configSource.read_float(iniSection, "pose_roll", 0)));

	m_highSensMode =
		configSource.read_bool(iniSection, "HOKUYO_HS_mode", m_highSensMode);

#ifdef _WIN32
	m_com_port =
		configSource.read_string(iniSection, "COM_port_WIN", m_com_port);
#else
	m_com_port =
		configSource.read_string(iniSection, "COM_port_LIN", m_com_port);
#endif

	m_ip_dir = configSource.read_string(iniSection, "IP_DIR", m_ip_dir);
	m_port_dir = configSource.read_int(iniSection, "PORT_DIR", m_port_dir);

	ASSERTMSG_(
		!m_com_port.empty() || !m_ip_dir.empty(),
		"Either COM_port or IP_DIR must be defined in the configuration file!");
	ASSERTMSG_(
		m_com_port.empty() || m_ip_dir.empty(),
		"Both COM_port and IP_DIR set! Please, define only one of them.");
	if (!m_ip_dir.empty())
	{
		ASSERTMSG_(
			m_port_dir,
			"A TCP/IP port number `PORT_DIR` must be specified for Ethernet "
			"connection");
	}

	m_disable_firmware_timestamp = configSource.read_bool(
		iniSection, "disable_firmware_timestamp", m_disable_firmware_timestamp);
	m_intensity = configSource.read_bool(iniSection, "intensity", m_intensity),

	MRPT_LOAD_HERE_CONFIG_VAR(
		scan_interval, int, m_scan_interval, configSource, iniSection);

	// Parent options:
	C2DRangeFinderAbstract::loadCommonParams(configSource, iniSection);
}

/*-------------------------------------------------------------
						turnOn
-------------------------------------------------------------*/
bool CHokuyoURG::turnOn()
{
	MRPT_START

	// Bound?
	if (!ensureStreamIsOpen()) return false;

	// If we are over a serial link, set it up:
	if (m_ip_dir.empty())
	{
		auto* COM = dynamic_cast<CSerialPort*>(m_stream);

		if (COM != nullptr)
		{
			// It is a COM:
			COM->setConfig(19200);
			COM->setTimeouts(100, 0, 200, 0, 0);

			// Assure the laser is off and quiet:
			switchLaserOff();
			std::this_thread::sleep_for(10ms);

			COM->purgeBuffers();
			std::this_thread::sleep_for(10ms);

			COM->setConfig(115200);
			switchLaserOff();
			std::this_thread::sleep_for(10ms);
			COM->purgeBuffers();
			std::this_thread::sleep_for(10ms);
			COM->setConfig(19200);
		}

		if (COM != nullptr)
		{
			// Set 115200 baud rate:
			setHighBaudrate();
			enableSCIP20();
			COM->setConfig(115200);
		}
	}
	else
	{
		auto* COM = dynamic_cast<CClientTCPSocket*>(m_stream);

		if (COM != nullptr)
		{
			// Assure the laser is off and quiet:
			switchLaserOff();
			std::this_thread::sleep_for(10ms);

			purgeBuffers();
			std::this_thread::sleep_for(10ms);

			switchLaserOff();
			std::this_thread::sleep_for(10ms);
			purgeBuffers();
		}
	}

	if (!enableSCIP20()) return false;

	// Turn on the laser:
	if (!switchLaserOn()) return false;

	// Set the motor speed:
	if (m_motorSpeed_rpm)
		if (!setMotorSpeed(m_motorSpeed_rpm)) return false;

	// Set HS mode:
	setHighSensitivityMode(m_highSensMode);

	// Display sensor information:
	if (!displaySensorInfo(&m_sensor_info)) return false;

	// Set for scanning angles:
	m_firstRange = m_sensor_info.scan_first;
	m_lastRange = m_sensor_info.scan_last;

	// Artificially reduced FOV?
	if (m_reduced_fov > 0 && m_reduced_fov < 2 * M_PI)
	{
		int center = (m_lastRange + m_firstRange) >> 1;
		const int half_range = static_cast<int>(
								   (m_sensor_info.scans_per_360deg / 360) *
								   RAD2DEG(m_reduced_fov)) >>
							   1;
		m_firstRange = center - half_range;
		m_lastRange = center + half_range;
		MRPT_LOG_INFO_STREAM(
			"[HOKUYO::turnOn] Using reduced FOV: ranges ["
			<< m_firstRange << "-" << m_lastRange << "] for "
			<< RAD2DEG(m_reduced_fov) << " deg. FOV");
	}

	if (!displayVersionInfo())
	{
		// return false; // It's not SO important
	}

	// Start!
	if (!startScanningMode()) return false;

	return true;

	MRPT_END
}

/*-------------------------------------------------------------
						turnOff
-------------------------------------------------------------*/
bool CHokuyoURG::turnOff()
{
	// Turn off the laser:
	if (!switchLaserOff()) return false;

	return true;
}

bool CHokuyoURG::setHighBaudrate()
{
	char rcv_status0, rcv_status1;
	if (!ensureStreamIsOpen()) return false;

	MRPT_LOG_DEBUG(
		"[CHokuyoURG::setHighBaudrate] Changing baudrate to 115200...");

	// Send command:
	sendCmd("SS115200\x0A");

	// Receive response:
	if (!receiveResponse(rcv_status0, rcv_status1))
	{
		MRPT_LOG_ERROR(
			"[CHokuyoURG::setHighBaudrate] Error waiting for response");
		return false;
	}

	MRPT_LOG_DEBUG("OK\n");
	return true;
}

/*-------------------------------------------------------------
						assureBufferHasBytes
-------------------------------------------------------------*/
bool CHokuyoURG::assureBufferHasBytes(const size_t nDesiredBytes)
{
	ASSERT_(nDesiredBytes < m_rx_buffer.capacity());

	if (m_rx_buffer.size() >= nDesiredBytes)
	{
		return true;
	}
	else
	{
		// Try to read more bytes:
		uint8_t buf[128];
		const size_t to_read = std::min(m_rx_buffer.available(), sizeof(buf));

		try
		{
			size_t nRead;

			if (!m_ip_dir.empty())
			{
				auto* client = dynamic_cast<CClientTCPSocket*>(m_stream);
				nRead = client->readAsync(buf, to_read, 100, 10);
			}
			else
			{
				nRead = m_stream->Read(buf, to_read);
			}

			m_rx_buffer.push_many(buf, nRead);
		}
		catch (std::exception&)
		{
			// 0 bytes read
		}

		return (m_rx_buffer.size() >= nDesiredBytes);
	}
}

bool CHokuyoURG::receiveResponse(char& rcv_status0, char& rcv_status1)
{
	m_rcv_data.clear();
	if (!ensureStreamIsOpen()) return false;
	ASSERT_(!m_lastSentMeasCmd.empty());

	try
	{
		// Process response:
		// ---------------------------------

		// COMMAND ECHO ---------
		unsigned int i = 0;
		const unsigned int verifLen = m_lastSentMeasCmd.size();

		if (verifLen)
		{
			do
			{
				if (!assureBufferHasBytes(verifLen - i)) return false;

				// If matches the echo, go on:
				if (m_rx_buffer.pop() == m_lastSentMeasCmd[i])
					i++;
				else
					i = 0;
			} while (i < verifLen);
		}

		// Now, the status bytes:
		if (!assureBufferHasBytes(2)) return false;

		rcv_status0 = m_rx_buffer.pop();
		rcv_status1 = m_rx_buffer.pop();

		// In SCIP2.0, there is an additional sum char:
		if (rcv_status1 != 0x0A)
		{
			// Yes, it is SCIP2.0
			if (!assureBufferHasBytes(1)) return false;

			// Ignore this byte: sumStatus
			m_rx_buffer.pop();
		}
		else
		{
			// Continue, it seems a SCIP1.1 response...
		}

		// After the status bytes, there is a LF:
		if (!assureBufferHasBytes(1)) return false;
		char nextChar = m_rx_buffer.pop();
		if (nextChar != 0x0A) return false;

		// -----------------------------------------------------------------------------
		// Now the data:
		// There's a problem here, we don't know in advance how many bytes to
		// read,
		//  so rely on the serial port class implemented buffer and call many
		//  times
		//  the read method with only 1 byte each time:
		// -----------------------------------------------------------------------------
		bool lastWasLF = false;
		i = 0;
		for (;;)
		{
			if (!assureBufferHasBytes(1))
			{
				return false;
			}
			m_rcv_data.push_back(m_rx_buffer.pop());
			i++;  // One more byte in the buffer

			// No data?
			if (i == 1 && m_rcv_data[0] == 0x0A)
			{
				m_rcv_data.clear();
				return true;
			}

			// Is it a LF?
			if (m_rcv_data[i - 1] == 0x0A)
			{
				if (!lastWasLF)
				{
					// Discard SUM+LF
					ASSERT_(i >= 2);
					i -= 2;
					m_rcv_data.resize(i);
				}
				else
				{
					// Discard this last LF:
					i--;

					// Done!
					m_rcv_data.resize(i);
					MRPT_LOG_DEBUG_STREAM(
						"[Hokuyo] receiveResponse(): RX `" << m_rcv_data
														   << "`");

					if (rcv_status0 != '0' &&
						(rcv_status0 != '9' && rcv_status1 != '9'))
					{
						MRPT_LOG_ERROR_STREAM(
							"[Hokuyo] Error LIDAR status: "
							<< (int)rcv_status0 << " after command: `"
							<< m_lastSentMeasCmd << "`");
						return false;
					}

					return true;
				}
				lastWasLF = true;
			}
			else
				lastWasLF = false;
		}
	}
	catch (const std::exception& e)
	{
		MRPT_LOG_ERROR_FMT(
			"[Hokuyo] receiveResponse() Exception: %s", e.what());
		return false;
	}
	catch (...)
	{
		return false;  // Serial port timeout,...
	}
}

bool CHokuyoURG::enableSCIP20()
{
	char rcv_status0, rcv_status1;
	if (!ensureStreamIsOpen()) return false;

	MRPT_LOG_DEBUG(
		"[CHokuyoURG::enableSCIP20] Changing protocol to SCIP2.0...");

	// Send command:
	sendCmd("SCIP2.0\x0A");

	// Receive response:
	if (!receiveResponse(rcv_status0, rcv_status1))
	{
		MRPT_LOG_ERROR_STREAM(
			__CURRENT_FUNCTION_NAME__ << ": Error in response");
		return false;
	}

	MRPT_LOG_DEBUG("OK\n");
	return true;
}

bool CHokuyoURG::switchLaserOn()
{
	char rcv_status0, rcv_status1;

	if (!ensureStreamIsOpen()) return false;

	MRPT_LOG_DEBUG("[CHokuyoURG::switchLaserOn] Switching laser ON...");

	// Send command:
	sendCmd("BM\x0A");

	// Receive response:
	if (!receiveResponse(rcv_status0, rcv_status1))
	{
		MRPT_LOG_ERROR_STREAM(
			__CURRENT_FUNCTION_NAME__ << ": Error in response");
		return false;
	}

	MRPT_LOG_DEBUG("OK\n");
	return true;
}

bool CHokuyoURG::switchLaserOff()
{
	char rcv_status0, rcv_status1;

	if (!ensureStreamIsOpen()) return false;

	MRPT_LOG_DEBUG("[CHokuyoURG::switchLaserOff] Switching laser OFF...");

	// Send command:
	sendCmd("QT\x0A");

	// Receive response:
	if (!receiveResponse(rcv_status0, rcv_status1))
	{
		MRPT_LOG_ERROR_STREAM(
			__CURRENT_FUNCTION_NAME__ << ": Error in response");
		return false;
	}

	MRPT_LOG_DEBUG("OK\n");
	return true;
}

void CHokuyoURG::setScanInterval(unsigned int skipScanCount)
{
	m_scan_interval = skipScanCount;
}
unsigned int CHokuyoURG::getScanInterval() const { return m_scan_interval; }
bool CHokuyoURG::setMotorSpeed(int motoSpeed_rpm)
{
	char rcv_status0, rcv_status1;
	if (!ensureStreamIsOpen()) return false;

	MRPT_LOG_DEBUG_FMT(
		"[CHokuyoURG::setMotorSpeed] Setting to %i rpm...", motoSpeed_rpm);

	// Send command:
	int motorSpeedCode = (600 - motoSpeed_rpm) / 6;
	if (motorSpeedCode < 0 || motorSpeedCode > 10)
	{
		MRPT_LOG_ERROR_STREAM(
			__CURRENT_FUNCTION_NAME__
			<< " Motorspeed must be in the range 540-600 rpm");
		return false;
	}

	char cmd[20];
	os::sprintf(cmd, 20, "CR%02i\x0A", motorSpeedCode);
	sendCmd(cmd);

	// Receive response:
	if (!receiveResponse(rcv_status0, rcv_status1))
	{
		MRPT_LOG_ERROR_STREAM(
			__CURRENT_FUNCTION_NAME__ << ": Error in response");
		return false;
	}

	MRPT_LOG_DEBUG("OK\n");
	return true;
}

/*-------------------------------------------------------------
						setHighSensitivityMode
-------------------------------------------------------------*/
bool CHokuyoURG::setHighSensitivityMode(bool enabled)
{
	char rcv_status0, rcv_status1;
	if (!ensureStreamIsOpen()) return false;

	MRPT_LOG_DEBUG_FMT(
		"[CHokuyoURG::setHighSensitivityMode] Setting HS mode to: %s...",
		enabled ? "true" : "false");

	// Send command:
	char cmd[20];
	os::sprintf(cmd, 20, "HS%i\x0A", enabled ? 1 : 0);
	sendCmd(cmd);

	// Receive response:
	if (!receiveResponse(rcv_status0, rcv_status1))
	{
		MRPT_LOG_ERROR_STREAM(
			__CURRENT_FUNCTION_NAME__ << ": Error in response");
		return false;
	}

	MRPT_LOG_DEBUG("OK\n");
	return true;
}

/*-------------------------------------------------------------
												setIntensityMode
-------------------------------------------------------------*/
bool CHokuyoURG::setIntensityMode(bool enabled)
{
	m_intensity = enabled;
	return true;
}

bool CHokuyoURG::displayVersionInfo()
{
	char rcv_status0, rcv_status1;
	if (!ensureStreamIsOpen()) return false;

	MRPT_LOG_DEBUG("[CHokuyoURG::displayVersionInfo] Asking info...");

	// Send command:
	sendCmd("VV\x0A");

	// Receive response:
	if (!receiveResponse(rcv_status0, rcv_status1))
	{
		MRPT_LOG_ERROR_STREAM(
			__CURRENT_FUNCTION_NAME__ << ": Error in response");
		return false;
	}

	MRPT_LOG_DEBUG("OK\n");

	// PRINT:
	for (auto& c : m_rcv_data)
		if (c == ';') c = '\n';
	m_rcv_data[m_rcv_data.size()] = '\0';

	if (!m_rcv_data.empty())
	{
		MRPT_LOG_INFO_STREAM(
			"\n------------- HOKUYO Scanner: Version Information ------\n"
			<< &m_rcv_data[0]
			<< "\n"
			   "-------------------------------------------------------\n\n");
	}
	return true;
}

/*-------------------------------------------------------------
						displaySensorInfo
-------------------------------------------------------------*/
bool CHokuyoURG::displaySensorInfo(TSensorInfo* out_data)
{
	char rcv_status0, rcv_status1;
	if (!ensureStreamIsOpen()) return false;

	MRPT_LOG_DEBUG("[CHokuyoURG::displaySensorInfo] Asking for info...");

	// Send command:
	sendCmd("PP\x0A");

	// Receive response:
	if (!receiveResponse(rcv_status0, rcv_status1))
	{
		MRPT_LOG_ERROR_STREAM(
			__CURRENT_FUNCTION_NAME__ << ": Error in response");
		return false;
	}
	MRPT_LOG_DEBUG("OK\n");

	// PRINT:
	for (auto& c : m_rcv_data)
		if (c == ';') c = '\n';
	m_rcv_data[m_rcv_data.size()] = '\0';

	if (!m_rcv_data.empty())
	{
		MRPT_LOG_INFO_STREAM(
			"\n------------- HOKUYO Scanner: Product Information ------\n"
			<< &m_rcv_data[0]
			<< "\n"
			   "-------------------------------------------------------\n\n");
	}

	// Parse the data:
	if (out_data)
	{
		const char* ptr;

		if (nullptr != (ptr = strstr(&m_rcv_data[0], "DMAX:")))
			out_data->d_max = 0.001 * atoi(ptr + 5);
		else
			MRPT_LOG_ERROR("[Hokuyo] displayVersionInfo() parse error");

		if (nullptr != (ptr = strstr(&m_rcv_data[0], "DMIN:")))
			out_data->d_min = 0.001 * atoi(ptr + 5);
		else
			MRPT_LOG_ERROR("[Hokuyo] displayVersionInfo() parse error");

		if (nullptr != (ptr = strstr(&m_rcv_data[0], "ARES:")))
			out_data->scans_per_360deg = atoi(ptr + 5);
		else
			MRPT_LOG_ERROR("[Hokuyo] displayVersionInfo() parse error");

		if (nullptr != (ptr = strstr(&m_rcv_data[0], "SCAN:")))
			out_data->motor_speed_rpm = atoi(ptr + 5);
		else
			MRPT_LOG_ERROR("[Hokuyo] displayVersionInfo() parse error");

		if (nullptr != (ptr = strstr(&m_rcv_data[0], "AMIN:")))
			out_data->scan_first = atoi(ptr + 5);
		else
			MRPT_LOG_ERROR("[Hokuyo] displayVersionInfo() parse error");

		if (nullptr != (ptr = strstr(&m_rcv_data[0], "AMAX:")))
			out_data->scan_last = atoi(ptr + 5);
		else
			MRPT_LOG_ERROR("[Hokuyo] displayVersionInfo() parse error");

		if (nullptr != (ptr = strstr(&m_rcv_data[0], "AFRT:")))
			out_data->scan_front = atoi(ptr + 5);
		else
			MRPT_LOG_ERROR("[Hokuyo] displayVersionInfo() parse error");

		if (nullptr != (ptr = strstr(&m_rcv_data[0], "MODL:")))
		{
			char aux[30];
			memcpy(aux, ptr + 5, 8);
			aux[8] = '\0';
			out_data->model = aux;
		}
		else
			MRPT_LOG_ERROR("[Hokuyo] displayVersionInfo() parse error");
	}

	return true;
}

bool CHokuyoURG::startScanningMode()
{
	char rcv_status0, rcv_status1;
	if (!ensureStreamIsOpen()) return false;

	MRPT_LOG_DEBUG("[CHokuyoURG::startScanningMode] Starting scanning mode...");

	// Send command:
	// 'M' 'D'
	// 'XXXX' (starting step)
	// 'XXXX' (end step)
	// 'XX' (cluster count)
	// 'X' (scan interval)
	// 'XX' (number of scans)
	char cmd[50];
	unsigned int scan_interval = m_scan_interval;
	if (scan_interval > 9) scan_interval = 9;
	os::sprintf(
		cmd, 50, "M%c%04u%04u01%u00\x0A", m_intensity ? 'E' : 'D', m_firstRange,
		m_lastRange, scan_interval);

	sendCmd(cmd);

	// Receive response:
	if (!receiveResponse(rcv_status0, rcv_status1))
	{
		MRPT_LOG_ERROR_STREAM(
			__CURRENT_FUNCTION_NAME__ << ": Error in response");
		return false;
	}

	MRPT_LOG_DEBUG("OK\n");
	return true;
}

bool CHokuyoURG::ensureStreamIsOpen()
{
	MRPT_START

	if (m_stream)
	{
		// Socket or USB connection?
		if (!m_ip_dir.empty() && m_port_dir)
		{
			// Has the port been disconected (USB serial ports)??
			auto* COM = dynamic_cast<CClientTCPSocket*>(m_stream);

			if (COM != nullptr)
			{
				if (COM->isConnected()) return true;

				// It has been disconnected... try to reconnect:
				MRPT_LOG_ERROR(
					"[CHokuyoURG] Socket connection lost! trying to "
					"reconnect...");

				try
				{
					COM->connect(m_ip_dir, m_port_dir);
					// OK, reconfigure the laser:
					turnOn();
					return true;
				}
				catch (...)
				{
					// Not yet..
					return false;
				}
			}
			else
			{
				return true;  // Assume OK
			}
		}
		else
		{
			// Has the port been disconected (USB serial ports)??
			auto* COM = dynamic_cast<CSerialPort*>(m_stream);
			if (COM != nullptr)
			{
				if (COM->isOpen()) return true;

				// It has been disconnected... try to reconnect:
				MRPT_LOG_ERROR_STREAM(
					__CURRENT_FUNCTION_NAME__
					<< ": Serial port connection lost! Trying to reconnect...");

				try
				{
					COM->open();
					// OK, reconfigure the laser:
					turnOn();
					return true;
				}
				catch (...)
				{
					// Not yet..
					return false;
				}
			}
			else
			{
				return true;  // Assume OK
			}
		}
	}
	else
	{
		if (m_com_port.empty() && m_ip_dir.empty() && !m_port_dir)
		{
			THROW_EXCEPTION(
				"No stream bound to the laser nor COM serial port or ip and "
				"port provided in 'm_com_port','m_ip_dir' and 'm_port_dir'");
		}

		if (!m_ip_dir.empty())
		{
			// Try to open the serial port:
			auto* theCOM = new CClientTCPSocket();

			MRPT_LOG_INFO_STREAM(
				__CURRENT_FUNCTION_NAME__ << " Connecting to " << m_ip_dir
										  << ":" << m_port_dir);
			theCOM->connect(m_ip_dir, m_port_dir);

			if (!theCOM->isConnected())
			{
				MRPT_LOG_ERROR_STREAM(
					__CURRENT_FUNCTION_NAME__
					<< " Cannot connect with the server '" << m_com_port
					<< "'");
				delete theCOM;
				return false;
			}

			// Bind:
			bindIO(theCOM);

			m_I_am_owner_serial_port = true;
		}
		else
		{
			// Try to open the serial port:
			auto* theCOM = new CSerialPort(m_com_port, true);

			if (!theCOM->isOpen())
			{
				MRPT_LOG_ERROR_STREAM(
					__CURRENT_FUNCTION_NAME__ << " Cannot open serial port '"
											  << m_com_port << "'");
				delete theCOM;
				return false;
			}

			// Bind:
			bindIO(theCOM);

			m_I_am_owner_serial_port = true;
		}

		return true;
	}
	MRPT_END
}

void CHokuyoURG::initialize()
{
	if (m_verbose) this->setMinLoggingLevel(mrpt::system::LVL_DEBUG);

	if (!ensureStreamIsOpen()) return;

	if (!turnOn())
	{
		MRPT_LOG_ERROR("[Hokuyo] Error initializing HOKUYO scanner");
		return;
	}
}

void CHokuyoURG::purgeBuffers()
{
	if (!ensureStreamIsOpen()) return;

	if (m_ip_dir.empty())
	{
		auto* COM = dynamic_cast<CSerialPort*>(m_stream);
		if (COM != nullptr)
		{
			COM->purgeBuffers();
		}
	}
	else  // Socket connection
	{
		auto* COM = dynamic_cast<CClientTCPSocket*>(m_stream);

		size_t to_read = COM->getReadPendingBytes();

		if (to_read)
		{
			void* buf = malloc(sizeof(uint8_t) * to_read);

			size_t nRead = m_stream->Read(buf, to_read);

			if (nRead != to_read)
				THROW_EXCEPTION(
					"Error in purge buffers: read and expected number of bytes "
					"are different.");

			free(buf);
		}
	}
}
