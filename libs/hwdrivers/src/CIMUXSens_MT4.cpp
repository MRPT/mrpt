/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"  // Precompiled headers

#include <mrpt/hwdrivers/CIMUXSens_MT4.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>

#include <iostream>
#include <thread>

#if !MRPT_HAS_xSENS
namespace mrpt::hwdrivers
{
struct CIMUXSens_MT4::Impl
{
};
}  // namespace mrpt::hwdrivers
#else

// Tell MSVC that we are not using MTSDK as DLL:
#define XDA_STATIC_LIB

#include <xscontroller/xscallback.h>
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsportinfo.h>
#include <xstypes/xsstatusflag.h>

// MTSDK expects this global symbol as "extern":
Journaller* gJournal = 0;

namespace mrpt::hwdrivers
{
class MyXSensCallback : public XsCallback
{
   public:
	CIMUXSens_MT4* me = nullptr;

   private:
	//	uint64_t m_timeStartUI = 0;
	mrpt::system::TTimeStamp m_timeStartTT;

   protected:
	void onLiveDataAvailable(XsDevice*, const XsDataPacket* ptrpacket) override
	{
		using namespace mrpt::obs;

		if (!me) return;
		if (!ptrpacket) return;
		const XsDataPacket& packet = *ptrpacket;

		// Data properly collected: extract data fields
		// -------------------------------------------------
		me->m_state = mrpt::hwdrivers::CGenericSensor::ssWorking;
		CObservationIMU::Ptr obs = std::make_shared<CObservationIMU>();

		if (packet.containsOrientation())
		{
			XsEuler euler = packet.orientationEuler();
			obs->rawMeasurements[IMU_YAW] = DEG2RAD(euler.yaw());
			obs->dataIsPresent[IMU_YAW] = true;
			obs->rawMeasurements[IMU_PITCH] = DEG2RAD(euler.pitch());
			obs->dataIsPresent[IMU_PITCH] = true;
			obs->rawMeasurements[IMU_ROLL] = DEG2RAD(euler.roll());
			obs->dataIsPresent[IMU_ROLL] = true;

			XsQuaternion quat = packet.orientationQuaternion();
			obs->rawMeasurements[IMU_ORI_QUAT_X] = quat.x();
			obs->dataIsPresent[IMU_ORI_QUAT_X] = true;
			obs->rawMeasurements[IMU_ORI_QUAT_Y] = quat.y();
			obs->dataIsPresent[IMU_ORI_QUAT_Y] = true;
			obs->rawMeasurements[IMU_ORI_QUAT_Z] = quat.z();
			obs->dataIsPresent[IMU_ORI_QUAT_Z] = true;
			obs->rawMeasurements[IMU_ORI_QUAT_W] = quat.w();
			obs->dataIsPresent[IMU_ORI_QUAT_W] = true;
		}

		if (packet.containsCalibratedAcceleration())
		{
			XsVector acc_data = packet.calibratedAcceleration();
			obs->rawMeasurements[IMU_X_ACC] = acc_data[0];
			obs->dataIsPresent[IMU_X_ACC] = true;
			obs->rawMeasurements[IMU_Y_ACC] = acc_data[1];
			obs->dataIsPresent[IMU_Y_ACC] = true;
			obs->rawMeasurements[IMU_Z_ACC] = acc_data[2];
			obs->dataIsPresent[IMU_Z_ACC] = true;
		}

		if (packet.containsCalibratedGyroscopeData())
		{
			XsVector gyr_data = packet.calibratedGyroscopeData();
			obs->rawMeasurements[IMU_YAW_VEL] = gyr_data[2];
			obs->dataIsPresent[IMU_YAW_VEL] = true;
			obs->rawMeasurements[IMU_PITCH_VEL] = gyr_data[1];
			obs->dataIsPresent[IMU_PITCH_VEL] = true;
			obs->rawMeasurements[IMU_ROLL_VEL] = gyr_data[0];
			obs->dataIsPresent[IMU_ROLL_VEL] = true;
		}

		if (packet.containsCalibratedMagneticField())
		{
			XsVector mag_data = packet.calibratedMagneticField();
			obs->rawMeasurements[IMU_MAG_X] = mag_data[0];
			obs->dataIsPresent[IMU_MAG_X] = true;
			obs->rawMeasurements[IMU_MAG_Y] = mag_data[1];
			obs->dataIsPresent[IMU_MAG_Y] = true;
			obs->rawMeasurements[IMU_MAG_Z] = mag_data[2];
			obs->dataIsPresent[IMU_MAG_Z] = true;
		}

		if (packet.containsVelocity())
		{
			XsVector vel_data = packet.velocity();
			obs->rawMeasurements[IMU_X_VEL] = vel_data[0];
			obs->dataIsPresent[IMU_X_VEL] = true;
			obs->rawMeasurements[IMU_Y_VEL] = vel_data[1];
			obs->dataIsPresent[IMU_Y_VEL] = true;
			obs->rawMeasurements[IMU_Z_VEL] = vel_data[2];
			obs->dataIsPresent[IMU_Z_VEL] = true;
		}

		if (packet.containsTemperature())
		{
			obs->rawMeasurements[IMU_TEMPERATURE] = packet.temperature();
			obs->dataIsPresent[IMU_TEMPERATURE] = true;
		}

		if (packet.containsAltitude())
		{
			obs->rawMeasurements[IMU_ALTITUDE] = packet.altitude();
			obs->dataIsPresent[IMU_ALTITUDE] = true;
		}

		// TimeStamp
#if 0  // I can't find a generic conversion between sample time and seconds!
		if (packet.containsSampleTime64())
		{
		dev->getDataPacketByIndex()
			const uint64_t nowUI = packet.sampleTime64();
			std::cout << "nowUI: " << nowUI << "\n";

			uint64_t AtUI = 0;
			if (m_timeStartUI == 0)
			{
				m_timeStartUI = nowUI;
				m_timeStartTT = mrpt::system::now();
			}
			else
				AtUI = nowUI - m_timeStartUI;  // ms

			obs->timestamp = m_timeStartTT + std::chrono::milliseconds(AtUI);
		}
		else
		if (packet.containsUtcTime())
		{
			XsTimeInfo utc = packet.utcTime();

			mrpt::system::TTimeParts parts;

			parts.day_of_week = 0;
			parts.daylight_saving = 0;
			parts.year = utc.m_year;
			parts.month = utc.m_month;
			parts.day = utc.m_day;
			parts.hour = utc.m_hour;
			parts.minute = utc.m_minute;
			parts.second = utc.m_second + (utc.m_nano * 1000000000.0);

			obs->timestamp = mrpt::system::buildTimestampFromParts(parts);
		}
		else
#endif
		obs->timestamp = mrpt::system::now();

		obs->sensorPose = me->m_sensorPose;
		obs->sensorLabel = me->m_sensorLabel;

		me->appendObservation(obs);

		if (packet.containsLatitudeLongitude())
		{
			XsVector lla_data = packet.latitudeLongitude();

			CObservationGPS::Ptr obsGPS = CObservationGPS::Create();
			gnss::Message_NMEA_RMC rGPSs;
			gnss::Message_NMEA_RMC::content_t& rGPS = rGPSs.fields;
			rGPS.latitude_degrees = lla_data[0];
			rGPS.longitude_degrees = lla_data[1];

			if (packet.containsStatus() && packet.status() & XSF_GpsValid)
				rGPS.validity_char = 'A';
			else
				rGPS.validity_char = 'V';

			if (packet.containsUtcTime())
			{
				auto utc = packet.utcTime();
				rGPS.UTCTime.hour = utc.m_hour;
				rGPS.UTCTime.minute = utc.m_minute;
				rGPS.UTCTime.sec = utc.m_second + (utc.m_nano * 1000000.0);
			}
			else
			{
				rGPS.UTCTime.hour =
					((obs->timestamp.time_since_epoch().count() /
					  (60 * 60 * ((uint64_t)1000000 / 100))) %
					 24);
				rGPS.UTCTime.minute =
					((obs->timestamp.time_since_epoch().count() /
					  (60 * ((uint64_t)1000000 / 100))) %
					 60);
				rGPS.UTCTime.sec = fmod(
					obs->timestamp.time_since_epoch().count() /
						(1000000.0 / 100),
					60);
			}

			if (packet.containsVelocity())
			{
				XsVector vel_data = packet.velocity();

				rGPS.speed_knots =
					sqrt(vel_data[0] * vel_data[0] + vel_data[1] * vel_data[1]);
				rGPS.direction_degrees = 0;  // Could be worked out from
				// velocity and magnatic field
				// perhaps.
			}
			else
				rGPS.speed_knots = rGPS.direction_degrees = 0;

			obsGPS->setMsg(rGPSs);
			obsGPS->timestamp = obs->timestamp;
			obsGPS->originalReceivedTimestamp = obs->timestamp;
			obsGPS->has_satellite_timestamp = false;
			obsGPS->sensorPose = me->m_sensorPose;
			obsGPS->sensorLabel = me->m_sensorLabel;

			me->appendObservation(obsGPS);
		}
	}
};

struct CIMUXSens_MT4::Impl
{
	Impl() = default;

	XsControl* m_xscontrol = nullptr;
	XsDevice* m_device = nullptr;
	XsPortInfo m_port;
	std::shared_ptr<MyXSensCallback> myCallback =
		std::make_shared<MyXSensCallback>();
};

}  // namespace mrpt::hwdrivers

#endif

// Include libraries in linking:
#if MRPT_HAS_xSENS
#ifdef _WIN32
// WINDOWS:
#if defined(_MSC_VER)
#pragma comment(lib, "SetupAPI.lib")
#pragma comment(lib, "WinUsb.lib")
#endif
#endif  // _WIN32
#endif  // MRPT_HAS_xSENS

IMPLEMENTS_GENERIC_SENSOR(CIMUXSens_MT4, mrpt::hwdrivers)

using namespace mrpt::obs;
using namespace mrpt::hwdrivers;
using namespace std;

CIMUXSens_MT4::CIMUXSens_MT4() : m_impl(mrpt::make_impl<CIMUXSens_MT4::Impl>())
{
	m_sensorLabel = "XSensMTi_MT4";

#if MRPT_HAS_xSENS
	m_impl->m_xscontrol = XsControl::construct();
	m_impl->myCallback->me = this;

#else
	THROW_EXCEPTION(
		"MRPT has been compiled with 'BUILD_XSENS'=OFF, so this class "
		"cannot be used.");
#endif
}

CIMUXSens_MT4::~CIMUXSens_MT4()
{
#if MRPT_HAS_xSENS
	close();
	m_impl->m_xscontrol->destruct();
#endif
}

void CIMUXSens_MT4::close()
{
#if MRPT_HAS_xSENS
	if (m_impl->m_device != nullptr)
	{
		m_impl->m_device->stopRecording();
		m_impl->m_device->closeLogFile();
		m_impl->m_device->removeCallbackHandler(m_impl->myCallback.get());
	}
	m_impl->m_xscontrol->closePort(m_impl->m_port);

#endif
}

/*-------------------------------------------------------------
					doProcess
-------------------------------------------------------------*/
void CIMUXSens_MT4::doProcess()
{
#if MRPT_HAS_xSENS
	if (m_state == ssError)
	{
		std::this_thread::sleep_for(200ms);
		initialize();
	}

	// Main processing happens asynchronously via callbacks.

#else
	THROW_EXCEPTION(
		"MRPT has been compiled with 'BUILD_XSENS'=OFF, so this class "
		"cannot be used.");
#endif
}

/*-------------------------------------------------------------
					initialize
-------------------------------------------------------------*/
void CIMUXSens_MT4::initialize()
{
#if MRPT_HAS_xSENS
	m_state = ssInitializing;

	try
	{
		// Try to open a specified device, or scan the bus?
		XsPortInfo mtPort;

		if (m_portname.empty())
		{
			if (m_verbose)
				cout << "[CIMUXSens_MT4] Scanning for USB devices...\n";

			XsPortInfoArray portInfoArray = XsScanner::scanPorts();

			for (const auto& portInfo : portInfoArray)
			{
				if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
				{
					mtPort = portInfo;
					break;
				}
			}

			if (mtPort.empty())
				THROW_EXCEPTION_FMT(
					"CIMUXSens_MT4: No 'portname' was specified and no "
					"compatible XSens device was found in the system (%u "
					"devices connected)",
					static_cast<unsigned int>(portInfoArray.size()));

			if (m_verbose)
				cout << "[CIMUXSens_MT4] Found " << portInfoArray.size()
					 << " devices. Opening the first one.\n";
		}
		else
		{
			cout << "[CIMUXSens_MT4] Using user-supplied portname '"
				 << m_portname << "' at " << m_port_bauds << " baudrate.\n";

			mtPort = XsPortInfo(m_portname, XsBaud_numericToRate(m_port_bauds));
		}

		if (mtPort.empty()) THROW_EXCEPTION("No MTi device found");

		// Use the first detected device or the specified one:
		if (!m_deviceId.empty())
		{
			if (mtPort.deviceId().toString().c_str() != m_deviceId)
				THROW_EXCEPTION_FMT(
					"Device with ID: %s not found", m_deviceId.c_str());
		}

		std::cout << mrpt::format(
			"[CIMUXSens_MT4] Found a device with ID: %s @ port: %s, baudrate: "
			"%d",
			mtPort.deviceId().toString().toStdString().c_str(),
			mtPort.portName().toStdString().c_str(), mtPort.baudrate());

		if (!m_impl->m_xscontrol->openPort(
				mtPort.portName().toStdString(), mtPort.baudrate()))
			THROW_EXCEPTION("Could not open port");

		m_impl->m_device = m_impl->m_xscontrol->device(mtPort.deviceId());
		ASSERT_(m_impl->m_device != nullptr);

		std::cout << mrpt::format(
			"[CIMUXSens_MT4] Device: %s, with ID: %s opened.",
			m_impl->m_device->productCode().toStdString().c_str(),
			m_impl->m_device->deviceId().toString().c_str());

		m_impl->m_device->addCallbackHandler(m_impl->myCallback.get());

		// Put the device in configuration mode
		if (m_verbose)
			cout << "[CIMUXSens_MT4] Putting device into configuration "
					"mode...\n";

		if (!m_impl->m_device->gotoConfig())
			THROW_EXCEPTION("Could not go to config");

		// read EMTS and device config stored in .mtb file header.
		if (!m_impl->m_device->readEmtsAndDeviceConfiguration())
			THROW_EXCEPTION("Could not read device configuration");

		// Set configuration:
		// if (mtPort.deviceId().isMti())
		{
			XsOutputConfigurationArray configArray;
			configArray.push_back(
				XsOutputConfiguration(XDI_SampleTime64, m_sampleFreq));
			configArray.push_back(
				XsOutputConfiguration(XDI_SampleTimeFine, m_sampleFreq));
			configArray.push_back(
				XsOutputConfiguration(XDI_SampleTimeCoarse, m_sampleFreq));
			configArray.push_back(
				XsOutputConfiguration(XDI_Quaternion, m_sampleFreq));
			configArray.push_back(
				XsOutputConfiguration(XDI_Temperature, m_sampleFreq));
			configArray.push_back(
				XsOutputConfiguration(XDI_Acceleration, m_sampleFreq));
			configArray.push_back(
				XsOutputConfiguration(XDI_RateOfTurn, m_sampleFreq));
			configArray.push_back(
				XsOutputConfiguration(XDI_MagneticField, m_sampleFreq));
			configArray.push_back(
				XsOutputConfiguration(XDI_VelocityXYZ, m_sampleFreq));

			configArray.push_back(
				XsOutputConfiguration(XDI_StatusByte, m_sampleFreq));
			configArray.push_back(
				XsOutputConfiguration(XDI_LatLon, m_sampleFreq));
			configArray.push_back(
				XsOutputConfiguration(XDI_UtcTime, m_sampleFreq));
			configArray.push_back(
				XsOutputConfiguration(XDI_AltitudeEllipsoid, m_sampleFreq));

			if (!m_impl->m_device->setOutputConfiguration(configArray))
				throw std::runtime_error(
					"Could not configure MTi device. Aborting.");
		}

		// Put the device in measurement mode
		if (m_verbose)
			cout << "[CIMUXSens_MT4] Putting device into measurement mode..."
				 << std::endl;

		if (!m_impl->m_device->gotoMeasurement())
			THROW_EXCEPTION("Could not put device into measurement mode");

		if (!m_xsensLogFile.empty())
		{
			if (m_impl->m_device->createLogFile(m_xsensLogFile) != XRV_OK)
				THROW_EXCEPTION_FMT(
					"Failed to create a log file! (%s)",
					m_xsensLogFile.c_str());
			else
				printf(
					"[CIMUXSens_MT4] Created a log file: %s",
					m_xsensLogFile.c_str());

			if (!m_impl->m_device->startRecording())
				THROW_EXCEPTION("Could not start recording");
		}

		m_state = ssWorking;
	}
	catch (std::exception&)
	{
		m_state = ssError;
		std::cerr << "[CIMUXSens_MT4] Error Could not initialize the device"
				  << std::endl;
		throw;
	}

#else
	THROW_EXCEPTION(
		"MRPT has been compiled with 'BUILD_XSENS'=OFF, so this class "
		"cannot be used.");
#endif
}

/*-------------------------------------------------------------
					loadConfig_sensorSpecific
-------------------------------------------------------------*/
void CIMUXSens_MT4::loadConfig_sensorSpecific(
	const mrpt::config::CConfigFileBase& c, const std::string& s)
{
	m_sensorPose.setFromValues(
		c.read_float(s, "pose_x", 0, false),
		c.read_float(s, "pose_y", 0, false),
		c.read_float(s, "pose_z", 0, false),
		DEG2RAD(c.read_float(s, "pose_yaw", 0, false)),
		DEG2RAD(c.read_float(s, "pose_pitch", 0, false)),
		DEG2RAD(c.read_float(s, "pose_roll", 0, false)));

	m_sampleFreq = c.read_int(s, "sampleFreq", m_sampleFreq, false);
	m_port_bauds = c.read_int(s, "baudRate", m_port_bauds, false);
	m_deviceId = c.read_string(s, "deviceId", m_deviceId, false);
	m_xsensLogFile = c.read_string(s, "logFile", m_xsensLogFile, false);

#ifdef _WIN32
	m_portname = c.read_string(s, "portname_WIN", m_portname, false);
#else
	m_portname = c.read_string(s, "portname_LIN", m_portname, false);
#endif
}
