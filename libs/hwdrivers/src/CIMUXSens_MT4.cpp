/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers


#include <mrpt/system/threads.h>
#include <mrpt/hwdrivers/CIMUXSens_MT4.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationGPS.h>

IMPLEMENTS_GENERIC_SENSOR(CIMUXSens_MT4,mrpt::hwdrivers)

using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::hwdrivers;
using namespace std;

#if MRPT_HAS_xSENS_MT4
	/* Copyright (c) Xsens Technologies B.V., 2006-2012. All rights reserved.

		  This source code is provided under the MT SDK Software License Agreement
	and is intended for use only by Xsens Technologies BV and
		   those that have explicit written permission to use it from
		   Xsens Technologies BV.

		  THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
		   KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
		   IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
		   PARTICULAR PURPOSE.
	 */
	#include <xsens/xsresultvalue.h>
	#include <xsens/xsbytearray.h>
	#include <xsens/xsmessagearray.h>
	#include <xsens/xsdeviceid.h>
	#include <xsens/xsportinfo.h>
	#include <xsens/xsoutputmode.h>
	#include <xsens/xsoutputsettings.h>
	#include <xsens/xsoutputconfigurationarray.h>
	#include <xsens/protocolhandler.h>
	#include <xsens/usbinterface.h>
	#include <xsens/serialinterface.h>
	#include <xsens/streaminterface.h>
	#include <xsens/xsportinfoarray.h>
	#include <xsens/xsdatapacket.h>
	#include <xsens/xsstatusflag.h>
	#include <xsens/xstime.h>
	#include <xsens/legacydatapacket.h>
	#include <xsens/int_xsdatapacket.h>
	#include <xsens/enumerateusbdevices.h>

	class DeviceClass
	{
	public:
		DeviceClass() : m_streamInterface(NULL) { }
		~DeviceClass()
		{
			if (m_streamInterface) {
				delete m_streamInterface;
				m_streamInterface=NULL;
			}
		}

		/*! \brief Open an IO device
			\param portInfo The info to use for opening the port
			\return True when successful
		*/
		bool openPort(const XsPortInfo& portInfo)
		{
			if (portInfo.isUsb())
				m_streamInterface = new UsbInterface();
			else
				m_streamInterface = new SerialInterface();

			if (m_streamInterface->open(portInfo) != XRV_OK)
				return false;
			return true;
		}

		/*! \brief Close an IO device
		*/
		void close()
		{
			if (m_streamInterface)
				m_streamInterface->close();
		}

		/*! \brief Read available data from the open IO device
			\details This function will attempt to read all available data from the open device (COM port
			or USB port).
			The function will read from the device, but it won't wait for data to become available.
			\param raw A XsByteArray to where the read data will be stored.
			\return Whether data has been read from the IO device
		*/
		XsResultValue readDataToBuffer(XsByteArray& raw)
		{
			// always read data and append it to the cache before doing analysis
			const int maxSz = 8192;
			XsResultValue res = m_streamInterface->readData(maxSz, raw);
			if (raw.size())
				return XRV_OK;

			return res;
		}

		/*! \brief Read all messages from the buffered read data after adding new data supplied in \a rawIn
			\details This function will read all present messages in the read buffer. In order for this function
			to work, you need to call readDataToBuffer() first.
			\param rawIn The buffered data in which to search for messages
			\param messages The messages found in the data
			\return The messages that were read.
		*/
		XsResultValue processBufferedData(XsByteArray& rawIn, XsMessageArray& messages)
		{
			ProtocolHandler protocol;

			if (rawIn.size())
				m_dataBuffer.append(rawIn);

			int popped = 0;
			messages.clear();

			for(;;)
			{
				XsByteArray raw(m_dataBuffer.data()+popped, m_dataBuffer.size()-popped);
				XsMessage message;
				MessageLocation location = protocol.findMessage(message, raw);

				if (location.isValid())
				{
					// message is valid, remove data from cache
					popped += location.m_size + location.m_startPos;
					messages.push_back(message);
				}
				else
				{
					if (popped)
						m_dataBuffer.pop_front(popped);

					if (messages.empty())
						return XRV_TIMEOUTNODATA;

					return XRV_OK;
				}
			}
		}

		/*! \brief Wait for the requested XsXbusMessageId
			\param xmid The message id to wait for
			\param rcv  The received message
			\return Whether the requested message was found
		*/
		bool waitForMessage(XsXbusMessageId xmid, XsMessage& rcv)
		{
			XsByteArray data;
			XsMessageArray msgs;
			bool foundAck = false;
			do
			{
				readDataToBuffer(data);
				processBufferedData(data, msgs);
				for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it)
					if ((*it).getMessageId() == xmid)
					{
						foundAck = true;
						rcv = *it;
					}
			} while (!foundAck);
			return foundAck;
		}

		/*! \brief Write a message to the IO device
			\param msg The message to write
			\return Whether the message could be written
		*/
		bool writeMessage(const XsMessage& msg)
		{
			XsByteArray raw;
			if (ProtocolHandler::composeMessage(raw, msg) < 0)
				return false;

			return (m_streamInterface->writeData(raw) == XRV_OK);
		}

		/*! \brief Put a device in config mode
			\return True when the device acknowledged config mode
		*/
		bool gotoConfig()
		{
			XsMessage snd(XMID_GotoConfig, 0), rcv;
			writeMessage(snd);

			return waitForMessage(XMID_GotoConfigAck, rcv);
		}

		/*! \brief Put a device in measurement mode
			\return True when the device acknowledged measurement mode
		*/
		bool gotoMeasurement()
		{
			XsMessage snd(XMID_GotoMeasurement, 0), rcv;
			writeMessage(snd);

			return waitForMessage(XMID_GotoMeasurementAck, rcv);
		}

		/*! \brief Request the product code from a device
			\return The product code when ok, otherwise an empty XsString
		*/
		XsString getProductCode()
		{
			XsMessage snd(XMID_ReqProductCode, 0), rcv;
			writeMessage(snd);

			if (waitForMessage(XMID_ProductCode,rcv))
			{
				const char* pc = (const char*) rcv.getDataBuffer(0);
				std::string result(pc?pc:"", rcv.getDataSize());
				std::string::size_type thingy = result.find(" ");
				if (thingy < 20)
					result.erase(result.begin() + thingy, result.end());	//lint !e534
				return XsString(result);
			}
			else
				return XsString();
		}

		/*! \brief Request the device id from a device
			\return The device id (XsDeviceId) when ok, otherwise an empty XsDeviceId
		*/
		XsDeviceId getDeviceId()
		{
			XsMessage snd(XMID_ReqDid, 0), rcv;
			writeMessage(snd);

			if (waitForMessage(XMID_DeviceId,rcv))
			{
				return rcv.getDataLong();
			}
			else
				return XsDeviceId();
		}

		/*! \brief Set the device mode of a device (outputmode and outputsettings)
			\param outputMode The XsOutputMode to set
			\param outputSettings The XsOutputSettings to set
			\return True when successful
		*/
		bool setDeviceMode(const XsOutputMode& outputMode, const XsOutputSettings& outputSettings)
		{
			XsMessage sndOM(XMID_SetOutputMode), sndOS(XMID_SetOutputSettings), rcv;

			sndOM.resizeData(2);
			sndOM.setDataShort((uint16_t) outputMode);
			writeMessage(sndOM);
			if (!waitForMessage(XMID_SetOutputModeAck, rcv))
				return false;

			XsMessage snd(XMID_SetOutputSettings);
			snd.resizeData(4);
			snd.setDataLong((uint32_t)outputSettings);
			writeMessage(sndOS);
			if (!waitForMessage(XMID_SetOutputSettingsAck, rcv))
				return false;

			return true;
		}

		/*! \brief Set the output configuration of a device
			\param config An array XsOutputConfigurationArray) containing the one or multiple XsOutputConfigurations
			\return True when successful
		*/
		bool setOutputConfiguration(XsOutputConfigurationArray& config)
		{
			XsMessage snd(XMID_SetOutputConfiguration, 4), rcv;
			if (config.size() == 0)
			{
				snd.setDataShort((uint16_t)XDI_None, 0);
				snd.setDataShort(0, 2);
			}
			else
			{
				for (XsSize i = 0; i < (XsSize) config.size(); ++i)
				{
					snd.setDataShort((uint16_t)config[i].m_dataIdentifier, i*4);
					snd.setDataShort(config[i].m_frequency, i*4+2);
				}
			}
			writeMessage(snd);

			return waitForMessage(XMID_SetOutputConfigurationAck, rcv);
		}




	private:
		StreamInterface *m_streamInterface;
		XsByteArray m_dataBuffer;
	};
#endif

// Adaptors for the "void*" memory blocks:
#define my_xsens_device  (*static_cast<DeviceClass*>(m_dev_ptr))
#define my_xsens_devid   (*static_cast<XsDeviceId*>(m_devid_ptr))

// Include libraries in linking:
#if MRPT_HAS_xSENS_MT4
	#ifdef MRPT_OS_WINDOWS
		// WINDOWS:
		#if defined(_MSC_VER) || defined(__BORLANDC__)
			#pragma comment (lib,"SetupAPI.lib")
			#pragma comment (lib,"WinUsb.lib")
		#endif
	#endif	// MRPT_OS_WINDOWS
#endif // MRPT_HAS_xSENS_MT4

/*-------------------------------------------------------------
					CIMUXSens_MT4
-------------------------------------------------------------*/
CIMUXSens_MT4::CIMUXSens_MT4( ) :
	m_port_bauds    (0),
	m_portname      (),
	m_sampleFreq    (100),
	m_timeStartUI   (0),
	m_timeStartTT   (0),
	m_sensorPose    (),
	m_dev_ptr       (NULL),
	m_devid_ptr     (NULL)
{
	m_sensorLabel = "XSensMTi_MT4";

#if MRPT_HAS_xSENS_MT4
    m_dev_ptr  = new DeviceClass;
    m_devid_ptr  = new XsDeviceId;
#else
	THROW_EXCEPTION("MRPT has been compiled with 'BUILD_XSENS_MT4'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
					~CIMUXSens_MT4
-------------------------------------------------------------*/
CIMUXSens_MT4::~CIMUXSens_MT4()
{
#if MRPT_HAS_xSENS_MT4
	my_xsens_device.close();
	delete static_cast<DeviceClass*>(m_dev_ptr);
	m_dev_ptr=NULL;

	delete static_cast<XsDeviceId*>(m_devid_ptr);
	m_devid_ptr=NULL;
#endif
}

/*-------------------------------------------------------------
					doProcess
-------------------------------------------------------------*/
void CIMUXSens_MT4::doProcess()
{
#if MRPT_HAS_xSENS_MT4
	if(m_state == ssError)
	{
		mrpt::system::sleep(200);
		initialize();
	}

	if(m_state == ssError)
		return;

	XsByteArray data;
	XsMessageArray msgs;

	my_xsens_device.readDataToBuffer(data);
	my_xsens_device.processBufferedData(data, msgs);
	for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it)
	{
		// Retrieve a packet
		XsDataPacket packet;
		if ((*it).getMessageId() == XMID_MtData)
		{
			LegacyDataPacket lpacket(1, false);

			lpacket.setMessage((*it));
			lpacket.setXbusSystem(false, false);
			lpacket.setDeviceId(my_xsens_devid, 0);
			lpacket.setDataFormat(XOM_Orientation, XOS_OrientationMode_Euler | XOS_Timestamp_PacketCounter | XOS_CalibratedMode_All/*XOS_OrientationMode_Quaternion*/,0);	//lint !e534
			XsDataPacket_assignFromXsLegacyDataPacket(&packet, &lpacket, 0);
		}
		else if ((*it).getMessageId() == XMID_MtData2) {
			packet.setMessage((*it));
			packet.setDeviceId(my_xsens_devid);
		}

		// Data properly collected: extract data fields
		// -------------------------------------------------
		m_state		= ssWorking;
		CObservationIMUPtr obs			= CObservationIMU::Create();

		if (packet.containsOrientation())
		{
			XsEuler euler = packet.orientationEuler();
			obs->rawMeasurements[IMU_YAW]   = DEG2RAD(euler.yaw());   obs->dataIsPresent[IMU_YAW] = true;
			obs->rawMeasurements[IMU_PITCH] = DEG2RAD(euler.pitch()); obs->dataIsPresent[IMU_PITCH]   = true;
			obs->rawMeasurements[IMU_ROLL]  = DEG2RAD(euler.roll());  obs->dataIsPresent[IMU_ROLL]  = true;

			XsQuaternion quat = packet.orientationQuaternion();
			obs->rawMeasurements[IMU_ORI_QUAT_X]   = quat.x();   obs->dataIsPresent[IMU_ORI_QUAT_X] = true;
			obs->rawMeasurements[IMU_ORI_QUAT_Y]   = quat.y();   obs->dataIsPresent[IMU_ORI_QUAT_Y] = true;
			obs->rawMeasurements[IMU_ORI_QUAT_Z]   = quat.z();   obs->dataIsPresent[IMU_ORI_QUAT_Z] = true;
			obs->rawMeasurements[IMU_ORI_QUAT_W]   = quat.w();   obs->dataIsPresent[IMU_ORI_QUAT_W] = true;
		}

		if (packet.containsCalibratedAcceleration())
		{
			XsVector acc_data = packet.calibratedAcceleration();
			obs->rawMeasurements[IMU_X_ACC] = acc_data[0]; obs->dataIsPresent[IMU_X_ACC] = true;
			obs->rawMeasurements[IMU_Y_ACC] = acc_data[1]; obs->dataIsPresent[IMU_Y_ACC] = true;
			obs->rawMeasurements[IMU_Z_ACC] = acc_data[2]; obs->dataIsPresent[IMU_Z_ACC] = true;
		}

		if (packet.containsCalibratedGyroscopeData())
		{
			XsVector gyr_data = packet.calibratedGyroscopeData();
			obs->rawMeasurements[IMU_YAW_VEL]   = gyr_data[2]; obs->dataIsPresent[IMU_YAW_VEL]   = true;
			obs->rawMeasurements[IMU_PITCH_VEL] = gyr_data[1]; obs->dataIsPresent[IMU_PITCH_VEL] = true;
			obs->rawMeasurements[IMU_ROLL_VEL]  = gyr_data[0]; obs->dataIsPresent[IMU_ROLL_VEL]  = true;
		}

		if (packet.containsCalibratedMagneticField())
		{
			XsVector mag_data = packet.calibratedMagneticField();
			obs->rawMeasurements[IMU_MAG_X]   = mag_data[0]; obs->dataIsPresent[IMU_MAG_X]   = true;
			obs->rawMeasurements[IMU_MAG_Y]   = mag_data[1]; obs->dataIsPresent[IMU_MAG_Y]   = true;
			obs->rawMeasurements[IMU_MAG_Z]   = mag_data[2]; obs->dataIsPresent[IMU_MAG_Z]   = true;
		}

		if (packet.containsVelocity())
		{
			XsVector vel_data = packet.velocity();
			obs->rawMeasurements[IMU_X_VEL]   = vel_data[0]; obs->dataIsPresent[IMU_X_VEL]   = true;
			obs->rawMeasurements[IMU_Y_VEL]   = vel_data[1]; obs->dataIsPresent[IMU_Y_VEL]   = true;
			obs->rawMeasurements[IMU_Z_VEL]   = vel_data[2]; obs->dataIsPresent[IMU_Z_VEL]   = true;
		}

		if (packet.containsTemperature())
		{
			obs->rawMeasurements[IMU_TEMPERATURE]   = packet.temperature(); obs->dataIsPresent[IMU_TEMPERATURE]   = true;
		}

		if (packet.containsAltitude())
		{
			obs->rawMeasurements[IMU_ALTITUDE ]   = packet.altitude(); obs->dataIsPresent[IMU_ALTITUDE ]   = true;
		}

		// TimeStamp
		if (packet.containsSampleTime64())
		{
			const uint64_t  nowUI = packet.sampleTime64();

			uint64_t AtUI = 0;
			if( m_timeStartUI == 0 )
			{
				m_timeStartUI = nowUI;
				m_timeStartTT = mrpt::system::now();
			}
			else
				AtUI	= nowUI - m_timeStartUI;

			double AtDO	= AtUI * 1000.0;								// Difference in intervals of 100 nsecs
			obs->timestamp		= m_timeStartTT	+ AtDO;
		}
		else if (packet.containsUtcTime())
		{
			XsUtcTime utc = packet.utcTime();

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
		else obs->timestamp		= mrpt::system::now();

		obs->sensorPose		= m_sensorPose;
		obs->sensorLabel	= m_sensorLabel;

		appendObservation(obs);

		if (packet.containsLatitudeLongitude())
		{
			XsVector lla_data = packet.latitudeLongitude();

			CObservationGPSPtr obsGPS = CObservationGPSPtr( new CObservationGPS() );
			gnss::Message_NMEA_RMC rGPSs;
			gnss::Message_NMEA_RMC::content_t & rGPS = rGPSs.fields;
			rGPS.latitude_degrees = lla_data[0];
			rGPS.longitude_degrees = lla_data[1];

			if (packet.containsStatus() && packet.status() & XSF_GpsValid)
				rGPS.validity_char = 'A';
			else
				rGPS.validity_char = 'V';

			if (packet.containsUtcTime())
			{
				XsUtcTime utc = packet.utcTime();
				rGPS.UTCTime.hour = utc.m_hour;
				rGPS.UTCTime.minute = utc.m_minute;
				rGPS.UTCTime.sec = utc.m_second + (utc.m_nano * 1000000.0);
			}
			else
			{
				rGPS.UTCTime.hour = ((obs->timestamp / (60 * 60 * ((uint64_t)1000000 / 100))) % 24);
				rGPS.UTCTime.minute = ((obs->timestamp / (60 * ((uint64_t)1000000 / 100))) % 60);
				rGPS.UTCTime.sec = fmod(obs->timestamp / (1000000.0 / 100), 60);
			}

			if (packet.containsVelocity())
			{
				XsVector vel_data = packet.velocity();

				rGPS.speed_knots = sqrt(vel_data[0] * vel_data[0] + vel_data[1] * vel_data[1]);
				rGPS.direction_degrees = 0; //Could be worked out from velocity and magnatic field perhaps.
			}
			else rGPS.speed_knots = rGPS.direction_degrees = 0;

			obsGPS->setMsg(rGPSs);
			obsGPS->timestamp = obs->timestamp;
			obsGPS->originalReceivedTimestamp = obs->timestamp;
			obsGPS->has_satellite_timestamp = false;
			obsGPS->sensorPose	= m_sensorPose;
			obsGPS->sensorLabel	= m_sensorLabel;

			appendObservation(obsGPS);
		}

		std::cout << std::flush;
	}
	msgs.clear();

#else
	THROW_EXCEPTION("MRPT has been compiled with 'BUILD_XSENS_MT4'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
					initialize
-------------------------------------------------------------*/
void CIMUXSens_MT4::initialize()
{
#if MRPT_HAS_xSENS_MT4
	m_state = ssInitializing;

	try
	{
		// Try to open a specified device, or scan the bus?
		XsPortInfoArray portInfoArray;

		if (m_portname.empty())
		{
			if (m_verbose) cout << "[CIMUXSens_MT4] Scanning for USB devices...\n";
			xsEnumerateUsbDevices(portInfoArray);

			if (portInfoArray.empty())
				THROW_EXCEPTION("CIMUXSens_MT4: No 'portname' was specified and no XSens device was found after scanning the system!")

			if (m_verbose) cout << "[CIMUXSens_MT4] Found " <<  portInfoArray.size() <<" devices. Opening the first one.\n";
		}
		else
		{
			XsPortInfo portInfo(m_portname, XsBaud::numericToRate(m_port_bauds));
			if (m_verbose) cout << "[CIMUXSens_MT4] Using user-supplied portname '"<<m_portname<<"' at "<<m_port_bauds<<" baudrate.\n";
			portInfoArray.push_back(portInfo);
		}

		// Use the first detected device
		XsPortInfo mtPort = portInfoArray.at(0);

		// Open the port with the detected device
		cout << "[CIMUXSens_MT4] Opening port " << mtPort.portName().toStdString() << std::endl;

		if (!my_xsens_device.openPort(mtPort))
			throw std::runtime_error("Could not open port. Aborting.");

		// Put the device in configuration mode
		if (m_verbose) cout << "[CIMUXSens_MT4] Putting device into configuration mode...\n";
		if (!my_xsens_device.gotoConfig()) // Put the device into configuration mode before configuring the device
			throw std::runtime_error("Could not put device into configuration mode. Aborting.");

		// Request the device Id to check the device type
		mtPort.setDeviceId(my_xsens_device.getDeviceId());

		my_xsens_devid = mtPort.deviceId();

		// Check if we have an MTi / MTx / MTmk4 device
		if (!mtPort.deviceId().isMtix() && !mtPort.deviceId().isMtMk4())
		{
			throw std::runtime_error("No MTi / MTx / MTmk4 device found. Aborting.");
		}
		cout << "[CIMUXSens_MT4] Found a device with id: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << std::endl;

		// Print information about detected MTi / MTx / MTmk4 device
		if (m_verbose) cout << "[CIMUXSens_MT4] Device: " << my_xsens_device.getProductCode().toStdString() << " opened." << std::endl;

		// Configure the device. Note the differences between MTix and MTmk4
		if (m_verbose) cout << "[CIMUXSens_MT4] Configuring the device..." << std::endl;
		if (mtPort.deviceId().isMtix())
		{
			XsOutputMode outputMode = XOM_Orientation; // output orientation data
			XsOutputSettings outputSettings = XOS_OrientationMode_Euler | XOS_Timestamp_PacketCounter | XOS_CalibratedMode_All; // XOS_OrientationMode_Quaternion; // output orientation data as quaternion

			// set the device configuration
			if (!my_xsens_device.setDeviceMode(outputMode, outputSettings))
				throw std::runtime_error("Could not configure MT device. Aborting.");
		}
		else if (mtPort.deviceId().isMtMk4())
		{
			XsOutputConfigurationArray configArray;
			configArray.push_back( XsOutputConfiguration(XDI_SampleTime64,m_sampleFreq) );
			configArray.push_back( XsOutputConfiguration(XDI_SampleTimeFine,m_sampleFreq) );
			configArray.push_back( XsOutputConfiguration(XDI_SampleTimeCoarse,m_sampleFreq) );
			configArray.push_back( XsOutputConfiguration(XDI_Quaternion,m_sampleFreq) );
			configArray.push_back( XsOutputConfiguration(XDI_Temperature,m_sampleFreq) );
			configArray.push_back( XsOutputConfiguration(XDI_Acceleration,m_sampleFreq) );
			configArray.push_back( XsOutputConfiguration(XDI_RateOfTurn,m_sampleFreq) );
			configArray.push_back( XsOutputConfiguration(XDI_MagneticField,m_sampleFreq) );
			configArray.push_back( XsOutputConfiguration(XDI_VelocityXYZ,m_sampleFreq) );

			configArray.push_back( XsOutputConfiguration(XDI_StatusByte, m_sampleFreq) );
			configArray.push_back( XsOutputConfiguration(XDI_LatLon, m_sampleFreq) );
			configArray.push_back( XsOutputConfiguration(XDI_UtcTime, m_sampleFreq) );
			configArray.push_back( XsOutputConfiguration(XDI_AltitudeEllipsoid, m_sampleFreq) );

			if (!my_xsens_device.setOutputConfiguration(configArray))
				throw std::runtime_error("Could not configure MTmk4 device. Aborting.");
		}
		else
		{
			throw std::runtime_error("Unknown device while configuring. Aborting.");
		}

		// Put the device in measurement mode
		if (m_verbose) cout << "[CIMUXSens_MT4] Putting device into measurement mode..." << std::endl;
		if (!my_xsens_device.gotoMeasurement())
			throw std::runtime_error("Could not put device into measurement mode. Aborting.");

		m_state = ssWorking;

	}
	catch(std::exception &)
	{
		m_state = ssError;
		std::cerr << "Error Could not initialize the device" << std::endl;
		throw;
	}

#else
	THROW_EXCEPTION("MRPT has been compiled with 'BUILD_XSENS_MT4'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
					loadConfig_sensorSpecific
-------------------------------------------------------------*/
void  CIMUXSens_MT4::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string	  &iniSection )
{
	m_sensorPose.setFromValues(
        configSource.read_float( iniSection, "pose_x", 0, false ),
        configSource.read_float( iniSection, "pose_y", 0, false ),
        configSource.read_float( iniSection, "pose_z", 0, false ),
        DEG2RAD( configSource.read_float( iniSection, "pose_yaw", 0, false ) ),
        DEG2RAD( configSource.read_float( iniSection, "pose_pitch", 0, false ) ),
        DEG2RAD( configSource.read_float( iniSection, "pose_roll", 0, false ) ) );

	m_sampleFreq = configSource.read_int(iniSection, "sampleFreq", m_sampleFreq, false );

	m_port_bauds = configSource.read_int(iniSection, "baudRate", m_port_bauds, false );

#ifdef MRPT_OS_WINDOWS
	m_portname = configSource.read_string(iniSection, "portname_WIN", m_portname, false );
#else
	m_portname = configSource.read_string(iniSection, "portname_LIN", m_portname, false );
#endif


}
