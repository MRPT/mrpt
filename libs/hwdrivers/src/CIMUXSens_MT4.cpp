/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hwdrivers.h> // Precompiled headers


#include <mrpt/system/threads.h>
#include <mrpt/hwdrivers/CIMUXSens_MT4.h>
#include <mrpt/slam/CObservationIMU.h>

IMPLEMENTS_GENERIC_SENSOR(CIMUXSens_MT4,mrpt::hwdrivers)

using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::hwdrivers;

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
	#include <xsens/xstime.h>
	#include <xsens/legacydatapacket.h>
	#include <xsens/int_xsdatapacket.h>
	#include <xsens/enumerateusbdevices.h>

	class DeviceClass
	{
	public:
		DeviceClass() { }
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
//#define cmt3	    (*static_cast<xsens::Cmt3*>(m_cmt3_ptr))
//#define deviceId    (*static_cast<CmtDeviceId*>(m_deviceId_ptr))


// Include libraries in linking:
#if MRPT_HAS_xSENS_MT4
	#ifdef MRPT_OS_WINDOWS
		// WINDOWS:
		#if defined(_MSC_VER) || defined(__BORLANDC__)
			#pragma comment (lib,"SetupAPI.lib")
		#endif
	#endif	// MRPT_OS_WINDOWS
#endif // MRPT_HAS_xSENS_MT4

/*-------------------------------------------------------------
					CIMUXSens_MT4
-------------------------------------------------------------*/
CIMUXSens_MT4::CIMUXSens_MT4( ) :
	m_COMbauds		(0),
	m_com_port		(),
	m_timeStartUI	(0),
	m_timeStartTT	(0),
	m_sensorPose    (),
	m_cmt3_ptr	(NULL),
	m_deviceId_ptr	(NULL),
	m_toutCounter	(0)
{
	m_sensorLabel = "XSensMTi_MT4";



	DeviceClass device;

	try
	{
		// Scan for connected USB devices
		std::cout << "Scanning for USB devices..." << std::endl;
		XsPortInfoArray portInfoArray;
		xsEnumerateUsbDevices(portInfoArray);
		if (!portInfoArray.size())
		{
			std::string portName;
			int baudRate;
#ifdef WIN32
			std::cout << "No USB Motion Tracker found." << std::endl << std::endl << "Please enter COM port name (eg. COM1): " <<
#else
			std::cout << "No USB Motion Tracker found." << std::endl << std::endl << "Please enter COM port name (eg. /dev/ttyUSB0): " <<
#endif
			std::endl;
			std::cin >> portName;
			std::cout << "Please enter baud rate (eg. 115200): ";
			std::cin >> baudRate;

			XsPortInfo portInfo(portName, XsBaud::numericToRate(baudRate));
			portInfoArray.push_back(portInfo);
		}

		// Use the first detected device
		XsPortInfo mtPort = portInfoArray.at(0);

		// Open the port with the detected device
		std::cout << "Opening port: " << (&mtPort.portName()[0]) << std::endl;
		if (!device.openPort(mtPort))
			throw std::runtime_error("Could not open port. Aborting.");

		// Put the device in configuration mode
		std::cout << "Putting device into configuration mode..." << std::endl;
		if (!device.gotoConfig()) // Put the device into configuration mode before configuring the device
		{
			throw std::runtime_error("Could not put device into configuration mode. Aborting.");
		}

		// Request the device Id to check the device type
		mtPort.setDeviceId(device.getDeviceId());

		// Check if we have an MTi / MTx / MTmk4 device
		if (!mtPort.deviceId().isMtix() && !mtPort.deviceId().isMtMk4())
		{
			throw std::runtime_error("No MTi / MTx / MTmk4 device found. Aborting.");
		}
		std::cout << "Found a device with id: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << std::endl;

		try
		{
			// Print information about detected MTi / MTx / MTmk4 device
			std::cout << "Device: " << device.getProductCode().toStdString() << " opened." << std::endl;

			// Configure the device. Note the differences between MTix and MTmk4
			std::cout << "Configuring the device..." << std::endl;
			if (mtPort.deviceId().isMtix())
			{
				XsOutputMode outputMode = XOM_Orientation; // output orientation data
				XsOutputSettings outputSettings = XOS_OrientationMode_Quaternion; // output orientation data as quaternion

				// set the device configuration
				if (!device.setDeviceMode(outputMode, outputSettings))
				{
					throw std::runtime_error("Could not configure MT device. Aborting.");
				}
			}
			else if (mtPort.deviceId().isMtMk4())
			{
				XsOutputConfiguration quat(XDI_Quaternion, 100);
				XsOutputConfigurationArray configArray;
				configArray.push_back(quat);
				if (!device.setOutputConfiguration(configArray))
				{

					throw std::runtime_error("Could not configure MTmk4 device. Aborting.");
				}
			}
			else
			{
				throw std::runtime_error("Unknown device while configuring. Aborting.");
			}

			// Put the device in measurement mode
			std::cout << "Putting device into measurement mode..." << std::endl;
			if (!device.gotoMeasurement())
			{
				throw std::runtime_error("Could not put device into measurement mode. Aborting.");
			}

			std::cout << "\nMain loop (press any key to quit)" << std::endl;
			std::cout << std::string(79, '-') << std::endl;

			XsByteArray data;
			XsMessageArray msgs;
			while (! mrpt::system::os::kbhit())
			{
				device.readDataToBuffer(data);
				device.processBufferedData(data, msgs);
				for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it)
				{
					// Retrieve a packet
					XsDataPacket packet;
					if ((*it).getMessageId() == XMID_MtData) {
						LegacyDataPacket lpacket(1, false);
						lpacket.setMessage((*it));
						lpacket.setXbusSystem(false, false);
						lpacket.setDeviceId(mtPort.deviceId(), 0);
						lpacket.setDataFormat(XOM_Orientation, XOS_OrientationMode_Quaternion,0);	//lint !e534
						XsDataPacket_assignFromXsLegacyDataPacket(&packet, &lpacket, 0);
					}
					else if ((*it).getMessageId() == XMID_MtData2) {
						packet.setMessage((*it));
						packet.setDeviceId(mtPort.deviceId());
					}

					// Get the quaternion data
					XsQuaternion quaternion = packet.orientationQuaternion();
					std::cout << "\r"
							  << "W:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.m_w
							  << ",X:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.m_x
							  << ",Y:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.m_y
							  << ",Z:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.m_z
					;

					// Convert packet to euler
					XsEuler euler = packet.orientationEuler();
					std::cout << ",Roll:" << std::setw(7) << std::fixed << std::setprecision(2) << euler.m_roll
							  << ",Pitch:" << std::setw(7) << std::fixed << std::setprecision(2) << euler.m_pitch
							  << ",Yaw:" << std::setw(7) << std::fixed << std::setprecision(2) << euler.m_yaw
					;

					std::cout << std::flush;
				}
				msgs.clear();
				XsTime::msleep(0);
			}
			mrpt::system::os::getch();
			std::cout << "\n" << std::string(79, '-') << "\n";
			std::cout << std::endl;
		}
		catch (std::runtime_error const & error)
		{
			std::cout << error.what() << std::endl;
		}
		catch (...)
		{
			std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
		}

		// Close port
		std::cout << "Closing port..." << std::endl;
		device.close();
	}
	catch (std::runtime_error const & error)
	{
		std::cout << error.what() << std::endl;
	}
	catch (...)
	{
		std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
	}



#if 0 && MRPT_HAS_xSENS_MT4
    m_cmt3_ptr  = new xsens::Cmt3[1];
    m_deviceId_ptr = new CmtDeviceId[1];

#else
	THROW_EXCEPTION("MRPT has been compiled with 'BUILD_XSENS_MT4'=OFF, so this class cannot be used.");
#endif

}

/*-------------------------------------------------------------
					~CIMUXSens_MT4
-------------------------------------------------------------*/
CIMUXSens_MT4::~CIMUXSens_MT4()
{
#if 0 && MRPT_HAS_xSENS_MT4
	cmt3.closePort();

    delete[] &cmt3;     m_cmt3_ptr= NULL;
    delete[] &deviceId; m_deviceId_ptr = NULL;
#endif
}

/*-------------------------------------------------------------
					doProcess
-------------------------------------------------------------*/
void CIMUXSens_MT4::doProcess()
{
#if 0 && MRPT_HAS_xSENS_MT4

	if(m_state == ssError)
	{
		mrpt::system::sleep(200);
		initialize();
	}

	if(m_state == ssError)
		return;

	XsensResultValue	res;
	unsigned int		cont = 0;

	do
	{
		CmtTimeStamp		nowUI;	// ms

		xsens::Packet packet(1/*NDevices*/,cmt3.isXm()/*Is Bus master*/);

		res = cmt3.waitForDataMessage(&packet);

		if( res == XRV_OK )
		{
			// Data properly collected
			nowUI		= packet.getRtc();
			m_state		= ssWorking;

			CObservationIMUPtr obs			= CObservationIMU::Create();

			// ANGLE MEASUREMENTS:
			if ( packet.containsOriEuler() )
			{
				CmtEuler	euler_data	= packet.getOriEuler();

				obs->rawMeasurements[IMU_YAW]	= DEG2RAD(euler_data.m_yaw);
				obs->dataIsPresent[IMU_YAW]		= true;
				obs->rawMeasurements[IMU_PITCH] = DEG2RAD(euler_data.m_pitch);
				obs->dataIsPresent[IMU_PITCH]	= true;
				obs->rawMeasurements[IMU_ROLL]	= DEG2RAD(euler_data.m_roll);
				obs->dataIsPresent[IMU_ROLL]	= true;
			}

			// ACCELEROMETERS MEASUREMENTS:
			if ( packet.containsCalAcc())
			{
				CmtVector 	acc_data = packet.getCalAcc(); // getRawAcc();

				obs->rawMeasurements[IMU_X_ACC]	= acc_data.m_data[0];
				obs->dataIsPresent[IMU_X_ACC]	= true;
				obs->rawMeasurements[IMU_Y_ACC]	= acc_data.m_data[1];
				obs->dataIsPresent[IMU_Y_ACC]	= true;
				obs->rawMeasurements[IMU_Z_ACC]	= acc_data.m_data[2];
				obs->dataIsPresent[IMU_Z_ACC]	= true;
			}

			// GYROSCOPES MEASUREMENTS:
			if ( packet.containsCalGyr())
			{
				CmtVector gir_data	= packet.getCalGyr(); // getRawGyr();

				obs->rawMeasurements[IMU_YAW_VEL]	= gir_data.m_data[2];
				obs->dataIsPresent[IMU_YAW_VEL]	= true;
				obs->rawMeasurements[IMU_PITCH_VEL]	= gir_data.m_data[1];
				obs->dataIsPresent[IMU_PITCH_VEL]	= true;
				obs->rawMeasurements[IMU_ROLL_VEL]	= gir_data.m_data[0];
				obs->dataIsPresent[IMU_ROLL_VEL]	= true;
			}

			// TimeStamp
			uint64_t AtUI = 0;
			if( m_timeStartUI == 0 )
			{
				m_timeStartUI = nowUI;
				m_timeStartTT = mrpt::system::now();
			}
			else
				AtUI	= nowUI - m_timeStartUI;

			double AtDO	= AtUI * 10000.0;								// Difference in intervals of 100 nsecs
			obs->timestamp		= m_timeStartTT	+ AtDO;
			obs->sensorPose		= m_sensorPose;
			obs->sensorLabel	= m_sensorLabel;

			appendObservation(obs);
			m_toutCounter	= 0;

		} // end if XRV_OK

		if(res == XRV_TIMEOUT)
		{
			if(++m_toutCounter>3)
			{
				m_toutCounter	= 0;
				m_state			= ssError;
				if( cmt3.isPortOpen() )
					cmt3.closePort();

				std::cerr << "[CIMUXSens_MT4::doProcess()] Error: No data available [XRV_TIMEOUT]" << std::endl;
			}
		} // end if XRV_TIMEOUT

		if(res == XRV_TIMEOUTNODATA)
		{
//			m_state			= ssError;
//			m_timeStartUI	= 0;
//			if( cmt3.isPortOpen() )
//				cmt3.closePort();
//			std::cerr << "[CIMUXSens_MT4::doProcess()] Error: No data available [XRV_TIMEOUTNODATA]" << std::endl;
		} // end if XRV_TIMEOUTNODATA
	} while( res == XRV_OK && cont++ < 30);

#else
	THROW_EXCEPTION("MRPT has been compiled with 'BUILD_XSENS_MT4'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
					lookForPort
-------------------------------------------------------------*/
bool CIMUXSens_MT4::searchPortAndConnect()
{
#if 0 && MRPT_HAS_xSENS_MT4
	uint32_t baudrate;
	if(cmt3.getBaudrate(baudrate) == XRV_OK)
		return true;

	XsensResultValue res;
	xsens::List<CmtPortInfo> portInfo;
	unsigned long portCount = 0;
	unsigned short mtCount = 0;

	if( m_com_port.empty() ) {		// Scan COM ports
		std::cout << "Scanning for connected Xsens devices..." << std::endl;
		xsens::cmtScanPorts(portInfo);
		portCount = portInfo.length();
		std::cout << "Done" << std::endl;
		if (portCount == 0) {
			std::cout << "No xSens device found" << std::endl;
			m_state = ssError;
			return false;

		} // end if (error)
	} // end if
	else														// Port defined by user in .ini file
	{
		CmtPortInfo	pInfo;
		pInfo.m_baudrate	= m_COMbauds;
		strcpy( pInfo.m_portName, m_com_port.c_str());  //m_portNr		= (unsigned char)m_com_port;
		portInfo.append( pInfo );
		portCount++;
	} // end else

	ASSERT_(portCount == 1);
	std::cout << "Using COM port " << portInfo[0].m_portName /*(long)portInfo[0].m_portNr*/ << " at " << portInfo[0].m_baudrate << " baud" << std::endl;
	std::cout << "Opening port..." << std::endl;
	//open the port which the device is connected to and connect at the device's baudrate.
	res = cmt3.openPort(portInfo[0].m_portName , portInfo[0].m_baudrate);
	if (res != XRV_OK) {
		std::cerr << "COM Port could not be opened" << std::endl;
		m_state = ssError;
		return false;
	}
	std::cout << "done" << std::endl;

	//get the Mt sensor count.
	std::cout << "Retrieving MotionTracker count (excluding attached Xbus Master(s))" << std::endl;
	mtCount = cmt3.getMtCount();
	std::cout << "MotionTracker count: " << mtCount << std::endl;

	ASSERT_(mtCount == 1);

	// retrieve the device IDs
	std::cout << "Retrieving MotionTracker device ID" << std::endl;
	res = cmt3.getDeviceId(mtCount, deviceId);
	std::cout << "Device ID at busId 1: " << (long) deviceId << std::endl;	//printf("Device ID at busId 1: %08x\n",(long) deviceId);
	if (res != XRV_OK) {
		std::cerr << "Device ID could not be gathered" << std::endl;
		m_state = ssError;
		return false;
	}

	return true;
#else
	return false;
#endif
} // end lookForPort

/*-------------------------------------------------------------
					initialize
-------------------------------------------------------------*/
void CIMUXSens_MT4::initialize()
{
#if 0 && MRPT_HAS_xSENS_MT4

	XsensResultValue	res;

	if(cmt3.isPortOpen())
		return;

	m_state = ssInitializing;

	// Search for the COM PORT and connect
	if(!searchPortAndConnect())
	{
		m_state = ssError;
		std::cerr << "Error Could not initialize the device" << std::endl;
		return;
	}

	std::cout << "xSens IMU detected and connected" << std::endl;
	CmtOutputMode		mode		= CMT_OUTPUTMODE_ORIENT | CMT_OUTPUTMODE_CALIB;
	CmtOutputSettings	settings	= CMT_OUTPUTSETTINGS_ORIENTMODE_EULER | CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT | CMT_OUTPUTSETTINGS_CALIBMODE_ACCGYR;

	// set the sensor to config state
	res = cmt3.gotoConfig();
	if (res != XRV_OK) {
		m_state = ssError;	//EXIT_ON_ERROR(res,"gotoConfig");
		std::cerr << "An error ocurred when setting the device to config mode" << std::endl;
		return;
	}

	unsigned short sampleFreq;
	sampleFreq = cmt3.getSampleFrequency();

	// set the device output mode for the device(s)
	std::cout << "Configuring mode selection" << std::endl;
	CmtDeviceMode deviceMode(mode, settings, sampleFreq);
	res = cmt3.setDeviceMode(deviceMode, true, deviceId);
	if (res != XRV_OK) {
		m_state = ssError;	//EXIT_ON_ERROR(res,"setDeviceMode");
		std::cerr << "An error ocurred when configuring the device" << std::endl;
		return;
	}

	// start receiving data
	res = cmt3.gotoMeasurement();
	if (res != XRV_OK) {
		m_state = ssError;	//EXIT_ON_ERROR(res,"gotoMeasurement");
		std::cerr << "An error ocurred when setting the device to measurement mode" << std::endl;
		return;
	}

	std::cout << "Getting initial TimeStamp" << std::endl;
	// Get initial TimeStamp
	xsens::Packet packet(1/*NDevices*/,cmt3.isXm()/*Is Bus master*/);
	do
	{
		res = cmt3.waitForDataMessage(&packet);
		if( res == XRV_OK )
		{
			m_timeStartUI = (uint64_t)packet.getRtc();
			m_timeStartTT = mrpt::system::now();
		} // end if
	} while( res != XRV_OK );

	std::cout << "Gathering data" << std::endl;
	m_state = ssWorking;

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

	m_COMbauds = configSource.read_int(iniSection, "baudRate", m_COMbauds, false );

#ifdef MRPT_OS_WINDOWS
	m_com_port = configSource.read_string(iniSection, "COM_port_WIN", m_com_port, false );
#else
	m_com_port = configSource.read_string(iniSection, "COM_port_LIN", m_com_port, false );
#endif


}
