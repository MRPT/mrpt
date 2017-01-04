/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/hwdrivers/CHokuyoURG.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/utils/CClientTCPSocket.h>
#include <mrpt/system/os.h>
#include <mrpt/opengl/CPlanarLaserScan.h> // in library mrpt-maps
#include <mrpt/opengl/CAxis.h>

IMPLEMENTS_GENERIC_SENSOR(CHokuyoURG,mrpt::hwdrivers)

using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::opengl;
using namespace std;

const int MINIMUM_PACKETS_TO_SET_TIMESTAMP_REFERENCE = 10;

/*-------------------------------------------------------------
						Constructor
-------------------------------------------------------------*/
CHokuyoURG::CHokuyoURG() :
	m_firstRange(44),
	m_lastRange(725),
	m_motorSpeed_rpm(0),
	m_sensorPose(0,0,0),
	m_rx_buffer(40000),
	m_highSensMode(false),
	m_reduced_fov(0),
	m_com_port(""),
	m_ip_dir(""),
	m_port_dir(10940),
	m_I_am_owner_serial_port(false),
	m_timeStartUI( 0 ),
	m_timeStartSynchDelay(0),
	m_disable_firmware_timestamp(false),
	m_intensity(false)
{
	m_sensorLabel = "Hokuyo";
}

/*-------------------------------------------------------------
					~CHokuyoURG
-------------------------------------------------------------*/
CHokuyoURG::~CHokuyoURG()
{
	if (m_stream)
	{
		turnOff();

		if (m_I_am_owner_serial_port)
			delete m_stream;
		m_stream = NULL;
	}

	// FAMD
    m_win.clear(); // clear window
}

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
void  CHokuyoURG::doProcessSimple(
	bool							&outThereIsObservation,
	mrpt::obs::CObservation2DRangeScan	&outObservation,
	bool							&hardwareError )
{
	outThereIsObservation	= false;
	hardwareError			= false;

	// Bound?
	if (!checkCOMisOpen())
	{
		m_timeStartUI = 0;
		m_timeStartSynchDelay = 0;
		hardwareError = true;
		return;
	}

	// Wait for a message:
	char			rcv_status0,rcv_status1;
	vector_byte		rcv_data(10000);
	int				rcv_dataLength;
	int				nRanges = m_lastRange-m_firstRange+1;
	int				expectedSize = nRanges*3 + 4;

	if(m_intensity)
	{
		expectedSize += nRanges*3;
	}

	m_state = ssWorking;
	if (!receiveResponse( m_lastSentMeasCmd.c_str(), rcv_status0,rcv_status1, (char*)&rcv_data[0], rcv_dataLength ) )
	{
		// No new data
		return ;
	}

	// DECODE:
	if (rcv_status0!='0' && rcv_status0!='9')
	{
		hardwareError = true;
		return;
	}

	// -----------------------------------------------
	//   Extract the observation:
	// -----------------------------------------------
	outObservation.timestamp = mrpt::system::now();

	if ( expectedSize!=rcv_dataLength )
	{
		MRPT_LOG_DEBUG_FMT("[CHokuyoURG::doProcess] ERROR: Expecting %u data bytes, received %u instead!\n",expectedSize,rcv_dataLength);
		hardwareError = true;
		return;
	}
	// Delay the sync of timestamps due to instability in the constant rate during the first few packets.
	bool do_timestamp_sync = !m_disable_firmware_timestamp;
	if (do_timestamp_sync && m_timeStartSynchDelay<MINIMUM_PACKETS_TO_SET_TIMESTAMP_REFERENCE)
	{
		do_timestamp_sync=false;
		m_timeStartSynchDelay++;
	}

	if (do_timestamp_sync)
	{
		// Extract the timestamp of the sensor:
		uint32_t nowUI	=
			((rcv_data[0]-0x30) << 18) +
			((rcv_data[1]-0x30) << 12) +
			((rcv_data[2]-0x30) << 6) +
			(rcv_data[3]-0x30);

		uint32_t AtUI = 0;
		if( m_timeStartUI == 0 )
		{
			m_timeStartUI = nowUI;
			m_timeStartTT = mrpt::system::now();
		}
		else	AtUI	= nowUI - m_timeStartUI;

		mrpt::system::TTimeStamp AtDO	=  mrpt::system::secondsToTimestamp( AtUI * 1e-3 /* Board time is ms */ );
		outObservation.timestamp = m_timeStartTT + AtDO;
	}

	// And the scan ranges:
	outObservation.rightToLeft = true;

	outObservation.aperture = nRanges *2*M_PI/m_sensor_info.scans_per_360deg;

	outObservation.maxRange	= m_sensor_info.d_max;
	outObservation.stdError = 0.010f;
	outObservation.sensorPose = m_sensorPose;
	outObservation.sensorLabel = m_sensorLabel;

	outObservation.resizeScan(nRanges);
	char		*ptr = (char*) &rcv_data[4];

	if(m_intensity)
		outObservation.setScanHasIntensity(true);

	for (int i=0;i<nRanges;i++)
	{
		int b1 = (*ptr++)-0x30;
		int b2 = (*ptr++)-0x30;
		int b3 = (*ptr++)-0x30;

		int range_mm = ( (b1 << 12) | (b2 << 6) | b3);

		outObservation.setScanRange(i, range_mm * 0.001f );
		outObservation.setScanRangeValidity(i, range_mm>=20 &&  (outObservation.scan[i] <= outObservation.maxRange) );

		if(m_intensity)
		{
			int b4 = (*ptr++)-0x30;
			int b5 = (*ptr++)-0x30;
			int b6 = (*ptr++)-0x30;
			outObservation.setScanIntensity(i, ( (b4 << 12) | (b5 << 6) | b6) );
		}
	}

	// Do filter:
	C2DRangeFinderAbstract::filterByExclusionAreas( outObservation );
	C2DRangeFinderAbstract::filterByExclusionAngles( outObservation );
	// Do show preview:
	C2DRangeFinderAbstract::processPreview(outObservation);

	outThereIsObservation = true;
}

/*-------------------------------------------------------------
						loadConfig_sensorSpecific
-------------------------------------------------------------*/
void  CHokuyoURG::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string	  &iniSection )
{
	m_reduced_fov = DEG2RAD( configSource.read_float(iniSection,"reduced_fov",0) ),

	m_motorSpeed_rpm	= configSource.read_int(iniSection,"HOKUYO_motorSpeed_rpm",0);
	m_sensorPose.setFromValues(
		configSource.read_float(iniSection,"pose_x",0),
		configSource.read_float(iniSection,"pose_y",0),
		configSource.read_float(iniSection,"pose_z",0),
		DEG2RAD( configSource.read_float(iniSection,"pose_yaw",0) ),
		DEG2RAD( configSource.read_float(iniSection,"pose_pitch",0) ),
		DEG2RAD( configSource.read_float(iniSection,"pose_roll",0) )
		);

	m_highSensMode = configSource.read_bool(iniSection,"HOKUYO_HS_mode",m_highSensMode);

#ifdef MRPT_OS_WINDOWS
	m_com_port = configSource.read_string(iniSection, "COM_port_WIN", m_com_port);
#else
	m_com_port = configSource.read_string(iniSection, "COM_port_LIN", m_com_port);
#endif

	m_ip_dir = configSource.read_string(iniSection, "IP_DIR", m_ip_dir );
	m_port_dir = configSource.read_int(iniSection, "PORT_DIR", m_port_dir );

	ASSERTMSG_(!m_com_port.empty() || !m_ip_dir.empty(), "Either COM_port or IP_DIR must be defined in the configuration file!");
	ASSERTMSG_(m_com_port.empty() || m_ip_dir.empty(), "Both COM_port and IP_DIR set! Please, define only one of them.");
	if (!m_ip_dir.empty()) { ASSERTMSG_(m_port_dir,"A TCP/IP port number `PORT_DIR` must be specified for Ethernet connection"); }

	m_disable_firmware_timestamp = configSource.read_bool(iniSection, "disable_firmware_timestamp", m_disable_firmware_timestamp);
	m_intensity = configSource.read_bool(iniSection,"intensity",m_intensity),

	// Parent options:
	C2DRangeFinderAbstract::loadCommonParams(configSource, iniSection);
}

/*-------------------------------------------------------------
						turnOn
-------------------------------------------------------------*/
bool  CHokuyoURG::turnOn()
{
	MRPT_START

	// Bound?
	if (!checkCOMisOpen()) return false;

	// If we are over a serial link, set it up:
	if ( m_ip_dir.empty() )
	{
		CSerialPort* COM = dynamic_cast<CSerialPort*>(m_stream);

		if (COM!=NULL)
		{
			// It is a COM:
			COM->setConfig( 19200 );
			COM->setTimeouts(100,0,200,0,0);

			// Assure the laser is off and quiet:
			switchLaserOff();
			mrpt::system::sleep(10);

			COM->purgeBuffers();
			mrpt::system::sleep(10);

			COM->setConfig( 115200 );
			switchLaserOff();
			mrpt::system::sleep(10);
			COM->purgeBuffers();
			mrpt::system::sleep(10);
			COM->setConfig( 19200 );
		}

		if (COM!=NULL)
		{
			// Set 115200 baud rate:
			setHighBaudrate();
			enableSCIP20();
			COM->setConfig( 115200 );
		}
	}
	else
	{
		CClientTCPSocket* COM = dynamic_cast<CClientTCPSocket*>(m_stream);


		if ( COM!=NULL )
		{
			// Assure the laser is off and quiet:
			switchLaserOff();
			mrpt::system::sleep(10);

			purgeBuffers();
			mrpt::system::sleep(10);

			switchLaserOff();
			mrpt::system::sleep(10);
			purgeBuffers();
		}

	}

	if (!enableSCIP20()) return false;

	// Turn on the laser:
	if (!switchLaserOn()) return false;

	// Set the motor speed:
	if (m_motorSpeed_rpm)
		if (!setMotorSpeed( m_motorSpeed_rpm )) return false;

	// Set HS mode:
	setHighSensitivityMode(m_highSensMode);

	// Display sensor information:
	if (!displaySensorInfo(&m_sensor_info )) return false;

	// Set for scanning angles:
	m_firstRange = m_sensor_info.scan_first;
	m_lastRange  = m_sensor_info.scan_last;

	// Artificially reduced FOV?
	if (m_reduced_fov>0 && m_reduced_fov<2*M_PI)
	{
		int center=(m_lastRange+m_firstRange)>>1;
		const int half_range = static_cast<int>((m_sensor_info.scans_per_360deg / 360) * RAD2DEG(m_reduced_fov))>>1;
		m_firstRange = center-half_range;
		m_lastRange = center+half_range;
		cout << "[HOKUYO::turnOn] Using reduced FOV: ranges [" << m_firstRange  << "-" << m_lastRange << "] for " << RAD2DEG(m_reduced_fov) << " deg. FOV" << endl;
	}

	if (!displayVersionInfo()) {
		//return false; // It's not SO important
	}

	// Start!
	if (!startScanningMode()) return false;

	return true;

	MRPT_END
}

/*-------------------------------------------------------------
						turnOff
-------------------------------------------------------------*/
bool  CHokuyoURG::turnOff()
{
	// Turn off the laser:
	if (!switchLaserOff()) return false;

	return true;
}

/*-------------------------------------------------------------
						setHighBaudrate
-------------------------------------------------------------*/
bool  CHokuyoURG::setHighBaudrate()
{
	char			cmd[20];
	char			rcv_status0,rcv_status1;
	char			rcv_data[100];
	size_t			toWrite;
	int				rcv_dataLength;

	if (!checkCOMisOpen()) return false;

	MRPT_LOG_DEBUG("[CHokuyoURG::setHighBaudrate] Changing baudrate to 115200...");

	// Send command:
	os::strcpy(cmd,20, "SS115200\x0A");
	toWrite = 9;

	m_stream->WriteBuffer(cmd,toWrite);

	// Receive response:
	if (!receiveResponse( cmd, rcv_status0,rcv_status1, rcv_data, rcv_dataLength ) )
	{
		std::cerr << "Error waiting for response\n";
		return false;
	}

	// DECODE:
	if (rcv_status0!='0')
	{
		std::cerr << "Error in LIDAR status: "<< (int)rcv_status0 <<"\n";
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
	ASSERT_( nDesiredBytes<m_rx_buffer.capacity() );

	if (m_rx_buffer.size()>=nDesiredBytes)
	{
		return true;
	}
	else
	{
		// Try to read more bytes:
		uint8_t       buf[128];
		const size_t  to_read=std::min(m_rx_buffer.available(),sizeof(buf));

		try
		{
			size_t nRead;

			if ( !m_ip_dir.empty() )
			{
				CClientTCPSocket	*client = dynamic_cast<CClientTCPSocket*>(m_stream);
				nRead = client->readAsync( buf, to_read, 100, 10 );
			}
			else
			{
				nRead = m_stream->ReadBuffer(buf,to_read);
			}

			m_rx_buffer.push_many(buf,nRead);
		}
		catch (std::exception &)
		{
			// 0 bytes read
		}

		return (m_rx_buffer.size()>=nDesiredBytes);
	}
}

/*-------------------------------------------------------------
						receiveResponse
-------------------------------------------------------------*/
bool  CHokuyoURG::receiveResponse(
		const char	*sentCmd_forEchoVerification,
		char	&rcv_status0,
		char	&rcv_status1,
		char	*rcv_data,
		int		&rcv_dataLength)
{
	if (!checkCOMisOpen()) return false;

    ASSERT_(sentCmd_forEchoVerification!=NULL);

	try
	{
		// Process response:
		// ---------------------------------

		// COMMAND ECHO ---------
		int i=0;
		const int verifLen = strlen(sentCmd_forEchoVerification);

		if (verifLen)
		{
			do
			{
				if (!assureBufferHasBytes( verifLen-i ))
					return false;

				// If matches the echo, go on:
				if ( m_rx_buffer.pop()==sentCmd_forEchoVerification[i] )
					i++;
				else
					i=0;
			} while ( i<verifLen );
		}

		//printf("VERIF.CMD OK!: %s", sentCmd_forEchoVerification);

		// Now, the status bytes:
        if (!assureBufferHasBytes( 2 ))
            return false;

		rcv_status0 = m_rx_buffer.pop();
		rcv_status1 = m_rx_buffer.pop();
        //printf("STATUS: %c%c\n", rcv_status0,rcv_status1);

        // In SCIP2.0, there is an additional sum char:
        if (rcv_status1!=0x0A)
		{
			// Yes, it is SCIP2.0
            if (!assureBufferHasBytes( 1 )) return false;

            // Ignore this byte: sumStatus
            m_rx_buffer.pop();

            //printf("STATUS SUM: %c\n",sumStatus);
		}
		else
		{
			// Continue, it seems a SCIP1.1 response...
		}

        // After the status bytes, there is a LF:
        if (!assureBufferHasBytes( 1 )) return false;
        char nextChar = m_rx_buffer.pop();
        if (nextChar!=0x0A)   return false;

        // -----------------------------------------------------------------------------
        // Now the data:
        // There's a problem here, we don't know in advance how many bytes to read,
        //  so rely on the serial port class implemented buffer and call many times
        //  the read method with only 1 byte each time:
        // -----------------------------------------------------------------------------
        bool lastWasLF=false;
        i=0;
        for (;;)
        {
            if (!assureBufferHasBytes(1))
            {
            	return false;
            }
            rcv_data[i] = m_rx_buffer.pop();

            //printf("%c",rcv_data[i]);

            i++;    // One more byte in the buffer

            // No data?
            if (i==1 && rcv_data[0]==0x0A)
            {
                rcv_dataLength = 0;
                return true;
            }

            // Is it a LF?
            if (rcv_data[i-1]==0x0A)
            {
                if (!lastWasLF)
                {
                    // Discard SUM+LF
                    ASSERT_(i>=2);
                    i-=2;
                }
                else
                {
                    // Discard this last LF:
                    i--;

                    // Done!
                    rcv_data[i]=0;
               //     printf("RX %u:\n'%s'\n",i,rcv_data);

                    rcv_dataLength = i;
                    return true;
                }
                lastWasLF = true;
            }
            else lastWasLF = false;
        }

	}
	catch(std::exception &)
	{
		//cerr << e.what() << endl;
		return false;
	}
	catch(...)
	{

		return false;	// Serial port timeout,...
	}
}

/*-------------------------------------------------------------
						enableSCIP20
-------------------------------------------------------------*/
bool  CHokuyoURG::enableSCIP20()
{
	char			cmd[20];
	char			rcv_status0,rcv_status1;
	char			rcv_data[100];
	size_t			toWrite;
	int				rcv_dataLength;

	if (!checkCOMisOpen()) return false;

	MRPT_LOG_DEBUG("[CHokuyoURG::enableSCIP20] Changing protocol to SCIP2.0...");

	// Send command:
	os::strcpy(cmd,20, "SCIP2.0\x0A");
	toWrite = 8;

	m_stream->WriteBuffer(cmd,toWrite);

	// Receive response:
	if (!receiveResponse( cmd, rcv_status0,rcv_status1, rcv_data, rcv_dataLength ) )
	{
		std::cerr << "Error waiting for response\n";
		return false;
	}


	// DECODE:
	if (rcv_status0!='0')
	{
		std::cerr << "Error in LIDAR status: "<< (int)rcv_status0 <<"\n";
		return false;
	}


	MRPT_LOG_DEBUG("OK\n");
	return true;
}

/*-------------------------------------------------------------
						switchLaserOn
-------------------------------------------------------------*/
bool  CHokuyoURG::switchLaserOn()
{
	char			cmd[20];
	char			rcv_status0,rcv_status1;
	char			rcv_data[100];
	size_t			toWrite;
	int				rcv_dataLength;

	if (!checkCOMisOpen()) return false;

	MRPT_LOG_DEBUG("[CHokuyoURG::switchLaserOn] Switching laser ON...");

	// Send command:
	os::strcpy(cmd,20, "BM\x0A");
	toWrite = 3;

	m_stream->WriteBuffer(cmd,toWrite);

	// Receive response:
	if (!receiveResponse( cmd, rcv_status0,rcv_status1, rcv_data, rcv_dataLength ) )
	{
		std::cerr << "Error waiting for response\n";
		return false;
	}

	// DECODE:
	if (rcv_status0!='0')
	{
		std::cerr << "Error in LIDAR status: "<< (int)rcv_status0 <<"\n";
		return false;
	}

	MRPT_LOG_DEBUG("OK\n");

	return true;
}

/*-------------------------------------------------------------
						switchLaserOff
-------------------------------------------------------------*/
bool  CHokuyoURG::switchLaserOff()
{
	char			cmd[20];
	char			rcv_status0,rcv_status1;
	char			rcv_data[100];
	size_t			toWrite;
	int				rcv_dataLength;

	if (!checkCOMisOpen()) return false;

	MRPT_LOG_DEBUG("[CHokuyoURG::switchLaserOff] Switching laser OFF...");

	// Send command:
	os::strcpy(cmd,20, "QT\x0A");
	toWrite = 3;

	m_stream->WriteBuffer(cmd,toWrite);

	// Receive response:
	if (!receiveResponse( cmd, rcv_status0,rcv_status1, rcv_data, rcv_dataLength ) )
	{
		std::cerr << "Error waiting for response\n";
		return false;
	}

	// DECODE:
	if (rcv_status0!='0')
	{
		std::cerr << "Error in LIDAR status: "<< (int)rcv_status0 <<"\n";
		return false;
	}

	MRPT_LOG_DEBUG("OK\n");
	return true;
}

/*-------------------------------------------------------------
						setMotorSpeed
-------------------------------------------------------------*/
bool  CHokuyoURG::setMotorSpeed(int motoSpeed_rpm)
{
	char			cmd[20];
	char			rcv_status0,rcv_status1;
	char			rcv_data[100];
	size_t			toWrite;
	int				rcv_dataLength;

	if (!checkCOMisOpen()) return false;

	MRPT_LOG_DEBUG_FMT("[CHokuyoURG::setMotorSpeed] Setting to %i rpm...",motoSpeed_rpm);

	// Send command:
	int		motorSpeedCode = (600 - motoSpeed_rpm) / 6;
	if (motorSpeedCode<0 || motorSpeedCode>10)
	{
		printf("ERROR! Motorspeed must be in the range 540-600 rpm\n");
		return false;
	}

	os::sprintf(cmd,20, "CR%02i\x0A",motorSpeedCode);
	toWrite = 5;

	m_stream->WriteBuffer(cmd,toWrite);

	// Receive response:
	if (!receiveResponse( cmd, rcv_status0,rcv_status1, rcv_data, rcv_dataLength ) )
	{
		std::cerr << "Error waiting for response\n";
		return false;
	}

	// DECODE:
	if (rcv_status0!='0')
	{
		std::cerr << "Error in LIDAR status: "<< (int)rcv_status0 <<"\n";
		return false;
	}

	MRPT_LOG_DEBUG("OK\n");
	return true;
}

/*-------------------------------------------------------------
						setHighSensitivityMode
-------------------------------------------------------------*/
bool  CHokuyoURG::setHighSensitivityMode(bool enabled)
{
	char			cmd[20];
	char			rcv_status0,rcv_status1;
	char			rcv_data[100];
	size_t			toWrite;
	int				rcv_dataLength;

	if (!checkCOMisOpen()) return false;

	MRPT_LOG_DEBUG_FMT("[CHokuyoURG::setHighSensitivityMode] Setting HS mode to: %s...", enabled ? "true":"false" );

	// Send command:
	os::sprintf(cmd,20, "HS%i\x0A",enabled ? 1:0);
	toWrite = 4;

	m_stream->WriteBuffer(cmd,toWrite);

	// Receive response:
	if (!receiveResponse( cmd, rcv_status0,rcv_status1, rcv_data, rcv_dataLength ) )
	{
		std::cerr << "Error waiting for response\n";
		return false;
	}

	// DECODE:
	if (rcv_status0!='0')
	{
		std::cerr << "Error in LIDAR status: "<< (int)rcv_status0 <<"\n";
		return false;
	}

	MRPT_LOG_DEBUG("OK\n");
	return true;
}

/*-------------------------------------------------------------
                                                setIntensityMode
-------------------------------------------------------------*/
bool  CHokuyoURG::setIntensityMode(bool enabled)
{
	m_intensity = enabled;
	return true;
}

/*-------------------------------------------------------------
						displayVersionInfo
-------------------------------------------------------------*/
bool  CHokuyoURG::displayVersionInfo( )
{
	char			cmd[20];
	char			rcv_status0,rcv_status1;
	char			rcv_data[2000];
	size_t			toWrite;
	int				rcv_dataLength;

	if (!checkCOMisOpen()) return false;

	MRPT_LOG_DEBUG("[CHokuyoURG::displayVersionInfo] Asking info...");

	// Send command:
	os::sprintf(cmd,20, "VV\x0A");
	toWrite = 3;

	m_stream->WriteBuffer(cmd,toWrite);

	// Receive response:
	if (!receiveResponse( cmd, rcv_status0,rcv_status1, rcv_data, rcv_dataLength ) )
	{
		std::cerr << "Error waiting for response\n";
		return false;
	}

	// DECODE:
	if (rcv_status0!='0')
	{
		std::cerr << "Error in LIDAR status: "<< (int)rcv_status0 <<"\n";
		return false;
	}

	MRPT_LOG_DEBUG("OK\n");

	// PRINT:
	for (int i=0;i<rcv_dataLength;i++)
	{
		if (rcv_data[i]==';')
			rcv_data[i]='\n';
	}
	rcv_data[rcv_dataLength]=0;

	MRPT_LOG_INFO_FMT(
		"\n------------- HOKUYO Scanner: Version Information ------\n"
		"%s\n"
		"-------------------------------------------------------\n\n",rcv_data);
	return true;
}

/*-------------------------------------------------------------
						displaySensorInfo
-------------------------------------------------------------*/
bool  CHokuyoURG::displaySensorInfo( TSensorInfo * out_data)
{
	char			cmd[20];
	char			rcv_status0,rcv_status1;
	char			rcv_data[1000];
	size_t			toWrite;
	int				rcv_dataLength;

	if (!checkCOMisOpen()) return false;

	MRPT_LOG_DEBUG("[CHokuyoURG::displaySensorInfo] Asking for info...");

	// Send command:
	os::sprintf(cmd,20, "PP\x0A");
	toWrite = 3;

	m_stream->WriteBuffer(cmd,toWrite);

	// Receive response:
	if (!receiveResponse( cmd, rcv_status0,rcv_status1, rcv_data, rcv_dataLength ) )
	{
		std::cerr << "Error waiting for response\n";
		return false;
	}

	// DECODE:
	if (rcv_status0!='0')
	{
		std::cerr << "Error in LIDAR status: "<< (int)rcv_status0 <<"\n";
		return false;
	}

	MRPT_LOG_DEBUG("OK\n");

	// PRINT:
	for (int i=0;i<rcv_dataLength;i++)
	{
		if (rcv_data[i]==';')
			rcv_data[i]='\n';
	}
	rcv_data[rcv_dataLength]=0;

	MRPT_LOG_INFO_FMT(
		"\n------------- HOKUYO Scanner: Product Information  ------\n"
		"%s\n"
		"-------------------------------------------------------\n\n",rcv_data);

	// Parse the data:
	if (out_data)
	{
		const char *ptr;

		if ( NULL != (ptr=strstr(rcv_data,"DMAX:")) )
				out_data->d_max = 0.001 * atoi( ptr+5 );
		else	cerr << "[CHokuyoURG::displayVersionInfo] Parse error: didn't find DMAX" << endl;

		if ( NULL != (ptr=strstr(rcv_data,"DMIN:")) )
				out_data->d_min= 0.001 * atoi( ptr+5 );
		else	cerr << "[CHokuyoURG::displayVersionInfo] Parse error: didn't find DMIN" << endl;

		if ( NULL != (ptr=strstr(rcv_data,"ARES:")) )
				out_data->scans_per_360deg= atoi( ptr+5 );
		else	cerr << "[CHokuyoURG::displayVersionInfo] Parse error: didn't find ARES" << endl;

		if ( NULL != (ptr=strstr(rcv_data,"SCAN:")) )
				out_data->motor_speed_rpm= atoi( ptr+5 );
		else	cerr << "[CHokuyoURG::displayVersionInfo] Parse error: didn't find SCAN" << endl;

		if ( NULL != (ptr=strstr(rcv_data,"AMIN:")) )
				out_data->scan_first= atoi( ptr+5 );
		else	cerr << "[CHokuyoURG::displayVersionInfo] Parse error: didn't find AMIN" << endl;
		if ( NULL != (ptr=strstr(rcv_data,"AMAX:")) )
				out_data->scan_last= atoi( ptr+5 );
		else	cerr << "[CHokuyoURG::displayVersionInfo] Parse error: didn't find AMAX" << endl;
		if ( NULL != (ptr=strstr(rcv_data,"AFRT:")) )
				out_data->scan_front= atoi( ptr+5 );
		else	cerr << "[CHokuyoURG::displayVersionInfo] Parse error: didn't find AFRT" << endl;

		if ( NULL != (ptr=strstr(rcv_data,"MODL:")) )
		{
			char aux[30];
			memcpy( aux, ptr+5, 8 );
			aux[8]='\0';
			out_data->model= aux;
		}
		else	cerr << "[CHokuyoURG::displayVersionInfo] Parse error: didn't find AFRT" << endl;

	}

	return true;
}

/*-------------------------------------------------------------
						startScanningMode
-------------------------------------------------------------*/
bool  CHokuyoURG::startScanningMode()
{
	char			cmd[100];
	char			rcv_status0,rcv_status1;
	char			rcv_data[6000];
	size_t			toWrite;
	int				rcv_dataLength;

	if (!checkCOMisOpen()) return false;

	MRPT_LOG_DEBUG("[CHokuyoURG::startScanningMode] Starting scanning mode...");

	// Send command:
	if(m_intensity)
		os::sprintf(cmd,50, "ME%04u%04u01000\x0A", m_firstRange,m_lastRange);
	else
		os::sprintf(cmd,50, "MD%04u%04u01000\x0A", m_firstRange,m_lastRange);
	toWrite = 16;

	m_lastSentMeasCmd = cmd;

	m_stream->WriteBuffer(cmd,toWrite);

	// Receive response:
	if (!receiveResponse( cmd, rcv_status0,rcv_status1, rcv_data, rcv_dataLength ) )
	{
		std::cerr << "Error waiting for response\n";
		return false;
	}

	// DECODE:
	if (rcv_status0!='0')
	{
		std::cerr << "Error in LIDAR status: "<< (int)rcv_status0 <<"\n";
		return false;
	}

	MRPT_LOG_DEBUG("OK\n");
	return true;
}

/*-------------------------------------------------------------
						checkCOMisOpen
-------------------------------------------------------------*/
bool  CHokuyoURG::checkCOMisOpen()
{
	MRPT_START

	if (m_stream)
	{
		// Socket or USB connection?
		if ( !m_ip_dir.empty() && m_port_dir )
		{
			// Has the port been disconected (USB serial ports)??
			CClientTCPSocket* COM = dynamic_cast<CClientTCPSocket*>(m_stream);

			if (COM!=NULL)
			{
				if (COM->isConnected())
					return true;

				// It has been disconnected... try to reconnect:
				cerr << "[CHokuyoURG] Socket connection lost! trying to reconnect..." << endl;

				try
				{
					COM->connect( m_ip_dir, m_port_dir );
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
				return true;		// Assume OK
			}
		}
		else
		{
			// Has the port been disconected (USB serial ports)??
			CSerialPort* COM = dynamic_cast<CSerialPort*>(m_stream);
			if (COM!=NULL)
			{
				if (COM->isOpen())
					return true;

				// It has been disconnected... try to reconnect:
				cerr << "[CHokuyoURG] Serial port connection lost! Trying to reconnect..." << endl;

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
				return true;		// Assume OK
			}
		}
	}
	else
	{
		if ( m_com_port.empty() && m_ip_dir.empty() && !m_port_dir )
		{
			THROW_EXCEPTION("No stream bound to the laser nor COM serial port or ip and port provided in 'm_com_port','m_ip_dir' and 'm_port_dir'");
		}

		if ( !m_ip_dir.empty() )
		{
			// Try to open the serial port:
			CClientTCPSocket	*theCOM = new CClientTCPSocket();

			theCOM->connect( m_ip_dir, m_port_dir );

			if (!theCOM->isConnected())
			{
				cerr << "[CHokuyoURG] Cannot connect with the server '" << m_com_port << "'" << endl;
				delete theCOM;
				return false;
			}

			// Bind:
			bindIO( theCOM );

			m_I_am_owner_serial_port=true;

		}

		else
		{
			// Try to open the serial port:
			CSerialPort		*theCOM = new CSerialPort(m_com_port, true);

			if (!theCOM->isOpen())
			{
				cerr << "[CHokuyoURG] Cannot open serial port '" << m_com_port << "'" << endl;
				delete theCOM;
				return false;
			}

			// Bind:
			bindIO( theCOM );

			m_I_am_owner_serial_port=true;

		}

		return true;
	}
	MRPT_END
}

/*-------------------------------------------------------------
						initialize
-------------------------------------------------------------*/
void CHokuyoURG::initialize()
{
	if (!checkCOMisOpen()) return;

	if (!turnOn())
	{
		cerr << "[CHokuyoURG::initialize] Error initializing HOKUYO scanner" << endl;
		return;
	}

}


/*-------------------------------------------------------------
						purgeBuffers
-------------------------------------------------------------*/
void CHokuyoURG::purgeBuffers()
{
	if (!checkCOMisOpen()) return;

	if ( m_ip_dir.empty() )
	{
		CSerialPort* COM = dynamic_cast<CSerialPort*>(m_stream);
		if (COM!=NULL)
		{
			COM->purgeBuffers();
		}
	}
	else  // Socket connection
	{
		CClientTCPSocket* COM = dynamic_cast<CClientTCPSocket*>(m_stream);

		size_t to_read = COM->getReadPendingBytes();

		if ( to_read )
		{

			void *buf = malloc(sizeof(uint8_t)*to_read);

			size_t nRead = m_stream->ReadBuffer(buf,to_read);

			if ( nRead != to_read )
				THROW_EXCEPTION("Error in purge buffers: read and expected number of bytes are different.");

			free( buf );
		}
	}
}
