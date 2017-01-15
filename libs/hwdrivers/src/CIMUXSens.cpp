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
#include <mrpt/hwdrivers/CIMUXSens.h>
#include <mrpt/obs/CObservationIMU.h>

IMPLEMENTS_GENERIC_SENSOR(CIMUXSens,mrpt::hwdrivers)

using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::hwdrivers;

#if MRPT_HAS_xSENS_MT3
	#include "xSens_MT3/cmt1.h"
	#include "xSens_MT3/cmt2.h"
	#include "xSens_MT3/cmt3.h"
	#include "xSens_MT3/cmtdef.h"
	#include "xSens_MT3/cmtmessage.h"
	#include "xSens_MT3/cmtpacket.h"
	#include "xSens_MT3/cmtscan.h"
	#include "xSens_MT3/xsens_fifoqueue.h"
	#include "xSens_MT3/xsens_janitors.h"
	#include "xSens_MT3/xsens_list.h"
	#include "xSens_MT3/xsens_std.h"
	#include "xSens_MT3/xsens_time.h"
#endif


// Adaptors for the "void*" memory blocks:
#define cmt3	    (*static_cast<xsens::Cmt3*>(m_cmt3_ptr))
#define deviceId    (*static_cast<CmtDeviceId*>(m_deviceId_ptr))


// Include libraries in linking:
#if MRPT_HAS_xSENS_MT3
	#ifdef MRPT_OS_WINDOWS
		// WINDOWS:
		#if defined(_MSC_VER) || defined(__BORLANDC__)
			#pragma comment (lib,"SetupAPI.lib")
		#endif
	#endif	// MRPT_OS_WINDOWS
#endif // MRPT_HAS_xSENS_MT3

/*-------------------------------------------------------------
					CIMUXSens
-------------------------------------------------------------*/
CIMUXSens::CIMUXSens( ) :
	m_COMbauds		(0),
	m_com_port		(),
	m_timeStartUI	(0),
	m_timeStartTT	(0),
	m_sensorPose    (),
	m_cmt3_ptr	(NULL),
	m_deviceId_ptr	(NULL),
	m_toutCounter	(0)
{
	m_sensorLabel = "XSensMTi";
#if MRPT_HAS_xSENS_MT3
    m_cmt3_ptr  = new xsens::Cmt3[1];
    m_deviceId_ptr = new CmtDeviceId[1];

#else
	THROW_EXCEPTION("MRPT has been compiled with 'BUILD_XSENS_MT3'=OFF, so this class cannot be used.");
#endif

}

/*-------------------------------------------------------------
					~CIMUXSens
-------------------------------------------------------------*/
CIMUXSens::~CIMUXSens()
{
#if MRPT_HAS_xSENS_MT3
	cmt3.closePort();

    delete[] &cmt3;     m_cmt3_ptr= NULL;
    delete[] &deviceId; m_deviceId_ptr = NULL;
#endif
}

/*-------------------------------------------------------------
					doProcess
-------------------------------------------------------------*/
void CIMUXSens::doProcess()
{
#if MRPT_HAS_xSENS_MT3

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

				std::cerr << "[CIMUXSens::doProcess()] Error: No data available [XRV_TIMEOUT]" << std::endl;
			}
		} // end if XRV_TIMEOUT

		if(res == XRV_TIMEOUTNODATA)
		{
//			m_state			= ssError;
//			m_timeStartUI	= 0;
//			if( cmt3.isPortOpen() )
//				cmt3.closePort();
//			std::cerr << "[CIMUXSens::doProcess()] Error: No data available [XRV_TIMEOUTNODATA]" << std::endl;
		} // end if XRV_TIMEOUTNODATA
	} while( res == XRV_OK && cont++ < 30);

#else
	THROW_EXCEPTION("MRPT has been compiled with 'BUILD_XSENS_MT3'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
					lookForPort
-------------------------------------------------------------*/
bool CIMUXSens::searchPortAndConnect()
{
#if MRPT_HAS_xSENS_MT3
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
void CIMUXSens::initialize()
{
#if MRPT_HAS_xSENS_MT3

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
	THROW_EXCEPTION("MRPT has been compiled with 'BUILD_XSENS_MT3'=OFF, so this class cannot be used.");
#endif
}

/*-------------------------------------------------------------
					loadConfig_sensorSpecific
-------------------------------------------------------------*/
void  CIMUXSens::loadConfig_sensorSpecific(
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
