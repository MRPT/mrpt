/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/utils/CStream.h>
#include <mrpt/utils/utils_defs.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/crc.h>
#include <mrpt/hwdrivers/CSickLaserUSB.h>


#ifdef MRPT_OS_WINDOWS
	#include <windows.h>
#endif

IMPLEMENTS_GENERIC_SENSOR(CSickLaserUSB,mrpt::hwdrivers)

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::hwdrivers;
using namespace mrpt::poses;


/*-------------------------------------------------------------
						CSickLaserUSB
-------------------------------------------------------------*/
CSickLaserUSB::CSickLaserUSB() :
	m_usbConnection( NULL ),
	m_serialNumber ( "LASER001" ),
	m_timeStartUI  ( 0 )
{
	MRPT_START
	m_sensorLabel = "SICKLMS";
	m_usbConnection = new CInterfaceFTDI();
	MRPT_END
}

/*-------------------------------------------------------------
						~CSickLaserUSB
-------------------------------------------------------------*/
CSickLaserUSB::~CSickLaserUSB()
{
	delete m_usbConnection;
	m_usbConnection = NULL;
}

/*-------------------------------------------------------------
						doProcessSimple
-------------------------------------------------------------*/
void  CSickLaserUSB::doProcessSimple(
	bool							&outThereIsObservation,
	mrpt::obs::CObservation2DRangeScan	&outObservation,
	bool							&hardwareError )
{
	outThereIsObservation	= false;
	hardwareError			= false;

	if ( !checkControllerIsConnected() )
	{
		hardwareError = true;
		return;
	}

	vector<float> 	ranges;
	unsigned char	LMS_stat;
	uint32_t		board_timestamp;
	bool			is_mm_mode;

    m_state = ssWorking;

	// Wait for a scan:
	if (!waitContinuousSampleFrame( ranges,LMS_stat, board_timestamp, is_mm_mode ))
		return;

	// Yes, we have a new scan:

	// -----------------------------------------------
	//   Extract the observation:
	// -----------------------------------------------
	uint32_t AtUI = 0;
	if( m_timeStartUI == 0 )
	{
		m_timeStartUI = board_timestamp;
		m_timeStartTT = mrpt::system::now();
	}
	else	AtUI = board_timestamp - m_timeStartUI;

	mrpt::system::TTimeStamp AtDO	=  mrpt::system::secondsToTimestamp( AtUI * 1e-3 /* Board time is ms */ );
	outObservation.timestamp = m_timeStartTT + AtDO - mrpt::system::secondsToTimestamp( 0.05 ); // 50 ms delay for scan sending from the scanner

	outObservation.sensorLabel  = m_sensorLabel;	// Set label

	// Extract the timestamp of the sensor:

	// And the scan ranges:
	outObservation.rightToLeft = true;
	outObservation.aperture = M_PIf;
	outObservation.maxRange	= is_mm_mode ? 32.7 : 81.0;
	outObservation.stdError = 0.003f;
	outObservation.sensorPose = m_sensorPose;

	outObservation.resizeScan(ranges.size());

	for (size_t i=0;i<ranges.size();i++) {
		outObservation.setScanRange(i, ranges[i]);
		outObservation.setScanRangeValidity(i, (ranges[i] <= outObservation.maxRange) );
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
void  CSickLaserUSB::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string	  &iniSection )
{
	m_serialNumber = configSource.read_string(iniSection,"SICKUSB_serialNumber",m_serialNumber);
	m_sensorPose = CPose3D(
		configSource.read_float(iniSection,"pose_x",0),
		configSource.read_float(iniSection,"pose_y",0),
		configSource.read_float(iniSection,"pose_z",0),
		DEG2RAD( configSource.read_float(iniSection,"pose_yaw",0) ),
		DEG2RAD( configSource.read_float(iniSection,"pose_pitch",0) ),
		DEG2RAD( configSource.read_float(iniSection,"pose_roll",0) )
		);
	// Parent options:
	C2DRangeFinderAbstract::loadCommonParams(configSource, iniSection);
}

/*-------------------------------------------------------------
						turnOn
-------------------------------------------------------------*/
bool  CSickLaserUSB::turnOn()
{
	return true;
}

/*-------------------------------------------------------------
						turnOff
-------------------------------------------------------------*/
bool  CSickLaserUSB::turnOff()
{
	return true;
}

/*-------------------------------------------------------------
					checkControllerIsConnected
-------------------------------------------------------------*/
bool  CSickLaserUSB::checkControllerIsConnected()
{
	// If device is already open, thats ok:
	if (m_usbConnection->isOpen())
		return true;

	// If it isn't, try to open it now:
	try
	{
		m_usbConnection->OpenBySerialNumber( m_serialNumber );
		m_usbConnection->ResetDevice( );
		mrpt::system::sleep(10);
		m_usbConnection->SetTimeouts( 10, 20 );	// read, write, in milliseconds
		mrpt::system::sleep(10);
		m_usbConnection->SetLatencyTimer(1);		// 1ms, the minimum
		mrpt::system::sleep(10);

		MRPT_LOG_INFO_FMT("[CSickLaserUSB] USB DEVICE S/N:'%s' OPEN SUCCESSFULLY!!!\n",m_serialNumber.c_str() );
		return true;
	}
	catch(std::exception &e)
	{
		MRPT_LOG_ERROR_FMT("[CSickLaserUSB] ERROR TRYING TO OPEN USB DEVICE S/N:'%s'\n%s",m_serialNumber.c_str(),e.what() );
		return false;
	}
}

/*-------------------------------------------------------------
					waitContinuousSampleFrame
-------------------------------------------------------------*/
bool  CSickLaserUSB::waitContinuousSampleFrame(
	vector<float> 	&out_ranges_meters,
	unsigned char 	&LMS_status,
	uint32_t 		&out_board_timestamp,
    bool 			&is_mm_mode )
{
	size_t 	nRead,nBytesToRead;
	size_t	nFrameBytes = 0;
	size_t	lenghtField;
	unsigned char	buf[2000];
	buf[2]=buf[3]=0;

	while ( nFrameBytes < (lenghtField=( 6 + (buf[2] | (buf[3] << 8))) ) + 5  /* for 32bit timestamp + end-flag */  )
	{
		if (lenghtField>800)
		{
			cout << "#";
			nFrameBytes = 0;	// No es cabecera de trama correcta
			buf[2]=buf[3]=0;
		}

		if (nFrameBytes<4)
			nBytesToRead = 1;
		else
			nBytesToRead = (5 /* for 32bit timestamp + end-flag */ + lenghtField) - nFrameBytes;

		try
		{
			nRead = m_usbConnection->ReadSync( buf+nFrameBytes,nBytesToRead );
		}
		catch (std::exception &e)
		{
			// Disconnected?
			MRPT_LOG_ERROR_FMT("[CSickLaserUSB::waitContinuousSampleFrame] Disconnecting due to comms error: %s\n", e.what());
			m_usbConnection->Close();
			m_timeStartUI = 0;
			return false;
		}

		if ( nRead == 0 && nFrameBytes==0 )
				return false;

		if (nRead>0)
		{
			// Lectura OK:
			// Era la primera?
			if (nFrameBytes>1 || (!nFrameBytes && buf[0]==0x02) || (nFrameBytes==1 && buf[1]==0x80))
					nFrameBytes+=nRead;
			else
			{
				nFrameBytes = 0;	// No es cabecera de trama correcta
				buf[2]=buf[3]=0;
			}
		}
	}

	// Frame received
	// --------------------------------------------------------------------------
	// | STX | ADDR | L1 | L2 | COM | INF1 | INF2 |	DATA	| STA | CRC1 | CRC2 |
	// --------------------------------------------------------------------------

	// Trama completa:
	//  Checkear que el byte de comando es 0xB0:
	if ( buf[4]!=0xB0 )	return false;

	// GET FRAME INFO
	int  info	 = buf[5] | (buf[6] << 8);	// Little Endian
	int  n_points = info & 0x01FF;
	is_mm_mode = 0 != ((info & 0xC000) >> 14);	// 0x00: cm 0x01: mm

	out_ranges_meters.resize(n_points);

	// Copiar rangos:
	short mask = is_mm_mode ? 0x7FFF : 0x1FFF;
	float meters_scale = is_mm_mode ? 0.001f : 0.01f;

	for (int i=0;i<n_points;i++)
		out_ranges_meters[i] = ( (buf[7+i*2] | (buf[8+i*2] << 8)) & mask ) * meters_scale;

	// Status
	LMS_status = buf[lenghtField-3];

	// End frame:
	if (buf[nFrameBytes-1]!=0x55)
	{
		//cerr << format("[CSickLaserUSB::waitContinuousSampleFrame] bad end flag") << endl;
#ifdef MRPT_OS_WINDOWS
		OutputDebugStringA("[CSickLaserUSB::waitContinuousSampleFrame] bad end flag\n");
#endif
		return false; // Bad CRC
	}

	// CRC:
	const uint16_t CRC = mrpt::utils::compute_CRC16(buf,lenghtField-2);
	const uint16_t CRC_packet = buf[lenghtField-2] | ( buf[lenghtField-1] << 8);
	if (CRC_packet!=CRC)
	{
		const string s = format("[CSickLaserUSB::waitContinuousSampleFrame] bad CRC len=%u nptns=%u: %i != %i\n", unsigned(lenghtField),unsigned(n_points), CRC_packet, CRC);
		cerr << s;
#ifdef MRPT_OS_WINDOWS
		OutputDebugStringA(s.c_str());
#endif
		return false; // Bad CRC
	}

	// Get USB board timestamp:
	out_board_timestamp =
		(uint32_t(buf[nFrameBytes-5]) << 24) |
		(uint32_t(buf[nFrameBytes-4]) << 16) |
		(uint32_t(buf[nFrameBytes-3]) << 8) |
		 uint32_t(buf[nFrameBytes-2]);

	// All OK
	return true;
}
