/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
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

#include <mrpt/hwdrivers.h> // Precompiled headers

#include <mrpt/utils/crc.h>
#include <mrpt/system/os.h>
#include <mrpt/hwdrivers/CBoardDLMS.h>
#include <mrpt/slam/CObservation2DRangeScan.h>

using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::hwdrivers;
using namespace std;

#define MAX_RANGE 32758 // For 32 m. maximum
#define NUM_REQUIRED_PTOS 361
#define NTRIES 5


IMPLEMENTS_GENERIC_SENSOR(CBoardDLMS,mrpt::hwdrivers)

/*-------------------------------------------------------------
					CBoardENoses
-------------------------------------------------------------*/
CBoardDLMS::CBoardDLMS( ) :
	m_usbSerialNumber	("DLMS-001"),
	m_timeStartUI		(0),
	m_timeStartTT		(0),
	m_mSensorPose		(),
	m_sSensorPose		()
{
}

/*-------------------------------------------------------------
					~CBoardENoses
-------------------------------------------------------------*/
CBoardDLMS::~CBoardDLMS()
{
}

/*-----------------------------------------------------------------
	CheckCRC
	 Output:	True if CRC OK and false if CRC fails
	 Input: 	*frame	-> Pointer to the frame
  -----------------------------------------------------------------*/
bool CBoardDLMS::checkCRC( const std::vector<unsigned char> &frame )
{
	uint16_t uLen, crc, my_crc;

	if( frame.size() < 8 || frame.size() > 760)
		return false;

	uLen = 4 + (uint16_t)frame[2] + (((uint16_t)frame[3])<<8);
	if( uLen != frame.size()-6 )
		return false;

	my_crc  = (((uint16_t)frame[uLen+1])<<8) + ((uint16_t)frame[uLen]);
	crc 	= mrpt::utils::compute_CRC16( &frame[0], uLen );

	if(crc == my_crc)
		return true;
	else
		return false;
} // end checkCRC

/*-------------------------------------------------------------
					loadConfig
-------------------------------------------------------------*/
void  CBoardDLMS::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const string	  &iniSection )
{
	MRPT_START
	m_usbSerialNumber	= configSource.read_string( iniSection, "USB_serialname", "", true );

	m_mSensorPose.setFromValues(
        configSource.read_float( iniSection, "mPose_x", 0, false ),
        configSource.read_float( iniSection, "mPose_y", 0, false ),
        configSource.read_float( iniSection, "mPose_z", 0, false ),
        DEG2RAD( configSource.read_float( iniSection, "mPose_yaw", 0, false ) ),
        DEG2RAD( configSource.read_float( iniSection, "mPose_pitch", 0, false ) ),
        DEG2RAD( configSource.read_float( iniSection, "mPose_roll", 0, false ) ) );

	m_sSensorPose.setFromValues(
        configSource.read_float( iniSection, "sPose_x", 0, false ),
        configSource.read_float( iniSection, "sPose_y", 0, false ),
        configSource.read_float( iniSection, "sPose_z", 0, false ),
        DEG2RAD( configSource.read_float( iniSection, "sPose_yaw", 0, false ) ),
        DEG2RAD( configSource.read_float( iniSection, "sPose_pitch", 0, false ) ),
        DEG2RAD( configSource.read_float( iniSection, "sPose_roll", 0, false ) ) );

	MRPT_END
}

/*-------------------------------------------------------------
					initialize
-------------------------------------------------------------*/
void CBoardDLMS::initialize()
{
	mrpt::system::TTimeStamp		tstamp;
	int								tries = 0;

	while( !queryTimeStamp( tstamp ) && tries < NTRIES )
		tries++;

	if (tries < NTRIES)
	{
		Purge();
		m_timeStartUI = (uint32_t)tstamp;
		m_timeStartTT = mrpt::system::now();
        m_state       = ssWorking;
	} // end if
} // end initialize

/*-------------------------------------------------------------
					doProcess
-------------------------------------------------------------*/
void CBoardDLMS::doProcess()
{
	CMessage	inMsg;
	int				frameSize;
	int				points, info, unit;

	try
	{
		while( receiveMessage( inMsg ) )
		{
			m_state=ssWorking;

			// Create observation.
			// Frame received (into inMsg.content)
			// --------------------------------------------------------------------------
			// | STX | ADDR | L1 | L2 | COM | INF1 | INF2 |	DATA	| STA | CRC1 | CRC2 |
			// --------------------------------------------------------------------------
			// Frame received (into inMsg.content) w/TIMESTAMP
			// --------------------------------------------------------------------------------------------------
			// | STX | ADDR | L1 | L2 | COM | INF1 | INF2 |	DATA	| STA | CRC1 | CRC2 | TS1 | TS2 | TS3 | TS4 |
			// --------------------------------------------------------------------------------------------------

			// Check the number of points into the message
			if( inMsg.content.size() == 736 && checkCRC( inMsg.content ) )
			{
				// GET FRAME INFO
				info		= (int)(inMsg.content[5])+((int)inMsg.content[6])*256;				// Little Endian
				points		= info & 0x01FF;
				unit		= (info & 0xC000) >> 14;											// 0x00: cm 0x01: mm

				frameSize	= (int)inMsg.content.size();

				if( points == NUM_REQUIRED_PTOS )
				{
					// -------------------------------
					// Set the observation parameters
					// -------------------------------
					CObservation2DRangeScanPtr myObs = CObservation2DRangeScan::Create();
					if( (int)inMsg.type == 0 )
					{
						myObs->sensorLabel	= "MasterLMS";
						myObs->sensorPose	= m_mSensorPose;
					} // end if
					if( (int)inMsg.type == 1 )
					{
						myObs->sensorLabel	= "SlaveLMS";
						myObs->sensorPose	= m_sSensorPose;
					}
					// TimeStamp
					uint32_t nowUI	= (uint32_t)inMsg.content[frameSize-1] + (((uint32_t)inMsg.content[frameSize-2])<<8) + (((uint32_t)inMsg.content[frameSize-3])<<16) + (((uint32_t)inMsg.content[frameSize-4])<<24);
					uint32_t AtUI = 0;
					if( m_timeStartUI == 0 )
					{
						m_timeStartUI = nowUI;
						m_timeStartTT = mrpt::system::now();
					}
					else
						AtUI	= nowUI - m_timeStartUI;

					mrpt::system::TTimeStamp AtDO	=  mrpt::system::secondsToTimestamp( AtUI * 1e-3 /* Board time is ms */ );
					myObs->timestamp	= m_timeStartTT	+ AtDO;

					myObs->aperture		= (float)M_PI;							// Aperture in radians
					myObs->rightToLeft	= true;									// Scan direction
					myObs->maxRange		= MAX_RANGE/1000.0f;					// Maximum angle in meters

					myObs->validRange.resize( points );							// Valid range vector
					myObs->scan.resize( points );								// Scan vector

					// --------------------------------
					// Fill the vector of measurements
					// --------------------------------
					vector<float>::iterator			itScan;
					vector<unsigned char>::iterator	itCont;
					vector<char>::iterator			itValid;

					for(itScan = myObs->scan.begin(), itCont = inMsg.content.begin()+7, itValid = myObs->validRange.begin();
						itScan != myObs->scan.end();
						itScan++, itValid++)
					{
						unsigned char lowByte = *itCont;
						unsigned char highByte = (*(itCont+1)) & 0x7F;			// Mask MSB FOR 15 bits data!
						float m;
						if( (lowByte + highByte*256.0f) > MAX_RANGE )			// Overflow value
						{
							m = MAX_RANGE/1000.0f;
							(*itValid) = 0;
						}
						else
						{
							m = (lowByte + highByte*256.0f)/( unit ? 1000.0f : 100.0f);			// Correct value
							(*itValid) = 1;
						}
						(*itScan) = m;
						itCont = itCont+2;
					} // end for

					appendObservation( myObs );
				} // end if (nPtos == NUM_REQUIRED_PTOS)
			}// end if checkCRC
			else
			{
				inMsg.content.size() == 736 ? std::cout << "Bad CRC!" << std::endl : std::cout << "Bad frame received!" << std::endl;
				Purge();
			}
		} // end while
	}
	catch (std::exception &e)
	{
		cerr << "[CBoardDLMS - doProces] Exception while gathering data: " << e.what() << endl;
		m_state = ssError;
		throw e;
	}
}

/*-------------------------------------------------------------
					queryFirmwareVersion
-------------------------------------------------------------*/
bool CBoardDLMS::queryFirmwareVersion( string &out_firmwareVersion )
{
	try
	{
		utils::CMessage		msg,msgRx;

		// Try to connect to the device:
		if (!checkConnectionAndConnect())	return false;

		msg.type = 0x10;
		sendMessage(msg);

		if (receiveMessage(msgRx) )
		{
			msgRx.getContentAsString( out_firmwareVersion );
			return true;
		}
		else
			return false;
	}
	catch(...)
	{
		Close();
		return false;
	}
}

/*-------------------------------------------------------------
					queryTimeStamp
-------------------------------------------------------------*/
bool CBoardDLMS::queryTimeStamp( mrpt::system::TTimeStamp &tstamp )
{
	try
	{
		utils::CMessage		msg,msgRx;

		// Try to connect to the device:
		if (!checkConnectionAndConnect())	return false;

		Purge();

		msg.type = 0x20;
		sendMessage(msg);

		do
		{
			if(!receiveMessage(msgRx))
			{
				mrpt::system::sleep(200);
				return false;
			}
		} while(msgRx.type != 0x90); // end do while

		tstamp = (uint64_t)msgRx.content[0] + (((uint64_t)msgRx.content[1])<<8) + (((uint64_t)msgRx.content[2])<<16) + (((uint64_t)msgRx.content[3])<<24);
		cout << "CBoardDLMS: USB Port open succesfully" << endl;
		cout << "Received Initial TimeStamp: " << tstamp << endl;

		//cout << "TS  Received: [" << msgRx.content.size() << "]"<< endl;
		//printf("%2X %2X %2X %2X\n", msgRx.content[0], msgRx.content[1], msgRx.content[2], msgRx.content[3]);
		//system::pause();
		return true;
	}
	catch(std::exception &e)
	{
		Close();
		cerr << "[CBoardDLMS]: Exception when querying Time Stamp: " << e.what() << endl;
		return false;
	}
}


/*-------------------------------------------------------------
					checkConnectionAndConnect
-------------------------------------------------------------*/
bool CBoardDLMS::checkConnectionAndConnect()
{
	if (isOpen())
		return true;

	try
	{
		OpenBySerialNumber( m_usbSerialNumber );
		mrpt::system::sleep(10);
		Purge();
		mrpt::system::sleep(10);
		SetLatencyTimer(1);
		SetTimeouts(3,100);
		return true;
	}
	catch(...)
	{
		// Error opening device:
		Close();
		cerr << "[CBoardDLMS]: Exception when opening the USB device" << endl;
		return false;
	}
}
