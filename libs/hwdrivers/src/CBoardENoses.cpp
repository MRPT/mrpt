/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"   // Precompiled headers

#include <mrpt/system/os.h>
#include <mrpt/hwdrivers/CBoardENoses.h>
#include <mrpt/utils/CMessage.h>
#include <mrpt/math/ops_vectors.h>

using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::hwdrivers;
using namespace std;

IMPLEMENTS_GENERIC_SENSOR(CBoardENoses,mrpt::hwdrivers)

/*-------------------------------------------------------------
					CBoardENoses
-------------------------------------------------------------*/
CBoardENoses::CBoardENoses( ) :
	m_usbSerialNumber 	("ENOSE001"),
	m_COM_port			(),
	m_COM_baud			(115200),
	m_stream_FTDI		(NULL),
	m_stream_SERIAL		(NULL)
{
	m_sensorLabel = "ENOSE";
	first_reading = true;
}

CBoardENoses::~CBoardENoses( )
{
	delete_safe(m_stream_FTDI);
	delete_safe(m_stream_SERIAL);
}

/*-------------------------------------------------------------
					loadConfig_sensorSpecific
-------------------------------------------------------------*/
void  CBoardENoses::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string			&iniSection )
{
	MRPT_START

	m_usbSerialNumber = configSource.read_string(iniSection, "USB_serialname","",false);

#ifdef MRPT_OS_WINDOWS
	m_COM_port = configSource.read_string(iniSection, "COM_port_WIN",m_COM_port);
#else
	m_COM_port = configSource.read_string(iniSection, "COM_port_LIN",m_COM_port);
#endif
	m_COM_baud = configSource.read_uint64_t(iniSection, "COM_baudRate",m_COM_baud);

	configSource.read_vector( iniSection, "enose_poses_x", vector<float>(0), enose_poses_x, true);
	configSource.read_vector( iniSection, "enose_poses_y", vector<float>(0), enose_poses_y, true);
	configSource.read_vector( iniSection, "enose_poses_z", vector<float>(0), enose_poses_z, true);

	configSource.read_vector( iniSection, "enose_poses_yaw", vector<float>(0), enose_poses_yaw, true);
	configSource.read_vector( iniSection, "enose_poses_pitch", vector<float>(0), enose_poses_pitch, true);
	configSource.read_vector( iniSection, "enose_poses_roll", vector<float>(0), enose_poses_roll, true);

	ASSERT_( enose_poses_x.size() == enose_poses_y.size() );
	ASSERT_( enose_poses_x.size() == enose_poses_z.size() );
	ASSERT_( enose_poses_x.size() == enose_poses_yaw.size() );
	ASSERT_( enose_poses_x.size() == enose_poses_pitch.size() );
	ASSERT_( enose_poses_x.size() == enose_poses_roll.size() );

	// Pass angles to radians:
	enose_poses_yaw *= M_PIf / 180.0f;
	enose_poses_pitch *= M_PIf / 180.0f;
	enose_poses_roll *= M_PIf / 180.0f;

	MRPT_END

}

/*-------------------------------------------------------------
					queryFirmwareVersion
-------------------------------------------------------------*/
bool CBoardENoses::queryFirmwareVersion( string &out_firmwareVersion )
{
	try
	{
		mrpt::utils::CMessage		msg,msgRx;

		// Try to connect to the device:
		CStream *comms = checkConnectionAndConnect();
		if (!comms)
			return false;

		msg.type = 0x10;
		comms->sendMessage(msg);

		if (comms->receiveMessage(msgRx) )
		{
			msgRx.getContentAsString( out_firmwareVersion );
			return true;
		}
		else
			return false;
	}
	catch(...)
	{
		// Close everything and start again:
		delete_safe(m_stream_SERIAL);
		delete_safe(m_stream_FTDI);
		return false;
	}
}


/*-------------------------------------------------------------
					checkConnectionAndConnect
-------------------------------------------------------------*/
CStream *CBoardENoses::checkConnectionAndConnect()
{
	// Make sure one of the two possible pipes is open:
	if (!m_stream_FTDI && !m_stream_SERIAL)
	{
		if (!m_COM_port.empty())
				m_stream_SERIAL = new CSerialPort();
		else 	m_stream_FTDI = new CInterfaceFTDI();
	}


	if (m_stream_FTDI)
	{	// FTDI pipe ==================
		if (m_stream_FTDI->isOpen())
			return m_stream_FTDI;
		try
		{
			m_stream_FTDI->OpenBySerialNumber( m_usbSerialNumber );
			mrpt::system::sleep(10);
			m_stream_FTDI->Purge();
			mrpt::system::sleep(10);
			m_stream_FTDI->SetLatencyTimer(1);
			m_stream_FTDI->SetTimeouts(10,100);
			return m_stream_FTDI;
		}
		catch(...)
		{	// Error opening device:
			m_stream_FTDI->Close();
			return NULL;
		}
	}
	else
	{	// Serial pipe ==================
		ASSERT_(m_stream_SERIAL)
		if (m_stream_SERIAL->isOpen())
			return m_stream_SERIAL;
		try
		{
			m_stream_SERIAL->open(m_COM_port);
			m_stream_SERIAL->setConfig(m_COM_baud);
			mrpt::system::sleep(10);
			m_stream_SERIAL->purgeBuffers();
			mrpt::system::sleep(10);
			//m_stream_SERIAL->setTimeouts(25,1,100, 1,20);
			m_stream_SERIAL->setTimeouts(50,1,100, 1,20);
			return m_stream_SERIAL;
		}
		catch(...)
		{	// Error opening device:
			m_stream_SERIAL->close();
			return NULL;
		}
	}
}


/*-------------------------------------------------------------
					getObservation
-------------------------------------------------------------*/
bool CBoardENoses::getObservation( mrpt::obs::CObservationGasSensors &obs )
{
	try
	{
		// Connected?
		CStream *comms = checkConnectionAndConnect();

		if (!comms)
			return false;

		utils::CMessage		msg;
		CObservationGasSensors::TObservationENose	newRead;

		obs.m_readings.clear();

		//// Send request:
		//msg.type = 0x11;
		//msg.content.clear();
		//comms->sendMessage( msg );

		//----------------------------MCE-nose FRAME--------------------------------------------------
		// Wait for e-nose frame:	<0x69><0x91><lenght><body><0x96> "Bytes"
		// Where <body> = [Numchamber, Activechamber, N sensors*M chambers*2, 2 timestamp] of uint16_t
		// MCE-nose provides a 136B body lenght which makes 140B total frame lenght
		
		if (!comms->receiveMessage( msg ))
		{
			return false;
		}

        //m_state = ssWorking;

		// Copy to "uint16_t":
		ASSERT_((msg.content.size() % 2)==0);

		vector<uint16_t>	readings( msg.content.size() / 2 );	// divide by 2 to pass from byte to word. 136B/2 = 68 Words

		if (msg.content.size()>0)
		{
			// Copy to a vector of 16bit integers:			
			memcpy( &readings[0],&msg.content[0],msg.content.size() * sizeof(msg.content[0]) );

			//HEADER Frame [ Nº of chambers/enoses (16b) , Active Chamber (16b)]
			size_t NumberOfChambers = (size_t)readings[0];
			size_t ActiveChamber = (size_t)readings[1];

			// Sensors readings info
			ASSERT_( ((readings.size() - 4) % NumberOfChambers)==0 );
			size_t wordsPereNose = (readings.size() - 4) / NumberOfChambers;


			// Process each chamber
			for (size_t i=0; i<NumberOfChambers; i++)
			{
				// ----------------------------------------------------------------------
				// Each "i" comprises a complete Enose reading: Gas sensors + temperature
				// ----------------------------------------------------------------------

				// Do we have the sensor position?
				if (i<enose_poses_x.size())
				{
					newRead.eNosePoseOnTheRobot = CPose3D(
						enose_poses_x[i],
						enose_poses_y[i],
						enose_poses_z[i],
						enose_poses_yaw[i],
						enose_poses_pitch[i],
						enose_poses_roll[i]);
				}
				else
					newRead.eNosePoseOnTheRobot = CPose3D(0,0,0);

				// Process the sensor codes:
				newRead.sensorTypes.clear();
				newRead.readingsVoltage.clear();
				newRead.hasTemperature = false;
				newRead.isActive = false;

				// check if active chamber
				if (i == (ActiveChamber))
					newRead.isActive = true;

				//process each sensor on this chamber "i"
				for (size_t idx=0 ; idx<wordsPereNose/2 ; idx++)
				{
					if ( readings[i*wordsPereNose + 2*idx + 2] != 0x0000 )	//not empty slot
					{
						// Is temperature?
						if (readings[i*wordsPereNose + 2*idx + 2] == 0xFFFF)
						{
							newRead.hasTemperature = true;
							newRead.temperature = ((int16_t)readings[i*wordsPereNose + 2*idx + 3]) / 32.0f;
						}
						else	//Is a gas sensors
						{
							// It is not a null code: There is a valid measure:
							newRead.sensorTypes.push_back( readings[i*wordsPereNose + 2*idx + 2] );

							// Pass from ADC value[12bits] to [0-2.5] volt range:
							newRead.readingsVoltage.push_back( ( readings[i*wordsPereNose + 2*idx + 3] * 5.0f) / 4096.0f  );
						}
					}
				} // end for each sensor on this eNose

				// Add to observations:
				if ( !newRead.sensorTypes.empty() )
					obs.m_readings.push_back( newRead );

			} // end for each i'th eNose

			obs.sensorLabel = m_sensorLabel;

			// Set Timestamp
			uint32_t *p = (uint32_t*)&readings[readings.size()-2];	//Get readings time from frame (always last 2 words)
			obs.timestamp = mrpt::system::secondsToTimestamp(((double)*p)/1000);

			if (first_reading)
			{
				initial_timestamp = mrpt::system::getCurrentTime() - obs.timestamp;
				first_reading = false;
			}
			obs.timestamp = obs.timestamp + initial_timestamp;

		} // end if message has data


		//CONTROL
		bool correct = true;

		if (obs.m_readings.size() != 4)
				correct = false;
		else
		{
			for (size_t ch=0; ch<obs.m_readings.size(); ch++)
			{
				if( (obs.m_readings[ch].sensorTypes.size() != 7) || (obs.m_readings[ch].readingsVoltage.size() != 7) )
					correct = false;
				else
				{

				}
			}
		}

		if (!correct)
			printf("Error en la observacion"); //For debug


		return !obs.m_readings.empty();	// Done OK!
	}
	catch(exception &e)
	{
		cerr << "[CBoardENoses::getObservation] Returning false due to exception: " << endl;
		cerr << e.what() << endl;
		return false;
	}
	catch(...)
	{
		return false;
	}
}



/*-------------------------------------------------------------
					doProcess
-------------------------------------------------------------*/
/** This method should be called periodically (at least at 1Hz to capture ALL the real-time data)
*  It is thread safe, i.e. you can call this from one thread, then to other methods from other threads.
*/
void  CBoardENoses::doProcess()
{
	CObservationGasSensorsPtr obs= CObservationGasSensors::Create();

	if (getObservation(*obs))
	{
		m_state = ssWorking;
		appendObservation( obs );
	}
	else
	{
		m_state = ssError;
	   // THROW_EXCEPTION("No observation received from the USB board!");
	}
}


/*-------------------------------------------------------------
					initialize
-------------------------------------------------------------*/
/** Tries to open the camera, after setting all the parameters with a call to loadConfig.
  *  \exception This method must throw an exception with a descriptive message if some critical error is found.
  */
void CBoardENoses::initialize()
{
	// We'll rather try it in doProcess() since it's quite usual that it fails
	//  on a first try, then works on the next ones.
	/*
	if (!checkConnectionAndConnect())
		THROW_EXCEPTION("Couldn't connect to the eNose board");
	*/
}

/*-------------------------------------------------------------
					setActiveChamber
-------------------------------------------------------------*/
/** Send to the MCE-nose the next Active Chamber */

bool CBoardENoses::setActiveChamber( unsigned char chamber )
{
	try
	{
		// Try to connect to the device:
		CStream *comms = checkConnectionAndConnect();
		if (!comms)
			return false;

		// Send a byte to set the Active chamber on device.
		// by default:  Byte_to_send = 10_ _ _ _10
		unsigned char buf[1];
		buf[0] = ((chamber & 15) << 2) | 130;	 //130= 10 0000 10

		comms->WriteBuffer(buf,1);	// Exceptions will be raised on errors here
		return true;
	}
	catch(...)
	{
		// Close everything and start again:
		delete_safe(m_stream_SERIAL);
		delete_safe(m_stream_FTDI);
		return false;
	}
}
