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
#include <mrpt/hwdrivers/CEnoseModular.h>
#include <mrpt/utils/CMessage.h>

using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::hwdrivers;
using namespace std;

IMPLEMENTS_GENERIC_SENSOR(CEnoseModular,mrpt::hwdrivers)

/*-------------------------------------------------------------
					CEnoseModular
-------------------------------------------------------------*/
CEnoseModular::CEnoseModular( ) :
	m_usbSerialNumber 	("ENOSE002"),
	m_COM_port			(),
	m_COM_baud			(115200),
	m_stream_FTDI		(NULL),
	m_stream_SERIAL		(NULL)
{
	m_sensorLabel = "EnoseModular";
	first_reading = true;
}

CEnoseModular::~CEnoseModular( )
{
	delete_safe(m_stream_FTDI);
	delete_safe(m_stream_SERIAL);
}

/*-------------------------------------------------------------
					loadConfig_sensorSpecific
-------------------------------------------------------------*/
void  CEnoseModular::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string			&iniSection )
{
	MRPT_START

	m_usbSerialNumber = configSource.read_string(iniSection, "USB_serialname","",false);

#ifdef MRPT_OS_WINDOWS
	m_COM_port = configSource.read_string(iniSection, "COM_port_WIN","COM1");
#else
	m_COM_port = configSource.read_string(iniSection, "COM_port_LIN",m_COM_port);
#endif
	m_COM_baud = configSource.read_uint64_t(iniSection, "COM_baudRate",m_COM_baud);


	MRPT_END

}


/*-------------------------------------------------------------
					checkConnectionAndConnect
-------------------------------------------------------------*/
CStream *CEnoseModular::checkConnectionAndConnect()
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
			//m_stream_SERIAL->setTimeouts(25,1,100, 1,20);
			m_stream_SERIAL->setTimeouts(50,1,100, 1,20);
			mrpt::system::sleep(10);
			m_stream_SERIAL->purgeBuffers();
			mrpt::system::sleep(10);
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
bool CEnoseModular::getObservation( mrpt::obs::CObservationGasSensors &obs )
{
	try
	{
		// Connected?
		CStream *comms = checkConnectionAndConnect();
		
		if (!comms)
		{
			cout << "ERORR: Problem connecting to Device." << endl;
			return false;
		}

		
		CObservationGasSensors::TObservationENose	newRead;
		obs.m_readings.clear();

		//---------------------------- Enose Modular FRAME --------------------------------------------------
		// Wait for e-nose frame:	<0x69><0x91><lenght><body><0x96> "Bytes"
		// Where <body> = <temp>[<SensorID_H><SensorID_L><Sensor_Value>] x N_senosrs
		// Modular-nose provides a 4B+body frame lenght 

		mrpt::utils::CMessage		msg;
		bool time_out = false;
		mrpt::system::TTimeStamp t1 = mrpt::system::getCurrentLocalTime();
		double time_out_val = 1;	//seconds
		
		while (!comms->receiveMessage( msg ) && !time_out)
		{
			if (mrpt::system::timeDifference(t1,mrpt::system::getCurrentLocalTime()) > time_out_val )
				time_out = true;
		}

		if (time_out)
		{
			cout << "[CEnoseModular - getObservation] measurement Timed-Out" << endl;
			return false;
		}

		if (msg.content.size()>0)
		{
			// Each sensor reading is composed of 3 Bytes [<SensorID_H><SensorID_L><Sensor_Value>]
			ASSERT_( (msg.content.size()-1)%3 == 0 )
			size_t numSensors = (msg.content.size()-1)/3;

			// Prepare the Enose observation
			newRead.sensorTypes.clear();
			newRead.readingsVoltage.clear();
			newRead.hasTemperature = true;			
			newRead.isActive = true;

			// Do we have the sensor position?
			if (enose_poses_x.size() != 0)
			{
				newRead.eNosePoseOnTheRobot = CPose3D(
					enose_poses_x[0],
					enose_poses_y[0],
					enose_poses_z[0],
					enose_poses_yaw[0],
					enose_poses_pitch[0],
					enose_poses_roll[0]);
			}
			else
				newRead.eNosePoseOnTheRobot = CPose3D(0,0,0);

			// Get Temperature (degrees C)
			newRead.temperature = msg.content[0]*1.65214 - 277.74648;

			//process all sensors
			for (size_t idx=0 ; idx<numSensors ; idx++)
			{
				// Copy ID (2 BYTES) To integer
				int sensorType_temp = 0;
				//memcpy( &sensorType, &msg.content[idx*3+1], 2*sizeof(msg.content[0]) );
				memcpy( &sensorType_temp, &msg.content[idx*3+1], sizeof(msg.content[0]) );
				int sensorType = sensorType_temp << (8) ;
				memcpy( &sensorType, &msg.content[idx*3+2], sizeof(msg.content[0]) );				
								
				//Add sensor Type (ID)
				newRead.sensorTypes.push_back(sensorType);

				// Transform from ADC value[8bits] to [0-0.6] volt range:
				newRead.readingsVoltage.push_back( ( msg.content[idx*3+3] * 0.6f) / 255.0f  );
				
			}
			
			//Purge buffers
			purgeBuffers();

			// Add data to observation:
			obs.m_readings.push_back( newRead );
			obs.sensorLabel = m_sensorLabel;
			obs.timestamp = mrpt::system::getCurrentTime();			
			return !obs.m_readings.empty();	// Done OK!

		} 
		else
		{
			cout << "Message was empty" << endl;
			return false;
		}
		
	}
	catch(exception &e)
	{
		cerr << "[CEnoseModular::getObservation] Returning false due to exception: " << endl;
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
void CEnoseModular::doProcess()
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
		cout << "No observation received from the USB board!" << endl;
	   // THROW_EXCEPTION("No observation received from the USB board!");
	}
}

/*-------------------------------------------------------------
						purgeBuffers
-------------------------------------------------------------*/
void CEnoseModular::purgeBuffers()
{
	if (!checkConnectionAndConnect()) 
		return;

	if (m_stream_FTDI)
	{	// FTDI pipe
		m_stream_FTDI->Purge();		
	}
	else
	{	//Serial pipe
		m_stream_SERIAL->purgeBuffers();
	}
}

