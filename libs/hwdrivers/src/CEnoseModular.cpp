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

#include <mrpt/system/os.h>
#include <mrpt/hwdrivers/CEnoseModular.h>
#include <mrpt/math.h>

using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::slam;
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
	m_COM_port = configSource.read_string(iniSection, "COM_port_WIN",m_COM_port);
#else
	m_COM_port = configSource.read_string(iniSection, "COM_port_LIN",m_COM_port);
#endif
	m_COM_baud = configSource.read_uint64_t(iniSection, "COM_baudRate",m_COM_baud);

	
	MRPT_END

}

/*-------------------------------------------------------------
					queryFirmwareVersion
-------------------------------------------------------------*/
bool CEnoseModular::queryFirmwareVersion( string &out_firmwareVersion )
{
	try
	{
		utils::CMessage		msg,msgRx;

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
bool CEnoseModular::getObservation( mrpt::slam::CObservationGasSensors &obs )
{
	try
	{
		// Connected?
		CStream *comms = checkConnectionAndConnect();

		if (!comms)
		{
			cout << "Problem connecting to Device." << endl;
			return false;
		}

		utils::CMessage		msg;
		CObservationGasSensors::TObservationENose	newRead;

		obs.m_readings.clear();

		//---------------------------- Enose Modular FRAME --------------------------------------------------
		// Wait for e-nose frame:	<0x69><0x91><lenght><body><0x96> "Bytes"
		// Where <body> = <temp>[<SensorID_H><SensorID_L><Sensor_Value>] x N_senosrs
		// Modular-nose provides a 4B+body frame lenght 
		
		if (!comms->receiveMessage( msg ))
		{
			cout << "Problem receiving Message." << endl;
			return false;
		}

		if (msg.content.size()>0)
		{
			// Each sensor reading is composed of 3 Bytes [<SensorID_H><SensorID_L><Sensor_Value>]
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

			// Get Temperature (ºC)
			newRead.temperature = msg.content[0]*1.65214 - 277.74648;

			//process all sensors
			for (size_t idx=0 ; idx<numSensors ; idx++)
			{
				// Copy ID (2 BYTES) To integer
				uint16_t sensorType;
				uint8_t sensorTypeH, sensorTypeL;
				memcpy( &sensorTypeH, &msg.content[idx*3+1], sizeof(msg.content[0]) );				
				memcpy( &sensorTypeL, &msg.content[idx*3+2], sizeof(msg.content[0]) );
				sensorType = sensorTypeH;
				sensorType = (sensorType<<8) + sensorTypeL;

				//Add sensor Type (ID)
				newRead.sensorTypes.push_back(sensorType);

				// Transform from ADC value[8bits] to [0-0.6] volt range:
				newRead.readingsVoltage.push_back( ( msg.content[idx*3+3] * 0.6f) / 255.0f  );
				
			}
			
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
void  CEnoseModular::doProcess()
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
					initialize
-------------------------------------------------------------*/
/** Tries to open the camera, after setting all the parameters with a call to loadConfig.
  *  \exception This method must throw an exception with a descriptive message if some critical error is found.
  */
void CEnoseModular::initialize()
{
	// We'll rather try it in doProcess() since it's quite usual that it fails
	//  on a first try, then works on the next ones.
	/*
	if (!checkConnectionAndConnect())
		THROW_EXCEPTION("Couldn't connect to the eNose board");
	*/
}