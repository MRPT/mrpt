/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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
#include <mrpt/hwdrivers/CGyroKVHDSP3000.h>
#include <mrpt/slam/CObservationIMU.h>




IMPLEMENTS_GENERIC_SENSOR(CGyroKVHDSP3000,mrpt::hwdrivers)

using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::hwdrivers;
using namespace std;

/*-------------------------------------------------------------
					CGyroKVHDSP3000
-------------------------------------------------------------*/
CGyroKVHDSP3000::CGyroKVHDSP3000( ) :
	m_COMbauds		(38400),
	m_com_port		(),
	m_sensorPose    (),
	m_mode			(RATE),
	m_firstInteration(true)
{

	m_state = ssInitializing;
	m_sensorLabel = "KVH_DSP3000";
	m_serialPort = NULL;

}

/*-------------------------------------------------------------
					~CGyroKVHDSP3000
-------------------------------------------------------------*/
CGyroKVHDSP3000::~CGyroKVHDSP3000()
{

	m_serialPort->close();
	delete m_serialPort;

}

/*-------------------------------------------------------------
					doProcess
-------------------------------------------------------------*/
void CGyroKVHDSP3000::doProcess()
{


	if(m_state == ssError)
	{
		mrpt::system::sleep(200);
		initialize();
	}

	if(m_state == ssError)
		return;
	
	string msg;
	CObservationIMUPtr observationGyro = CObservationIMU::Create();
	observationGyro->timestamp = mrpt::system::now();

	msg = m_serialPort->ReadString(-1,NULL,"\n");

	observationGyro->sensorPose = m_sensorPose;
	observationGyro->sensorLabel = m_sensorLabel;
	string delimiter(" ");
	vector<string> words;
	mrpt::system::tokenize(msg, delimiter, words);
	if(words.size() < 2) return;
	if(words[1].c_str()[0] == '0') return;
	double mesure = atof(words[0].c_str());
	switch(m_mode)
	{
		case RATE :
			observationGyro->rawMeasurements[IMU_YAW_VEL] = DEG2RAD(mesure);
			observationGyro->dataIsPresent[IMU_YAW_VEL] = true;
			break;
		case INTEGRATED_ANGLE :
		case INCREMENTAL_ANGLE :
			observationGyro->rawMeasurements[IMU_YAW] = DEG2RAD(mesure);
			observationGyro->dataIsPresent[IMU_YAW] = true;
			break;
	}

	if(!m_firstInteration)
	{
		appendObservation(observationGyro);
	}
	else m_firstInteration = false;


}

/*-------------------------------------------------------------
					initialize
-------------------------------------------------------------*/
void CGyroKVHDSP3000::initialize()
{

	m_process_rate = 100;
	
        /* 
          Open modem device for reading and writing and not as controlling tty
          because we don't want to get killed if linenoise sends CTRL-C.
        */
      if(m_serialPort) delete m_serialPort;
      m_serialPort = new CSerialPort(m_com_port);
      if(!(m_serialPort->isOpen())) THROW_EXCEPTION("can't open serial port");
      cout<<"m_COMbaud "<<m_COMbauds<<endl;
         m_serialPort->setConfig(m_COMbauds);

	changeMode(m_mode);
	resetIncrementalAngle();
	m_state = ssWorking;

}

/*-------------------------------------------------------------
					loadConfig_sensorSpecific
-------------------------------------------------------------*/
void  CGyroKVHDSP3000::loadConfig_sensorSpecific(
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
	string operatingMode = configSource.read_string(iniSection, "operatingMode", "rate", false);
	cout << "Operating mode : " << operatingMode << endl;	
	if(operatingMode == "incremental")
	{
		m_mode = INCREMENTAL_ANGLE;
		cout << "Incremental mode" << endl;
	}
	else if(operatingMode == "integral")
	{	
		m_mode = INTEGRATED_ANGLE;
		cout << "Integrated mode" << endl;
	}
	else
	{
		m_mode = RATE;
		cout << "Rate mode" << endl;
	}
	m_com_port = configSource.read_string(iniSection, "COM_port_LIN", m_com_port, false );

}

void CGyroKVHDSP3000::changeMode(GYRO_MODE _newMode)
{

	m_mode = _newMode;
	char commande[3];
	switch(m_mode)
	{
	case RATE :
		commande[0] = 'R';
		break;
	case INTEGRATED_ANGLE :
		commande[0] = 'P';
		break;
	case INCREMENTAL_ANGLE :
	 	commande[0] = 'A';				// incremental.
		break;
	}
	commande[1] = 0x0A; commande[2] = 0;
	// we send the command four times to be sure that the command will be interpreted by the sensor.
	if(m_serialPort->Write( commande, 3*sizeof(char)) <= 0)
	{
		THROW_EXCEPTION("can't write on serial port");
	}

}

void CGyroKVHDSP3000::resetIncrementalAngle(void)
{

	if(m_mode != RATE)
	{
		char commande[3];
		commande[0] = 'Z';
		commande[1] = '\n';
		commande[2] = 0;
		if(m_serialPort->Write( commande, 3*sizeof(char)) <= 0)
		{
			THROW_EXCEPTION("can't write on serial port");
		}
	}

}

