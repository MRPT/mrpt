/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#include <mrpt/system/threads.h>
#include <mrpt/hwdrivers/CGyroKVHDSP3000.h>
#include <mrpt/slam/CObservationIMU.h>

#ifndef MRPT_OS_WINDOWS
	#include <sys/types.h>
	#include <sys/stat.h>
	#include <fcntl.h>
	#include <termios.h>
	#include <stdio.h>
#endif

IMPLEMENTS_GENERIC_SENSOR(CGyroKVHDSP3000,mrpt::hwdrivers)

using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::hwdrivers;
using namespace std;

/*-------------------------------------------------------------
					CGyroKVHDSP3000
-------------------------------------------------------------*/
CGyroKVHDSP3000::CGyroKVHDSP3000( ) :
	m_COMbauds		(0),
	m_com_port		(),
	m_sensorPose    (),
	m_mode			(RATE)
{
#ifndef MRPT_OS_WINDOWS
	m_state = ssInitializing;
	m_sensorLabel = "KVH_DSP3000";
#else
	THROW_EXCEPTION("The KVH DSP 3000 is only supported on linux");
#endif
}

/*-------------------------------------------------------------
					~CGyroKVHDSP3000
-------------------------------------------------------------*/
CGyroKVHDSP3000::~CGyroKVHDSP3000()
{
#ifndef MRPT_OS_WINDOWS
	close(m_serialPort);
#endif
}

/*-------------------------------------------------------------
					doProcess
-------------------------------------------------------------*/
void CGyroKVHDSP3000::doProcess()
{
#ifndef MRPT_OS_WINDOWS
	if(m_state == ssError)
	{
		mrpt::system::sleep(200);
		initialize();
	}

	if(m_state == ssError)
		return;
	
	char message[23];
	CObservationIMUPtr observationGyro = CObservationIMU::Create();
	observationGyro->timestamp = mrpt::system::now();
	size_t readBytes = read(m_serialPort, message, 22);
	message[readBytes+1] = 0x00;
	if(readBytes == 1) readBytes = read(m_serialPort, message, 20);  // it was the previous '\n'....
	if(readBytes < 19 ) return;
	observationGyro->sensorPose = m_sensorPose;
	observationGyro->sensorLabel = m_sensorLabel;
	string msg(message);
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
			break;
		case INTEGRATED_ANGLE :
		case INCREMENTAL_ANGLE :
			observationGyro->rawMeasurements[IMU_YAW] = DEG2RAD(mesure);
			break;
	}

	appendObservation(observationGyro);
#endif
}

/*-------------------------------------------------------------
					initialize
-------------------------------------------------------------*/
void CGyroKVHDSP3000::initialize()
{
#ifndef MRPT_OS_WINDOWS
	m_process_rate = 100;
	
       	struct termios newtio;
        /* 
          Open modem device for reading and writing and not as controlling tty
          because we don't want to get killed if linenoise sends CTRL-C.
        */
         m_serialPort = open(m_com_port.c_str(), O_RDWR | O_NOCTTY ); 
         if (m_serialPort<0)
	 {
		m_state = ssError;
		THROW_EXCEPTION("Can't open serial port");
	 }
        
         bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */
        
        /* 
          BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
          CRTSCTS : output hardware flow control (only used if the cable has
                    all necessary lines. See sect. 7 of Serial-HOWTO)
          CS8     : 8n1 (8bit,no parity,1 stopbit)
          CLOCAL  : local connection, no modem contol
          CREAD   : enable receiving characters
        */
         newtio.c_cflag = B38400 | CS8 | CLOCAL | CREAD;
         
        /*
          IGNPAR  : ignore bytes with parity errors
          ICRNL   : map CR to NL (otherwise a CR input on the other computer
                    will not terminate input)
          otherwise make device raw (no other input processing)
        */
         newtio.c_iflag = IGNPAR | ICRNL;
         
        /*
         Raw output.
        */
         newtio.c_oflag = 0;
         
        /*
          ICANON  : enable canonical input
          disable all echo functionality, and don't send signals to calling program
        */
         newtio.c_lflag = ICANON | ECHO | ECHOE;
         
        /* 
          initialize all control characters 
          default values can be found in /usr/include/termios.h, and are given
          in the comments, but we don't need them here
        */
         newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */ 
         newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
         newtio.c_cc[VERASE]   = 0;     /* del */
         newtio.c_cc[VKILL]    = 0;     /* @ */
         newtio.c_cc[VEOF]     = 4;     /* Ctrl-d */
         newtio.c_cc[VTIME]    = 0;     /* inter-character timer unused */
         newtio.c_cc[VMIN]     = 5;     /* blocking read until 1 character arrives */
         newtio.c_cc[VSWTC]    = 0;     /* '\0' */
         newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */ 
         newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
         newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
         newtio.c_cc[VEOL]     = '\n';	/* '\n' */
         newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
         newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
         newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
         newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
         newtio.c_cc[VEOL2]    = 0;     /* '\0' */
        
        /* 
          now clean the modem line and activate the settings for the port
        */
        tcflush(m_serialPort, TCIFLUSH);
        tcsetattr(m_serialPort,TCSANOW,&newtio);
	
	changeMode(m_mode);
	resetIncrementalAngle();
	m_state = ssWorking;
#endif
}

/*-------------------------------------------------------------
					loadConfig_sensorSpecific
-------------------------------------------------------------*/
void  CGyroKVHDSP3000::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string	  &iniSection )
{
#ifndef MRPT_OS_WINDOWS
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
#endif
}

void CGyroKVHDSP3000::changeMode(GYRO_MODE _newMode)
{
#ifndef MRPT_OS_WINDOWS
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
	if(write(m_serialPort, commande, 3*sizeof(char)) <= 0)
	{
		THROW_EXCEPTION("can't write on serial port");
	}
#endif
}

void CGyroKVHDSP3000::resetIncrementalAngle(void)
{
#ifndef MRPT_OS_WINDOWS
	if(m_mode != RATE)
	{
		char commande[3];
		commande[0] = 'Z';
		commande[1] = '\n';
		commande[2] = 0;
		if(write(m_serialPort, commande, 3*sizeof(char)) <= 0)
		{
			THROW_EXCEPTION("can't write on serial port");
		}
	}
#endif
}

