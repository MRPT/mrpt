/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
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



#include <mrpt/hwdrivers.h>  // Precompiled headers

#include <mrpt/hwdrivers/CImpinjRFID.h>

#include <string.h>
using namespace mrpt::hwdrivers;

IMPLEMENTS_GENERIC_SENSOR(CImpinjRFID,mrpt::hwdrivers)

CImpinjRFID::CImpinjRFID()
{
	m_sensorLabel = "RFID";
}

void  CImpinjRFID::loadConfig_sensorSpecific(
	const mrpt::utils::CConfigFileBase &configSource,
	const std::string			&iniSection )
{
	MRPT_START
		// TEMPORARILY DISABLED
/*		pose_x_1 = configSource.read_float(iniSection,"pose_x_1",0,true);
		pose_y_1 = configSource.read_float(iniSection,"pose_y_1",0,true);
		pose_z_1 = configSource.read_float(iniSection,"pose_z_1",0,true);
		pose_roll_1 = configSource.read_float(iniSection,"pose_roll_1",0,true);
		pose_pitch_1 = configSource.read_float(iniSection,"pose_pitch_1",0,true);
		pose_yaw_1 = configSource.read_float(iniSection,"pose_yaw_1",0,true);
		
		pose_x_2 = configSource.read_float(iniSection,"pose_x_2",0,true);
		pose_y_2 = configSource.read_float(iniSection,"pose_y_2",0,true);
		pose_z_2 = configSource.read_float(iniSection,"pose_z_2",0,true);
		pose_roll_2 = configSource.read_float(iniSection,"pose_roll_2",0,true);
		pose_pitch_2 = configSource.read_float(iniSection,"pose_pitch_2",0,true);
		pose_yaw_2 = configSource.read_float(iniSection,"pose_yaw_2",0,true);
*/
		port = configSource.read_int(iniSection,"listen_port",0,true);


	MRPT_END
}

/*---------------------------------------------------------------
					getObservation
    Get the power of a given network as an observation
 ---------------------------------------------------------------*/
void CImpinjRFID::connect()
{
	
		// Start the server
		server = new mrpt::utils::CServerTCPSocket(port);

		// The server waits for the client to connect
		client = server->accept();
}

/*---------------------------------------------------------------
					getObservation
    Get the power of a given network as an observation
 ---------------------------------------------------------------*/
bool CImpinjRFID::getObservation( mrpt::slam::CObservationRFID &outObservation )
{
	try{
		char msg[34];
		char cmd[20];
		char ant_port;
		char epc[24];
		char rx_pwr[5];
		char *tmp;

		// send an observation command to the device interface program
		strcpy(cmd, "OBS\0");
		client->writeAsync(cmd,10);

		// receive a reading from the sensor through the socket
		if (client->readAsync(msg,34) > 0)
		{
			// the received string is in the format: ANT_PORT EPC RX_PWR ZERO_FILL
			ant_port =  *(strtok(msg," ",&tmp));
			strcpy(epc,strtok(NULL," ",&tmp));
			strcpy(rx_pwr,strtok(NULL, " ",&tmp));
			
			// Fill the observation
			outObservation.antennaPort = ant_port;
			outObservation.epc = std::string(epc);
			outObservation.power = atof(rx_pwr);
			outObservation.sensorLabel = m_sensorLabel;
			//std::cout << "mrpt::hwdrivers::CImpinjRFID::getObservation() " << "\n\tsensorLabel: " << outObservation.sensorLabel << "\n\ttimestamp: " << outObservation.timestamp << "\n\tEPC: " << outObservation.epc << std::endl;
			return true;
		} 
		else { return false; }
	}
	catch(exception &e)
	{
		cerr << "[CImpinjRFID::getObservation] Returning false due to exception: " << endl;
		cerr << e.what() << endl;
		return false;
	}
}

/*---------------------------------------------------------------
					getObservation
    Get the power of a given network as an observation
 ---------------------------------------------------------------*/
void CImpinjRFID::closeReader()
{
	char cmd[5];
	// send a kill command to the device interface program
	strcpy(cmd, "END\0");
	client->writeAsync(cmd,10);
	client->close();
}