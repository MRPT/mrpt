/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Emil Khatib  <emilkhatib@gmail.com>	                       |
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
#ifndef  CImpinjRFID_H
#define  CImpinjRFID_H

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/slam/CObservationRFID.h>
#include <mrpt/utils/CConfigFileBase.h>

#include <mrpt/utils/CClientTCPSocket.h>
#include <mrpt/utils/CServerTCPSocket.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** This class implements an interface to an Impinj RFID reader. This object connects to a program that does the actual communication with the receiver. This is done because the manufacturer only provides libraries for C# and Java. The program that runs the device must be started after this object
		  */
		class HWDRIVERS_IMPEXP CImpinjRFID : public mrpt::hwdrivers::CGenericSensor
		{
				DEFINE_GENERIC_SENSOR(CImpinjRFID)

		private:

			/** Poses (Antenna 1)
			 */
			float pose_x_1, pose_y_1, pose_z_1, pose_yaw_1, pose_pitch_1, pose_roll_1;

			/** Poses (Antenna 2)
			 */
			float pose_x_2, pose_y_2, pose_z_2, pose_yaw_2, pose_pitch_2, pose_roll_2;

			/** Server port
			 */
			int port;

			/** Server socket (listens for the incoming connection)
			 */
			mrpt::utils::CServerTCPSocket *server;

			/** Client socket (handles the connection to the client)
			 */
			mrpt::utils::CClientTCPSocket *client;



		public:
			/** Default constructor.
			 */
			CImpinjRFID();
			virtual ~CImpinjRFID(){};
			void connect();
			void doProcess(){};
			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string			&section);
			bool getObservation( mrpt::slam::CObservationRFID &outObservation );
			void closeReader();
		}; // End of class def.

	} // End of namespace
} // End of namespace
#endif
