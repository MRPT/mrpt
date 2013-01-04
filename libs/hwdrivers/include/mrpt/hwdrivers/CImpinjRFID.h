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

			/** IP of the module (usually 127.0.0.1)
			 */
			std::string IPm;

			/** Reader name
			 */
			std::string reader_name;

			/** Server socket (listens for the incoming connection)
			 */
			mrpt::utils::CServerTCPSocket *server;

			/** Client socket (handles the connection to the client)
			 */
			mrpt::utils::CClientTCPSocket *client;

			/** Driver executable path
			 */
			std::string driver_path;

			/** Connection status
			 */
			bool connected;

			/** start the external driver
			 */
			void startDriver();

			static void dummy_startDriver(CImpinjRFID *o);


		public:
			/** Default constructor.
			 */
			CImpinjRFID();
			virtual ~CImpinjRFID();

			/** Connect to the reader.
			 */
			void connect();

			void doProcess();

			void initialize();


			void  loadConfig_sensorSpecific(
				const mrpt::utils::CConfigFileBase &configSource,
				const std::string			&section);

			/** Gets the information of the tags as a timestamped observation
			* NOTE: Deprecated, use getObservations instead. See CGenericSensor documentation. This function is kept for internal use of the module
			* \return Returns true if the observation was correct, and false otherwise
			* \sa mrpt::hwdrivers::CGenericSensor
			*/
			bool getObservation( mrpt::slam::CObservationRFID &obs );


			/** Close the connection to the reader.
			 */
			void closeReader();
		}; // End of class def.

	} // End of namespace
} // End of namespace
#endif
