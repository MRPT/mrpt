/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CImpinjRFID_H
#define  CImpinjRFID_H

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/obs/CObservationRFID.h>
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
			bool getObservation( mrpt::obs::CObservationRFID &obs );


			/** Close the connection to the reader.
			 */
			void closeReader();
		}; // End of class def.

	} // End of namespace
} // End of namespace
#endif
