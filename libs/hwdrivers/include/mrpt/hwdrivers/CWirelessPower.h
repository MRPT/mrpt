/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>		                       |
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
#ifndef  CWirelessPower_H
#define  CWirelessPower_H

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/slam/CObservationWirelessPower.h>
#include <mrpt/utils/CConfigFileBase.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** This class implements a wireless power probe.
		  *  \sa mrpt::slam::CWirelessPowerGridMap2D, mrpt::slam::CObservationWirelessPower
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP CWirelessPower : public mrpt::hwdrivers::CGenericSensor
		{
				DEFINE_GENERIC_SENSOR(CWirelessPower)

		private:
			/** SSID of the WiFi network
			 */
			std::string ssid;

			/** GUID of the WiFi interface
			 */
			std::string guid;

			/** Handle to the WLAN server (Windows)
			 */
			void* hClient;

			/** Poses
			 */
			float pose_x, pose_y, pose_z, pose_yaw, pose_pitch, pose_roll;

		public:
			/** Default constructor.
			 */
			CWirelessPower();
			virtual ~CWirelessPower(){};

			/** Set the SSID and GUID of the target network.
			 * \exception std::exception In case there is a failure
			 * \param ssid_ SSID of the target network
			 * \param guid_ GUID of the network interface
			 */
			void setNet(std::string ssid_, std::string guid_);

			 void doProcess();
			 void  loadConfig_sensorSpecific(
				 const mrpt::utils::CConfigFileBase &configSource,
				const std::string			&section);

			/** Gets a list of the interfaces
			 * \exception std::exception In case there is a failure
			 * \return std::vector returns the identifiers (GUID) of the available interfaces
			 */
			std::vector<std::string>	ListInterfaces();


			/** Gets the power of a given network
			 * \exception std::exception In case there is a failure
			 * \return Returns the power (0-100).
			 */
			int		GetPower();


			/** Gets the power of a given network as a timestamped observation
			 * NOTE: Deprecated, use getObservations instead. See CGenericSensor documentation. This function is kept for internal use of the module
			 * \return Returns true if the observation was correct, and false otherwise
			 * \sa mrpt::hwdrivers::CGenericSensor
			 */

			bool getObservation( mrpt::slam::CObservationWirelessPower &outObservation );


			/** Gets a list of the networks available for an interface
			 * \exception std::exception In case there is a failure
			 * \return std::vector returns handles to the available networks of a given interface
			 */
			std::vector<std::string>	ListNetworks();

		}; // End of class def.

	} // End of namespace
} // End of namespace
#endif
