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
