/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef  CWirelessPower_H
#define  CWirelessPower_H

#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/obs/CObservationWirelessPower.h>
#include <mrpt/utils/CConfigFileBase.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** This class implements a wireless power probe.
		  *  \sa mrpt::maps::CWirelessPowerGridMap2D, mrpt::obs::CObservationWirelessPower
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

			bool getObservation( mrpt::obs::CObservationWirelessPower &outObservation );


			/** Gets a list of the networks available for an interface
			 * \exception std::exception In case there is a failure
			 * \return std::vector returns handles to the available networks of a given interface
			 */
			std::vector<std::string>	ListNetworks();

		}; // End of class def.

	} // End of namespace
} // End of namespace
#endif
