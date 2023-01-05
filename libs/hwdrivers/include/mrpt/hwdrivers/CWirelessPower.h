/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/obs/CObservationWirelessPower.h>

namespace mrpt::hwdrivers
{
/** This class implements a wireless power probe.
 *  \sa mrpt::maps::CWirelessPowerGridMap2D,
 * mrpt::obs::CObservationWirelessPower
 * \ingroup mrpt_hwdrivers_grp
 */
class CWirelessPower : public mrpt::hwdrivers::CGenericSensor
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
	void* hClient{nullptr};

	/** Poses
	 */
	float pose_x{0}, pose_y{0}, pose_z{0}, pose_yaw{0}, pose_pitch{0},
		pose_roll{0};

   public:
	/** Default constructor.
	 */
	CWirelessPower();
	~CWirelessPower() override = default;

	/** Set the SSID and GUID of the target network.
	 * \exception std::exception In case there is a failure
	 * \param ssid SSID of the target network
	 * \param guid GUID of the network interface
	 */
	void setNet(std::string ssid, std::string guid);

	void doProcess() override;
	void loadConfig_sensorSpecific(
		const mrpt::config::CConfigFileBase& configSource,
		const std::string& section) override;

	/** Gets a list of the interfaces
	 * \exception std::exception In case there is a failure
	 * \return std::vector returns the identifiers (GUID) of the available
	 * interfaces
	 */
	std::vector<std::string> ListInterfaces();

	/** Gets the power of a given network
	 * \exception std::exception In case there is a failure
	 * \return Returns the power (0-100).
	 */
	int GetPower();

	/** Gets the power of a given network as a timestamped observation
	 * NOTE: Deprecated, use getObservations instead. See CGenericSensor
	 * documentation. This function is kept for internal use of the module
	 * \return Returns true if the observation was correct, and false otherwise
	 * \sa mrpt::hwdrivers::CGenericSensor
	 */

	bool getObservation(mrpt::obs::CObservationWirelessPower& outObservation);

	/** Gets a list of the networks available for an interface
	 * \exception std::exception In case there is a failure
	 * \return std::vector returns handles to the available networks of a given
	 * interface
	 */
	std::vector<std::string> ListNetworks();

};	// End of class def.

}  // namespace mrpt::hwdrivers
