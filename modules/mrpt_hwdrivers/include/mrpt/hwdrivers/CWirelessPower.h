/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/obs/CObservationWirelessPower.h>

#include <optional>

namespace mrpt::hwdrivers
{
/** \brief Measures the received signal strength (RSSI) of a WiFi network.
 *
 * Reports the signal level (0-100) of the specified SSID/interface pair as
 * a mrpt::obs::CObservationWirelessPower observation, suitable for building
 * wireless power grid maps.
 *
 * \note Platform support: Windows (WlanAPI) and Linux (via /proc/net/wireless).
 * \sa mrpt::maps::CWirelessPowerGridMap2D,
 *     mrpt::obs::CObservationWirelessPower
 * \ingroup mrpt_hwdrivers_grp
 */
class CWirelessPower : public mrpt::hwdrivers::CGenericSensor
{
  DEFINE_GENERIC_SENSOR(CWirelessPower)

 private:
  /** SSID of the WiFi network
   */
  std::string m_ssid;

  /** GUID of the WiFi interface
   */
  std::string m_guid;

  /** Handle to the WLAN server (Windows)
   */
  void* hClient{nullptr};

  /** Poses
   */
  float pose_x{0}, pose_y{0}, pose_z{0}, pose_yaw{0}, pose_pitch{0}, pose_roll{0};

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
      const mrpt::config::CConfigFileBase& configSource, const std::string& section) override;

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

  /** \deprecated Use grabFrame() instead. */
  [[deprecated("Use grabFrame() instead")]] bool getObservation(
      mrpt::obs::CObservationWirelessPower& outObservation);

  /** Gets the power of a given network as a timestamped observation.
   * \return std::nullopt on any error, or the observation on success.
   */
  [[nodiscard]] std::optional<mrpt::obs::CObservationWirelessPower> grabFrame()
  {
    mrpt::obs::CObservationWirelessPower obs;
    if (!getObservation(obs))
    {
      return std::nullopt;
    }
    return obs;
  }

  /** Gets a list of the networks available for an interface
   * \exception std::exception In case there is a failure
   * \return std::vector returns handles to the available networks of a given
   * interface
   */
  std::vector<std::string> ListNetworks();

};  // End of class def.

}  // namespace mrpt::hwdrivers
