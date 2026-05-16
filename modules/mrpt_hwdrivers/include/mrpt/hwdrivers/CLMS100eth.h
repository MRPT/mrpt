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

#include <mrpt/comms/CClientTCPSocket.h>
#include <mrpt/hwdrivers/C2DRangeFinderAbstract.h>

namespace mrpt::hwdrivers
{
/** \brief Driver for the SICK LMS100 2-D laser range-finder over Ethernet.
 *
 * Connects to the device via TCP/IP and acquires 2-D laser scans as
 * mrpt::obs::CObservation2DRangeScan observations. The fixed hardware
 * parameters of the LMS100 are: FOV 270 deg, angular resolution 0.25 deg,
 * scan frequency 25 Hz, and maximum range 20 m.
 *
 * \warning Before using this class the scanner must be pre-configured with
 * the SICK SOPAS software at least once (to set the frame rate and save to
 * flash), because this driver does not handle the full SOPAS configuration
 * protocol.
 *
 * \note No external library is required; communication is via a plain TCP
 * socket through mrpt::comms::CClientTCPSocket.
 *
 * This "software driver" implements the communication protocol for interfacing
 *a SICK LMS100 laser scanners through an ethernet controller.
 *   This class does not need to be bind, i.e. you do not need to call
 *C2DRangeFinderAbstract::bindIO.
 *   Connection is established when user call the turnOn() method. You can
 *pass to the class's constructor the LMS100 's ip address and port.
 *   Device will be configured with the following parameters :
 * - Start Angle : -45 deg (imposed by hardware)
 * - Stop Angle : +225 deg (imposed by hardware)
 * - Apperture : 270 deg (imposed by hardware)
 * - Angular resolution : 0.25 deg
 * - Scan frequency : 25 Hz
 * - Max Range : 20m (imposed by hardware).
 *
 * <b>Important note:</b> SICK LMS 1xx devices have two levels of
 *configuration. In its present implementation, this class only handles one of
 *them, so
 *    <b>before using this class</b>, you must "pre-configure" your scanner
 *with the SICK's software "SOAP" (this software ships with the device),
 *    and set the framerate with this software. Of course, you have to
 *pre-configure the device just once, then save that configuration in its flash
 *memory.
 *
 * To get a laser scan you must proceed like that :
 * \code
 *     CLMS200Eth laser(string("192.168.0.10"), 1234);
 *     laser.turnOn();
 *     bool isOutObs, hardwareError;
 *     CObservation2DRangeScan outObs;
 *     laser.doProcessSimple(isOutObs, outObs, hardwareError);
 * \endcode
 *
 * The sensor pose on the vehicle could be loaded from an ini configuration
 *file with :
 *  \code
 *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
 * -------------------------------------------------------
 *   [supplied_section_name]
 *	  ip_address = 192.168.0.50 ;a string which is the SICK's ip adress
 *(default is 192.168.0.1)
 *   TCP_port = 1234			; an integer value : the tcp ip port on which the
 *sick is listening (default is 2111).
 *   pose_x=0.21	; Laser range scaner 3D position in the robot (meters)
 *   pose_y=0
 *   pose_z=0.34
 *   pose_yaw=0	; Angles in degrees
 *   pose_pitch=0
 *   pose_roll=0
 *  \endcode
 * This class doesn't configure the SICK LMS sensor, it is recommended to
 *configure the sensor via the
 * the SICK software : SOPAS.
 * \note This class was contributed by Adrien Barral - Robopec (France)
 * \ingroup mrpt_hwdrivers_grp
 */
class CLMS100Eth : public C2DRangeFinderAbstract
{
  DEFINE_GENERIC_SENSOR(CLMS100Eth)
 public:
  /** \brief Constructor.
   *
   * \param[in] _ip   IPv4 address of the LMS100 (default "192.168.0.1").
   * \param[in] _port TCP port on the device (default 2111).
   */
  CLMS100Eth(std::string _ip = std::string("192.168.0.1"), unsigned int _port = 2111);

  /** \brief Destructor. Closes the TCP connection and frees resources. */
  ~CLMS100Eth() override;

  /** \brief Acquires one laser scan from the device.
   *
   * \param[out] outThereIsObservation Set to true when a valid scan is ready.
   * \param[out] outObservation        Filled with the new scan on success.
   * \param[out] hardwareError         Set to true on communication failure.
   * \exception std::exception If the received frame has an unexpected status
   *            or unsupported data channel (e.g. RSSI instead of DIST1).
   */
  void doProcessSimple(
      bool& outThereIsObservation,
      mrpt::obs::CObservation2DRangeScan& outObservation,
      bool& hardwareError) override;

  /** \brief Opens the TCP connection and starts the measurement stream.
   *
   * Must be called before requesting any laser scans.
   * \return true on success, false on error.
   */
  bool turnOn() override;

  /** \brief Stops measurement and closes the TCP connection.
   *
   * Also called automatically by the destructor.
   * \return true on success, false on error.
   */
  bool turnOff() override;

  /** \brief Sets the 3-D pose of the sensor on the robot.
   *
   * Equivalent to loading the pose from a configuration file.
   * \param[in] _pose The sensor pose in robot coordinates.
   */
  void setSensorPose(const mrpt::poses::CPose3D& _pose);

  /** \brief Periodic processing step (called at process_rate Hz).
   *
   * Internally calls doProcessSimple() and queues the resulting observation.
   */
  void doProcess() override;

  /** \brief Initializes the sensor using parameters loaded from a config file.
   *
   * \exception std::exception On failure to connect or configure the device.
   */
  void initialize() override;

 private:
  std::string m_ip;
  unsigned int m_port = 0;
  mrpt::comms::CClientTCPSocket m_client;
  bool m_turnedOn{false};
  std::string m_cmd;
  bool m_connected{false};
  unsigned int m_scanFrequency = 0;  // hertz
  double m_angleResolution = 0;      // degrees
  double m_startAngle = 0;           // degrees
  double m_stopAngle = 0;            // degrees
  mrpt::poses::CPose3D m_sensorPose;
  double m_maxRange{20.0};
  double m_beamApperture = 0;

  void generateCmd(const char* cmd);
  bool checkIsConnected();
  bool decodeLogIn(char* msg);
  bool decodeScanCfg(std::istringstream& stream);
  bool decodeScanDataCfg(std::istringstream& stream);
  bool decodeScan(char* buf, mrpt::obs::CObservation2DRangeScan& outObservation);
  void sendCommand(const char* cmd);
  void roughPrint(char* msg);

 protected:
  /** Load sensor pose on the robot, or keep the default sensor pose.
   */
  void loadConfig_sensorSpecific(
      const mrpt::config::CConfigFileBase& configSource, const std::string& iniSection) override;
};
}  // namespace mrpt::hwdrivers
