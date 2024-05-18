/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/comms/CSerialPort.h>
#include <mrpt/containers/circular_buffer.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>

#include <functional>

namespace mrpt::hwdrivers
{
/** A driver for Taobotics IMU.
 *
 * Supported models: HFI-A9, HFI-B9, HFI-B6.
 *
 * ## Frame format
 * From analysis the provided [python scripts]()
 * we found that the sensor uses these data frames, documented here for
 * reference:
 *
 * ### Model: "hfi-a9"
 *
 *  Two frames:
 *  Frame field :  0xAA 0x55 0x2c | (9 * 4[float]) D0-D8  | ?? ??
 *  Byte index  :   0     1    2  | 3 4 5 6 |  ...        | 48
 *   Data:
 *    - D{0,1,2}: wx,wy,wz
 *    - D{3,4,5}: accx,accy,accz
 *    - D{6,7,8}: magx,magy,magz
 *
 *  Frame field :  0xAA 0x55 0x14 | (5 * 4[float]) D0-D4   | ??
 *  Byte index  :   0     1    2  |  3 ... |      ...      | 24
 *
 * ### Model: "hfi-b6"
 *
 *  Frame field :  0x55 | TYPE |  D0  |  D1  |  D2  |  D3  | CHKSUM
 *  Byte index  :   0   |  1   | 2  3 | 4  5 | 6  7 | 8  9 |   10
 *  Type:
 *  0x51: acceleration, 0x52: angular velocity, 0x53: euler angles
 *
 * ### Configuration file block
 *  \code
 *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
 * -------------------------------------------------------
 *   [supplied_section_name]
 *    process_rate = 500   # (Hz) must be larger than the sensor rate
 *    serialPort  = /dev/ttyUSB0
 *    sensorModel = hfi-a9
 *    pose_x=0	    # Sensor 3D position relative to the robot (meters)
 *    pose_y=0
 *    pose_z=0
 *    pose_yaw=0	# Angles in degrees
 *    pose_pitch=0
 *    pose_roll=0
 *    sensorLabel = name   # Label of the sensor
 *  \endcode
 *
 * If used programmatically, this class will be used as:
 *
 * \code
 * CTaoboticsIMU imu;
 * /// ...
 * CConfigFile conf("conf.ini");
 * /// ...
 * imu.loadConfig_sensorSpecific(conf, "IMU");
 * /// ...
 * while(1) {
 * 	imu.doProcess();
 *	TListObservations rateObs;
 * 	imu.getObservations(rateObs);
 *	// ....
 * \endcode
 * \ingroup mrpt_hwdrivers_grp
 */
class CTaoboticsIMU : public hwdrivers::CGenericSensor
{
  DEFINE_GENERIC_SENSOR(CTaoboticsIMU)

 public:
  CTaoboticsIMU();
  ~CTaoboticsIMU() override;

  /** See the class documentation at the top for expected parameters */
  void loadConfig_sensorSpecific(
      const mrpt::config::CConfigFileBase& configSource, const std::string& iniSection) override;

  /** This method will be invoked at a minimum rate of "process_rate" (Hz)
   *  \exception This method must throw an exception with a descriptive
   * message if some critical error is found.
   */
  void doProcess() override;

  /** Opens the serial port and start streaming data.
   * You must have called loadConfig_sensorSpecific before
   * calling this function, or set the configuration via the provided methods,
   * e.g. setSerialPort(), etc.
   */
  void initialize() override;

  /// Must be called before initialize(). If not set, the default is
  /// "/dev/ttyUSB0". Use "COM1", etc. for Windows.
  void setSerialPort(const std::string& serialPort);

  /// Must be called before initialize(). If not called, the default 921600 is
  /// used.
  void setSerialBaudRate(int rate);

 protected:
  /** This serial port will be attempted to be opened automatically when this
   * class is first used to request data from the device.
   * \sa comms::CSerialPort
   */
  int m_baudRate = 921600;
  std::string m_com_port = "/dev/ttyUSB0";
  std::string m_sensorModel = "hfi-a9";
  mrpt::poses::CPose3D m_sensorPose;

  /** Auxiliary buffer for readings */
  mrpt::containers::circular_buffer<uint8_t> m_rx_buffer{4096};

  /** The serial port connection */
  std::unique_ptr<mrpt::comms::CSerialPort> m_serialPort;

  mrpt::obs::CObservationIMU::Ptr m_observationGyro;

  using parser_t = std::function<std::vector<mrpt::obs::CObservation::Ptr>(
      CTaoboticsIMU*, mrpt::containers::circular_buffer<uint8_t>& /*buf*/)>;

  parser_t m_activeParser;

  std::vector<mrpt::obs::CObservation::Ptr> parser_hfi_b6(
      mrpt::containers::circular_buffer<uint8_t>& buf) const;

  std::vector<mrpt::obs::CObservation::Ptr> parser_hfi_a9(
      mrpt::containers::circular_buffer<uint8_t>& buf) const;

};  // end of class

}  // namespace mrpt::hwdrivers
