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

#include <mrpt/core/pimpl.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/poses/CPose3D.h>

namespace mrpt::hwdrivers
{
/** \brief Interfaces XSens 4th-generation IMUs (MTi 10-series, MTi 100-series)
 * via the XSens MT SDK.
 *
 * Communicates over USB or a serial COM port. Produces observations of type
 * mrpt::obs::CObservationIMU containing orientation, angular velocity, linear
 * acceleration, and optionally magnetometer data.
 *
 * \note Requires XSens MT SDK (libxsensmt or equivalent). On Linux, the user
 * must be in the 'dialout' group or install the provided udev rules.
 *
 * A class for interfacing XSens 4th generation Inertial Measuring Units
 * (IMUs): MTi 10-series, MTi 100-series.
 *  Usage considerations:
 *    - In Windows, you only need to install XSens drivers.
 *    - In Linux, this class requires the system libraries: libusb-1.0 &
 * libudev (dev packages). Accessing USB devices may require
 *      running the program as super user ("sudo"). To avoid that, Or, install
 * <code> MRPT/scripts/52-xsens.rules </code> in <code>/etc/udev/rules.d/</code>
 * to allow access to all users.
 *
 *  \code
 *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
 * -------------------------------------------------------
 *   [supplied_section_name]
 *    pose_x=0	    // Sensor 3D position relative to the robot (meters)
 *    pose_y=0
 *    pose_z=0
 *    pose_yaw=0	// Angles in degrees
 *    pose_pitch=0
 *    pose_roll=0
 *    sensorLabel = <label>   // Label of the sensor
 *    #sampleFreq  = 100  // The requested rate of sensor packets (default:
 * 100Hz)
 *    # If a portname is not provided, the first found device will be opened:
 *    #portname_LIN	= USB002:005
 *    #portname_WIN	= \\?\usb#vid_2639&pid_0003#...
 *    #baudRate	    = 115200   // Baudrate for communicating, only if
 *                             // the port is a COM port
 *    #deviceId     = xxxxx    // Device ID to open, or first one if empty.
 *    #logFile      = xxxx     // If provided, will enable XSens SDK's own log
 *  \endcode
 *
 *  \note Set the environment variable "MRPT_HWDRIVERS_VERBOSE" to "1" to
 * enable diagnostic information while using this class.
 *
 * \ingroup mrpt_hwdrivers_grp
 */
class CIMUXSens_MT4 : public hwdrivers::CGenericSensor
{
  DEFINE_GENERIC_SENSOR(CIMUXSens_MT4)
 public:
  /** \brief Default constructor. Does not open any device. */
  CIMUXSens_MT4();

  /** \brief Destructor. Closes the device if open. */
  ~CIMUXSens_MT4() override;

  /** \brief Polls the IMU for new data and queues any received observations.
   *
   * Called automatically at process_rate Hz by rawlog-grabber.
   * \exception std::exception On a critical hardware error.
   */
  void doProcess() override;

  /** \brief Opens the XSens device and configures it for data streaming.
   *
   * \exception std::exception If the device cannot be found or initialized.
   */
  void initialize() override;

  /** \brief Closes the connection to the XSens device. */
  void close();

 protected:
  /** The interface to the file: */
  struct Impl;
  mrpt::pimpl<Impl> m_impl;

  /** Baudrate, only for COM ports. */
  int m_port_bauds{0};
  /** The USB or COM port name (if blank -> autodetect) */
  std::string m_portname;

  /** Device ID to open, or first one if empty string. */
  std::string m_deviceId;

  std::string m_xsensLogFile;

  int m_sampleFreq{100};

  mrpt::poses::CPose3D m_sensorPose;

  /** See the class documentation at the top for expected parameters */
  void loadConfig_sensorSpecific(
      const mrpt::config::CConfigFileBase& configSource, const std::string& iniSection) override;

  friend class MyXSensCallback;

};  // end of class

}  // namespace mrpt::hwdrivers
