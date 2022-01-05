/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: mrpt_ros bridge
	FILE: imu.h
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/
#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <mrpt/obs/CObservationIMU.h>
#include <sensor_msgs/Imu.h>

#include <cstring>	// size_t

/// ROS message:    http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
/// MRPT message:
/// https://github.com/MRPT/mrpt/blob/master/libs/obs/include/mrpt/obs/CObservationIMU.h

namespace mrpt::ros1bridge
{
/** \addtogroup mrpt_ros1bridge_grp
 * @{ */

/** Convert sensor_msgs/Imu -> mrpt::obs::CObservationIMU
 * // STILL NEED TO WRITE CODE FOR COVARIANCE
 * \return true on sucessful conversion, false on any error.
 */
bool fromROS(const sensor_msgs::Imu& msg, mrpt::obs::CObservationIMU& obj);

/** Convert mrpt::obs::CObservationIMU -> sensor_msgs/Imu
 *  The user must supply the "msg_header" field to be copied into the output
 * message object, since that part does not appear in MRPT classes.
 *
 *  Since COnservationIMU does not contain covariance terms NEED TO fix those.
 * \return true on sucessful conversion, false on any error.
 */
bool toROS(
	const mrpt::obs::CObservationIMU& obj, const std_msgs::Header& msg_header,
	sensor_msgs::Imu& msg);

/** @} */

}  // namespace mrpt::ros1bridge
