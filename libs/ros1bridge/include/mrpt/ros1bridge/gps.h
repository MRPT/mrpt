/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
  APPLICATION: mrpt_ros bridge
  FILE: GPS.h
  AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#pragma once

#include <mrpt/obs/CObservationGPS.h>
#include <sensor_msgs/NavSatFix.h>

/// ROS message:    http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html
/// MRPT message:
/// https://github.com/MRPT/mrpt/blob/master/libs/obs/include/mrpt/obs/CObservationGPS.h

/** Conversion functions between ROS 1 <-> MRPT types.
 * \ingroup mrpt_ros1bridge_grp
 */
namespace mrpt::ros1bridge
{
/** \addtogroup mrpt_ros1bridge_grp
 * @{ */

/** Convert sensor_msgs/NavSatFix -> mrpt::obs::CObservationGPS
 * \return true on sucessful conversion, false on any error.
 */
bool fromROS(const sensor_msgs::NavSatFix& msg, mrpt::obs::CObservationGPS& obj);

/** Convert mrpt::obs::CObservationGPS -> sensor_msgs/NavSatFix
 *  The user must supply the "msg_header" field to be copied into the output
 * message object, since that part does not appear in MRPT classes.
 *
 * \return true on sucessful conversion, only if the input observation has a GGA
 * message.
 */
bool toROS(
    const mrpt::obs::CObservationGPS& obj,
    const std_msgs::Header& msg_header,
    sensor_msgs::NavSatFix& msg);

/** @} */

}  // namespace mrpt::ros1bridge
