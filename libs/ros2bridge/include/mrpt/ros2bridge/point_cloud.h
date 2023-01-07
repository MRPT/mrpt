/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <sensor_msgs/msg/point_cloud.hpp>

namespace mrpt::ros2bridge
{
/** \addtogroup mrpt_ros2bridge_grp
 * @{ */

/** @name Point clouds: ROS <-> MRPT
 *  @{ */

/** Convert sensor_msgs/PointCloud -> mrpt::maps::CSimplePointsMap
 *  CSimplePointsMap only contains (x,y,z) data, so
 * sensor_msgs::PointCloud::channels are ignored.
 * \return true on sucessful conversion, false on any error.
 * \sa toROS
 */
bool fromROS(
	const sensor_msgs::msg::PointCloud& msg, mrpt::maps::CSimplePointsMap& obj);

/** Convert mrpt::maps::CSimplePointsMap -> sensor_msgs/PointCloud
 *  The user must supply the "msg_header" field to be copied into the output
 * message object, since that part does not appear in MRPT classes.
 *
 *  Since CSimplePointsMap only contains (x,y,z) data,
 * sensor_msgs::PointCloud::channels will be empty.
 * \return true on sucessful conversion, false on any error.
 * \sa fromROS
 */
bool toROS(
	const mrpt::maps::CSimplePointsMap& obj,
	const std_msgs::msg::Header& msg_header, sensor_msgs::msg::PointCloud& msg);

/** @} @}
 */

}  // namespace mrpt::ros2bridge
