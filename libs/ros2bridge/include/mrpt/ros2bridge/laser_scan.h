/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/obs_frwds.h>
#include <mrpt/poses/poses_frwds.h>

#include <cstdint>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

namespace mrpt::ros2bridge
{
/** \addtogroup mrpt_ros2bridge_grp
 * @{ */

/** ROS->MRPT: Takes a sensor_msgs::msg::LaserScan and the relative pose of the
 * laser wrt base_link and builds a CObservation2DRangeScan \return true on
 * sucessful conversion, false on any error. \sa toROS
 */
bool fromROS(
	const sensor_msgs::msg::LaserScan& msg, const mrpt::poses::CPose3D& pose,
	mrpt::obs::CObservation2DRangeScan& obj);

/** MRPT->ROS: Takes a CObservation2DRangeScan and outputs range data in
 * sensor_msgs::msg::LaserScan
 * \return true on sucessful conversion, false on any error.
 * \sa fromROS
 */
bool toROS(
	const mrpt::obs::CObservation2DRangeScan& obj,
	sensor_msgs::msg::LaserScan& msg);

/** MRPT->ROS: Takes a CObservation2DRangeScan and outputs range data in
 * sensor_msgs::msg::LaserScan + the relative pose of the laser wrt base_link
 * \return true on sucessful conversion, false on any error.
 * \sa fromROS
 */
bool toROS(
	const mrpt::obs::CObservation2DRangeScan& obj,
	sensor_msgs::msg::LaserScan& msg, geometry_msgs::msg::Pose& pose);

/** @} */

}  // namespace mrpt::ros2bridge
