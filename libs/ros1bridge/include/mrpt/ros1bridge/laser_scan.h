/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <geometry_msgs/Pose.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/poses/poses_frwds.h>
#include <sensor_msgs/LaserScan.h>

#include <cstdint>
#include <string>

namespace mrpt::ros1bridge
{
/** \addtogroup mrpt_ros1bridge_grp
 * @{ */

/** ROS->MRPT: Takes a sensor_msgs::LaserScan and the relative pose of the laser
 * wrt base_link and builds a CObservation2DRangeScan
 * \return true on sucessful conversion, false on any error.
 * \sa toROS
 */
bool fromROS(
	const sensor_msgs::LaserScan& msg, const mrpt::poses::CPose3D& pose,
	mrpt::obs::CObservation2DRangeScan& obj);

/** MRPT->ROS: Takes a CObservation2DRangeScan and outputs range data in
 * sensor_msgs::LaserScan
 * \return true on sucessful conversion, false on any error.
 * \sa fromROS
 */
bool toROS(
	const mrpt::obs::CObservation2DRangeScan& obj, sensor_msgs::LaserScan& msg);

/** MRPT->ROS: Takes a CObservation2DRangeScan and outputs range data in
 * sensor_msgs::LaserScan + the relative pose of the laser wrt base_link
 * \return true on sucessful conversion, false on any error.
 * \sa fromROS
 */
bool toROS(
	const mrpt::obs::CObservation2DRangeScan& obj, sensor_msgs::LaserScan& msg,
	geometry_msgs::Pose& pose);

/** @} */

}  // namespace mrpt::ros1bridge
