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
	FILE: stereo_image.h
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#pragma once

#include <mrpt/obs/CObservationStereoImages.h>

#include <sensor_msgs/msg/image.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

namespace mrpt::ros2bridge
{
/** \addtogroup mrpt_ros2bridge_grp
 * @{ */

bool toROS(
	const mrpt::obs::CObservationStereoImages& obj,
	const std_msgs::msg::Header& msg_header, sensor_msgs::msg::Image& left,
	sensor_msgs::msg::Image& right,
	stereo_msgs::msg::DisparityImage& disparity);

/** @} */

}  // namespace mrpt::ros2bridge
