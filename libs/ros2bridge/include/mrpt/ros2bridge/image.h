/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: mrpt_ros bridge
	FILE: image.h
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#pragma once

#include <mrpt/img/CImage.h>
#include <mrpt/obs/CObservationImage.h>

#include <cstring>	// size_t
#include <sensor_msgs/msg/image.hpp>

namespace mrpt::ros2bridge
{
/** \addtogroup mrpt_ros2bridge_grp
 * @{ */

/** Makes a deep copy of the image data */
mrpt::img::CImage fromROS(const sensor_msgs::msg::Image& i);

/** Makes a deep copy of the image data */
sensor_msgs::msg::Image toROS(
	const mrpt::img::CImage& i, const std_msgs::msg::Header& msg_header);
/** @} */

}  // namespace mrpt::ros2bridge
