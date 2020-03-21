/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
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
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

namespace mrpt::ros1bridge
{
/** \addtogroup mrpt_ros1bridge_grp
 * @{ */

bool toROS(
	const mrpt::obs::CObservationStereoImages& obj,
	const std_msgs::Header& msg_header, sensor_msgs::Image& left,
	sensor_msgs::Image& right, stereo_msgs::DisparityImage& disparity);

/** @} */

}  // namespace mrpt::ros1bridge
