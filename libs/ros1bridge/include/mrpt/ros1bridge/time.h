/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/system/datetime.h>
#include <ros/time.h>

namespace mrpt::ros1bridge
{
/** \addtogroup mrpt_ros1bridge_grp
 * @{ */

/**
 * converts ros time to mrpt time
 * @param src ros time
 * @param des mrpt time
 */
mrpt::system::TTimeStamp fromROS(const ros::Time& src);

/**
 * converts mrpt time to ros time
 * @param src ros time
 * @param des mrpt time
 */
ros::Time toROS(const mrpt::system::TTimeStamp& src);

/** @} */

};	// namespace mrpt::ros1bridge
