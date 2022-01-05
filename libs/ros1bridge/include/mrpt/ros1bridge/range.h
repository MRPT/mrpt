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
	FILE: range.h
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#pragma once

#include <mrpt/obs/CObservationRange.h>
#include <sensor_msgs/Range.h>

/// ROS message :   http://docs.ros.org/api/sensor_msgs/html/msg/Range.html
/// MRPT message:
/// https://github.com/MRPT/mrpt/blob/master/libs/obs/include/mrpt/obs/CObservationRange.h

namespace mrpt::ros1bridge
{
/** \addtogroup mrpt_ros1bridge_grp
 * @{ */

/** Convert sensor_msgs/Range -> mrpt::obs::CObservationRange
 * \return true on sucessful conversion, false on any error.
 */
bool fromROS(const sensor_msgs::Range& msg, mrpt::obs::CObservationRange& obj);

/** Convert mrpt::obs::CObservationRange -> sensor_msgs/Range
 *  The user must supply the "msg_header" field to be copied into the output
 * message object, since that part does not appear in MRPT classes.
 *
 *  Since COnservation does not contain "radiation_type",
 * sensor_msgs::Range::radiation_type will be empty. \return true on sucessful
 * conversion, false on any error.
 */
bool toROS(
	const mrpt::obs::CObservationRange& obj, const std_msgs::Header& msg_header,
	sensor_msgs::Range* msg);

/** @} */

}  // namespace mrpt::ros1bridge
