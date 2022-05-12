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
	FILE: range.cpp
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#include <mrpt/ros2bridge/range.h>

namespace mrpt::ros2bridge
{
bool fromROS(const sensor_msgs::msg::Range& msg, mrpt::obs::CObservationRange& obj)
{
	obj.minSensorDistance = msg.min_range;
	obj.maxSensorDistance = msg.max_range;
	obj.sensorConeApperture = msg.field_of_view;

	/// again this is amibiguous as can't be certain of number of measurement
	/// from corresponding ROS message
	obj.sensedData.at(0).sensedDistance = msg.range;
	return true;
}

bool toROS(
	const mrpt::obs::CObservationRange& obj, const std_msgs::Header& msg_header,
	sensor_msgs::msg::Range* msg)
{
	long num_range = obj.sensedData.size();

	// 1) sensor_msgs::msg::Range:: header
	for (long i = 0; i < num_range; i++)
		msg[i].header = msg_header;

	// 2) sensor_msg::Range parameters
	for (long i = 0; i < num_range; i++)
	{
		msg[i].max_range = obj.maxSensorDistance;
		msg[i].min_range = obj.minSensorDistance;
		msg[i].field_of_view = obj.sensorConeApperture;
	}

	/// following part needs to be double checked, it looks incorrect
	/// ROS has single number float for range, MRPT has a list of
	/// sensedDistances
	for (long i = 0; i < num_range; i++)
		msg[i].range = obj.sensedData.at(i).sensedDistance;

	/// currently the following are not available in MRPT for corresponding
	/// range ROS message NO corresponding value for MRPT radiation_type at
	/// http://mrpt.ual.es/reference/devel/_c_observation_range_8h_source.html
	// msg.radiation_type
	return true;
}

}  // namespace mrpt::ros2bridge

/// Range ROS message
/*
uint8 ULTRASOUND=0
uint8 INFRARED=1
std_msgs/Header header
uint8 radiation_type
float32 field_of_view
float32 min_range
float32 max_range
float32 range
*/
