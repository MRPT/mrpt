/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/ros1bridge/point_cloud.h>
#include <ros/console.h>

using namespace mrpt::maps;

namespace mrpt::ros1bridge
{
bool fromROS(const sensor_msgs::PointCloud& msg, CSimplePointsMap& obj)
{
	const size_t N = msg.points.size();

	obj.clear();
	obj.reserve(N);
	for (size_t i = 0; i < N; i++)
		obj.insertPoint(msg.points[i].x, msg.points[i].y, msg.points[i].z);

	return true;
}

bool toROS(
	const CSimplePointsMap& obj, const std_msgs::Header& msg_header,
	sensor_msgs::PointCloud& msg)
{
	// 1) sensor_msgs::PointCloud:: header
	msg.header = msg_header;

	// 2) sensor_msgs::PointCloud:: points
	const size_t N = obj.size();
	msg.points.resize(N);
	for (size_t i = 0; i < N; i++)
	{
		geometry_msgs::Point32& pt = msg.points[i];
		obj.getPoint(i, pt.x, pt.y, pt.z);
	}

	// 3) sensor_msgs::PointCloud:: channels
	msg.channels.clear();

	return true;
}

}  // namespace mrpt::ros1bridge
