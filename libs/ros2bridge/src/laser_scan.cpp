/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/ros2bridge/laser_scan.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/version.h>

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

bool mrpt::ros2bridge::fromROS(
	const sensor_msgs::msg::LaserScan& msg, const mrpt::poses::CPose3D& pose,
	mrpt::obs::CObservation2DRangeScan& obj)
{
	obj.timestamp = fromROS(msg.header.stamp);
	obj.rightToLeft = true;
	obj.sensorLabel = msg.header.frame_id;
	obj.aperture = msg.angle_max - msg.angle_min;
	obj.maxRange = msg.range_max;
	obj.sensorPose = pose;

	ASSERT_(msg.ranges.size() > 1);

	const size_t N = msg.ranges.size();
	const double ang_step = obj.aperture / (N - 1);
	const double fov05 = 0.5 * obj.aperture;
	const double inv_ang_step = (N - 1) / obj.aperture;

	obj.resizeScan(N);
	for (std::size_t i_mrpt = 0; i_mrpt < N; i_mrpt++)
	{
		// ROS indices go from msg.angle_min to msg.angle_max, while
		// in MRPT they go from -FOV/2 to +FOV/2.
		int i_ros = inv_ang_step * (-fov05 - msg.angle_min + ang_step * i_mrpt);
		if (i_ros < 0) i_ros += N;
		else if (i_ros >= (int)N)
			i_ros -= N;	 // wrap around 2PI...

		// set the scan
		const float r = msg.ranges[i_ros];
		obj.setScanRange(i_mrpt, r);

		// set the validity of the scan
		const auto ri = obj.getScanRange(i_mrpt);
		const bool r_valid =
			((ri < (msg.range_max * 0.99)) && (ri > msg.range_min));
		obj.setScanRangeValidity(i_mrpt, r_valid);
	}

	return true;
}

bool mrpt::ros2bridge::toROS(
	const mrpt::obs::CObservation2DRangeScan& obj,
	sensor_msgs::msg::LaserScan& msg)
{
	const size_t nRays = obj.getScanSize();
	if (!nRays) return false;

	msg.angle_min = -0.5f * obj.aperture;
	msg.angle_max = 0.5f * obj.aperture;
	msg.angle_increment = obj.aperture / (obj.getScanSize() - 1);

	// setting the following values to zero solves a rviz visualization problem
	msg.time_increment = 0.0;  // 1./30.; // Anything better?
	msg.scan_time = 0.0;  // msg.time_increment; // idem?

	msg.range_min = 0.02f;
	msg.range_max = obj.maxRange;

	msg.ranges.resize(nRays);
	for (size_t i = 0; i < nRays; i++)
		msg.ranges[i] = obj.getScanRange(i);

	// Set header data:
	msg.header.stamp = toROS(obj.timestamp);
	msg.header.frame_id = obj.sensorLabel;

	return true;
}

bool mrpt::ros2bridge::toROS(
	const mrpt::obs::CObservation2DRangeScan& obj,
	sensor_msgs::msg::LaserScan& msg, geometry_msgs::msg::Pose& pose)
{
	toROS(obj, msg);
	pose = toROS_Pose(obj.sensorPose);
	return true;
}
