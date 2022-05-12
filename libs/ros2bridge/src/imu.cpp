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
	FILE: imu.cpp
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#include <mrpt/ros2bridge/imu.h>

namespace mrpt::ros2bridge
{
bool fromROS(const sensor_msgs::Imu& msg, mrpt::obs::CObservationIMU& obj)
{
	using namespace mrpt::obs;

	if (msg.orientation_covariance.at(0) >= 0)
	{
		obj.set(IMU_ORI_QUAT_X, msg.orientation.x);
		obj.set(IMU_ORI_QUAT_Y, msg.orientation.y);
		obj.set(IMU_ORI_QUAT_Z, msg.orientation.z);
		obj.set(IMU_ORI_QUAT_W, msg.orientation.w);
	}

	if (msg.linear_acceleration_covariance.at(0) >= 0)
	{
		obj.set(IMU_X_ACC, msg.linear_acceleration.x);
		obj.set(IMU_Y_ACC, msg.linear_acceleration.y);
		obj.set(IMU_Z_ACC, msg.linear_acceleration.z);
	}

	if (msg.angular_velocity_covariance.at(0) >= 0)
	{
		obj.set(IMU_WX, msg.angular_velocity.x);
		obj.set(IMU_WY, msg.angular_velocity.y);
		obj.set(IMU_WZ, msg.angular_velocity.z);
	}

	return true;
}

bool toROS(
	const mrpt::obs::CObservationIMU& obj, const std_msgs::Header& msg_header,
	sensor_msgs::Imu& msg)
{
	using namespace mrpt::obs;

	msg.header = msg_header;

	const auto& m = obj.rawMeasurements;
	if (obj.has(IMU_ORI_QUAT_X))
	{
		msg.orientation.x = m.at(IMU_ORI_QUAT_X);
		msg.orientation.y = m.at(IMU_ORI_QUAT_Y);
		msg.orientation.z = m.at(IMU_ORI_QUAT_Z);
		msg.orientation.w = m.at(IMU_ORI_QUAT_W);

		msg.orientation_covariance.fill(0.01);
	}
	else
	{
		msg.orientation_covariance.fill(-1);
	}

	if (obj.has(IMU_X_ACC))
	{
		msg.linear_acceleration.x = m.at(IMU_X_ACC);
		msg.linear_acceleration.y = m.at(IMU_Y_ACC);
		msg.linear_acceleration.z = m.at(IMU_Z_ACC);

		msg.linear_acceleration_covariance.fill(0.01);
	}
	else
	{
		msg.linear_acceleration_covariance.fill(-1);
	}

	if (obj.has(IMU_WX))
	{
		msg.angular_velocity.x = m.at(IMU_WX);
		msg.angular_velocity.y = m.at(IMU_WY);
		msg.angular_velocity.z = m.at(IMU_WZ);

		msg.angular_velocity_covariance.fill(0.01);
	}
	else
	{
		msg.angular_velocity_covariance.fill(-1);
	}

	return true;
}

}  // namespace mrpt::ros2bridge

/*
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
 */
