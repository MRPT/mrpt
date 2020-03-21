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
	FILE: imu.cpp
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#include <mrpt/ros1bridge/imu.h>

namespace mrpt::ros1bridge
{
bool fromROS(const sensor_msgs::Imu& msg, mrpt::obs::CObservationIMU& obj)
{
	using namespace mrpt::obs;

	obj.set(IMU_ORI_QUAT_X, msg.orientation.x);
	obj.set(IMU_ORI_QUAT_Y, msg.orientation.y);
	obj.set(IMU_ORI_QUAT_Z, msg.orientation.z);
	obj.set(IMU_ORI_QUAT_W, msg.orientation.w);

	obj.set(IMU_X_ACC, msg.linear_acceleration.x);
	obj.set(IMU_Y_ACC, msg.linear_acceleration.y);
	obj.set(IMU_Z_ACC, msg.linear_acceleration.z);

	obj.set(IMU_WX, msg.angular_velocity.x);
	obj.set(IMU_WY, msg.angular_velocity.y);
	obj.set(IMU_WZ, msg.angular_velocity.z);

	// NEED TO WRITE CODE FOR COVARIANCE
	return true;
}

bool toROS(
	const mrpt::obs::CObservationIMU& obj, const std_msgs::Header& msg_header,
	sensor_msgs::Imu& msg)
{
	using namespace mrpt::obs;

	msg.header = msg_header;

	const auto& measurements = obj.rawMeasurements;
	msg.orientation.x = measurements.at(IMU_ORI_QUAT_X);
	msg.orientation.y = measurements.at(IMU_ORI_QUAT_Y);
	msg.orientation.z = measurements.at(IMU_ORI_QUAT_Z);
	msg.orientation.w = measurements.at(IMU_ORI_QUAT_W);

	/// computing acceleration in global navigation frame not in local vehicle
	/// frame, might be the other way round
	msg.linear_acceleration.x = measurements.at(IMU_X_ACC);
	msg.linear_acceleration.y = measurements.at(IMU_Y_ACC);
	msg.linear_acceleration.z = measurements.at(IMU_Z_ACC);

	msg.angular_velocity.x = measurements.at(IMU_WX);
	msg.angular_velocity.y = measurements.at(IMU_WY);
	msg.angular_velocity.z = measurements.at(IMU_WZ);

	// msg.angular_velocity_covariance
	return true;
}

}  // namespace mrpt::ros1bridge

/*
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
 */
