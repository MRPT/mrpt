/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*
 * pose.cpp
 *
 *  Created on: Mar 14, 2012
 *      Author: Pablo Iñigo Blasco
 *
 *      To understand better how this is implemented see the references:
 *      - http://www.mrpt.org/2D_3D_Geometry
 *
 * Aug 17, 2019: Refactored into mrpt::ros2bridge library for MRPT 2.0 (JLBC)
 *
 */

#if !defined(IS_MRPT_ROS1BRIDGE)
// ROS1
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#else
// ROS2
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#endif

#include <mrpt/core/exceptions.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussianInf.h>
#include <mrpt/ros2bridge/pose.h>
#include <tf2/LinearMath/Matrix3x3.h>

// MRPT -> ROS functions:
namespace mrpt::ros2bridge
{
tf2::Matrix3x3 toROS(const mrpt::math::CMatrixDouble33& src)
{
	tf2::Matrix3x3 des;
	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++)
			des[r][c] = src(r, c);
	return des;
}

tf2::Transform toROS_tfTransform(const mrpt::poses::CPose2D& src)
{
	return toROS_tfTransform(mrpt::poses::CPose3D(src));
}

geometry_msgs::msg::Pose toROS_Pose(const mrpt::poses::CPose2D& src)
{
	return toROS_Pose(mrpt::poses::CPose3D(src));
}

tf2::Transform toROS_tfTransform(const mrpt::math::TPose2D& src)
{
	return toROS_tfTransform(mrpt::poses::CPose3D(mrpt::math::TPose3D(src)));
}

geometry_msgs::msg::Pose toROS_Pose(const mrpt::math::TPose2D& src)
{
	geometry_msgs::msg::Pose des;

	des.position.x = src.x;
	des.position.y = src.y;
	des.position.z = 0;

	const double yaw = src.phi;
	if (std::abs(yaw) < 1e-10)
	{
		des.orientation.x = 0.;
		des.orientation.y = 0.;
		des.orientation.z = .5 * yaw;
		des.orientation.w = 1.;
	}
	else
	{
		const double s = ::sin(yaw * .5);
		const double c = ::cos(yaw * .5);
		des.orientation.x = 0.;
		des.orientation.y = 0.;
		des.orientation.z = s;
		des.orientation.w = c;
	}

	return des;
}

tf2::Transform toROS_tfTransform(const mrpt::poses::CPose3D& src)
{
	tf2::Transform des;
	des.setBasis(toROS(src.getRotationMatrix()));
	des.setOrigin(tf2::Vector3(src.x(), src.y(), src.z()));
	return des;
}

geometry_msgs::msg::Pose toROS_Pose(const mrpt::poses::CPose3D& src)
{
	geometry_msgs::msg::Pose des;
	des.position.x = src.x();
	des.position.y = src.y();
	des.position.z = src.z();

	mrpt::math::CQuaternionDouble q;
	src.getAsQuaternion(q);

	des.orientation.x = q.x();
	des.orientation.y = q.y();
	des.orientation.z = q.z();
	des.orientation.w = q.r();

	return des;
}

tf2::Transform toROS_tfTransform(const mrpt::math::TPose3D& src)
{
	return toROS_tfTransform(mrpt::poses::CPose3D(src));
}

geometry_msgs::msg::Pose toROS_Pose(const mrpt::math::TPose3D& src)
{
	return toROS_Pose(mrpt::poses::CPose3D(src));
}

geometry_msgs::msg::PoseWithCovariance toROS_Pose(
	const mrpt::poses::CPose3DPDFGaussian& src)
{
	geometry_msgs::msg::PoseWithCovariance des;
	des.pose = toROS_Pose(src.mean);

	// Read REP103: http://ros.org/reps/rep-0103.html#covariance-representation
	// # Row-major representation of the 6x6 covariance matrix
	// # The orientation parameters use a fixed-axis representation.
	// # In order, the parameters are:
	// # (x, y, z, rotation about X axis, rotation about Y axis, rotation about
	// Z axis)
	// float64[36] covariance
	// Old comment: "MRPT uses non-fixed axis for 6x6 covariance: should use a
	// transform Jacobian here!"
	//           JL ==> Nope! non-fixed z-y-x equals fixed x-y-z rotations.

	// X,Y,Z,YAW,PITCH,ROLL
	const unsigned int indxs_map[6] = {0, 1, 2, 5, 4, 3};
	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 6; j++)
			des.covariance[indxs_map[i] * 6 + indxs_map[j]] = src.cov(i, j);
	return des;
}

geometry_msgs::msg::PoseWithCovariance toROS(
	const mrpt::poses::CPose3DPDFGaussianInf& src)
{
	mrpt::poses::CPose3DPDFGaussian src2;
	src2.copyFrom(src);
	return toROS_Pose(src2);
}

geometry_msgs::msg::PoseWithCovariance toROS(
	const mrpt::poses::CPosePDFGaussian& src)
{
	geometry_msgs::msg::PoseWithCovariance des;

	des.pose = toROS_Pose(src.mean);

	// Read REP103: http://ros.org/reps/rep-0103.html#covariance-representation
	// Old comment: "MRPT uses non-fixed axis for 6x6 covariance: should use a
	// transform Jacobian here!"
	//           JL ==> Nope! non-fixed z-y-x equals fixed x-y-z rotations.

	// geometry_msgs/PoseWithCovariance msg stores the covariance matrix in
	// row-major representation
	// Indexes are :
	// [ 0   1   2   3   4   5  ]
	// [ 6   7   8   9   10  11 ]
	// [ 12  13  14  15  16  17 ]
	// [ 18  19  20  21  22  23 ]
	// [ 24  25  26  27  28  29 ]
	// [ 30  31  32  33  34  35 ]

	des.covariance[0] = src.cov(0, 0);
	des.covariance[1] = src.cov(0, 1);
	des.covariance[5] = src.cov(0, 2);
	des.covariance[6] = src.cov(1, 0);
	des.covariance[7] = src.cov(1, 1);
	des.covariance[11] = src.cov(1, 2);
	des.covariance[30] = src.cov(2, 0);
	des.covariance[31] = src.cov(2, 1);
	des.covariance[35] = src.cov(2, 2);

	return des;
}

geometry_msgs::msg::PoseWithCovariance toROS(
	const mrpt::poses::CPosePDFGaussianInf& src)
{
	mrpt::poses::CPosePDFGaussian src2;
	src2.copyFrom(src);

	return toROS(src2);
}

geometry_msgs::msg::Quaternion toROS(const mrpt::math::CQuaternionDouble& src)
{
	geometry_msgs::msg::Quaternion des;
	des.x = src.x();
	des.y = src.y();
	des.z = src.z();
	des.w = src.r();
	return des;
}

}  // namespace mrpt::ros2bridge

// ROS -> MRPT functions:
namespace mrpt::ros2bridge
{
mrpt::poses::CPose3D fromROS(const tf2::Transform& src)
{
	mrpt::poses::CPose3D des;
	const tf2::Vector3& t = src.getOrigin();
	des.x(t[0]);
	des.y(t[1]);
	des.z(t[2]);
	des.setRotationMatrix(fromROS(src.getBasis()));
	return des;
}
mrpt::math::CMatrixDouble33 fromROS(const tf2::Matrix3x3& src)
{
	mrpt::math::CMatrixDouble33 des;
	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++)
			des(r, c) = src[r][c];
	return des;
}

mrpt::poses::CPose3D fromROS(const geometry_msgs::msg::Pose& src)
{
	const mrpt::math::CQuaternionDouble q(
		src.orientation.w, src.orientation.x, src.orientation.y,
		src.orientation.z);
	return mrpt::poses::CPose3D(
		q, src.position.x, src.position.y, src.position.z);
}

mrpt::poses::CPose3DPDFGaussian fromROS(
	const geometry_msgs::msg::PoseWithCovariance& src)
{
	mrpt::poses::CPose3DPDFGaussian des;

	des.mean = fromROS(src.pose);

	const unsigned int indxs_map[6] = {0, 1, 2, 5, 4, 3};

	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 6; j++)
			des.cov(i, j) = src.covariance[indxs_map[i] * 6 + indxs_map[j]];

	return des;
}

mrpt::math::CQuaternionDouble fromROS(const geometry_msgs::msg::Quaternion& src)
{
	mrpt::math::CQuaternionDouble des;
	des.x(src.x);
	des.y(src.y);
	des.z(src.z);
	des.r(src.w);
	return des;
}

}  // namespace mrpt::ros2bridge
