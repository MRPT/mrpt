/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/poses/poses_frwds.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

#include <cstring>	// size_t
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace mrpt::ros2bridge
{
/** \addtogroup mrpt_ros2bridge_grp
 * @{ */

tf2::Matrix3x3 toROS(const mrpt::math::CMatrixDouble33& src);

tf2::Transform toROS_tfTransform(const mrpt::poses::CPose2D& src);
geometry_msgs::msg::Pose toROS_Pose(const mrpt::poses::CPose2D& src);

tf2::Transform toROS_tfTransform(const mrpt::math::TPose2D& src);
geometry_msgs::msg::Pose toROS_Pose(const mrpt::math::TPose2D& src);

tf2::Transform toROS_tfTransform(const mrpt::poses::CPose3D& src);
geometry_msgs::msg::Pose toROS_Pose(const mrpt::poses::CPose3D& src);

tf2::Transform toROS_tfTransform(const mrpt::math::TPose3D& src);
geometry_msgs::msg::Pose toROS_Pose(const mrpt::math::TPose3D& src);

geometry_msgs::msg::PoseWithCovariance toROS_Pose(
	const mrpt::poses::CPose3DPDFGaussian& src);

geometry_msgs::msg::PoseWithCovariance toROS(
	const mrpt::poses::CPose3DPDFGaussianInf& src);

geometry_msgs::msg::PoseWithCovariance toROS(
	const mrpt::poses::CPosePDFGaussian& src);

geometry_msgs::msg::PoseWithCovariance toROS(
	const mrpt::poses::CPosePDFGaussianInf& src);

geometry_msgs::msg::Quaternion toROS(const mrpt::math::CQuaternionDouble& src);

mrpt::poses::CPose3D fromROS(const tf2::Transform& src);
mrpt::math::CMatrixDouble33 fromROS(const tf2::Matrix3x3& src);
mrpt::poses::CPose3D fromROS(const geometry_msgs::msg::Pose& src);
mrpt::poses::CPose3DPDFGaussian fromROS(
	const geometry_msgs::msg::PoseWithCovariance& src);
mrpt::math::CQuaternionDouble fromROS(
	const geometry_msgs::msg::Quaternion& src);

/** @} */
}  // namespace mrpt::ros2bridge
