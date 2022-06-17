/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/NavState.h>	// Velocity3
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TTwist3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

namespace mrpt::gtsam_wrappers
{
/** @name mrpt_gtsam_wrappers GTSAM <-> MRPT conversion functions
 *  \note New in MRPT 2.3.3
 *  \sa <mrpt/poses/gtsam_wrappers.h>
 * @{ */

static gtsam::Point3 toPoint3(const mrpt::math::TPoint3D& p)
{
	return gtsam::Point3(p.x, p.y, p.z);
}

static gtsam::Pose3 toPose3(const mrpt::poses::CPose3D& in)
{
	mrpt::math::CMatrixDouble44 in_homography;
	in.getHomogeneousMatrix(in_homography);
	return gtsam::Pose3(in_homography.asEigen());
}

static mrpt::math::TPoint3D toTPoint3(const gtsam::Point3& p)
{
	return {p.x(), p.y(), p.z()};
}

static mrpt::math::TTwist3D toTTwist3D(const gtsam::Velocity3& v)
{
	mrpt::math::TTwist3D t;
	t.vx = v.x();
	t.vy = v.y();
	t.vz = v.z();
	return t;
}
static std::array<double, 3> toVelArray(const gtsam::Velocity3& v)
{
	return {v.x(), v.y(), v.z()};
}

static mrpt::math::CMatrixDouble66 to_mrpt_se3_cov6(
	const gtsam::Matrix6& se3_cov)
{
	mrpt::math::CMatrixDouble66 C;
	// MRPT : X ,Y,Z,YAW,PITCH,ROLL
	// GTSAM: RX RY RZ X Y Z
	constexpr unsigned int mapping[6] = {5, 4, 3, 0, 1, 2};
	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 6; j++)
			C(mapping[i], mapping[j]) = se3_cov(i, j);

	return C;
}

static gtsam::Matrix6 to_gtsam_se3_cov6_isotropic(
	const mrpt::math::CMatrixDouble66& se3_cov)
{
	gtsam::Matrix6 C;
	// MRPT : X ,Y,Z,YAW,PITCH,ROLL
	// GTSAM: RX RY RZ X Y Z
	constexpr unsigned int mapping[6] = {5, 4, 3, 0, 1, 2};
	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 6; j++)
			C(i, j) = se3_cov(mapping[i], mapping[j]);

	return C;
}

static void jacobian_rodrigues_from_quat(
	mrpt::math::CMatrixFixed<double, 3, 4>& out_domega_dq,
	const mrpt::math::CQuaternion<double>& quaternion)
{
	assert(quaternion.asEigen().isUnitary());

	double qr = quaternion.r();
	double qx = quaternion.x();
	double qy = quaternion.y();
	double qz = quaternion.z();

	mrpt::math::CVectorFixed<double, 3> q_imaginary;
	q_imaginary(0) = qx;
	q_imaginary(1) = qy;
	q_imaginary(2) = qz;
	double q_imaginary_norm = q_imaginary.norm();
	double q_imaginary_norm_cubic = std::pow(q_imaginary_norm, 3);

	assert(1 - qr * qr >= 0.0);
	assert(q_imaginary_norm > 0.0);

	out_domega_dq(0, 0) = -qx;
	out_domega_dq(1, 0) = qy;
	out_domega_dq(2, 0) = -qz;
	double denom_inv = 2.0 / q_imaginary_norm / std::sqrt(1 - qr * qr);
	out_domega_dq.col(0).array() *= denom_inv;

	out_domega_dq(0, 1) = qy * qy + qz * qz;
	out_domega_dq(0, 2) = -qx * qy;
	out_domega_dq(0, 3) = -qx * qz;
	out_domega_dq(1, 2) = qx * qx + qz * qz;
	out_domega_dq(1, 3) = -qy * qz;
	out_domega_dq(2, 3) = qy * qy + qy * qy;
	denom_inv = 2.0 * acos(qr) / q_imaginary_norm_cubic;
	out_domega_dq.block(0, 1, 3, 3).array() *= denom_inv;
	out_domega_dq(1, 1) = out_domega_dq(0, 2);
	out_domega_dq(2, 1) = out_domega_dq(0, 3);
	out_domega_dq(2, 2) = out_domega_dq(1, 3);
}

static void jacobian_rodrigues_from_YPR(
	mrpt::math::CMatrixDouble33& out_domega_dypr,
	const mrpt::poses::CPose3D& pose)
{
	mrpt::math::CMatrixDouble34 out_domega_dq;
	mrpt::math::CMatrixDouble43 out_dq_dypr;
	mrpt::math::CQuaternion<double> q;
	pose.getAsQuaternion(q, out_dq_dypr);
	if (q.r() >= 1.0 - 1e-9)
	{
		out_domega_dypr = mrpt::math::CMatrixDouble33::Zero();
		out_domega_dypr(0, 2) = 1.0;
		out_domega_dypr(1, 1) = 1.0;
		out_domega_dypr(2, 0) = 1.0;
	}
	else
	{
		jacobian_rodrigues_from_quat(out_domega_dq, q);
		out_domega_dypr = out_domega_dq.asEigen() * out_dq_dypr.asEigen();
	}
}

static void jacobian_pose_rodrigues_from_YPR(
	mrpt::math::CMatrixDouble66& out_pose_domega_dypr,
	const mrpt::poses::CPose3D& pose)
{
	mrpt::math::CMatrixDouble33 out_rotation_domega_dypr;
	jacobian_rodrigues_from_YPR(out_rotation_domega_dypr, pose);
	out_pose_domega_dypr = mrpt::math::CMatrixDouble66::Zero();
	out_pose_domega_dypr.asEigen().block(0, 0, 3, 3) =
		Eigen::Matrix3d::Identity();
	out_pose_domega_dypr.asEigen().block(3, 3, 3, 3) =
		out_rotation_domega_dypr.asEigen();
}

static void to_gtsam_se3_cov6(
	const mrpt::poses::CPose3DPDFGaussian& in, gtsam::Pose3& pose_out,
	gtsam::Matrix6& cov_out)
{
	pose_out = toPose3(in.mean);
	mrpt::math::CMatrixDouble66 out_pose_domega_dypr;
	jacobian_pose_rodrigues_from_YPR(out_pose_domega_dypr, in.mean);

	mrpt::math::CMatrixDouble66 cov_rod(
		out_pose_domega_dypr.asEigen() * in.cov.asEigen() *
		out_pose_domega_dypr.asEigen().transpose());

	cov_out = to_gtsam_se3_cov6_isotropic(cov_rod);	 // reordering
}

/** @} */

}  // namespace mrpt::gtsam_wrappers
