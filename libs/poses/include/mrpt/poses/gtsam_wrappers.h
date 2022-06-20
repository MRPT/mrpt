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
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/gtsam_wrappers.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

namespace mrpt::gtsam_wrappers
{
/** @name mrpt_gtsam_wrappers GTSAM <-> MRPT conversion functions
 *  \note New in MRPT 2.3.3
 *  \sa <mrpt/math/gtsam_wrappers.h>
 * @{ */

static gtsam::Pose3 toPose3(const mrpt::poses::CPose3D& p)
{
	return gtsam::Pose3(
		gtsam::Rot3(p.getRotationMatrix().asEigen()),
		gtsam::Point3(p.x(), p.y(), p.z()));
}

static mrpt::math::TPose3D toTPose3D(const gtsam::Rot3& p)
{
	const auto R = p.matrix();
	return mrpt::poses::CPose3D(
			   mrpt::math::CMatrixDouble33(R),
			   mrpt::math::CVectorFixedDouble<3>())
		.asTPose();
}

static gtsam::Pose3 toPose3(const mrpt::math::TPose3D& p)
{
	return gtsam::Pose3(
		gtsam::Rot3(p.getRotationMatrix().asEigen()),
		gtsam::Point3(p.x, p.y, p.z));
}

static mrpt::math::TPose3D toTPose3D(const gtsam::Pose3& p)
{
	const auto HM = p.matrix();
	const auto H = mrpt::math::CMatrixDouble44(HM);
	return mrpt::poses::CPose3D(H).asTPose();
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
