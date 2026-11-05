/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <mrpt/core/exceptions.h>
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

inline gtsam::Pose3 toPose3(const mrpt::poses::CPose3D& p)
{
  return gtsam::Pose3(
      gtsam::Rot3(p.getRotationMatrix().asEigen()), gtsam::Point3(p.x(), p.y(), p.z()));
}

inline mrpt::math::TPose3D toTPose3D(const gtsam::Rot3& p)
{
  const auto R = p.matrix();
  return mrpt::poses::CPose3D(mrpt::math::CMatrixDouble33(R), mrpt::math::CVectorFixedDouble<3>())
      .asTPose();
}

inline gtsam::Pose3 toPose3(const mrpt::math::TPose3D& p)
{
  return gtsam::Pose3(gtsam::Rot3(p.getRotationMatrix().asEigen()), gtsam::Point3(p.x, p.y, p.z));
}

inline mrpt::math::TPose3D toTPose3D(const gtsam::Pose3& p)
{
  const auto HM = p.matrix();
  const auto H = mrpt::math::CMatrixDouble44(HM);
  return mrpt::poses::CPose3D(H).asTPose();
}

/** Jacobian of the GTSAM local coordinates (rx,ry,rz,tx,ty,tz) of the
 * body-frame increment xi = Logmap(mean^-1 * pose) with respect to MRPT's
 * (x,y,z,yaw,pitch,roll), evaluated at the mean. The rotation part is the
 * body-frame angular velocity of R = Rz(yaw)*Ry(pitch)*Rx(roll), the
 * translation part is R^T times the global translation increment.
 */
inline gtsam::Matrix6 jacobian_gtsam_local_wrt_ypr(const mrpt::poses::CPose3D& mean)
{
  const double cp = std::cos(mean.pitch());
  const double sp = std::sin(mean.pitch());
  const double cr = std::cos(mean.roll());
  const double sr = std::sin(mean.roll());
  gtsam::Matrix6 J = gtsam::Matrix6::Zero();
  // clang-format off
  J.block<3, 3>(0, 3) <<
      -sp,      0,   1,
      cp * sr,  cr,  0,
      cp * cr, -sr,  0;
  // clang-format on
  J.block<3, 3>(3, 0) = mean.getRotationMatrix().asEigen().transpose();
  return J;
}

/** Converts an MRPT Gaussian SE(3) pose, with a (x,y,z,yaw,pitch,roll)
 * covariance, into a GTSAM Pose3 and its covariance in GTSAM local coordinates
 * (rx,ry,rz,tx,ty,tz), i.e. for the body-frame increment
 * xi = Logmap(mean^-1 * pose). This is a first-order (linearized) conversion.
 * \sa to_mrpt_se3_pdf()
 */
inline void to_gtsam_se3_cov6(
    const mrpt::poses::CPose3DPDFGaussian& in, gtsam::Pose3& pose_out, gtsam::Matrix6& cov_out)
{
  pose_out = toPose3(in.mean);
  const gtsam::Matrix6 J = jacobian_gtsam_local_wrt_ypr(in.mean);
  cov_out = J * in.cov.asEigen() * J.transpose();
}

/** Inverse of to_gtsam_se3_cov6(): converts a GTSAM Pose3 and its covariance
 * in GTSAM local coordinates (rx,ry,rz,tx,ty,tz) into an MRPT Gaussian SE(3)
 * pose with a (x,y,z,yaw,pitch,roll) covariance. First-order conversion.
 * \exception std::exception At pitch = +-90 deg, where (yaw,pitch,roll) is
 * singular.
 * Unlike mrpt::gtsam_wrappers::to_mrpt_se3_cov6(), which only reorders the
 * entries and is therefore exact only at the identity rotation, this one
 * takes the pose into account.
 * \note New in MRPT 3.7.0
 */
inline mrpt::poses::CPose3DPDFGaussian to_mrpt_se3_pdf(
    const gtsam::Pose3& pose, const gtsam::Matrix6& cov)
{
  mrpt::poses::CPose3DPDFGaussian out;
  out.mean = mrpt::poses::CPose3D(toTPose3D(pose));
  ASSERTMSG_(
      std::abs(std::cos(out.mean.pitch())) > 1e-12,
      "to_mrpt_se3_pdf(): (yaw,pitch,roll) is singular at pitch = +-90 deg");
  const gtsam::Matrix6 Ji = jacobian_gtsam_local_wrt_ypr(out.mean).inverse();
  out.cov.asEigen() = Ji * cov * Ji.transpose();
  return out;
}

/** @} */

}  // namespace mrpt::gtsam_wrappers
