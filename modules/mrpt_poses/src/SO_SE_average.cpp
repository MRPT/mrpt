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

#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/poses/SO_SE_average.h>

#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::poses;

// -----------   SO_average<2> --------------------
SO_average<2>::SO_average() = default;

void SO_average<2>::clear()
{
  m_count = .0;
  m_accum_x = m_accum_y = .0;
}
void SO_average<2>::append(const double orientation_rad) { append(orientation_rad, 1.0); }
void SO_average<2>::append(const double orientation_rad, const double weight)
{
  m_count += weight;
  m_accum_x += cos(orientation_rad) * weight;
  m_accum_y += sin(orientation_rad) * weight;
}
double SO_average<2>::get_average() const
{
  ASSERT_GT_(m_count, 0);
  const double x = m_accum_x / m_count;
  const double y = m_accum_y / m_count;
  // The average is undetermined if the mean 2D point lies at the origin:
  if (std::hypot(x, y) < 1e-10)
  {
    if (enable_exception_on_undeterminate)
    {
      throw std::runtime_error("[SO_average<2>::get_average()] Undetermined average value");
    }
    return 0;
  }
  return atan2(y, x);
}

// -----------   SO_average<3> --------------------
SO_average<3>::SO_average() : m_accum_rot() { clear(); }
void SO_average<3>::clear()
{
  m_count = .0;
  m_accum_rot.setZero();
}
void SO_average<3>::append(const mrpt::math::CMatrixDouble33& M) { append(M, 1.0); }
void SO_average<3>::append(const mrpt::math::CMatrixDouble33& M, const double weight)
{
  m_count += weight;
  m_accum_rot.asEigen() += weight * M.asEigen();
}
// Closest rotation (in the Frobenius norm) to the accumulated matrix, see:
// eq. (3.7) in "MEANS AND AVERAGING IN THE GROUP OF ROTATIONS", MAHER
// MOAKHER, 2002. Computed from the SVD M = U*S*V^T as
// R = U*diag(1,1,det(U*V^T))*V^T, which is also valid for rank-2 matrices.
mrpt::math::CMatrixDouble33 SO_average<3>::get_average() const
{
  ASSERT_GT_(m_count, 0);
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      m_accum_rot.asEigen(), Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Vector3d sv = svd.singularValues();

  // The solution is unique only if the two largest singular values are > 0:
  if (!(sv[1] > 1e-10 * sv[0]))
  {
    if (enable_exception_on_undeterminate)
    {
      throw std::runtime_error("[SO_average<3>::get_average()] Undetermined average value");
    }
    return mrpt::math::CMatrixDouble33::Identity();
  }

  Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
  if ((svd.matrixU() * svd.matrixV().transpose()).determinant() < 0)
  {
    D(2, 2) = -1.0;
  }
  return mrpt::math::CMatrixDouble33(svd.matrixU() * D * svd.matrixV().transpose());
}

// -----------   SE_average<2> --------------------
SE_average<2>::SE_average() : m_rot_part() { clear(); }
void SE_average<2>::clear()
{
  m_count = .0;
  m_accum_x = m_accum_y = .0;
  m_rot_part.clear();
}
void SE_average<2>::append(const mrpt::poses::CPose2D& p) { append(p, 1.0); }
void SE_average<2>::append(const mrpt::poses::CPose2D& p, const double weight)
{
  m_count += weight;
  m_accum_x += weight * p.x();
  m_accum_y += weight * p.y();
  m_rot_part.append(p.phi(), weight);
}
void SE_average<2>::append(const mrpt::math::TPose2D& p, const double weight)
{
  m_count += weight;
  m_accum_x += weight * p.x;
  m_accum_y += weight * p.y;
  m_rot_part.append(p.phi, weight);
}
void SE_average<2>::get_average(mrpt::poses::CPose2D& ret_mean) const
{
  ASSERT_GT_(m_count, 0);
  ret_mean.x(m_accum_x / m_count);
  ret_mean.y(m_accum_y / m_count);
  const_cast<SO_average<2>*>(&m_rot_part)->enable_exception_on_undeterminate =
      this->enable_exception_on_undeterminate;
  ret_mean.phi(m_rot_part.get_average());
}

// -----------   SE_average<3> --------------------
SE_average<3>::SE_average() : m_rot_part() { clear(); }
void SE_average<3>::clear()
{
  m_count = .0;
  m_accum_x = m_accum_y = m_accum_z = .0;
  m_rot_part.clear();
}
void SE_average<3>::append(const mrpt::poses::CPose3D& p) { append(p, 1.0); }
void SE_average<3>::append(const mrpt::poses::CPose3D& p, const double weight)
{
  m_count += weight;
  m_accum_x += weight * p.x();
  m_accum_y += weight * p.y();
  m_accum_z += weight * p.z();
  m_rot_part.append(p.getRotationMatrix(), weight);
}
void SE_average<3>::append(const mrpt::math::TPose3D& p, const double weight)
{
  append(CPose3D(p), weight);
}
void SE_average<3>::get_average(mrpt::poses::CPose3D& ret_mean) const
{
  ASSERT_GT_(m_count, 0);
  ret_mean.x(m_accum_x / m_count);
  ret_mean.y(m_accum_y / m_count);
  ret_mean.z(m_accum_z / m_count);
  const_cast<SO_average<3>*>(&m_rot_part)->enable_exception_on_undeterminate =
      this->enable_exception_on_undeterminate;
  ret_mean.setRotationMatrix(m_rot_part.get_average());
}
