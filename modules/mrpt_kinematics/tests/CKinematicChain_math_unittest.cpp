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

#include <gtest/gtest.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/kinematics/CKinematicChain.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

using namespace mrpt::kinematics;

namespace
{
using Mat4 = std::array<std::array<double, 4>, 4>;

Mat4 identity4()
{
  Mat4 m{};
  for (int i = 0; i < 4; i++)
  {
    m[i][i] = 1.0;
  }
  return m;
}

Mat4 mul4(const Mat4& a, const Mat4& b)
{
  Mat4 m{};
  for (int r = 0; r < 4; r++)
  {
    for (int c = 0; c < 4; c++)
    {
      for (int k = 0; k < 4; k++)
      {
        m[r][c] += a[r][k] * b[k][c];
      }
    }
  }
  return m;
}

/** Independent standard (distal) Denavit-Hartenberg link transform:
 *  Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha) */
Mat4 dhStandard(double theta, double d, double a, double alpha)
{
  Mat4 rz = identity4();
  rz[0][0] = std::cos(theta);
  rz[0][1] = -std::sin(theta);
  rz[1][0] = std::sin(theta);
  rz[1][1] = std::cos(theta);

  Mat4 tz = identity4();
  tz[2][3] = d;

  Mat4 tx = identity4();
  tx[0][3] = a;

  Mat4 rx = identity4();
  rx[1][1] = std::cos(alpha);
  rx[1][2] = -std::sin(alpha);
  rx[2][1] = std::sin(alpha);
  rx[2][2] = std::cos(alpha);

  return mul4(mul4(mul4(rz, tz), tx), rx);
}

Mat4 toMat4(const mrpt::poses::CPose3D& p)
{
  const auto h = p.getHomogeneousMatrixVal<mrpt::math::CMatrixDouble44>();
  Mat4 m{};
  for (int r = 0; r < 4; r++)
  {
    for (int c = 0; c < 4; c++)
    {
      m[r][c] = h(r, c);
    }
  }
  return m;
}

double maxAbsDiff(const mrpt::poses::CPose3D& p, const Mat4& m)
{
  const Mat4 h = toMat4(p);
  double d = 0;
  for (int r = 0; r < 4; r++)
  {
    for (int c = 0; c < 4; c++)
    {
      d = std::max(d, std::abs(h[r][c] - m[r][c]));
    }
  }
  return d;
}
}  // namespace

TEST(CKinematicChain, forward_kinematics_matches_standard_dh_random)
{
  auto& rng = mrpt::random::getRandomGenerator();
  rng.randomize(1234);

  for (int trial = 0; trial < 100; trial++)
  {
    CKinematicChain c;
    const mrpt::poses::CPose3D origin(
        rng.drawUniform(-2.0, 2.0), rng.drawUniform(-2.0, 2.0), rng.drawUniform(-2.0, 2.0),
        rng.drawUniform(-M_PI, M_PI), rng.drawUniform(-1.4, 1.4), rng.drawUniform(-M_PI, M_PI));
    c.setOriginPose(origin);

    const size_t nLinks = 1 + static_cast<size_t>(trial % 6);
    std::vector<double> q(nLinks);
    for (size_t i = 0; i < nLinks; i++)
    {
      const bool isPrismatic = (trial + i) % 2 == 0;
      c.addLink(
          rng.drawUniform(-M_PI, M_PI), rng.drawUniform(-1.0, 1.0), rng.drawUniform(-1.0, 1.0),
          rng.drawUniform(-M_PI, M_PI), isPrismatic);
      q[i] = rng.drawUniform(-2.0, 2.0);
    }
    c.setConfiguration(q);

    std::vector<mrpt::poses::CPose3D> poses;
    c.recomputeAllPoses(poses);
    ASSERT_EQ(poses.size(), nLinks + 1);

    Mat4 expected = toMat4(origin);
    EXPECT_LT(maxAbsDiff(poses[0], expected), 1e-12);
    for (size_t i = 0; i < nLinks; i++)
    {
      const auto& l = c.getLink(i);
      // The joint variable must land in `d` (prismatic) or `theta` (revolute):
      if (l.is_prismatic)
      {
        EXPECT_DOUBLE_EQ(l.d, q[i]);
      }
      else
      {
        EXPECT_DOUBLE_EQ(l.theta, q[i]);
      }
      expected = mul4(expected, dhStandard(l.theta, l.d, l.a, l.alpha));
      EXPECT_LT(maxAbsDiff(poses[i + 1], expected), 1e-10) << "trial=" << trial << " i=" << i;
    }
  }
}

TEST(CKinematicChain, twist_alpha_rotates_next_z_axis_about_x)
{
  // alpha=+90deg maps Z_1 onto -Y_0, so a prismatic `d` on the next link
  // moves the tip along -Y_0:
  CKinematicChain c;
  c.addLink(.0, .0, .0, M_PI * 0.5, false);
  c.addLink(.0, 0.8, .0, .0, true);

  std::vector<mrpt::poses::CPose3D> poses;
  c.recomputeAllPoses(poses);
  ASSERT_EQ(poses.size(), 3U);
  EXPECT_NEAR(poses[2].x(), .0, 1e-12);
  EXPECT_NEAR(poses[2].y(), -0.8, 1e-12);
  EXPECT_NEAR(poses[2].z(), .0, 1e-12);

  // The `a` offset is along the rotated X_1 axis (after theta), not X_0:
  CKinematicChain c2;
  c2.addLink(M_PI * 0.5, 0.3, 0.5, .0, false);
  c2.recomputeAllPoses(poses);
  EXPECT_NEAR(poses[1].x(), .0, 1e-12);
  EXPECT_NEAR(poses[1].y(), 0.5, 1e-12);
  EXPECT_NEAR(poses[1].z(), 0.3, 1e-12);
}

TEST(CKinematicChain, rotated_origin_composes_on_the_left)
{
  CKinematicChain c;
  c.addLink(0.3, 0.2, 0.7, 0.4, false);
  c.addLink(-0.5, 0.1, 0.4, -0.9, true);

  std::vector<mrpt::poses::CPose3D> local;
  c.recomputeAllPoses(local);

  const mrpt::poses::CPose3D origin(1.0, -2.0, 0.5, 0.7, -0.3, 1.1);
  c.setOriginPose(origin);
  std::vector<mrpt::poses::CPose3D> global;
  c.recomputeAllPoses(global);

  ASSERT_EQ(local.size(), global.size());
  for (size_t i = 0; i < local.size(); i++)
  {
    const mrpt::poses::CPose3D expected = origin + local[i];
    EXPECT_LT(maxAbsDiff(global[i], toMat4(expected)), 1e-12) << "i=" << i;
  }
}

TEST(CKinematicChain, serialization_roundtrip_preserves_forward_kinematics)
{
  CKinematicChain c;
  c.addLink(0.3, 0.2, 0.7, 0.4, false);
  c.addLink(-0.5, 0.1, 0.4, -0.9, true);
  c.addLink(1.2, -0.3, 0.2, 2.1, false);
  c.setOriginPose(mrpt::poses::CPose3D(1.0, -2.0, 0.5, 0.7, -0.3, 1.1));

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << c;
  buf.Seek(0);
  CKinematicChain c2;
  arch >> c2;

  std::vector<mrpt::poses::CPose3D> p1;
  std::vector<mrpt::poses::CPose3D> p2;
  c.recomputeAllPoses(p1);
  c2.recomputeAllPoses(p2);
  ASSERT_EQ(p1.size(), p2.size());
  for (size_t i = 0; i < p1.size(); i++)
  {
    EXPECT_LT(maxAbsDiff(p2[i], toMat4(p1[i])), 1e-12) << "i=" << i;
  }
}
