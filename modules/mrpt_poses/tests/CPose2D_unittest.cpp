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

#include <CTraitsTest.h>
#include <gtest/gtest.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>

#include <Eigen/Dense>
#include <cmath>
#include <vector>

template class mrpt::CTraitsTest<mrpt::poses::CPose2D>;

using namespace mrpt::poses;
using namespace mrpt::math;

TEST(CPose2D, ConstructorsFromOtherTypes)
{
  CPoint2D pt2(1, 2);
  CPose2D fromPt2(pt2);
  EXPECT_DOUBLE_EQ(fromPt2.x(), 1.0);
  EXPECT_DOUBLE_EQ(fromPt2.y(), 2.0);
  EXPECT_DOUBLE_EQ(fromPt2.phi(), 0.0);

  CPoint3D pt3(1, 2, 3);
  CPose2D fromPt3(pt3);
  EXPECT_DOUBLE_EQ(fromPt3.x(), 1.0);
  EXPECT_DOUBLE_EQ(fromPt3.y(), 2.0);

  CPose3D pose3(1, 2, 3, 0.3, 0, 0);
  CPose2D fromPose3(pose3);
  EXPECT_DOUBLE_EQ(fromPose3.x(), 1.0);
  EXPECT_DOUBLE_EQ(fromPose3.y(), 2.0);
  EXPECT_NEAR(fromPose3.phi(), 0.3, 1e-9);

  TPose2D tp(1, 2, 0.1);
  CPose2D fromTPose(tp);
  EXPECT_DOUBLE_EQ(fromTPose.x(), 1.0);
  EXPECT_NEAR(fromTPose.phi(), 0.1, 1e-9);
}

TEST(CPose2D, SerializationBothVersions)
{
  CPose2D p(1, 2, 0.3);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << p;
  buf.Seek(0);
  CPose2D p2;
  arch >> p2;
  EXPECT_TRUE(p == p2);
  EXPECT_FALSE(p != p2);
}

TEST(CPose2D, RotationMatricesAndHomogeneous)
{
  CPose2D p(1, 2, 0.3);

  CMatrixDouble22 R2 = p.getRotationMatrix();
  CMatrixDouble22 R2b;
  CMatrixDouble33 R3;
  CMatrixDouble44 H = p.getHomogeneousMatrix();
  CMatrixDouble44 H2;
#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  // Exercise the deprecated output-argument overloads for coverage:
  p.getRotationMatrix(R2b);
  p.getRotationMatrix(R3);
  p.getHomogeneousMatrix(H2);
#if defined(__clang__) || defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
  EXPECT_TRUE(R2 == R2b);
  EXPECT_NEAR(R3(0, 0), R2(0, 0), 1e-9);
  EXPECT_NEAR(R3(2, 2), 1.0, 1e-9);

  EXPECT_TRUE(H == H2);
  EXPECT_NEAR(H(0, 3), 1.0, 1e-9);
  EXPECT_NEAR(H(1, 3), 2.0, 1e-9);
}

TEST(CPose2D, CompositionOperatorsWithPointsAndPoses)
{
  CPose2D p(1, 0, M_PI / 2);

  CPose3D sum3d = p + CPose3D(0, 0, 0, 0, 0, 0);
  EXPECT_NEAR(sum3d.x(), 1.0, 1e-9);

  CPoint2D pt2 = p + CPoint2D(1, 0);
  EXPECT_NEAR(pt2.x(), 1.0, 1e-6);
  EXPECT_NEAR(pt2.y(), 1.0, 1e-6);

  CPoint3D pt3 = p + CPoint3D(1, 0, 5);
  EXPECT_NEAR(pt3.x(), 1.0, 1e-6);
  EXPECT_NEAR(pt3.y(), 1.0, 1e-6);
  EXPECT_NEAR(pt3.z(), 5.0, 1e-9);

  double gx;
  double gy;
  p.composePoint(1, 0, gx, gy);
  EXPECT_NEAR(gx, 1.0, 1e-6);
  EXPECT_NEAR(gy, 1.0, 1e-6);

  TPoint2D g2;
  p.composePoint(TPoint2D(1, 0), g2);
  EXPECT_NEAR(g2.x, 1.0, 1e-6);

  TPoint3D g3;
  p.composePoint(TPoint3D(1, 0, 5), g3);
  EXPECT_NEAR(g3.x, 1.0, 1e-6);
  EXPECT_NEAR(g3.z, 5.0, 1e-9);
  TPoint3D g3b = p.composePoint(TPoint3D(1, 0, 5));
  EXPECT_NEAR(g3b.x, g3.x, 1e-9);

  double lx;
  double ly;
  p.inverseComposePoint(1.0, 1.0, lx, ly);
  EXPECT_NEAR(lx, 1.0, 1e-6);
  EXPECT_NEAR(ly, 0.0, 1e-6);

  TPoint2D l2;
  p.inverseComposePoint(TPoint2D(1, 1), l2);
  EXPECT_NEAR(l2.x, 1.0, 1e-6);
  TPoint2D l2b = p.inverseComposePoint(TPoint2D(1, 1));
  EXPECT_NEAR(l2b.x, l2.x, 1e-9);

  TPoint2D pointRes = p + TPoint2D(1, 0);
  EXPECT_NEAR(pointRes.x, 1.0, 1e-6);

  CPose2D a;
  a.composeFrom(p, CPose2D(1, 0, 0));
  EXPECT_NEAR(a.x(), 1.0, 1e-6);

  CPose2D b;
  b.inverseComposeFrom(p, CPose2D(0, 0, 0));
  EXPECT_NEAR(b.x(), 1.0, 1e-6);

  CPose2D c(1, 0, 0);
  c += CPose2D(1, 0, 0);
  EXPECT_NEAR(c.x(), 2.0, 1e-9);

  CPose3D diff = p - CPose3D(0, 0, 0, 0, 0, 0);
  EXPECT_NEAR(diff.x(), 1.0, 1e-6);
}

TEST(CPose2D, ScalarOpsAddComponentsAndInverse)
{
  CPose2D p(1, 2, 0.1);
  p.AddComponents(CPose2D(1, 1, 0.1));
  EXPECT_NEAR(p.x(), 2.0, 1e-9);
  EXPECT_NEAR(p.y(), 3.0, 1e-9);

  CPose2D q(1, 2, 0.1);
  q *= 2.0;
  EXPECT_NEAR(q.x(), 2.0, 1e-9);
  EXPECT_NEAR(q.y(), 4.0, 1e-9);
  EXPECT_NEAR(q.phi(), 0.2, 1e-9);

  CPose2D r(1, 2, 0.3);
  CPose2D negR = -r;
  CPose2D rr = r;
  rr.inverse();
  EXPECT_NEAR(negR.x(), rr.x(), 1e-9);
  EXPECT_NEAR(negR.y(), rr.y(), 1e-9);

  CPose2D::vector_t v;
  r.asVector(v);
  EXPECT_NEAR(v[0], 1.0, 1e-9);
  EXPECT_NEAR(v[2], 0.3, 1e-9);

  EXPECT_EQ(r.getOppositeScalar(), CPose2D(-1, -2, -0.3));

  EXPECT_NEAR(r.distance2DFrobeniusTo(r), 0.0, 1e-9);

  EXPECT_FALSE(r.asString().empty());
  const TPose2D tp = r.asTPose();
  EXPECT_NEAR(tp.x, 1.0, 1e-9);
  EXPECT_NEAR(tp.y, 2.0, 1e-9);
  EXPECT_NEAR(tp.phi, 0.3, 1e-9);
}

TEST(CPose2D, FromStringVariants)
{
  CPose2D p;
  p.fromString("[1.0 2.0 30.0]");
  EXPECT_DOUBLE_EQ(p.x(), 1.0);
  EXPECT_DOUBLE_EQ(p.y(), 2.0);

  CPose2D p2;
  p2.fromStringRaw("1.0 2.0 30.0");
  EXPECT_DOUBLE_EQ(p2.x(), 1.0);
  EXPECT_NEAR(p2.phi(), p.phi(), 1e-9);
}

namespace
{
std::vector<CPose2D> randomPoses2D(size_t n)
{
  auto& rng = mrpt::random::getRandomGenerator();
  rng.randomize(1234);
  std::vector<CPose2D> ps;
  for (size_t i = 0; i < n; i++)
  {
    ps.emplace_back(rng.drawUniform(-5, 5), rng.drawUniform(-5, 5), rng.drawUniform(-M_PI, M_PI));
  }
  // Angles right at the wrap-around:
  ps.emplace_back(1, 2, M_PI);
  ps.emplace_back(-1, 0.5, -M_PI + 1e-12);
  return ps;
}

void expectSamePose(const CPose2D& a, const CPose2D& b, double tol)
{
  EXPECT_NEAR(a.x(), b.x(), tol);
  EXPECT_NEAR(a.y(), b.y(), tol);
  EXPECT_NEAR(mrpt::math::wrapToPi(a.phi() - b.phi()), 0.0, tol);
}
}  // namespace

TEST(CPose2D, GroupIdentitiesAndMatrixForm)
{
  const auto ps = randomPoses2D(10);
  for (const auto& a : ps)
  {
    expectSamePose(a + (-a), CPose2D(), 1e-12);
    expectSamePose((-a) + a, CPose2D(), 1e-12);
    for (const auto& b : ps)
    {
      expectSamePose(a - b, (-b) + a, 1e-12);
      expectSamePose(b + (a - b), a, 1e-12);
      // Matrix form of the composition:
      const auto Hab = (a + b).getHomogeneousMatrix();
      const Eigen::Matrix4d Href =
          a.getHomogeneousMatrix().asEigen() * b.getHomogeneousMatrix().asEigen();
      EXPECT_LT((Hab.asEigen() - Href).array().abs().maxCoeff(), 1e-12);
      // The resulting angle stays wrapped:
      EXPECT_LE(std::abs((a + b).phi()), M_PI);
      EXPECT_LE(std::abs((a - b).phi()), M_PI);
    }
  }
}

TEST(CPose2D, ComposeAndInverseComposePoint)
{
  for (const auto& a : randomPoses2D(10))
  {
    const TPoint2D l(0.3, -2.0);
    TPoint2D g;
    a.composePoint(l, g);
    const auto l2 = a.inverseComposePoint(g);
    EXPECT_NEAR(l2.x, l.x, 1e-12);
    EXPECT_NEAR(l2.y, l.y, 1e-12);
    // Agreement with CPose3D:
    const auto g3 = CPose3D(a).composePoint(TPoint3D(l.x, l.y, 0.7));
    EXPECT_NEAR(g3.x, g.x, 1e-12);
    EXPECT_NEAR(g3.y, g.y, 1e-12);
    EXPECT_NEAR(g3.z, 0.7, 1e-12);
  }
}

TEST(CPose2D, InverseComposeFromAliasing)
{
  const CPose2D A(1.0, 2.0, 0.4);
  const CPose2D B(-2.0, 0.5, -1.0);
  const CPose2D ref = (-B) + A;
  CPose2D a = A;
  a.inverseComposeFrom(a, B);
  expectSamePose(a, ref, 1e-12);
  CPose2D b = B;
  b.inverseComposeFrom(A, b);
  expectSamePose(b, ref, 1e-12);
}

TEST(CPose2D, BracketWriteInvalidatesTrigCache)
{
  CPose2D p(0, 0, 0);
  EXPECT_NEAR(p.phi_cos(), 1.0, 1e-15);
  p[2] = M_PI / 2;
  const CPoint2D g = p + CPoint2D(1, 0);
  EXPECT_NEAR(g.x(), 0.0, 1e-12);
  EXPECT_NEAR(g.y(), 1.0, 1e-12);
}
