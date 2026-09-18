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
#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TSegment2D.h>
#include <mrpt/math/TSegment3D.h>
#include <mrpt/serialization/CArchive.h>

#include <sstream>

using namespace mrpt::math;

TEST(TLine2D, factories)
{
  // A*x + B*y + C = 0 <=> the horizontal line y=1
  const auto l = TLine2D::FromCoefficientsABC(0, 1, -1);
  EXPECT_NEAR(l.evaluatePoint({5.0, 1.0}), 0.0, 1e-12);
  EXPECT_TRUE(l.contains({5.0, 1.0}));
  EXPECT_FALSE(l.contains({5.0, 2.0}));

  const auto l2 = TLine2D::FromTwoPoints({0.0, 0.0}, {1.0, 1.0});
  EXPECT_TRUE(l2.contains({2.0, 2.0}));

  // Two identical points do not define a line:
  EXPECT_THROW(TLine2D::FromTwoPoints({1.0, 1.0}, {1.0, 1.0}), std::logic_error);
}

TEST(TLine2D, distances)
{
  // The line y=0
  const auto l = TLine2D::FromCoefficientsABC(0, 1, 0);

  EXPECT_NEAR(l.distance({3.0, 2.0}), 2.0, 1e-12);
  EXPECT_NEAR(l.distance({3.0, -2.0}), 2.0, 1e-12);
  EXPECT_NEAR(l.signedDistance({3.0, 2.0}), 2.0, 1e-12);
  EXPECT_NEAR(l.signedDistance({3.0, -2.0}), -2.0, 1e-12);
}

TEST(TLine2D, vectorsAndUnitarize)
{
  auto l = TLine2D::FromCoefficientsABC(0, 2, -4);

  double n[2] = {0, 0};
  l.getNormalVector(n);
  EXPECT_NEAR(n[0], 0.0, 1e-12);
  EXPECT_NEAR(n[1], 2.0, 1e-12);

  double d[2] = {0, 0};
  l.getDirectorVector(d);
  EXPECT_NEAR(d[0], -2.0, 1e-12);
  EXPECT_NEAR(d[1], 0.0, 1e-12);

  l.unitarize();
  l.getNormalVector(n);
  EXPECT_NEAR(std::hypot(n[0], n[1]), 1.0, 1e-12);
  // Still the same line (y=2):
  EXPECT_TRUE(l.contains({7.0, 2.0}));
}

TEST(TLine2D, getAsPose2D)
{
  // Vertical line x=3: coefs[0]!=0, so the "force y=0" branch applies
  {
    const auto l = TLine2D::FromCoefficientsABC(1, 0, -3);
    TPose2D p;
    l.getAsPose2D(p);
    EXPECT_NEAR(p.x, 3.0, 1e-12);
    EXPECT_NEAR(p.y, 0.0, 1e-12);
  }
  // Horizontal line y=2: coefs[0]==0, so the "force x=0" branch applies
  {
    const auto l = TLine2D::FromCoefficientsABC(0, 1, -2);
    TPose2D p;
    l.getAsPose2D(p);
    EXPECT_NEAR(p.x, 0.0, 1e-12);
    EXPECT_NEAR(p.y, 2.0, 1e-12);
  }
  // Forcing the origin, which must lie on the line:
  {
    const auto l = TLine2D::FromCoefficientsABC(0, 1, -2);
    TPose2D p;
    l.getAsPose2DForcingOrigin({5.0, 2.0}, p);
    EXPECT_NEAR(p.x, 5.0, 1e-12);
    EXPECT_NEAR(p.y, 2.0, 1e-12);
    EXPECT_THROW(l.getAsPose2DForcingOrigin({5.0, 3.0}, p), std::logic_error);
  }
}

TEST(TLine2D, conversions3D)
{
  const auto l = TLine2D::FromCoefficientsABC(0, 1, -2);

  TLine3D l3;
  l.generate3DObject(l3);
  EXPECT_NEAR(l3.director[2], 0.0, 1e-12);
  // A horizontal 2D line uses x as the free coordinate for its base point:
  EXPECT_NEAR(l3.pBase.x, 0.0, 1e-12);
  EXPECT_NEAR(l3.pBase.y, 2.0, 1e-12);

  // Back to 2D:
  const TLine2D lBack(l3);
  EXPECT_TRUE(lBack.contains({9.0, 2.0}));

  // A line normal to the XY plane cannot be projected onto it:
  const TLine3D vertical(TPoint3D(0, 0, 0), TPoint3D(0, 0, 1));
  EXPECT_THROW(TLine2D{vertical}, std::logic_error);
}

TEST(TLine2D, serializationAndPrint)
{
  const auto l = TLine2D::FromCoefficientsABC(1, 2, 3);

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << l;
  buf.Seek(0);
  TLine2D l2;
  arch >> l2;
  EXPECT_NEAR(l2.coefs[0], 1.0, 1e-12);
  EXPECT_NEAR(l2.coefs[2], 3.0, 1e-12);

  EXPECT_FALSE(l.asString().empty());
  std::stringstream ss;
  ss << l;
  EXPECT_FALSE(ss.str().empty());
}

TEST(TSegment2D, basics)
{
  const TSegment2D s({0.0, 0.0}, {10.0, 0.0});

  EXPECT_NEAR(s.length(), 10.0, 1e-12);
  EXPECT_TRUE(s.contains({5.0, 0.0}));
  EXPECT_FALSE(s.contains({5.0, 1.0}));
  EXPECT_FALSE(s.contains({15.0, 0.0}));

  EXPECT_NEAR(s.distance({5.0, 3.0}), 3.0, 1e-12);
  EXPECT_NEAR(std::abs(s.signedDistance({5.0, 3.0})), 3.0, 1e-12);
  EXPECT_NEAR(s.signedDistance({5.0, 3.0}), -s.signedDistance({5.0, -3.0}), 1e-12);
}

TEST(TSegment2D, ordering)
{
  const TSegment2D a({0.0, 0.0}, {1.0, 0.0});
  const TSegment2D b({0.0, 0.0}, {2.0, 0.0});
  const TSegment2D c({1.0, 0.0}, {2.0, 0.0});

  EXPECT_TRUE(a < b);  // same first point, second decides
  EXPECT_FALSE(b < a);
  EXPECT_TRUE(a < c);  // first point decides
  EXPECT_FALSE(c < a);
}

TEST(TSegment2D, conversions3D)
{
  const TSegment2D s({0.0, 0.0}, {10.0, 0.0});

  TSegment3D s3;
  s.generate3DObject(s3);
  EXPECT_NEAR(s3.point2.x, 10.0, 1e-12);

  const TSegment2D sBack(s3);
  EXPECT_NEAR(sBack.length(), 10.0, 1e-12);

  // A segment normal to the XY plane projects onto a single point:
  const TSegment3D vertical(TPoint3D(1, 2, 0), TPoint3D(1, 2, 5));
  EXPECT_THROW(TSegment2D{vertical}, std::logic_error);
}

TEST(TSegment2D, serializationAndPrint)
{
  const TSegment2D s({1.0, 2.0}, {3.0, 4.0});

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << s;
  buf.Seek(0);
  TSegment2D s2;
  arch >> s2;
  EXPECT_EQ(s2.point1, s.point1);
  EXPECT_EQ(s2.point2, s.point2);

  EXPECT_FALSE(s.asString().empty());
  std::stringstream ss;
  ss << s;
  EXPECT_FALSE(ss.str().empty());
}
