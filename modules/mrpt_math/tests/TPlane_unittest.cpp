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
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TPlane.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::math;

TEST(TPlane, EvaluatePointAndContains)
{
  const auto pl = TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
  EXPECT_NEAR(pl.evaluatePoint({0, 0, 5}), 5.0, 1e-9);
  EXPECT_TRUE(pl.contains(TPoint3D(1, 1, 0)));
  EXPECT_FALSE(pl.contains(TPoint3D(1, 1, 1)));
}

TEST(TPlane, ContainsLine)
{
  const auto pl = TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
  const auto lineInPlane = TLine3D::FromTwoPoints({0, 0, 0}, {1, 1, 0});
  const auto lineNotInPlane = TLine3D::FromTwoPoints({0, 0, 0}, {1, 1, 1});
  const auto lineParallelOffset = TLine3D::FromTwoPoints({0, 0, 5}, {1, 1, 5});

  EXPECT_TRUE(pl.contains(lineInPlane));
  EXPECT_FALSE(pl.contains(lineNotInPlane));
  EXPECT_FALSE(pl.contains(lineParallelOffset));
}

TEST(TPlane, DistanceAndSignedDistance)
{
  const auto pl = TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
  EXPECT_NEAR(pl.distance(TPoint3D(1, 1, -3)), 3.0, 1e-9);
  EXPECT_NEAR(pl.signedDistance(TPoint3D(1, 1, -3)), -3.0, 1e-9);
  EXPECT_NEAR(pl.signedDistance(TPoint3D(1, 1, 3)), 3.0, 1e-9);
}

TEST(TPlane, DistanceToLine)
{
  const auto pl = TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
  // Line crossing the plane -> distance is 0
  const auto crossing = TLine3D::FromTwoPoints({0, 0, -1}, {0, 0, 1});
  EXPECT_NEAR(pl.distance(crossing), 0.0, 1e-9);

  // Line parallel to the plane, offset by 4 -> distance is 4
  const auto parallel = TLine3D::FromTwoPoints({0, 0, 4}, {1, 1, 4});
  EXPECT_NEAR(pl.distance(parallel), 4.0, 1e-9);
}

TEST(TPlane, NormalVectors)
{
  // FromPointAndNormal() already stores a unitary normal internally.
  const auto pl = TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 5});
  const auto n = pl.getNormalVector();
  EXPECT_NEAR(n.z, 1.0, 1e-9);

  const auto un = pl.getUnitaryNormalVector();
  EXPECT_NEAR(un.norm(), 1.0, 1e-9);
}

TEST(TPlane, UnitaryNormalVectorThrowsOnDegeneratePlane)
{
  TPlane degenerate;
  degenerate.coefs[0] = degenerate.coefs[1] = degenerate.coefs[2] = degenerate.coefs[3] = 0.0;
  EXPECT_THROW(degenerate.getUnitaryNormalVector(), std::exception);
}

TEST(TPlane, Unitarize)
{
  auto pl = TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 5});
  pl.unitarize();
  EXPECT_NEAR(
      std::sqrt(pl.coefs[0] * pl.coefs[0] + pl.coefs[1] * pl.coefs[1] + pl.coefs[2] * pl.coefs[2]),
      1.0, 1e-9);
}

TEST(TPlane, GetAsPose3D)
{
  const auto pl = TPlane::FromPointAndNormal({0, 0, 3}, {0, 0, 1});
  TPose3D pose;
  pl.getAsPose3D(pose);
  // XY plane of the pose should coincide with the plane Z=3
  EXPECT_NEAR(pose.z, 3.0, 1e-6);
}

TEST(TPlane, GetAsPose3DForcingOrigin)
{
  const auto pl = TPlane::FromPointAndNormal({0, 0, 3}, {0, 0, 1});
  TPose3D pose;
  pl.getAsPose3DForcingOrigin(TPoint3D(1, 2, 3), pose);
  EXPECT_NEAR(pose.x, 1.0, 1e-6);
  EXPECT_NEAR(pose.y, 2.0, 1e-6);
  EXPECT_NEAR(pose.z, 3.0, 1e-6);

  // Overload returning by value:
  const auto pose2 = pl.getAsPose3DForcingOrigin(TPoint3D(1, 2, 3));
  EXPECT_NEAR(pose2.x, 1.0, 1e-6);
}

TEST(TPlane, GetAsPose3DForcingOriginThrowsIfPointNotInPlane)
{
  const auto pl = TPlane::FromPointAndNormal({0, 0, 3}, {0, 0, 1});
  TPose3D pose;
  EXPECT_THROW(pl.getAsPose3DForcingOrigin(TPoint3D(1, 2, 100), pose), std::exception);
}

TEST(TPlane, CtorFrom3PointsThrowsOnCollinearPoints)
{
  EXPECT_THROW(TPlane(TPoint3D(0, 0, 0), TPoint3D(1, 1, 1), TPoint3D(2, 2, 2)), std::exception);
}

TEST(TPlane, CtorFromPointAndLine)
{
  const auto line = TLine3D::FromTwoPoints({0, 0, 0}, {1, 0, 0});
  const TPlane pl(TPoint3D(0, 1, 0), line);
  EXPECT_TRUE(pl.contains(TPoint3D(0, 0, 0)));
  EXPECT_TRUE(pl.contains(TPoint3D(1, 0, 0)));
  EXPECT_TRUE(pl.contains(TPoint3D(0, 1, 0)));
}

TEST(TPlane, CtorFromPointAndLineThrowsIfPointOnLine)
{
  const auto line = TLine3D::FromTwoPoints({0, 0, 0}, {1, 0, 0});
  EXPECT_THROW(TPlane(TPoint3D(5, 0, 0), line), std::exception);
}

TEST(TPlane, CtorFromTwoLinesIntersecting)
{
  const auto l1 = TLine3D::FromTwoPoints({-1, 0, 0}, {1, 0, 0});
  const auto l2 = TLine3D::FromTwoPoints({0, -1, 0}, {0, 1, 0});
  const TPlane pl(l1, l2);
  EXPECT_TRUE(pl.contains(TPoint3D(0, 0, 0)));
}

TEST(TPlane, CtorFromTwoLinesParallelDistinct)
{
  const auto l1 = TLine3D::FromTwoPoints({0, 0, 0}, {1, 0, 0});
  const auto l2 = TLine3D::FromTwoPoints({0, 1, 0}, {1, 1, 0});
  const TPlane pl(l1, l2);
  EXPECT_TRUE(pl.contains(TPoint3D(0, 0, 0)));
  EXPECT_TRUE(pl.contains(TPoint3D(0, 1, 0)));
}

TEST(TPlane, CtorFromTwoLinesSameLineThrows)
{
  const auto l1 = TLine3D::FromTwoPoints({0, 0, 0}, {1, 0, 0});
  const auto l2 = TLine3D::FromTwoPoints({5, 0, 0}, {10, 0, 0});
  EXPECT_THROW(TPlane(l1, l2), std::exception);
}

TEST(TPlane, CtorFromTwoLinesNonIntersectingSkewThrows)
{
  const auto l1 = TLine3D::FromTwoPoints({0, 0, 0}, {1, 0, 0});
  const auto l2 = TLine3D::FromTwoPoints({0, 1, 0}, {0, 1, 1});
  EXPECT_THROW(TPlane(l1, l2), std::exception);
}

TEST(TPlane, SerializationRoundTrip)
{
  const auto pl = TPlane::FromPointAndNormal({1, 2, 3}, {0, 0, 1});

  mrpt::io::CMemoryStream membuf;
  auto arch = mrpt::serialization::archiveFrom(membuf);
  arch << pl;
  membuf.Seek(0);

  TPlane pl2;
  arch >> pl2;

  for (int i = 0; i < 4; i++)
  {
    EXPECT_NEAR(pl.coefs[i], pl2.coefs[i], 1e-9);
  }
}

TEST(TPlane, AsStringAndStreamOperator)
{
  const auto pl = TPlane::FromPointAndNormal({0, 0, 0}, {0, 0, 1});
  EXPECT_FALSE(pl.asString().empty());

  std::ostringstream ss;
  ss << pl;
  EXPECT_EQ(ss.str(), pl.asString());
}
