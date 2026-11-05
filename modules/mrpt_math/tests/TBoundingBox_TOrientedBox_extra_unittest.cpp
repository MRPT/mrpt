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
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TOrientedBox.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/random/RandomGenerators.h>

#include <cmath>
#include <vector>

using namespace mrpt::math;

namespace
{
mrpt::random::CRandomGenerator& rng() { return mrpt::random::getRandomGenerator(); }

TPose3D randomPose()
{
  return {rng().drawUniform(-5, 5), rng().drawUniform(-5, 5),     rng().drawUniform(-5, 5),
          rng().drawUniform(-3, 3), rng().drawUniform(-1.5, 1.5), rng().drawUniform(-3, 3)};
}
}  // namespace

TEST(TBoundingBoxExtra, UnionOfEmptyBoxes)
{
  // The empty (+inf,-inf) box is the neutral element of the union.
  const auto e = TBoundingBox::PlusMinusInfinity();
  TBoundingBox u;
  ASSERT_NO_THROW(u = e.unionWith(e));
  // Union with an empty box is the identity:
  const TBoundingBox b({0, 1, 2}, {3, 4, 5});
  EXPECT_EQ(b.unionWith(e), b);
  EXPECT_EQ(e.unionWith(b), b);
}

TEST(TBoundingBoxExtra, IntersectionWithinEpsilonGapIsOrdered)
{
  // Boxes separated by a gap smaller than "epsilon": the result is flat.
  const TBoundingBox a({0, 0, 0}, {1, 1, 1});
  const TBoundingBox b({1.00005, 0, 0}, {2, 1, 1});
  const auto i = a.intersection(b);
  ASSERT_TRUE(i.has_value());
  EXPECT_LE(i->min.x, i->max.x) << i->asString();
  EXPECT_GE(i->volume(), 0.0);
}

TEST(TBoundingBoxExtra, IntersectionAndContainsRandom)
{
  rng().randomize(1234);
  for (int k = 0; k < 500; k++)
  {
    const auto a = TBoundingBox::FromUnsortedPoints(
        {rng().drawUniform(-5, 5), rng().drawUniform(-5, 5), rng().drawUniform(-5, 5)},
        {rng().drawUniform(-5, 5), rng().drawUniform(-5, 5), rng().drawUniform(-5, 5)});
    const auto b = TBoundingBox::FromUnsortedPoints(
        {rng().drawUniform(-5, 5), rng().drawUniform(-5, 5), rng().drawUniform(-5, 5)},
        {rng().drawUniform(-5, 5), rng().drawUniform(-5, 5), rng().drawUniform(-5, 5)});
    const auto i = a.intersection(b, 0.0);
    const auto u = a.unionWith(b);
    for (int s = 0; s < 20; s++)
    {
      const TPoint3D p(
          rng().drawUniform(-6, 6), rng().drawUniform(-6, 6), rng().drawUniform(-6, 6));
      const bool inA = a.containsPoint(p);
      const bool inB = b.containsPoint(p);
      EXPECT_EQ(i.has_value() && i->containsPoint(p), inA && inB);
      if (inA || inB)
      {
        EXPECT_TRUE(u.containsPoint(p));
      }
    }
  }
}

TEST(TOrientedBoxExtra, PlanesAndVerticesRandomPose)
{
  rng().randomize(1234);
  for (int k = 0; k < 100; k++)
  {
    const TPose3D pose = randomPose();
    const TPoint3D size(
        rng().drawUniform(0.5, 4), rng().drawUniform(0.5, 4), rng().drawUniform(0.5, 4));
    const TOrientedBox box(pose, size);
    const auto planes = box.getBoxPlanes();
    const auto& verts = box.vertices();
    ASSERT_EQ(verts.size(), 8u);
    const TPoint3D center(pose.x, pose.y, pose.z);

    // Outward normals: the center is on the negative side, at half the size.
    const double halfSizes[6] = {size.x / 2, size.y / 2, size.y / 2,
                                 size.z / 2, size.x / 2, size.z / 2};
    for (size_t i = 0; i < planes.size(); i++)
    {
      EXPECT_NEAR(planes[i].signedDistance(center), -halfSizes[i], 1e-9) << "plane " << i;
      int nOnPlane = 0;
      for (const auto& v : verts)
      {
        const double d = planes[i].signedDistance(v);
        EXPECT_LT(d, 1e-9);
        if (std::abs(d) < 1e-9)
        {
          nOnPlane++;
        }
      }
      EXPECT_EQ(nOnPlane, 4) << "plane " << i;
    }
    // Vertex numbering: bit 0 -> +X', bit 1 -> +Y', bit 2 -> +Z'.
    for (size_t i = 0; i < 8; i++)
    {
      const TPoint3D local(
          (i & 1) ? size.x / 2 : -size.x / 2, (i & 2) ? size.y / 2 : -size.y / 2,
          (i & 4) ? size.z / 2 : -size.z / 2);
      EXPECT_NEAR((verts[i] - pose.composePoint(local)).norm(), 0.0, 1e-9) << "vertex " << i;
    }
  }
}

TEST(TOrientedBoxExtra, AxisAlignedBoxOfYawedBox)
{
  rng().randomize(1234);
  for (int k = 0; k < 100; k++)
  {
    const double yaw = rng().drawUniform(-M_PI, M_PI);
    const TPoint3D size(
        rng().drawUniform(0.5, 4), rng().drawUniform(0.5, 4), rng().drawUniform(0.5, 4));
    const TOrientedBox box(TPose3D(1, 2, 3, yaw, 0, 0), size);
    const auto bb = box.getAxisAlignedBox();
    const double c = std::abs(std::cos(yaw));
    const double s = std::abs(std::sin(yaw));
    const double hx = 0.5 * (c * size.x + s * size.y);
    const double hy = 0.5 * (s * size.x + c * size.y);
    EXPECT_NEAR(bb.min.x, 1 - hx, 1e-9);
    EXPECT_NEAR(bb.max.x, 1 + hx, 1e-9);
    EXPECT_NEAR(bb.min.y, 2 - hy, 1e-9);
    EXPECT_NEAR(bb.max.y, 2 + hy, 1e-9);
    EXPECT_NEAR(bb.min.z, 3 - size.z / 2, 1e-9);
    EXPECT_NEAR(bb.max.z, 3 + size.z / 2, 1e-9);
  }
}

TEST(TOrientedBoxExtra, SmallBoxPlanes)
{
  // Millimeter-sized boxes.
  const TOrientedBox box(TPose3D(0, 0, 0, 0, 0, 0), TPoint3D(2e-3, 2e-3, 2e-3));
  ASSERT_NO_THROW((void)box.getBoxPlanes());
}
