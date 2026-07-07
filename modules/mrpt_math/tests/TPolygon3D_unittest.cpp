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
#include <mrpt/math/TPlane.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TSegment3D.h>

using namespace mrpt::math;

TEST(TPolygon3D, Generate2DObject)
{
  const TPolygon3D poly{
      {0, 0, 5},
      {1, 0, 5},
      {1, 1, 5}
  };
  TPolygon2D poly2;
  poly.generate2DObject(poly2);
  ASSERT_EQ(poly2.size(), 3u);
  EXPECT_NEAR(poly2[1].x, 1.0, 1e-9);
}

TEST(TPolygon3D, DistanceToPointInPlane)
{
  const TPolygon3D poly{
      {0, 0, 0},
      {2, 0, 0},
      {2, 2, 0},
      {0, 2, 0}
  };
  EXPECT_NEAR(poly.distance(TPoint3D(1, 1, 0)), 0.0, 1e-6);
}

TEST(TPolygon3D, DistanceToPointOffPlane)
{
  const TPolygon3D poly{
      {0, 0, 0},
      {2, 0, 0},
      {2, 2, 0},
      {0, 2, 0}
  };
  // Directly above the center, at height 3
  EXPECT_NEAR(poly.distance(TPoint3D(1, 1, 3)), 3.0, 1e-3);
}

TEST(TPolygon3D, ContainsPointInsideAndOutside)
{
  const TPolygon3D poly{
      {0, 0, 0},
      {2, 0, 0},
      {2, 2, 0},
      {0, 2, 0}
  };
  EXPECT_TRUE(poly.contains(TPoint3D(1, 1, 0)));
  EXPECT_FALSE(poly.contains(TPoint3D(1, 1, 1)));  // off-plane
  EXPECT_FALSE(poly.contains(TPoint3D(5, 5, 0)));  // in-plane but outside
}

TEST(TPolygon3D, GetAsSegmentList)
{
  const TPolygon3D poly{
      {0, 0, 0},
      {1, 0, 0},
      {1, 1, 0}
  };
  std::vector<TSegment3D> segs;
  poly.getAsSegmentList(segs);
  ASSERT_EQ(segs.size(), 3u);
  EXPECT_EQ(segs[2].point2, poly[0]);
}

TEST(TPolygon3D, GetPlaneAndBestFittingPlane)
{
  const TPolygon3D poly{
      {0, 0, 3},
      {1, 0, 3},
      {1, 1, 3},
      {0, 1, 3}
  };
  TPlane pl;
  ASSERT_TRUE(poly.getPlane(pl));
  EXPECT_TRUE(pl.contains(TPoint3D(0.5, 0.5, 3)));

  TPlane fittedPl;
  poly.getBestFittingPlane(fittedPl);
  EXPECT_TRUE(fittedPl.contains(TPoint3D(0.5, 0.5, 3)));
}

TEST(TPolygon3D, GetCenter)
{
  const TPolygon3D poly{
      {0, 0, 0},
      {2, 0, 0},
      {2, 2, 0},
      {0, 2, 0}
  };
  TPoint3D c;
  poly.getCenter(c);
  EXPECT_NEAR(c.x, 1.0, 1e-9);
  EXPECT_NEAR(c.y, 1.0, 1e-9);
  EXPECT_NEAR(c.z, 0.0, 1e-9);
}

TEST(TPolygon3D, IsSkew)
{
  const TPolygon3D planar{
      {0, 0, 0},
      {1, 0, 0},
      {1, 1, 0},
      {0, 1, 0}
  };
  const TPolygon3D skew{
      {0, 0, 0},
      {1, 0, 0},
      {1, 1, 1},
      {0, 1, 0}
  };
  EXPECT_FALSE(planar.isSkew());
  EXPECT_TRUE(skew.isSkew());
}

TEST(TPolygon3D, RemoveRepeatedAndRedundantVertices)
{
  TPolygon3D poly{
      {0, 0, 0},
      {0, 0, 0},
      {1, 0, 0},
      {2, 0, 0},
      {2, 2, 0},
      {0, 2, 0}
  };
  poly.removeRepeatedVertices();
  EXPECT_EQ(poly.size(), 5u);

  poly.removeRedundantVertices();
  // The collinear point (1,0,0) between (0,0,0) and (2,0,0) is removed too.
  EXPECT_EQ(poly.size(), 4u);
}

TEST(TPolygon3D, CtorFromTPolygon2D)
{
  const TPolygon2D poly2{
      {0, 0},
      {1, 0},
      {1, 1}
  };
  const TPolygon3D poly3(poly2);
  ASSERT_EQ(poly3.size(), 3u);
  EXPECT_NEAR(poly3[1].x, 1.0, 1e-9);
  EXPECT_NEAR(poly3[1].z, 0.0, 1e-9);
}

TEST(TPolygon3D, CreateRegularPolygon)
{
  TPolygon3D poly;
  TPolygon3D::createRegularPolygon(5, 2.0, poly);
  ASSERT_EQ(poly.size(), 5u);
  for (const auto& v : poly)
  {
    EXPECT_NEAR(std::hypot(v.x, v.y), 2.0, 1e-9);
    EXPECT_NEAR(v.z, 0.0, 1e-9);
  }
}

TEST(TPolygon3D, CreateRegularPolygonWithPose)
{
  TPolygon3D poly;
  const TPose3D pose(0.0, 0.0, 10.0, 0.0, 0.0, 0.0);
  TPolygon3D::createRegularPolygon(4, 1.0, poly, pose);
  ASSERT_EQ(poly.size(), 4u);
  for (const auto& v : poly)
  {
    EXPECT_NEAR(v.z, 10.0, 1e-9);
  }
}

TEST(TPolygon3D, CreateRegularPolygonThrowsOnInvalidArgs)
{
  TPolygon3D poly;
  EXPECT_THROW(TPolygon3D::createRegularPolygon(2, 1.0, poly), std::exception);
  EXPECT_THROW(TPolygon3D::createRegularPolygon(4, 0.0, poly), std::exception);
}

TEST(TPolygon3D, StreamOperator)
{
  const TPolygon3D poly{
      {0, 0, 0},
      {1, 0, 0}
  };
  std::ostringstream ss;
  ss << poly;
  EXPECT_FALSE(ss.str().empty());
}
