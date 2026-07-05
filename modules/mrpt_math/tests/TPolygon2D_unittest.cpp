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
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/TPolygon3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TSegment2D.h>

using namespace mrpt::math;

TEST(TPolygon2D, DistancePointInsideIsZero)
{
  const TPolygon2D poly{
      {0, 0},
      {2, 0},
      {2, 2},
      {0, 2}
  };
  EXPECT_NEAR(poly.distance(TPoint2D(1, 1)), 0.0, 1e-9);
}

TEST(TPolygon2D, DistancePointOutside)
{
  const TPolygon2D poly{
      {0, 0},
      {2, 0},
      {2, 2},
      {0, 2}
  };
  EXPECT_NEAR(poly.distance(TPoint2D(3, 1)), 1.0, 1e-9);
}

TEST(TPolygon2D, DistanceEmptyPolygonThrows)
{
  const TPolygon2D poly;
  EXPECT_THROW(poly.distance(TPoint2D(0, 0)), std::exception);
}

TEST(TPolygon2D, GetBoundingBox)
{
  const TPolygon2D poly{
      {-1, -2},
      { 3,  0},
      { 1,  5}
  };
  TPoint2D pmin;
  TPoint2D pmax;
  poly.getBoundingBox(pmin, pmax);
  EXPECT_NEAR(pmin.x, -1.0, 1e-9);
  EXPECT_NEAR(pmin.y, -2.0, 1e-9);
  EXPECT_NEAR(pmax.x, 3.0, 1e-9);
  EXPECT_NEAR(pmax.y, 5.0, 1e-9);
}

TEST(TPolygon2D, GetBoundingBoxEmptyThrows)
{
  const TPolygon2D poly;
  TPoint2D pmin;
  TPoint2D pmax;
  EXPECT_THROW(poly.getBoundingBox(pmin, pmax), std::exception);
}

TEST(TPolygon2D, GetAsSegmentList)
{
  const TPolygon2D poly{
      {0, 0},
      {1, 0},
      {1, 1}
  };
  std::vector<TSegment2D> segs;
  poly.getAsSegmentList(segs);
  ASSERT_EQ(segs.size(), 3u);
  EXPECT_EQ(segs[2].point2, poly[0]);
}

TEST(TPolygon2D, Generate3DObject)
{
  const TPolygon2D poly{
      {0, 0},
      {1, 0},
      {1, 1}
  };
  TPolygon3D poly3;
  poly.generate3DObject(poly3);
  ASSERT_EQ(poly3.size(), 3u);
  EXPECT_NEAR(poly3[1].x, 1.0, 1e-9);
  EXPECT_NEAR(poly3[1].z, 0.0, 1e-9);
}

TEST(TPolygon2D, GetCenter)
{
  const TPolygon2D poly{
      {0, 0},
      {2, 0},
      {2, 2},
      {0, 2}
  };
  TPoint2D c;
  poly.getCenter(c);
  EXPECT_NEAR(c.x, 1.0, 1e-9);
  EXPECT_NEAR(c.y, 1.0, 1e-9);
}

TEST(TPolygon2D, IsConvexTriangleAlwaysTrue)
{
  const TPolygon2D poly{
      {0, 0},
      {1, 0},
      {0, 1}
  };
  EXPECT_TRUE(poly.isConvex());
}

TEST(TPolygon2D, IsConvexSquareTrue)
{
  const TPolygon2D poly{
      {0, 0},
      {2, 0},
      {2, 2},
      {0, 2}
  };
  EXPECT_TRUE(poly.isConvex());
}

TEST(TPolygon2D, IsConvexConcaveFalse)
{
  // L-shape
  const TPolygon2D poly{
      {0, 0},
      {2, 0},
      {2, 1},
      {1, 1},
      {1, 2},
      {0, 2}
  };
  EXPECT_FALSE(poly.isConvex());
}

TEST(TPolygon2D, RemoveRepeatedVertices)
{
  TPolygon2D poly{
      {0, 0},
      {0, 0},
      {1, 0},
      {1, 1}
  };
  poly.removeRepeatedVertices();
  EXPECT_EQ(poly.size(), 3u);
}

TEST(TPolygon2D, RemoveRedundantVertices)
{
  // A collinear point on one edge should be removed.
  TPolygon2D poly{
      {0, 0},
      {1, 0},
      {2, 0},
      {2, 2},
      {0, 2}
  };
  poly.removeRedundantVertices();
  EXPECT_EQ(poly.size(), 4u);
}

TEST(TPolygon2D, GetPlotData)
{
  const TPolygon2D poly{
      {0, 0},
      {1, 0},
      {1, 1}
  };
  std::vector<double> x;
  std::vector<double> y;
  poly.getPlotData(x, y);
  ASSERT_EQ(x.size(), 4u);  // N+1, closing the loop
  EXPECT_NEAR(x[3], x[0], 1e-9);
  EXPECT_NEAR(y[3], y[0], 1e-9);
}

TEST(TPolygon2D, CtorFromTPolygon3D)
{
  const TPolygon3D poly3{
      {0, 0, 5},
      {1, 0, 5},
      {1, 1, 5}
  };
  const TPolygon2D poly2(poly3);
  ASSERT_EQ(poly2.size(), 3u);
  EXPECT_NEAR(poly2[1].x, 1.0, 1e-9);
}

TEST(TPolygon2D, CreateRegularPolygon)
{
  TPolygon2D poly;
  TPolygon2D::createRegularPolygon(4, 1.0, poly);
  ASSERT_EQ(poly.size(), 4u);
  for (const auto& v : poly)
  {
    EXPECT_NEAR(std::hypot(v.x, v.y), 1.0, 1e-9);
  }
}

TEST(TPolygon2D, CreateRegularPolygonWithPose)
{
  TPolygon2D poly;
  const TPose2D pose(10.0, 20.0, 0.0);
  TPolygon2D::createRegularPolygon(4, 1.0, poly, pose);
  ASSERT_EQ(poly.size(), 4u);
  TPoint2D c;
  poly.getCenter(c);
  EXPECT_NEAR(c.x, 10.0, 1e-6);
  EXPECT_NEAR(c.y, 20.0, 1e-6);
}

TEST(TPolygon2D, CreateRegularPolygonThrowsOnInvalidArgs)
{
  TPolygon2D poly;
  EXPECT_THROW(TPolygon2D::createRegularPolygon(2, 1.0, poly), std::exception);
  EXPECT_THROW(TPolygon2D::createRegularPolygon(4, 0.0, poly), std::exception);
}

TEST(TPolygon2D, StreamOperator)
{
  const TPolygon2D poly{
      {0, 0},
      {1, 0}
  };
  std::ostringstream ss;
  ss << poly;
  EXPECT_FALSE(ss.str().empty());
}
