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
#include <mrpt/math/TPolygonWithPlane.h>

using namespace mrpt::math;

TEST(TPolygonWithPlane, DefaultCtor)
{
  TPolygonWithPlane p;
  EXPECT_TRUE(p.poly.empty());
}

TEST(TPolygonWithPlane, FromPlanarPolygonXY)
{
  TPolygon3D poly3d;
  poly3d.push_back(TPoint3D(0, 0, 1));
  poly3d.push_back(TPoint3D(1, 0, 1));
  poly3d.push_back(TPoint3D(1, 1, 1));
  poly3d.push_back(TPoint3D(0, 1, 1));

  TPolygonWithPlane pwp(poly3d);

  // the plane must be z=1 (or z=-1 depending on normal orientation)
  EXPECT_NEAR(std::abs(pwp.plane.coefs[2]), 1.0, 1e-6);

  // the 2D projection must have the same number of vertices
  EXPECT_EQ(pwp.poly2D.size(), poly3d.size());
}

TEST(TPolygonWithPlane, GetPlanesVector)
{
  TPolygon3D poly1;
  poly1.push_back(TPoint3D(0, 0, 0));
  poly1.push_back(TPoint3D(1, 0, 0));
  poly1.push_back(TPoint3D(1, 1, 0));

  TPolygon3D poly2;
  poly2.push_back(TPoint3D(0, 0, 5));
  poly2.push_back(TPoint3D(1, 0, 5));
  poly2.push_back(TPoint3D(0, 1, 5));

  std::vector<TPolygon3D> polys{poly1, poly2};
  std::vector<TPolygonWithPlane> out;
  TPolygonWithPlane::getPlanes(polys, out);

  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0].poly.size(), poly1.size());
  EXPECT_EQ(out[1].poly.size(), poly2.size());
}
