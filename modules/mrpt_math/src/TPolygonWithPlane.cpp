/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include "math-precomp.h"  // Precompiled headers
//
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPolygonWithPlane.h>
#include <mrpt/math/geometry.h>

using namespace mrpt::math;
using std::vector;

TPolygonWithPlane::TPolygonWithPlane(const TPolygon3D& p) : poly(p)
{
  poly.getBestFittingPlane(plane);
  plane.getAsPose3D(pose);
  // inversePose = -pose;
  CMatrixDouble44 P_inv;
  pose.getInverseHomogeneousMatrix(P_inv);
  inversePose.fromHomogeneousMatrix(P_inv);

  internal::unsafeProjectPolygon(poly, inversePose, poly2D);
}
void TPolygonWithPlane::getPlanes(
    const vector<TPolygon3D>& oldPolys, vector<TPolygonWithPlane>& newPolys)
{
  size_t N = oldPolys.size();
  newPolys.resize(N);
  for (size_t i = 0; i < N; i++) newPolys[i] = oldPolys[i];
}
