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

#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/TTriangle.h>

#include <cmath>

using namespace mrpt::viz;

// packet size= 3 vertices, each:
// XYZ (float) + normal (XYZ float) + UV (float) + tangent (XYZ float) + RGBA (u8)
static_assert(sizeof(TTriangle) == (sizeof(float) * (3 + 3 + 2 + 3) + 4) * 3);

void TTriangle::computeNormals()
{
  const float ax = x(1) - x(0);
  const float ay = y(1) - y(0);
  const float az = z(1) - z(0);
  const float bx = x(2) - x(0);
  const float by = y(2) - y(0);
  const float bz = z(2) - z(0);

  const mrpt::math::TVector3Df no = {ay * bz - az * by, -ax * bz + az * bx, ax * by - ay * bx};
  for (auto& v : vertices) v.normal = no;
}

void TTriangle::computeTangents()
{
  // Edges of the triangle
  const float e1x = x(1) - x(0), e1y = y(1) - y(0), e1z = z(1) - z(0);
  const float e2x = x(2) - x(0), e2y = y(2) - y(0), e2z = z(2) - z(0);

  // Delta UVs
  const float du1 = u(1) - u(0), dv1 = v(1) - v(0);
  const float du2 = u(2) - u(0), dv2 = v(2) - v(0);

  const float det = du1 * dv2 - du2 * dv1;
  if (std::abs(det) < 1e-12f)
  {
    // Degenerate UV mapping: use a default tangent
    for (auto& vtx : vertices) vtx.tangent = {1.0f, 0.0f, 0.0f};
    return;
  }

  const float r = 1.0f / det;
  const mrpt::math::TVector3Df tangent = {
      r * (dv2 * e1x - dv1 * e2x), r * (dv2 * e1y - dv1 * e2y), r * (dv2 * e1z - dv1 * e2z)};

  for (auto& vtx : vertices) vtx.tangent = tangent;
}

void TTriangle::writeTo(mrpt::serialization::CArchive& o) const
{
  for (const auto& p : vertices)
  {
    const auto& pp = p.xyzrgba;
    o << pp.pt << pp.r << pp.g << pp.b << pp.a << p.normal;
  }
}
void TTriangle::readFrom(mrpt::serialization::CArchive& in)
{
  for (auto& p : vertices)
  {
    auto& pp = p.xyzrgba;
    in >> pp.pt >> pp.r >> pp.g >> pp.b >> pp.a >> p.normal;
  }
}
