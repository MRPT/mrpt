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

#include <mrpt/core/exceptions.h>
#include <mrpt/math/poly_roots.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/nav/planners/nav_plan_geometry_utils.h>

#include <limits>

using namespace mrpt;
using namespace mrpt::math;

std::optional<double> mrpt::nav::collision_free_dist_segment_circ_robot(
    const mrpt::math::TPoint2D& p0,
    const mrpt::math::TPoint2D& p1,
    const double R,
    const mrpt::math::TPoint2D& o)
{
  using mrpt::square;

  // Unit vector from start -> end:
  mrpt::math::TPoint2D u = (p1 - p0);
  const double L = u.norm();
  ASSERT_GT_(L, 1e-10);
  u *= 1.0 / L;

  /*
  syms x y d ux uy o.x o.y R real
  f=(x+d*ux-o.x)^2+(y+d*uy-o.y)^2-R^2
  coeffs ->
  [ (o.x - x)^2 + (o.y - y)^2 - R^2, - 2*ux*(o.x - x) - 2*uy*(o.y - y), ux^2 +
  uy^2]
  */

  // quadratic eq: a*d^2 + b*d+c=0
  const double a = square(u.x) + square(u.y);
  const double b = -2 * u.x * (o.x - p0.x) - 2 * u.y * (o.y - p0.y);
  const double c = square(o.x - p0.x) + square(o.y - p0.y) - square(R);

  double r1, r2;
  const int nsols = mrpt::math::solve_poly2(a, b, c, r1, r2);

  if (nsols <= 0) return std::nullopt;

  double r_min;
  if (nsols == 1)
  {
    r_min = r1;
  }
  else if (r1 < 0 && r2 < 0)
  {
    return std::nullopt;
  }
  else if (r1 < 0)
  {
    r_min = r2;
  }
  else if (r2 < 0)
  {
    r_min = r1;
  }
  else
  {
    r_min = std::min(r1, r2);
  }

  if (r_min > L) return std::nullopt;

  // A real, valid collision:
  return r_min;
}

std::optional<double> mrpt::nav::collision_free_dist_arc_circ_robot(
    const double arc_radius, const double R, const mrpt::math::TPoint2D& o)
{
  ASSERT_GT_(std::abs(arc_radius), 1e-10);

  // Already in collision at the starting pose: zero collision-free distance.
  if (o.norm() <= R) return .0;

  // The robot center travels along the circle of radius |arc_radius| centered
  // at (0, arc_radius). It touches the obstacle wherever that circle meets the
  // one of radius R around the obstacle, so this is a two-circle intersection:
  const mrpt::math::TPoint2D arcCenter(.0, arc_radius);
  const double arcR = std::abs(arc_radius);

  const auto v = o - arcCenter;
  const double d = v.norm();

  // Separate circles, or one strictly inside the other (the latter means the
  // robot never *stops* enclosing the obstacle, so there is no first touch):
  if (d > arcR + R || d < std::abs(arcR - R) || d < 1e-10) return std::nullopt;

  const double a = (d * d + arcR * arcR - R * R) / (2 * d);
  const double h2 = arcR * arcR - a * a;
  if (h2 < 0) return std::nullopt;
  const double h = std::sqrt(h2);

  const mrpt::math::TPoint2D pm = arcCenter + v * (a / d);
  const mrpt::math::TPoint2D perp(-v.y / d, v.x / d);

  // Of the (up to) two touch points, keep the one reached first, measuring the
  // turned angle from the starting pose (0,0,0) in the direction of motion:
  double minAngle = std::numeric_limits<double>::max();
  for (const double side : {+1.0, -1.0})
  {
    const auto p = pm + perp * (side * h);
    // (x,y) order is intentionally like this: it makes th=0 at the origin.
    double th = std::atan2(p.x - arcCenter.x, -(p.y - arcCenter.y));
    if (arc_radius < 0) th = M_PI - th;
    mrpt::keep_min(minAngle, mrpt::math::wrapTo2Pi(th));
  }

  return arcR * minAngle;
}
