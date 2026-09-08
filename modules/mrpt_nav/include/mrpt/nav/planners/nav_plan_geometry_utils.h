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

#pragma once

#include <mrpt/math/TPoint2D.h>

#include <optional>

namespace mrpt::nav
{
/** @addtogroup  nav_geom_grp Motion planning geometry utility functions
 * (`#include <mrpt/nav/nav_plan_geometry_utils.h>`)
 *  \ingroup mrpt_nav_grp
 * @{ */

/** Computes the collision-free distance for a linear segment path between two
 * points, for a circular robot, and a point obstacle.
 * \return The distance along the segment at which the robot first touches the
 * obstacle, or std::nullopt if the segment is collision-free.
 * \exception std::runtime_error If the two points are closer than an epsilon
 * (1e-10)
 */
[[nodiscard]] std::optional<double> collision_free_dist_segment_circ_robot(
    const mrpt::math::TPoint2D& p_start,
    const mrpt::math::TPoint2D& p_end,
    const double robot_radius,
    const mrpt::math::TPoint2D& obstacle);

/** Computes the collision-free distance for a forward path (+X) circular arc
 * path segment from pose (0,0,0) and radius of curvature `arc_radius`
 * (>0 -> turn towards +Y, <0 -> towards -Y), a circular robot and a point
 * obstacle.
 * \return The arc length at which the robot first touches the obstacle (0 if
 * it is already in collision at the starting pose), or std::nullopt if the arc
 * is collision-free (which includes the degenerate case of the robot enclosing
 * the obstacle at every pose along the arc).
 */
[[nodiscard]] std::optional<double> collision_free_dist_arc_circ_robot(
    const double arc_radius, const double robot_radius, const mrpt::math::TPoint2D& obstacle);

/** \deprecated Use the std::optional-returning overload. */
[[deprecated("Use the std::optional-returning overload")]] inline bool
collision_free_dist_segment_circ_robot(
    const mrpt::math::TPoint2D& p_start,
    const mrpt::math::TPoint2D& p_end,
    const double robot_radius,
    const mrpt::math::TPoint2D& obstacle,
    double& out_col_dist)
{
  const auto d = collision_free_dist_segment_circ_robot(p_start, p_end, robot_radius, obstacle);
  out_col_dist = d.value_or(-1.0);
  return d.has_value();
}

/** \deprecated Use the std::optional-returning overload. */
[[deprecated("Use the std::optional-returning overload")]] inline bool
collision_free_dist_arc_circ_robot(
    const double arc_radius,
    const double robot_radius,
    const mrpt::math::TPoint2D& obstacle,
    double& out_col_dist)
{
  const auto d = collision_free_dist_arc_circ_robot(arc_radius, robot_radius, obstacle);
  out_col_dist = d.value_or(-1.0);
  return d.has_value();
}

/** @} */
}  // namespace mrpt::nav
