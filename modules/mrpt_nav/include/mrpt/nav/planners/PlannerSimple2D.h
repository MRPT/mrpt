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

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/poses/CPose2D.h>

#include <deque>
#include <optional>

namespace mrpt::nav
{
/** \addtogroup nav_planners Path planning
 * \ingroup mrpt_nav_grp
 * @{ */

/** Searches for collision-free path in 2D occupancy grids for holonomic
 * circular robots.
 *  The implementation first enlargest obstacles with robot radius, then applies
 * a
 *  wavefront algorithm to find the shortest free path between origin and target
 * 2D points.
 *
 * Notice that this simple planner does not take into account robot kinematic
 * constraints.
 */
class PlannerSimple2D
{
 public:
  PlannerSimple2D() = default;
  virtual ~PlannerSimple2D() = default;

  /** The maximum occupancy probability to consider a cell as an obstacle,
   * default=0.5  */
  float occupancyThreshold{0.5f};

  /** The minimum distance between points in the returned found path
   * (default=0.4); Notice
   *  that full grid resolution is used in path finding, this is only a way
   * to reduce the
   *  amount of redundant information to be returned.
   */
  float minStepInReturnedPath{0.4f};

  /** The aproximate robot radius used in the planification. Default is 0.35m
   */
  float robotRadius{0.35f};

  /** Computes the optimal path for a circular robot, in the given occupancy
   * grid map, from the origin location to a target point. Additional
   * parameters are the public member variables of this class.
   *
   * \param theMap The occupancy gridmap used for the planning.
   * \param origin The starting pose of the robot, in "map" coordinates.
   * \param target The desired target pose, in "map" coordinates.
   * \param maxSearchPathLength The maximum path length to search for, in
   * meters (-1 = no limit)
   *
   * \return The found path, in global coordinates relative to "map", or
   * std::nullopt if no path exists (which includes either endpoint falling
   * outside the gridmap).
   *
   * \sa robotRadius
   * \exception std::exception On any error
   */
  [[nodiscard]] std::optional<std::deque<mrpt::math::TPoint2D>> computePath(
      const mrpt::maps::COccupancyGridMap2D& theMap,
      const mrpt::poses::CPose2D& origin,
      const mrpt::poses::CPose2D& target,
      float maxSearchPathLength = -1) const;

  /** \deprecated Use the std::optional-returning overload. */
  [[deprecated("Use the std::optional-returning computePath()")]] void computePath(
      const mrpt::maps::COccupancyGridMap2D& theMap,
      const mrpt::poses::CPose2D& origin,
      const mrpt::poses::CPose2D& target,
      std::deque<mrpt::math::TPoint2D>& path,
      bool& notFound,
      float maxSearchPathLength = -1) const
  {
    auto res = computePath(theMap, origin, target, maxSearchPathLength);
    notFound = !res.has_value();
    path = notFound ? std::deque<mrpt::math::TPoint2D>() : std::move(*res);
  }
};

/** @} */
}  // namespace mrpt::nav
