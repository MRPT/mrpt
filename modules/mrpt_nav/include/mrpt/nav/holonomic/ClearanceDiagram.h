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

#include <mrpt/serialization/serialization_frwds.h>
#include <mrpt/viz/viz_frwds.h>

#include <cstdint>
#include <map>
#include <vector>

namespace mrpt::nav
{
/** How ClearanceDiagram::getClearance() condenses the samples stored along one
 *  path into a single number.
 *  \ingroup nav_tpspace
 */
enum class ClearanceQuery : uint8_t
{
  /** Clearance of the sampled robot pose that covers the query distance. */
  AtDistance = 0,
  /** Mean clearance of all sampled poses from the path origin up to the
   * query distance. */
  MeanUpToDistance
};

/** Clearance information for one particular PTG and one set of obstacles.
 *
 * For each of a decimated subset of the PTG paths, it holds the normalized
 * clearance (distance from the robot shape to the closest obstacle, divided by
 * the PTG reference distance) at a handful of poses sampled along that path.
 * Both the map keys (TP-Space distances) and the values (clearances) are
 * normalized to [0,1] w.r.t. the PTG reference distance.
 *
 * Usage:
 * - Declare an object of this type (it will be initialized to "empty"),
 * - Call CParameterizedTrajectoryGenerator::initClearanceDiagram()
 * - Repeatedly call CParameterizedTrajectoryGenerator::updateClearance() for
 * each 2D obstacle point.
 *
 *  \ingroup nav_tpspace
 */
class ClearanceDiagram
{
 public:
  /** default ctor */
  ClearanceDiagram();
  /** Reset to default, empty state */
  void clear();
  /** Initializes the container to allocate `decimated_num_paths` entries, as
   * a decimated
   * subset of a total of `actual_num_paths` paths */
  void resize(size_t actual_num_paths, size_t decimated_num_paths);
  bool empty() const { return m_raw_clearances.empty(); }
  size_t get_actual_num_paths() const { return m_actual_num_paths; }
  size_t get_decimated_num_paths() const { return m_raw_clearances.size(); }

  /** Gets the normalized clearance for path `k` at the normalized TP-Space
   * distance `TPS_query_distance`, condensed as per `mode`.
   *
   * Only the samples up to and including the first one at or past the query
   * distance take part in the result. Returns 0 for an empty diagram.
   */
  [[nodiscard]] double getClearance(
      uint16_t k, double TPS_query_distance, ClearanceQuery mode) const;

  /** \deprecated Use the ClearanceQuery overload. Note that the `bool` flag
   * used to select the opposite mode to the one it is named after. */
  [[deprecated(
      "Use the ClearanceQuery overload: the bool flag selected the mode opposite to its "
      "own documentation")]] [[nodiscard]] double
  getClearance(uint16_t k, double TPS_query_distance, bool integrate_over_path) const
  {
    return getClearance(
        k, TPS_query_distance,
        integrate_over_path ? ClearanceQuery::MeanUpToDistance : ClearanceQuery::AtDistance);
  }

  void renderAs3DObject(
      mrpt::viz::CMesh& mesh,
      double min_x,
      double max_x,
      double min_y,
      double max_y,
      double cell_res,
      ClearanceQuery mode) const;

  void readFromStream(mrpt::serialization::CArchive& in);
  void writeToStream(mrpt::serialization::CArchive& out) const;

  /** [normalized TPS distance in [0,1]] =>
   * normalized_clearance_for_exactly_that_robot_pose  */
  using dist2clearance_t = std::map<double, double>;
  dist2clearance_t& get_path_clearance(size_t actual_k);
  const dist2clearance_t& get_path_clearance(size_t actual_k) const;

  dist2clearance_t& get_path_clearance_decimated(size_t decim_k)
  {
    return m_raw_clearances[decim_k];
  }
  const dist2clearance_t& get_path_clearance_decimated(size_t decim_k) const
  {
    return m_raw_clearances[decim_k];
  }

  size_t real_k_to_decimated_k(size_t k) const;
  size_t decimated_k_to_real_k(size_t k) const;

 protected:
  /** Container: [decimated_path_k][TPS_distance] =>
   * normalized_clearance_for_exactly_that_robot_pose  */
  std::vector<dist2clearance_t> m_raw_clearances;

  size_t m_actual_num_paths{0};  // The decimated number of paths is implicit in
  // raw_clearances.size()
  double m_k_a2d{.0}, m_k_d2a{.0};
};

}  // namespace mrpt::nav
