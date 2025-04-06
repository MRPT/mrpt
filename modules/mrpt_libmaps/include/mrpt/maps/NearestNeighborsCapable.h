/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>

namespace mrpt::maps
{
/** Virtual interface for maps having the capability of searching the closest
 * neighbor(s) of a given query 2D or 3D point.
 *
 * Note this is more generic than mrpt::math::KDTreeCapable since it does not
 * assume the use of KD-trees, and it is also non templatized, so users can use
 * dynamic casting to interact with maps in a generic way.
 *
 * \note New in MRPT 2.11.3
 * \ingroup mrpt_maps_grp
 */
class NearestNeighborsCapable
{
 public:
  NearestNeighborsCapable() = default;
  virtual ~NearestNeighborsCapable() = default;

  /** @name API of the NearestNeighborsCapable virtual interface
    @{ */

  /** Returns true if the rest of `nn_*` methods will populate the output
   * indices values with 0-based contiguous **indices**.
   * Returns false if indices are actually sparse **ID numbers** without any
   * expectation of they be contiguous or start near zero.
   */
  [[nodiscard]] virtual bool nn_has_indices_or_ids() const = 0;

  /** Must be called before calls to `nn_*_search()` to ensure the required
   *  data structures are ready for queries (e.g. KD-trees). Useful in
   *  multithreading applications.
   */
  virtual void nn_prepare_for_2d_queries() const
  {
    // Default: do nothing
  }

  /** Must be called before calls to `nn_*_search()` to ensure the required
   *  data structures are ready for queries (e.g. KD-trees). Useful in
   *  multithreading applications.
   */
  virtual void nn_prepare_for_3d_queries() const
  {
    // Default: do nothing
  }

  /** If nn_has_indices_or_ids() returns `true`, this must return the number
   * of "points" (or whatever entity) the indices correspond to. Otherwise,
   * the return value should be ignored.
   */
  [[nodiscard]] virtual size_t nn_index_count() const = 0;

  /** Search for the closest 3D point to a given one.
   *
   * \param[in]  query The query input point.
   * \param[out] result The found closest point.
   * \param[out] out_dist_sqr The square Euclidean distance between the query
   * and the returned point.
   * \param[out] resultIndexOrID The index or ID of the result point in the
   * map.
   *
   * \return True if successful, false if no point was found.
   */
  [[nodiscard]] virtual bool nn_single_search(
      const mrpt::math::TPoint3Df& query,
      mrpt::math::TPoint3Df& result,
      float& out_dist_sqr,
      uint64_t& resultIndexOrIDOrID) const = 0;

  /// \overload for 2D points
  [[nodiscard]] virtual bool nn_single_search(
      const mrpt::math::TPoint2Df& query,
      mrpt::math::TPoint2Df& result,
      float& out_dist_sqr,
      uint64_t& resultIndexOrIDOrID) const = 0;

  /** Search for the `N` closest 3D points to a given one.
   *
   * \param[in]  query The query input point.
   * \param[out] results The found closest points.
   * \param[out] out_dists_sqr The square Euclidean distances between the
   * query and the returned point.
   * \param[out] resultIndicesOrIDs The indices or IDs of the result points.
   *
   */
  virtual void nn_multiple_search(
      const mrpt::math::TPoint3Df& query,
      const size_t N,
      std::vector<mrpt::math::TPoint3Df>& results,
      std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs) const = 0;

  /// \overload for 2D points
  virtual void nn_multiple_search(
      const mrpt::math::TPoint2Df& query,
      const size_t N,
      std::vector<mrpt::math::TPoint2Df>& results,
      std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs) const = 0;

  /** Radius search for closest 3D points to a given one.
   *
   * \param[in]  query The query input point.
   * \param[in]  search_radius_sqr The search radius, **squared**.
   * \param[out] results The found closest points.
   * \param[out] out_dists_sqr The square Euclidean distances between the
   * query and the returned point.
   * \param[out] resultIndicesOrIDs The indices or IDs of the result points.
   * \param[in] maxPoints If !=0, the maximum number of neigbors to return.
   */
  virtual void nn_radius_search(
      const mrpt::math::TPoint3Df& query,
      const float search_radius_sqr,
      std::vector<mrpt::math::TPoint3Df>& results,
      std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs,
      size_t maxPoints = 0) const = 0;

  /// \overload for 2D points
  virtual void nn_radius_search(
      const mrpt::math::TPoint2Df& query,
      const float search_radius_sqr,
      std::vector<mrpt::math::TPoint2Df>& results,
      std::vector<float>& out_dists_sqr,
      std::vector<uint64_t>& resultIndicesOrIDs,
      size_t maxPoints = 0) const = 0;

  /** @} */
};

}  // namespace mrpt::maps
