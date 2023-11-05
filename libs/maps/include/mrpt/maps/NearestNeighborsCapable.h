/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
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

	/** Search for the closest 3D point to a given one.
	 *
	 * \param[in]  query The query input point.
	 * \param[out] result The found closest point.
	 * \param[out] out_dist_sqr The square Euclidean distance between the query
	 * and the returned point.
	 *
	 * \return True if successful, false if no point was found.
	 */
	[[nodiscard]] virtual bool nn_single_search(
		const mrpt::math::TPoint3Df& query, mrpt::math::TPoint3Df& result,
		float& out_dist_sqr) const = 0;

	/// \overload for 2D points
	[[nodiscard]] virtual bool nn_single_search(
		const mrpt::math::TPoint2Df& query, mrpt::math::TPoint2Df& result,
		float& out_dist_sqr) const = 0;

	/** Search for the `N` closest 3D points to a given one.
	 *
	 * \param[in]  query The query input point.
	 * \param[out] results The found closest points.
	 * \param[out] out_dists_sqr The square Euclidean distances between the
	 * query and the returned point.
	 */
	virtual void nn_multiple_search(
		const mrpt::math::TPoint3Df& query, const size_t N,
		std::vector<mrpt::math::TPoint3Df>& results,
		std::vector<float>& out_dists_sqr) const = 0;

	/// \overload for 2D points
	virtual void nn_multiple_search(
		const mrpt::math::TPoint2Df& query, const size_t N,
		std::vector<mrpt::math::TPoint2Df>& results,
		std::vector<float>& out_dists_sqr) const = 0;

	/** Radius search for closest 3D points to a given one.
	 *
	 * \param[in]  query The query input point.
	 * \param[in]  search_radius_sqr The search radius, **squared**.
	 * \param[out] results The found closest points.
	 * \param[out] out_dists_sqr The square Euclidean distances between the
	 * query and the returned point.
	 */
	virtual void nn_radius_search(
		const mrpt::math::TPoint3Df& query, const float search_radius_sqr,
		std::vector<mrpt::math::TPoint3Df>& results,
		std::vector<float>& out_dists_sqr) const = 0;

	/// \overload for 2D points
	virtual void nn_radius_search(
		const mrpt::math::TPoint2Df& query, const float search_radius_sqr,
		std::vector<mrpt::math::TPoint2Df>& results,
		std::vector<float>& out_dists_sqr) const = 0;

	/** @} */
};

}  // namespace mrpt::maps
