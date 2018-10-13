/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/vision/CFeatureExtraction.h>

#include <mrpt/system/CObserver.h>

namespace mrpt::slam
{
/**  A class for detecting features from occupancy grid maps.
 *   The main method is "COccupancyGridMapFeatureExtractor::extractFeatures()",
 * which makes use
 *    of an advanced cache mechanism to avoid redoing work when applied several
 * times on the same
 *    occupancy grid maps (unless they changed in the meanwhile).
 *
 *  For an uncached version (which is a static method that can be called
 * without instantiating COccupancyGridMapFeatureExtractor)
 *  see COccupancyGridMapFeatureExtractor::uncached_extractFeatures()
 *
 * \ingroup mrpt_slam_grp
 */
class COccupancyGridMapFeatureExtractor : public mrpt::system::CObserver
{
   public:
	/** Computes a set of distinctive landmarks from an occupancy grid, and
	 * store them (previous content is not erased!) into the given landmarks
	 * map.
	 *   Landmarks type can be any declared in
	 * mrpt::vision::CFeatureExtraction::TOptions
	 *
	 * \note See the paper "..."
	 * \sa uncached_extractFeatures
	 */
	void extractFeatures(
		const mrpt::maps::COccupancyGridMap2D& grid,
		mrpt::maps::CLandmarksMap& outMap, const size_t number_of_features,
		const mrpt::vision::TDescriptorType descriptors,
		const mrpt::vision::CFeatureExtraction::TOptions& feat_options);

	/** Computes a set of distinctive landmarks from an occupancy grid, and
	 * store them (previous content is not erased!) into the given landmarks
	 * map.
	 *   Landmarks type can be any declared in
	 * mrpt::vision::CFeatureExtraction::TOptions
	 *
	 * \note See the paper "..."
	 * \sa uncached_extractFeatures
	 */
	static void uncached_extractFeatures(
		const mrpt::maps::COccupancyGridMap2D& grid,
		mrpt::maps::CLandmarksMap& outMap, const size_t number_of_features,
		const mrpt::vision::TDescriptorType descriptors,
		const mrpt::vision::CFeatureExtraction::TOptions& feat_options);

   protected:
	/** This will receive the events from maps in order to purge the cache. */
	void OnEvent(const mrpt::system::mrptEvent& e) override;
	using TCache = std::map<
		const mrpt::maps::COccupancyGridMap2D*, mrpt::maps::CLandmarksMap::Ptr>;
	/** A cache of already computed maps. */
	TCache m_cache;

};  // End of class def.

}  // namespace mrpt::slam
