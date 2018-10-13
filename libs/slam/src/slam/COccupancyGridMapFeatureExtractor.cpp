/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "slam-precomp.h"  // Precompiled headers

#include <mrpt/slam/COccupancyGridMapFeatureExtractor.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::img;
using namespace mrpt::system;

void COccupancyGridMapFeatureExtractor::uncached_extractFeatures(
	const mrpt::maps::COccupancyGridMap2D& grid,
	mrpt::maps::CLandmarksMap& outMap, const size_t number_of_features,
	const mrpt::vision::TDescriptorType descriptors,
	const mrpt::vision::CFeatureExtraction::TOptions& feat_options)
{
	MRPT_START

	// get the gridmap as an image:
	CImage img(1, 1, 1);
	grid.getAsImageFiltered(img, true /*vertical flip*/, false /* force RGB */);

	// Detect features:
	vision::CFeatureExtraction fExt;
	vision::CFeatureList lstFeatures;

	fExt.options = feat_options;
	fExt.options.patchSize = 0;  // Do NOT extract patch

	// Detect interest points:
	fExt.detectFeatures(img, lstFeatures, 0 /* Init ID */, number_of_features);

	// Extract descriptors:
	if (descriptors != mrpt::vision::descAny)
		fExt.computeDescriptors(img, lstFeatures, descriptors);

	// Copy all the features to a map of landmarks:
	for (auto& lstFeature : lstFeatures)
	{
		CLandmark lm;
		lm.ID = lstFeature->ID;
		lm.features.resize(1);

		lm.features[0] = lstFeature;  // Insert the full feature there:

		lm.pose_mean.x =
			grid.getXMin() + (lstFeature->x + 0.5f) * grid.getResolution();
		lm.pose_mean.y =
			grid.getYMin() + (lstFeature->y + 0.5f) * grid.getResolution();
		lm.pose_mean.z = 0;

		lm.pose_cov_11 = lm.pose_cov_22 = lm.pose_cov_33 =
			square(grid.getResolution());
		lm.pose_cov_12 = lm.pose_cov_13 = lm.pose_cov_23 = 0;

		lm.seenTimesCount = 1;

		outMap.landmarks.push_back(lm);
	}

	MRPT_END_WITH_CLEAN_UP(try {
		grid.saveMetricMapRepresentationToFile(
			"__DEBUG_DUMP_GRIDMAP_ON_EXCEPTION");
	} catch (...){});
}

/*---------------------------------------------------------------
						extractFeatures
  ---------------------------------------------------------------*/
void COccupancyGridMapFeatureExtractor::extractFeatures(
	const mrpt::maps::COccupancyGridMap2D& grid,
	mrpt::maps::CLandmarksMap& outMap, const size_t number_of_features,
	const mrpt::vision::TDescriptorType descriptors,
	const mrpt::vision::CFeatureExtraction::TOptions& feat_options)
{
#if 0
	// Un-cashed version:
	uncached_extractFeatures(grid,outMap,number_of_features,descriptors,feat_options);
#else
	// Use cache mechanism:

	auto it = m_cache.find(&grid);
	if (it == m_cache.end())
	{
		// We have to recompute the features:
		CLandmarksMap::Ptr theMap = mrpt::make_aligned_shared<CLandmarksMap>();

		uncached_extractFeatures(
			grid, *theMap, number_of_features, descriptors, feat_options);

		outMap = *theMap;

		// Insert into the cache:
		m_cache[&grid] = theMap;
	}
	else
	{
		// Already in the cache:
		outMap = *(it->second);
	}

#endif
}

// This will receive the events from maps in order to purge the cache.
void COccupancyGridMapFeatureExtractor::OnEvent(const mrptEvent& e)
{
	const COccupancyGridMap2D* src = nullptr;

	// Upon map change or destruction, remove from our cache:
	if (e.isOfType<mrptEventOnDestroy>())
		src = static_cast<const COccupancyGridMap2D*>(
			static_cast<const mrptEventOnDestroy*>(&e)->source_object);
	if (e.isOfType<mrptEventMetricMapClear>())
		src = static_cast<const COccupancyGridMap2D*>(
			static_cast<const mrptEventMetricMapClear*>(&e)->source_map);
	if (e.isOfType<mrptEventMetricMapInsert>())
		src = static_cast<const COccupancyGridMap2D*>(
			static_cast<const mrptEventMetricMapClear*>(&e)->source_map);

	if (src)
	{
		// Remove from cache:
		m_cache.erase(src);

		// Unsubscribe:
		this->observeEnd(*const_cast<COccupancyGridMap2D*>(src));
	}
}
