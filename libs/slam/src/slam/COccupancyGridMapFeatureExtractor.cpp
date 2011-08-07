/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/slam.h>  // Precompiled header


#include <mrpt/slam/COccupancyGridMapFeatureExtractor.h>


using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;


/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
COccupancyGridMapFeatureExtractor::COccupancyGridMapFeatureExtractor()
{
}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
COccupancyGridMapFeatureExtractor::~COccupancyGridMapFeatureExtractor()
{
}


/*---------------------------------------------------------------
						uncached_extractFeatures
  ---------------------------------------------------------------*/
void COccupancyGridMapFeatureExtractor::uncached_extractFeatures(
	const mrpt::slam::COccupancyGridMap2D &grid,
	mrpt::slam::CLandmarksMap	&outMap,
	const size_t  number_of_features,
	const mrpt::vision::TDescriptorType	descriptors,
	const mrpt::vision::CFeatureExtraction::TOptions  &feat_options
	)
{
	MRPT_START

	// get the gridmap as an image:
	CImage	img(1,1,1);
	grid.getAsImageFiltered(img, true /*vertical flip*/,  false /* force RGB */ );

	// Detect features:
	vision::CFeatureExtraction					fExt;
	vision::CFeatureList						lstFeatures;

	fExt.options = feat_options;
	fExt.options.patchSize = 0; // Do NOT extract patch

	// Detect interest points:
	fExt.detectFeatures( img, lstFeatures,0 /* Init ID */, number_of_features );

	// Extract descriptors:
	if (descriptors!=descAny)
		fExt.computeDescriptors(img, lstFeatures, descriptors);

	// Copy all the features to a map of landmarks:
	for (vision::CFeatureList::iterator it=lstFeatures.begin();it!=lstFeatures.end();++it)
	{
		CLandmark lm;
		lm.ID = (*it)->ID;
		lm.features.resize(1);

		lm.features[0] = *it;  // Insert the full feature there:

		lm.pose_mean.x = grid.getXMin() + ((*it)->x+0.5f)* grid.getResolution();
		lm.pose_mean.y = grid.getYMin() + ((*it)->y+0.5f)* grid.getResolution();
		lm.pose_mean.z = 0;

		lm.pose_cov_11=
		lm.pose_cov_22=
		lm.pose_cov_33= square(grid.getResolution());
		lm.pose_cov_12=lm.pose_cov_13=lm.pose_cov_23 = 0;

		lm.seenTimesCount = 1;

		outMap.landmarks.push_back( lm );
	}

	MRPT_END_WITH_CLEAN_UP( try { grid.saveMetricMapRepresentationToFile("__DEBUG_DUMP_GRIDMAP_ON_EXCEPTION"); } catch(...){} );
}

/*---------------------------------------------------------------
						extractFeatures
  ---------------------------------------------------------------*/
void COccupancyGridMapFeatureExtractor::extractFeatures(
	const mrpt::slam::COccupancyGridMap2D &grid,
	mrpt::slam::CLandmarksMap	&outMap,
	const size_t  number_of_features,
	const mrpt::vision::TDescriptorType	descriptors,
	const mrpt::vision::CFeatureExtraction::TOptions  &feat_options
	)
{
#if 0
	// Un-cashed version:
	uncached_extractFeatures(grid,outMap,number_of_features,descriptors,feat_options);
#else
	// Use cache mechanism:

	TCache::const_iterator it=m_cache.find(&grid);
	if (it==m_cache.end())
	{
		// We have to recompute the features:
		CLandmarksMapPtr theMap = CLandmarksMap::Create();

		uncached_extractFeatures(grid,*theMap,number_of_features,descriptors,feat_options);

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
void COccupancyGridMapFeatureExtractor::OnEvent(const mrptEvent &e)
{
	const COccupancyGridMap2D *src = NULL;

	// Upon map change or destruction, remove from our cache:
	if (e.isOfType<mrptEventOnDestroy>()) src = static_cast<const COccupancyGridMap2D*>( static_cast<const mrptEventOnDestroy*>(&e)->source_object );
	if (e.isOfType<mrptEventMetricMapClear>()) src = static_cast<const COccupancyGridMap2D*>( static_cast<const mrptEventMetricMapClear*>(&e)->source_map );
	if (e.isOfType<mrptEventMetricMapInsert>()) src = static_cast<const COccupancyGridMap2D*>( static_cast<const mrptEventMetricMapClear*>(&e)->source_map );

	if (src)
	{
		// Remove from cache:
		m_cache.erase(src);

		// Unsubscribe:
		this->observeEnd( *const_cast<COccupancyGridMap2D*>(src) );
	}

}

