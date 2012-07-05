/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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

