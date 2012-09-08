/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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
#ifndef COccupancyGridMapFeatureExtractor_H
#define COccupancyGridMapFeatureExtractor_H

#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/slam/CLandmarksMap.h>
#include <mrpt/vision/CFeatureExtraction.h>

#include <mrpt/utils/CObserver.h>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
	namespace slam
	{
		using namespace mrpt::math;

		/**  A class for detecting features from occupancy grid maps. 
		  *   The main method is "COccupancyGridMapFeatureExtractor::extractFeatures()", which makes use
		  *    of an advanced cache mechanism to avoid redoing work when applied several times on the same
		  *    occupancy grid maps (unless they changed in the meanwhile). 
		  *  
		  *  For an uncached version (which is a static method that can be called without instantiating COccupancyGridMapFeatureExtractor)
		  *  see COccupancyGridMapFeatureExtractor::uncached_extractFeatures()
		  *
		  * \ingroup mrpt_slam_grp
		  */
		class SLAM_IMPEXP COccupancyGridMapFeatureExtractor : public mrpt::utils::CObserver
		{
		public:
			COccupancyGridMapFeatureExtractor(); //!< Default ctor
			~COccupancyGridMapFeatureExtractor(); //!< Destructor

			/** Computes a set of distinctive landmarks from an occupancy grid, and store them (previous content is not erased!) into the given landmarks map.
			  *   Landmarks type can be any declared in mrpt::vision::CFeatureExtraction::TOptions 
			  *
			  * \note See the paper "..."
			  * \sa uncached_extractFeatures
			  */
			void extractFeatures(
				const mrpt::slam::COccupancyGridMap2D &grid,
				mrpt::slam::CLandmarksMap	&outMap,
				const size_t  number_of_features,
				const mrpt::vision::TDescriptorType	descriptors,
				const mrpt::vision::CFeatureExtraction::TOptions  &feat_options
				);

			/** Computes a set of distinctive landmarks from an occupancy grid, and store them (previous content is not erased!) into the given landmarks map.
			  *   Landmarks type can be any declared in mrpt::vision::CFeatureExtraction::TOptions 
			  *
			  * \note See the paper "..."
			  * \sa uncached_extractFeatures
			  */
			static void uncached_extractFeatures(
				const mrpt::slam::COccupancyGridMap2D &grid,
				mrpt::slam::CLandmarksMap	&outMap,
				const size_t  number_of_features,
				const mrpt::vision::TDescriptorType	descriptors,
				const mrpt::vision::CFeatureExtraction::TOptions  &feat_options
				);

		protected:
			void OnEvent(const mrptEvent &e); //!< This will receive the events from maps in order to purge the cache.
			typedef std::map<const mrpt::slam::COccupancyGridMap2D*,mrpt::slam::CLandmarksMapPtr> TCache;
			TCache m_cache; //!< A cache of already computed maps.


		}; // End of class def.

	} // End of namespace
} // End of namespace

#endif
