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
