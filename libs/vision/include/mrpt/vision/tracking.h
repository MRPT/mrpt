/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#ifndef mrpt_vision_tracking_H
#define mrpt_vision_tracking_H

#include <mrpt/vision/types.h>
#include <mrpt/vision/link_pragmas.h>

#include <mrpt/vision/CFeature.h>
#include <mrpt/utils/CImage.h>

#include <mrpt/utils/metaprogramming.h>

namespace mrpt
{
	namespace vision
	{
		using namespace mrpt::slam;
		using namespace mrpt::math;
		using namespace mrpt::utils;

		/**  @name All related to feature tracking
		     @{
		  */

		/** (Deprecated) Track a set of features from img1 -> img2 using sparse optimal flow (classic KL method)
		  *  \sa OpenCV's method cvCalcOpticalFlowPyrLK
		  */
		MRPT_DECLARE_DEPRECATED_FUNCTION("Deprecated: See CGenericFeatureTracker",
		 void VISION_IMPEXP trackFeatures(
			const CImage &old_img,
			const CImage &new_img,
			vision::CFeatureList &featureList,
			const unsigned int window_width = 15,
			const unsigned int window_height = 15 ) );


		/** A virtual interface for all feature trackers.
		  */
		struct VISION_IMPEXP  CGenericFeatureTracker
		{
			/** Optional list of extra parameters to the algorithm. */
			mrpt::utils::TParametersDouble extra_params;

			/** Default ctor */
			inline CGenericFeatureTracker()
			{ }
			/** Ctor with extra parameters */
			inline CGenericFeatureTracker(mrpt::utils::TParametersDouble extraParams) : extra_params(extraParams)
			{ }

			/** Perform feature tracking from "old_img" to "new_img", with a (possibly empty) list of previously tracked features "inout_featureList".
			  *  This is a list of parameters (in "extraParams") accepted by ALL implementations of feature tracker (see each derived class for more specific parameters).
			  *		- "add_new_features" (Default=0). If set to "1", new features will be also added to the existing ones in areas of the image poor of features.
			  * This method does:
			  *    - Convert old and new images to grayscale, if they're in color.
			  *    - Call the pure virtual "trackFeatures_impl" method.
			  *    - Implement the optional detection of new features if "add_new_features"!=0.
			  */
			void trackFeatures(
				const CImage &old_img,
				const CImage &new_img,
				vision::CFeatureList &inout_featureList );

			/** A wrapper around the basic trackFeatures() method, but keeping the original list of features unmodified and returns the tracked ones in a new list. */
			inline void trackFeaturesNewList(
				const CImage &old_img,
				const CImage &new_img,
				const vision::CFeatureList &in_featureList,
				vision::CFeatureList &out_featureList
				)
			{
				out_featureList = in_featureList;
				std::for_each(
					out_featureList.begin(),out_featureList.end(),
					mrpt::utils::metaprogramming::ObjectMakeUnique() );
				this->trackFeatures(old_img, new_img, out_featureList);
			}
		protected:
			/** The tracking method implementation, to be implemented in children classes. */
			virtual void trackFeatures_impl(
				const CImage &old_img,
				const CImage &new_img,
				vision::CFeatureList &inout_featureList ) = 0;
		};


		/** Track a set of features from old_img -> new_img using sparse optimal flow (classic KL method).
		  *  See CFeatureTracker_KL::trackFeatures for the list of existing parameters.
		  *  \sa OpenCV's method cvCalcOpticalFlowPyrLK
		  */
		struct VISION_IMPEXP CFeatureTracker_KL : public CGenericFeatureTracker
		{
			/** Default ctor */
			inline CFeatureTracker_KL() { }
			/** Ctor with extra parameters */
			inline CFeatureTracker_KL(mrpt::utils::TParametersDouble extraParams) : CGenericFeatureTracker(extraParams)	{ }

		protected:
			/** Optional parameters that can be passed in "extra_params":
			  *		- "window_width"  (Default=15)
			  *		- "window_height" (Default=15)
			  */
			virtual void trackFeatures_impl(
				const CImage &old_img,
				const CImage &new_img,
				vision::CFeatureList &inout_featureList );
		};

		/** Track a set of features from old_img -> new_img by patch correlation over the closest FAST features, using a KD-tree for looking closest correspondences.
		  *  See CFeatureTracker_FAST::trackFeatures for a list of optional parameters.
		  */
		struct VISION_IMPEXP CFeatureTracker_FAST : public CGenericFeatureTracker
		{
			/** Ctor */
			CFeatureTracker_FAST(const mrpt::utils::TParametersDouble & extraParams = mrpt::utils::TParametersDouble() );

			struct VISION_IMPEXP TExtraOutputInfo
			{
				size_t  raw_FAST_feats_detected;  //!< In the new_img with the last adaptive threshold
			};

			TExtraOutputInfo  last_execution_extra_info; //!< Updated with each call to trackFeatures()

		protected:
			/**  Optional parameters that can be passed in "extra_params":
			  *		- "window_width"  (Default=15)
			  *		- "window_height" (Default=15)
			  */
			virtual void trackFeatures_impl(
				const CImage &old_img,
				const CImage &new_img,
				vision::CFeatureList &inout_featureList );

		private:
			int		m_detector_adaptive_thres;		//!< threshold for cvFAST()
			size_t	m_hysteresis_min_num_feats, m_hysteresis_max_num_feats; //!< for the adaptive control of "m_detector_adaptive_thres"
		};


		/** Track a set of features from old_img -> new_img by patch matching over a fixed window centered at each feature's previous location.
		  *  See CFeatureTracker_PatchMatch::trackFeatures for a list of optional parameters.
		  */
		struct VISION_IMPEXP CFeatureTracker_PatchMatch : public CGenericFeatureTracker
		{
			/** Ctor */
			CFeatureTracker_PatchMatch(const mrpt::utils::TParametersDouble & extraParams = mrpt::utils::TParametersDouble() );

		protected:
			/**  Optional parameters that can be passed in "extra_params":
			  *		- "window_width"  (Default=15)
			  *		- "window_height" (Default=15)
			  *		- "match_method" (Default=0)
			  *
			  *  Possible values for "match_method":
			  *		- 0 : Normalized cross correlation
			  */
			virtual void trackFeatures_impl(
				const CImage &old_img,
				const CImage &new_img,
				vision::CFeatureList &inout_featureList );
		};


		/** (Deprecated) Track a set of features from img1 -> img2 using sparse optimal flow (classic KL method)
		  *  \sa OpenCV's method cvCalcOpticalFlowPyrLK
		  */
		MRPT_DECLARE_DEPRECATED_FUNCTION("Deprecated: See CGenericFeatureTracker",
		void VISION_IMPEXP trackFeatures(
			const CImage &inImg1,
			const CImage &inImg2,
			const CFeatureList &inFeatureList,
			CFeatureList &outFeatureList,
			const unsigned int window_width,
			const unsigned int window_height)
			);


		/** Search for correspondences which are not in the same row and deletes them
		  * ...
		  */
		void VISION_IMPEXP checkTrackedFeatures( CFeatureList &leftList,
							CFeatureList &rightList,
							vision::TMatchingOptions options);

		/** Tracks a set of features in an image.
		  */
		void VISION_IMPEXP trackFeatures2(
			const CImage &inImg1,
			const CImage &inImg2,
			CFeatureList &featureList,
			const unsigned int window_width = 15,
			const unsigned int window_height = 15);

		/** Filter bad correspondences by distance
		  * ...
		  */
		void VISION_IMPEXP filterBadCorrsByDistance( mrpt::utils::TMatchingPairList &list,	// The list of correspondences
										unsigned int numberOfSigmas );				// Threshold



		/**  @}  */
	}
}


#endif
