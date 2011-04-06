/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
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

#ifndef mrpt_vision_tracking_H
#define mrpt_vision_tracking_H

#include <mrpt/vision/types.h>
#include <mrpt/vision/link_pragmas.h>

#include <mrpt/vision/CFeature.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/utils/TParameters.h>

#include <mrpt/utils/metaprogramming.h>

#include <memory> 	// for auto_ptr

namespace mrpt
{
	namespace vision
	{
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


		/** A virtual interface for all feature trackers, implementing the part of feature tracking that is common to any specific tracker implementation.
		  *   This class provides a quite robust tracking of features, avoiding as many outliers as possible but not all of them:
		  *    more robust tracking would require application-specific information and could be done in a number of very different approaches,
		  *    so this class will not try to do any kind of RANSAC or any other advanced outlier rejection; instead, it should
		  *    be done by the users or the classes that employ this class.
		  *
		  *   The basic usage of this class is as follows:
		  *    \code
		  *       CFeatureTracker_KL    tracker;  // Note: CFeatureTracker_KL is the most robust implementation for now.
		  *       tracker.extra_params["add_new_features"] = 1;  // Enable detection of new features, not only tracking
		  *       tracker.extra_params[...] = ...
		  *       // ....
		  *       CFeatureList	theFeats;  // The list of features
		  *       mrpt::utils::CImage  previous_img, current_img;
		  *
		  *       while (true) {
		  *           current_img = ... // Grab new image.
		  *           if ( previous_img_is_ok )
		  *               tracker.trackFeatures(previous_img, current_img, theFeats);
		  *           previous_img = current_img;
		  *       }
		  *    \endcode
		  *
		  *  Below follows the list of optional parameters for "extra_params" which can be set
		  *  and will be understood by this base class for any specific tracker implementation.
		  *  Note that all parameters are double's, but boolean flags are emulated by the values 0.0 (false) and 1.0 (true).
		  *
		  *  List of parameters:
		  * <table border="1" >
		  *   <tr><td align="center" > <b>Parameter name</b>  </td>  <td align="center" > <b>Default value</b> </td> <td align="center" > <b>Comments</b> </td> </tr>
		  *   <tr><td align="center" > add_new_features  </td>  <td align="center" > 0 </td>
		  *      <td> If set to "1", the class will not only track existing features, but will also perform (after doing the actual tracking) an efficient
		  *            search for new features with the FAST detector, and will add them to the passed "CFeatureList" if they fulfill a set of restrictions,
		  *            as stablished by the other parameters (see <i>add_new_feat_min_separation</i>,<i>add_new_feat_max_features</i>,<i>minimum_KLT_response_to_add</i>).
		  *        </td> </tr>
		  *   <tr><td align="center" > add_new_feat_min_separation  </td>  <td align="center" > 15 </td>
		  *      <td> If <i>add_new_features</i>==1,  this is the minimum separation (in pixels) to any other (old, or new) feature for it
		  *             being considered a candidate to be added.
		  *         </td> </tr>
		  *   <tr><td align="center" > add_new_feat_max_features  </td>  <td align="center" > 100 </td>
		  *      <td> If <i>add_new_features</i>==1,  FAST features are detected in each frame, and only the best <i>add_new_feat_max_features</i> keypoints
		  *             (ordered by their KLT response) will be considered as eligible for addition to the set of tracked features.
		  *         </td> </tr>
		  *   <tr><td align="center" > add_new_feat_patch_size  </td>  <td align="center" > 11 </td>
		  *      <td> If <i>add_new_features</i>==1,  for each new added feature, this is the size of the patch to be extracted around the keypoint (set to 0 if patches are not required at all).
		  *          </td> </tr>
		  *   <tr><td align="center" > minimum_KLT_response_to_add  </td>  <td align="center" > 10 </td>
		  *      <td> If <i>add_new_features</i>==1, this sets the minimum KLT response of candidate FAST features to be added in each frame, if they also fulfil the other restrictions (e.g. min.distance).
		  *         </td> </tr>
		  *   <tr><td align="center" > check_KLT_response_every  </td>  <td align="center" > 0 </td>
		  *      <td> If >0, it will compute the KLT response at each feature point every <i>N</i> frames
		  *            and those below <i>minimum_KLT_response</i> will be marked as "lost" in their "track_status" field.
		  *        </td> </tr>
		  *   <tr><td align="center" > minimum_KLT_response  </td>  <td align="center" > 5 </td>
		  *      <td> See explanation of <i>check_KLT_response_every</i>.
		  *        </td> </tr>
		  *   <tr><td align="center" > KLT_response_half_win  </td>  <td align="center" > 4 </td>
		  *      <td> When computing the KLT response of features (see <i>minimum_KLT_response</i> and <i>minimum_KLT_response_to_add</i>),
		  *            the window centered at the point for its estimation will be of size (2*W+1)x(2*W+1), with <i>W</i> being this parameter value.
		  *       </td> </tr>
		  *   <tr><td align="center" > update_patches_every  </td>  <td align="center" > 0 </td>
		  *      <td> If !=0, the patch associated to each feature will be updated with every N'th frame. </td> </tr>
		  * </table>
		  *
		  *  This class also offers a time profiler, disabled by default (see getProfiler and enableTimeLogger).
		  *
		  * \sa CFeatureTracker_KL, the example application "track-video-features".
		  */
		struct VISION_IMPEXP  CGenericFeatureTracker
		{
			/** Optional list of extra parameters to the algorithm. */
			mrpt::utils::TParametersDouble extra_params;

			/** Default ctor */
			inline CGenericFeatureTracker() : m_timlog(false), m_update_patches_counter(0),m_check_KLT_counter(0),m_detector_adaptive_thres(10)
			{ }
			/** Ctor with extra parameters */
			inline CGenericFeatureTracker(mrpt::utils::TParametersDouble extraParams) : extra_params(extraParams), m_timlog(false), m_update_patches_counter(0),m_check_KLT_counter(0),m_detector_adaptive_thres(10)
			{ }
			/** Dtor */
			virtual ~CGenericFeatureTracker()
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

			/** Returns a read-only reference to the internal time logger */
			inline const mrpt::utils::CTimeLogger  & getProfiler() const { return m_timlog; }
			/** Returns a reference to the internal time logger */
			inline mrpt::utils::CTimeLogger  & getProfiler() { return m_timlog; }

			/** Returns a read-only reference to the internal time logger */
			inline void enableTimeLogger(bool enable=true) { m_timlog.enable(enable); }

		protected:
			/** The tracking method implementation, to be implemented in children classes. */
			virtual void trackFeatures_impl(
				const CImage &old_img,
				const CImage &new_img,
				vision::CFeatureList &inout_featureList ) = 0;

			mrpt::utils::CTimeLogger  m_timlog; //!< the internal time logger, disabled by default.

		private:
			size_t		m_update_patches_counter;	//!< for use when "update_patches_every">=1
			size_t		m_check_KLT_counter;	//!< For use when "check_KLT_response_every">=1
			int			m_detector_adaptive_thres;  //!< For use in "add_new_features" == true

		};

		typedef std::auto_ptr<CGenericFeatureTracker> CGenericFeatureTrackerAutoPtr;


		/** Track a set of features from old_img -> new_img using sparse optimal flow (classic KL method).
		  *
		  *  See CGenericFeatureTracker for a more detailed explanation on how to use this class.
		  *
		  *   List of additional parameters in "extra_params" (apart from those in CGenericFeatureTracker) accepted by this class:
		  *		- "window_width"  (Default=15)
		  *		- "window_height" (Default=15)
		  *
		  *  \sa OpenCV's method cvCalcOpticalFlowPyrLK
		  */
		struct VISION_IMPEXP CFeatureTracker_KL : public CGenericFeatureTracker
		{
			/** Default ctor */
			inline CFeatureTracker_KL() { }
			/** Ctor with extra parameters */
			inline CFeatureTracker_KL(mrpt::utils::TParametersDouble extraParams) : CGenericFeatureTracker(extraParams)	{ }

		protected:
			virtual void trackFeatures_impl(
				const CImage &old_img,
				const CImage &new_img,
				vision::CFeatureList &inout_featureList );
		};

		/** Track a set of features from old_img -> new_img by patch correlation over the closest FAST features, using a KD-tree for looking closest correspondences.
		  *  See CGenericFeatureTracker for a more detailed explanation on how to use this class.
		  *
		  *   List of additional parameters in "extra_params" (apart from those in CGenericFeatureTracker) accepted by this class:
		  *		- "window_width"  (Default=15)
		  *		- "window_height" (Default=15)
		  *
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
			virtual void trackFeatures_impl(
				const CImage &old_img,
				const CImage &new_img,
				vision::CFeatureList &inout_featureList );

		private:
			int		m_detector_adaptive_thres;		//!< threshold for cvFAST()
			size_t	m_hysteresis_min_num_feats, m_hysteresis_max_num_feats; //!< for the adaptive control of "m_detector_adaptive_thres"
		};


		/** Track a set of features from old_img -> new_img by patch matching over a fixed window centered at each feature's previous location.
		  *  See CGenericFeatureTracker for a more detailed explanation on how to use this class.
		  *
		  *   List of additional parameters in "extra_params" (apart from those in CGenericFeatureTracker) accepted by this class:
		  *		- "window_width"  (Default=15)
		  *		- "window_height" (Default=15)
		  *
		  */
		struct VISION_IMPEXP CFeatureTracker_PatchMatch : public CGenericFeatureTracker
		{
			/** Ctor */
			CFeatureTracker_PatchMatch(const mrpt::utils::TParametersDouble & extraParams = mrpt::utils::TParametersDouble() );

		protected:
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
		  *  Deprecated: See CGenericFeatureTracker
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
