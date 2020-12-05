/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/vision/types.h>

#include <mrpt/containers/yaml.h>
#include <mrpt/img/CImage.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/vision/TKeyPoint.h>
#include <memory>  // for unique_ptr

namespace mrpt::vision
{
/** \addtogroup vision_tracking Feature detection and tracking
 *  \ingroup mrpt_vision_grp
 *   @{  */

/** A virtual interface for all feature trackers, implementing the part of
 * feature tracking that is common to any specific tracker implementation.
 *   This class provides a quite robust tracking of features, avoiding as many
 * outliers as possible but not all of them:
 *    more robust tracking would require application-specific information and
 * could be done in a number of very different approaches,
 *    so this class will not try to do any kind of RANSAC or any other advanced
 * outlier rejection; instead, it should
 *    be done by the users or the classes that employ this class.
 *
 *   The basic usage of this class is as follows:
 *    \code
 *       CFeatureTracker_KL    tracker;  // Note: CFeatureTracker_KL is the
 * most robust implementation for now.
 *       tracker.extra_params["add_new_features"] = 1;  // Enable detection of
 * new features, not only tracking
 *       tracker.extra_params[...] = ...
 *       // ....
 *       TKeyPointList theFeats;  // The list of features
 *       mrpt::img::CImage  previous_img, current_img;
 *
 *       while (true) {
 *           current_img = ... // Grab new image.
 *           if ( previous_img_is_ok )
 *               tracker.trackFeatures(previous_img, current_img, theFeats);
 *           previous_img = current_img;
 *       }
 *    \endcode
 *
 *  Below follows the list of optional parameters for "extra_params" which can
 * be set
 *  and will be understood by this base class for any specific tracker
 * implementation.
 *  Note that all parameters are double's, but boolean flags are emulated by
 * the values 0.0 (false) and 1.0 (true).
 *
 *  List of parameters:
 * <table border="1" >
 *   <tr><td align="center" > <b>Parameter name</b>  </td>  <td align="center"
 * > <b>Default value</b> </td> <td align="center" > <b>Comments</b> </td> </tr>
 *   <tr><td align="center" > add_new_features  </td>  <td align="center" > 0
 * </td>
 *      <td> If set to "1", the class will not only track existing features,
 * but will also perform (after doing the actual tracking) an efficient
 *            search for new features with the FAST detector, and will add them
 * to the passed feature list if they fulfill a set of restrictions,
 *            as stablished by the other parameters (see
 * <i>add_new_feat_min_separation</i>,<i>add_new_feat_max_features</i>,<i>minimum_KLT_response_to_add</i>).
 *        </td> </tr>
 *   <tr><td align="center" > add_new_feat_min_separation  </td>  <td
 * align="center" > 15 </td>
 *      <td> If <i>add_new_features</i>==1,  this is the minimum separation (in
 * pixels) to any other (old, or new) feature for it
 *             being considered a candidate to be added.
 *         </td> </tr>
 *   <tr><td align="center" > desired_num_features_adapt  </td>  <td
 * align="center" > (img_width*img_height)/512 </td>
 *      <td> If <i>add_new_features</i>==1, the threshold of the FAST(ER)
 * feature detector is dynamically adapted such as the number of
 *        raw FAST keypoints is around this number. This number should be much
 * higher than the real desired numbre of features, since this
 *        one includes many features concentrated in space which are later
 * discarded for the minimum distance.
 *         </td> </tr>
 *   <tr><td align="center" > desired_num_features </td>  <td align="center" >
 * 100 </td>
 *      <td> If <i>add_new_features</i>==1, the target number of the patch
 * associated to each feature will be updated with every N'th frame. </td> </tr>
 *   <tr><td align="center" > add_new_feat_patch_size  </td>  <td
 * align="center" > 11 </td>
 *      <td> If <i>add_new_features</i>==1,  for each new added feature, this
 * is the size of the patch to be extracted around the keypoint (set to 0 if
 * patches are not required at all).
 *          </td> </tr>
 *   <tr><td align="center" > minimum_KLT_response_to_add  </td>  <td
 * align="center" > 10 </td>
 *      <td> If <i>add_new_features</i>==1, this sets the minimum KLT response
 * of candidate FAST features to be added in each frame, if they also fulfil the
 * other restrictions (e.g. min.distance).
 *         </td> </tr>
 *   <tr><td align="center" > check_KLT_response_every  </td>  <td
 * align="center" > 0 </td>
 *      <td> If >0, it will compute the KLT response at each feature point
 * every <i>N</i> frames
 *            and those below <i>minimum_KLT_response</i> will be marked as
 * "lost" in their "track_status" field.
 *        </td> </tr>
 *   <tr><td align="center" > minimum_KLT_response  </td>  <td align="center" >
 * 5 </td>
 *      <td> See explanation of <i>check_KLT_response_every</i>.
 *        </td> </tr>
 *   <tr><td align="center" > KLT_response_half_win  </td>  <td align="center"
 * > 4 </td>
 *      <td> When computing the KLT response of features (see
 * <i>minimum_KLT_response</i> and <i>minimum_KLT_response_to_add</i>),
 *            the window centered at the point for its estimation will be of
 * size (2*W+1)x(2*W+1), with <i>W</i> being this parameter value.
 *       </td> </tr>
 *   <tr><td align="center" > update_patches_every  </td>  <td align="center" >
 * 0 </td>
 *      <td> If !=0, the patch associated to each feature will be updated with
 * every N'th frame. </td> </tr>
 *   <tr><td align="center" > remove_lost_features  </td>  <td align="center" >
 * 0 </td>
 *      <td> If !=0, out-of-bound features or those lost while tracking, will
 * be automatically removed from the list of features.
 *            Otherwise, the user will have to manually remove them by checking
 * the track_status field. </td> </tr>
 * </table>
 *
 *  This class also offers a time profiler, disabled by default (see
 * getProfiler and enableTimeLogger).
 *
 * \sa CFeatureTracker_KL, the example application "track-video-features".
 */
struct CGenericFeatureTracker
{
	/** Optional list of extra parameters to the algorithm. */
	mrpt::containers::yaml extra_params;

	/** Default ctor */
	inline CGenericFeatureTracker() {}

	/** Ctor with extra parameters */
	inline CGenericFeatureTracker(const mrpt::containers::yaml& extraParams)
		: extra_params(extraParams)
	{
	}

	/** Dtor */
	virtual ~CGenericFeatureTracker() = default;

	/** Perform feature tracking from "old_img" to "new_img", with a (possibly
	 *empty) list of previously tracked features "inout_featureList".
	 *  This is a list of parameters (in "extraParams") accepted by ALL
	 *implementations of feature tracker (see each derived class for more
	 *specific parameters).
	 *		- "add_new_features" (Default=0). If set to "1", new features will
	 *be
	 *also added to the existing ones in areas of the image poor of features.
	 * This method does:
	 *    - Convert old and new images to grayscale, if they're in color.
	 *    - Call the pure virtual "trackFeatures_impl" method.
	 *    - Implement the optional detection of new features if
	 *"add_new_features"!=0.
	 */
	void trackFeatures(
		const mrpt::img::CImage& old_img, const mrpt::img::CImage& new_img,
		TKeyPointList& inout_featureList);

	/** overload with subpixel precision */
	void trackFeatures(
		const mrpt::img::CImage& old_img, const mrpt::img::CImage& new_img,
		TKeyPointfList& inout_featureList);

	/** Returns a read-only reference to the internal time logger */
	inline const mrpt::system::CTimeLogger& getProfiler() const
	{
		return m_timlog;
	}
	/** Returns a reference to the internal time logger */
	inline mrpt::system::CTimeLogger& getProfiler() { return m_timlog; }
	/** Returns a read-only reference to the internal time logger */
	inline void enableTimeLogger(bool enable = true)
	{
		m_timlog.enable(enable);
	}

	/** Returns the current adaptive threshold used by the FAST(ER) detector to
	 * find out new features in empty areas */
	inline int getDetectorAdaptiveThreshold() const
	{
		return m_detector_adaptive_thres;
	}

	struct TExtraOutputInfo
	{
		/** In the new_img with the last adaptive threshold */
		size_t raw_FAST_feats_detected;
		/** The number of features which were deleted due to OOB, bad tracking,
		 * etc... (only if "remove_lost_features" is enabled) */
		size_t num_deleted_feats;
	};

	/** Updated with each call to trackFeatures() */
	TExtraOutputInfo last_execution_extra_info;

   protected:
	/** The tracking method implementation, to be implemented in children
	 * classes. */
	virtual void trackFeatures_impl(
		const mrpt::img::CImage& old_img, const mrpt::img::CImage& new_img,
		TKeyPointfList& inout_featureList);

	/** The tracking method implementation, to be implemented in children
	 * classes. */
	virtual void trackFeatures_impl(
		const mrpt::img::CImage& old_img, const mrpt::img::CImage& new_img,
		TKeyPointList& inout_featureList) = 0;

	/** the internal time logger, disabled by default. */
	mrpt::system::CTimeLogger m_timlog{false};

	/** This field is clared by \a trackFeatures() before calling \a
	 * trackFeatures_impl(), and
	 *   can be filled out with newly defected FAST(ER) features in the latter.
	 * If it's not the case, feats will be computed anyway if the user enabled
	 * the "add_new_features" option.
	 */
	mrpt::vision::TKeyPointList m_newly_detected_feats;

	/** Adapts the threshold \a m_detector_adaptive_thres according to the real
	 * and desired number of features just detected */
	void updateAdaptiveNewFeatsThreshold(
		const size_t nNewlyDetectedFeats, const size_t desired_num_features);

   private:
	/** for use when "update_patches_every">=1 */
	size_t m_update_patches_counter{0};
	/** For use when "check_KLT_response_every">=1 */
	size_t m_check_KLT_counter{0};
	/** For use in "add_new_features" == true */
	int m_detector_adaptive_thres{10};

	template <typename FEATLIST>
	void internal_trackFeatures(
		const mrpt::img::CImage& old_img, const mrpt::img::CImage& new_img,
		FEATLIST& inout_featureList);
};

using CGenericFeatureTrackerAutoPtr = std::unique_ptr<CGenericFeatureTracker>;

/** Track a set of features from old_img -> new_img using sparse optimal flow
 *(classic KL method).
 *
 *  See CGenericFeatureTracker for a more detailed explanation on how to use
 *this class.
 *
 *   List of additional parameters in "extra_params" (apart from those in
 *CGenericFeatureTracker) accepted by this class:
 *		- "window_width"  (Default=15)
 *		- "window_height" (Default=15)
 *		- "LK_levels" (Default=3) Number of pyramids to build for LK tracking
 *(this
 *parameter only has effects when tracking with CImage's, not with
 *CImagePyramid's).
 *		- "LK_max_iters" (Default=10) Max. number of iterations in LK tracking.
 *		- "LK_epsilon" (Default=0.1) Minimum epsilon step in interations of
 *LK_tracking.
 *		- "LK_max_tracking_error" (Default=150.0) The maximum "tracking error"
 *of
 *LK tracking such as a feature is marked as "lost".
 *
 *  \sa OpenCV's method cvCalcOpticalFlowPyrLK
 */
struct CFeatureTracker_KL : public CGenericFeatureTracker
{
	/** Default ctor */
	inline CFeatureTracker_KL() = default;
	/** Ctor with extra parameters */
	inline CFeatureTracker_KL(const mrpt::containers::yaml& extraParams)
		: CGenericFeatureTracker(extraParams)
	{
	}

   protected:
	void trackFeatures_impl(
		const mrpt::img::CImage& old_img, const mrpt::img::CImage& new_img,
		TKeyPointList& inout_featureList) override;
	void trackFeatures_impl(
		const mrpt::img::CImage& old_img, const mrpt::img::CImage& new_img,
		TKeyPointfList& inout_featureList) override;

   private:
	template <typename FEATLIST>
	void trackFeatures_impl_templ(
		const mrpt::img::CImage& old_img, const mrpt::img::CImage& new_img,
		FEATLIST& inout_featureList);
};

/**  @}  */  // end of grouping
}  // namespace mrpt::vision
