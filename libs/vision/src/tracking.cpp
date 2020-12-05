/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "vision-precomp.h"  // Precompiled headers

#include <mrpt/3rdparty/do_opencv_includes.h>
#include <mrpt/math/ops_matrices.h>
#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/vision/tracking.h>

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::img;
using namespace mrpt::tfest;
using namespace mrpt::math;
using namespace std;

// ------------------------------- internal helper templates
// ---------------------------------
namespace mrpt::vision::detail
{
template <typename FEATLIST>
inline void trackFeatures_checkResponses(
	FEATLIST& featureList, const CImage& cur_gray,
	const float minimum_KLT_response, const unsigned int KLT_response_half_win,
	const unsigned int max_x, const unsigned int max_y);

template <>
inline void trackFeatures_checkResponses<CFeatureList>(
	CFeatureList& featureList, const CImage& cur_gray,
	const float minimum_KLT_response, const unsigned int KLT_response_half_win,
	const unsigned int max_x, const unsigned int max_y)
{
	for (auto& ft : featureList)
	{
		if (ft.track_status != status_TRACKED)
			continue;  // Skip if it's not correctly tracked.

		const unsigned int x = ft.keypoint.pt.x;
		const unsigned int y = ft.keypoint.pt.y;
		if (x > KLT_response_half_win && y > KLT_response_half_win &&
			x < max_x && y < max_y)
		{  // Update response:
			ft.response = cur_gray.KLT_response(x, y, KLT_response_half_win);

			// Is it good enough?
			// http://grooveshark.com/s/Goonies+Are+Good+Enough/2beBfO?src=5
			if (ft.response < minimum_KLT_response)
			{  // Nope!
				ft.track_status = status_LOST;
			}
		}
		else
		{  // Out of bounds
			ft.response = 0;
			ft.track_status = status_OOB;
		}
	}
}  // end of trackFeatures_checkResponses<>

template <class FEAT_LIST>
inline void trackFeatures_checkResponses_impl_simple(
	FEAT_LIST& featureList, const CImage& cur_gray,
	const float minimum_KLT_response, const unsigned int KLT_response_half_win,
	const unsigned int max_x_, const unsigned int max_y_)
{
	if (featureList.empty()) return;

	using pixel_coord_t = typename FEAT_LIST::feature_t::pixel_coord_t;
	const auto half_win = static_cast<pixel_coord_t>(KLT_response_half_win);
	const auto max_x = static_cast<pixel_coord_t>(max_x_);
	const auto max_y = static_cast<pixel_coord_t>(max_y_);

	for (int N = featureList.size() - 1; N >= 0; --N)
	{
		typename FEAT_LIST::feature_t& ft = featureList[N];
		if (ft.track_status != status_TRACKED)
			continue;  // Skip if it's not correctly tracked.

		if (ft.pt.x > half_win && ft.pt.y > half_win && ft.pt.x < max_x &&
			ft.pt.y < max_y)
		{  // Update response:
			ft.response =
				cur_gray.KLT_response(ft.pt.x, ft.pt.y, KLT_response_half_win);

			// Is it good enough?
			// http://grooveshark.com/s/Goonies+Are+Good+Enough/2beBfO?src=5
			if (ft.response < minimum_KLT_response)
			{  // Nope!
				ft.track_status = status_LOST;
			}
		}
		else
		{  // Out of bounds
			ft.response = 0;
			ft.track_status = status_OOB;
		}
	}
}  // end of trackFeatures_checkResponses<>

template <>
inline void trackFeatures_checkResponses<TKeyPointList>(
	TKeyPointList& featureList, const CImage& cur_gray,
	const float minimum_KLT_response, const unsigned int KLT_response_half_win,
	const unsigned int max_x, const unsigned int max_y)
{
	trackFeatures_checkResponses_impl_simple<TKeyPointList>(
		featureList, cur_gray, minimum_KLT_response, KLT_response_half_win,
		max_x, max_y);
}
template <>
inline void trackFeatures_checkResponses<TKeyPointfList>(
	TKeyPointfList& featureList, const CImage& cur_gray,
	const float minimum_KLT_response, const unsigned int KLT_response_half_win,
	const unsigned int max_x, const unsigned int max_y)
{
	trackFeatures_checkResponses_impl_simple<TKeyPointfList>(
		featureList, cur_gray, minimum_KLT_response, KLT_response_half_win,
		max_x, max_y);
}

template <typename FEATLIST>
inline void trackFeatures_updatePatch(
	FEATLIST& featureList, const CImage& cur_gray);

template <>
inline void trackFeatures_updatePatch<CFeatureList>(
	CFeatureList& featureList, const CImage& cur_gray)
{
	for (auto& ft : featureList)
	{
		if (ft.track_status != status_TRACKED)
			continue;  // Skip if it's not correctly tracked.

		const size_t patch_width = ft.patch->getWidth();
		const size_t patch_height = ft.patch->getHeight();
		if (patch_width > 0 && patch_height > 0)
		{
			try
			{
				const int offset = (int)patch_width / 2;  // + 1;
				ft.patch.emplace();
				cur_gray.extract_patch(
					*ft.patch, round(ft.keypoint.pt.x) - offset,
					round(ft.keypoint.pt.y) - offset, patch_width,
					patch_height);
			}
			catch (std::exception&)
			{
				ft.track_status = status_OOB;  // Out of bounds!
			}
		}
	}
}  // end of trackFeatures_updatePatch<>
template <>
inline void trackFeatures_updatePatch<TKeyPointList>(
	[[maybe_unused]] TKeyPointList& featureList,
	[[maybe_unused]] const CImage& cur_gray)
{
	// This list type does not have patch stored explicitly
}  // end of trackFeatures_updatePatch<>
template <>
inline void trackFeatures_updatePatch<TKeyPointfList>(
	[[maybe_unused]] TKeyPointfList& featureList,
	[[maybe_unused]] const CImage& cur_gray)
{
	// This list type does not have patch stored explicitly
}  // end of trackFeatures_updatePatch<>

template <typename FEATLIST>
inline void trackFeatures_addNewFeats(
	FEATLIST& featureList, const TKeyPointList& new_feats,
	const std::vector<size_t>& sorted_indices, const size_t nNewToCheck,
	const size_t maxNumFeatures, const float minimum_KLT_response_to_add,
	const double threshold_sqr_dist_to_add_new, const size_t patchSize,
	const CImage& cur_gray, TFeatureID& max_feat_ID_at_input);

template <>
inline void trackFeatures_addNewFeats<CFeatureList>(
	CFeatureList& featureList, const TKeyPointList& new_feats,
	const std::vector<size_t>& sorted_indices, const size_t nNewToCheck,
	const size_t maxNumFeatures, const float minimum_KLT_response_to_add,
	const double threshold_sqr_dist_to_add_new, const size_t patchSize,
	const CImage& cur_gray, TFeatureID& max_feat_ID_at_input)
{
	const TImageSize imgSize = cur_gray.getSize();
	const int offset = (int)patchSize / 2 + 1;
	const int w_off = int(imgSize.x - offset);
	const int h_off = int(imgSize.y - offset);

	for (size_t i = 0; i < nNewToCheck && featureList.size() < maxNumFeatures;
		 i++)
	{
		const TKeyPoint& feat = new_feats[sorted_indices[i]];

		if (feat.response < minimum_KLT_response_to_add) continue;

		double min_dist_sqr = square(10000);

		if (!featureList.empty())
		{
			min_dist_sqr =
				featureList.kdTreeClosestPoint2DsqrError(feat.pt.x, feat.pt.y);
		}

		if (min_dist_sqr > threshold_sqr_dist_to_add_new &&
			feat.pt.x > offset && feat.pt.y > offset && feat.pt.x < w_off &&
			feat.pt.y < h_off)
		{
			// Add new feature:
			CFeature ft;
			ft.type = featFAST;
			ft.keypoint.ID = ++max_feat_ID_at_input;
			ft.keypoint.pt.x = feat.pt.x;
			ft.keypoint.pt.y = feat.pt.y;
			ft.response = feat.response;
			ft.orientation = 0;
			ft.keypoint.octave = 1;
			ft.patchSize = patchSize;  // The size of the feature patch

			// Image patch surronding the feature
			if (patchSize > 0)
			{
				ft.patch.emplace();
				cur_gray.extract_patch(
					*ft.patch, round(feat.pt.x) - offset,
					round(feat.pt.y) - offset, patchSize, patchSize);
			}

			featureList.emplace_back(std::move(ft));
		}
	}
}  // end of trackFeatures_addNewFeats<>

template <class FEAT_LIST>
inline void trackFeatures_addNewFeats_simple_list(
	FEAT_LIST& featureList, const TKeyPointList& new_feats,
	const std::vector<size_t>& sorted_indices, const size_t nNewToCheck,
	const size_t maxNumFeatures, const float minimum_KLT_response_to_add,
	const double threshold_sqr_dist_to_add_new,
	[[maybe_unused]] const size_t patchSize,
	[[maybe_unused]] const CImage& cur_gray, TFeatureID& max_feat_ID_at_input)
{
	// Version with KD-tree
	CFeatureListKDTree<typename FEAT_LIST::feature_t> kdtree(
		featureList.getVector());

	for (size_t i = 0; i < nNewToCheck && featureList.size() < maxNumFeatures;
		 i++)
	{
		const TKeyPoint& feat = new_feats[sorted_indices[i]];
		if (feat.response < minimum_KLT_response_to_add) break;  // continue;

		// Check the min-distance:
		double min_dist_sqr = std::numeric_limits<double>::max();

		if (!featureList.empty())
		{
			min_dist_sqr =
				kdtree.kdTreeClosestPoint2DsqrError(feat.pt.x, feat.pt.y);
		}

		if (min_dist_sqr > threshold_sqr_dist_to_add_new)
		{
			// OK: accept it
			featureList.emplace_back(feat.pt.x, feat.pt.y);
			kdtree.mark_as_outdated();

			// Fill out the rest of data:
			typename FEAT_LIST::feature_t& newFeat = featureList.back();

			newFeat.ID = ++max_feat_ID_at_input;
			newFeat.response = feat.response;
			newFeat.octave = 0;
			/** Inactive: right after detection, and before being tried to track
			 */
			newFeat.track_status = status_IDLE;
		}
	}
}  // end of trackFeatures_addNewFeats<>

template <>
inline void trackFeatures_addNewFeats<TKeyPointList>(
	TKeyPointList& featureList, const TKeyPointList& new_feats,
	const std::vector<size_t>& sorted_indices, const size_t nNewToCheck,
	const size_t maxNumFeatures, const float minimum_KLT_response_to_add,
	const double threshold_sqr_dist_to_add_new, const size_t patchSize,
	const CImage& cur_gray, TFeatureID& max_feat_ID_at_input)
{
	trackFeatures_addNewFeats_simple_list<TKeyPointList>(
		featureList, new_feats, sorted_indices, nNewToCheck, maxNumFeatures,
		minimum_KLT_response_to_add, threshold_sqr_dist_to_add_new, patchSize,
		cur_gray, max_feat_ID_at_input);
}
template <>
inline void trackFeatures_addNewFeats<TKeyPointfList>(
	TKeyPointfList& featureList, const TKeyPointList& new_feats,
	const std::vector<size_t>& sorted_indices, const size_t nNewToCheck,
	const size_t maxNumFeatures, const float minimum_KLT_response_to_add,
	const double threshold_sqr_dist_to_add_new, const size_t patchSize,
	const CImage& cur_gray, TFeatureID& max_feat_ID_at_input)
{
	trackFeatures_addNewFeats_simple_list<TKeyPointfList>(
		featureList, new_feats, sorted_indices, nNewToCheck, maxNumFeatures,
		minimum_KLT_response_to_add, threshold_sqr_dist_to_add_new, patchSize,
		cur_gray, max_feat_ID_at_input);
}

// Return the number of removed features
template <typename FEATLIST>
inline size_t trackFeatures_deleteOOB(
	FEATLIST& trackedFeats, const size_t img_width, const size_t img_height,
	const int MIN_DIST_MARGIN_TO_STOP_TRACKING);

template <typename FEATLIST>
inline size_t trackFeatures_deleteOOB_impl_simple_feat(
	FEATLIST& trackedFeats, const size_t img_width, const size_t img_height,
	const int MIN_DIST_MARGIN_TO_STOP_TRACKING)
{
	if (trackedFeats.empty()) return 0;

	std::vector<size_t> survival_idxs;
	const size_t N = trackedFeats.size();

	// 1st: Build list of survival indexes:
	survival_idxs.reserve(N);
	for (size_t i = 0; i < N; i++)
	{
		const typename FEATLIST::feature_t& ft = trackedFeats[i];
		const TFeatureTrackStatus status = ft.track_status;
		bool eras = (status_TRACKED != status && status_IDLE != status);
		if (!eras)
		{
			// Also, check if it's too close to the image border:
			const int x = ft.pt.x;
			const int y = ft.pt.y;
			if (x < MIN_DIST_MARGIN_TO_STOP_TRACKING ||
				y < MIN_DIST_MARGIN_TO_STOP_TRACKING ||
				x > static_cast<int>(
						img_width - MIN_DIST_MARGIN_TO_STOP_TRACKING) ||
				y > static_cast<int>(
						img_height - MIN_DIST_MARGIN_TO_STOP_TRACKING))
			{
				eras = true;
			}
		}
		if (!eras) survival_idxs.push_back(i);
	}

	// 2nd: Build updated list:
	const size_t N2 = survival_idxs.size();
	const size_t n_removed = N - N2;
	for (size_t i = 0; i < N2; i++)
	{
		if (survival_idxs[i] != i)
			trackedFeats[i] = trackedFeats[survival_idxs[i]];
	}
	trackedFeats.resize(N2);
	return n_removed;
}  // end of trackFeatures_deleteOOB

template <>
inline size_t trackFeatures_deleteOOB(
	TKeyPointList& trackedFeats, const size_t img_width,
	const size_t img_height, const int MIN_DIST_MARGIN_TO_STOP_TRACKING)
{
	return trackFeatures_deleteOOB_impl_simple_feat<TKeyPointList>(
		trackedFeats, img_width, img_height, MIN_DIST_MARGIN_TO_STOP_TRACKING);
}
template <>
inline size_t trackFeatures_deleteOOB(
	TKeyPointfList& trackedFeats, const size_t img_width,
	const size_t img_height, const int MIN_DIST_MARGIN_TO_STOP_TRACKING)
{
	return trackFeatures_deleteOOB_impl_simple_feat<TKeyPointfList>(
		trackedFeats, img_width, img_height, MIN_DIST_MARGIN_TO_STOP_TRACKING);
}

template <>
inline size_t trackFeatures_deleteOOB(
	CFeatureList& trackedFeats, const size_t img_width, const size_t img_height,
	const int MIN_DIST_MARGIN_TO_STOP_TRACKING)
{
	auto itFeat = trackedFeats.begin();
	size_t n_removed = 0;
	while (itFeat != trackedFeats.end())
	{
		const TFeatureTrackStatus status = itFeat->track_status;
		bool eras = (status_TRACKED != status && status_IDLE != status);
		if (!eras)
		{
			// Also, check if it's too close to the image border:
			const float x = itFeat->keypoint.pt.x;
			const float y = itFeat->keypoint.pt.y;
			if (x < MIN_DIST_MARGIN_TO_STOP_TRACKING ||
				y < MIN_DIST_MARGIN_TO_STOP_TRACKING ||
				x > (img_width - MIN_DIST_MARGIN_TO_STOP_TRACKING) ||
				y > (img_height - MIN_DIST_MARGIN_TO_STOP_TRACKING))
			{
				eras = true;
			}
		}
		if (eras)  // Erase or keep?
		{
			itFeat = trackedFeats.erase(itFeat);
			n_removed++;
		}
		else
			++itFeat;
	}
	return n_removed;
}  // end of trackFeatures_deleteOOB
}  // namespace mrpt::vision::detail
// ---------------------------- end of internal helper templates
// -------------------------------

void CGenericFeatureTracker::trackFeatures_impl(
	[[maybe_unused]] const CImage& old_img,
	[[maybe_unused]] const CImage& new_img,
	[[maybe_unused]] TKeyPointfList& inout_featureList)
{
	THROW_EXCEPTION("Method not implemented by derived class!");
}

/** Perform feature tracking from "old_img" to "new_img", with a (possibly
 *empty) list of previously tracked features "featureList".
 *  This is a list of parameters (in "extraParams") accepted by ALL
 *implementations of feature tracker (see each derived class for more specific
 *parameters).
 *		- "add_new_features" (Default=0). If set to "1", new features will be
 *also
 *added to the existing ones in areas of the image poor of features.
 * This method actually first call the pure virtual "trackFeatures_impl"
 *method, then implements the optional detection of new features if
 *"add_new_features"!=0.
 */
template <typename FEATLIST>
void CGenericFeatureTracker::internal_trackFeatures(
	const CImage& old_img, const CImage& new_img, FEATLIST& featureList)
{
	mrpt::system::CTimeLoggerEntry tleg(m_timlog, "CGenericFeatureTracker");

	const size_t img_width = new_img.getWidth();
	const size_t img_height = new_img.getHeight();

	// Take the maximum ID of "old" features so new feats (if
	// "add_new_features==true") will be id+1, id+2, ...
	TFeatureID max_feat_ID_at_input = 0;
	if (!featureList.empty()) max_feat_ID_at_input = featureList.getMaxID();

	// Grayscale images
	// =========================================
	m_timlog.enter("CGenericFeatureTracker.to_grayscale");

	const CImage prev_gray(old_img, FAST_REF_OR_CONVERT_TO_GRAY);
	const CImage cur_gray(new_img, FAST_REF_OR_CONVERT_TO_GRAY);

	m_timlog.leave("CGenericFeatureTracker.to_grayscale");

	// =================================
	// (1st STEP)  Do the actual tracking
	// =================================
	m_newly_detected_feats.clear();

	m_timlog.enter("CGenericFeatureTracker.trackFeatures_impl");

	trackFeatures_impl(prev_gray, cur_gray, featureList);

	m_timlog.leave("CGenericFeatureTracker.trackFeatures_impl");

	// ========================================================
	// (2nd STEP) For successfully followed features, check their KLT response??
	// ========================================================
	const int check_KLT_response_every =
		extra_params.getOrDefault<int>("check_KLT_response_every", 1);
	const float minimum_KLT_response =
		extra_params.getOrDefault<float>("minimum_KLT_response", 30.f);
	const unsigned int KLT_response_half_win =
		extra_params.getOrDefault<unsigned int>("KLT_response_half_win", 8U);

	if (check_KLT_response_every > 0 &&
		++m_check_KLT_counter >= size_t(check_KLT_response_every))
	{
		m_timlog.enter("CGenericFeatureTracker.check_KLT_responses");
		m_check_KLT_counter = 0;

		const unsigned int max_x = img_width - KLT_response_half_win;
		const unsigned int max_y = img_height - KLT_response_half_win;

		detail::trackFeatures_checkResponses(
			featureList, cur_gray, minimum_KLT_response, KLT_response_half_win,
			max_x, max_y);

		m_timlog.leave("CGenericFeatureTracker.check_KLT_responses");

	}  // end check_KLT_response_every

	// ============================================================
	// (3rd STEP)  Remove Out-of-bounds or badly tracked features
	//   or those marked as "bad" by their low KLT response
	// ============================================================
	const bool remove_lost_features =
		extra_params.getOrDefault<bool>("remove_lost_features", false);

	if (remove_lost_features)
	{
		m_timlog.enter("CGenericFeatureTracker.OOB_remove");

		static const int MIN_DIST_MARGIN_TO_STOP_TRACKING = 10;

		const size_t nRemoved = detail::trackFeatures_deleteOOB(
			featureList, img_width, img_height,
			MIN_DIST_MARGIN_TO_STOP_TRACKING);

		m_timlog.leave("CGenericFeatureTracker.OOB_remove");

		last_execution_extra_info.num_deleted_feats = nRemoved;
	}
	else
	{
		last_execution_extra_info.num_deleted_feats = 0;
	}

	// ========================================================
	// (4th STEP) For successfully followed features, update its patch:
	// ========================================================
	const int update_patches_every =
		extra_params.getOrDefault<int>("update_patches_every", 0);

	if (update_patches_every > 0 &&
		++m_update_patches_counter >= size_t(update_patches_every))
	{
		mrpt::system::CTimeLoggerEntry tle(
			m_timlog, "CGenericFeatureTracker.update_patches");

		m_update_patches_counter = 0;

		// Update the patch for each valid feature:
		detail::trackFeatures_updatePatch(featureList, cur_gray);

	}  // end if update_patches_every

	// ========================================================
	// (5th STEP) Do detection of new features??
	// ========================================================
	const bool add_new_features =
		extra_params.getOrDefault<bool>("add_new_features", true);
	const double threshold_dist_to_add_new =
		extra_params.getOrDefault<double>("add_new_feat_min_separation", 15.0);

	// Additional operation: if "add_new_features==true", find new features and
	// add them in areas spare of valid features:
	if (add_new_features)
	{
		mrpt::system::CTimeLoggerEntry tle(
			m_timlog, "CGenericFeatureTracker.add_new_features");

		// Look for new features and save in "m_newly_detected_feats", if
		// they're not already computed:
		if (m_newly_detected_feats.empty())
		{
			// Do the detection
			CFeatureExtraction fe;
			fe.options.featsType = featFAST;
			fe.options.FASTOptions.threshold = m_detector_adaptive_thres;

			CFeatureList new_feats;

			fe.detectFeatures(cur_gray, new_feats);

			m_newly_detected_feats.clear();
			m_newly_detected_feats.reserve(new_feats.size());

			for (const CFeature& f : new_feats)
			{
				m_newly_detected_feats.push_back(TKeyPoint(f.keypoint));
			}
		}

		// Extra out info.
		const size_t N = m_newly_detected_feats.size();
		last_execution_extra_info.raw_FAST_feats_detected = N;

		// Update the adaptive threshold.
		const size_t desired_num_features = extra_params.getOrDefault<size_t>(
			"desired_num_features_adapt",
			size_t((img_width * img_height) >> 9));
		updateAdaptiveNewFeatsThreshold(N, desired_num_features);

		// Use KLT response instead of the OpenCV's original "response" field:
		{
			const unsigned int max_x = img_width - KLT_response_half_win;
			const unsigned int max_y = img_height - KLT_response_half_win;
			for (size_t i = 0; i < N; i++)
			{
				const unsigned int x = m_newly_detected_feats[i].pt.x;
				const unsigned int y = m_newly_detected_feats[i].pt.y;
				if (x > KLT_response_half_win && y > KLT_response_half_win &&
					x < max_x && y < max_y)
					m_newly_detected_feats[i].response =
						cur_gray.KLT_response(x, y, KLT_response_half_win);
				else
					m_newly_detected_feats[i].response = 0;  // Out of bounds
			}
		}

		//  Sort them by "response": It's ~100 times faster to sort a list of
		//      indices "sorted_indices" than sorting directly the actual list
		//      of features "m_newly_detected_feats"
		std::vector<size_t> sorted_indices(N);
		for (size_t i = 0; i < N; i++) sorted_indices[i] = i;

		std::sort(
			sorted_indices.begin(), sorted_indices.end(),
			KeypointResponseSorter<TKeyPointList>(m_newly_detected_feats));

		// For each new good feature, add it to the list of tracked ones only if
		// it's pretty
		//  isolated:

		const size_t nNewToCheck = std::min(size_t(1500), N);
		const double threshold_sqr_dist_to_add_new =
			square(threshold_dist_to_add_new);
		const size_t maxNumFeatures = extra_params.getOrDefault<size_t>(
			"add_new_feat_max_features", 100U);
		const size_t patchSize =
			extra_params.getOrDefault<size_t>("add_new_feat_patch_size", 0U);
		const float minimum_KLT_response_to_add =
			extra_params.getOrDefault<float>(
				"minimum_KLT_response_to_add", 70.f);

		// Do it:
		detail::trackFeatures_addNewFeats(
			featureList, m_newly_detected_feats, sorted_indices, nNewToCheck,
			maxNumFeatures, minimum_KLT_response_to_add,
			threshold_sqr_dist_to_add_new, patchSize, cur_gray,
			max_feat_ID_at_input);
	}

}  // end of CGenericFeatureTracker::trackFeatures

void CGenericFeatureTracker::trackFeatures(
	const CImage& old_img, const CImage& new_img, TKeyPointList& featureList)
{
	internal_trackFeatures<TKeyPointList>(old_img, new_img, featureList);
}

void CGenericFeatureTracker::trackFeatures(
	const CImage& old_img, const CImage& new_img, TKeyPointfList& featureList)
{
	internal_trackFeatures<TKeyPointfList>(old_img, new_img, featureList);
}

void CGenericFeatureTracker::updateAdaptiveNewFeatsThreshold(
	const size_t nNewlyDetectedFeats, const size_t desired_num_features)
{
	const size_t hysteresis_min_num_feats = desired_num_features * 0.9;
	const size_t hysteresis_max_num_feats = desired_num_features * 1.1;

	if (nNewlyDetectedFeats < hysteresis_min_num_feats)
		m_detector_adaptive_thres = std::max(
			2.0, std::min(
					 m_detector_adaptive_thres - 1.0,
					 m_detector_adaptive_thres * 0.8));
	else if (nNewlyDetectedFeats > hysteresis_max_num_feats)
		m_detector_adaptive_thres = std::max(
			m_detector_adaptive_thres + 1.0, m_detector_adaptive_thres * 1.2);
}
