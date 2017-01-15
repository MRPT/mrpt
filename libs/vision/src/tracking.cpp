/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers


#include <mrpt/vision/tracking.h>

#include <mrpt/vision/CFeatureExtraction.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 


using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

// ------------------------------- internal helper templates ---------------------------------
namespace mrpt {
	namespace vision {
		namespace detail {

			template <typename FEATLIST>
			inline void trackFeatures_checkResponses(FEATLIST &featureList,const CImage &cur_gray,const float minimum_KLT_response,const unsigned int KLT_response_half_win,const unsigned int max_x, const unsigned int max_y);

			template <>
			inline void trackFeatures_checkResponses<CFeatureList>(CFeatureList &featureList,const CImage &cur_gray,const float minimum_KLT_response,const unsigned int KLT_response_half_win,const unsigned int max_x, const unsigned int max_y)
			{
				const CFeatureList::iterator itFeatEnd = featureList.end();
				for (CFeatureList::iterator itFeat = featureList.begin(); itFeat!=itFeatEnd ;  ++itFeat)
				{
					CFeature* ft = itFeat->pointer();
					if (ft->track_status!=status_TRACKED)
						continue; // Skip if it's not correctly tracked.

					const unsigned int x = ft->x;
					const unsigned int y = ft->y;
					if (x>KLT_response_half_win && y>KLT_response_half_win && x<max_x && y<max_y)
					{	// Update response:
						ft->response = cur_gray.KLT_response(x,y,KLT_response_half_win);

						// Is it good enough? http://grooveshark.com/s/Goonies+Are+Good+Enough/2beBfO?src=5
						if (ft->response<minimum_KLT_response)
						{	// Nope!
							ft->track_status = status_LOST;
						}
					}
					else
					{	// Out of bounds
						ft->response = 0;
						ft->track_status = status_OOB;
					}
				}
			} // end of trackFeatures_checkResponses<>

			template <class FEAT_LIST>
			inline void trackFeatures_checkResponses_impl_simple(FEAT_LIST &featureList,const CImage &cur_gray,const float minimum_KLT_response,const unsigned int KLT_response_half_win,const unsigned int max_x_, const unsigned int max_y_)
			{
				if (featureList.empty()) return;

				typedef typename FEAT_LIST::feature_t::pixel_coord_t pixel_coord_t;
				const pixel_coord_t half_win = static_cast<pixel_coord_t>(KLT_response_half_win);
				const pixel_coord_t max_x = static_cast<pixel_coord_t>(max_x_);
				const pixel_coord_t max_y = static_cast<pixel_coord_t>(max_y_);

				for (int N = featureList.size()-1; N>=0 ; --N)
				{
					typename FEAT_LIST::feature_t & ft = featureList[N];
					if (ft.track_status!=status_TRACKED)
						continue; // Skip if it's not correctly tracked.

					if (ft.pt.x>half_win && ft.pt.y>half_win && ft.pt.x<max_x && ft.pt.y<max_y)
					{	// Update response:
						ft.response = cur_gray.KLT_response(ft.pt.x,ft.pt.y,KLT_response_half_win);

						// Is it good enough? http://grooveshark.com/s/Goonies+Are+Good+Enough/2beBfO?src=5
						if (ft.response<minimum_KLT_response)
						{	// Nope!
							ft.track_status = status_LOST;
						}
					}
					else
					{	// Out of bounds
						ft.response = 0;
						ft.track_status = status_OOB;
					}
				}
			} // end of trackFeatures_checkResponses<>

			template <>
			inline void trackFeatures_checkResponses<TSimpleFeatureList>(TSimpleFeatureList &featureList,const CImage &cur_gray,const float minimum_KLT_response,const unsigned int KLT_response_half_win,const unsigned int max_x, const unsigned int max_y)
			{
				trackFeatures_checkResponses_impl_simple<TSimpleFeatureList>(featureList,cur_gray,minimum_KLT_response,KLT_response_half_win,max_x,max_y);
			}
			template <>
			inline void trackFeatures_checkResponses<TSimpleFeaturefList>(TSimpleFeaturefList &featureList,const CImage &cur_gray,const float minimum_KLT_response,const unsigned int KLT_response_half_win,const unsigned int max_x, const unsigned int max_y)
			{
				trackFeatures_checkResponses_impl_simple<TSimpleFeaturefList>(featureList,cur_gray,minimum_KLT_response,KLT_response_half_win,max_x,max_y);
			}


			template <typename FEATLIST>
			inline void trackFeatures_updatePatch(FEATLIST &featureList,const CImage &cur_gray);

			template <>
			inline void trackFeatures_updatePatch<CFeatureList>(CFeatureList &featureList,const CImage &cur_gray)
			{
				for (CFeatureList::iterator itFeat = featureList.begin(); itFeat != featureList.end();  ++itFeat)
				{
					CFeature* ft = itFeat->pointer();
					if (ft->track_status!=status_TRACKED)
						continue; // Skip if it's not correctly tracked.

					const size_t patch_width  = ft->patch.getWidth();
					const size_t patch_height = ft->patch.getHeight();
					if (patch_width>0 && patch_height>0)
					{
						try
						{
							const int offset = (int)patch_width/2; // + 1;
							cur_gray.extract_patch(
								ft->patch,
								round( ft->x ) - offset,
								round( ft->y ) - offset,
								patch_width,
								patch_height );
						}
						catch (std::exception &)
						{
							ft->track_status = status_OOB; // Out of bounds!
						}
					}
				}
			} // end of trackFeatures_updatePatch<>
			template <>
			inline void trackFeatures_updatePatch<TSimpleFeatureList>(TSimpleFeatureList &featureList,const CImage &cur_gray)
			{
				MRPT_UNUSED_PARAM(featureList); MRPT_UNUSED_PARAM(cur_gray);
				// This list type does not have patch stored explicitly
			} // end of trackFeatures_updatePatch<>
			template <>
			inline void trackFeatures_updatePatch<TSimpleFeaturefList>(TSimpleFeaturefList &featureList,const CImage &cur_gray)
			{
				MRPT_UNUSED_PARAM(featureList); MRPT_UNUSED_PARAM(cur_gray);
				// This list type does not have patch stored explicitly
			} // end of trackFeatures_updatePatch<>

			template <typename FEATLIST>
			inline void trackFeatures_addNewFeats(FEATLIST &featureList,const TSimpleFeatureList &new_feats, const std::vector<size_t> &sorted_indices, const size_t nNewToCheck,const size_t maxNumFeatures, const float minimum_KLT_response_to_add,const double threshold_sqr_dist_to_add_new,const size_t patchSize,const CImage &cur_gray, TFeatureID  &max_feat_ID_at_input);

			template <>
			inline void trackFeatures_addNewFeats<CFeatureList>(CFeatureList &featureList,const TSimpleFeatureList &new_feats, const std::vector<size_t> &sorted_indices, const size_t nNewToCheck,const size_t maxNumFeatures,const float minimum_KLT_response_to_add,const double threshold_sqr_dist_to_add_new,const size_t patchSize,const CImage &cur_gray, TFeatureID  &max_feat_ID_at_input)
			{
				const TImageSize imgSize = cur_gray.getSize();
				const int 	 offset		= (int)patchSize/2 + 1;
				const int    w_off      = int(imgSize.x - offset);
				const int    h_off      = int(imgSize.y - offset);

				for (size_t i=0;i<nNewToCheck && featureList.size()<maxNumFeatures;i++)
				{
					const TSimpleFeature &feat = new_feats[ sorted_indices[i] ];

					if (feat.response<minimum_KLT_response_to_add) continue;

					double min_dist_sqr = square(10000);

					if (!featureList.empty())
					{
						//m_timlog.enter("[CGenericFeatureTracker] add new features.kdtree");
						min_dist_sqr = featureList.kdTreeClosestPoint2DsqrError(feat.pt.x,feat.pt.y );
						//m_timlog.leave("[CGenericFeatureTracker] add new features.kdtree");
					}

					if (min_dist_sqr>threshold_sqr_dist_to_add_new &&
						feat.pt.x > offset &&
						feat.pt.y > offset &&
						feat.pt.x < w_off &&
						feat.pt.y < h_off )
					{
						// Add new feature:
						CFeaturePtr ft		= CFeature::Create();
						ft->type			= featFAST;
						ft->ID				= ++max_feat_ID_at_input;
						ft->x				= feat.pt.x;
						ft->y				= feat.pt.y;
						ft->response		= feat.response;
						ft->orientation		= 0;
						ft->scale			= 1;
						ft->patchSize		= patchSize;		// The size of the feature patch

						if( patchSize > 0 )
							cur_gray.extract_patch(
								ft->patch,
								round( ft->x ) - offset,
								round( ft->y ) - offset,
								patchSize,
								patchSize );						// Image patch surronding the feature

						featureList.push_back( ft );
					}
				}
			} // end of trackFeatures_addNewFeats<>

			template <class FEAT_LIST>
			inline void trackFeatures_addNewFeats_simple_list(FEAT_LIST &featureList,const TSimpleFeatureList &new_feats, const std::vector<size_t> &sorted_indices, const size_t nNewToCheck,const size_t maxNumFeatures,const float minimum_KLT_response_to_add,const double threshold_sqr_dist_to_add_new,const size_t patchSize,const CImage &cur_gray, TFeatureID  &max_feat_ID_at_input)
			{
#if 0
				// Brute-force version:
				const int max_manhatan_dist = std::sqrt(2*threshold_sqr_dist_to_add_new);

				for (size_t i=0;i<nNewToCheck && featureList.size()<maxNumFeatures;i++)
				{
					const TSimpleFeature &feat = new_feats[ sorted_indices[i] ];
					if (feat.response<minimum_KLT_response_to_add) break; // continue;

					// Check the min-distance:
					int manh_dist = std::numeric_limits<int>::max();
					for (size_t j=0;j<featureList.size();j++)
					{
						const TSimpleFeature &existing = featureList[j];
						const int d = std::abs(existing.pt.x-feat.pt.x)+std::abs(existing.pt.y-feat.pt.y);
						mrpt::utils::keep_min(manh_dist, d);
					}

					if (manh_dist<max_manhatan_dist)
						continue; // Already occupied! skip.

					// OK: accept it
					featureList.push_back_fast(feat.pt.x,feat.pt.y);  // (x,y)
					//featureList.mark_kdtree_as_outdated();

					// Fill out the rest of data:
					TSimpleFeature &newFeat = featureList.back();

					newFeat.ID			= ++max_feat_ID_at_input;
					newFeat.response	= feat.response;
					newFeat.octave		= 0;
					newFeat.track_status = status_IDLE;  //!< Inactive: right after detection, and before being tried to track
				}
#elif 0
				// Version with an occupancy grid:
				const int grid_cell_log2 = round( std::log(std::sqrt(threshold_sqr_dist_to_add_new)*0.5)/std::log(2.0));

				int grid_lx = 1+(cur_gray.getWidth() >> grid_cell_log2);
				int grid_ly = 1+(cur_gray.getHeight()>> grid_cell_log2);

				mrpt::math::CMatrixBool & occupied_sections = featureList.getOccupiedSectionsMatrix();

				occupied_sections.setSize(grid_lx,grid_ly);  // See the comments above for an explanation.
				occupied_sections.fillAll(false);

				for (size_t i=0;i<featureList.size();i++)
				{
					const TSimpleFeature &feat = featureList[i];
					const int section_idx_x = feat.pt.x >> grid_cell_log2;
					const int section_idx_y = feat.pt.y >> grid_cell_log2;

					if (!section_idx_x || !section_idx_y || section_idx_x>=grid_lx-1 || section_idx_y>=grid_ly-1)
						continue; // This may be too radical, but speeds up the logic below...

					// Mark sections as occupied
					bool *ptr1 = &occupied_sections.get_unsafe(section_idx_x-1,section_idx_y-1);
					bool *ptr2 = &occupied_sections.get_unsafe(section_idx_x-1,section_idx_y  );
					bool *ptr3 = &occupied_sections.get_unsafe(section_idx_x-1,section_idx_y+1);
					ptr1[0]=ptr1[1]=ptr1[2]=true;
					ptr2[0]=ptr2[1]=ptr2[2]=true;
					ptr3[0]=ptr3[1]=ptr3[2]=true;
				}

				for (size_t i=0;i<nNewToCheck && featureList.size()<maxNumFeatures;i++)
				{
					const TSimpleFeature &feat = new_feats[ sorted_indices[i] ];
					if (feat.response<minimum_KLT_response_to_add) break; // continue;

					// Check the min-distance:
					const int section_idx_x = feat.pt.x >> grid_cell_log2;
					const int section_idx_y = feat.pt.y >> grid_cell_log2;

					if (!section_idx_x || !section_idx_y || section_idx_x>=grid_lx-2 || section_idx_y>=grid_ly-2)
						continue; // This may be too radical, but speeds up the logic below...

					if (occupied_sections(section_idx_x,section_idx_y))
						continue; // Already occupied! skip.

					// Mark section as occupied
					bool *ptr1 = &occupied_sections.get_unsafe(section_idx_x-1,section_idx_y-1);
					bool *ptr2 = &occupied_sections.get_unsafe(section_idx_x-1,section_idx_y  );
					bool *ptr3 = &occupied_sections.get_unsafe(section_idx_x-1,section_idx_y+1);

					ptr1[0]=ptr1[1]=ptr1[2]=true;
					ptr2[0]=ptr2[1]=ptr2[2]=true;
					ptr3[0]=ptr3[1]=ptr3[2]=true;

					// OK: accept it
					featureList.push_back_fast(feat.pt.x,feat.pt.y);  // (x,y)
					//featureList.mark_kdtree_as_outdated();

					// Fill out the rest of data:
					TSimpleFeature &newFeat = featureList.back();

					newFeat.ID			= ++max_feat_ID_at_input;
					newFeat.response	= feat.response;
					newFeat.octave		= 0;
					newFeat.track_status = status_IDLE;  //!< Inactive: right after detection, and before being tried to track
				}
#else
				MRPT_UNUSED_PARAM(patchSize); MRPT_UNUSED_PARAM(cur_gray);
				// Version with KD-tree
				CFeatureListKDTree<typename FEAT_LIST::feature_t>  kdtree(featureList.getVector());


				for (size_t i=0;i<nNewToCheck && featureList.size()<maxNumFeatures;i++)
				{
					const TSimpleFeature &feat = new_feats[ sorted_indices[i] ];
					if (feat.response<minimum_KLT_response_to_add) break; // continue;

					// Check the min-distance:
					double min_dist_sqr = std::numeric_limits<double>::max();

					if (!featureList.empty())
					{
						//m_timlog.enter("[CGenericFeatureTracker] add new features.kdtree");
						min_dist_sqr = kdtree.kdTreeClosestPoint2DsqrError(feat.pt.x,feat.pt.y );
						//m_timlog.leave("[CGenericFeatureTracker] add new features.kdtree");
					}

					if (min_dist_sqr>threshold_sqr_dist_to_add_new)
					{
						// OK: accept it
						featureList.push_back_fast(feat.pt.x,feat.pt.y);  // (x,y)
						kdtree.mark_as_outdated();

						// Fill out the rest of data:
						typename FEAT_LIST::feature_t &newFeat = featureList.back();

						newFeat.ID			= ++max_feat_ID_at_input;
						newFeat.response	= feat.response;
						newFeat.octave		= 0;
						newFeat.track_status = status_IDLE;  //!< Inactive: right after detection, and before being tried to track
					}
				}

#endif
			} // end of trackFeatures_addNewFeats<>

			template <>
			inline void trackFeatures_addNewFeats<TSimpleFeatureList>(TSimpleFeatureList &featureList,const TSimpleFeatureList &new_feats, const std::vector<size_t> &sorted_indices, const size_t nNewToCheck,const size_t maxNumFeatures,const float minimum_KLT_response_to_add,const double threshold_sqr_dist_to_add_new,const size_t patchSize,const CImage &cur_gray, TFeatureID  &max_feat_ID_at_input)
			{
				trackFeatures_addNewFeats_simple_list<TSimpleFeatureList>(featureList,new_feats,sorted_indices,nNewToCheck,maxNumFeatures,minimum_KLT_response_to_add,threshold_sqr_dist_to_add_new,patchSize,cur_gray, max_feat_ID_at_input);
			}
			template <>
			inline void trackFeatures_addNewFeats<TSimpleFeaturefList>(TSimpleFeaturefList &featureList,const TSimpleFeatureList &new_feats, const std::vector<size_t> &sorted_indices, const size_t nNewToCheck,const size_t maxNumFeatures,const float minimum_KLT_response_to_add,const double threshold_sqr_dist_to_add_new,const size_t patchSize,const CImage &cur_gray, TFeatureID  &max_feat_ID_at_input)
			{
				trackFeatures_addNewFeats_simple_list<TSimpleFeaturefList>(featureList,new_feats,sorted_indices,nNewToCheck,maxNumFeatures,minimum_KLT_response_to_add,threshold_sqr_dist_to_add_new,patchSize,cur_gray, max_feat_ID_at_input);
			}


			// Return the number of removed features
			template <typename FEATLIST>
			inline size_t trackFeatures_deleteOOB(
				FEATLIST &trackedFeats,
				const size_t img_width, const size_t img_height,
				const int MIN_DIST_MARGIN_TO_STOP_TRACKING);

			template <typename FEATLIST>
			inline size_t trackFeatures_deleteOOB_impl_simple_feat(
				FEATLIST &trackedFeats,
				const size_t img_width, const size_t img_height,
				const int MIN_DIST_MARGIN_TO_STOP_TRACKING)
			{
				if (trackedFeats.empty()) return 0;

				std::vector<size_t> survival_idxs;
				const size_t N = trackedFeats.size();

				// 1st: Build list of survival indexes:
				survival_idxs.reserve(N);
				for (size_t i=0;i<N;i++)
				{
					const typename FEATLIST::feature_t &ft = trackedFeats[i];
					const TFeatureTrackStatus status = ft.track_status;
					bool eras = (status_TRACKED!=status && status_IDLE!=status);
					if (!eras)
					{
						// Also, check if it's too close to the image border:
						const int x= ft.pt.x;
						const int y= ft.pt.y;
						if (x<MIN_DIST_MARGIN_TO_STOP_TRACKING  || y<MIN_DIST_MARGIN_TO_STOP_TRACKING ||
							x>static_cast<int>(img_width-MIN_DIST_MARGIN_TO_STOP_TRACKING) ||
							y>static_cast<int>(img_height-MIN_DIST_MARGIN_TO_STOP_TRACKING))
						{
							eras = true;
						}
					}
					if (!eras) survival_idxs.push_back(i);
				}

				// 2nd: Build updated list:
				const size_t N2 = survival_idxs.size();
				const size_t n_removed = N-N2;
				for (size_t i=0;i<N2;i++)
				{
					if (survival_idxs[i]!=i)
						trackedFeats[i] = trackedFeats[ survival_idxs[i] ];
				}
				trackedFeats.resize(N2);
				return n_removed;
			} // end of trackFeatures_deleteOOB

			template <>
			inline size_t trackFeatures_deleteOOB(
				TSimpleFeatureList &trackedFeats,
				const size_t img_width, const size_t img_height,
				const int MIN_DIST_MARGIN_TO_STOP_TRACKING)
			{
				return trackFeatures_deleteOOB_impl_simple_feat<TSimpleFeatureList>(trackedFeats,img_width,img_height,MIN_DIST_MARGIN_TO_STOP_TRACKING);
			}
			template <>
			inline size_t trackFeatures_deleteOOB(
				TSimpleFeaturefList &trackedFeats,
				const size_t img_width, const size_t img_height,
				const int MIN_DIST_MARGIN_TO_STOP_TRACKING)
			{
				return trackFeatures_deleteOOB_impl_simple_feat<TSimpleFeaturefList>(trackedFeats,img_width,img_height,MIN_DIST_MARGIN_TO_STOP_TRACKING);
			}

			template <>
			inline size_t trackFeatures_deleteOOB(
				CFeatureList &trackedFeats,
				const size_t img_width, const size_t img_height,
				const int MIN_DIST_MARGIN_TO_STOP_TRACKING)
			{
				MRPT_UNUSED_PARAM(MIN_DIST_MARGIN_TO_STOP_TRACKING);
				CFeatureList::iterator itFeat = trackedFeats.begin();
				size_t n_removed = 0;
				while (itFeat!=trackedFeats.end())
				{
					const TFeatureTrackStatus status = (*itFeat)->track_status;
					bool eras = (status_TRACKED!=status && status_IDLE!=status);
					if (!eras)
					{
						// Also, check if it's too close to the image border:
						const float x= (*itFeat)->x;
						const float y= (*itFeat)->y;
						static const float MIN_DIST_MARGIN_TO_STOP_TRACKING = 10;
						if (x<MIN_DIST_MARGIN_TO_STOP_TRACKING  || y<MIN_DIST_MARGIN_TO_STOP_TRACKING ||
							x>(img_width-MIN_DIST_MARGIN_TO_STOP_TRACKING) ||
							y>(img_height-MIN_DIST_MARGIN_TO_STOP_TRACKING))
						{
							eras = true;
						}
					}
					if (eras)	// Erase or keep?
					{
						itFeat = trackedFeats.erase(itFeat);
						n_removed++;
					}
					else ++itFeat;
				}
				return n_removed;
			} // end of trackFeatures_deleteOOB


		}
	}
} // end NS's
// ---------------------------- end of internal helper templates -------------------------------


void CGenericFeatureTracker::trackFeatures_impl(const CImage &old_img,const CImage &new_img,TSimpleFeaturefList  &inout_featureList )
{
	MRPT_UNUSED_PARAM(old_img); MRPT_UNUSED_PARAM(new_img); MRPT_UNUSED_PARAM(inout_featureList);
	THROW_EXCEPTION("Method not implemented by derived class!")
}


/** Perform feature tracking from "old_img" to "new_img", with a (possibly empty) list of previously tracked features "featureList".
  *  This is a list of parameters (in "extraParams") accepted by ALL implementations of feature tracker (see each derived class for more specific parameters).
  *		- "add_new_features" (Default=0). If set to "1", new features will be also added to the existing ones in areas of the image poor of features.
  * This method actually first call the pure virtual "trackFeatures_impl" method, then implements the optional detection of new features if "add_new_features"!=0.
  */
template <typename FEATLIST>
void CGenericFeatureTracker::internal_trackFeatures(
	const CImage &old_img,
	const CImage &new_img,
	FEATLIST &featureList )
{
	m_timlog.enter("[CGenericFeatureTracker::trackFeatures] Complete iteration");

	const size_t  img_width  = new_img.getWidth();
	const size_t  img_height = new_img.getHeight();

	// Take the maximum ID of "old" features so new feats (if "add_new_features==true") will be id+1, id+2, ...
	TFeatureID  max_feat_ID_at_input = 0;
	if (!featureList.empty())
		max_feat_ID_at_input = featureList.getMaxID();

	// Grayscale images
	// =========================================
	m_timlog.enter("[CGenericFeatureTracker] Convert grayscale");

	const CImage prev_gray(old_img, FAST_REF_OR_CONVERT_TO_GRAY);
	const CImage cur_gray(new_img, FAST_REF_OR_CONVERT_TO_GRAY);

	m_timlog.leave("[CGenericFeatureTracker] Convert grayscale");

	// =================================
	// (1st STEP)  Do the actual tracking
	// =================================
	m_newly_detected_feats.clear();

	m_timlog.enter("[CGenericFeatureTracker] trackFeatures_impl");

	trackFeatures_impl(prev_gray,cur_gray,featureList);

	m_timlog.leave("[CGenericFeatureTracker] trackFeatures_impl");


	// ========================================================
	// (2nd STEP) For successfully followed features, check their KLT response??
	// ========================================================
	const int	check_KLT_response_every = extra_params.getWithDefaultVal("check_KLT_response_every",0);
	const float minimum_KLT_response = extra_params.getWithDefaultVal("minimum_KLT_response",5);
	const unsigned int KLT_response_half_win = extra_params.getWithDefaultVal("KLT_response_half_win",4);

	if (check_KLT_response_every>0 && ++m_check_KLT_counter>=size_t(check_KLT_response_every))
	{
		m_timlog.enter("[CGenericFeatureTracker] check KLT responses");
		m_check_KLT_counter = 0;

		const unsigned int max_x = img_width  - KLT_response_half_win;
		const unsigned int max_y = img_height - KLT_response_half_win;

		detail::trackFeatures_checkResponses(featureList,cur_gray,minimum_KLT_response,KLT_response_half_win,max_x,max_y);

		m_timlog.leave("[CGenericFeatureTracker] check KLT responses");

	} // end check_KLT_response_every


	// ============================================================
	// (3rd STEP)  Remove Out-of-bounds or badly tracked features
	//   or those marked as "bad" by their low KLT response
	// ============================================================
	const bool   remove_lost_features = extra_params.getWithDefaultVal("remove_lost_features",0)!=0;

	if (remove_lost_features)
	{
		m_timlog.enter("[CGenericFeatureTracker] removal of OOB");

		static const int MIN_DIST_MARGIN_TO_STOP_TRACKING = 10;

		const size_t nRemoved = detail::trackFeatures_deleteOOB(
			featureList,
			img_width, img_height,
			MIN_DIST_MARGIN_TO_STOP_TRACKING);

		m_timlog.leave("[CGenericFeatureTracker] removal of OOB");

		last_execution_extra_info.num_deleted_feats = nRemoved;
	}
	else
	{
		last_execution_extra_info.num_deleted_feats = 0;
	}


	// ========================================================
	// (4th STEP) For successfully followed features, update its patch:
	// ========================================================
	const int update_patches_every = extra_params.getWithDefaultVal("update_patches_every",0);

	if (update_patches_every>0 && ++m_update_patches_counter>=size_t(update_patches_every))
	{
		m_timlog.enter("[CGenericFeatureTracker] update patches");
		m_update_patches_counter = 0;

		// Update the patch for each valid feature:
		detail::trackFeatures_updatePatch(featureList,cur_gray);

		m_timlog.leave("[CGenericFeatureTracker] update patches");
	} // end if update_patches_every

	// ========================================================
	// (5th STEP) Do detection of new features??
	// ========================================================
	const bool   add_new_features = extra_params.getWithDefaultVal("add_new_features",0)!=0;
	const double threshold_dist_to_add_new = extra_params.getWithDefaultVal("add_new_feat_min_separation",15);

	// Additional operation: if "add_new_features==true", find new features and add them in
	//  areas spare of valid features:
	if (add_new_features)
	{
		m_timlog.enter("[CGenericFeatureTracker] add new features");

		// Look for new features and save in "m_newly_detected_feats", if they're not already computed:
		if (m_newly_detected_feats.empty())
		{
			// Do the detection
			CFeatureExtraction::detectFeatures_SSE2_FASTER12(
				cur_gray,
				m_newly_detected_feats,
				m_detector_adaptive_thres );
		}

		const size_t N = m_newly_detected_feats.size();

		last_execution_extra_info.raw_FAST_feats_detected = N; // Extra out info.

		// Update the adaptive threshold.
		const size_t desired_num_features = extra_params.getWithDefaultVal("desired_num_features_adapt", size_t( (img_width*img_height)>>9 ) );
		updateAdaptiveNewFeatsThreshold(N,desired_num_features);

		// Use KLT response instead of the OpenCV's original "response" field:
		{
			const unsigned int max_x = img_width-KLT_response_half_win;
			const unsigned int max_y = img_height-KLT_response_half_win;
			for (size_t i=0;i<N;i++)
			{
				const unsigned int x = m_newly_detected_feats[i].pt.x;
				const unsigned int y = m_newly_detected_feats[i].pt.y;
				if (x>KLT_response_half_win && y>KLT_response_half_win && x<max_x && y<max_y)
						m_newly_detected_feats[i].response = cur_gray.KLT_response(x,y,KLT_response_half_win);
				else	m_newly_detected_feats[i].response = 0; // Out of bounds
			}
		}

		//  Sort them by "response": It's ~100 times faster to sort a list of
		//      indices "sorted_indices" than sorting directly the actual list of features "m_newly_detected_feats"
		std::vector<size_t> sorted_indices(N);
		for (size_t i=0;i<N;i++)  sorted_indices[i]=i;

		std::sort( sorted_indices.begin(), sorted_indices.end(), KeypointResponseSorter<TSimpleFeatureList>(m_newly_detected_feats) );

		// For each new good feature, add it to the list of tracked ones only if it's pretty
		//  isolated:

		const size_t nNewToCheck = std::min( size_t(1500), N );
		const double threshold_sqr_dist_to_add_new = square(threshold_dist_to_add_new);
		const size_t maxNumFeatures = extra_params.getWithDefaultVal("add_new_feat_max_features",100);
		const size_t patchSize = extra_params.getWithDefaultVal("add_new_feat_patch_size",11);

		const float minimum_KLT_response_to_add = extra_params.getWithDefaultVal("minimum_KLT_response_to_add",10);

		// Do it:
		detail::trackFeatures_addNewFeats(featureList,m_newly_detected_feats,sorted_indices,nNewToCheck,maxNumFeatures,minimum_KLT_response_to_add,threshold_sqr_dist_to_add_new,patchSize,cur_gray,max_feat_ID_at_input);

		m_timlog.leave("[CGenericFeatureTracker] add new features");
	}

	m_timlog.leave("[CGenericFeatureTracker::trackFeatures] Complete iteration");

} // end of CGenericFeatureTracker::trackFeatures


void CGenericFeatureTracker::trackFeatures(
	const CImage &old_img,
	const CImage &new_img,
	CFeatureList &featureList )
{
	internal_trackFeatures<CFeatureList>(old_img,new_img,featureList);
}

void CGenericFeatureTracker::trackFeatures(
	const CImage &old_img,
	const CImage &new_img,
	TSimpleFeatureList &featureList )
{
	internal_trackFeatures<TSimpleFeatureList>(old_img,new_img,featureList);
}

void CGenericFeatureTracker::trackFeatures(
	const CImage &old_img,
	const CImage &new_img,
	TSimpleFeaturefList &featureList )
{
	internal_trackFeatures<TSimpleFeaturefList>(old_img,new_img,featureList);
}

void CGenericFeatureTracker::updateAdaptiveNewFeatsThreshold(
	const size_t nNewlyDetectedFeats,
	const size_t desired_num_features)
{
	const size_t hysteresis_min_num_feats = desired_num_features * 0.9;
	const size_t hysteresis_max_num_feats = desired_num_features * 1.1;

	if (nNewlyDetectedFeats<hysteresis_min_num_feats)
		m_detector_adaptive_thres = std::max(2.0,std::min(m_detector_adaptive_thres-1.0, m_detector_adaptive_thres*0.8));
	else if (nNewlyDetectedFeats>hysteresis_max_num_feats)
		m_detector_adaptive_thres = std::max(m_detector_adaptive_thres+1.0, m_detector_adaptive_thres*1.2);
}


/*------------------------------------------------------------
					checkTrackedFeatures
-------------------------------------------------------------*/
void vision::checkTrackedFeatures( CFeatureList &leftList,
							    CFeatureList &rightList,
								vision::TMatchingOptions options)
{
	ASSERT_( leftList.size() == rightList.size() );

	//std::cout << std::endl << "Tracked features checking ..." << std::endl;

	CFeatureList::iterator	itLeft, itRight;
	size_t					u,v;
	double					res;

	for( itLeft = leftList.begin(), itRight = rightList.begin(); itLeft != leftList.end(); )
	{
		bool delFeat = false;
		if( (*itLeft)->x < 0 || (*itLeft)->y < 0 ||							// Out of bounds
			(*itRight)->x < 0 || (*itRight)->y < 0 ||						// Out of bounds
			fabs( (*itLeft)->y - (*itRight)->y ) > options.epipolar_TH )	// Not fulfillment of the epipolar constraint
		{
			// Show reason
			std::cout << "Bad tracked match:";
			if( (*itLeft)->x < 0 || (*itLeft)->y < 0 || (*itRight)->x < 0 || (*itRight)->y < 0 )
				std::cout << " Out of bounds: (" << (*itLeft)->x << "," << (*itLeft)->y << " & (" << (*itRight)->x << "," << (*itRight)->y << ")" << std::endl;

			if( fabs( (*itLeft)->y - (*itRight)->y ) > options.epipolar_TH )
				std::cout << " Bad row checking: " << fabs( (*itLeft)->y - (*itRight)->y ) << std::endl;

			delFeat = true;
		}
		else
		{
			// Compute cross correlation:
			openCV_cross_correlation( (*itLeft)->patch, (*itRight)->patch, u, v, res );

			if( res < options.minCC_TH )
			{
				std::cout << "Bad tracked match (correlation failed):" << " CC Value: " << res << std::endl;
				delFeat = true;
			}
		} // end if

		if( delFeat ) // Erase the pair of features
		{
			itLeft = leftList.erase( itLeft );
			itRight = rightList.erase( itRight );
		}
		else
		{
			itLeft++;
			itRight++;
		}
	} // end for
} // end checkTrackedFeatures




/*-------------------------------------------------------------
					filterBadCorrsByDistance
-------------------------------------------------------------*/
void  vision::filterBadCorrsByDistance( TMatchingPairList &feat_list, unsigned int numberOfSigmas )
{
	ASSERT_( numberOfSigmas > 0 );
	//	MRPT_UNUSED_PARAM( numberOfSigmas );
	MRPT_START

	TMatchingPairList::iterator	itPair;
	CMatrix	dist;
	double  v_mean, v_std;
	unsigned int count = 0;

	dist.setSize( feat_list.size(), 1 );
	//v_mean.resize(1);
	//v_std.resize(1);

	// Compute mean and standard deviation of the distance
	for( itPair = feat_list.begin(); itPair != feat_list.end() ; itPair++, count++ ) {
		//cout << "(" << itPair->other_x << "," << itPair->other_y << "," << itPair->this_z << ")" << "- (" << itPair->this_x << "," << itPair->this_y << "," << itPair->other_z << "): ";
		//cout << sqrt( square( itPair->other_x - itPair->this_x ) + square( itPair->other_y - itPair->this_y ) + square( itPair->other_z - itPair->this_z ) ) << endl;
		dist( count, 0 ) = sqrt( square( itPair->other_x - itPair->this_x ) + square( itPair->other_y - itPair->this_y ) + square( itPair->other_z - itPair->this_z ) );
	}

	dist.meanAndStdAll( v_mean, v_std );

	cout << endl << "*****************************************************" << endl;
	cout << "Mean: " << v_mean << " - STD: " << v_std << endl;
	cout << endl << "*****************************************************" << endl;

	// Filter out bad points
	unsigned int idx = 0;
	//for( int idx = (int)feat_list.size()-1; idx >= 0; idx-- )
	for( itPair = feat_list.begin(); itPair != feat_list.end(); idx++)
	{
		//if( dist( idx, 0 ) > 1.2 )
		if( fabs( dist(idx,0) - v_mean ) > v_std*numberOfSigmas )
		{
			cout << "Outlier deleted: " << dist( idx, 0 ) << " vs " << v_std*numberOfSigmas << endl;
			itPair = feat_list.erase( itPair );
		}
		else
			itPair++;
	}

	MRPT_END
} // end filterBadCorrsByDistance
