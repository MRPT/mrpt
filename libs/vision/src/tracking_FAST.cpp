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

#include <mrpt/vision.h>  // Precompiled headers


#include <mrpt/vision/tracking.h>
#include <mrpt/vision/CFeatureExtraction.h>

#include "do_opencv_includes.h"

#if MRPT_HAS_SSE2
#	include <mrpt/utils/SSE_types.h>
#endif

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;



#if MRPT_HAS_OPENCV
#if MRPT_OPENCV_VERSION_NUM >= 0x211

// Auxiliary function:
double match_template_SQDIFF(
	const IplImage* img,
	const IplImage* patch,
	const unsigned int patch_pos_x,
	const unsigned int patch_pos_y)
{
// Only for 8bit grayscale:
	ASSERTDEB_(img->depth==patch->depth && patch->depth==8)
	ASSERTDEB_(img->nChannels == 1 && patch->nChannels == 1 )
	ASSERTDEB_(patch_pos_x + patch->width<=(size_t)img->width)
	ASSERTDEB_(patch_pos_y + patch->height<=(size_t)img->height)

#if MRPT_HAS_SSE2 && 0 // TO DO
	//_mm_sad_epu8
	// See: http://software.intel.com/en-us/articles/motion-estimation-with-intel-streaming-simd-extensions-4-intel-sse4/
#else
	// Non-vectorized version:
	unsigned int sq_diff = 0;



	for (unsigned int y=0;y<(unsigned int)patch->height;y++)
	{
		const uint8_t *pixel_img   = reinterpret_cast<const uint8_t *>(img->imageData) + img->widthStep*(y+patch_pos_y) + patch_pos_x;
		const uint8_t *pixel_patch = reinterpret_cast<const uint8_t *>(patch->imageData) + patch->widthStep*y + 0;

		for (unsigned int x=patch->width;x!=0;x--)
			sq_diff += square( (*pixel_img++) - (*pixel_patch++) );
	}

	return 1.0 - sq_diff / (square(255.0)* patch->width *  patch->height);
#endif
}

#endif
#endif

/** Ctor  */
CFeatureTracker_FAST::CFeatureTracker_FAST(const mrpt::utils::TParametersDouble &extraParams) :
	CGenericFeatureTracker			( extraParams ),
	m_fast_detector_adaptive_thres	( 10 ),
	m_hysteresis_min_num_feats		( 2500 ),
	m_hysteresis_max_num_feats		( 3500 )
{
}


/** Track a set of features from old_img -> new_img by patch correlation over the closest FAST features, using a KD-tree for looking closest correspondences.
*  Optional parameters that can be passed in "extra_params":
*		- "window_width"  (Default=15)
*		- "window_height" (Default=15)
*/
void CFeatureTracker_FAST::trackFeatures_impl(
	const CImage &old_img,
	const CImage &new_img,
	vision::CFeatureList &featureList )
{
	MRPT_START
#if MRPT_HAS_OPENCV
#if MRPT_OPENCV_VERSION_NUM >= 0x211

	const unsigned int window_width = extra_params.getWithDefaultVal("window_width",15);
	const unsigned int window_height = extra_params.getWithDefaultVal("window_height",15);

	using namespace cv;

	// =======================================================================
	//                   OVERVIEW OF THE TRACKING ALGORITHM
	//
	// 1) Detect ALL available features in the new image.
	// 2) Update the adaptive threshold.
	// 3) For each old feature:
	//    3.A) Look for a set of closest ones among the new feats.
	//    3.B) Keep the one with the best patch similarity.
	// 4) Optionally: Add NEW features from the list of already detected ones,
	//     so we do tracking + new feats detection at once.
	//
	// =======================================================================

	// =======================================================================
	// 1) Detect ALL available features in the new image.
	// =======================================================================
	// Look for new features:
	//FastFeatureDetector fastDetector( m_fast_detector_adaptive_thres , true /* non-max supres. */ );
	// Create a temporary gray image, if needed:
	const CImage new_img_gray(new_img, FAST_REF_OR_CONVERT_TO_GRAY);

	// Do the detection
	m_timlog.enter("[CFeatureTracker_FAST::track]: detect FAST");

	MRPT_TODO("Reuse this list in the generic tracker while looking for new feats")

	TSimpleFeatureList new_feats;

	// Do the detection
	CFeatureExtraction::detectFeatures_SSE2_FASTER12(
		new_img_gray,
		new_feats,
		m_fast_detector_adaptive_thres );

	m_timlog.leave("[CFeatureTracker_FAST::track]: detect FAST");

#if 0
	{
		mrpt::utils::CImage  dbg_img;
		new_img_gray.colorImage(dbg_img);
		vector<TPoint2D> pts;
		for (size_t i=0;i<new_feats.size();i++) pts.push_back(TPoint2D(new_feats[i].pt.x,new_feats[i].pt.y));
		dbg_img.drawFeaturesSimple(pts);
		static int cnt = 1;
		dbg_img.saveToFile( format("dbg_%04i.jpg",cnt++) );
	}
#endif

	// # of detected feats.
	const size_t N = new_feats.size();
	//cout << "N: " << N << endl;

	last_execution_extra_info.raw_FAST_feats_detected = N; // Extra out info.

	// =======================================================================
	// 2) Update the adaptive threshold.
	// =======================================================================
	if (N<m_hysteresis_min_num_feats) 		m_fast_detector_adaptive_thres*=0.8;
	else if (N>m_hysteresis_max_num_feats)	m_fast_detector_adaptive_thres*=1.2;

	// =======================================================================
	// 3) For each old feature:
	//    3.A) Look for a set of closest ones among the new feats.
	// =======================================================================

	// # of closest features to look for:
	size_t max_query_results = 20;
	keep_min(max_query_results, N);

	// Maximum distance for ignoring a "close" potential match:
	const float max_sq_dist_to_check = square(window_width) + square(window_height);

	std::vector<float>  closest_xs(max_query_results), closest_ys(max_query_results);
	std::vector<float>  closest_dist_sqr(max_query_results);

	m_timlog.enter("[CFeatureTracker_FAST::track]: testMatchAll");

	// For each old feature:
	for (size_t i=0;i<featureList.size();i++)
	{
		ASSERTDEB_(featureList[i].present())
		CFeaturePtr &feat = featureList[i];

#if 0
	// CRITERIA: Similarity of the KLT value

		// Get the size of the patch so we know when we are too close to a border.

		const unsigned int KLT_win_half_size = 4;
		const unsigned int max_border_x = new_img_gray.getWidth() - KLT_win_half_size -1;
		const unsigned int max_border_y = new_img_gray.getHeight() - KLT_win_half_size -1;

		m_timlog.enter("[CFeatureTracker_FAST::track]: testMatch.kdtree");

		std::vector<size_t> closest_idxs =
		new_feats.kdTreeNClosestPoint2D(
			feat->x,feat->y,
			max_query_results,
			closest_xs,closest_ys,closest_dist_sqr);

		m_timlog.leave("[CFeatureTracker_FAST::track]: testMatch.kdtree");

		ASSERTDEB_(closest_idxs.size()==max_query_results)

		m_timlog.enter("[CFeatureTracker_FAST::track]: testMatch");

		// The result list is sorted in ascending distance, go thru it:
		std::map<double,size_t>  potential_matches;
		for (size_t k=0;k<max_query_results;k++)
		{
			if (closest_dist_sqr[k]>max_sq_dist_to_check)
				break; // Too far.

			const unsigned int px = new_feats[closest_idxs[k]].pt.x;
			const unsigned int py = new_feats[closest_idxs[k]].pt.y;

			if (px<=KLT_win_half_size ||
				py<=KLT_win_half_size ||
				px>=max_border_x ||
				py>=max_border_y)
			{
				// Can't match the entire patch with the image... what to do?
				// -> Discard it and mark as OOB:
				feat->track_status = status_OOB;
				break;
			}

			// This is a potential match: Check the KLT value to see if it matches:
			const float new_klt = new_img_gray.KLT_response(px,py,KLT_win_half_size);

			potential_matches[ std::abs(new_klt-feat->response) ] = closest_idxs[k];
		}

		m_timlog.leave("[CFeatureTracker_FAST::track]: testMatch");

		const size_t idxBestMatch = potential_matches.empty() ?
			std::string::npos
			:
			potential_matches.begin()->second;

#else
	// CRITERIA: Patch matching:

		// Get the size of the patch so we know when we are too close to a border.
		ASSERTDEB_(feat->patch.getWidth()>1)
		ASSERTDEB_(feat->patch.getWidth()==feat->patch.getHeight())

		const unsigned int patch_half_size = (feat->patch.getWidth()-1)>>1;
		const unsigned int max_border_x = new_img_gray.getWidth() - patch_half_size -1;
		const unsigned int max_border_y = new_img_gray.getHeight() - patch_half_size -1;

		m_timlog.enter("[CFeatureTracker_FAST::track]: testMatch.kdtree");

		std::vector<size_t> closest_idxs =
		new_feats.kdTreeNClosestPoint2D(
			feat->x,feat->y,
			max_query_results,
			closest_xs,closest_ys,closest_dist_sqr);

		m_timlog.leave("[CFeatureTracker_FAST::track]: testMatch.kdtree");

		ASSERTDEB_(closest_idxs.size()==max_query_results)

		m_timlog.enter("[CFeatureTracker_FAST::track]: testMatch");

		// The result list is sorted in ascending distance, go thru it:
		std::map<double,size_t>  potential_matches;
		for (size_t k=0;k<max_query_results;k++)
		{
			if (closest_dist_sqr[k]>max_sq_dist_to_check)
				break; // Too far.

			const unsigned int px = new_feats[closest_idxs[k]].pt.x;
			const unsigned int py = new_feats[closest_idxs[k]].pt.y;

			if (px<=patch_half_size ||
				py<=patch_half_size ||
				px>=max_border_x ||
				py>=max_border_y)
			{
				// Can't match the entire patch with the image... what to do?
				// -> Discard it and mark as OOB:
				feat->track_status = status_OOB;
				break;
			}

			// This is a potential match: Check the patch to see if it matches:
			//printf("testing %u against (%u,%u)\n",(unsigned)i, px,py );

			// Possible algorithms: { TM_SQDIFF=0, TM_SQDIFF_NORMED=1, TM_CCORR=2, TM_CCORR_NORMED=3, TM_CCOEFF=4, TM_CCOEFF_NORMED=5 };

			double match_quality  = match_template_SQDIFF(
				reinterpret_cast<IplImage*>( new_img_gray.getAsIplImage()),
				reinterpret_cast<IplImage*>( feat->patch.getAsIplImage()),
				px-patch_half_size-1,
				py-patch_half_size-1);

			potential_matches[match_quality] = closest_idxs[k];
		}

		m_timlog.leave("[CFeatureTracker_FAST::track]: testMatch");

		// Keep the best match: Since std::map<> is ordered, the last is the best one:
		const size_t idxBestMatch = potential_matches.empty() ?
			std::string::npos
			:
			potential_matches.rbegin()->second;
#endif

		if (!potential_matches.empty())
		{
			// OK: Accept it:
			feat->track_status	= status_TRACKED;
			feat->x				= new_feats[idxBestMatch].pt.x;
			feat->y				= new_feats[idxBestMatch].pt.y;
		}
		else
		{
			// No potential match!
			// Mark as "lost":
			feat->track_status = status_LOST;
		}
	}
	m_timlog.leave("[CFeatureTracker_FAST::track]: testMatchAll");

#else
	THROW_EXCEPTION("This function requires OpenCV >= 2.1.1")
#endif
#else
	THROW_EXCEPTION("MRPT has been compiled without OpenCV!")
#endif
	MRPT_END
}


