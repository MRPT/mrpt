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

#include <mrpt/vision.h>  // Precompiled headers


#include <mrpt/vision/tracking.h>
#include <mrpt/vision/CFeatureExtraction.h>

#include "do_opencv_includes.h"

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
	unsigned int sq_diff = 0;

	// Only for 8bit grayscale:
	ASSERT_(img->depth==patch->depth && patch->depth==8)
	ASSERT_(img->nChannels == 1 && patch->nChannels == 1 )
	ASSERT_(patch_pos_x + patch->width<=(size_t)img->width)
	ASSERT_(patch_pos_y + patch->height<=(size_t)img->height)

	for (unsigned int y=0;y<(unsigned int)patch->height;y++)
	{
		const uint8_t *pixel_img   = reinterpret_cast<const uint8_t *>(img->imageData) + img->widthStep*(y+patch_pos_y) + patch_pos_x;
		const uint8_t *pixel_patch = reinterpret_cast<const uint8_t *>(patch->imageData) + patch->widthStep*y + 0;

		for (unsigned int x=patch->width;x!=0;x--)
			sq_diff += mrpt::square( (*pixel_img++) - (*pixel_patch++) );
	}

	return 1.0 - sq_diff / (square(255.0)* patch->width *  patch->height);
}

#endif
#endif

/** Ctor  */
CFeatureTracker_FAST::CFeatureTracker_FAST(const mrpt::utils::TParametersDouble &extraParams) :
	CGenericFeatureTracker			( extraParams ),
	m_detector_adaptive_thres		( 10 ),
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
	vector<KeyPoint> cv_feats; // The opencv keypoint output vector

	FastFeatureDetector fastDetector( m_detector_adaptive_thres , true /* non-max supres. */ );

	// Create a temporary gray image, if needed:
	mrpt::utils::CImage  new_img_gray(UNINITIALIZED_IMAGE);
	if (new_img.isColor())
			new_img.grayscale(new_img_gray);	// Create a new auxiliary grayscale image
	else	new_img_gray.setFromImageReadOnly( new_img );  // Copy the IPLImage, but do not own the memory

	// Do the detection
	const Mat new_img_gray_mat = cvarrToMat( reinterpret_cast<IplImage*>(new_img_gray.getAsIplImage()) );
	fastDetector.detect( new_img_gray_mat, cv_feats );

#if 0
	{
		mrpt::utils::CImage  dbg_img;
		new_img_gray.colorImage(dbg_img);
		vector<TPoint2D> pts;
		for (size_t i=0;i<cv_feats.size();i++) pts.push_back(TPoint2D(cv_feats[i].pt.x,cv_feats[i].pt.y));
		dbg_img.drawFeaturesSimple(pts);
		static int cnt = 1;
		dbg_img.saveToFile( format("dbg_%04i.jpg",cnt++) );
	}
#endif

	// # of detected feats.
	const size_t N = cv_feats.size();
	//cout << "N: " << N << endl;

	last_execution_extra_info.raw_FAST_feats_detected = N; // Extra out info.

	// =======================================================================
	// 2) Update the adaptive threshold.
	// =======================================================================
	if (N<m_hysteresis_min_num_feats) 		m_detector_adaptive_thres*=0.8;
	else if (N>m_hysteresis_max_num_feats)	m_detector_adaptive_thres*=1.2;

	// =======================================================================
	// 3) For each old feature:
	//    3.A) Look for a set of closest ones among the new feats.
	// =======================================================================

	// Build the KD-tree of these new feats:
	CSimple2DKDTree  kdtree(cv_feats);

	// # of closest features to look for:
	size_t max_query_results = 100;
	keep_min(max_query_results, N);

	// Maximum distance for ignoring a "close" potential match:
	const float max_sq_dist_to_check = square(window_width) + square(window_height);

	std::vector<float>  closest_xs(max_query_results), closest_ys(max_query_results);
	std::vector<float>  closest_dist_sqr(max_query_results);

	// For each old feature:
	for (size_t i=0;i<featureList.size();i++)
	{
		ASSERTDEB_(featureList[i].present())
		CFeaturePtr &feat = featureList[i];

		// Get the size of the patch so we know when we are too close to a border.
		ASSERTDEB_(feat->patch.getWidth()>1)
		ASSERTDEB_(feat->patch.getWidth()==feat->patch.getHeight())

		const unsigned int patch_half_size = (feat->patch.getWidth()-1)>>1;
		const unsigned int max_border_x = new_img_gray.getWidth() - patch_half_size -1;
		const unsigned int max_border_y = new_img_gray.getHeight() - patch_half_size -1;

		std::vector<size_t> closest_idxs =
		kdtree.kdTreeNClosestPoint2D(
			feat->x,feat->y,
			max_query_results,
			closest_xs,closest_ys,closest_dist_sqr);

		ASSERTDEB_(closest_idxs.size()==max_query_results)

		// The result list is sorted in ascending distance, go thru it:
		std::map<double,size_t>  potential_matches;
		for (size_t k=0;k<max_query_results;k++)
		{
			if (closest_dist_sqr[k]>max_sq_dist_to_check)
				break; // Too far.

			const unsigned int px = cv_feats[closest_idxs[k]].pt.x;
			const unsigned int py = cv_feats[closest_idxs[k]].pt.y;

			if (px<patch_half_size ||
				py<patch_half_size ||
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

		if (!potential_matches.empty())
		{
			// Keep the best match: Since std::map<> is ordered, the last is the best one:
			const double valBestMatch = potential_matches.rbegin()->first;
			const size_t idxBestMatch = potential_matches.rbegin()->second;

			// OK: Accept it:
			feat->track_status	= status_TRACKED;
			feat->x				= cv_feats[idxBestMatch].pt.x;
			feat->y				= cv_feats[idxBestMatch].pt.y;
		}
		else
		{
			// No potential match!
			// Mark as "lost":
			feat->track_status = status_LOST;
		}
	}

#else
	THROW_EXCEPTION("This function requires OpenCV >= 2.1.1")
#endif
#else
	THROW_EXCEPTION("MRPT has been compiled without OpenCV!")
#endif
	MRPT_END
}


