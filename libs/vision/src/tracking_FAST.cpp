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

#include <mrpt/math/KDTreeCapable.h>

#include "do_opencv_includes.h"

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

// Auxiliary KD-tree search class for vector<KeyPoint>:
#if MRPT_HAS_OPENCV
#if MRPT_OPENCV_VERSION_NUM >= 0x211
class CSimple2DKDTree : public mrpt::math::KDTreeCapable
{
public:
	const vector<cv::KeyPoint> & m_data;
	CSimple2DKDTree(const vector<cv::KeyPoint> & data) : m_data(data) {  }

protected:
	/** Must return the number of data points */
	virtual size_t kdtree_get_point_count() const {
		return m_data.size();
	}
	/** Must fill out the data points in "data", such as the i'th point will be stored in (data[i][0],...,data[i][nDims-1]). */
	virtual void kdtree_fill_point_data(ANNpointArray &data, const int nDims) const
	{
		const size_t N = m_data.size();
		for (size_t i=0;i<N;i++) {
			data[i][0] = m_data[i].pt.x;
			data[i][1] = m_data[i].pt.y;
		}
	}
};
#endif
#endif

/** Ctor  */
CFeatureTracker_FAST::CFeatureTracker_FAST(const mrpt::utils::TParametersDouble &extraParams) :
	CGenericFeatureTracker			( extraParams ),
	m_detector_adaptive_thres		( 10 ),
	m_hysteresis_min_num_feats		( 100 ),
	m_hysteresis_max_num_feats		( 300 )
{
}


/** Track a set of features from old_img -> new_img by patch correlation over the closest FAST features, using a KD-tree for looking closest correspondences.
*  Optional parameters that can be passed in "extra_params":
*		- "window_width"  (Default=15)
*		- "window_height" (Default=15)
*/
void CFeatureTracker_FAST::trackFeatures(
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
	// 4) Optionally: Add NEW features from the list of already detected ones.
	//
	// =======================================================================

	// =======================================================================
	// 1) Detect ALL available features in the new image.
	// =======================================================================
	vector<KeyPoint> cv_feats; // The opencv keypoint output vector

	FastFeatureDetector fastDetector( m_detector_adaptive_thres , true /* non-max supres. */ );

	{ // Create a temporary gray image, if needed:
		IplImage* img = reinterpret_cast<IplImage*>(new_img.getAsIplImage());
		IplImage* cGrey;

		if( img->nChannels == 1 )
			cGrey = img;										// Input image is already 'grayscale'
		else
		{
			cGrey = cvCreateImage( cvGetSize( img ), 8, 1);
			cvCvtColor( img, cGrey, CV_BGR2GRAY );				// Convert input image into 'grayscale'
		}

		const Mat theImg = cvarrToMat( cGrey );
		fastDetector.detect( theImg, cv_feats );  // Do the detection

		if( img->nChannels != 1 )
			cvReleaseImage( &cGrey );
	}

	const size_t N = cv_feats.size();  // # of detected feats.

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
	size_t max_query_results = 40;
	keep_min(max_query_results, N);

	printf("Raw feats: %u\n",(unsigned) N);

	// Maximum distance for ignoring a "close" potential match:
	const float max_sq_dist_to_check = square(window_width) + square(window_height);

	std::vector<float>  closest_xs(max_query_results), closest_ys(max_query_results);
	std::vector<float>  closest_dist_sqr(max_query_results);

	// For each old feature:
	for (size_t i=0;i<featureList.size();i++)
	{
		ASSERTDEB_(featureList[i].present())
		CFeaturePtr &feat = featureList[i];

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

			// This is a potential match: Check the patch to see if it matches:
			printf("testing %u against (%.02f,%.02f)\n",(unsigned)i, cv_feats[closest_idxs[k]].pt.x,cv_feats[closest_idxs[k]].pt.y );
			double match_quality = 0.9;
			potential_matches[match_quality] = closest_idxs[k];
		}

		// Keep the best match: Since std::map<> is ordered, the last is the best one:
		const size_t idxBestMatch = potential_matches.rbegin()->second;
	}

	printf("DONE!\n");


#else
	THROW_EXCEPTION("This function requires OpenCV >= 2.1.1")
#endif
#else
	THROW_EXCEPTION("MRPT has been compiled without OpenCV!")
#endif
	MRPT_END
}


