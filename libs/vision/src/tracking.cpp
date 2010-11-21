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
using namespace std;

/** Perform feature tracking from "old_img" to "new_img", with a (possibly empty) list of previously tracked features "featureList".
  *  This is a list of parameters (in "extraParams") accepted by ALL implementations of feature tracker (see each derived class for more specific parameters).
  *		- "add_new_features" (Default=0). If set to "1", new features will be also added to the existing ones in areas of the image poor of features.
  * This method actually first call the pure virtual "trackFeatures_impl" method, then implements the optional detection of new features if "add_new_features"!=0.
  */
void CGenericFeatureTracker::trackFeatures(
	const CImage &old_img,
	const CImage &new_img,
	vision::CFeatureList &featureList )
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

		for (CFeatureList::iterator itFeat = featureList.begin(); itFeat != featureList.end();  ++itFeat)
		{
			CFeature* ft = itFeat->pointer();
			if (ft->track_status!=status_TRACKED)
				continue; // Skip if it's not correctly tracked.

			const unsigned int x = ft->x;
			const unsigned int y = ft->y;
			if (x>KLT_response_half_win && y>KLT_response_half_win && x<max_x && y<max_y)
			{	// Update response:
				ft->response = cur_gray.KLT_response(x,y,KLT_response_half_win);

				// Is it good enough? http://www.youtube.com/watch?v=5kMi9tvuuZY
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

		m_timlog.leave("[CGenericFeatureTracker] check KLT responses");

	} // end check_KLT_response_every


	// ========================================================
	// (3rd STEP) For successfully followed features, update its patch:
	// ========================================================
	const int update_patches_every = extra_params.getWithDefaultVal("update_patches_every",0);

	if (update_patches_every>0 && ++m_update_patches_counter>=size_t(update_patches_every))
	{
		m_timlog.enter("[CGenericFeatureTracker] update patches");
		m_update_patches_counter = 0;

		// Update the patch for each valid feature:
		for (CFeatureList::iterator itFeat = featureList.begin(); itFeat != featureList.end();  ++itFeat)
		{
			CFeature* ft = itFeat->pointer();
			if (ft->track_status!=status_TRACKED)
				continue; // Skip if it's not correctly tracked.

			const size_t patch_width  = ft->patch.getWidth();
			const size_t patch_height = ft->patch.getHeight();
			if (patch_width>0 && patch_height>0)
			{
				const int offset = (int)patch_width/2; // + 1;
				try
				{
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

		m_timlog.leave("[CGenericFeatureTracker] update patches");
	} // end if update_patches_every

	// ========================================================
	// (4th STEP) Do detection of new features??
	// ========================================================
	const bool   add_new_features = extra_params.getWithDefaultVal("add_new_features",0)!=0;
	const double threshold_dist_to_add_new = extra_params.getWithDefaultVal("add_new_feat_min_separation",15);

	// Additional operation: if "add_new_features==true", find new features and add them in
	//  areas spare of valid features:
	if (add_new_features)
	{
		m_timlog.enter("[CGenericFeatureTracker] add new features");

#if MRPT_OPENCV_VERSION_NUM >= 0x210
		using namespace cv;

		// Look for new features:
		vector<KeyPoint> new_feats; // The opencv keypoint output vector

		// Do the detection
# if MRPT_OPENCV_VERSION_NUM >= 0x211
	// Modern version:
	FastFeatureDetector fastDetector( m_detector_adaptive_thres, true /* non-max supres. */ );
	const Mat new_img_gray_mat = cvarrToMat( cur_gray.getAs<IplImage>() );
	fastDetector.detect( new_img_gray_mat, new_feats );
# elif MRPT_OPENCV_VERSION_NUM >= 0x210
	// Older version:
	FAST(cur_gray.getAs<IplImage>(), new_feats, m_detector_adaptive_thres, true /* non-max supres. */ );
# endif

		const size_t N = new_feats.size();

		// Update the adaptive threshold.
		const size_t desired_num_feats = (img_width >> 5) * (img_height >> 5);
		const size_t hysteresis_min_num_feats = desired_num_feats * 0.9;
		const size_t hysteresis_max_num_feats = desired_num_feats * 1.1;

		if (N<hysteresis_min_num_feats) 		m_detector_adaptive_thres = std::max(2.0,std::min(m_detector_adaptive_thres-1.0, m_detector_adaptive_thres*0.8));
		else if (N>hysteresis_max_num_feats)	m_detector_adaptive_thres = std::max(m_detector_adaptive_thres+1.0, m_detector_adaptive_thres*1.2);

		// Use KLT response instead of the OpenCV's original "response" field:
		{
			const unsigned int max_x = img_width-KLT_response_half_win;
			const unsigned int max_y = img_height-KLT_response_half_win;
			for (size_t i=0;i<N;i++)
			{
				const unsigned int x = new_feats[i].pt.x;
				const unsigned int y = new_feats[i].pt.y;
				if (x>KLT_response_half_win && y>KLT_response_half_win && x<max_x && y<max_y)
						new_feats[i].response = cur_gray.KLT_response(x,y,KLT_response_half_win);
				else	new_feats[i].response = 0; // Out of bounds
			}
		}

		//  Sort them by "response": It's ~100 times faster to sort a list of
		//      indices "sorted_indices" than sorting directly the actual list of features "cv_feats"
		std::vector<size_t> sorted_indices(N);
		for (size_t i=0;i<N;i++)  sorted_indices[i]=i;
		std::sort( sorted_indices.begin(), sorted_indices.end(), KeypointCompCache(new_feats) );

		// For each new good feature, add it to the list of tracked ones only if it's pretty
		//  isolated:

		const size_t nNewToCheck = std::min( size_t(1500), N );
		const double threshold_sqr_dist_to_add_new = square(threshold_dist_to_add_new);
		const size_t maxNumFeatures = extra_params.getWithDefaultVal("add_new_feat_max_features",100);
		const size_t patchSize = extra_params.getWithDefaultVal("add_new_feat_patch_size",11);
		const int 	 offset		= (int)patchSize/2 + 1;

		const float minimum_KLT_response_to_add = extra_params.getWithDefaultVal("minimum_KLT_response_to_add",10);

		for (size_t i=0;i<nNewToCheck && featureList.size()<maxNumFeatures;i++)
		{
			const KeyPoint &kp = new_feats[sorted_indices[i]];

			if (kp.response<minimum_KLT_response_to_add) continue;

			double min_dist_sqr = square(10000);

			if (!featureList.empty())
				min_dist_sqr = featureList.kdTreeClosestPoint2DsqrError(kp.pt.x,kp.pt.y );

			if (min_dist_sqr>threshold_sqr_dist_to_add_new &&
				kp.pt.x > offset &&
				kp.pt.y > offset &&
				kp.pt.x < img_width - offset &&
				kp.pt.y < img_height - offset )
			{
				// Add new feature:
				CFeaturePtr ft		= CFeature::Create();
				ft->type			= featFAST;
				ft->ID				= ++max_feat_ID_at_input;
				ft->x				= kp.pt.x;
				ft->y				= kp.pt.y;
				ft->response		= kp.response;
				ft->orientation		= kp.angle;
				ft->scale			= kp.octave;
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
#else
	THROW_EXCEPTION("add_new_features requires OpenCV >=2.1.0")
#endif //MRPT_OPENCV_VERSION_NUM

		m_timlog.leave("[CGenericFeatureTracker] add new features");
	}

	m_timlog.leave("[CGenericFeatureTracker::trackFeatures] Complete iteration");

} // end of CGenericFeatureTracker::trackFeatures


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



/*------------------------------------------------------------
					trackFeatures
-------------------------------------------------------------*/
void  vision::trackFeatures2(
	const CImage &inImg1,
	const CImage &inImg2,
	CFeatureList &featureList,
	const unsigned int window_width,
	const unsigned int window_height)
{
MRPT_START;
#if MRPT_HAS_OPENCV

	CFeatureExtraction	fExt;
	CFeatureList		auxList;
	CMatchedFeatureList mList;

	//CDisplayWindow		w1, w2;
	//w1.showImageAndPoints( inImg1, featureList );

	fExt.options.featsType = featFAST;
	fExt.detectFeatures( inImg2, auxList, 0, 200 );
	//w2.showImageAndPoints( inImg2, auxList );

	TMatchingOptions	opts;
	opts.matching_method		= TMatchingOptions::mmSAD;
	opts.useEpipolarRestriction = false;
	matchFeatures( featureList, auxList, mList, opts );

	featureList.resize( mList.size() );
	CMatchedFeatureList::iterator	itmList;
	CFeatureList::iterator			itFeat;

	for( itFeat = featureList.begin(), itmList = mList.begin(); itFeat != featureList.end(); ++itFeat )
		(*itFeat) = itmList->second;

#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif
MRPT_END;
} // end trackFeatures2



/*-------------------------------------------------------------
					filterBadCorrsByDistance
-------------------------------------------------------------*/
void  vision::filterBadCorrsByDistance( TMatchingPairList &feat_list, unsigned int numberOfSigmas )
{
	ASSERT_( numberOfSigmas > 0 );
	//	MRPT_UNUSED_PARAM( numberOfSigmas );
	MRPT_START;

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

	MRPT_END;
} // end filterBadCorrsByDistance

