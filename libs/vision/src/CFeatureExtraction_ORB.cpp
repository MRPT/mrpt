/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/vision/CFeatureExtraction.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::vision;
using namespace mrpt::system;
using namespace std;

/************************************************************************************************
*								extractFeaturesORB												*
************************************************************************************************/
void  CFeatureExtraction::extractFeaturesORB(
	const mrpt::utils::CImage	& inImg,
	CFeatureList			    & feats,
	const unsigned int			init_ID,
	const unsigned int			nDesiredFeatures,
	const TImageROI			    & ROI )  const
{
	MRPT_UNUSED_PARAM(ROI);
	MRPT_START

#if MRPT_HAS_OPENCV
#	if MRPT_OPENCV_VERSION_NUM < 0x240
		THROW_EXCEPTION("This function requires OpenCV > 2.4.0")
#	else

	using namespace cv;

	vector<KeyPoint> cv_feats; // OpenCV keypoint output vector
	Mat              cv_descs; // OpenCV descriptor output

	const bool use_precomputed_feats = feats.size() > 0;

	if( use_precomputed_feats )
	{
		cv_feats.resize( feats.size() );
		for( size_t k = 0; k < cv_feats.size(); ++k )
		{
			cv_feats[k].pt.x = feats[k]->x;
			cv_feats[k].pt.y = feats[k]->y;
		}
	}
	
	// Make sure we operate on a gray-scale version of the image:
	const CImage inImg_gray( inImg, FAST_REF_OR_CONVERT_TO_GRAY );
	const Mat cvImg = cv::cvarrToMat( inImg_gray.getAs<IplImage>() );

	// The detector and descriptor
#	if MRPT_OPENCV_VERSION_NUM < 0x300
	Ptr<Feature2D> orb = Algorithm::create<Feature2D>("Feature2D.ORB");
	orb->operator()( cvImg, Mat(), cv_feats, cv_descs, use_precomputed_feats );
#else
	const size_t n_feats_2_extract = nDesiredFeatures == 0 ? 1000 : 3*nDesiredFeatures;
	Ptr<cv::ORB> orb = cv::ORB::create( n_feats_2_extract, options.ORBOptions.scale_factor, options.ORBOptions.n_levels );
	orb->detectAndCompute(cvImg, Mat(), cv_feats, cv_descs, use_precomputed_feats );
#endif
	
	const size_t n_feats = cv_feats.size();

	// if we had input features, just convert cv_feats to CFeatures and return
	const unsigned int patch_size_2	= options.patchSize/2;
	unsigned int f_id				= init_ID;
	if( use_precomputed_feats )
	{ 
		for( size_t k = 0; k < n_feats; ++k )
		{
			feats[k]->descriptors.ORB.resize( cv_descs.cols );
			for( int m = 0; m < cv_descs.cols; ++m )
				feats[k]->descriptors.ORB[m] = cv_descs.at<uchar>(k,m);		// to do: memcopy

			/*
			feats[k].response	= cv_feats[k].response;
			feats[k].scale		= cv_feats[k].size;
			feats[k].angle		= cv_feats[k].orientation;
			feats[k].ID			= f_id++;
			*/
			feats[k]->type		= featORB;

			if( options.ORBOptions.extract_patch && options.patchSize > 0 )
			{
				inImg.extract_patch(
				feats[k]->patch,
				round( feats[k]->x ) - patch_size_2,
				round( feats[k]->y ) - patch_size_2,
				options.patchSize,
				options.patchSize );
			}
		}
		return;
	} 

	//  1) Sort the fearues by "response": It's ~100 times faster to sort a list of
	//      indices "sorted_indices" than sorting directly the actual list of features "cv_feats"
	std::vector<size_t> sorted_indices(n_feats);
	for (size_t i=0;i<n_feats;i++)  sorted_indices[i]=i;
	std::sort( sorted_indices.begin(), sorted_indices.end(), KeypointResponseSorter< vector<KeyPoint> >(cv_feats) );

	//  2) Filter by "min-distance" (in options.ORBOptions.min_distance)
	//  3) Convert to MRPT CFeatureList format.
	// Steps 2 & 3 are done together in the while() below.
	// The "min-distance" filter is done by means of a 2D binary matrix where each cell is marked when one
	// feature falls within it. This is not exactly the same than a pure "min-distance" but is pretty close
	// and for large numbers of features is much faster than brute force search of kd-trees.
	// (An intermediate approach would be the creation of a mask image updated for each accepted feature, etc.)

	const bool do_filter_min_dist = options.ORBOptions.min_distance > 1;
	const unsigned int occupied_grid_cell_size = options.ORBOptions.min_distance/2.0;
	const float occupied_grid_cell_size_inv = 1.0f/occupied_grid_cell_size;

	unsigned int grid_lx = !do_filter_min_dist ? 1 : (unsigned int)(1 + inImg.getWidth() * occupied_grid_cell_size_inv);
	unsigned int grid_ly = !do_filter_min_dist ? 1 : (unsigned int)(1 + inImg.getHeight() * occupied_grid_cell_size_inv );

	mrpt::math::CMatrixBool occupied_sections(grid_lx,grid_ly);  // See the comments above for an explanation.
	occupied_sections.fillAll(false);

	const size_t n_max_feats = nDesiredFeatures > 0 ? std::min(size_t(nDesiredFeatures),n_feats) : n_feats;

	if( !options.addNewFeatures )
        feats.clear();
	// feats.reserve( feats.size() + n_max_feats );

	const size_t imgH = inImg.getHeight();
	const size_t imgW = inImg.getWidth();
	size_t k = 0;
	size_t c_feats = 0;
	while( c_feats < n_max_feats && k < n_feats )
	{
		const size_t idx = sorted_indices[k++];
		const KeyPoint & kp = cv_feats[ idx ];
		if( options.ORBOptions.extract_patch && options.patchSize > 0 )
		{
			// check image boundaries for extracting the patch
			const int xBorderInf = (int)floor( kp.pt.x - patch_size_2 );
			const int xBorderSup = (int)floor( kp.pt.x + patch_size_2 );
			const int yBorderInf = (int)floor( kp.pt.y - patch_size_2 );
			const int yBorderSup = (int)floor( kp.pt.y + patch_size_2 );

			if (!( xBorderSup < (int)imgW && xBorderInf > 0 && yBorderSup < (int)imgH && yBorderInf > 0 ))
				continue; // nope, skip.
		}

		if (do_filter_min_dist)
		{
			// Check the min-distance:
			const size_t section_idx_x = size_t(kp.pt.x * occupied_grid_cell_size_inv);
			const size_t section_idx_y = size_t(kp.pt.y * occupied_grid_cell_size_inv);

			if (occupied_sections(section_idx_x,section_idx_y))
				continue; // Already occupied! skip.

			// Mark section as occupied
			occupied_sections.set_unsafe(section_idx_x,section_idx_y, true);
			if (section_idx_x>0)			occupied_sections.set_unsafe(section_idx_x-1,section_idx_y, true);
			if (section_idx_y>0)			occupied_sections.set_unsafe(section_idx_x,section_idx_y-1, true);
			if (section_idx_x<grid_lx-1)	occupied_sections.set_unsafe(section_idx_x+1,section_idx_y, true);
			if (section_idx_y<grid_ly-1)	occupied_sections.set_unsafe(section_idx_x,section_idx_y+1, true);
		}

		// All tests passed: add new feature:
		CFeaturePtr ft		= CFeature::Create();
		ft->type			= featORB;
		ft->ID				= f_id++;
		ft->x				= kp.pt.x;
		ft->y				= kp.pt.y;
		ft->response		= kp.response;
		ft->orientation		= kp.angle;
		ft->scale			= kp.octave;
		ft->patchSize		= 0;

		// descriptor
		ft->descriptors.ORB.resize( cv_descs.cols );
		for( int m = 0; m < cv_descs.cols; ++m )
			ft->descriptors.ORB[m] = cv_descs.at<uchar>(idx,m);

		if( options.ORBOptions.extract_patch && options.patchSize > 0 )
		{
			ft->patchSize	= options.patchSize;		// The size of the feature patch

			inImg.extract_patch(
				ft->patch,
				round( ft->x ) - patch_size_2,
				round( ft->y ) - patch_size_2,
				options.patchSize,
				options.patchSize );					// Image patch surronding the feature
		}
	
		feats.push_back( ft );
		c_feats++;
	}
#	endif
#endif
	MRPT_END
}

void CFeatureExtraction::internal_computeORBDescriptors( 
	const CImage	& in_img,
	CFeatureList	& in_features) const
{
#if MRPT_HAS_OPENCV
#	if MRPT_OPENCV_VERSION_NUM < 0x240
		THROW_EXCEPTION("This function requires OpenCV > 2.4.0")
#	else
	using namespace cv;

	const size_t n_feats = in_features.size(); 
	const CImage inImg_gray( in_img, FAST_REF_OR_CONVERT_TO_GRAY );

	// convert from CFeatureList to vector<KeyPoint>
	vector<KeyPoint> cv_feats( n_feats );
	for( size_t k = 0; k < n_feats; ++k )
	{
		KeyPoint & kp = cv_feats[k];
		kp.pt.x		= in_features[k]->x;
		kp.pt.y		= in_features[k]->y;
		kp.angle	= in_features[k]->orientation;
		kp.size		= in_features[k]->scale;
	} // end-for

	Mat cvImg(cv::cvarrToMat(inImg_gray.getAs<IplImage>()));
	Mat cv_descs;

#	if MRPT_OPENCV_VERSION_NUM < 0x300
	Ptr<Feature2D> orb = Algorithm::create<Feature2D>("Feature2D.ORB");
	orb->operator()( cvImg, Mat(), cv_feats, cv_descs, true /* use_precomputed_feats */ );
#else
	Ptr<cv::ORB> orb = cv::ORB::create( n_feats, options.ORBOptions.scale_factor, options.ORBOptions.n_levels );
	orb->detectAndCompute(cvImg, Mat(), cv_feats, cv_descs, true /* use_precomputed_feats */ );
#endif

	// add descriptor to CFeatureList
	for( size_t k = 0; k < n_feats; ++k )
	{
		in_features[k]->descriptors.ORB.resize( cv_descs.cols );
		for( int i = 0; i < cv_descs.cols; ++i )
			in_features[k]->descriptors.ORB[i] = cv_descs.at<uchar>(k,i);

	} // end-for
#	endif
#endif

} // end-internal_computeORBImageDescriptors


