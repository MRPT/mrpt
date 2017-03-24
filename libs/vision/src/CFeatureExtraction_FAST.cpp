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
using namespace mrpt::vision;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;


/************************************************************************************************
*								extractFeaturesFAST												*
************************************************************************************************/
void  CFeatureExtraction::extractFeaturesFAST(
	const mrpt::utils::CImage	& inImg,
	CFeatureList			    & feats,
	unsigned int			    init_ID,
	unsigned int			    nDesiredFeatures,
	const TImageROI			    & ROI,
	const CMatrixBool           * mask )  const
{
	MRPT_UNUSED_PARAM(ROI);
	MRPT_START

#if MRPT_HAS_OPENCV
#	if MRPT_OPENCV_VERSION_NUM < 0x210
		THROW_EXCEPTION("This function requires OpenCV > 2.1.0")
#	else

	using namespace cv;

	vector<KeyPoint> cv_feats; // The opencv keypoint output vector

	// Make sure we operate on a gray-scale version of the image:
	const CImage inImg_gray( inImg, FAST_REF_OR_CONVERT_TO_GRAY );

	// JL: Instead of
	//	int aux = options.FASTOptions.threshold; ....
	//  It's better to use an adaptive threshold, controlled from our caller outside.

#if MRPT_OPENCV_VERSION_NUM >= 0x211

//    cv::Mat *mask ;
//    if( _mask )
//       mask = static_cast<cv::Mat*>(_mask);

	const Mat theImg = cvarrToMat( inImg_gray.getAs<IplImage>() );

    cv::Mat cvMask;
    if( options.useMask )
    {
        cout << "using mask" << endl;
        size_t maskW = mask->getColCount(), maskH = mask->getRowCount();
        ASSERT_( maskW == inImg_gray.getWidth() && maskH == inImg_gray.getHeight() );

        // Convert Mask into CV type
        cvMask = cv::Mat::ones( maskH, maskW, CV_8UC1 );
        for( int ii = 0; ii < int(maskW); ++ii )
            for( int jj = 0; jj < int(maskH); ++jj )
            {
                if( !mask->get_unsafe(jj,ii) )
                {
                    cvMask.at<char>(ii,jj) = (char)0;
                }
            }
    }


#	if MRPT_OPENCV_VERSION_NUM < 0x300
	FastFeatureDetector fastDetector( options.FASTOptions.threshold, options.FASTOptions.nonmax_suppression );
	fastDetector.detect( theImg, cv_feats );
#else
	Ptr<cv::FastFeatureDetector> fastDetector = cv::FastFeatureDetector::create(options.FASTOptions.threshold, options.FASTOptions.nonmax_suppression );
	fastDetector->detect( theImg, cv_feats );
#endif

#elif MRPT_OPENCV_VERSION_NUM >= 0x210
	FAST(inImg_gray.getAs<IplImage>(), cv_feats, options.FASTOptions.threshold, options.FASTOptions.nonmax_suppression );
#endif

	// *All* the features have been extracted.
	const size_t N = cv_feats.size();

	// Use KLT response instead of the OpenCV's original "response" field:
	if (options.FASTOptions.use_KLT_response)
	{
		const unsigned int KLT_half_win = 4;
		const unsigned int max_x = inImg_gray.getWidth() - 1 - KLT_half_win;
		const unsigned int max_y = inImg_gray.getHeight() - 1 - KLT_half_win;
		for (size_t i=0;i<N;i++)
		{
			const unsigned int x = cv_feats[i].pt.x;
			const unsigned int y = cv_feats[i].pt.y;
			if (x>KLT_half_win && y>KLT_half_win && x<=max_x && y<=max_y)
					cv_feats[i].response = inImg_gray.KLT_response(x,y,KLT_half_win);
			else	cv_feats[i].response = -100;
		}
	}

	// Now:
	//  1) Sort them by "response": It's ~100 times faster to sort a list of
	//      indices "sorted_indices" than sorting directly the actual list of features "cv_feats"
	std::vector<size_t> sorted_indices(N);
	for (size_t i=0;i<N;i++)  sorted_indices[i]=i;
	std::sort( sorted_indices.begin(), sorted_indices.end(), KeypointResponseSorter<vector<KeyPoint> >(cv_feats) );

	//  2) Filter by "min-distance" (in options.FASTOptions.min_distance)
	//  3) Convert to MRPT CFeatureList format.
	// Steps 2 & 3 are done together in the while() below.
	// The "min-distance" filter is done by means of a 2D binary matrix where each cell is marked when one
	// feature falls within it. This is not exactly the same than a pure "min-distance" but is pretty close
	// and for large numbers of features is much faster than brute force search of kd-trees.
	// (An intermediate approach would be the creation of a mask image updated for each accepted feature, etc.)

	const bool do_filter_min_dist = options.FASTOptions.min_distance>1;

	// Used half the min-distance since we'll later mark as occupied the ranges [i-1,i+1] for a feature at "i"
	const unsigned int occupied_grid_cell_size = options.FASTOptions.min_distance/2.0;
	const float occupied_grid_cell_size_inv = 1.0f/occupied_grid_cell_size;

	unsigned int grid_lx = !do_filter_min_dist ? 1 : (unsigned int)(1 + inImg.getWidth() * occupied_grid_cell_size_inv);
	unsigned int grid_ly = !do_filter_min_dist ? 1 : (unsigned int)(1 + inImg.getHeight() * occupied_grid_cell_size_inv );

	mrpt::math::CMatrixBool  occupied_sections(grid_lx,grid_ly);  // See the comments above for an explanation.
	occupied_sections.fillAll(false);


	unsigned int	nMax		= (nDesiredFeatures!=0 && N > nDesiredFeatures) ? nDesiredFeatures : N;
	const int 		offset		= (int)this->options.patchSize/2 + 1;
	const size_t	size_2		= options.patchSize/2;
	const size_t 	imgH		= inImg.getHeight();
	const size_t 	imgW		= inImg.getWidth();
	unsigned int	i			= 0;
	unsigned int	cont		= 0;
	TFeatureID		nextID		= init_ID;

	if( !options.addNewFeatures )
		feats.clear();

	while( cont != nMax && i!=N )
	{
		// Take the next feature fromt the ordered list of good features:
		const KeyPoint &kp = cv_feats[ sorted_indices[i] ];
		i++;

		// Patch out of the image??
		const int xBorderInf = (int)floor( kp.pt.x - size_2 );
		const int xBorderSup = (int)floor( kp.pt.x + size_2 );
		const int yBorderInf = (int)floor( kp.pt.y - size_2 );
		const int yBorderSup = (int)floor( kp.pt.y + size_2 );

		if (!( xBorderSup < (int)imgW && xBorderInf > 0 && yBorderSup < (int)imgH && yBorderInf > 0 ))
			continue; // nope, skip.

		if (do_filter_min_dist)
		{
			// Check the min-distance:
			const size_t section_idx_x = size_t(kp.pt.x * occupied_grid_cell_size_inv);
			const size_t section_idx_y = size_t(kp.pt.y * occupied_grid_cell_size_inv);

			if (occupied_sections(section_idx_x,section_idx_y))
				continue; // Already occupied! skip.

			// Mark section as occupied
			occupied_sections.set_unsafe(section_idx_x,section_idx_y, true);
			if (section_idx_x>0)	occupied_sections.set_unsafe(section_idx_x-1,section_idx_y, true);
			if (section_idx_y>0)	occupied_sections.set_unsafe(section_idx_x,section_idx_y-1, true);
			if (section_idx_x<grid_lx-1)	occupied_sections.set_unsafe(section_idx_x+1,section_idx_y, true);
			if (section_idx_y<grid_ly-1)	occupied_sections.set_unsafe(section_idx_x,section_idx_y+1, true);
		}

		// All tests passed: add new feature:
		CFeaturePtr ft		= CFeature::Create();
		ft->type			= featFAST;
		ft->ID				= nextID++;
		ft->x				= kp.pt.x;
		ft->y				= kp.pt.y;
		ft->response		= kp.response;
		ft->orientation		= kp.angle;
		ft->scale			= kp.octave;
		ft->patchSize		= options.patchSize;		// The size of the feature patch

		if( options.patchSize > 0 )
		{
			inImg.extract_patch(
				ft->patch,
				round( ft->x ) - offset,
				round( ft->y ) - offset,
				options.patchSize,
				options.patchSize );						// Image patch surronding the feature
		}
		feats.push_back( ft );
		++cont;
	}
	//feats.resize( cont );  // JL: really needed???

#	endif
#endif
	MRPT_END
}


