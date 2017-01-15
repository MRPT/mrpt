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


#if MRPT_HAS_OPENCV
#	include "faster/faster_corner_prototypes.h"
#endif

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace std;


// ------------  SSE2-optimized implementations of FASTER -------------
void CFeatureExtraction::detectFeatures_SSE2_FASTER9(const CImage &img, TSimpleFeatureList & corners, const int threshold, bool append_to_list, uint8_t octave,std::vector<size_t> * out_feats_index_by_row)
{
#if MRPT_HAS_OPENCV
	const IplImage *IPL = img.getAs<IplImage>();
	ASSERTDEB_(IPL && IPL->nChannels==1)
	if (!append_to_list) corners.clear();

	fast_corner_detect_9 (IPL,corners,threshold,octave,out_feats_index_by_row);
#else
	THROW_EXCEPTION("MRPT built without OpenCV support!")
#endif
}
void CFeatureExtraction::detectFeatures_SSE2_FASTER10(const CImage &img, TSimpleFeatureList & corners, const int threshold, bool append_to_list, uint8_t octave,std::vector<size_t> * out_feats_index_by_row)
{
#if MRPT_HAS_OPENCV
	const IplImage *IPL = img.getAs<IplImage>();
	ASSERTDEB_(IPL && IPL->nChannels==1)
	if (!append_to_list) corners.clear();

	fast_corner_detect_10 (IPL,corners,threshold,octave,out_feats_index_by_row);
#else
	THROW_EXCEPTION("MRPT built without OpenCV support!")
#endif
}
void CFeatureExtraction::detectFeatures_SSE2_FASTER12(const CImage &img, TSimpleFeatureList & corners, const int threshold, bool append_to_list, uint8_t octave,std::vector<size_t> * out_feats_index_by_row)
{
#if MRPT_HAS_OPENCV
	const IplImage *IPL = img.getAs<IplImage>();
	ASSERTDEB_(IPL && IPL->nChannels==1)
	if (!append_to_list) corners.clear();

	fast_corner_detect_12 (IPL,corners,threshold,octave,out_feats_index_by_row);
#else
	THROW_EXCEPTION("MRPT built without OpenCV support!")
#endif
}

/************************************************************************************************
*								extractFeaturesFASTER											*
************************************************************************************************/
// N_fast = 9, 10, 12
void  CFeatureExtraction::extractFeaturesFASTER_N(
	const int					N_fast,
	const mrpt::utils::CImage	& inImg,
	CFeatureList			    & feats,
	unsigned int			    init_ID,
	unsigned int			    nDesiredFeatures,
	const TImageROI			    & ROI )  const
{
	MRPT_UNUSED_PARAM(ROI);
	MRPT_START

#if MRPT_HAS_OPENCV
	// Make sure we operate on a gray-scale version of the image:
	const CImage inImg_gray( inImg, FAST_REF_OR_CONVERT_TO_GRAY );

	const IplImage *IPL = inImg_gray.getAs<IplImage>();

	TSimpleFeatureList corners;
	TFeatureType type_of_this_feature;

	switch (N_fast)
	{
	case 9:  fast_corner_detect_9 (IPL,corners, options.FASTOptions.threshold, 0, NULL); type_of_this_feature=featFASTER9; break;
	case 10: fast_corner_detect_10(IPL,corners, options.FASTOptions.threshold, 0, NULL); type_of_this_feature=featFASTER10; break;
	case 12: fast_corner_detect_12(IPL,corners, options.FASTOptions.threshold, 0, NULL); type_of_this_feature=featFASTER12; break;
	default:
		THROW_EXCEPTION("Only the 9,10,12 FASTER detectors are implemented.")
		break;
	};

	// *All* the features have been extracted.
	const size_t N = corners.size();

	// Now:
	//  1) Sort them by "response": It's ~100 times faster to sort a list of
	//      indices "sorted_indices" than sorting directly the actual list of features "corners"
	std::vector<size_t> sorted_indices(N);
	for (size_t i=0;i<N;i++)  sorted_indices[i]=i;

	// Use KLT response
	if (options.FASTOptions.use_KLT_response ||
		nDesiredFeatures!=0 // If the user wants us to limit the number of features, we need to do it according to some quality measure
		)
	{
		const int KLT_half_win = 4;
		const int max_x = inImg_gray.getWidth() - 1 - KLT_half_win;
		const int max_y = inImg_gray.getHeight() - 1 - KLT_half_win;

		for (size_t i=0;i<N;i++)
		{
			const int x = corners[i].pt.x;
			const int y = corners[i].pt.y;
			if (x>KLT_half_win && y>KLT_half_win && x<=max_x && y<=max_y)
					corners[i].response = inImg_gray.KLT_response(x,y,KLT_half_win);
			else	corners[i].response = -100;
		}

		std::sort( sorted_indices.begin(), sorted_indices.end(), KeypointResponseSorter<TSimpleFeatureList>(corners) );
	}
	else
	{
		for (size_t i=0;i<N;i++)
			corners[i].response = 0;
	}

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
	const int		size_2		= options.patchSize/2;
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
		const TSimpleFeature &feat = corners[ sorted_indices[i] ];
		i++;

		// Patch out of the image??
		const int xBorderInf =  feat.pt.x - size_2;
		const int xBorderSup =  feat.pt.x + size_2;
		const int yBorderInf =  feat.pt.y - size_2;
		const int yBorderSup =  feat.pt.y + size_2;

		if (!( xBorderSup < (int)imgW && xBorderInf > 0 && yBorderSup < (int)imgH && yBorderInf > 0 ))
			continue; // nope, skip.

		if (do_filter_min_dist)
		{
			// Check the min-distance:
			const size_t section_idx_x = size_t(feat.pt.x * occupied_grid_cell_size_inv);
			const size_t section_idx_y = size_t(feat.pt.y * occupied_grid_cell_size_inv);

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
		ft->type			= type_of_this_feature;
		ft->ID				= nextID++;
		ft->x				= feat.pt.x;
		ft->y				= feat.pt.y;
		ft->response		= feat.response;
		ft->orientation		= 0;
		ft->scale			= 1;
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

#endif
	MRPT_END
}


