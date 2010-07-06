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

#include <mrpt/vision/CFeatureExtraction.h>

#include "do_opencv_includes.h"


using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::system;
using namespace std;

// Don't do this since it seems FAST features don't provide a "response" measure.
//  Enable in the future if it's implemented in OpenCV.
#define DO_SORT_BY_RESPONSE_QUALITY


#if MRPT_HAS_OPENCV && defined(DO_SORT_BY_RESPONSE_QUALITY)
#	if MRPT_OPENCV_VERSION_NUM>=0x200
using namespace cv;
bool KeypointComp( KeyPoint k1, KeyPoint k2 ) { return (k1.response > k2.response); }
struct KeypointCompCache : public std::binary_function<size_t,size_t,bool>
{
	const vector<KeyPoint> &m_data;
	KeypointCompCache( const vector<KeyPoint> &data ) : m_data(data) { }
	bool operator() (size_t k1, size_t k2 ) const {
		return (m_data[k1].response > m_data[k2].response);
	}
};
#	endif
#endif

/************************************************************************************************
*								extractFeaturesFAST												*
************************************************************************************************/
void  CFeatureExtraction::extractFeaturesFAST(
	const mrpt::utils::CImage			&inImg,
	CFeatureList			&feats,
	unsigned int			init_ID,
	unsigned int			nDesiredFeatures,
	const TImageROI			&ROI )  const
{
	MRPT_START

#if MRPT_HAS_OPENCV
#	if MRPT_OPENCV_VERSION_NUM < 0x200
		THROW_EXCEPTION("This function requires OpenCV >= 2.0.0")
#	else // MRPT_OPENCV_VERSION_NUM < 0x200

	using namespace cv;

	vector<KeyPoint> cv_feats; // The opencv keypoint output vector

	// JL: Instead of
	//	int aux = options.FASTOptions.threshold; ....
	//  It's better to use an adaptive threshold, controlled from our caller outside.

#	if MRPT_OPENCV_VERSION_NUM >= 0x211

	FastFeatureDetector fastDetector( options.FASTOptions.threshold, options.FASTOptions.nonmax_suppression );
	IplImage* img, *cGrey;

	img = (IplImage*)inImg.getAsIplImage();

	if( img->nChannels == 1 )
		cGrey = img;										// Input image is already 'grayscale'
	else
	{
		cGrey = cvCreateImage( cvGetSize( img ), 8, 1);
		cvCvtColor( img, cGrey, CV_BGR2GRAY );				// Convert input image into 'grayscale'
	}

	Mat theImg = cvarrToMat( cGrey );

	fastDetector.detect( theImg, cv_feats );

	if( img->nChannels != 1 )
		cvReleaseImage( &cGrey );

#	elif MRPT_OPENCV_VERSION_NUM > 0x200
	CvImage img, cGrey;
	img.attach( (IplImage*)inImg.getAsIplImage(), false );	// Attach Image as IplImage and do not use ref counter

	if( img.channels() == 1 )
		cGrey = img;										// Input image is already 'grayscale'
	else
	{
		cGrey.create( cvGetSize( img ), 8, 1);
		cvCvtColor( img, cGrey, CV_BGR2GRAY );				// Convert input image into 'grayscale'
	}
	IplImage* _img = cGrey;

	FAST(_img, cv_feats, options.FASTOptions.threshold, options.FASTOptions.nonmax_suppression );
#	endif

	// *All* the features have been extracted.
	const size_t	N			= cv_feats.size();

	// Now:
	//  1) Sort them by "response": It's ~100 times faster to sort a list of
	//      indices "sorted_indices" than sorting directly the actual list of features "cv_feats"
#ifdef DO_SORT_BY_RESPONSE_QUALITY
	std::vector<size_t> sorted_indices(N);
	for (size_t i=0;i<N;i++)  sorted_indices[i]=i;
	std::sort( sorted_indices.begin(), sorted_indices.end(), KeypointCompCache(cv_feats) );
#endif

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

	mrpt::math::CMatrixBool  occupied_sections(grid_lx,grid_ly);
	occupied_sections.fillAll(false);


	unsigned int	nMax		= (nDesiredFeatures!=0 && N > nDesiredFeatures) ? nDesiredFeatures : N;
	unsigned int    decimation_step = nDesiredFeatures==0 ? 1 : N/nDesiredFeatures;
	const int 		offset		= (int)this->options.patchSize/2 + 1;
	const size_t	size_2		= options.patchSize/2;
	const size_t 	imgH		= inImg.getHeight();
	const size_t 	imgW		= inImg.getWidth();
	unsigned int	i			= 0;
	unsigned int	cont		= 0;
	TFeatureID		nextID		= init_ID;
	feats.clear();
	//for(; cont != nMax && i != N; i++)
	while( cont != nMax )
	{
		// Take the next feature fromt the ordered list of good features:
#ifdef DO_SORT_BY_RESPONSE_QUALITY
		const KeyPoint &kp = cv_feats[ sorted_indices[i] ];
		i++;
#else
		const KeyPoint &kp = cv_feats[i];
		i+=decimation_step;
		if (i>=N) i-=(N-1);
#endif

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

#	endif // else of MRPT_OPENCV_VERSION_NUM < 0x200
#endif
	MRPT_END
}


