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

bool featureComp( CFeaturePtr f1, CFeaturePtr f2 ) { return ( f1->KLT_val > f2->KLT_val ); }

#if MRPT_HAS_OPENCV
#	if MRPT_OPENCV_VERSION_NUM>=0x200
using namespace cv;
bool KeypointComp( KeyPoint k1, KeyPoint k2 ) { return (k1.response > k2.response); }
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

	vector<KeyPoint> cv_feats; // The opencv keypoint output vector

	// JL: It's better to use an adaptive threshold, controlled from our caller. 
//	int aux = options.FASTOptions.threshold;
//	if( nDesiredFeatures != 0 )
//	{
//		double a = 89.81;
//		double b = -0.4107*nDesiredFeatures;
//		double c = 134.7;
//		double d = -0.003121*nDesiredFeatures;
//
//		aux = max(0,(int)(a*exp(b) + c*exp(d))-20);
//	}

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

	sort( cv_feats.begin(), cv_feats.end(), KeypointComp );


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

	//vector<KeyPoint>  cv_feats;

	IplImage* _img = cGrey;

	FAST(_img, cv_feats, options.FASTOptions.threshold, options.FASTOptions.nonmax_suppression );
	sort( cv_feats.begin(), cv_feats.end(), KeypointComp );

#	endif

	const size_t	N			= cv_feats.size();
	unsigned int	nMax		= nDesiredFeatures != 0 && N > nDesiredFeatures ? nDesiredFeatures : N;
	const int 		offset		= (int)this->options.patchSize/2 + 1;
	const size_t	size_2		= options.patchSize/2;
	const size_t 	imgH		= inImg.getHeight();
	const size_t 	imgW		= inImg.getWidth();
	unsigned int	i			= 0;
	unsigned int	cont		= 0;
	TFeatureID		nextID		= init_ID;
	feats.clear();
	while( cont != nMax && i != N )
	{
		const int xBorderInf = (int)floor( cv_feats[i].pt.x - size_2 );
		const int xBorderSup = (int)floor( cv_feats[i].pt.x + size_2 );
		const int yBorderInf = (int)floor( cv_feats[i].pt.y - size_2 );
		const int yBorderSup = (int)floor( cv_feats[i].pt.y + size_2 );

		if( xBorderSup < (int)imgW && xBorderInf > 0 && yBorderSup < (int)imgH && yBorderInf > 0 )
		{
			CFeaturePtr ft		= CFeature::Create();
			ft->type			= featFAST;
			ft->ID				= nextID++;
			ft->x				= cv_feats[i].pt.x;
			ft->y				= cv_feats[i].pt.y;
			ft->KLT_val			= cv_feats[i].response;
			ft->orientation		= cv_feats[i].angle;
			ft->scale			= cv_feats[i].octave;
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
		++i;
	}
	feats.resize( cont );

#	endif // else of MRPT_OPENCV_VERSION_NUM < 0x200
#endif
	MRPT_END
}


