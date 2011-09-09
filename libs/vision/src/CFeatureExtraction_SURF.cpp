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

#include <mrpt/vision/CFeatureExtraction.h>


#include "do_opencv_includes.h"


using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::system;
using namespace std;


/************************************************************************************************
*								extractFeaturesSURF  									        *
************************************************************************************************/
void  CFeatureExtraction::extractFeaturesSURF(
		const mrpt::utils::CImage		&inImg,
		CFeatureList			&feats,
		unsigned int			init_ID,
		unsigned int			nDesiredFeatures,
		const TImageROI			&ROI) const
{
#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM >= 0x111

	const int EXTENDED_DESCRIPTOR = options.SURFOptions.rotation_invariant ? 1:0;

	IplImage* img, *cGrey;
	img = (IplImage*)inImg.getAsIplImage();

	if( img->nChannels == 1 )
		cGrey = img;										// Input image is already 'grayscale'
	else
	{
		cGrey = cvCreateImage( cvGetSize( img ), 8, 1);
		cvCvtColor( img, cGrey, CV_BGR2GRAY );				// Convert input image into 'grayscale'
	}

	CvSeq *kp	=	NULL;
	CvSeq *desc	=	NULL;
	CvMemStorage *storage = cvCreateMemStorage(0);

	// Extract the SURF points:
	cvExtractSURF( cGrey, NULL, &kp, &desc, storage, cvSURFParams(600, EXTENDED_DESCRIPTOR) );
	// *** HAVE YOU HAD A COMPILER ERROR NEAR THIS LINE?? : You need OpenCV >=1.1.0, final release or a SVN version ***

	// -----------------------------------------------------------------
	// MRPT Wrapping
	// -----------------------------------------------------------------
	feats.clear();
	unsigned int	nCFeats		= init_ID;
	int				limit;
	int				offset		= (int)this->options.patchSize/2 + 1;
	unsigned int	imgH		= inImg.getHeight();
	unsigned int	imgW		= inImg.getWidth();

	if( nDesiredFeatures == 0 )
		limit = kp->total;
	else
		limit = (int)nDesiredFeatures < kp->total ? (int)nDesiredFeatures : kp->total;

	for( int i = 0; i < limit; i++ )
	{
		// Get the OpenCV SURF point
		CvSURFPoint *point;
		CFeaturePtr ft = CFeature::Create();
		point = (CvSURFPoint*)cvGetSeqElem( kp, i );

		const int xBorderInf = (int)floor( point->pt.x - options.patchSize/2 );
		const int xBorderSup = (int)floor( point->pt.x + options.patchSize/2 );
		const int yBorderInf = (int)floor( point->pt.y - options.patchSize/2 );
		const int yBorderSup = (int)floor( point->pt.y + options.patchSize/2 );

		if( options.patchSize == 0 || ( (xBorderSup < (int)imgW) && (xBorderInf > 0) && (yBorderSup < (int)imgH) && (yBorderInf > 0) ) )
		{
			ft->type		= featSURF;
			ft->x			= point->pt.x;				// X position
			ft->y			= point->pt.y;				// Y position
			ft->orientation = point->dir;				// Orientation
			ft->scale		= point->size*1.2/9;		// Scale
			ft->ID			= nCFeats++;				// Feature ID into extraction
			ft->patchSize	= options.patchSize;		// The size of the feature patch

			if( options.patchSize > 0 )
			{
				inImg.extract_patch(
					ft->patch,
					round( ft->x ) - offset,
					round( ft->y ) - offset,
					options.patchSize,
					options.patchSize );				// Image patch surronding the feature
			}

			// Get the SURF descriptor
			float* d = (float*)cvGetSeqElem( desc, i );
			ft->descriptors.SURF.resize( EXTENDED_DESCRIPTOR ? 128 : 64 );
			std::vector<float>::iterator itDesc;
			unsigned int k;

			for( k = 0, itDesc = ft->descriptors.SURF.begin(); k < ft->descriptors.SURF.size(); k++, itDesc++ )
				*itDesc = d[k];

			feats.push_back( ft );

		} // end if
	} // end for

	cvReleaseMemStorage(&storage); // Free memory

	if( img->nChannels != 1 )
		cvReleaseImage( &cGrey );

#else
	THROW_EXCEPTION("Method not available since either MRPT has been compiled without OpenCV or OpenCV version is incorrect (Required 1.1.0)")
#endif //MRPT_HAS_OPENCV
} // end extractFeaturesSURF


/************************************************************************************************
*						internal_computeSurfDescriptors
************************************************************************************************/
void  CFeatureExtraction::internal_computeSurfDescriptors(
	const mrpt::utils::CImage	&inImg,
	CFeatureList		&in_features) const
{
#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM >= 0x111

	if (in_features.empty()) return;

	const int EXTENDED_DESCRIPTOR = options.SURFOptions.rotation_invariant ? 1:0;

	IplImage* img, *cGrey;
	img = (IplImage*)inImg.getAsIplImage();

	if( img->nChannels == 1 )
		cGrey = img;										// Input image is already 'grayscale'
	else
	{
		cGrey = cvCreateImage( cvGetSize( img ), 8, 1);
		cvCvtColor( img, cGrey, CV_BGR2GRAY );				// Convert input image into 'grayscale'
	}

	CvMemStorage *storage = cvCreateMemStorage(0);

	// Fill in the desired key-points:
	CvSeq *kp	=  cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvSURFPoint), storage );
	for (CFeatureList::iterator	itList=in_features.begin();itList!=in_features.end();++itList)
	{
		CvSURFPoint point = cvSURFPoint(
			cvPoint2D32f((*itList)->x,(*itList)->y),
			0,  // Laplacian
			16  //sizes[layer]
			);

		cvSeqPush( kp, &point );
	}


	CvSeq *desc	=  NULL;

	// Only computes the descriptors:
	cvExtractSURF( cGrey, NULL, &kp, &desc, storage, cvSURFParams(600, EXTENDED_DESCRIPTOR), 1 /* Use precomputed key-points */ );
	// *** HAVE YOU HAD A COMPILER ERROR NEAR THIS LINE?? : You need OpenCV >=1.1.0, final release or a SVN version ***

	// -----------------------------------------------------------------
	// MRPT Wrapping
	// -----------------------------------------------------------------
	CFeatureList::iterator	itList;
	int i;
	for (i=0, itList=in_features.begin();itList!=in_features.end();itList++,i++)
	{
		// Get the OpenCV SURF point
		CFeaturePtr ft = *itList;

		CvSURFPoint *point = (CvSURFPoint*)cvGetSeqElem( kp, i );

		ft->orientation = point->dir;				// Orientation
		ft->scale		= point->size*1.2/9;		// Scale

		// Get the SURF descriptor
		float* d = (float*)cvGetSeqElem( desc, i );
		ft->descriptors.SURF.resize( EXTENDED_DESCRIPTOR ? 128 : 64 );
		std::vector<float>::iterator itDesc;
		unsigned int k;
		for( k = 0, itDesc = ft->descriptors.SURF.begin(); k < ft->descriptors.SURF.size(); k++, itDesc++ )
			*itDesc = d[k];

	} // end for


	cvReleaseMemStorage(&storage); // Free memory

	if( img->nChannels != 1 )
		cvReleaseImage( &cGrey );

#else
			THROW_EXCEPTION("Method not available since either MRPT has been compiled without OpenCV or OpenCV version is incorrect (Required 1.1.0)")
#endif //MRPT_HAS_OPENCV
}  // end internal_computeSurfDescriptors

