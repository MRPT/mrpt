/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/vision.h>  // Precompiled headers

#include <mrpt/vision/CFeatureExtraction.h>


// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 


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

	const CImage img_grayscale(inImg, FAST_REF_OR_CONVERT_TO_GRAY);
	const IplImage* cGrey = img_grayscale.getAs<IplImage>();

	CvSeq *kp	=	NULL;
	CvSeq *desc	=	NULL;
	CvMemStorage *storage = cvCreateMemStorage(0);

	// Extract the SURF points:
	CvSURFParams surf_params = cvSURFParams(options.SURFOptions.hessianThreshold, options.SURFOptions.rotation_invariant ? 1:0);
	surf_params.nOctaves = options.SURFOptions.nOctaves;
	surf_params.nOctaveLayers = options.SURFOptions.nLayersPerOctave;

	cvExtractSURF( cGrey, NULL, &kp, &desc, storage, surf_params);

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
			ft->descriptors.SURF.resize( options.SURFOptions.rotation_invariant ? 128 : 64 );
			std::vector<float>::iterator itDesc;
			unsigned int k;

			for( k = 0, itDesc = ft->descriptors.SURF.begin(); k < ft->descriptors.SURF.size(); k++, itDesc++ )
				*itDesc = d[k];

			feats.push_back( ft );

		} // end if
	} // end for

	cvReleaseMemStorage(&storage); // Free memory

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

	const CImage img_grayscale(inImg, FAST_REF_OR_CONVERT_TO_GRAY);
	const IplImage* cGrey = img_grayscale.getAs<IplImage>();

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
	// Extract the SURF points:
	CvSURFParams surf_params = cvSURFParams(options.SURFOptions.hessianThreshold, options.SURFOptions.rotation_invariant ? 1:0);
	surf_params.nOctaves = options.SURFOptions.nOctaves;
	surf_params.nOctaveLayers = options.SURFOptions.nLayersPerOctave;

	cvExtractSURF( cGrey, NULL, &kp, &desc, storage, surf_params, 1 /* Use precomputed key-points */ );
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
		ft->descriptors.SURF.resize( options.SURFOptions.rotation_invariant ? 128 : 64 );
		std::vector<float>::iterator itDesc;
		unsigned int k;
		for( k = 0, itDesc = ft->descriptors.SURF.begin(); k < ft->descriptors.SURF.size(); k++, itDesc++ )
			*itDesc = d[k];

	} // end for


	cvReleaseMemStorage(&storage); // Free memory

#else
			THROW_EXCEPTION("Method not available since either MRPT has been compiled without OpenCV or OpenCV version is incorrect (Required 1.1.0)")
#endif //MRPT_HAS_OPENCV
}  // end internal_computeSurfDescriptors

