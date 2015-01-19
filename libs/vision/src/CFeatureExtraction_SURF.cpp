/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
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
#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM >= 0x240
	using namespace cv;

	const CImage img_grayscale(inImg, FAST_REF_OR_CONVERT_TO_GRAY);
	const Mat img = cvarrToMat( img_grayscale.getAs<IplImage>() );

	vector<KeyPoint> cv_feats; // OpenCV keypoint output vector
	Mat              cv_descs; // OpenCV descriptor output


	Ptr<Feature2D> surf = Algorithm::create<Feature2D>("Feature2D.SURF");
	if( surf.empty() )
		CV_Error(CV_StsNotImplemented, "OpenCV was built without SURF support");

	surf->set("hessianThreshold", options.SURFOptions.hessianThreshold);
	surf->set("nOctaves", options.SURFOptions.nOctaves);
	surf->set("nOctaveLayers", options.SURFOptions.nLayersPerOctave);
	//surf->set("upright", params.upright != 0);
	surf->set("extended", options.SURFOptions.rotation_invariant);

#	if MRPT_OPENCV_VERSION_NUM < 0x300
	surf->operator()(img, Mat(), cv_feats, cv_descs);
#else
	surf->detectAndCompute(img, Mat(), cv_feats, cv_descs);
#endif

	// -----------------------------------------------------------------
	// MRPT Wrapping
	// -----------------------------------------------------------------
	feats.clear();
	unsigned int	nCFeats		= init_ID;
	int				offset		= (int)this->options.patchSize/2 + 1;
	unsigned int	imgH		= inImg.getHeight();
	unsigned int	imgW		= inImg.getWidth();

	const size_t n_feats = (nDesiredFeatures== 0) ? 
		cv_feats.size()
		:
		std::min((size_t)nDesiredFeatures, cv_feats.size());
	
	for(size_t i = 0; i < n_feats; i++ )
	{
		// Get the OpenCV SURF point
		CFeaturePtr ft = CFeature::Create();
		const KeyPoint &point = cv_feats[i];

		const int xBorderInf = (int)floor( point.pt.x - options.patchSize/2 );
		const int xBorderSup = (int)floor( point.pt.x + options.patchSize/2 );
		const int yBorderInf = (int)floor( point.pt.y - options.patchSize/2 );
		const int yBorderSup = (int)floor( point.pt.y + options.patchSize/2 );

		if( options.patchSize == 0 || ( (xBorderSup < (int)imgW) && (xBorderInf > 0) && (yBorderSup < (int)imgH) && (yBorderInf > 0) ) )
		{
			ft->type		= featSURF;
			ft->x			= point.pt.x;				// X position
			ft->y			= point.pt.y;				// Y position
			ft->orientation = point.angle;				// Orientation
			ft->scale		= point.size*1.2/9;		// Scale
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
			ft->descriptors.SURF.resize( cv_descs.cols );
			for( int m = 0; m < cv_descs.cols; ++m )
				ft->descriptors.SURF[m] = cv_descs.at<float>(i,m);

			feats.push_back( ft );

		} // end if
	} // end for

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
#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM >= 0x240
	using namespace cv;

	if (in_features.empty()) return;

	const CImage img_grayscale(inImg, FAST_REF_OR_CONVERT_TO_GRAY);
	const Mat img = cvarrToMat( img_grayscale.getAs<IplImage>() );

	vector<KeyPoint> cv_feats; // OpenCV keypoint output vector
	Mat              cv_descs; // OpenCV descriptor output

	// Fill in the desired key-points:
	cv_feats.resize(in_features.size());
	for (size_t i=0;i<in_features.size();++i)
	{
		cv_feats[i].pt.x = in_features[i]->x;
		cv_feats[i].pt.y = in_features[i]->y;
		cv_feats[i].size = 16;  //sizes[layer];
	}
	
	// Only computes the descriptors:
	Ptr<Feature2D> surf = Algorithm::create<Feature2D>("Feature2D.SURF");
	if( surf.empty() )
		CV_Error(CV_StsNotImplemented, "OpenCV was built without SURF support");
	surf->set("hessianThreshold", options.SURFOptions.hessianThreshold);
	surf->set("nOctaves", options.SURFOptions.nOctaves);
	surf->set("nOctaveLayers", options.SURFOptions.nLayersPerOctave);
	//surf->set("upright", params.upright != 0);
	surf->set("extended", options.SURFOptions.rotation_invariant);

	surf->compute(img, cv_feats, cv_descs);
	
	// -----------------------------------------------------------------
	// MRPT Wrapping
	// -----------------------------------------------------------------
	CFeatureList::iterator	itList;
	int i;
	for (i=0, itList=in_features.begin();itList!=in_features.end();itList++,i++)
	{
		// Get the OpenCV SURF point
		CFeaturePtr ft = *itList;
		const KeyPoint &point = cv_feats[i];

		ft->orientation = point.angle;				// Orientation
		ft->scale		= point.size*1.2/9;		// Scale

		// Get the SURF descriptor
		ft->descriptors.SURF.resize( cv_descs.cols );
		for( int m = 0; m < cv_descs.cols; ++m )
			ft->descriptors.SURF[m] = cv_descs.at<float>(i,m);		// Get the SURF descriptor
	} // end for

#else
			THROW_EXCEPTION("Method not available since either MRPT has been compiled without OpenCV or OpenCV version is incorrect (Required 1.1.0)")
#endif //MRPT_HAS_OPENCV
}  // end internal_computeSurfDescriptors

