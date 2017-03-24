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
using namespace std;


/************************************************************************************************
*							extractFeaturesKLT
************************************************************************************************/
void CFeatureExtraction::extractFeaturesKLT(
		const mrpt::utils::CImage			&inImg,
		CFeatureList			&feats,
		unsigned int			init_ID,
		unsigned int			nDesiredFeatures,
		const TImageROI			&ROI) const
{
//#define VERBOSE_TIMING

#ifdef VERBOSE_TIMING
	CTicTac tictac;
#endif
		MRPT_START

		#if MRPT_HAS_OPENCV
        const unsigned int MAX_COUNT = 300;

		// -----------------------------------------------------------------
		// Create OpenCV Local Variables
		// -----------------------------------------------------------------
		int				count = 0;
		int				nPts;

#ifdef VERBOSE_TIMING
		tictac.Tic();
#endif
		const cv::Mat img( cv::cvarrToMat( inImg.getAs<IplImage>() ) );

#ifdef VERBOSE_TIMING
		cout << "[KLT] Attach: " << tictac.Tac()*1000.0f << endl;
#endif
		const CImage inImg_gray( inImg, FAST_REF_OR_CONVERT_TO_GRAY );
		const cv::Mat cGrey( cv::cvarrToMat( inImg_gray.getAs<IplImage>() ) );

		nDesiredFeatures <= 0 ? nPts = MAX_COUNT : nPts = nDesiredFeatures;

#ifdef VERBOSE_TIMING
		tictac.Tic();
#endif

#ifdef VERBOSE_TIMING
		cout << "[KLT] Create: " << tictac.Tac()*1000.0f << endl;
#endif
		count = nPts;										// Number of points to find

		// -----------------------------------------------------------------
		// Select good features with subpixel accuracy (USING HARRIS OR KLT)
		// -----------------------------------------------------------------
		const bool use_harris = ( options.featsType == featHarris );

#ifdef VERBOSE_TIMING
		tictac.Tic();
#endif
		std::vector<cv::Point2f> points;
		cv::goodFeaturesToTrack(
			cGrey,points, nPts, 
			(double)options.harrisOptions.threshold,    // for rejecting weak local maxima ( with min_eig < threshold*max(eig_image) )
			(double)options.harrisOptions.min_distance, // minimum distance between features
			cv::noArray(), // mask
			3, // blocksize
			use_harris, /* harris */
			options.harrisOptions.k 
			);
#ifdef VERBOSE_TIMING
		cout << "[KLT] Find feats: " << tictac.Tac()*1000.0f << endl;
#endif

		if( nDesiredFeatures > 0 && count < nPts )
			cout << "\n[WARNING][selectGoodFeaturesKLT]: Only " << count << " of " << nDesiredFeatures << " points could be extracted in the image." << endl;

		if( options.FIND_SUBPIXEL )
		{
#ifdef VERBOSE_TIMING
			tictac.Tic();
#endif
			// Subpixel interpolation
			cv::cornerSubPix(cGrey,points,
				cv::Size(3,3), cv::Size(-1,-1),
				cv::TermCriteria( CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10, 0.05 ));

#ifdef VERBOSE_TIMING
			cout << "[KLT] subpixel: " << tictac.Tac()*1000.0f << endl;
#endif
		}

		// -----------------------------------------------------------------
		// Fill output structure
		// -----------------------------------------------------------------
#ifdef VERBOSE_TIMING
		tictac.Tic();
#endif

		feats.clear();
		unsigned int	borderFeats = 0;
		unsigned int	nCFeats		= init_ID;
		int				i			= 0;
		const int		limit		= min( nPts, count );
		int				offset		= (int)this->options.patchSize/2 + 1;
		unsigned int	imgH		= inImg.getHeight();
		unsigned int	imgW		= inImg.getWidth();

		while( i < limit )
		{
			const int xBorderInf = (int)floor( points[i].x - options.patchSize/2 );
			const int xBorderSup = (int)floor( points[i].x + options.patchSize/2 );
			const int yBorderInf = (int)floor( points[i].y - options.patchSize/2 );
			const int yBorderSup = (int)floor( points[i].y + options.patchSize/2 );

			if( options.patchSize==0 || ( (xBorderSup < (int)imgW) && (xBorderInf > 0) && (yBorderSup < (int)imgH) && (yBorderInf > 0) ) )
			{
				CFeaturePtr ft = CFeature::Create();

				ft->type		= featKLT;
				ft->x			= points[i].x;				// X position
				ft->y			= points[i].y;				// Y position
				ft->track_status = status_TRACKED;		    // Feature Status
				ft->response	= 0.0;						// A value proportional to the quality of the feature (unused yet)
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

				feats.push_back( ft );

			} // end if
			else
				borderFeats++;

			i++;
		} // end while

#ifdef VERBOSE_TIMING
		cout << "[KLT] Create output: " << tictac.Tac()*1000.0f << endl;
#endif


		#else
			THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
		#endif

		MRPT_END

} // end of function

MRPT_TODO("Delete? Refactor / join to mrpt::vision::CGenericFeatureTracker?")
#if 0
/*------------------------------------------------------------
					findMoreFeatures
-------------------------------------------------------------*/
void  CFeatureExtraction::findMoreFeatures( const mrpt::utils::CImage &img,
											const CFeatureList &inList,
											CFeatureList &outList,
											unsigned int nDesiredFeats) const
{
#if MRPT_HAS_OPENCV
	MRPT_START

	if( options.featsType == featHarris || options.featsType == featKLT )
	{
		// Mask the points already stored in the list
		cv::Mat mask = cv::Mat::ones(img.getHeight(), img.getWidth(), CV_8UC1 ); // Set the whole mask to 1 initially

		CFeatureList::const_iterator itKLT;

		TFeatureID mxID = 0;

		for( itKLT = inList.begin(); itKLT != inList.end(); itKLT++ )
		{
			int cx = (int)(*itKLT)->x;
			int cy = (int)(*itKLT)->y;

			// Mask surronding pixels
			size_t xxI = max( 0, cx - 15 );
			size_t xxE = min( cx + 15, mask.cols()-1 );
			size_t yyI = max( 0, cy - 15 );
			size_t yyE = min( cy + 15, (int)mask.rows()-1 );

			for(size_t yy = yyI; yy < yyE; yy++)
				for(size_t xx = xxI; xx < xxE; xx++)
					cvSet2D( mask, yy, xx, zero );

			if( (*itKLT)->ID > mxID )
				mxID = (*itKLT)->ID;

		}
		selectGoodFeaturesKLT( img, outList, mxID + 1, nDesiredFeats, &mask );
	}
	else if( options.featsType == featFAST )
	{
		TFeatureID mxID = inList.getMaxID();
		extractFeaturesFAST( img, outList, mxID+1 );		// Find all the possible FAST features

		// Delete all that are too close to the current ones
		CFeatureList::iterator			itOutList;
		CFeatureList::const_iterator	itInList;
		for( itInList = inList.begin(); itInList != inList.end(); ++itInList )
		{
			for( itOutList = outList.begin(); itOutList != outList.end(); )
			{
				if( fabs((*itOutList)->x-(*itInList)->x) < 15 ||  fabs((*itOutList)->y-(*itInList)->y) < 15 )
					itOutList = outList.erase( itOutList );
				else
					++itOutList;
			} // end-for
		} // end-for

		if( nDesiredFeats != 0 && outList.size() > nDesiredFeats )
			outList.resize( nDesiredFeats );
	}
	MRPT_END
#else
	THROW_EXCEPTION("MRPT was compiled without OpenCV")
#endif
} // end findMoreFeatures
#endif


MRPT_TODO("Delete? Is not this a duplicate of extractFeaturesKLT ()???")
#if 0
/************************************************************************************************
*								selectGoodFeaturesKLT												*
************************************************************************************************/
void  CFeatureExtraction::selectGoodFeaturesKLT(
		const mrpt::utils::CImage			&img,
		CFeatureList			&feats,
		unsigned int			init_ID,
		unsigned int			nDesiredFeatures) const
{
#if  MRPT_HAS_OPENCV
	// Get image size
	size_t imgW = img.getWidth();
	size_t imgH = img.getHeight();

	// Check image size and ROI limits
	ASSERT_( imgH > 0 && imgW > 0 );													// Proper Image Size
	ASSERT_( ROI.xMin >= 0 && ROI.yMin >= 0 && ROI.xMax < imgW && ROI.yMax < imgH );	// Proper TImageROI Limits

	// Mask
	CvMatrix mask( imgH, imgW, CV_8UC1 );
	CvScalar zero = cvRealScalar( 0.0 );
	CvScalar one = cvRealScalar( 1.0 );

	if( ROI.xMin == 0 && ROI.xMax == 0 && ROI.yMin == 0 && ROI.yMax == 0 )		// Use the whole image (except the borders)
	{
		if( options.patchSize == 0 )
			selectGoodFeaturesKLT( img, feats, init_ID, nDesiredFeatures, NULL );
		else
		{
			// We mask the borders of the image in order to get an appropiate patch (according to the patch size)
			// ---------------------------------------------------------------------
			// 00000000000000000000000000000000000
			// 00000000000000000000000000000000000
			// 00111111111111111111111111111111100
			// 00111111111111111111111111111111100
			// 00111111111111111111111111111111100
			// 00111111111111111111111111111111100
			// 00111111111111111111111111111111100
			// 00111111111111111111111111111111100
			// 00111111111111111111111111111111100
			// 00000000000000000000000000000000000
			// 00000000000000000000000000000000000

			cvSet( mask, one );												// Set the whole mask to 1 initially

			size_t border = (options.patchSize-1)/2 + 1;
			size_t xx,yy;													// Put outside of the "for"s to avoid errors in VC6
			for( xx = 0; xx < imgW; xx++ )
				for( yy = 0; yy < border; yy++ )
					cvSet2D( mask, yy, xx, zero );							// Set to 0 the pixels not to be processed

			for( xx = 0; xx < imgW; xx++ )
				for( yy = imgH - border; yy < imgH; yy++ )
					cvSet2D( mask, yy, xx, zero );							// Set to 0 the pixels not to be processed

			for( xx = 0; xx < border; xx++ )
				for( yy = border; yy < imgH - border; yy++ )
					cvSet2D( mask, yy, xx, zero ) ;							// Set to 0 the pixels not to be processed

			for( xx = imgW - border; xx < imgW; xx++ )
				for( yy = border; yy < imgH - border; yy++ )
					cvSet2D( mask, yy, xx, zero ) ;							// Set to 0 the pixels not to be processed

			selectGoodFeaturesKLT( img, feats, init_ID, nDesiredFeatures, &mask );
		} // end if-else patchSize == 0
	} // end if ROI does not exist
	else
	{
		cvSet( mask, zero );												// Set the whole mask to 0 initially
		for( unsigned int xx = ROI.xMin; xx < ROI.xMax; xx++ )
			for( unsigned int yy = ROI.yMin; yy < ROI.yMax; yy++ )
				cvSet2D( mask, yy, xx, one) ;								// Set to 1 the pixels to be processed

		selectGoodFeaturesKLT( img, feats, init_ID, nDesiredFeatures, &mask );
	} // end if-else there exists a ROI
#else
	THROW_EXCEPTION("MRPT was compiled without OpenCV")
#endif
}

#endif


