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
*								selectGoodFeaturesKLT  									        *
************************************************************************************************/
void CFeatureExtraction::selectGoodFeaturesKLT(
		const mrpt::utils::CImage		&inImg,
		CFeatureList		&feats,
		unsigned int		init_ID,
		unsigned int		nDesiredFeatures,
		void				*mask_ ) const
{
	const unsigned int MAX_COUNT = 300;

//#define VERBOSE_TIMING

#ifdef VERBOSE_TIMING
	CTicTac tictac;
#endif
		MRPT_START

		#if MRPT_HAS_OPENCV

		// Reinterpret opencv formal arguments
		CvMatrix *mask = reinterpret_cast<CvMatrix*>(mask_);

		// -----------------------------------------------------------------
		// Create OpenCV Local Variables
		// -----------------------------------------------------------------
		int				count = 0;
		int				nPts;

		CvImage img, cGrey;

#ifdef VERBOSE_TIMING
		tictac.Tic();
#endif
		img.attach( (IplImage*)inImg.getAsIplImage(), false );	// Attach Image as IplImage and do not use ref counter
#ifdef VERBOSE_TIMING
		cout << "[KLT] Attach: " << tictac.Tac()*1000.0f << endl;
#endif
		if( img.channels() == 1 )
			cGrey = img;										// Input image is already 'grayscale'
		else
		{
			cGrey.create( cvGetSize( img ), 8, 1);
			cvCvtColor( img, cGrey, CV_BGR2GRAY );				// Convert input image into 'grayscale'
		}

		nDesiredFeatures <= 0 ? nPts = MAX_COUNT : nPts = nDesiredFeatures;

		std::vector<CvPoint2D32f> points(nPts);

		CvImage eig, temp;									// temporary and auxiliary images

#ifdef VERBOSE_TIMING
		tictac.Tic();
#endif
		eig.create( cvGetSize( cGrey ), 32, 1 );
		temp.create( cvGetSize( cGrey ), 32, 1 );
#ifdef VERBOSE_TIMING
		cout << "[KLT] Create: " << tictac.Tac()*1000.0f << endl;
#endif
		count = nPts;										// Number of points to find


#if 0	// Temporary debug
		{
			static int i=0;
			cvSaveImage( format("debug_map_%05i.bmp",++i).c_str(), cGrey);
		}
#endif
		// -----------------------------------------------------------------
		// Select good features with subpixel accuracy (USING HARRIS OR KLT)
		// -----------------------------------------------------------------
		if( options.featsType == featHarris )
		{
#ifdef VERBOSE_TIMING
			tictac.Tic();
#endif
			cvGoodFeaturesToTrack( cGrey, eig, temp, &points[0], &count,	// input and output data
				(double)options.harrisOptions.threshold,					// for rejecting weak local maxima ( with min_eig < threshold*max(eig_image) )
				(double)options.harrisOptions.min_distance,					// minimum distance between features
				mask ? (*mask) : static_cast<const CvMat*>(NULL),			// ROI
				(double)options.harrisOptions.radius,						// size of the block of pixels used
				1,															// use Harris
				options.harrisOptions.k );									// k factor for the Harris algorithm
#ifdef VERBOSE_TIMING
			cout << "[KLT] Find feats: " << tictac.Tac()*1000.0f << endl;
#endif
		}
		else
		{
#ifdef VERBOSE_TIMING
			tictac.Tic();
#endif
			cvGoodFeaturesToTrack( cGrey, eig, temp, &points[0], &count,	// input and output data
				(double)options.KLTOptions.threshold,						// for rejecting weak local maxima ( with min_eig < threshold*max(eig_image) )
				(double)options.KLTOptions.min_distance,					// minimum distance between features
				mask ? (*mask) : static_cast<const CvMat*>(NULL),			// ROI
				options.KLTOptions.radius,									// size of the block of pixels used
				0,															// use Kanade Lucas Tomasi
				0.04 );														// un-used parameter
#ifdef VERBOSE_TIMING
			cout << "[KLT]: Find feats: " << tictac.Tac()*1000.0f << endl;
#endif
		}

		if( nDesiredFeatures > 0 && count < nPts )
			cout << "\n[WARNING][selectGoodFeaturesKLT]: Only " << count << " of " << nDesiredFeatures << " points could be extracted in the image." << endl;

		if( options.FIND_SUBPIXEL )
		{
#ifdef VERBOSE_TIMING
			tictac.Tic();
#endif
			// Subpixel interpolation
			cvFindCornerSubPix( cGrey, &points[0], count,
				cvSize(3,3), cvSize(-1,-1),
				cvTermCriteria( CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10, 0.05 ));

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
				ft->track_status = statusKLT_TRACKED;		// Feature Status
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
		CvMatrix mask( img.getHeight(), img.getWidth(), CV_8UC1 );
		CvScalar zero = cvRealScalar( 0.0 );
		CvScalar one = cvRealScalar( 1.0 );

		cvSet( mask, one );												// Set the whole mask to 1 initially

		CFeatureList::const_iterator itKLT;

		TFeatureID mxID = 0;

		for( itKLT = inList.begin(); itKLT != inList.end(); itKLT++ )
		{
			int cx = (int)(*itKLT)->x;
			int cy = (int)(*itKLT)->y;

			// Mask surronding pixels
			size_t xxI = max( 0, cx - 15 );
			size_t xxE = min( cx + 15, (int)mask.cols()-1 );
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

/************************************************************************************************
*								extractFeaturesKLT												*
************************************************************************************************/
void  CFeatureExtraction::extractFeaturesKLT(
		const mrpt::utils::CImage			&img,
		CFeatureList			&feats,
		unsigned int			init_ID,
		unsigned int			nDesiredFeatures,
		const TImageROI			&ROI) const
{
#if MRPT_HAS_OPENCV
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



