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


#include <mrpt/vision/tracking.h>

#include <mrpt/vision/CFeatureExtraction.h>

//#include <mrpt/poses/CPoint3D.h>
//#include <mrpt/slam/CLandmarksMap.h>
//#include <mrpt/slam/CObservationVisualLandmarks.h>
//#include <mrpt/slam/CObservationStereoImages.h>
//#include <mrpt/slam/CObservationBearingRange.h>
//#include <mrpt/system/filesystem.h>
//#include <mrpt/gui/CDisplayWindow.h>
//#include <mrpt/utils/CTicTac.h>
//#include <mrpt/math/utils.h>
//#include <mrpt/math/ops_vectors.h>
//#include <mrpt/math/lightweight_geom_data.h>
//#include <mrpt/math/geometry.h>


#include "do_opencv_includes.h"


using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::gui;
using namespace std;


#if MRPT_HAS_OPENCV
typedef std::vector<CvPoint2D32f> CvPoint2D32fVector;
#endif


/*------------------------------------------------------------
					checkTrackedFeatures
-------------------------------------------------------------*/
void vision::checkTrackedFeatures( CFeatureList &leftList,
							    CFeatureList &rightList,
								vision::TMatchingOptions options)
{
	ASSERT_( leftList.size() == rightList.size() );

	//std::cout << std::endl << "Tracked features checking ..." << std::endl;

	CFeatureList::iterator	itLeft, itRight;
	size_t					u,v;
	double					res;

	for( itLeft = leftList.begin(), itRight = rightList.begin(); itLeft != leftList.end(); )
	{
		bool delFeat = false;
		if( (*itLeft)->x < 0 || (*itLeft)->y < 0 ||							// Out of bounds
			(*itRight)->x < 0 || (*itRight)->y < 0 ||						// Out of bounds
			fabs( (*itLeft)->y - (*itRight)->y ) > options.epipolar_TH )	// Not fulfillment of the epipolar constraint
		{
			// Show reason
			std::cout << "Bad tracked match:";
			if( (*itLeft)->x < 0 || (*itLeft)->y < 0 || (*itRight)->x < 0 || (*itRight)->y < 0 )
				std::cout << " Out of bounds: (" << (*itLeft)->x << "," << (*itLeft)->y << " & (" << (*itRight)->x << "," << (*itRight)->y << ")" << std::endl;

			if( fabs( (*itLeft)->y - (*itRight)->y ) > options.epipolar_TH )
				std::cout << " Bad row checking: " << fabs( (*itLeft)->y - (*itRight)->y ) << std::endl;

			delFeat = true;
		}
		else
		{
			// Compute cross correlation:
			openCV_cross_correlation( (*itLeft)->patch, (*itRight)->patch, u, v, res );

			if( res < options.minCC_TH )
			{
				std::cout << "Bad tracked match (correlation failed):" << " CC Value: " << res << std::endl;
				delFeat = true;
			}
		} // end if

		if( delFeat ) // Erase the pair of features
		{
			itLeft = leftList.erase( itLeft );
			itRight = rightList.erase( itRight );
		}
		else
		{
			itLeft++;
			itRight++;
		}
	} // end for
} // end checkTrackedFeatures



/*------------------------------------------------------------
					trackFeatures
-------------------------------------------------------------*/
void  vision::trackFeatures2(
	const CImage &inImg1,
	const CImage &inImg2,
	CFeatureList &featureList,
	const unsigned int &window_width,
	const unsigned int &window_height)
{
MRPT_START;
#if MRPT_HAS_OPENCV

	CFeatureExtraction	fExt;
	CFeatureList		auxList;
	CMatchedFeatureList mList;

	CDisplayWindow		w1, w2;
	w1.showImageAndPoints( inImg1, featureList );

	fExt.options.featsType = featFAST;
	fExt.detectFeatures( inImg2, auxList, 0, 200 );
	w2.showImageAndPoints( inImg2, auxList );

	TMatchingOptions	opts;
	opts.matching_method		= TMatchingOptions::mmSAD;
	opts.useEpipolarRestriction = false;
	matchFeatures2( featureList, auxList, mList, opts );

	featureList.resize( mList.size() );
	CMatchedFeatureList::iterator	itmList;
	CFeatureList::iterator			itFeat;

	for( itFeat = featureList.begin(), itmList = mList.begin(); itFeat != featureList.end(); ++itFeat )
		(*itFeat) = itmList->second;

MRPT_END;

#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif
} // end trackFeatures2

/** Track a set of features from old_img -> new_img using sparse optimal flow (classic KL method)
  *  Optional parameters that can be passed in "extra_params":
  *		- "window_width"  (Default=15)
  *		- "window_height" (Default=15)
  *
  *  \sa OpenCV's method cvCalcOpticalFlowPyrLK
  */
void vision::trackFeatures_KL(
	const CImage &inImg1,
	const CImage &inImg2,
	vision::CFeatureList &featureList,
	const mrpt::utils::TParametersDouble &extra_params )
{
MRPT_START;

	unsigned int window_width = extra_params.getWithDefaultVal("window_width",15);
	unsigned int window_height = extra_params.getWithDefaultVal("window_height",15);

#if MRPT_HAS_OPENCV

// Both images must be of the same size
ASSERT_( inImg1.getWidth() == inImg2.getWidth() && inImg1.getHeight() == inImg2.getHeight() );

// Use OpenCV Implementation
// OpenCV Local Variables
std::vector<CvPoint2D32fVector>		points(2);
std::vector<char>	status;

int	flags		= 0;
int	nFeatures	= (int)featureList.size();					// Number of features

// Grayscale images
IplImage *pGrey, *cGrey;

if( inImg1.isColor() && inImg2.isColor() )
{
	// Input Images
	IplImage* pImg = (IplImage*)inImg1.getAsIplImage();
	IplImage* cImg = (IplImage*)inImg2.getAsIplImage();

	pGrey = cvCreateImage( cvGetSize( pImg ), 8, 1 );
	cGrey = cvCreateImage( cvGetSize( cImg ), 8, 1 );

	// Conver to grayscale
	cvCvtColor( pImg, pGrey, CV_BGR2GRAY );
	cvCvtColor( cImg, cGrey, CV_BGR2GRAY );
}
else
{
	pGrey = (IplImage*)inImg1.getAsIplImage();
	cGrey = (IplImage*)inImg2.getAsIplImage();
}

// Pyramids
IplImage* pPyr = NULL;
IplImage* cPyr = NULL;

// Arrays definition
points[0].resize(nFeatures);	// = (CvPoint2D32f*)cvAlloc( nFeatures*sizeof( points[0][0] ) );
points[1].resize(nFeatures);	// = (CvPoint2D32f*)cvAlloc( nFeatures*sizeof( points[0][0] ) );
status.resize(nFeatures);		// = (char*)cvAlloc( nFeatures );

// Array conversion MRPT->OpenCV
CFeatureList::iterator		itFeat;
int							i;				// Counter
for( i = 0, itFeat = featureList.begin(); i < nFeatures && itFeat != featureList.end(); i++, itFeat++  )
{
	points[0][i].x = (*itFeat)->x;
	points[0][i].y = (*itFeat)->y;
} // end for

// ********************************************************************************
// Optical Flow Computation (new method)
//cv::Mat								pImage, cImage;
//std::vector<cv::Point2f>			pPoints, cPoints;
//std::vector<cv::Point2f>::iterator	itPoints;
//std::vector<float>					err;
//std::vector<unsigned char>			cStatus;
//
//pImage = cv::cvarrToMat( (IplImage*)inImg1.getAsIplImage(), false );
//cImage = cv::cvarrToMat( (IplImage*)inImg2.getAsIplImage(), false );
//
//CFeatureList::iterator				itFeat;
//int									i;				// Counter
//
//// Array conversion MRPT->OpenCV
//pPoints.resize( featureList.size() );
//for( itPoints = pPoints.begin(), itFeat = featureList.begin(); itFeat != featureList.end(); itFeat++, itPoints++ )
//	*itPoints = cv::Point2f( (*itFeat)->x, (*itFeat)->y );
//CTicTac clock;
//clock.Tic();
//
//cv::calcOpticalFlowPyrLK( pImage, cImage, pPoints, cPoints, cStatus, err, cv::Size( window_width, window_height ) );
//cout << "Tiempo: " << clock.Tac() << endl;
//// Array conversion OpenCV->MRPT
//for( i = 0, itPoints = cPoints.begin(), itFeat = featureList.begin(); itFeat != featureList.end(); i++, itFeat++, itPoints++ )
//{
//	if( cStatus[i] == 1 &&
//		itPoints->x > 0 && itPoints->y > 0 &&
//		itPoints->x < pImage.cols && itPoints->y < pImage.rows )
//	{
//		// Feature could be tracked
//		(*itFeat)->x			= itPoints->x;
//		(*itFeat)->y			= itPoints->y;
//		(*itFeat)->KLT_status	= statusKLT_TRACKED;
//		(*itFeat)->KLT_val		= err[i];
//	} // end if
//	else
//	{
//		// Feature could not be tracked
//		(*itFeat)->x			= -1;
//		(*itFeat)->y			= -1;
//		(*itFeat)->KLT_status	= cStatus[i] == 0 ? statusKLT_IDLE : statusKLT_OOB;
//	} // end else
//} // end-for
// ********************************************************************************

for( i = 0, itFeat = featureList.begin(); i < nFeatures && itFeat != featureList.end(); i++, itFeat++  )
{
	points[0][i].x = (*itFeat)->x;
	points[0][i].y = (*itFeat)->y;
} // end for
cvCalcOpticalFlowPyrLK( pGrey, cGrey, pPyr, cPyr,
	&points[0][0], &points[1][0], nFeatures, cvSize( window_width, window_height ), 3, &status[0], NULL,
	cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), flags );

// Array conversion OpenCV->MRPT
for( i = 0, itFeat = featureList.begin(); i < nFeatures && itFeat != featureList.end(); i++, itFeat++  )
{
	if( status[i] == 1 &&
		points[1][i].x > 0 && points[1][i].y > 0 &&
		points[1][i].x < pGrey->width && points[1][i].y < pGrey->height )
	{
		// Feature could be tracked
		(*itFeat)->x			= points[1][i].x;
		(*itFeat)->y			= points[1][i].y;
		(*itFeat)->KLT_status	= statusKLT_TRACKED;
	} // end if
	else	// Feature could not be tracked
	{
		(*itFeat)->x			= -1;
		(*itFeat)->y			= -1;
		(*itFeat)->KLT_status	= status[i] == 0 ? statusKLT_IDLE : statusKLT_OOB;
	} // end else
} // end for

// Free memory
if( inImg1.isColor() && inImg2.isColor() )
{
	cvReleaseImage( &pGrey );
	cvReleaseImage( &cGrey );
}

cvReleaseImage( &pPyr );
cvReleaseImage( &cPyr );

//cvFree( (void**)&points[0] );
//cvFree( (void**)&points[1] );
//cvFree( (void**)&status );

#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif

	MRPT_END;
} // end trackFeatures


/*------------------------------------------------------------
					trackFeatures

	OLD VERSION: Redirect to trackFeatures_KL

-------------------------------------------------------------*/
void  vision::trackFeatures(
	const CImage &inImg1,
	const CImage &inImg2,
	CFeatureList &featureList,
	const unsigned int &window_width,
	const unsigned int &window_height)
{
	vision::trackFeatures_KL(
		inImg1,inImg2,
		featureList,
		TParametersDouble("window_width",(double)window_width,"window_height",(double)window_height,NULL)
		);
}

/*------------------------------------------------------------
					trackFeatures
-------------------------------------------------------------*/
void  vision::trackFeatures( const CImage &inImg1,
				 				  const CImage &inImg2,
								  const CFeatureList &inFeatureList,
								  CFeatureList &outFeatureList,
								  const unsigned int &window_width,
								  const unsigned int &window_height)
{
MRPT_START;

MRPT_TODO("Remove this function / unify with the overloaded one!")

#if MRPT_HAS_OPENCV

MRPT_START;

// Both images must be of the same size
ASSERT_( inImg1.getWidth() == inImg2.getWidth() && inImg1.getHeight() == inImg2.getHeight() );

// Use OpenCV Implementation
// OpenCV Local Variables
std::vector<CvPoint2D32fVector>		points(2);
std::vector<char>	status;

int	flags		= 0;
const int	nFeatures	= (int)inFeatureList.size();					// Number of features

// Grayscale images
IplImage *pGrey, *cGrey;

if( inImg1.isColor() && inImg2.isColor() )
{
	// Input Images
	IplImage* pImg = (IplImage*)inImg1.getAsIplImage();
	IplImage* cImg = (IplImage*)inImg2.getAsIplImage();

	pGrey = cvCreateImage( cvGetSize( pImg ), 8, 1 );
	cGrey = cvCreateImage( cvGetSize( cImg ), 8, 1 );

	// Conver to grayscale
	cvCvtColor( pImg, pGrey, CV_BGR2GRAY );
	cvCvtColor( cImg, cGrey, CV_BGR2GRAY );
}
else
{
	pGrey = (IplImage*)inImg1.getAsIplImage();
	cGrey = (IplImage*)inImg2.getAsIplImage();
}

// Pyramids
IplImage* pPyr = NULL;
IplImage* cPyr = NULL;

// Arrays definition
points[0].resize(nFeatures);	// = (CvPoint2D32f*)cvAlloc( nFeatures*sizeof( points[0][0] ) );
points[1].resize(nFeatures);	// = (CvPoint2D32f*)cvAlloc( nFeatures*sizeof( points[0][0] ) );
status.resize(nFeatures);		// = (char*)cvAlloc( nFeatures );

// Array conversion MRPT->OpenCV
CFeatureList::const_iterator		itFeat;
int							i;				// Counter
for( i = 0, itFeat = inFeatureList.begin(); i < nFeatures && itFeat != inFeatureList.end(); ++i, ++itFeat  )
{
	points[0][i].x = (*itFeat)->x;
	points[0][i].y = (*itFeat)->y;
} // end for

cvCalcOpticalFlowPyrLK( pGrey, cGrey, pPyr, cPyr,
	&points[0][0], &points[1][0], nFeatures, cvSize( window_width, window_height ), 3, &status[0], NULL,
	cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), flags );

// outFeatureList.resize( nFeatures );

// Array conversion OpenCV->MRPT
for( i = 0, itFeat = inFeatureList.begin(); i < nFeatures && itFeat != inFeatureList.end(); ++i, ++itFeat )
{
	CFeaturePtr feat = CFeature::Create();

	if( status[i] == 1 &&
		points[1][i].x > 0 && points[1][i].y > 0 &&
		points[1][i].x < pGrey->width && points[1][i].y < pGrey->height )
	{
		// Feature could be tracked
		feat->x			= points[1][i].x;
		feat->y			= points[1][i].y;
		feat->KLT_status	= statusKLT_TRACKED;
	} // end if
	else	// Feature could not be tracked
	{
		feat->x				= -1;
		feat->y				= -1;
		feat->KLT_status	= status[i] == 0 ? statusKLT_IDLE : statusKLT_OOB;
	} // end else
	feat->ID = (*itFeat)->ID;
	outFeatureList.push_back( feat );
} // end for

// Free memory
if( inImg1.isColor() && inImg2.isColor() )
{
	cvReleaseImage( &pGrey );
	cvReleaseImage( &cGrey );
}

cvReleaseImage( &pPyr );
cvReleaseImage( &cPyr );

//cvFree( (void**)&points[0] );
//cvFree( (void**)&points[1] );
//cvFree( (void**)&status );

MRPT_END;

#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif

	MRPT_END;
} // end trackFeatures



/*-------------------------------------------------------------
					filterBadCorrsByDistance
-------------------------------------------------------------*/
void  vision::filterBadCorrsByDistance( TMatchingPairList &feat_list, unsigned int numberOfSigmas )
{
	ASSERT_( numberOfSigmas > 0 );
	//	MRPT_UNUSED_PARAM( numberOfSigmas );
	MRPT_START;

	TMatchingPairList::iterator	itPair;
	CMatrix									dist;
	float									v_mean, v_std;
	unsigned int							count = 0;

	dist.setSize( feat_list.size(), 1 );
	//v_mean.resize(1);
	//v_std.resize(1);

	// Compute mean and standard deviation of the distance
	for( itPair = feat_list.begin(); itPair != feat_list.end() ; itPair++, count++ ) {
		//cout << "(" << itPair->other_x << "," << itPair->other_y << "," << itPair->this_z << ")" << "- (" << itPair->this_x << "," << itPair->this_y << "," << itPair->other_z << "): ";
		//cout << sqrt( square( itPair->other_x - itPair->this_x ) + square( itPair->other_y - itPair->this_y ) + square( itPair->other_z - itPair->this_z ) ) << endl;
		dist( count, 0 ) = sqrt( square( itPair->other_x - itPair->this_x ) + square( itPair->other_y - itPair->this_y ) + square( itPair->other_z - itPair->this_z ) );
	}

	dist.meanAndStdAll( v_mean, v_std );

	cout << endl << "*****************************************************" << endl;
	cout << "Mean: " << v_mean << " - STD: " << v_std << endl;
	cout << endl << "*****************************************************" << endl;

	// Filter out bad points
	unsigned int idx = 0;
	//for( int idx = (int)feat_list.size()-1; idx >= 0; idx-- )
	for( itPair = feat_list.begin(); itPair != feat_list.end(); idx++)
	{
		//if( dist( idx, 0 ) > 1.2 )
		if( fabs( dist(idx,0) - v_mean ) > v_std*numberOfSigmas )
		{
			cout << "Outlier deleted: " << dist( idx, 0 ) << " vs " << v_std*numberOfSigmas << endl;
			itPair = feat_list.erase( itPair );
		}
		else
			itPair++;
	}

	MRPT_END;
} // end filterBadCorrsByDistance

