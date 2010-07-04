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

#include "do_opencv_includes.h"


using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace std;


#if MRPT_HAS_OPENCV
typedef std::vector<CvPoint2D32f> CvPoint2D32fVector;
#endif


/** Track a set of features from old_img -> new_img using sparse optimal flow (classic KL method)
  *  Optional parameters that can be passed in "extra_params":
  *		- "window_width"  (Default=15)
  *		- "window_height" (Default=15)
  *
  *  \sa OpenCV's method cvCalcOpticalFlowPyrLK
  */
void CFeatureTracker_KL::trackFeatures(
	const CImage &inImg1,
	const CImage &inImg2,
	vision::CFeatureList &featureList )
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
//		(*itFeat)->track_status	= statusKLT_TRACKED;
//		(*itFeat)->KLT_val		= err[i];
//	} // end if
//	else
//	{
//		// Feature could not be tracked
//		(*itFeat)->x			= -1;
//		(*itFeat)->y			= -1;
//		(*itFeat)->track_status	= cStatus[i] == 0 ? statusKLT_IDLE : statusKLT_OOB;
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
		(*itFeat)->track_status	= statusKLT_TRACKED;
	} // end if
	else	// Feature could not be tracked
	{
		(*itFeat)->x			= -1;
		(*itFeat)->y			= -1;
		(*itFeat)->track_status	= status[i] == 0 ? statusKLT_IDLE : statusKLT_OOB;
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

	OLD/DEPRECATED VERSION: Redirect to CFeatureTracker_KL
-------------------------------------------------------------*/
void  vision::trackFeatures(
	const CImage &inImg1,
	const CImage &inImg2,
	CFeatureList &featureList,
	const unsigned int window_width,
	const unsigned int window_height)
{
	CFeatureTracker_KL klt( TParametersDouble(
		"window_width",(double)window_width,
		"window_height",(double)window_height,
		NULL) );

	klt.trackFeatures(inImg1,inImg2,featureList);
}

/*------------------------------------------------------------
					trackFeatures

	OLD/DEPRECATED VERSION: Redirect to CFeatureTracker_KL
-------------------------------------------------------------*/
void  vision::trackFeatures(
	const CImage &inImg1,
	const CImage &inImg2,
	const CFeatureList &inFeatureList,
	CFeatureList &outFeatureList,
	const unsigned int window_width,
	const unsigned int window_height )
{
	CFeatureTracker_KL klt( TParametersDouble(
		"window_width",(double)window_width,
		"window_height",(double)window_height,
		NULL) );

	klt.trackFeaturesNewList(inImg1,inImg2,inFeatureList, outFeatureList);
}
