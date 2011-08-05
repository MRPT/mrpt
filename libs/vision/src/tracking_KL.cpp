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

#include <mrpt/system/memory.h>
#include <mrpt/vision/tracking.h>
#include <mrpt/vision/CFeatureExtraction.h>

#include "do_opencv_includes.h"


using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace std;


/** Track a set of features from old_img -> new_img using sparse optimal flow (classic KL method)
  *  Optional parameters that can be passed in "extra_params":
  *		- "window_width"  (Default=15)
  *		- "window_height" (Default=15)
  *
  *  \sa OpenCV's method cvCalcOpticalFlowPyrLK
  */
void CFeatureTracker_KL::trackFeatures_impl(
	const CImage &old_img,
	const CImage &new_img,
	vision::CFeatureList &featureList )
{
MRPT_START;

	const unsigned int 	window_width = extra_params.getWithDefaultVal("window_width",15);
	const unsigned int 	window_height = extra_params.getWithDefaultVal("window_height",15);

#if MRPT_HAS_OPENCV

	// Both images must be of the same size
	ASSERT_( old_img.getWidth() == new_img.getWidth() && old_img.getHeight() == new_img.getHeight() );

	const size_t  img_width  = old_img.getWidth();
	const size_t  img_height = old_img.getHeight();

	// OpenCV Local Variables
	std::vector<CvPoint2D32fVector>		points(2);
	std::vector<char>	status;

	const size_t nFeatures	= featureList.size();					// Number of features

	// Grayscale images
	const CImage prev_gray(old_img, FAST_REF_OR_CONVERT_TO_GRAY);
	const CImage cur_gray(new_img, FAST_REF_OR_CONVERT_TO_GRAY);

	// Arrays definition
	points[0].resize(nFeatures);
	points[1].resize(nFeatures);
	status.resize(nFeatures);

	// Array conversion MRPT->OpenCV
	CFeatureList::iterator		itFeat;
	size_t   i;				// Counter
	for( i = 0, itFeat = featureList.begin(); i < nFeatures && itFeat != featureList.end(); i++, itFeat++  )
	{
		points[0][i].x = (*itFeat)->x;
		points[0][i].y = (*itFeat)->y;
	} // end for

	// local scope for auxiliary variables around cvCalcOpticalFlowPyrLK()
	if (!featureList.empty())
	{
		IplImage *prev_gray_ipl = reinterpret_cast<IplImage *>(prev_gray.getAsIplImage());
		IplImage *cur_gray_ipl  = reinterpret_cast<IplImage *>(cur_gray.getAsIplImage());

		// Pyramids
		// JL: It seems that cache'ing the pyramids of previous images doesn't really improve the efficiency (!?!?)
		IplImage* pPyr = NULL;
		IplImage* cPyr = NULL;

		int	flags = 0;

		cvCalcOpticalFlowPyrLK(prev_gray_ipl, cur_gray_ipl, pPyr, cPyr,
			&points[0][0], &points[1][0], nFeatures, cvSize( window_width, window_height ), 3, &status[0], NULL,
			cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), flags );

		cvReleaseImage( &pPyr );
		cvReleaseImage( &cPyr );
	}

	// Array conversion OpenCV->MRPT
	for( i = 0, itFeat = featureList.begin(); i < nFeatures && itFeat != featureList.end(); i++, itFeat++  )
	{
		if( status[i] == 1 &&
			points[1][i].x > 0 && points[1][i].y > 0 &&
			points[1][i].x < img_width && points[1][i].y < img_height )
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

	featureList.mark_kdtree_as_outdated(); // the internal KD-tree must be recomputed

#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif

	MRPT_END;
} // end trackFeatures


void CFeatureTracker_KL::trackFeatures_impl(
	const CImage &old_img,
	const CImage &new_img,
	TSimpleFeatureList &featureList )
{
MRPT_START;

	const unsigned int 	window_width = extra_params.getWithDefaultVal("window_width",15);
	const unsigned int 	window_height = extra_params.getWithDefaultVal("window_height",15);

#if MRPT_HAS_OPENCV

	// Both images must be of the same size
	ASSERT_( old_img.getWidth() == new_img.getWidth() && old_img.getHeight() == new_img.getHeight() );

	const size_t  img_width  = old_img.getWidth();
	const size_t  img_height = old_img.getHeight();

	const size_t nFeatures	= featureList.size();					// Number of features

	// Grayscale images
	const CImage prev_gray(old_img, FAST_REF_OR_CONVERT_TO_GRAY);
	const CImage cur_gray(new_img, FAST_REF_OR_CONVERT_TO_GRAY);

	// Array conversion MRPT->OpenCV
	if (nFeatures>0)
	{
		CvPoint2D32f *points[2];

		points[0] = reinterpret_cast<CvPoint2D32f *>( mrpt_alloca(sizeof(CvPoint2D32f)*nFeatures) );
		points[1] = reinterpret_cast<CvPoint2D32f *>( mrpt_alloca(sizeof(CvPoint2D32f)*nFeatures) );

		std::vector<char>	status(nFeatures);

		for(size_t i=0;i<nFeatures;++i)
		{
			points[0][i].x = featureList[i].pt.x;
			points[0][i].y = featureList[i].pt.y;
		} // end for

		// local scope for auxiliary variables around cvCalcOpticalFlowPyrLK()
		IplImage *prev_gray_ipl = reinterpret_cast<IplImage *>(prev_gray.getAsIplImage());
		IplImage *cur_gray_ipl  = reinterpret_cast<IplImage *>(cur_gray.getAsIplImage());

		// Pyramids
		// JL: It seems that cache'ing the pyramids of previous images doesn't really improve the efficiency (!?!?)
		IplImage* pPyr = NULL;
		IplImage* cPyr = NULL;

		int	flags = 0;

		const int    LK_levels    = 3; // 3;
		const int    LK_max_iters = 5; //20;
		const double LK_epsilon   = 0.5; //0.03;

		cvCalcOpticalFlowPyrLK(prev_gray_ipl, cur_gray_ipl, pPyr, cPyr,
			&points[0][0], &points[1][0], nFeatures, cvSize( window_width, window_height ), LK_levels, &status[0], NULL,
			cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,LK_max_iters,LK_epsilon), flags );


		cvReleaseImage( &pPyr );
		cvReleaseImage( &cPyr );

		for(size_t i=0;i<nFeatures;++i)
		{
			TSimpleFeature &ft = featureList[i];

			if( status[i] == 1 &&
				points[1][i].x > 0 && points[1][i].y > 0 &&
				points[1][i].x < img_width && points[1][i].y < img_height )
			{
				// Feature could be tracked
				ft.pt.x			= round(points[1][i].x);
				ft.pt.y			= round(points[1][i].y);
				ft.track_status	= statusKLT_TRACKED;
			} // end if
			else	// Feature could not be tracked
			{
				ft.pt.x			= -1;
				ft.pt.y			= -1;
				ft.track_status	= status[i] == 0 ? statusKLT_IDLE : statusKLT_OOB;
			} // end else
		} // end for

		featureList.mark_kdtree_as_outdated(); // the internal KD-tree must be recomputed

		mrpt_alloca_free( points[0] );
		mrpt_alloca_free( points[1] );

	}


#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif

	MRPT_END;
} // end trackFeatures

