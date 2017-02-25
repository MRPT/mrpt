/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/vision/chessboard_find_corners.h>
#include <mrpt/math/geometry.h> // crossProduct3D()

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 
#include "checkerboard_ocamcalib_detector.h"

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;


/** Look for the corners of a chessboard in the image
  * \param cornerCoords [OUT] The pixel coordinates of all the corners.
  * \param check_size_x [IN] The number of squares, in the X direction
  * \param check_size_y [IN] The number of squares, in the Y direction
  * \param normalize_image [IN] Whether to normalize the image before detection
  * \param useScaramuzzaMethod [IN] Whether to use the alternative, more robust method by M. Rufli, D. Scaramuzza, and R. Siegwart.
  *
  * \return true on success
  */
bool mrpt::vision::findChessboardCorners(
	const mrpt::utils::CImage &in_img,
	std::vector<TPixelCoordf> 	&cornerCoords,
	unsigned int  check_size_x,
	unsigned int  check_size_y,
	bool		normalize_image,
	bool		useScaramuzzaMethod)
{
#if MRPT_HAS_OPENCV
	MRPT_START

	ASSERT_(check_size_y>0 && check_size_x>0)

	// Grayscale version:
	const CImage img( in_img, FAST_REF_OR_CONVERT_TO_GRAY );

	// Try with expanded versions of the image if it fails to detect the checkerboard:
	int corners_count;
	bool corners_found=false;

	const CvSize check_size = cvSize(check_size_x, check_size_y);

	const int CORNERS_COUNT = check_size_x * check_size_y;

	vector<CvPoint2D32f> corners_list;
	corners_count = CORNERS_COUNT;
	corners_list.resize( CORNERS_COUNT );

	cornerCoords.clear();

	int find_chess_flags = cv::CALIB_CB_ADAPTIVE_THRESH;
	if (normalize_image)
		find_chess_flags |= cv::CALIB_CB_NORMALIZE_IMAGE;

	if (!useScaramuzzaMethod)
	{
		cv::Mat cvImg = cv::cvarrToMat(img.getAs<IplImage>());
		
		vector<cv::Point2f> pointbuf;

		// Standard OpenCV's function:
		corners_found = 0 != cv::findChessboardCorners(
			cvImg,
			check_size,
			pointbuf,
			find_chess_flags);

		corners_list.resize(pointbuf.size());
		for (size_t i=0;i<pointbuf.size();i++) {
			corners_list[i].x = pointbuf[i].x;
			corners_list[i].y = pointbuf[i].y;
		}
	}
	else
	{
		// Return: -1: errors, 0: not found, 1: found OK
		corners_found = 1 == cvFindChessboardCorners3(
			img,
			check_size,
			corners_list
			);
	}

	// Check # of corners:
	if (corners_found && corners_count!=CORNERS_COUNT)
		corners_found=false;

	if( corners_found )
	{
		// Refine corners:
		cvFindCornerSubPix(
			img.getAs<IplImage>(),
			&corners_list[0],
			corners_count,
			cvSize(5,5), 	// window
			cvSize(-1,-1),
			cvTermCriteria( CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10, 0.01f ));

		// save the corners in the data structure:
		int y;
		unsigned int k;
		for( y = 0, k = 0; y < check_size.height; y++ )
			for( int x = 0; x < check_size.width; x++, k++ )
				cornerCoords.push_back(  TPixelCoordf( corners_list[k].x, corners_list[k].y ) );
	}


	return corners_found;

	MRPT_END
#else
	return false;
#endif
}


/** Look for the corners of one or more chessboard/checkerboards in the image.
  *  This method uses an improved version of OpenCV's cvFindChessboardCorners published
  *   by M. Rufli, D. Scaramuzza, and R. Siegwart.
  *  That method has been extended in this MRPT implementation to automatically detect a
  *   number of different checkerboards in the same image.
  *
  * \param cornerCoords [OUT] A vector of N vectors of pixel coordinates, for each of the N chessboards detected.
  * \param check_size_x [IN] The number of squares, in the X direction
  * \param check_size_y [IN] The number of squares, in the Y direction
  *
  *
  * \sa mrpt::vision::checkerBoardCameraCalibration, drawChessboardCorners
  */
void mrpt::vision::findMultipleChessboardsCorners(
	const mrpt::utils::CImage &in_img,
	std::vector<std::vector<TPixelCoordf> > 	&cornerCoords,
	unsigned int  check_size_x,
	unsigned int  check_size_y )
{
#if MRPT_HAS_OPENCV
	// Grayscale version:
	const CImage img( in_img, FAST_REF_OR_CONVERT_TO_GRAY );

	std::vector<std::vector<CvPoint2D32f> >  corners_list;

	// Return: -1: errors, 0: not found, 1: found OK
	bool corners_found = find_chessboard_corners_multiple(
		img,
		cvSize(check_size_x,check_size_y),
		corners_list);

	if( corners_found && !corners_list.empty() )
	{
		// Alloc space for output points:
		cornerCoords.resize( corners_list.size() );

		// Refine corners:
		for (size_t i=0;i<corners_list.size();i++)
		{
			ASSERT_(corners_list[i].size()==check_size_x*check_size_y)

			cvFindCornerSubPix(
				img.getAs<IplImage>(),
				&corners_list[i][0],
				check_size_x*check_size_y,
				cvSize(5,5), 	// window
				cvSize(-1,-1),
				cvTermCriteria( CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10, 0.01f ));

			// save the corners in the data structure:
			for( unsigned int y = 0, k = 0; y < check_size_y; y++ )
				for( unsigned int x = 0; x < check_size_x; x++, k++ )
					cornerCoords[i].push_back(  TPixelCoordf( corners_list[i][k].x, corners_list[i][k].y ) );


			// Consistency of the counter-clockwise XYZ reference system and corners ORDER.

			// Key idea: The cross product of X * Y must point outwards the screen:

			const mrpt::math::TPoint2D
				pt_0  = cornerCoords[i][0],
				pt_x1 = cornerCoords[i][1],
				pt_y1 = cornerCoords[i][check_size_x];
			const mrpt::math::TPoint3D Ax = pt_x1 - pt_0;  // z=0
			const mrpt::math::TPoint3D Ay = pt_y1 - pt_0;  // z=0

			const Eigen::Matrix<double,3,1> Az = mrpt::math::crossProduct3D(Ax,Ay);
			if (Az[2] > 0) {
				// Invert all rows (X):
				for( unsigned int y = 0; y < check_size_y; y++ )
					std::reverse( cornerCoords[i].begin()+y*check_size_x, cornerCoords[i].begin()+(y+1)*check_size_x );
			}
		}
	}
	else
	{	// Not found.
		cornerCoords.clear();
		return;
	}

#endif
}


