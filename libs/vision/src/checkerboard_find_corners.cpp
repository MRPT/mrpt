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

#include <mrpt/vision/chessboard_find_corners.h>

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

	int find_chess_flags = CV_CALIB_CB_ADAPTIVE_THRESH;
	if (normalize_image)
		find_chess_flags |= CV_CALIB_CB_NORMALIZE_IMAGE;

	if (!useScaramuzzaMethod)
	{
		// Standard OpenCV's function:
		corners_found = 0 != cvFindChessboardCorners(
			img.getAs<IplImage>(),
			check_size,
			&corners_list[0],
			&corners_count,
			find_chess_flags);
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
		}
	}
	else
	{	// Not found.
		cornerCoords.clear();
		return;
	}

#endif
}


