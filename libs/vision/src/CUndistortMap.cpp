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
#include <mrpt/vision/CUndistortMap.h>

#include "do_opencv_includes.h"

using namespace mrpt;
using namespace mrpt::vision;


// Ctor: Leave all vectors empty
CUndistortMap::CUndistortMap()
{
}


/** Prepares the mapping from the distortion parameters of a camera.
  * Must be called before invoking \a undistort().
  */
void CUndistortMap::setFromCamParams(const mrpt::utils::TCamera &campar)
{
	MRPT_START
#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x200
	m_camera_params = campar;

	// Convert to opencv's format:
	double aux1[3][3], aux2[1][4];
	for (int i=0;i<3;i++)
	for (int j=0;j<3;j++)
	aux1[i][j] = campar.intrinsicParams(i,j);
	for (int i=0;i<4;i++)
	aux2[0][i]=campar.dist[i];

	const cv::Mat inMat( 3,3, CV_64F, aux1 );
	const cv::Mat distM( 1, 4, CV_64F, aux2 );

	m_dat_mapx.resize(2*campar.nrows*campar.ncols);
	m_dat_mapy.resize(campar.nrows*campar.ncols);

	CvMat mapx = cvMat(campar.nrows,campar.ncols,  CV_16SC2, &m_dat_mapx[0] );
	CvMat mapy = cvMat(campar.nrows,campar.ncols,  CV_16UC1, &m_dat_mapy[0] );

	cv::Mat _mapx = cv::cvarrToMat(&mapx,false);
	cv::Mat _mapy = cv::cvarrToMat(&mapy,false);

	cv::initUndistortRectifyMap( inMat, distM, cv::Mat(), inMat, _mapx.size(), _mapx.type(), _mapx, _mapy );
#else
	THROW_EXCEPTION("MRPT built without OpenCV >=2.0.0!")
#endif
	MRPT_END
}

/** Undistort the input image and saves the result in-place- \a setFromCamParams() must have been set prior to calling this.
  */
void CUndistortMap::undistort(const mrpt::utils::CImage &in_img, mrpt::utils::CImage &out_img) const
{
	MRPT_START
	if (m_dat_mapx.empty())
		THROW_EXCEPTION("Error: setFromCamParams() must be called prior to undistort().")

#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x200
	CvMat mapx = cvMat(m_camera_params.nrows,m_camera_params.ncols,  CV_16SC2, const_cast<int16_t*>(&m_dat_mapx[0]) );  // Wrappers on the data as a CvMat's.
	CvMat mapy = cvMat(m_camera_params.nrows,m_camera_params.ncols,  CV_16UC1, const_cast<uint16_t*>(&m_dat_mapy[0]) );

	IplImage *srcImg = static_cast<IplImage *>( in_img.getAsIplImage() );	// Source Image
	IplImage *outImg = cvCreateImage( cvGetSize( srcImg ), srcImg->depth, srcImg->nChannels );
	cvRemap(srcImg, outImg, &mapx, &mapy);	//cv::remap(src, dst_part, map1_part, map2_part, INTER_LINEAR, BORDER_CONSTANT );
	out_img.setFromIplImage(outImg);
#endif
	MRPT_END
}

/** Undistort the input image and saves the result in-place- \a setFromCamParams() must have been set prior to calling this.
  */
void CUndistortMap::undistort(mrpt::utils::CImage &in_out_img) const
{
	MRPT_START
	if (m_dat_mapx.empty())
		THROW_EXCEPTION("Error: setFromCamParams() must be called prior to undistort().")

#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x200
	CvMat mapx = cvMat(m_camera_params.nrows,m_camera_params.ncols,  CV_16SC2, const_cast<int16_t*>(&m_dat_mapx[0]) );  // Wrappers on the data as a CvMat's.
	CvMat mapy = cvMat(m_camera_params.nrows,m_camera_params.ncols,  CV_16UC1, const_cast<uint16_t*>(&m_dat_mapy[0]) );

	IplImage *srcImg = static_cast<IplImage *>( in_out_img.getAsIplImage() );	// Source Image
	IplImage *outImg = cvCreateImage( cvGetSize( srcImg ), srcImg->depth, srcImg->nChannels );
	cvRemap(srcImg, outImg, &mapx, &mapy);	//cv::remap(src, dst_part, map1_part, map2_part, INTER_LINEAR, BORDER_CONSTANT );
	in_out_img.setFromIplImage(outImg);
#endif
	MRPT_END
}

