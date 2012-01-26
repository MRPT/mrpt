/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
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
#include <mrpt/vision/CStereoRectifyMap.h>

#include "do_opencv_includes.h"

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::math;


// Ctor: Leave all vectors empty
CStereoRectifyMap::CStereoRectifyMap() :
	m_alpha(-1),
	m_resize_output(false),
	m_enable_both_centers_coincide(false),
	m_resize_output_value(0,0),
	m_interpolation_method(mrpt::utils::IMG_INTERP_LINEAR)
{
}

void CStereoRectifyMap::internal_invalidate()
{
	m_dat_mapx_left.clear();  // don't do a "strong clear" since memory is likely
	m_dat_mapx_right.clear(); // to be reasigned soon.
	m_dat_mapy_left.clear();
	m_dat_mapy_right.clear();
}

void CStereoRectifyMap::setAlpha(double alpha)
{
	m_alpha = alpha;

	this->internal_invalidate();
}

void CStereoRectifyMap::enableResizeOutput(bool enable, unsigned int target_width, unsigned int target_height)
{
	m_resize_output = enable;
	m_resize_output_value.x = target_width;
	m_resize_output_value.y = target_height;

	this->internal_invalidate();
}

void CStereoRectifyMap::enableBothCentersCoincide(bool enable)
{
	m_enable_both_centers_coincide = enable;

	this->internal_invalidate();
}



/** Prepares the mapping from the distortion parameters of a camera.
  * Must be called before invoking \a undistort().
  */
void CStereoRectifyMap::setFromCamParams(const mrpt::utils::TStereoCamera & params)
{
	MRPT_START
#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x200
	const mrpt::utils::TCamera &cam1 = params.leftCamera;
	const mrpt::utils::TCamera &cam2 = params.rightCamera;

	ASSERT_( cam1.ncols==cam2.ncols && cam1.nrows==cam2.nrows  )

	const uint32_t ncols = cam1.ncols;
	const uint32_t nrows = cam1.nrows;

	const cv::Size trg_size = m_resize_output ?
		cv::Size(m_resize_output_value.x,m_resize_output_value.y)  // User requested image scaling
		:
		cv::Size();  // Default=don't scale

	const uint32_t ncols_out = m_resize_output ? m_resize_output_value.x : ncols;
	const uint32_t nrows_out = m_resize_output ? m_resize_output_value.y : nrows;

	// save a copy for future reference
	m_camera_params = params;

	// Create OpenCV's wrappers for output maps:
	m_dat_mapx_left.resize(2*nrows_out*ncols_out);
	m_dat_mapy_left.resize(nrows_out*ncols_out);

	m_dat_mapx_right.resize(2*nrows_out*ncols_out);
	m_dat_mapy_right.resize(nrows_out*ncols_out);

	CvMat mapx_left = cvMat(nrows_out,ncols_out,  CV_16SC2, &m_dat_mapx_left[0] );
	CvMat mapy_left = cvMat(nrows_out,ncols_out,  CV_16UC1, &m_dat_mapy_left[0] );
	CvMat mapx_right = cvMat(nrows_out,ncols_out,  CV_16SC2, &m_dat_mapx_right[0] );
	CvMat mapy_right = cvMat(nrows_out,ncols_out,  CV_16UC1, &m_dat_mapy_right[0] );

	cv::Mat _mapx_left  = cv::cvarrToMat(&mapx_left,false);
	cv::Mat _mapy_left  = cv::cvarrToMat(&mapy_left,false);
	cv::Mat _mapx_right = cv::cvarrToMat(&mapx_right,false);
	cv::Mat _mapy_right = cv::cvarrToMat(&mapy_right,false);


	// right camera pose: Rotation
    CMatrixDouble44 hMatrix;
	// NOTE!: OpenCV seems to expect the INVERSE of the pose we keep, so invert it:
	params.rightCameraPose.getInverseHomogeneousMatrix( hMatrix );

    double m1[3][3];
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            m1[i][j] = hMatrix(i,j);

	// right camera pose: translation
	double rcTrans[3] = { hMatrix(0,3), hMatrix(1,3), hMatrix(2,3) };

    double ipl[3][3], ipr[3][3], dpl[5], dpr[5];
    for( unsigned int i = 0; i < 3; ++i )
        for( unsigned int j = 0; j < 3; ++j )
        {
            ipl[i][j] = cam1.intrinsicParams(i,j);
            ipr[i][j] = cam2.intrinsicParams(i,j);
        }

	for( unsigned int i = 0; i < 5; ++i )
    {
        dpl[i] = cam1.dist[i];
        dpr[i] = cam2.dist[i];
    }

	const cv::Mat R( 3, 3, CV_64F, &m1 );
    const cv::Mat T( 3, 1, CV_64F, &rcTrans );

    const cv::Mat K1(3,3,CV_64F,ipl);
    const cv::Mat K2(3,3,CV_64F,ipr);
    const cv::Mat D1(1,5,CV_64F,dpl);
    const cv::Mat D2(1,5,CV_64F,dpr);

    double _R1[3][3], _R2[3][3], _P1[3][4], _P2[3][4], _Q[4][4];
    cv::Mat R1(3,3,CV_64F,_R1);
    cv::Mat R2(3,3,CV_64F,_R2);
    cv::Mat P1(3,4,CV_64F,_P1);
    cv::Mat P2(3,4,CV_64F,_P2);
    cv::Mat Q(4,4,CV_64F,_Q);

	const cv::Size img_size(ncols,nrows);
	const cv::Size real_trg_size = m_resize_output ? trg_size : img_size; // Note: trg_size is Size() by default

    cv::stereoRectify(
        K1, D1,
        K2, D2,
        img_size,
        R, T,
        R1, R2, P1, P2, Q,
		m_enable_both_centers_coincide ? cv::CALIB_ZERO_DISPARITY : 0,
		m_alpha,
		trg_size // Size() by default=no resize
		);
        // Rest of arguments -> default

	cv::initUndistortRectifyMap( K1, D1, R1, P1, real_trg_size, CV_16SC2, _mapx_left, _mapy_left );
	cv::initUndistortRectifyMap( K2, D2, R2, P2, real_trg_size, CV_16SC2, _mapx_right, _mapy_right );

	// Populate the parameter matrices of the output rectified images:
    for( unsigned int i = 0; i < 3; ++i )
        for( unsigned int j = 0; j < 3; ++j )
        {
			m_rectified_image_params.leftCamera.intrinsicParams(i,j) = _P1[i][j];
			m_rectified_image_params.rightCamera.intrinsicParams(i,j) = _P2[i][j];
		}
	// They have no distortion:
	m_rectified_image_params.leftCamera.dist.fill(0);
	m_rectified_image_params.rightCamera.dist.fill(0);

	// Target image size:
	m_rectified_image_params.leftCamera.ncols = real_trg_size.width;
	m_rectified_image_params.leftCamera.nrows = real_trg_size.height;

	m_rectified_image_params.rightCamera.ncols = real_trg_size.width;
	m_rectified_image_params.rightCamera.nrows = real_trg_size.height;

	// Rest of params don't change:
	m_rectified_image_params.leftCamera.focalLengthMeters = params.leftCamera.focalLengthMeters;
	m_rectified_image_params.rightCamera.focalLengthMeters = params.rightCamera.focalLengthMeters;
	m_rectified_image_params.rightCameraPose = params.rightCameraPose;

#else
	THROW_EXCEPTION("MRPT built without OpenCV >=2.0.0!")
#endif
	MRPT_END
}

void CStereoRectifyMap::rectify(
	const mrpt::utils::CImage &in_left_image,
	const mrpt::utils::CImage &in_right_image,
	mrpt::utils::CImage &out_left_image,
	mrpt::utils::CImage &out_right_image
	) const
{
	MRPT_START

#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x200
	const uint32_t ncols = m_camera_params.leftCamera.ncols;
	const uint32_t nrows = m_camera_params.leftCamera.nrows;

	const CvSize trg_size = m_resize_output ?
		cvSize(m_resize_output_value.x, m_resize_output_value.y)
		:
		cvSize(ncols,nrows);

	out_left_image.resize( trg_size.width, trg_size.height, in_left_image.isColor() ? 3:1, in_left_image.isOriginTopLeft() );
	out_right_image.resize( trg_size.width, trg_size.height, in_left_image.isColor() ? 3:1, in_left_image.isOriginTopLeft() );

	const IplImage * in_left  = in_left_image.getAs<IplImage>();
	const IplImage * in_right = in_right_image.getAs<IplImage>();

	IplImage * out_left  = out_left_image.getAs<IplImage>();
	IplImage * out_right = out_right_image.getAs<IplImage>();

	this->rectify_IPL(in_left, in_right, out_left, out_right);

#endif
	MRPT_END
}

// In place:
void CStereoRectifyMap::rectify(
	mrpt::utils::CImage &left_image,
	mrpt::utils::CImage &right_image,
	const bool use_internal_mem_cache) const
{
	MRPT_START

#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x200
	const uint32_t ncols = m_camera_params.leftCamera.ncols;
	const uint32_t nrows = m_camera_params.leftCamera.nrows;

	const CvSize trg_size = m_resize_output ?
		cvSize(m_resize_output_value.x, m_resize_output_value.y)
		:
		cvSize(ncols,nrows);

	const IplImage * in_left  = left_image.getAs<IplImage>();
	const IplImage * in_right = right_image.getAs<IplImage>();

	IplImage * out_left_image, *out_right_image;
	if (use_internal_mem_cache)
	{
		m_cache1.resize( trg_size.width, trg_size.height, left_image.isColor() ? 3:1, left_image.isOriginTopLeft() );
		m_cache2.resize( trg_size.width, trg_size.height, right_image.isColor() ? 3:1, right_image.isOriginTopLeft() );

		out_left_image  = m_cache1.getAs<IplImage>();
		out_right_image = m_cache2.getAs<IplImage>();
	}
	else
	{
		out_left_image  = cvCreateImage( trg_size, in_left->depth, in_left->nChannels );
		out_right_image = cvCreateImage( trg_size, in_right->depth, in_right->nChannels );
	}

	this->rectify_IPL(
		in_left, in_right,
		out_left_image, out_right_image);

	if (use_internal_mem_cache)
	{
		// Copy the data: we have avoided one allocation & one deallocation
		// (If the sizes match, these calls have no effects)
		left_image.resize( trg_size.width, trg_size.height, left_image.isColor() ? 3:1, left_image.isOriginTopLeft() );
		right_image.resize( trg_size.width, trg_size.height, right_image.isColor() ? 3:1, right_image.isOriginTopLeft() );

		cvCopy(out_left_image,  left_image.getAs<IplImage>());
		cvCopy(out_right_image, right_image.getAs<IplImage>());
	}
	else
	{
		// Move the internal pointers: but we deallocate the old contents and needed to allocate this one
		left_image.setFromIplImage(out_left_image);
		right_image.setFromIplImage(out_right_image);
	}

#endif
	MRPT_END
}

/** Overloaded version for in-place rectification of image pairs stored in a mrpt::slam::CObservationStereoImages.
  *  Upon return, the new camera intrinsic parameters will be already stored in the observation object.
  */
void CStereoRectifyMap::rectify(
	mrpt::slam::CObservationStereoImages & stereo_image_observation,
	const bool use_internal_mem_cache) const
{
	MRPT_START
	ASSERT_(stereo_image_observation.hasImageRight)

	// Rectify images:
	this->rectify( stereo_image_observation.imageLeft, stereo_image_observation.imageRight, use_internal_mem_cache );

	// Copy output image parameters:
	stereo_image_observation.setStereoCameraParams( this->m_rectified_image_params );

	MRPT_END
}


/** Just like rectify() but directly works with OpenCV's "IplImage*", which must be passed as "void*" to avoid header dependencies */
void CStereoRectifyMap::rectify_IPL(
	const void* srcImg_left,
	const void* srcImg_right,
	void* outImg_left,
	void* outImg_right) const
{
	MRPT_START
	ASSERT_(srcImg_left!=outImg_left && srcImg_right!=outImg_right)

	if (!isSet()) THROW_EXCEPTION("Error: setFromCamParams() must be called prior to rectify().")

#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x200
	const uint32_t ncols = m_camera_params.leftCamera.ncols;
	const uint32_t nrows = m_camera_params.leftCamera.nrows;

	const uint32_t ncols_out = m_resize_output ? m_resize_output_value.x : ncols;
	const uint32_t nrows_out = m_resize_output ? m_resize_output_value.y : nrows;


	const CvMat mapx_left = cvMat(nrows_out,ncols_out,  CV_16SC2, const_cast<int16_t*>(&m_dat_mapx_left[0]) );
	const CvMat mapy_left = cvMat(nrows_out,ncols_out,  CV_16UC1, const_cast<uint16_t*>(&m_dat_mapy_left[0]) );
	const CvMat mapx_right = cvMat(nrows_out,ncols_out,  CV_16SC2, const_cast<int16_t*>(&m_dat_mapx_right[0]) );
	const CvMat mapy_right = cvMat(nrows_out,ncols_out,  CV_16UC1, const_cast<uint16_t*>(&m_dat_mapy_right[0]) );

	const cv::Mat mapx1 = cv::cvarrToMat(&mapx_left);
	const cv::Mat mapy1 = cv::cvarrToMat(&mapy_left);
	const cv::Mat mapx2 = cv::cvarrToMat(&mapx_right);
	const cv::Mat mapy2 = cv::cvarrToMat(&mapy_right);

	const cv::Mat src1 = cv::cvarrToMat(srcImg_left);
	const cv::Mat src2 = cv::cvarrToMat(srcImg_right);
	cv::Mat dst1 = cv::cvarrToMat(outImg_left);
	cv::Mat dst2 = cv::cvarrToMat(outImg_right);

    cv::remap( src1, dst1, mapx1, mapy1,static_cast<int>(m_interpolation_method),cv::BORDER_CONSTANT, cvScalarAll(0) );
    cv::remap( src2, dst2, mapx2, mapy2,static_cast<int>(m_interpolation_method),cv::BORDER_CONSTANT, cvScalarAll(0) );

#endif
	MRPT_END
}




const mrpt::utils::TStereoCamera & CStereoRectifyMap::getRectifiedImageParams() const
{
	if (!isSet()) THROW_EXCEPTION("Error: setFromCamParams() must be called before.")
	return m_rectified_image_params;
}

const mrpt::utils::TCamera & CStereoRectifyMap::getRectifiedLeftImageParams() const
{
	if (!isSet()) THROW_EXCEPTION("Error: setFromCamParams() must be called before.")
	return m_rectified_image_params.leftCamera;
}

const mrpt::utils::TCamera & CStereoRectifyMap::getRectifiedRightImageParams() const
{
	if (!isSet()) THROW_EXCEPTION("Error: setFromCamParams() must be called before.")
	return m_rectified_image_params.rightCamera;
}

