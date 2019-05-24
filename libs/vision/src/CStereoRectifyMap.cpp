/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "vision-precomp.h"  // Precompiled headers

#include <mrpt/otherlibs/do_opencv_includes.h>
#include <mrpt/vision/CStereoRectifyMap.h>
#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::vision;
using namespace mrpt::img;
using namespace mrpt::math;

#if MRPT_HAS_OPENCV
#include <opencv2/core/eigen.hpp>

static void do_rectify(
	const CStereoRectifyMap& me, const cv::Mat& src_left,
	const cv::Mat& src_right, cv::Mat& out_left, cv::Mat& out_right,
	int16_t* map_xl, int16_t* map_xr, uint16_t* map_yl, uint16_t* map_yr,
	int interp_method)
{
	MRPT_START
	ASSERTMSG_(
		src_left.data != out_left.data && src_right.data != out_right.data,
		"in-place rectify not supported");

	if (!me.isSet())
		THROW_EXCEPTION(
			"Error: setFromCamParams() must be called prior to rectify().");

	const uint32_t ncols = me.getCameraParams().leftCamera.ncols;
	const uint32_t nrows = me.getCameraParams().leftCamera.nrows;

	const int ncols_out =
		me.isEnabledResizeOutput() ? me.getResizeOutputSize().x : ncols;
	const int nrows_out =
		me.isEnabledResizeOutput() ? me.getResizeOutputSize().y : nrows;

	const CvMat mapx_left = cvMat(nrows_out, ncols_out, CV_16SC2, map_xl);
	const CvMat mapy_left = cvMat(nrows_out, ncols_out, CV_16UC1, map_yl);
	const CvMat mapx_right = cvMat(nrows_out, ncols_out, CV_16SC2, map_xr);
	const CvMat mapy_right = cvMat(nrows_out, ncols_out, CV_16UC1, map_yr);

	const cv::Mat mapx1 = cv::cvarrToMat(&mapx_left);
	const cv::Mat mapy1 = cv::cvarrToMat(&mapy_left);
	const cv::Mat mapx2 = cv::cvarrToMat(&mapx_right);
	const cv::Mat mapy2 = cv::cvarrToMat(&mapy_right);

	cv::remap(
		src_left, out_left, mapx1, mapy1, interp_method, cv::BORDER_CONSTANT,
		cvScalarAll(0));
	cv::remap(
		src_right, out_right, mapx2, mapy2, interp_method, cv::BORDER_CONSTANT,
		cvScalarAll(0));
	MRPT_END
}
#endif

void CStereoRectifyMap::internal_invalidate()
{
	// don't do a "strong clear" since memory is likely to be reasigned soon.
	m_dat_mapx_left.clear();
	m_dat_mapx_right.clear();
	m_dat_mapy_left.clear();
	m_dat_mapy_right.clear();
}

void CStereoRectifyMap::setAlpha(double alpha)
{
	m_alpha = alpha;

	this->internal_invalidate();
}

void CStereoRectifyMap::enableResizeOutput(
	bool enable, unsigned int target_width, unsigned int target_height)
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
void CStereoRectifyMap::setFromCamParams(const mrpt::img::TStereoCamera& params)
{
	MRPT_START
#if MRPT_HAS_OPENCV
	const mrpt::img::TCamera& cam1 = params.leftCamera;
	const mrpt::img::TCamera& cam2 = params.rightCamera;

	ASSERT_(cam1.ncols == cam2.ncols && cam1.nrows == cam2.nrows);

	const uint32_t ncols = cam1.ncols;
	const uint32_t nrows = cam1.nrows;

	const cv::Size trg_size =
		m_resize_output
			? cv::Size(
				  m_resize_output_value.x,
				  m_resize_output_value.y)  // User requested image scaling
			: cv::Size();  // Default=don't scale

	const uint32_t ncols_out =
		m_resize_output ? m_resize_output_value.x : ncols;
	const uint32_t nrows_out =
		m_resize_output ? m_resize_output_value.y : nrows;

	// save a copy for future reference
	m_camera_params = params;

	// Create OpenCV's wrappers for output maps:
	m_dat_mapx_left.resize(2 * nrows_out * ncols_out);
	m_dat_mapy_left.resize(nrows_out * ncols_out);

	m_dat_mapx_right.resize(2 * nrows_out * ncols_out);
	m_dat_mapy_right.resize(nrows_out * ncols_out);

	CvMat mapx_left =
		cvMat(nrows_out, ncols_out, CV_16SC2, &m_dat_mapx_left[0]);
	CvMat mapy_left =
		cvMat(nrows_out, ncols_out, CV_16UC1, &m_dat_mapy_left[0]);
	CvMat mapx_right =
		cvMat(nrows_out, ncols_out, CV_16SC2, &m_dat_mapx_right[0]);
	CvMat mapy_right =
		cvMat(nrows_out, ncols_out, CV_16UC1, &m_dat_mapy_right[0]);

	cv::Mat _mapx_left = cv::cvarrToMat(&mapx_left, false);
	cv::Mat _mapy_left = cv::cvarrToMat(&mapy_left, false);
	cv::Mat _mapx_right = cv::cvarrToMat(&mapx_right, false);
	cv::Mat _mapy_right = cv::cvarrToMat(&mapy_right, false);

	// right camera pose: Rotation
	CMatrixDouble44 hMatrix;
	// NOTE!: OpenCV seems to expect the INVERSE of the pose we keep, so invert
	// it:
	mrpt::poses::CPose3D(params.rightCameraPose)
		.getInverseHomogeneousMatrix(hMatrix);

	double m1[3][3];
	for (unsigned int i = 0; i < 3; ++i)
		for (unsigned int j = 0; j < 3; ++j) m1[i][j] = hMatrix(i, j);

	// right camera pose: translation
	double rcTrans[3] = {hMatrix(0, 3), hMatrix(1, 3), hMatrix(2, 3)};

	double ipl[3][3], ipr[3][3], dpl[5], dpr[5];
	for (unsigned int i = 0; i < 3; ++i)
		for (unsigned int j = 0; j < 3; ++j)
		{
			ipl[i][j] = cam1.intrinsicParams(i, j);
			ipr[i][j] = cam2.intrinsicParams(i, j);
		}

	for (unsigned int i = 0; i < 5; ++i)
	{
		dpl[i] = cam1.dist[i];
		dpr[i] = cam2.dist[i];
	}

	const cv::Mat R(3, 3, CV_64F, &m1);
	const cv::Mat T(3, 1, CV_64F, &rcTrans);

	const cv::Mat K1(3, 3, CV_64F, ipl);
	const cv::Mat K2(3, 3, CV_64F, ipr);
	const cv::Mat D1(1, 5, CV_64F, dpl);
	const cv::Mat D2(1, 5, CV_64F, dpr);

	double _R1[3][3], _R2[3][3], _P1[3][4], _P2[3][4], _Q[4][4];
	cv::Mat R1(3, 3, CV_64F, _R1);
	cv::Mat R2(3, 3, CV_64F, _R2);
	cv::Mat P1(3, 4, CV_64F, _P1);
	cv::Mat P2(3, 4, CV_64F, _P2);
	cv::Mat Q(4, 4, CV_64F, _Q);

	const cv::Size img_size(ncols, nrows);
	const cv::Size real_trg_size =
		m_resize_output ? trg_size
						: img_size;  // Note: trg_size is Size() by default

	// OpenCV 2.3+ has this signature:
	cv::stereoRectify(
		K1, D1, K2, D2, img_size, R, T, R1, R2, P1, P2, Q,
		m_enable_both_centers_coincide ? cv::CALIB_ZERO_DISPARITY : 0, m_alpha,
		trg_size  // Size() by default=no resize
	);
	// Rest of arguments -> default

	cv::initUndistortRectifyMap(
		K1, D1, R1, P1, real_trg_size, CV_16SC2, _mapx_left, _mapy_left);
	cv::initUndistortRectifyMap(
		K2, D2, R2, P2, real_trg_size, CV_16SC2, _mapx_right, _mapy_right);

	// Populate the parameter matrices of the output rectified images:
	for (unsigned int i = 0; i < 3; ++i)
		for (unsigned int j = 0; j < 3; ++j)
		{
			m_rectified_image_params.leftCamera.intrinsicParams(i, j) =
				_P1[i][j];
			m_rectified_image_params.rightCamera.intrinsicParams(i, j) =
				_P2[i][j];
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
	m_rectified_image_params.leftCamera.focalLengthMeters =
		params.leftCamera.focalLengthMeters;
	m_rectified_image_params.rightCamera.focalLengthMeters =
		params.rightCamera.focalLengthMeters;

	// R1: Rotation of left camera after rectification:
	// R2: idem for right cam:
	{
		Eigen::Matrix3d Re;
		cv::cv2eigen(R1, Re);
		CPose3D RR1;
		RR1.setRotationMatrix(CMatrixDouble33(Re));
		m_rot_left = CPose3DQuat(RR1);
	}
	{
		Eigen::Matrix3d Re;
		cv::cv2eigen(R2, Re);
		CPose3D RR;
		RR.setRotationMatrix(CMatrixDouble33(Re));
		m_rot_right = CPose3DQuat(RR);
	}

	m_rectified_image_params.rightCameraPose = params.rightCameraPose;

#else
	THROW_EXCEPTION("MRPT built without OpenCV >=2.0.0!");
#endif
	MRPT_END
}

void CStereoRectifyMap::rectify(
	const mrpt::img::CImage& in_left_image,
	const mrpt::img::CImage& in_right_image, mrpt::img::CImage& out_left_image,
	mrpt::img::CImage& out_right_image) const
{
	MRPT_START

#if MRPT_HAS_OPENCV
	const uint32_t ncols = m_camera_params.leftCamera.ncols;
	const uint32_t nrows = m_camera_params.leftCamera.nrows;

	const CvSize trg_size =
		m_resize_output
			? cvSize(m_resize_output_value.x, m_resize_output_value.y)
			: cvSize(ncols, nrows);

	out_left_image.resize(
		trg_size.width, trg_size.height, in_left_image.getChannelCount());
	out_right_image.resize(
		trg_size.width, trg_size.height, in_left_image.getChannelCount());

	const cv::Mat in_left = in_left_image.asCvMat<cv::Mat>(SHALLOW_COPY);
	const cv::Mat in_right = in_right_image.asCvMat<cv::Mat>(SHALLOW_COPY);

	cv::Mat& out_left = out_left_image.asCvMatRef();
	cv::Mat& out_right = out_right_image.asCvMatRef();

	do_rectify(
		*this, in_left, in_right, out_left, out_right,
		const_cast<int16_t*>(&m_dat_mapx_left[0]),
		const_cast<int16_t*>(&m_dat_mapx_right[0]),
		const_cast<uint16_t*>(&m_dat_mapy_left[0]),
		const_cast<uint16_t*>(&m_dat_mapy_right[0]),
		static_cast<int>(m_interpolation_method));

#endif
	MRPT_END
}

void CStereoRectifyMap::rectify(
	mrpt::obs::CObservationStereoImages& o,
	const bool use_internal_mem_cache) const
{
	MRPT_START
	ASSERT_(o.hasImageRight);

	// Rectify images:
	if (use_internal_mem_cache)
	{
		static mrpt::img::CImage left_rect, right_rect;
		this->rectify(o.imageLeft, o.imageRight, left_rect, right_rect);
		o.imageLeft = left_rect;
		o.imageRight = right_rect;
	}
	else
	{
		mrpt::img::CImage left_rect, right_rect;
		this->rectify(o.imageLeft, o.imageRight, left_rect, right_rect);
		o.imageLeft = left_rect;
		o.imageRight = right_rect;
	}

	// Copy output image parameters:
	o.setStereoCameraParams(this->m_rectified_image_params);

	// Correct poses:
	o.cameraPose += m_rot_left;

	const double d = o.rightCameraPose.m_coords.norm();
	// the translation is now pure in the +X direction:
	o.rightCameraPose = CPose3DQuat(d, .0, .0, mrpt::math::CQuaternionDouble());

	MRPT_END
}

const mrpt::img::TStereoCamera& CStereoRectifyMap::getRectifiedImageParams()
	const
{
	if (!isSet())
		THROW_EXCEPTION("Error: setFromCamParams() must be called before.");
	return m_rectified_image_params;
}

const mrpt::img::TCamera& CStereoRectifyMap::getRectifiedLeftImageParams() const
{
	if (!isSet())
		THROW_EXCEPTION("Error: setFromCamParams() must be called before.");
	return m_rectified_image_params.leftCamera;
}

const mrpt::img::TCamera& CStereoRectifyMap::getRectifiedRightImageParams()
	const
{
	if (!isSet())
		THROW_EXCEPTION("Error: setFromCamParams() must be called before.");
	return m_rectified_image_params.rightCamera;
}

void CStereoRectifyMap::setRectifyMaps(
	const std::vector<int16_t>& left_x, const std::vector<uint16_t>& left_y,
	const std::vector<int16_t>& right_x, const std::vector<uint16_t>& right_y)
{
	m_dat_mapx_left.resize(left_x.size());
	m_dat_mapy_left.resize(left_y.size());
	m_dat_mapx_right.resize(right_x.size());
	m_dat_mapy_right.resize(right_y.size());

	std::copy(left_x.begin(), left_x.end(), m_dat_mapx_left.begin());
	std::copy(left_y.begin(), left_y.end(), m_dat_mapy_left.begin());
	std::copy(right_x.begin(), right_x.end(), m_dat_mapx_right.begin());
	std::copy(right_y.begin(), right_y.end(), m_dat_mapy_right.begin());
}

void CStereoRectifyMap::setRectifyMapsFast(
	std::vector<int16_t>& left_x, std::vector<uint16_t>& left_y,
	std::vector<int16_t>& right_x, std::vector<uint16_t>& right_y)
{
	left_x.swap(m_dat_mapx_left);
	left_y.swap(m_dat_mapy_left);
	right_x.swap(m_dat_mapx_right);
	right_y.swap(m_dat_mapy_right);
}
