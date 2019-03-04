/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "vision-precomp.h"  // Precompiled headers
#include <mrpt/vision/CUndistortMap.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::img;

// Ctor: Leave all vectors empty
CUndistortMap::CUndistortMap() = default;
/** Prepares the mapping from the distortion parameters of a camera.
 * Must be called before invoking \a undistort().
 */
void CUndistortMap::setFromCamParams(const mrpt::img::TCamera& campar)
{
	MRPT_START
#if MRPT_HAS_OPENCV
	m_camera_params = campar;

	// Convert to opencv's format:
	double aux1[3][3], aux2[1][5];
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++) aux1[i][j] = campar.intrinsicParams(i, j);
	for (int i = 0; i < 5; i++) aux2[0][i] = campar.dist[i];

	const cv::Mat inMat(3, 3, CV_64F, aux1);
	const cv::Mat distM(1, 5, CV_64F, aux2);

	m_dat_mapx.resize(2 * campar.nrows * campar.ncols);
	m_dat_mapy.resize(campar.nrows * campar.ncols);

	cv::Mat mapx(campar.nrows, campar.ncols, CV_16SC2, &m_dat_mapx[0]);
	cv::Mat mapy(campar.nrows, campar.ncols, CV_16UC1, &m_dat_mapy[0]);

	cv::initUndistortRectifyMap(
		inMat, distM, cv::Mat(), inMat, mapx.size(), mapx.type(), mapx, mapy);
#else
	THROW_EXCEPTION("MRPT built without OpenCV >=2.0.0!");
#endif
	MRPT_END
}

/** Undistort the input image and saves the result in-place- \a
 * setFromCamParams() must have been set prior to calling this.
 */
void CUndistortMap::undistort(
	const mrpt::img::CImage& in_img, mrpt::img::CImage& out_img) const
{
	MRPT_START
	if (m_dat_mapx.empty())
		THROW_EXCEPTION(
			"Error: setFromCamParams() must be called prior to undistort().");

#if MRPT_HAS_OPENCV
	using namespace cv;
	Mat mapx(
		m_camera_params.nrows, m_camera_params.ncols, CV_16SC2,
		const_cast<int16_t*>(&m_dat_mapx[0]));
	Mat mapy(
		m_camera_params.nrows, m_camera_params.ncols, CV_16UC1,
		const_cast<uint16_t*>(&m_dat_mapy[0]));

	out_img.resize(
		in_img.getWidth(), in_img.getHeight(), in_img.getChannelCount());

	cv::remap(
		in_img.asCvMat<Mat>(SHALLOW_COPY), out_img.asCvMat<Mat>(SHALLOW_COPY),
		mapx, mapy, INTER_LINEAR);
#endif
	MRPT_END
}

/** Undistort the input image and saves the result in-place- \a
 * setFromCamParams() must have been set prior to calling this.
 */
void CUndistortMap::undistort(mrpt::img::CImage& in_out_img) const
{
	MRPT_START
	if (m_dat_mapx.empty())
		THROW_EXCEPTION(
			"Error: setFromCamParams() must be called prior to undistort().");

#if MRPT_HAS_OPENCV
	cv::Mat mapx(
		m_camera_params.nrows, m_camera_params.ncols, CV_16SC2,
		const_cast<int16_t*>(&m_dat_mapx[0]));
	cv::Mat mapy(
		m_camera_params.nrows, m_camera_params.ncols, CV_16UC1,
		const_cast<uint16_t*>(&m_dat_mapy[0]));

	cv::Mat in = in_out_img.asCvMat<cv::Mat>(SHALLOW_COPY);
	cv::Mat out(in.size(), in.type());

	cv::remap(in, out, mapx, mapy, cv::INTER_LINEAR);

	in_out_img = CImage(out, SHALLOW_COPY);
#endif
	MRPT_END
}
