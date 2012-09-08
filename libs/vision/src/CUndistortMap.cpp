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
#include <mrpt/vision/CUndistortMap.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 

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

	const IplImage *srcImg = in_img.getAs<IplImage>();	// Source Image
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

	const IplImage *srcImg = in_out_img.getAs<IplImage>();	// Source Image
	IplImage *outImg = cvCreateImage( cvGetSize( srcImg ), srcImg->depth, srcImg->nChannels );
	cvRemap(srcImg, outImg, &mapx, &mapy);	//cv::remap(src, dst_part, map1_part, map2_part, INTER_LINEAR, BORDER_CONSTANT );
	in_out_img.setFromIplImage(outImg);
#endif
	MRPT_END
}

