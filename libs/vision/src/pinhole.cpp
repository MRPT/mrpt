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


#include <mrpt/vision/pinhole.h>


#if MRPT_HAS_OPENCV
	// OPENCV HEADERS
	#define CV_NO_CVV_IMAGE   // Avoid CImage name crash


#	if MRPT_OPENCV_VERSION_NUM>=0x211
#		include <opencv2/core/core.hpp>
#		include <opencv2/highgui/highgui.hpp>
#		include <opencv2/imgproc/imgproc.hpp>
#		include <opencv2/imgproc/imgproc_c.h>

#		include <opencv2/calib3d/calib3d.hpp>
#	else
#		include <cv.h>
#		include <highgui.h>
#	endif

	#ifdef CImage	// For old OpenCV versions (<=1.0.0)
	#undef CImage
	#endif
#endif // MRPT_HAS_OPENCV

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;


/* -------------------------------------------------------
				projectPoints_no_distortion
   ------------------------------------------------------- */
void mrpt::vision::pinhole::projectPoints_no_distortion(
	const std::vector<mrpt::poses::CPoint3D> &in_points_3D,
	const mrpt::poses::CPose3D &cameraPose,
	const mrpt::math::CMatrixDouble33 & intrinsicParams,
	std::vector<TPixelCoordf> &projectedPoints,
	bool accept_points_behind )
{
	MRPT_START

	// Do NOT distort points:
	static const vector_double distortion_dummy(4,0);

	projectPoints_with_distortion(
		in_points_3D,
		cameraPose,
		intrinsicParams,
		distortion_dummy,
		projectedPoints,
		accept_points_behind );
	MRPT_END
}


/* -------------------------------------------------------
				projectPoints_with_distortion
   ------------------------------------------------------- */
void mrpt::vision::pinhole::projectPoints_with_distortion(
	const std::vector<mrpt::poses::CPoint3D> &in_points_3D,
	const mrpt::poses::CPose3D &cameraPose,
	const mrpt::math::CMatrixDouble33 & intrinsicParams,
	const std::vector<double> & distortionParams,
	std::vector<mrpt::vision::TPixelCoordf> &projectedPoints,
	bool accept_points_behind
	)
{
	MRPT_START
#if MRPT_HAS_OPENCV

	ASSERT_(size(intrinsicParams,1)==3);
	ASSERT_(size(intrinsicParams,2)==3);
	ASSERT_(distortionParams.size()==4 || distortionParams.size()==5);

	const size_t N = in_points_3D.size();
	projectedPoints.resize(N);

	if (!N) return;  // Nothing to do

	vector<CvPoint3D64f>  objPoints(N);
	vector<CvPoint2D64f>  imgPoints(N);

	// generate points relative to camera:
	for (size_t i=0;i<N;i++)
	{
		CPoint3D  pt_rel_to_cam = in_points_3D[i] - cameraPose;
		objPoints[i].x = pt_rel_to_cam.x();
		objPoints[i].y = pt_rel_to_cam.y();
		objPoints[i].z = pt_rel_to_cam.z();
	}

	// Points are already translated & rotated:
	static double rotation_matrix[] = {1,0,0, 0,1,0, 0,0,1 };
	static double translation_vector[] = {0,0,0 };

	// Projection matrix:
	//   0 1 2
	//   3 4 5
	//   6 7 8
	vector_double proj_matrix(9);
	proj_matrix[0] = intrinsicParams.get_unsafe(0,0);
	proj_matrix[4] = intrinsicParams.get_unsafe(1,1);
	proj_matrix[2] = intrinsicParams.get_unsafe(0,2);
	proj_matrix[5] = intrinsicParams.get_unsafe(1,2);

	// Do the projection:
	cvProjectPointsSimple(
		N, // int point_count,
		&objPoints[0],
		rotation_matrix,
		translation_vector,
		&proj_matrix[0],
		const_cast<double*>(&distortionParams[0]),
		&imgPoints[0] );

	for (size_t i=0;i<N;i++)
	{
		if (accept_points_behind || objPoints[i].z>0 )
		{  // Valid point or we accept them:
			projectedPoints[i].x = imgPoints[i].x;
			projectedPoints[i].y = imgPoints[i].y;
		}
		else
		{ 	// Invalid point behind the camera:
			projectedPoints[i].x = -1;
			projectedPoints[i].y = -1;
		}
	}

#else
	THROW_EXCEPTION("Function not available: MRPT was compiled without OpenCV")
#endif
	MRPT_END
}



/* -------------------------------------------------------
				undistort_points
   ------------------------------------------------------- */
void mrpt::vision::pinhole::undistort_points(
	const std::vector<mrpt::vision::TPixelCoordf>  &in_dist_pixels,
	std::vector<mrpt::vision::TPixelCoordf> &out_pixels,
	const mrpt::math::CMatrixDouble33 & A,
	const std::vector<double> & Dk )
{
	MRPT_START

	ASSERT_(size(A,1)==3);
	ASSERT_(size(A,2)==3);
	ASSERT_(Dk.size()==4 || Dk.size()==5);

	vector_double k = Dk;
	if (k.size()<5) k.resize(5);

	// based on code from OpenCV 1.1.0, function cvUndistortPoints, file cvundistort.cpp
	// Jose Luis: Great code clean up wrt opencv's since we assume C++ and availability of MRPT's matrices.
	const size_t n = in_dist_pixels.size();
	out_pixels.resize(n);

    const double fx = A(0,0);
    const double fy = A(1,1);
    const double ifx = 1./fx;
    const double ify = 1./fy;
    const double cx = A(0,2);
    const double cy = A(1,2);

    for( size_t i = 0; i < n; i++ )
    {
        double x = in_dist_pixels[i].x;
		double y = in_dist_pixels[i].y;

        double x0 = x = (x - cx)*ifx;
        double y0 = y = (y - cy)*ify;

        // compensate distortion iteratively
        for( unsigned int j = 0; j < 5; j++ )
        {
            double r2 = x*x + y*y;
            double icdist = 1./(1 + ((k[4]*r2 + k[1])*r2 + k[0])*r2);
            double deltaX = 2*k[2]*x*y + k[3]*(r2 + 2*x*x);
            double deltaY = k[2]*(r2 + 2*y*y) + 2*k[3]*x*y;
            x = (x0 - deltaX)*icdist;
            y = (y0 - deltaY)*icdist;
        }

		// Save undistorted pixel coords:
		out_pixels[i].x = x;
		out_pixels[i].y = y;

    } // end for i

	MRPT_END
}
