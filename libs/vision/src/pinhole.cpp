/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers


#include <mrpt/vision/pinhole.h>


// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::vision::pinhole;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;


/* -------------------------------------------------------
				projectPoints_no_distortion
   ------------------------------------------------------- */
void mrpt::vision::pinhole::projectPoints_no_distortion(
	const std::vector<mrpt::math::TPoint3D> &in_points_3D,
	const mrpt::poses::CPose3D &cameraPose,
	const mrpt::math::CMatrixDouble33 & intrinsicParams,
	std::vector<TPixelCoordf> &projectedPoints,
	bool accept_points_behind )
{
	MRPT_START

	// Do NOT distort points:
	static const std::vector<double> distortion_dummy(4,0);

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
	const std::vector<mrpt::math::TPoint3D> &in_points_3D,
	const mrpt::poses::CPose3D &cameraPose,
	const mrpt::math::CMatrixDouble33 & intrinsicParams,
	const std::vector<double> & distortionParams,
	std::vector<mrpt::utils::TPixelCoordf> &projectedPoints,
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

	// generate points relative to camera:
	for (size_t i=0;i<N;i++)
		cameraPose.inverseComposePoint(
			in_points_3D[i].x,in_points_3D[i].y,in_points_3D[i].z,
			objPoints[i].x,objPoints[i].y,objPoints[i].z
			);

	// Points are already translated & rotated:
	static double rotation_matrix[] = {1,0,0, 0,1,0, 0,0,1 };
	static double translation_vector[] = {0,0,0 };

	// Projection matrix:
	//   0 1 2
	//   3 4 5
	//   6 7 8
	CVectorDouble proj_matrix(9);
	proj_matrix[0] = intrinsicParams.get_unsafe(0,0);
	proj_matrix[4] = intrinsicParams.get_unsafe(1,1);
	proj_matrix[2] = intrinsicParams.get_unsafe(0,2);
	proj_matrix[5] = intrinsicParams.get_unsafe(1,2);

	// Do the projection:
	cv::Mat object_points = cv::Mat( N, 1, CV_64FC3, &objPoints[0] );
	
	cv::Mat rotvec;
	cv::Rodrigues( cv::Mat( 3, 3, CV_64FC1, rotation_matrix ), rotvec);

	cv::Mat _translation_vector = cv::Mat( 3, 1, CV_64FC1, translation_vector );
	cv::Mat camera_matrix = cv::Mat( 3, 3, CV_64FC1, &proj_matrix[0] );
	cv::Mat dist_coeffs = cv::Mat( 5, 1, CV_64FC1, const_cast<double*>(&distortionParams[0]) );

	vector<cv::Point2d> image_points;

	cv::projectPoints(
		object_points, rotvec, _translation_vector,
		camera_matrix, dist_coeffs, image_points );

	for (size_t i=0;i<N;i++)
	{
		if (accept_points_behind || objPoints[i].z>0 )
		{  // Valid point or we accept them:
			projectedPoints[i].x = image_points[i].x;
			projectedPoints[i].y = image_points[i].y;
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
	const std::vector<mrpt::utils::TPixelCoordf>  &in_dist_pixels,
	std::vector<mrpt::utils::TPixelCoordf> &out_pixels,
	const mrpt::math::CMatrixDouble33 & A,
	const std::vector<double> & Dk )
{	// Hub function:
	TCamera cam;
	cam.intrinsicParams = A;
	ASSERT_(Dk.size()<=cam.dist.static_size)
	for (size_t i=0;i<cam.dist.static_size;i++) cam.dist[i] = Dk[i];
	undistort_points(in_dist_pixels,out_pixels,cam);
}

void mrpt::vision::pinhole::undistort_points(
	const std::vector<mrpt::utils::TPixelCoordf>  &in_dist_pixels,
	std::vector<mrpt::utils::TPixelCoordf> &out_pixels,
	const mrpt::utils::TCamera  &cameraModel)
{
	MRPT_START

	// based on code from OpenCV 1.1.0, function cvUndistortPoints, file cvundistort.cpp
	// Jose Luis: Great code clean up wrt opencv's since we assume C++ and availability of MRPT's matrices.
	const size_t n = in_dist_pixels.size();
	out_pixels.resize(n);

    const double fx = cameraModel.fx();
    const double fy = cameraModel.fy();
    const double ifx = 1./fx;
    const double ify = 1./fy;
    const double cx = cameraModel.cx();
    const double cy = cameraModel.cy();

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
            double icdist = 1./(1 + ((cameraModel.dist[4]*r2 + cameraModel.dist[1])*r2 + cameraModel.dist[0])*r2);
            double deltaX = 2*cameraModel.dist[2]*x*y + cameraModel.dist[3]*(r2 + 2*x*x);
            double deltaY = cameraModel.dist[2]*(r2 + 2*y*y) + 2*cameraModel.dist[3]*x*y;
            x = (x0 - deltaX)*icdist;
            y = (y0 - deltaY)*icdist;
        }

		// Save undistorted pixel coords:
		out_pixels[i].x = x*fx + cx;
		out_pixels[i].y = y*fy + cy;

    } // end for i

	MRPT_END
}

/** Undistort one point given by its pixel coordinates and the camera parameters.
  * \sa undistort_points
  */
void mrpt::vision::pinhole::undistort_point(
	const TPixelCoordf  &inPt,
	TPixelCoordf        &outPt,
	const mrpt::utils::TCamera  &cameraModel)
{
	MRPT_START


	// based on code from OpenCV 1.1.0, function cvUndistortPoints, file cvundistort.cpp
	// Jose Luis: Great code clean up wrt opencv's since we assume C++ and availability of MRPT's matrices.
    const double fx = cameraModel.fx();
    const double fy = cameraModel.fy();
    const double ifx = 1./fx;
    const double ify = 1./fy;
    const double cx = cameraModel.cx();
    const double cy = cameraModel.cy();

	double x = inPt.x;
	double y = inPt.y;

	double x0 = x = (x - cx)*ifx;
	double y0 = y = (y - cy)*ify;

	// compensate distortion iteratively
	for( unsigned int j = 0; j < 5; j++ )
	{
		double r2 = x*x + y*y;
		double icdist = 1./(1 + ((cameraModel.dist[4]*r2 + cameraModel.dist[1])*r2 + cameraModel.dist[0])*r2);
		double deltaX = 2*cameraModel.dist[2]*x*y + cameraModel.dist[3]*(r2 + 2*x*x);
		double deltaY = cameraModel.dist[2]*(r2 + 2*y*y) + 2*cameraModel.dist[3]*x*y;
		x = (x0 - deltaX)*icdist;
		y = (y0 - deltaY)*icdist;
	}

	// Save undistorted pixel coords:
	outPt.x = x*fx + cx;
	outPt.y = y*fy + cy;

	MRPT_END
}


void mrpt::vision::pinhole::projectPoints_with_distortion(
	const std::vector<mrpt::math::TPoint3D>  &P,
	const mrpt::utils::TCamera  &params,
	const mrpt::poses::CPose3DQuat &cameraPose,
	std::vector<mrpt::utils::TPixelCoordf>  &pixels,
	bool accept_points_behind
	)
{
	MRPT_START

	pixels.resize( P.size() );
	std::vector<mrpt::math::TPoint3D>::const_iterator itPoints;
	std::vector<mrpt::utils::TPixelCoordf>::iterator itPixels;
	unsigned int k = 0;
	for( itPoints = P.begin(), itPixels = pixels.begin(); itPoints != P.end(); ++itPoints, ++itPixels, ++k )
	{
		// Change the reference system to that wrt the camera
		TPoint3D nP;
		cameraPose.inverseComposePoint( itPoints->x, itPoints->y, itPoints->z, nP.x, nP.y, nP.z );

		// Pinhole model:
		const double x = nP.x/nP.z;
		const double y = nP.y/nP.z;

		// Radial distortion:
		const double r2 = square(x)+square(y);
		const double r4 = square(r2);
		const double r6 = r2*r4;
		const double A  = 1+params.dist[0]*r2+params.dist[1]*r4+params.dist[4]*r6;
		const double B  = 2*x*y;
		if( A > 0 && (accept_points_behind || nP.z > 0) )
		{
			itPixels->x = params.cx() + params.fx() * ( x*A + params.dist[2]*B + params.dist[3]*(r2+2*square(x)) );
			itPixels->y = params.cy() + params.fy() * ( y*A + params.dist[3]*B + params.dist[2]*(r2+2*square(y)) );
		}
		else
		{
			itPixels->x = -1.0;
			itPixels->y = -1.0;
		}
	} // end-for

	MRPT_END
}

/* -------------------------------------------------------
				projectPoint_with_distortion
   ------------------------------------------------------- */
void mrpt::vision::pinhole::projectPoint_with_distortion(
	const mrpt::math::TPoint3D  &P,
	const mrpt::utils::TCamera  &params,
	mrpt::utils::TPixelCoordf  &pixel,
	bool accept_points_behind
	)
{
	MRPT_UNUSED_PARAM(accept_points_behind);
	// Pinhole model:
	const double x = P.x/P.z;
	const double y = P.y/P.z;

	// Radial distortion:
	const double r2 = square(x)+square(y);
	const double r4 = square(r2);
	const double r6 = r2*r4;

	pixel.x = params.cx() + params.fx() *(  x*(1+params.dist[0]*r2+params.dist[1]*r4+params.dist[4]*r6) + 2*params.dist[2]*x*y+params.dist[3]*(r2+2*square(x))  );
	pixel.y = params.cy() + params.fy() *(  y*(1+params.dist[0]*r2+params.dist[1]*r4+params.dist[4]*r6) + 2*params.dist[3]*x*y+params.dist[2]*(r2+2*square(y))  );
}


/* -------------------------------------------------------
					undistortPixels
   ------------------------------------------------------- */
//void mrpt::vision::pinhole::undistortPixels(
//	const std::vector<mrpt::utils::TPixelCoordf>	&inputPixels, 		/* distorted pixels in image */
//	const mrpt::math::CMatrixDouble33				&intrinsicParams,	/* intrinsic parameters of the camera */
//	const std::vector<double>						&distortionParams,	/* k1 k2 p1 p2 */
//	const unsigned int								&resX,				/* X-resolution of the image */
//	const unsigned int								&resY,				/* Y-resolution of the image */
//	const double									&pixelSize,			/* pixel size (square)*/
//	std::vector<mrpt::utils::TPixelCoordf>			&outputPixels		/* estimated undistorted pixels in image */
//    )
//{
//	MRPT_START
//
//	ASSERT_( distortionParams.size() >= 4 );
//	const double k1 = distortionParams[0];
//	const double k2 = distortionParams[1];
//	const double p1 = distortionParams[2];
//	const double p2 = distortionParams[3];
//
//	const double fx = intrinsicParams(0,0);
//	const double fy = intrinsicParams(1,1);
//	const double cx = intrinsicParams(0,2);
//	const double cy = intrinsicParams(1,2);
//
//	CMatrixFixedNumeric<double,43,43> dx, dy;
//
//	// Compute the undistortion params according to Heittilä code.
//	// Generate a regular meshgrid of size 43x43 and distort them
//	std::vector<mrpt::utils::TPixelCoordf>				grid;			// The 43x43 grid with distorted
//	std::vector<mrpt::utils::TPixelCoordf>::iterator	itGrid;
//
//	grid.resize( 43 );
//	unsigned int c;
//	double px, py;
//	for( c = 0, itGrid = grid.begin(); itGrid != grid.end(); ++itGrid )
//	{
//		px = -resX/40 + c*resX/40;
//		for( unsigned int k = 0; k < 43; ++k )
//		{
//			py = -resY/40 + k*resY/40;
//			const double dx = ( px - cx )*pixelSize;
//			const double dy = ( py - cy )*pixelSize;
//
//			const double r2 = dx*dx + dy*dy;
//			const double delta = k1*r2 + k2*r2*r2;
//
//			const double ncx = dx*(1+delta)+2*p1*dx*dy+p2*(r2+2*dx*dx);
//			const double ncy = dy*(1+delta)+p1*(r2+2*dy*dy)+2*p2*dx*dy;
//
//			(*itGrid)->x = ncx/pixelSize + cx;
//			(*itGrid)->y = ncy/pixelSize + cy;
//		}
//	} // end-itGrid
//
//	// DISTORT POINTS
//	dx=(dp(:,1)-Cpx)*Sx/NDX/Asp;
//	dy=(dp(:,2)-Cpy)*Sy/NDY;
//
//	r2=dx.*dx+dy.*dy;
//	delta=Rad1*r2+Rad2*r2.*r2;
//
//	cx=dx.*(1+delta)+2*Tan1*dx.*dy+Tan2*(r2+2*dx.*dx);
//	cy=dy.*(1+delta)+Tan1*(r2+2*dy.*dy)+2*Tan2*dx.*dy;
//
//	p=NDX*Asp*cx/Sx+Cpx;
//	p(:,2)=NDY*cy/Sy+Cpy;
//
//
//	sys=configc(name);
//	NDX=sys(1); NDY=sys(2); Sx=sys(3); Sy=sys(4);
//	Asp=par(1); Foc=par(2);
//	Cpx=par(3); Cpy=par(4);
//	Rad1=par(5); Rad2=par(6);
//	Tan1=par(7); Tan2=par(8);
//
//	// Generate a meshgrid of points
//	[dx,dy]=meshgrid(-NDX/40:NDX/40:NDX+NDX/40,-NDY/40:NDY/40:NDY+NDY/40);
//	cc=imcorr(name,par,[dx(:) dy(:)]);
//	cx=(cc(:,1)-Cpx)/NDX*Sx/Asp;
//	cy=(cc(:,2)-Cpy)/NDY*Sy;
//
//	r2=cx.*cx+cy.*cy;
//	delta=Rad1*r2+Rad2*r2.*r2;
//
//	Q=1+(4*Rad1*r2+6*Rad2*r2.*r2+8*Tan1*cy+8*Tan2*cx);
//
//	dx=cx-(cx.*delta+2*Tan1*cx.*cy+Tan2*(r2+2*cx.*cx))./Q;
//	dy=cy-(cy.*delta+Tan1*(r2+2*cy.*cy)+2*Tan2*cx.*cy)./Q;
//
//
//	r2=dx.*dx+dy.*dy;
//
//	Tx=[dx.*r2 dx.*r2.*r2 2*dx.*dy r2+2*dx.*dx];
//	Ty=[dy.*r2 dy.*r2.*r2 r2+2*dy.*dy 2*dx.*dy];
//	T=[Tx;Ty];
//	e=[cx-dx;cy-dy];
//	a=pinv(T)*e;
//	par=par(:);
//	a=[par(1:4);a];
//
//	MRPT_END
//}



