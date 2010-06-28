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

#include <mrpt/vision/camera_calib_ba.h>
#include <mrpt/math/CLevenbergMarquardt.h>

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

// Auxiliary small functions for "camera_calib_ba":
void cam2vec(const TCamera &camPar,vector_double &x);
void vec2cam(const vector_double &x, TCamera &camPar);
void camera_calib_ba_err_func(
	const vector_double &x,
	const std::vector< std::vector<mrpt::utils::TPixelCoordf> > &y,
	vector_double &err);

/*-------------------------------------------------------------
				camera_calib_ba

	See the header for documentation
-------------------------------------------------------------*/
double mrpt::vision::camera_calib_ba(
	const std::vector< std::vector<mrpt::utils::TPixelCoordf> >  & in_tracked_feats,
	unsigned int camera_ncols,
	unsigned int camera_nrows,
	mrpt::utils::TCamera &out_optimal_params,
	TCamCalibBAResults &out_info
	)
{
	ASSERT_(camera_ncols>1 && camera_nrows>1)
	ASSERT_(in_tracked_feats.size()>1)
	ASSERT_(in_tracked_feats[0].size()>0)

	const size_t nFrames = in_tracked_feats.size();
	const size_t nFeats  = in_tracked_feats[0].size();

	// Make sure that we have the same feats in all frames:
	for (size_t i=0;i<nFrames;i++)
		if (in_tracked_feats[i].size()!=nFeats)
		{
			THROW_EXCEPTION("in_tracked_feats must contain the same number of features in each frame.")
		}

	// First, the easy part: the resolution
	TCamera  camPar;
	camPar.ncols = camera_ncols;
	camPar.nrows = camera_nrows;

	// First wild guesses of the camera params:
	camPar.intrinsicParams.zeros();
	camPar.intrinsicParams(2,2) = 1;
	camPar.intrinsicParams(0,0) = camera_ncols;  // cx
	camPar.intrinsicParams(1,1) = camera_ncols;  // cy
	camPar.intrinsicParams(0,2) = camera_ncols >> 1; // fx
	camPar.intrinsicParams(1,2) = camera_nrows >> 1; // fy


	// Launch LM optimization:
	typedef CLevenbergMarquardtTempl<vector_double, std::vector< std::vector<mrpt::utils::TPixelCoordf> > > TMyLevMar;

	TMyLevMar::TResultInfo  info;

	vector_double optimal_x;
	vector_double initial_x;

	cam2vec(camPar,initial_x);

	const size_t camParLen = initial_x.size();

	// "nFrames-1" because the first camera pose is fixed as the origin of coordinates...
	const size_t stateLen =
		camParLen +			/* camera params */
		(nFrames-1) * 7  +	/* camera poses */
		nFeats * 3;			/* features (x y z) world location */

	initial_x.resize(stateLen);

	// Need a first gross estimate of the state vector:
	// ----------------------------------------------------
	// 1) Camera params: already there.

	// 2) camera poses:
	for (size_t i=0;i<nFrames-1;i++)
	{
		initial_x[camParLen+i*7+0] = // XYZ
		initial_x[camParLen+i*7+1] =
		initial_x[camParLen+i*7+2] = 0;

		initial_x[camParLen+i*7+3] = 1.0; // qr qx qy qz
		initial_x[camParLen+i*7+4] =
		initial_x[camParLen+i*7+5] =
		initial_x[camParLen+i*7+6] = 0;
	}

	// 3) gross initial guess of landmarks positions:
	for (size_t i=0;i<nFeats;i++)
	{
		TPoint3D p = mrpt::vision::pixelTo3D( in_tracked_feats[0][i], camPar.intrinsicParams );
		p*=3.0; // a priori distance...

		initial_x[camParLen+(nFrames-1)*7+i*3+0] = p.x;
		initial_x[camParLen+(nFrames-1)*7+i*3+1] = p.y;
		initial_x[camParLen+(nFrames-1)*7+i*3+2] = p.z;
	}

	// Increments for numeric Jacobians:
	const vector_double increments_x(initial_x.size(), 1e-9);

	// Do the optimization ===================
	TMyLevMar::execute(
		optimal_x,
		initial_x,
		&camera_calib_ba_err_func,
		increments_x,
		in_tracked_feats,
		info,
		true, /* verbose */
		1000, /* max iter */
		1e-3,
		1e-9,
		1e-9,
		false
		);

	// Return the optimal params:
	vec2cam(optimal_x, out_optimal_params);

	// --------------------------------
	// Extra data:
	// --------------------------------
	out_info.camera_poses.resize(nFrames);
	for (size_t nFr = 1; nFr<nFrames;nFr++)
	{
		if (!nFr)
		{
			out_info.camera_poses[0] = CPose3DQuat();
			continue;
		}
		const double qr = optimal_x[camParLen+7*(nFr-1)+3];
		const double qx = optimal_x[camParLen+7*(nFr-1)+4];
		const double qy = optimal_x[camParLen+7*(nFr-1)+5];
		const double qz = optimal_x[camParLen+7*(nFr-1)+6];
		const double qmod_ = 1.0/std::sqrt(square(qr)+square(qx)+square(qy)+square(qz));
		out_info.camera_poses[nFr] = CPose3DQuat(
			optimal_x[camParLen+7*(nFr-1)+0],  // XYZ
			optimal_x[camParLen+7*(nFr-1)+1],
			optimal_x[camParLen+7*(nFr-1)+2],
			CQuaternionDouble( qr*qmod_,qx*qmod_,qy*qmod_,qz*qmod_ )
			);
	}
	out_info.landmark_positions.resize(nFeats);
	for (size_t nFe = 0; nFe<nFeats;nFe++)
	{
		out_info.landmark_positions[nFe] = TPoint3D(
			optimal_x[camParLen+7*(nFrames-1)+3*nFe+0],
			optimal_x[camParLen+7*(nFrames-1)+3*nFe+1],
			optimal_x[camParLen+7*(nFrames-1)+3*nFe+2] );
	}

	// Average error:
	return std::sqrt( info.final_sqr_err / nFrames * nFeats );
}


// vectors: 1x9: [fx fy cx cy k1 k2 p1 p2 k3]
void cam2vec(const TCamera &camPar,vector_double &x)
{
	if (x.size()<9) x.resize(9);

	x[0] = camPar.fx();
	x[1] = camPar.fy();
	x[2] = camPar.cx();
	x[3] = camPar.cy();

	for (size_t i=0;i<camPar.dist.static_size;i++)
		x[4+i] = camPar.dist[i];
}

void vec2cam(const vector_double &x, TCamera &camPar)
{
	camPar.intrinsicParams(0,0) = x[0]; // fx
	camPar.intrinsicParams(1,1) = x[1]; // fy
	camPar.intrinsicParams(0,2) = x[2]; // cx
	camPar.intrinsicParams(1,2) = x[3]; // cy

	for (size_t i=0;i<camPar.dist.static_size;i++)
		camPar.dist[i] = x[4+i];
}

void camera_calib_ba_err_func(
	const vector_double &x,
	const std::vector< std::vector<mrpt::utils::TPixelCoordf> > &y,
	vector_double &err)
{
	const size_t nFrames = y.size();
	const size_t nFeats  = y[0].size();

	const size_t camParLen = 4+5;
	ASSERT_( x.size() == camParLen + (nFrames-1) * 7 + nFeats * 3 )

	// Recover camera params:
	TCamera  camPars;
	vec2cam(x,camPars);

	// Compute reprojection errors (x,y) for each feature.
	err.clear();
	err.reserve(nFrames * nFeats * 2 );

	double total_err2 = 0;

	for (size_t nFr = 0; nFr<nFrames;nFr++)
	{
		CPose3DQuat  camPose;
		if (nFr>0)
		{
			const double qr = x[camParLen+7*(nFr-1)+3];
			const double qx = x[camParLen+7*(nFr-1)+4];
			const double qy = x[camParLen+7*(nFr-1)+5];
			const double qz = x[camParLen+7*(nFr-1)+6];
			const double qmod_ = 1.0/std::sqrt(square(qr)+square(qx)+square(qy)+square(qz));

			camPose = CPose3DQuat(
				x[camParLen+7*(nFr-1)+0],  // XYZ
				x[camParLen+7*(nFr-1)+1],
				x[camParLen+7*(nFr-1)+2],
				CQuaternionDouble( qr*qmod_,qx*qmod_,qy*qmod_,qz*qmod_ )
				);
		}
		// else -> default is at the origin.

		for (size_t nFe = 0; nFe<nFeats;nFe++)
		{
			const TPoint3D world_point(
				x[camParLen+7*(nFrames-1)+3*nFe+0],
				x[camParLen+7*(nFrames-1)+3*nFe+1],
				x[camParLen+7*(nFrames-1)+3*nFe+2] );

			// ptWrtCam = world_point (-) camPose
			TPoint3D ptWrtCam;
			camPose.inverseComposePoint(world_point, ptWrtCam);

			TPixelCoordf reprojPoint;
			mrpt::vision::pinhole::projectPoint_with_distortion(ptWrtCam,camPars,reprojPoint, true );

			const double errx = reprojPoint.x - y[nFr][nFe].x;
			const double erry = reprojPoint.y - y[nFr][nFe].y;

			err.push_back( errx );
			err.push_back( erry );

			total_err2+=square(errx)+square(erry);
		} // end nFe
	} // end nFr

	//cout << "err: " << err << endl;
	//cout << format("Avr reproj error: %.015f px\n", std::sqrt(total_err2/(nFrames*nFeats)) );
}

