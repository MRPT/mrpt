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

#include <mrpt/obs.h>   // Precompiled headers



#include <mrpt/slam/CObservation3DRangeScan.h>
#include <mrpt/poses/CPosePDF.h>

#include <mrpt/math/utils.h>

using namespace std;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservation3DRangeScan, CObservation,mrpt::slam)

/*---------------------------------------------------------------
							Constructor
 ---------------------------------------------------------------*/
CObservation3DRangeScan::CObservation3DRangeScan( ) :
	hasPoints3D(false),
	hasRangeImage(false),
	hasIntensityImage(false),
	hasConfidenceImage(false),
	cameraParams(),
	maxRange( 5.0f ),
	sensorPose(),
	stdError( 0.01f )
{
}

/*---------------------------------------------------------------
							Destructor
 ---------------------------------------------------------------*/
CObservation3DRangeScan::~CObservation3DRangeScan()
{
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservation3DRangeScan::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 2;
	else
	{
		// The data
		out << maxRange << sensorPose;

		// Old in v0:
		// uint32_t	N = scan_x.size();
		// out << N;
		//	out.WriteBuffer( &scan_x[0], sizeof(scan_x[0])*N );
		//	out.WriteBuffer( &scan_y[0], sizeof(scan_y[0])*N );
		//	out.WriteBuffer( &scan_z[0], sizeof(scan_z[0])*N );
		//	out.WriteBuffer( &validRange[0],sizeof(validRange[0])*N );

		out << hasPoints3D;
		if (hasPoints3D)
		{
			uint32_t N = points3D_x.size();
			out << N;
			if (N)
			{
				out.WriteBuffer( &points3D_x[0], sizeof(points3D_x[0])*N );
				out.WriteBuffer( &points3D_y[0], sizeof(points3D_y[0])*N );
				out.WriteBuffer( &points3D_z[0], sizeof(points3D_z[0])*N );
			}
		}

		out << hasRangeImage; if (hasRangeImage) out << rangeImage;
		out << hasIntensityImage; if (hasIntensityImage)  out << intensityImage;
		out << hasConfidenceImage; if (hasConfidenceImage) out << confidenceImage;

		out << cameraParams; // New in v2

		out << stdError;
		out << timestamp;
		out << sensorLabel;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservation3DRangeScan::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
	case 2:
		{
			uint32_t		N;

			in >> maxRange >> sensorPose;

			if (version>0)
				in >> hasPoints3D;
			else hasPoints3D = true;

			if (hasPoints3D)
			{
				in >> N;
				points3D_x.resize(N);
				points3D_y.resize(N);
				points3D_z.resize(N);

				if (N)
				{
					in.ReadBuffer( &points3D_x[0], sizeof(points3D_x[0])*N);
					in.ReadBuffer( &points3D_y[0], sizeof(points3D_x[0])*N);
					in.ReadBuffer( &points3D_z[0], sizeof(points3D_x[0])*N);

					if (version==0)
					{
						vector<char> validRange(N);  // for v0.
						in.ReadBuffer( &validRange[0], sizeof(validRange[0])*N );
					}
				}
			}
			else
			{
				points3D_x.clear();
				points3D_y.clear();
				points3D_z.clear();
			}

			if (version>=1)
			{
				in >> hasRangeImage;
				if (hasRangeImage)
					in >> rangeImage;

				in >> hasIntensityImage;
				if (hasIntensityImage)
					in >>intensityImage;

				in >> hasConfidenceImage;
				if (hasConfidenceImage)
					in >> confidenceImage;

				if (version>=2)
				{
					in >> cameraParams;
				}
			}

			in >> stdError;
			in >> timestamp;
			in >> sensorLabel;

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

void CObservation3DRangeScan::swap(CObservation3DRangeScan &o)
{
	CObservation::swap(o);

	std::swap(hasPoints3D,o.hasPoints3D);
	points3D_x.swap(o.points3D_x);
	points3D_y.swap(o.points3D_y);
	points3D_z.swap(o.points3D_z);

	std::swap(hasRangeImage,o.hasRangeImage);
	rangeImage.swap(o.rangeImage);

	std::swap(hasIntensityImage,o.hasIntensityImage);
	intensityImage.swap(o.intensityImage);

	std::swap(hasConfidenceImage,o.hasConfidenceImage);
	confidenceImage.swap(o.confidenceImage);

	std::swap(maxRange, o.maxRange);
	std::swap(sensorPose, o.sensorPose);
	std::swap(stdError, o.stdError);

	std::swap(cameraParams,o.cameraParams);
}

void CObservation3DRangeScan::unload()
{
	intensityImage.unload();
	confidenceImage.unload();
}


// ==============  Auxiliary function for "recoverCameraCalibrationParameters"  =========================

#define CALIB_DECIMAT  10

namespace mrpt
{
	namespace slam
	{
		namespace detail
		{
			struct TLevMarData
			{
				const CObservation3DRangeScan &obs;
				const double z_offset;
				TLevMarData(const CObservation3DRangeScan &obs_, const double z_offset_ ) : 
					obs(obs_),z_offset(z_offset_) {} 
			};

			void cam2vec(const TCamera &camPar,vector_double &x)
			{
				if (x.size()<4+4) x.resize(4+4);

				x[0] = camPar.fx();
				x[1] = camPar.fy();
				x[2] = camPar.cx();
				x[3] = camPar.cy();

				for (size_t i=0;i<4;i++)
					x[4+i] = camPar.dist[i];
			}
			void vec2cam(const vector_double &x, TCamera &camPar)
			{
				camPar.intrinsicParams(0,0) = x[0]; // fx
				camPar.intrinsicParams(1,1) = x[1]; // fy
				camPar.intrinsicParams(0,2) = x[2]; // cx
				camPar.intrinsicParams(1,2) = x[3]; // cy

				for (size_t i=0;i<4;i++)
					camPar.dist[i] = x[4+i];
			}
			void cost_func(
				const vector_double &par,
				const TLevMarData &d,
				vector_double &err)
			{
				const CObservation3DRangeScan &obs = d.obs;

				TCamera params;
				vec2cam(par,params);

				const size_t nC = obs.rangeImage.getColCount();
				const size_t nR = obs.rangeImage.getRowCount();


				err.clear();
				err.reserve( nC*nR/square(CALIB_DECIMAT) );

				for (size_t r=0;r<nR;r+=CALIB_DECIMAT)
				{
					for (size_t c=0;c<nC;c+=CALIB_DECIMAT)
					{
						const size_t idx = nC*r + c;

						TPoint3D		p( obs.points3D_x[idx]+d.z_offset, obs.points3D_y[idx], obs.points3D_z[idx] );
						TPoint3D		P(-p.y, -p.z , p.x );
						TPixelCoordf	pixel;
						{	// mrpt-obs shouldn't depend on mrpt-vision just for this!
							//pinhole::projectPoint_with_distortion(p_wrt_cam,cam,pixel);

							// Pinhole model:
							const double x = P.x/P.z;
							const double y = P.y/P.z;

							// Radial distortion:
							const double r2 = square(x)+square(y);
							const double r4 = square(r2);
							
							pixel.x = params.cx() + params.fx() *(  x*(1+params.dist[0]*r2+params.dist[1]*r4+ 2*params.dist[2]*x*y+params.dist[3]*(r2+2*square(x))  )  );
							pixel.y = params.cy() + params.fy() *(  y*(1+params.dist[0]*r2+params.dist[1]*r4+ 2*params.dist[3]*x*y+params.dist[2]*(r2+2*square(y)))  );
						}


						// In theory, it should be (r,c):
						err.push_back( c-pixel.x );
						err.push_back( r-pixel.y );
					}
				}
			} // end error_func
		}
	}
}



/** A Levenberg-Marquart-based optimizer to recover the calibration parameters of a 3D camera given a range (depth) image and the corresponding 3D point cloud. 
  * \param camera_offset The offset (in meters) in the +X direction of the point cloud. It's 1cm for SwissRanger SR4000.
  * \return The final average reprojection error per pixel (typ <0.05 px)
  */
double CObservation3DRangeScan::recoverCameraCalibrationParameters(
	const CObservation3DRangeScan	&obs,
	mrpt::utils::TCamera			&out_camParams,
	const double camera_offset)
{
	MRPT_START

	ASSERT_(obs.hasRangeImage && obs.hasPoints3D)
	ASSERT_(obs.points3D_x.size() == obs.points3D_y.size() && obs.points3D_x.size() == obs.points3D_z.size())

	typedef CLevenbergMarquardtTempl<vector_double, detail::TLevMarData > TMyLevMar;
	TMyLevMar::TResultInfo  info;

	const size_t nR = obs.rangeImage.getRowCount();
	const size_t nC = obs.rangeImage.getColCount();

	TCamera camInit;
	camInit.ncols = nC;
	camInit.nrows = nR;
	camInit.intrinsicParams(0,0) = 250;
	camInit.intrinsicParams(1,1) = 250;
	camInit.intrinsicParams(0,2) = nC >> 1;
	camInit.intrinsicParams(1,2) = nR >> 1;

	vector_double initial_x;
	detail::cam2vec(camInit,initial_x);

	initial_x.resize(8);
	vector_double increments_x(initial_x.size(), 1e-4);

	vector_double optimal_x;

	TMyLevMar::execute(
		optimal_x,
		initial_x,
		&mrpt::slam::detail::cost_func,
		increments_x,
		detail::TLevMarData(obs,camera_offset),
		info,
		false, /* verbose */
		1000 , /* max iter */
		1e-3,
		1e-9,
		1e-9,
		false
		);

	const double avr_px_err = sqrt(info.final_sqr_err/(nC*nR/square(CALIB_DECIMAT)));

	out_camParams.ncols = nC;
	out_camParams.nrows = nR;
	out_camParams.focalLengthMeters = camera_offset;
	detail::vec2cam(optimal_x,out_camParams);

	return avr_px_err;

	MRPT_END
}

