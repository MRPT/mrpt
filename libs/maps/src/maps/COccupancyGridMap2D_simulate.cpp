/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "maps-precomp.h" // Precomp header

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/utils/round.h> // round()

#include <mrpt/random.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace std;

// See docs in header
void  COccupancyGridMap2D::laserScanSimulator(
	mrpt::obs::CObservation2DRangeScan	        &inout_Scan,
	const CPose2D					&robotPose,
	float						    threshold,
	size_t							N,
	float						    noiseStd,
	unsigned int				    decimation,
	float							angleNoiseStd ) const
{
	MRPT_START

	ASSERT_(decimation>=1)
	ASSERT_(N>=2)

	// Sensor pose in global coordinates
	CPose3D		sensorPose3D = CPose3D(robotPose) + inout_Scan.sensorPose;
	// Aproximation: grid is 2D !!!
	CPose2D		sensorPose(sensorPose3D);

	// Scan size:

	inout_Scan.scan.resize(N);
	inout_Scan.validRange.resize(N);

	double  A = sensorPose.phi() + (inout_Scan.rightToLeft ? -0.5:+0.5) *inout_Scan.aperture;
	const double AA = (inout_Scan.rightToLeft ? 1.0:-1.0) * (inout_Scan.aperture / (N-1));

	const float free_thres = 1.0f - threshold;

	for (size_t i=0;i<N;i+=decimation,A+=AA*decimation)
	{
		bool valid;
		simulateScanRay(
			sensorPose.x(),sensorPose.y(),A,
			inout_Scan.scan[i],valid,
			inout_Scan.maxRange, free_thres,
			noiseStd, angleNoiseStd );
		inout_Scan.validRange[i] = valid ? 1:0;
	}

	MRPT_END
}

void  COccupancyGridMap2D::sonarSimulator(
	CObservationRange	        &inout_observation,
	const CPose2D				&robotPose,
	float						threshold,
	float						rangeNoiseStd,
	float						angleNoiseStd) const
{
	const float free_thres = 1.0f - threshold;

	for (CObservationRange::iterator itR=inout_observation.begin();itR!=inout_observation.end();++itR)
	{
		const CPose2D sensorAbsolutePose = CPose2D( CPose3D(robotPose) + CPose3D(itR->sensorPose) );

		// For each sonar cone, simulate several rays and keep the shortest distance:
		ASSERT_(inout_observation.sensorConeApperture>0)
		size_t nRays = round(1+ inout_observation.sensorConeApperture / DEG2RAD(1.0) );

		double direction = sensorAbsolutePose.phi() - 0.5*inout_observation.sensorConeApperture;
		const double Adir = inout_observation.sensorConeApperture / nRays;

		float min_detected_obs=0;
		for (size_t i=0;i<nRays;i++, direction+=Adir )
		{
			bool valid;
			float sim_rang;
			simulateScanRay(
				sensorAbsolutePose.x(), sensorAbsolutePose.y(), direction,
				sim_rang, valid,
				inout_observation.maxSensorDistance, free_thres,
				rangeNoiseStd, angleNoiseStd );

			if (valid && (sim_rang<min_detected_obs || !i))
				min_detected_obs = sim_rang;
		}
		// Save:
		itR->sensedDistance = min_detected_obs;
	}
}

void COccupancyGridMap2D::simulateScanRay(
	const double start_x,const double start_y,const double angle_direction,
	float &out_range,bool &out_valid,
	const double max_range_meters,
	const float threshold_free,
	const double noiseStd, const double angleNoiseStd ) const
{
	const double A_ = angle_direction + (angleNoiseStd>.0 ? randomGenerator.drawGaussian1D_normalized()*angleNoiseStd : .0);

	// Unit vector in the directorion of the ray:
#ifdef HAVE_SINCOS
	double Arx,Ary;
	::sincos(A_, &Ary,&Arx);
#else
	const double Arx =  cos(A_);
	const double Ary =  sin(A_);
#endif

	// Ray tracing, until collision, out of the map or out of range:
	const unsigned int max_ray_len = mrpt::utils::round(max_range_meters/resolution);
	unsigned int ray_len=0;
	unsigned int firstUnknownCellDist=max_ray_len+1;

	// Use integers for all ray tracing for efficiency
#define INTPRECNUMBIT 10
#define int_x2idx(_X) (_X>>INTPRECNUMBIT)
#define int_y2idx(_Y) (_Y>>INTPRECNUMBIT)

	int64_t rxi = static_cast<int64_t>( ((start_x-x_min)/resolution) * (1L <<INTPRECNUMBIT));
	int64_t ryi = static_cast<int64_t>( ((start_y-y_min)/resolution) * (1L <<INTPRECNUMBIT));

	const int64_t Arxi = static_cast<int64_t>( RAYTRACE_STEP_SIZE_IN_CELL_UNITS * Arx * (1L <<INTPRECNUMBIT) );
	const int64_t Aryi = static_cast<int64_t>( RAYTRACE_STEP_SIZE_IN_CELL_UNITS * Ary * (1L <<INTPRECNUMBIT) );

	cellType hitCellOcc_int = 0; // p2l(0.5f)
	const cellType threshold_free_int = p2l(threshold_free);
	int x, y=int_y2idx(ryi);

	while ( (x=int_x2idx(rxi))>=0 && (y=int_y2idx(ryi))>=0 &&
		x<static_cast<int>(size_x) && y<static_cast<int>(size_y) && (hitCellOcc_int=map[x+y*size_x])>threshold_free_int &&
		ray_len<max_ray_len )
	{
		if ( abs(hitCellOcc_int)<=1 )
			mrpt::utils::keep_min(firstUnknownCellDist, ray_len );

		rxi+=Arxi;
		ryi+=Aryi;
		ray_len++;
	}

	// Store:
	// Check out of the grid?
	// Tip: if x<0, (unsigned)(x) will also be >>> size_x ;-)
	if (abs(hitCellOcc_int)<=1 || static_cast<unsigned>(x)>=size_x || static_cast<unsigned>(y)>=size_y )
	{
		out_valid = false;

		if (firstUnknownCellDist<ray_len)
			out_range = RAYTRACE_STEP_SIZE_IN_CELL_UNITS*firstUnknownCellDist*resolution;
		else	out_range = RAYTRACE_STEP_SIZE_IN_CELL_UNITS*ray_len*resolution;
	}
	else
	{ 	// No: The normal case:
		out_range = RAYTRACE_STEP_SIZE_IN_CELL_UNITS*ray_len*resolution;
		out_valid = out_range<max_range_meters;
		// Add additive Gaussian noise:
		if (noiseStd>0 && out_valid)
			out_range+=  noiseStd*randomGenerator.drawGaussian1D_normalized();
	}
}
