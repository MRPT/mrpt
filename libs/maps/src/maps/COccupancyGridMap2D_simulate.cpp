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

#include <mrpt/maps.h>  // Precompiled header

#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/slam/CObservationRange.h>

#include <mrpt/random.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace std;

/*---------------------------------------------------------------
						laserScanSimulator

 Simulates a range scan into the current grid map.
   The simulated scan is stored in a CObservation2DRangeScan object, which is also used
    to pass some parameters: all previously stored characteristics (as aperture,...) are
	  taken into account for simulation. Only a few more parameters are needed. Additive gaussian noise can be optionally added to the simulated scan.
		inout_Scan [IN/OUT] This must be filled with desired parameters before calling, and will contain the scan samples on return.
		robotPose [IN] The robot pose in this map coordinates. Recall that sensor pose relative to this robot pose must be specified in the observation object.
		threshold [IN] The minimum occupancy threshold to consider a cell to be occupied, for example 0.5.
		N [IN] The count of range scan "rays", by default to 361.
		noiseStd [IN] The standard deviation of measurement noise. If not desired, set to 0.
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::laserScanSimulator(
		CObservation2DRangeScan	        &inout_Scan,
		const CPose2D					&robotPose,
		float						    threshold,
		size_t							N,
		float						    noiseStd,
		unsigned int				    decimation,
		float							angleNoiseStd ) const
{
	MRPT_START

	ASSERT_(decimation>=1);

	// Sensor pose in global coordinates
	CPose3D		sensorPose3D = CPose3D(robotPose) + inout_Scan.sensorPose;
	// Aproximation: grid is 2D !!!
	CPose2D		sensorPose(sensorPose3D);

    // Scan size:
    inout_Scan.scan.resize(N);
    inout_Scan.validRange.resize(N);

    double  A, AA;
	if (inout_Scan.rightToLeft)
	{
		A = sensorPose.phi() - 0.5*inout_Scan.aperture;
		AA = (inout_Scan.aperture / N);
	}
	else
	{
		A = sensorPose.phi() + 0.5*inout_Scan.aperture;
		AA = -(inout_Scan.aperture / N);
	}

    const float free_thres = 1.0f - threshold;
    const unsigned int max_ray_len = round(inout_Scan.maxRange / resolution);

    for (size_t i=0;i<N;i+=decimation,A+=AA*decimation)
    {
    	bool valid;
    	simulateScanRay(
			sensorPose.x(),sensorPose.y(),A,
			inout_Scan.scan[i],valid,
			max_ray_len, free_thres,
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
    const unsigned int max_ray_len = round(inout_observation.maxSensorDistance / resolution);

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
				max_ray_len, free_thres,
				rangeNoiseStd, angleNoiseStd );

			if (valid && (sim_rang<min_detected_obs || !i))
				min_detected_obs = sim_rang;
    	}
    	// Save:
    	itR->sensedDistance = min_detected_obs;
	}
}

inline void COccupancyGridMap2D::simulateScanRay(
	const double start_x,const double start_y,const double angle_direction,
	float &out_range,bool &out_valid,
	const unsigned int max_ray_len,
	const float threshold_free,
	const double noiseStd, const double angleNoiseStd ) const
{
	const double A_ = angle_direction + randomGenerator.drawGaussian1D_normalized()*angleNoiseStd;

	// Unit vector in the directorion of the ray:
#ifdef HAVE_SINCOS
	double Arx,Ary;
	::sincos(A_, &Ary,&Arx);
	Arx*=resolution;
	Ary*=resolution;
#else
	const double Arx =  cos(A_)*resolution;
	const double Ary =  sin(A_)*resolution;
#endif

	// Ray tracing, until collision, out of the map or out of range:
	unsigned int ray_len=0;
	unsigned int firstUnknownCellDist=max_ray_len+1;
	double rx=start_x;
	double ry=start_y;
	float hitCellOcc = 0.5f;
	int x, y=y2idx(ry);

	while ( (x=x2idx(rx))>=0 && (y=y2idx(ry))>=0 &&
			 x<static_cast<int>(size_x) && y<static_cast<int>(size_y) && (hitCellOcc=getCell(x,y))>threshold_free &&
			 ray_len<max_ray_len  )
	{
		if ( fabs(hitCellOcc-0.5)<0.01f )
			mrpt::utils::keep_min(firstUnknownCellDist, ray_len );

		rx+=Arx;
		ry+=Ary;
		ray_len++;
	}

	// Store:
	// Check out of the grid?
	// Tip: if x<0, (unsigned)(x) will also be >>> size_x ;-)
	if (fabs(hitCellOcc-0.5)<0.01f || static_cast<unsigned>(x)>=size_x || static_cast<unsigned>(y)>=size_y )
	{
		out_valid = false;

		if (firstUnknownCellDist<ray_len)
				out_range = firstUnknownCellDist*resolution;
		else	out_range = ray_len*resolution;
	}
	else
	{ 	// No: The normal case:
		out_range = ray_len*resolution;
		out_valid = ray_len<max_ray_len;
		// Add additive Gaussian noise:
		if (noiseStd>0 && out_valid)
			out_range+=  noiseStd*randomGenerator.drawGaussian1D_normalized();
	}
}
