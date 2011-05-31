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

#include <mrpt/slam.h>  // Precompiled header

#include <mrpt/slam/observations_overlap.h>
#include <mrpt/slam/CPointsMap.h>

using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;

/** Estimates the "overlap" or "matching ratio" of two observations (range [0,1]), possibly taking into account their relative positions.
  *  \note This is used in mrpt::slam::CIncrementalMapPartitioner
  */
double mrpt::slam::observationsOverlap(
	const mrpt::slam::CObservation* o1,
	const mrpt::slam::CObservation* o2,
	const mrpt::poses::CPose3D *pose_o2_wrt_o1 )
{
	if (IS_CLASS(o1,CObservation2DRangeScan) && IS_CLASS(o2,CObservation2DRangeScan))
	{
		const CObservation2DRangeScan *this_obs = static_cast<const CObservation2DRangeScan *>(o1);
		const CObservation2DRangeScan *obs      = static_cast<const CObservation2DRangeScan *>(o2);

		const CPointsMap *map1 = this_obs->buildAuxPointsMap<mrpt::slam::CPointsMap>();
		const CPointsMap *map2 = obs->buildAuxPointsMap<mrpt::slam::CPointsMap>();

		// if PDF is available, get "mean" value as an estimation:
		CPose3D	  otherObsPose;
		if (pose_o2_wrt_o1)
			otherObsPose = *pose_o2_wrt_o1;

		const float maxDisForCorrespondence = 0.04f;
		mrpt::utils::TMatchingPairList	correspondences;
		float	correspondencesRatio;
		static const CPoint3D pivotPoint(0,0,0);

		map1->computeMatchingWith3D(
				map2,
				otherObsPose,
				maxDisForCorrespondence,
				0,
				pivotPoint,
				correspondences,
				correspondencesRatio);

		return correspondencesRatio;
	}
	else
	{
		// No idea...
		return 0;
	}

}

/** Estimates the "overlap" or "matching ratio" of two set of observations (range [0,1]), possibly taking into account their relative positions.
  *   This method computes the average between each of the observations in each SF.
  *  \note This is used in mrpt::slam::CIncrementalMapPartitioner
  */
double mrpt::slam::observationsOverlap(
	const mrpt::slam::CSensoryFrame &sf1,
	const mrpt::slam::CSensoryFrame &sf2,
	const mrpt::poses::CPose3D *pose_sf2_wrt_sf1 )
{
	// Return the average value:
	size_t N=0;
	double accum = 0;
	for (CSensoryFrame::const_iterator i1=sf1.begin();i1!=sf1.end();++i1)
	{
		for (CSensoryFrame::const_iterator i2=sf2.begin();i2!=sf2.end();++i2)
		{
			accum += observationsOverlap(*i1,*i2);
			N++;
		}
	}
	return  N ? (accum/N) : 0;
}


