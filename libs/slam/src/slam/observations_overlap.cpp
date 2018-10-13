/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "slam-precomp.h"  // Precompiled headers

#include <mrpt/slam/observations_overlap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>

using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace std;

/** Estimates the "overlap" or "matching ratio" of two observations (range
 * [0,1]), possibly taking into account their relative positions.
 *  \note This is used in mrpt::slam::CIncrementalMapPartitioner
 */
double mrpt::slam::observationsOverlap(
	const mrpt::obs::CObservation* o1, const mrpt::obs::CObservation* o2,
	const mrpt::poses::CPose3D* pose_o2_wrt_o1)
{
	if (IS_CLASS(o1, CObservation2DRangeScan) &&
		IS_CLASS(o2, CObservation2DRangeScan))
	{
		const auto* this_obs = static_cast<const CObservation2DRangeScan*>(o1);
		const auto* obs = static_cast<const CObservation2DRangeScan*>(o2);

		const auto* map1 =
			this_obs->buildAuxPointsMap<mrpt::maps::CPointsMap>();
		const auto* map2 = obs->buildAuxPointsMap<mrpt::maps::CPointsMap>();

		// if PDF is available, get "mean" value as an estimation:
		CPose3D otherObsPose;
		if (pose_o2_wrt_o1) otherObsPose = *pose_o2_wrt_o1;

		mrpt::tfest::TMatchingPairList correspondences;
		mrpt::maps::TMatchingParams matchParams;
		mrpt::maps::TMatchingExtraResults matchExtraResults;

		matchParams.maxDistForCorrespondence = 0.04f;
		matchParams.maxAngularDistForCorrespondence = 0;
		map1->determineMatching3D(
			map2,  // The other map
			otherObsPose,  // The other map pose
			correspondences, matchParams, matchExtraResults);

		return matchExtraResults.correspondencesRatio;
	}
	else
	{
		// No idea...
		return 0;
	}
}

/** Estimates the "overlap" or "matching ratio" of two set of observations
 * (range [0,1]), possibly taking into account their relative positions.
 *   This method computes the average between each of the observations in each
 * SF.
 *  \note This is used in mrpt::slam::CIncrementalMapPartitioner
 */
double mrpt::slam::observationsOverlap(
	const mrpt::obs::CSensoryFrame& sf1, const mrpt::obs::CSensoryFrame& sf2,
	const mrpt::poses::CPose3D* pose_sf2_wrt_sf1)
{
	MRPT_UNUSED_PARAM(pose_sf2_wrt_sf1);
	// Return the average value:
	size_t N = 0;
	double accum = 0;
	for (const auto& i1 : sf1)
	{
		for (const auto& i2 : sf2)
		{
			accum += observationsOverlap(i1, i2);
			N++;
		}
	}
	return N ? (accum / N) : 0;
}
