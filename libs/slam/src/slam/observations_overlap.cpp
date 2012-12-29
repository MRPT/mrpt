/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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


