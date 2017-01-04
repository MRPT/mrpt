/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef observations_overlap_H
#define observations_overlap_H

#include <mrpt/obs/obs_frwds.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
	namespace slam
	{
		/**  \addtogroup mrpt_slam_grp 
		  *   @{ */
	
		/** @name Observations overlap functions 
		    @{  */
	
		/** Estimates the "overlap" or "matching ratio" of two observations (range [0,1]), possibly taking into account their relative positions.
		  *  \note This is used in mrpt::slam::CIncrementalMapPartitioner
		  */
		double SLAM_IMPEXP observationsOverlap(
			const mrpt::obs::CObservation* o1,
			const mrpt::obs::CObservation* o2,
			const mrpt::poses::CPose3D *pose_o2_wrt_o1 = NULL );

		/** Estimates the "overlap" or "matching ratio" of two observations (range [0,1]), possibly taking into account their relative positions.
		  *  \note This is used in mrpt::slam::CIncrementalMapPartitioner
		  */
		inline double observationsOverlap(
			const mrpt::obs::CObservationPtr & o1,
			const mrpt::obs::CObservationPtr & o2,
			const mrpt::poses::CPose3D *pose_o2_wrt_o1 = NULL ) 
		{
			return observationsOverlap(o1.pointer(), o2.pointer(), pose_o2_wrt_o1 );
		}
			
		/** Estimates the "overlap" or "matching ratio" of two set of observations (range [0,1]), possibly taking into account their relative positions.
		  *   This method computes the average between each of the observations in each SF.
		  *  \note This is used in mrpt::slam::CIncrementalMapPartitioner
		  */
		double SLAM_IMPEXP observationsOverlap(
			const mrpt::obs::CSensoryFrame &sf1,
			const mrpt::obs::CSensoryFrame &sf2,
			const mrpt::poses::CPose3D *pose_sf2_wrt_sf1 = NULL );

		/** Estimates the "overlap" or "matching ratio" of two set of observations (range [0,1]), possibly taking into account their relative positions.
		  *   This method computes the average between each of the observations in each SF.
		  *  \note This is used in mrpt::slam::CIncrementalMapPartitioner
		  */
		inline double observationsOverlap(
			const mrpt::obs::CSensoryFramePtr &sf1,
			const mrpt::obs::CSensoryFramePtr &sf2,
			const mrpt::poses::CPose3D *pose_sf2_wrt_sf1 = NULL )
		{
			return observationsOverlap(*sf1.pointer(), *sf2.pointer(), pose_sf2_wrt_sf1);
		}

		/** @} */
		
		/** @} */ // end grouping

	} // End of namespace
} // End of namespace

#endif
