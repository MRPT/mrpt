/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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
#ifndef observations_overlap_H
#define observations_overlap_H

#include <mrpt/obs.h>

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
			const mrpt::slam::CObservation* o1,
			const mrpt::slam::CObservation* o2,
			const mrpt::poses::CPose3D *pose_o2_wrt_o1 = NULL );

		/** Estimates the "overlap" or "matching ratio" of two observations (range [0,1]), possibly taking into account their relative positions.
		  *  \note This is used in mrpt::slam::CIncrementalMapPartitioner
		  */
		inline double observationsOverlap(
			const mrpt::slam::CObservationPtr & o1,
			const mrpt::slam::CObservationPtr & o2,
			const mrpt::poses::CPose3D *pose_o2_wrt_o1 = NULL ) 
		{
			return observationsOverlap(o1.pointer(), o2.pointer(), pose_o2_wrt_o1 );
		}
			
		/** Estimates the "overlap" or "matching ratio" of two set of observations (range [0,1]), possibly taking into account their relative positions.
		  *   This method computes the average between each of the observations in each SF.
		  *  \note This is used in mrpt::slam::CIncrementalMapPartitioner
		  */
		double SLAM_IMPEXP observationsOverlap(
			const mrpt::slam::CSensoryFrame &sf1,
			const mrpt::slam::CSensoryFrame &sf2,
			const mrpt::poses::CPose3D *pose_sf2_wrt_sf1 = NULL );

		/** Estimates the "overlap" or "matching ratio" of two set of observations (range [0,1]), possibly taking into account their relative positions.
		  *   This method computes the average between each of the observations in each SF.
		  *  \note This is used in mrpt::slam::CIncrementalMapPartitioner
		  */
		inline double observationsOverlap(
			const mrpt::slam::CSensoryFramePtr &sf1,
			const mrpt::slam::CSensoryFramePtr &sf2,
			const mrpt::poses::CPose3D *pose_sf2_wrt_sf1 = NULL )
		{
			return observationsOverlap(*sf1.pointer(), *sf2.pointer(), pose_sf2_wrt_sf1);
		}

		/** @} */
		
		/** @} */ // end grouping

	} // End of namespace
} // End of namespace

#endif
