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
