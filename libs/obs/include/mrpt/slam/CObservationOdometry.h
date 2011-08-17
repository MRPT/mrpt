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

#ifndef CObservationOdometry_H
#define CObservationOdometry_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>

namespace mrpt
{
	namespace slam
	{
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationOdometry, CObservation,OBS_IMPEXP  )

		/** An observation of the current (cumulative) odometry for a wheeled robot.
		 *   This kind of observation will only occur in a "observation-only" rawlog file, otherwise
		 *    odometry are modeled with actions. Refer to the <a href="http://www.mrpt.org/Rawlog_Format">page on rawlogs</a>.
		 *
		 * \sa CObservation, CActionRobotMovement2D
		 * \ingroup mrpt_obs_grp
		 */
		class OBS_IMPEXP CObservationOdometry : public CObservation
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CObservationOdometry )

		 public:
			/** Constructor
			 */
			CObservationOdometry( );

			poses::CPose2D		odometry;		//!< The absolute odometry measurement (IT IS NOT INCREMENTAL)

			bool  hasEncodersInfo; //!< "true" means that "encoderLeftTicks" and "encoderRightTicks" contain valid values.

			/** For odometry only: the ticks count for each wheel in ABSOLUTE VALUE (IT IS NOT INCREMENTAL) (positive means FORWARD, for both wheels);
			  * \sa hasEncodersInfo
			  */
			int32_t	 encoderLeftTicks,encoderRightTicks;

			bool	hasVelocities;		//!< "true" means that "velocityLin" and "velocityAng" contain valid values.

			/** The velocity of the robot, linear in meters/sec and angular in rad/sec.
			  */
			float	velocityLin, velocityAng;


			/** A general method to retrieve the sensor pose on the robot.
			  *  It has no effects in this class
			  * \sa setSensorPose
			  */
			void getSensorPose( CPose3D &out_sensorPose ) const { out_sensorPose=CPose3D(0,0,0); }

			/** A general method to change the sensor pose on the robot.
			  *  It has no effects in this class
			  * \sa getSensorPose
			  */
			void setSensorPose( const CPose3D & ) {  }

		}; // End of class def.

	} // End of namespace
} // End of namespace

#endif
