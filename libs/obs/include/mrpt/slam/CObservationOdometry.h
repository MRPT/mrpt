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
