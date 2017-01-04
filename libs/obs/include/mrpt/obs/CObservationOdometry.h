/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CObservationOdometry_H
#define CObservationOdometry_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/lightweight_geom_data.h>

namespace mrpt
{
	namespace obs
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
			CObservationOdometry(); //!< Default ctor

			mrpt::poses::CPose2D odometry; //!< The absolute odometry measurement (IT IS NOT INCREMENTAL)

			bool  hasEncodersInfo; //!< "true" means that "encoderLeftTicks" and "encoderRightTicks" contain valid values.
			/** For differential-driven robots: The ticks count for each wheel in ABSOLUTE VALUE (IT IS NOT INCREMENTAL) (positive means FORWARD, for both wheels); \sa hasEncodersInfo  */
			int32_t	 encoderLeftTicks,encoderRightTicks;

			bool                 hasVelocities;  //!< "true" means that `velocityLocal` contains valid values.
			mrpt::math::TTwist2D velocityLocal; //!< Velocity, in the robot (local) frame of reference (+X=forward).

			// See base class docs
			void getSensorPose( mrpt::poses::CPose3D &out_sensorPose ) const MRPT_OVERRIDE { out_sensorPose=mrpt::poses::CPose3D(0,0,0); }
			void setSensorPose( const mrpt::poses::CPose3D & ) MRPT_OVERRIDE {  }
			void getDescriptionAsText(std::ostream &o) const MRPT_OVERRIDE;

		}; // End of class def.
		DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CObservationOdometry, CObservation,OBS_IMPEXP  )

	} // End of namespace
} // End of namespace

#endif
