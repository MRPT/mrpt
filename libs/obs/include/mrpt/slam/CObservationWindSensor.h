/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CObservationWindSensor_H
#define CObservationWindSensor_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>

namespace mrpt
{
namespace slam
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationWindSensor, CObservation, OBS_IMPEXP)

	/** Declares a class derived from "CObservation" that represents the wind measurements taken on the robot by an anemometer.
	 * The observation is composed by two magnitudes:
	 * wind speed (m/s)
	 * wind direction (deg)
	 *
	 * \sa CObservation
	 * \ingroup mrpt_obs_grp
	 */

	class OBS_IMPEXP CObservationWindSensor : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationWindSensor )

	 public:
		/** Constructor */
		CObservationWindSensor( );

		 /** @name The data members
		  * @{ */

		double speed;		//!< The wind speed in m/s
		double direction;	//!< The wind flow direction in deg
		mrpt::poses::CPose3D  sensorPoseOnRobot; //!< The location of the sensing anemometer on the robot coordinate framework

		/** @} */

		void getSensorPose( CPose3D &out_sensorPose ) const;

		/** A general method to change the sensor pose on the robot.
		  *  It has no effects in this class
		  * \sa getSensorPose  */
		void setSensorPose( const CPose3D &newSensorPose );

	}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
