/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#ifndef CObservationWindSensor_H
#define CObservationWindSensor_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>

namespace mrpt
{
namespace obs
{

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
		DEFINE_SERIALIZABLE( CObservationWindSensor )

	 public:
		/** Constructor */
		CObservationWindSensor( );

		 /** @name The data members
		  * @{ */

		/** The wind speed in m/s */
		double speed;		
		/** The wind flow direction in deg */
		double direction;	
		/** The location of the sensing anemometer on the robot coordinate framework */
		mrpt::poses::CPose3D  sensorPoseOnRobot; 

		/** @} */

		// See base class docs
		void getSensorPose( mrpt::poses::CPose3D &out_sensorPose ) const override;
		void setSensorPose( const mrpt::poses::CPose3D &newSensorPose ) override;
		void getDescriptionAsText(std::ostream &o) const override;

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CObservationWindSensor, CObservation, OBS_IMPEXP)


	} // End of namespace
} // End of namespace

#endif
