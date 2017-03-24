/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CObservationRange_H
#define CObservationRange_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>

namespace mrpt
{
namespace obs
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationRange , CObservation,OBS_IMPEXP )

	/** Declares a class derived from "CObservation" that
	       encapsules a single range measurement, and associated parameters. This can be used
	 *     for example to store measurements from infrared proximity sensors (IR) or ultrasonic sensors (sonars).
	 *
	 * \sa CObservation
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CObservationRange : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationRange )

	 public:
		/** Default constructor.
		 */
		CObservationRange( );

		 /** The data members
		  */
		float	minSensorDistance;
		float	maxSensorDistance;
		float	sensorConeApperture;  //!< Cone aperture of each ultrasonic beam, in radians.

		struct OBS_IMPEXP TMeasurement
		{
			TMeasurement() : sensorID(0), sensorPose(), sensedDistance(0)
			{}

			/** Some kind of sensor ID which identifies it on the bus (if applicable, 0 otherwise)
			  */
			uint16_t	sensorID;

			/** The 6D position of the sensor on the robot.
			  */
			math::TPose3D	sensorPose;

			/** The measured range, in meters (or a value of 0 if there was no detected echo).
			  */
			float		sensedDistance;
		};

		typedef std::deque<TMeasurement>                    TMeasurementList;
		typedef std::deque<TMeasurement>::const_iterator    const_iterator;
		typedef std::deque<TMeasurement>::iterator          iterator;

		TMeasurementList sensedData; //!< All the measurements


		iterator begin() { return sensedData.begin(); }
		iterator end() { return sensedData.end(); }
		const_iterator begin() const { return sensedData.begin(); }
		const_iterator end() const { return sensedData.end(); }

		// See base class docs
		void getSensorPose( mrpt::poses::CPose3D &out_sensorPose ) const MRPT_OVERRIDE;
		void setSensorPose( const mrpt::poses::CPose3D &newSensorPose ) MRPT_OVERRIDE;
		void getDescriptionAsText(std::ostream &o) const MRPT_OVERRIDE;

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CObservationRange , CObservation,OBS_IMPEXP )
	} // End of namespace
} // End of namespace
#endif
