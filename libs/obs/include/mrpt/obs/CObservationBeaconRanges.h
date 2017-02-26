/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CObservationBeaconRanges_H
#define CObservationBeaconRanges_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>

namespace mrpt
{
namespace obs
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationBeaconRanges, CObservation, OBS_IMPEXP  )

	/** Declares a class derived from "CObservation" that represents one (or more) range measurements to labeled beacons.
	 * \sa CObservation
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CObservationBeaconRanges : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationBeaconRanges )

	 public:
		/** Default constructor.
		 */
		CObservationBeaconRanges( );

		 /** Information about the sensor:
		  */
		float	minSensorDistance, maxSensorDistance;

		/** The "sigma" of the sensor, assuming a zero-mean Gaussian noise model.
		  */
		float	stdError;

		/** Each one of the measurements:
			*/
		struct OBS_IMPEXP TMeasurement
		{
			TMeasurement() : sensorLocationOnRobot(), sensedDistance(0),beaconID(INVALID_BEACON_ID)
			{}

			/** The position of the sensor on the robot.
			  */
		 mrpt::poses::CPoint3D		sensorLocationOnRobot;

			/** The sensed range itself (in meters).
			  */
			float			sensedDistance;

			/** The ID of the sensed beacon (or INVALID_BEACON_ID)
			  */
			int32_t			beaconID;
		};

		/** The list of observed ranges:
		  */
		std::deque<TMeasurement> sensedData;

		/** The (X,Y,PHI) pose estimated by the UWB software, for comparison purposes (Added in streamming version 1)  */
		mrpt::poses::CPose2D auxEstimatePose;
		
		 /** Prints out the contents of the object.
		   */
		 void  debugPrintOut();

		 void getSensorPose( mrpt::poses::CPose3D &out_sensorPose ) const MRPT_OVERRIDE;// See base class docs.
		 void setSensorPose( const mrpt::poses::CPose3D &newSensorPose ) MRPT_OVERRIDE;// See base class docs.
		 void getDescriptionAsText(std::ostream &o) const MRPT_OVERRIDE;// See base class docs

		/** Easy look-up into the vector sensedData, returns the range for a given beacon, or 0 if the beacon is not observed.
		  */
		float getSensedRangeByBeaconID(int32_t beaconID);

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CObservationBeaconRanges, CObservation, OBS_IMPEXP  )


	} // End of namespace
} // End of namespace

#endif
