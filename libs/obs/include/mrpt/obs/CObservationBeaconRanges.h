/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CSerializable.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>

namespace mrpt::obs
{
/** Declares a class derived from "CObservation" that represents one (or more)
 * range measurements to labeled beacons.
 * \sa CObservation
 * \ingroup mrpt_obs_grp
 */
class CObservationBeaconRanges : public CObservation
{
	DEFINE_SERIALIZABLE(CObservationBeaconRanges)

   public:
	/** ctor */
	CObservationBeaconRanges();

	/** Info about sensor */
	float minSensorDistance{0}, maxSensorDistance{1e2f};
	/** The "sigma" of the sensor, assuming a zero-mean Gaussian noise model. */
	float stdError{1e-2f};

	/** Each one of the measurements */
	struct TMeasurement
	{
		TMeasurement()
			: sensorLocationOnRobot(),

			  beaconID(INVALID_BEACON_ID)
		{
		}

		/** Position of the sensor on the robot */
		mrpt::poses::CPoint3D sensorLocationOnRobot;
		/** The sensed range itself (in meters). */
		float sensedDistance{0};
		/** The ID of the sensed beacon (or INVALID_BEACON_ID if unknown) */
		int32_t beaconID;
	};

	/** The list of observed ranges */
	std::deque<TMeasurement> sensedData;

	/** The (X,Y,PHI) pose estimated by the UWB software, for comparison
	 * purposes (Added in streamming version 1) */
	mrpt::poses::CPose2D auxEstimatePose;

	/** Prints out the contents of the object  */
	void debugPrintOut();

	void getSensorPose(mrpt::poses::CPose3D& out_sensorPose)
		const override;  // See base class docs.
	void setSensorPose(const mrpt::poses::CPose3D& newSensorPose)
		override;  // See base class docs.
	void getDescriptionAsText(
		std::ostream& o) const override;  // See base class docs

	/** Easy look-up into the vector sensedData, returns the range for a given
	 * beacon, or 0 if the beacon is not observed */
	float getSensedRangeByBeaconID(int32_t beaconID);

};  // End of class def.

}  // namespace mrpt::obs
