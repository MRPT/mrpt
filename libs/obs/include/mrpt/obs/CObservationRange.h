/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CSerializable.h>

namespace mrpt::obs
{
/** Declares a class derived from "CObservation" that
	   encapsules a single range measurement, and associated parameters. This
 can be used
 *     for example to store measurements from infrared proximity sensors (IR) or
 ultrasonic sensors (sonars).
 *
 * \sa CObservation
 * \ingroup mrpt_obs_grp
 */
class CObservationRange : public CObservation
{
	DEFINE_SERIALIZABLE(CObservationRange, mrpt::obs)

   public:
	CObservationRange() = default;

	/** The data members
	 */
	float minSensorDistance{0};
	float maxSensorDistance{5};

	/** Cone aperture of each ultrasonic beam, in radians. */
	float sensorConeApperture = mrpt::d2f(20.0_deg);

	struct TMeasurement
	{
		TMeasurement() : sensorPose() {}
		/** Some kind of sensor ID which identifies it on the bus (if
		 * applicable, 0 otherwise)
		 */
		uint16_t sensorID{0};

		/** The 6D position of the sensor on the robot.
		 */
		math::TPose3D sensorPose;

		/** The measured range, in meters (or a value of 0 if there was no
		 * detected echo).
		 */
		float sensedDistance{0};
	};

	using TMeasurementList = std::deque<TMeasurement>;
	using const_iterator = std::deque<TMeasurement>::const_iterator;
	using iterator = std::deque<TMeasurement>::iterator;

	/** All the measurements */
	TMeasurementList sensedData;

	iterator begin() { return sensedData.begin(); }
	iterator end() { return sensedData.end(); }
	const_iterator begin() const { return sensedData.begin(); }
	const_iterator end() const { return sensedData.end(); }
	// See base class docs
	void getSensorPose(mrpt::poses::CPose3D& out_sensorPose) const override;
	void setSensorPose(const mrpt::poses::CPose3D& newSensorPose) override;
	void getDescriptionAsText(std::ostream& o) const override;

};  // End of class def.
}  // namespace mrpt::obs
