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
/** This represents a measurement of the wireless strength perceived by the
 * robot.
 *  The signal level is given as a percentage.
 *
 * \sa CObservation, mrpt::hwdrivers::CWirelessPower for a software sensor
 * capable of reading this kind of observations.
 * \ingroup mrpt_obs_grp
 */
class CObservationWirelessPower : public CObservation
{
	DEFINE_SERIALIZABLE(CObservationWirelessPower, mrpt::obs)

   public:
	/** @name The data members
	 * @{ */

	/** The power or signal strength as sensed by the Wifi receiver (In
	 * percentage: [0-100]) */
	double power{0};
	/** The location of the sensing antenna on the robot coordinate framework */
	mrpt::poses::CPose3D sensorPoseOnRobot;

	/** @} */

	void getSensorPose(mrpt::poses::CPose3D& out_sensorPose)
		const override;  // See base class docs
	void setSensorPose(const mrpt::poses::CPose3D& newSensorPose)
		override;  // See base class docs
	void getDescriptionAsText(
		std::ostream& o) const override;  // See base class docs

};  // End of class def.

}  // namespace mrpt::obs
