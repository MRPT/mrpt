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
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>

namespace mrpt::obs
{
/** This represents a measurement of the batteries on the robot.
 *  The battery levels are in volts in the form of the public members:
 *	- voltageMainRobotBattery
 *	- voltageMainRobotComputer
 *  - voltageOtherBatteries
 *
 *  There are boolean flags for signaling when the corresponding values have
 *been filled out or not.
 *
 * \sa CObservation
 * \ingroup mrpt_obs_grp
 */
class CObservationBatteryState : public CObservation
{
	DEFINE_SERIALIZABLE(CObservationBatteryState)

   public:
	/** Constructor
	 */
	CObservationBatteryState();

	/** The data members
	 * \sa voltageMainRobotBatteryIsValid,voltageMainRobotComputerIsValid
	 */
	double voltageMainRobotBattery{0}, voltageMainRobotComputer{0};

	/** These values must be true if the corresponding fields contain valid
	 * values.
	 * \sa voltageMainRobotBattery,voltageMainRobotComputer
	 */
	bool voltageMainRobotBatteryIsValid{false},
		voltageMainRobotComputerIsValid{false};

	/** The users can use this vector for any arbitrary number of batteries or
	 * any other analog measurements.
	 * \sa voltageOtherBatteriesValid
	 */
	mrpt::math::CVectorDouble voltageOtherBatteries;

	/** These values must be true if the corresponding fields contain valid
	 * values (it MUST has the same size than voltageOtherBatteries)
	 */
	std::vector<bool> voltageOtherBatteriesValid;

	void getSensorPose(mrpt::poses::CPose3D& out_sensorPose)
		const override;  // See base class docs.
	void setSensorPose(const mrpt::poses::CPose3D& newSensorPose)
		override;  // See base class docs.
	void getDescriptionAsText(
		std::ostream& o) const override;  // See base class docs

};  // End of class def.

}  // namespace mrpt::obs
