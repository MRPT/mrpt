/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CSerializable.h>

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
	DEFINE_SERIALIZABLE(CObservationBatteryState, mrpt::obs)

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
		const override;	 // See base class docs.
	void setSensorPose(const mrpt::poses::CPose3D& newSensorPose)
		override;  // See base class docs.
	void getDescriptionAsText(
		std::ostream& o) const override;  // See base class docs

	// See base class docs:
	bool exportTxtSupported() const override { return true; }
	std::string exportTxtHeader() const override;
	std::string exportTxtDataRow() const override;

};	// End of class def.

}  // namespace mrpt::obs
