/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
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
/** Declares a class derived from "CObservation" that represents the wind
 * measurements taken on the robot by an anemometer.
 * The observation is composed by two magnitudes:
 * wind speed (m/s)
 * wind direction (deg)
 *
 * \sa CObservation
 * \ingroup mrpt_obs_grp
 */
class CObservationWindSensor : public CObservation
{
	DEFINE_SERIALIZABLE(CObservationWindSensor, mrpt::obs)

   public:
	/** @name The data members
	 * @{ */

	double speed = 0;  //!< Wind speed [m/s]
	double direction = 0;  //!< Wind flow direction [degrees]

	/** The location of the sensing anemometer on the robot frame */
	mrpt::poses::CPose3D sensorPoseOnRobot;

	/** @} */

	// See base class docs
	void getSensorPose(mrpt::poses::CPose3D& out_sensorPose) const override;
	void setSensorPose(const mrpt::poses::CPose3D& newSensorPose) override;
	void getDescriptionAsText(std::ostream& o) const override;

	// See base class docs:
	bool exportTxtSupported() const override { return true; }
	std::string exportTxtHeader() const override;
	std::string exportTxtDataRow() const override;

};	// End of class def.

}  // namespace mrpt::obs
