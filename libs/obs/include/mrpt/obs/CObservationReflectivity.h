/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>

namespace mrpt::obs
{
/** Declares a class derived from "CObservation" that encapsules a single
 * short-range reflectivity measurement.
 *    This can be used for example to store readings from IR sensors (Lego
 * Mindstorm NXT, etc...).
 *
 * \sa mrpt::obs::CReflectivityGridMap2D, CObservation
 * \ingroup mrpt_obs_grp
 */
class CObservationReflectivity : public CObservation
{
	DEFINE_SERIALIZABLE(CObservationReflectivity)

   public:
	/** The read reflectivity level, in the range [0,1] (0=black, 1=white).
	 */
	float reflectivityLevel{0.5f};

	/** The channel for this observation. If channel=-1, it can be inserted into
	 * any CReflectivityGridMap2D. Otherwise, it can only be inserted into
	 * reflectivity maps with the same channel. (Default=-1)
	 */
	int16_t channel{-1};

	/** The pose of this sensor in robot's local coordinates.
	 */
	mrpt::poses::CPose3D sensorPose;

	/** 1-sigma of the sensor Gaussian noise (in the same normalized units than
	 * \a reflectivityLevel)
	 */
	float sensorStdNoise{0.2f};

	// See base class docs
	void getSensorPose(mrpt::poses::CPose3D& out_sensorPose) const override
	{
		out_sensorPose = sensorPose;
	}
	void setSensorPose(const mrpt::poses::CPose3D& newSensorPose) override
	{
		sensorPose = newSensorPose;
	}
	void getDescriptionAsText(std::ostream& o) const override;

};  // End of class def.

}  // namespace mrpt::obs
