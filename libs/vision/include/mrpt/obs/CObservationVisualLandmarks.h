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
#include <mrpt/maps/CLandmarksMap.h>

namespace mrpt::obs
{
/** Declares a class derived from "CObservation" that stores a Landmarks Map as
 * seen from a stereo camera at a given instant of time.
 *
 * \sa CLandmarksMap, CObservation
 * \ingroup mrpt_vision_grp
 */
class CObservationVisualLandmarks : public CObservation
{
	DEFINE_SERIALIZABLE(CObservationVisualLandmarks)

   public:
	/** Constructor */
	CObservationVisualLandmarks();

	/** The 3D pose of the reference camera relative to robot coordinates. */
	mrpt::poses::CPose3D refCameraPose;
	/** The landmarks, with coordinates origin in the camera reference system.
	 */
	mrpt::maps::CLandmarksMap landmarks;

	/** Implements the virtual method in charge of finding the likelihood
	 *between this
	 *   and another observation, probably only of the same derived class. The
	 *operator
	 *   may be asymmetric.
	 *
	 * \param anotherObs The other observation to compute likelihood with.
	 * \param anotherObsPose If known, the belief about the robot pose when the
	 *other observation was taken can be supplied here, or nullptr if it is
	 *unknown.
	 *
	 * \return Returns a likelihood measurement, in the range [0,1].
	 *	\exception std::exception On any error, as another observation being of
	 *an invalid class.
	 */
	float likelihoodWith(
		const mrpt::obs::CObservation* anotherObs,
		const mrpt::poses::CPosePDF* anotherObsPose = nullptr) const;

	// See base class docs
	void getSensorPose(mrpt::poses::CPose3D& out_sensorPose) const override
	{
		out_sensorPose = refCameraPose;
	}
	void setSensorPose(const mrpt::poses::CPose3D& newSensorPose) override
	{
		refCameraPose = newSensorPose;
	}
	void getDescriptionAsText(std::ostream& o) const override;

};  // End of class def.

}  // namespace mrpt::obs
