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
/** This class stores a skeleton as tracked by OPENNI2 & NITE2 libraries from
 * PrimeSense sensors
 *
 * \sa CObservation
 * \note Class introduced in MRPT 1.3.1
 * \ingroup mrpt_obs_grp
 */
class CObservationSkeleton : public CObservation
{
	DEFINE_SERIALIZABLE(CObservationSkeleton)

   public:
	/** Constructor.
	 */
	CObservationSkeleton()
		: sensorPose(),
		  head(),
		  neck(),
		  torso(),
		  left_shoulder(),
		  left_elbow(),
		  left_hand(),
		  left_hip(),
		  left_knee(),
		  left_foot(),
		  right_shoulder(),
		  right_elbow(),
		  right_hand(),
		  right_hip(),
		  right_knee(),
		  right_foot()
	{
	}

	/** Destructor
	 */
	~CObservationSkeleton() override = default;
	/** The pose of the sensor on the robot. */
	mrpt::poses::CPose3D sensorPose;

	/** A generic joint for the skeleton observation */
	struct TSkeletonJoint
	{
		/** Default constructor */
		TSkeletonJoint() = default;
		/** 3D position */
		double x{.0}, y{.0}, z{.0};
		/** Confidence value [0...1] */
		double conf{.0};
	};

	/** The skeleton joints (15)*/
	TSkeletonJoint head, neck, torso, left_shoulder, left_elbow, left_hand,
		left_hip, left_knee, left_foot, right_shoulder, right_elbow, right_hand,
		right_hip, right_knee, right_foot;

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
