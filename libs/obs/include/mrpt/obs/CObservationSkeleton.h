/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CObservationSkeleton_H
#define CObservationSkeleton_H

#include <mrpt/utils/CSerializable.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>

namespace mrpt
{
namespace obs
{

	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CObservationSkeleton , CObservation, OBS_IMPEXP)

	/** This class stores a skeleton as tracked by OPENNI2 & NITE2 libraries from PrimeSense sensors
	 *
	 * \sa CObservation
	 * \note Class introduced in MRPT 1.3.1
	 * \ingroup mrpt_obs_grp
	 */
	class OBS_IMPEXP CObservationSkeleton : public CObservation
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CObservationSkeleton )

	 public:
		/** Constructor.
		*/
		CObservationSkeleton(  ) :
			sensorPose(),
			head(), neck(), torso(), 
			left_shoulder(), left_elbow(), left_hand(), left_hip(), left_knee(), left_foot(),
			right_shoulder(), right_elbow(), right_hand(), right_hip(), right_knee(), right_foot()
		{}

		/** Destructor
		*/
		virtual ~CObservationSkeleton()
		{ }

		/** The pose of the sensor on the robot. */
		mrpt::poses::CPose3D  sensorPose;

		/** A generic joint for the skeleton observation */
		struct OBS_IMPEXP TSkeletonJoint
		{
			/** Default constructor */
			TSkeletonJoint() : x(.0), y(.0), z(.0), conf(.0) {}

			/** 3D position */
			double x,y,z;

			/** Confidence value [0...1] */
			double conf;
		};

		/** The skeleton joints (15)*/
		TSkeletonJoint head, neck, torso, 
			left_shoulder, left_elbow, left_hand, left_hip, left_knee, left_foot,
			right_shoulder, right_elbow, right_hand, right_hip, right_knee, right_foot;

		void getSensorPose( mrpt::poses::CPose3D & out_sensorPose) const  MRPT_OVERRIDE { out_sensorPose = sensorPose; }
		void setSensorPose( const mrpt::poses::CPose3D & newSensorPose ) MRPT_OVERRIDE { sensorPose = newSensorPose; }
		void getDescriptionAsText(std::ostream &o) const MRPT_OVERRIDE;

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CObservationSkeleton , CObservation, OBS_IMPEXP)

	} // End of namespace
} // End of namespace

#endif
