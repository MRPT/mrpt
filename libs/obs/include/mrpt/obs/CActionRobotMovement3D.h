/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CActionRobotMovement3D_H
#define CActionRobotMovement3D_H

#include <mrpt/obs/CAction.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>

namespace mrpt
{
namespace obs
{
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CActionRobotMovement3D, CAction, OBS_IMPEXP )

	/** Represents a probabilistic 3D (6D) movement.
	*   Currently this can be determined from visual odometry for full 6D, or from wheel encoders for 2D movements only.
	*
	* \ingroup mrpt_obs_grp
	* \sa CAction
	*/
	class OBS_IMPEXP CActionRobotMovement3D : public CAction
	{
		// This must be added to any CSerializable derived class:
		DEFINE_SERIALIZABLE( CActionRobotMovement3D )

	public:
		/** A list of posible ways for estimating the content of a CActionRobotMovement3D object.
			*/
		enum TEstimationMethod
		{
			emOdometry = 0,
			emVisualOdometry
		};

		/** Constructor
		  */
		CActionRobotMovement3D();

		/** Destructor
		  */
		virtual ~CActionRobotMovement3D();

		/** The 3D pose change probabilistic estimation.
		  */
		poses::CPose3DPDFGaussian		poseChange;
		poses::CPose3DQuatPDFGaussian	poseChangeQuat;


		/** This fields indicates the way this estimation was obtained.
		  */
		TEstimationMethod		estimationMethod;

		/** Each "true" entry means that the corresponding "velocities" element contains valid data - There are 6 entries.
		  */
		vector_bool				hasVelocities;

		/** The velocity of the robot in each of 6D: v_x,v_y,v_z,v_yaw,v_pitch,v_roll (linear in meters/sec and angular in rad/sec).
		  */
		mrpt::math::CVectorFloat	velocities;

	}; // End of class def.
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( CActionRobotMovement3D, CAction, OBS_IMPEXP )


	} // End of namespace
} // End of namespace

#endif
