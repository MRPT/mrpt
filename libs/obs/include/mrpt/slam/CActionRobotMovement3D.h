/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef CActionRobotMovement3D_H
#define CActionRobotMovement3D_H

#include <mrpt/slam/CAction.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>

namespace mrpt
{
namespace slam
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
		vector_float			velocities;

	}; // End of class def.


	} // End of namespace
} // End of namespace

#endif
