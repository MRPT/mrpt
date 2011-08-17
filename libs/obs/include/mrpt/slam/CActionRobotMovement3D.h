/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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
