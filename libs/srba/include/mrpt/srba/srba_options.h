/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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

#pragma once

#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/lightweight_geom_data.h>

namespace mrpt
{
namespace srba
{
	/** \defgroup mrpt_srba_options Struct traits used for setting different SRBA options at compile time
		* \ingroup mrpt_srba_grp */

	/** \addtogroup mrpt_srba_options
		* @{ */

		/** Usage: A possible type for RBA_OPTIONS::sensor_pose_on_robot_t. 
		  * Meaning: The robot pose and the sensor pose coincide, i.e. the sensor pose on the robot is the identitity transformation.  */
		struct sensor_pose_on_robot_none
		{
			/** In this case there are no needed parameters */
			struct parameters_t 
			{
			};

			/** Typedefs for determining whether the result of combining a KF pose (+) a sensor pose leads to a SE(2) or SE(3) pose */
			template <size_t KF_POSE_DIMS> struct resulting_pose_t;
			template <> struct resulting_pose_t<3> { typedef mrpt::poses::CPose2D pose_t; };
			template <> struct resulting_pose_t<6> { typedef mrpt::poses::CPose3D pose_t; };
			
			template <class POSE>
			static inline void robot2sensor(const POSE & robot, POSE & sensor, const parameters_t &p) { 
				sensor = robot;
			}

			/** Take into account the possible displacement of the sensor wrt the keyframe when evaluating the Jacobian dh_dx */
			template <class MATRIX>
			static inline void jacob_dh_dx_rotate( MATRIX & dh_dx, const parameters_t &p) { 
				/* nothing to do, since there's no displacement */ 
			} 
			template <class LANDMARK_T>
			static inline void sensor2robot_point(typename landmark_traits<LANDMARK_T>::array_landmark_t & pt, const parameters_t &p) {
				/* nothing to do, since there's no displacement */ 
			}
		};

		/** Usage: A possible type for RBA_OPTIONS::sensor_pose_on_robot_t. 
		  * Meaning: The sensor is located at an arbitrary SE(3) pose wrt the robot reference frame. */
		struct sensor_pose_on_robot_se3
		{
			/** In this case there are no needed parameters */
			struct parameters_t
			{
				mrpt::poses::CPose3D  relative_pose;
			};

			/** Typedefs for determining whether the result of combining a KF pose (+) a sensor pose leads to a SE(2) or SE(3) pose */
			template <size_t KF_POSE_DIMS> struct resulting_pose_t;
			template <> struct resulting_pose_t<3> { typedef mrpt::poses::CPose3D pose_t; };
			template <> struct resulting_pose_t<6> { typedef mrpt::poses::CPose3D pose_t; };

			template <class KF_POSE>
			static inline void robot2sensor(const KF_POSE & robot, mrpt::poses::CPose3D & sensor, const parameters_t &p) { 
				sensor.composeFrom(robot,p.relative_pose);
			}
			/** Take into account the possible displacement of the sensor wrt the keyframe when evaluating the Jacobian dh_dx */
			template <class MATRIX>
			static inline void jacob_dh_dx_rotate( MATRIX & dh_dx, const parameters_t &p) { 
				// dh(R2S+x)_dx = dh(x')_dx' * d(x')_dx
				dh_dx = dh_dx * p.relative_pose.getRotationMatrix();
			} 
			
			template <class LANDMARK_T>
			static inline void sensor2robot_point(typename landmark_traits<LANDMARK_T>::array_landmark_t & pt, const parameters_t &p) {
				landmark_traits<LANDMARK_T>::composePosePoint(pt, p.relative_pose);
			}
		};


		/** @} */

} // End of namespace
} // end of namespace
