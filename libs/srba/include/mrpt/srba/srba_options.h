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

		/** \name  Types for RBA_OPTIONS::sensor_pose_on_robot_t 
		  *  @{ */

		/** Typedefs for determining whether the result of combining a KF pose (+) a sensor pose leads to a SE(2) or SE(3) pose */
		template <class SENSOR_POSE_CLASS, size_t KF_POSE_DIMS> struct resulting_pose_t;

		/** Usage: A possible type for RBA_OPTIONS::sensor_pose_on_robot_t.
		  * Meaning: The robot pose and the sensor pose coincide, i.e. the sensor pose on the robot is the identitity transformation.  */
		struct sensor_pose_on_robot_none
		{
			/** In this case there are no needed parameters */
			struct parameters_t
			{
			};

			/** Gets the pose of the sensor given the robot pose (robot->sensor) */
			template <class POSE>
			static inline void robot2sensor(const POSE & robot, POSE & sensor, const parameters_t &p) {
				sensor = robot;
			}
			/** Converts a pose relative to the robot coordinate frame (P) into a pose relative to the sensor (RES = P \ominus POSE_IN_ROBOT ) */
			template <class KF_POSE> 
			static inline void pose_robot2sensor(const KF_POSE & pose_wrt_robot, KF_POSE /*mrpt::poses::CPose3D*/ & pose_wrt_sensor, const parameters_t &p) {
				pose_wrt_sensor = pose_wrt_robot;
			}
			/** Converts a point relative to the robot coordinate frame (P) into a point relative to the sensor (RES = P \ominus POSE_IN_ROBOT ) */
			template <class POINT> 
			static inline void point_robot2sensor(const POINT & pt_wrt_robot, POINT & pt_wrt_sensor, const parameters_t &p) {
				pt_wrt_sensor=pt_wrt_robot;
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
		/** Typedefs for determining whether the result of combining a KF pose (+) a sensor pose leads to a SE(2) or SE(3) pose */
		template <> struct resulting_pose_t<sensor_pose_on_robot_none,3> { typedef mrpt::poses::CPose2D pose_t; };
		template <> struct resulting_pose_t<sensor_pose_on_robot_none,6> { typedef mrpt::poses::CPose3D pose_t; };

		/** Usage: A possible type for RBA_OPTIONS::sensor_pose_on_robot_t.
		  * Meaning: The sensor is located at an arbitrary SE(3) pose wrt the robot reference frame. */
		struct sensor_pose_on_robot_se3
		{
			/** In this case there are no needed parameters */
			struct parameters_t
			{
				mrpt::poses::CPose3D  relative_pose;
			};

			/** Gets the pose of the sensor given the robot pose (robot->sensor) */
			template <class KF_POSE> 
			static inline void robot2sensor(const KF_POSE & robot, mrpt::poses::CPose3D & sensor, const parameters_t &p) {
				sensor.composeFrom(robot,p.relative_pose);
			}
			/** Converts a pose relative to the robot coordinate frame (P) into a pose relative to the sensor (RES = P \ominus POSE_IN_ROBOT ) */
			template <class KF_POSE> 
			static inline void pose_robot2sensor(const KF_POSE & pose_wrt_robot, mrpt::poses::CPose3D & pose_wrt_sensor, const parameters_t &p) {
				pose_wrt_sensor.inverseComposeFrom(pose_wrt_robot,p.relative_pose);
			}
			/** Converts a point relative to the robot coordinate frame (P) into a point relative to the sensor (RES = P \ominus POSE_IN_ROBOT ) */
			template <class POINT> 
			static inline void point_robot2sensor(const POINT & pt_wrt_robot, POINT & pt_wrt_sensor, const parameters_t &p) {
				p.relative_pose.inverseComposePoint(pt_wrt_robot,pt_wrt_sensor);
			}
			/** Take into account the possible displacement of the sensor wrt the keyframe when evaluating the Jacobian dh_dx */
			template <class MATRIX>
			static inline void jacob_dh_dx_rotate( MATRIX & dh_dx, const parameters_t &p) {
				// dh(R2S+x)_dx = dh(x')_dx' * d(x')_dx
				dh_dx = dh_dx * p.relative_pose.getRotationMatrix().transpose();
			}

			template <class LANDMARK_T>
			static inline void sensor2robot_point(typename landmark_traits<LANDMARK_T>::array_landmark_t & pt, const parameters_t &p) {
				landmark_traits<LANDMARK_T>::composePosePoint(pt, p.relative_pose);
			}
		};
		/** Typedefs for determining whether the result of combining a KF pose (+) a sensor pose leads to a SE(2) or SE(3) pose */
		template <> struct resulting_pose_t<sensor_pose_on_robot_se3,3> { typedef mrpt::poses::CPose3D pose_t; };
		template <> struct resulting_pose_t<sensor_pose_on_robot_se3,6> { typedef mrpt::poses::CPose3D pose_t; };

		/** @} */ // end @name
		// ---------------------------------------------------------------------------------------------

		/** \name  Types for RBA_OPTIONS::obs_noise_matrix_t 
		  *  @{ */

		/** Usage: A possible type for RBA_OPTIONS::obs_noise_matrix_t.
		  * Meaning: The sensor noise matrix is the same for all observations and equal to \sigma * I(identity).  */
		struct observation_noise_identity
		{
			/** Observation noise parameters to be filled by the user in srba.parameters.obs_noise */
			struct parameters_t
			{
				/** One sigma of the Gaussian noise assumed for every component of observations (Default value: 1) */
				double std_noise_observations;

				parameters_t() : std_noise_observations (1.) 
				{ }
			};

			/** Internal struct for data that must be stored for each observation  */
			struct noise_data_per_obs_t
			{
				// None: all obs. have the same value="std_noise_observations" 
			};

			/** Must execute H+= J1^t * \Lambda * J2 */
			template <class MATRIX_H,class MATRIX_J1,class MATRIX_J2>
			inline static void accum_JtJ(MATRIX_H & H, const MATRIX_J1 & J1, const MATRIX_J2 &J2, const size_t obs_idx, const parameters_t & obs_noise_params) 
			{
				H.noalias() += J1.transpose() * J2;  // The constant scale factor 1/sigma will be applied in the end (below)
			}
			/** Do scaling, if applicable, to H after end of all calls to accum_JtJ()  */
			template <class MATRIX_H>
			inline static void scale_H(MATRIX_H & H, const parameters_t & obs_noise_params) 
			{
				ASSERTDEB_(obs_noise_params.std_noise_observations>0)
				H *= 1.0/obs_noise_params.std_noise_observations;
			}

			/** Must execute grad+= J^t * \Lambda * r */
			template <class VECTOR_GRAD,class MATRIX_J,class VECTOR_R>
			inline static void accum_Jtr(VECTOR_GRAD & g, const MATRIX_J & J, const VECTOR_R &r, const size_t obs_idx, const parameters_t & obs_noise_params) 
			{
				g.noalias() += J.transpose() * r;  // The constant scale factor 1/sigma will be applied in the end (below)
			}
			/** Do scaling, if applicable, to GRAD after end of all calls to accum_Jtr()  */
			template <class VECTOR_GRAD>
			inline static void scale_Jtr(VECTOR_GRAD & g, const parameters_t & obs_noise_params) 
			{
				ASSERTDEB_(obs_noise_params.std_noise_observations>0)
				g *= 1.0/obs_noise_params.std_noise_observations;
			}

		};  // end of "observation_noise_identity"

		/** Usage: A possible type for RBA_OPTIONS::obs_noise_matrix_t.
		  * Meaning: The sensor noise matrix is the same for all observations and equal to \sigma * I(identity).  */
		template <class OBS_TYPE>
		struct observation_noise_constant_matrix
		{
			static const size_t OBS_DIMS = OBS_TYPE::OBS_DIMS;  //!< The dimension of one observation

			typedef Eigen::Matrix<double,OBS_DIMS,OBS_DIMS>  obs_noise_matrix_t; //!< Type for symetric, positive-definite noise matrices.

			/** Observation noise parameters to be filled by the user in srba.parameters.obs_noise */
			struct parameters_t
			{
				/** The constant information matrix (inverse of covariance) for all the observations (\Lambda in common SLAM notation) */
				obs_noise_matrix_t  lambda;

				parameters_t() : lambda( obs_noise_matrix_t::Identity() ) 
				{ }
			};

			/** Internal struct for data that must be stored for each observation  */
			struct noise_data_per_obs_t
			{
				// None: all obs. have the same value
			};

			/** Must execute H+= J1^t * \Lambda * J2 */
			template <class MATRIX_H,class MATRIX_J1,class MATRIX_J2>
			inline static void accum_JtJ(MATRIX_H & H, const MATRIX_J1 & J1, const MATRIX_J2 &J2, const size_t obs_idx, const parameters_t & obs_noise_params) 
			{
				H.noalias() += J1.transpose() * obs_noise_params.lambda * J2;
			}
			/** Do scaling, if applicable, to H after end of all calls to accum_JtJ()  */
			template <class MATRIX_H>
			inline static void scale_H(MATRIX_H & H, const parameters_t & obs_noise_params) 
			{  // Nothing else to do.
			}

			/** Must execute grad+= J^t * \Lambda * r */
			template <class VECTOR_GRAD,class MATRIX_J,class VECTOR_R>
			inline static void accum_Jtr(VECTOR_GRAD & g, const MATRIX_J & J, const VECTOR_R &r, const size_t obs_idx, const parameters_t & obs_noise_params) 
			{
				g.noalias() += J.transpose() * obs_noise_params.lambda * r;
			}
			/** Do scaling, if applicable, to GRAD after end of all calls to accum_Jtr()  */
			template <class VECTOR_GRAD>
			inline static void scale_Jtr(VECTOR_GRAD & g, const parameters_t & obs_noise_params) 
			{  // Nothing else to do.
			}

		};  // end of "observation_noise_identity"


		/** @} */ // end @name


		/** @} */ // end \addtogroup

} // End of namespace
} // end of namespace
