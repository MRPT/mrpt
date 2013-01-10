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

#include <mrpt/poses/CPose3DQuat.h>

namespace mrpt { namespace srba {

	/** @name Sensor models: one for each combination of {landmark_parameterization,observation_type}
	    @{ */

	/** Sensor model: 3D landmarks in Euclidean coordinates + Monocular camera observations (no distortion) */
	template <>
	struct sensor_model<landmark_traits_Euclidean3D,observation_traits_MonocularCamera>
	{
		// --------------------------------------------------------------------------------
		// Typedefs for the sake of generality in the signature of methods below:
		//   *DONT FORGET* to change these when writing new sensor models.
		// --------------------------------------------------------------------------------
		typedef observation_traits_MonocularCamera  OBS_T;  
		typedef landmark_traits_Euclidean3D         LANDMARK_T;
		// --------------------------------------------------------------------------------

		static const size_t OBS_DIMS = OBS_T::OBS_DIMS;
		static const size_t LM_DIMS  = LANDMARK_T::LM_DIMS;
		
		typedef Eigen::Matrix<double,OBS_DIMS,LM_DIMS>  TJacobian_dh_dx;     //!< A Jacobian of the correct size for each dh_dx
		typedef landmark_traits<LANDMARK_T>::point_t    point_t;             //!< a 2D or 3D point
		typedef OBS_T::TObservationParams               TObservationParams;


		/** Executes the observation model:  
		  * \param[out] out_obs The output of the predicted sensor value
		  * \param[in] base_pose_wrt_observer The relative pose of the observed landmark's base KF, wrt to the current observer KF.
		  * \param[in] lm_pos The relative landmark position wrt its base KF.
		  * \param[in] params The sensor-specific parameters.
		  */
		template <class POSE_T>
		static void observe(
			observation_traits<OBS_T>::array_obs_t              & out_obs, 
			const POSE_T                                        & base_pose_wrt_observer,
			const landmark_traits<LANDMARK_T>::array_landmark_t & lm_pos,
			const OBS_T::TObservationParams                     & params)
		{
			double x,y,z; // wrt cam (local coords)
			base_pose_wrt_observer.composePoint(lm_pos[0],lm_pos[1],lm_pos[2], x,y,z);
			ASSERT_(z!=0)

			// Pinhole model:
			out_obs[0] = params.camera_calib.cx() + params.camera_calib.fx() * x/z;
			out_obs[1] = params.camera_calib.cy() + params.camera_calib.fy() * y/z;
		}

		/** Evaluates the partial Jacobian dh_dx:
		  * \code
		  *            d h(x')
		  * dh_dx = -------------
		  *             d x' 
		  *
		  * \endcode
		  *  With: 
		  *    - x' = x^{j,i}_l  The relative location of the observed landmark wrt to the robot/camera at the instant of observation. (See notation on papers)
		  *    - h(x): Observation model: h(): landmark location --> observation
		  * 
		  * \param[out] dh_dx The output matrix Jacobian. Values at input are undefined (i.e. they cannot be asssumed to be zeros by default).
		  * \param[in]  xji_l The relative location of the observed landmark wrt to the robot/camera at the instant of observation.
		  * \param[in] sensor_params Sensor-specific parameters, as set by the user.
		  *
		  * \return true if the Jacobian is well-defined, false to mark it as ill-defined and ignore it during this step of the optimization
		  */
		static bool eval_jacob_dh_dx(
			TJacobian_dh_dx          & dh_dx,
			const point_t            & xji_l, 
			const TObservationParams & sensor_params)
		{
			// If the point is behind us, mark this Jacobian as invalid. This is probably a temporary situation until we get closer to the optimum.
			if (xji_l.z<=0)
				return false;

			const double pz_inv = 1.0/xji_l.z;
			const double pz_inv2 = pz_inv*pz_inv;

			const double cam_fx = sensor_params.camera_calib.fx();
			const double cam_fy = sensor_params.camera_calib.fy();

			dh_dx.coeffRef(0,0)=  cam_fx * pz_inv;
			dh_dx.coeffRef(0,1)=  0;
			dh_dx.coeffRef(0,2)=  -cam_fx * xji_l.x * pz_inv2;

			dh_dx.coeffRef(1,0)=  0;
			dh_dx.coeffRef(1,1)=  cam_fy * pz_inv;
			dh_dx.coeffRef(1,2)=  -cam_fy * xji_l.y * pz_inv2;

			return true;
		}

		/** Inverse observation model for firs-seen landmarks. Needed to avoid having landmarks at (0,0,0) which 
		  *  leads to undefined Jacobians. This is invoked only when both "unknown_relative_position_init_val" and "is_fixed" are "false" 
		  *  in an observation. 
		  * The LM location must not be exact at all, just make sure it doesn't have an undefined Jacobian.
		  *
		  * \param[out] out_lm_pos The relative landmark position wrt the current observing KF.
		  * \param[in]  obs The observation itself.
		  * \param[in]   params The sensor-specific parameters.
		  */
		static void inverse_sensor_model(
			landmark_traits<LANDMARK_T>::array_landmark_t & out_lm_pos,
			const observation_traits<OBS_T>::obs_data_t   & obs, 
			const OBS_T::TObservationParams               & params)
		{
			//     ¬ +Z 
			//    / 
			//   / 
			//  +----> +X
			//  |
			//  |
			//  V +Y
			//
			const double fx = params.camera_calib.fx();
			const double fy = params.camera_calib.fy();
			const double cx = params.camera_calib.cx();
			const double cy = params.camera_calib.cy();
			out_lm_pos[0] = (obs.px.x-cx)/fx;
			out_lm_pos[1] = (obs.px.y-cy)/fy;
			out_lm_pos[2] = 1; // Depth in Z (undefined for monocular cameras, just use any !=0 value)
		}

	};  // end of struct sensor_model<landmark_traits_Euclidean3D,observation_traits_MonocularCamera>

	// -------------------------------------------------------------------------------------------------------------

	/** Sensor model: 3D landmarks in Euclidean coordinates + Stereo camera observations (no distortion) */
	template <>
	struct sensor_model<landmark_traits_Euclidean3D,observation_traits_StereoCamera>
	{
		// --------------------------------------------------------------------------------
		// Typedefs for the sake of generality in the signature of methods below:
		//   *DONT FORGET* to change these when writing new sensor models.
		// --------------------------------------------------------------------------------
		typedef observation_traits_StereoCamera     OBS_T;  
		typedef landmark_traits_Euclidean3D         LANDMARK_T;
		// --------------------------------------------------------------------------------

		static const size_t OBS_DIMS = OBS_T::OBS_DIMS;
		static const size_t LM_DIMS  = LANDMARK_T::LM_DIMS;
		
		typedef Eigen::Matrix<double,OBS_DIMS,LM_DIMS>  TJacobian_dh_dx;     //!< A Jacobian of the correct size for each dh_dx
		typedef landmark_traits<LANDMARK_T>::point_t    point_t;             //!< a 2D or 3D point
		typedef OBS_T::TObservationParams               TObservationParams;


		/** Executes the observation model:  
		  * \param[out] out_obs The output of the predicted sensor value
		  * \param[in] base_pose_wrt_observer The relative pose of the observed landmark's base KF, wrt to the current observer KF.
		  * \param[in] lm_pos The relative landmark position wrt its base KF.
		  * \param[in] params The sensor-specific parameters.
		  */
		template <class POSE_T>
		static void observe(
			observation_traits<OBS_T>::array_obs_t              & out_obs, 
			const POSE_T                                        & base_pose_wrt_observer,
			const landmark_traits<LANDMARK_T>::array_landmark_t & lm_pos,
			const OBS_T::TObservationParams                     & params)
		{
			double lx,ly,lz; // wrt cam (local coords)
			base_pose_wrt_observer.composePoint(lm_pos[0],lm_pos[1],lm_pos[2], lx,ly,lz);
			ASSERT_(lz!=0)

			// Pinhole model: Left camera.
			const mrpt::utils::TCamera &lc = params.camera_calib.leftCamera;
			out_obs[0] = lc.cx() + lc.fx() * lx/lz;
			out_obs[1] = lc.cy() + lc.fy() * ly/lz;

			// Project point relative to right-camera:
			const mrpt::poses::CPose3DQuat R2L = -params.camera_calib.rightCameraPose; // R2L = (-) Left-to-right_camera_pose
			
			// base_wrt_right_cam = ( (-)L2R ) (+) L2Base
			//mrpt::poses::CPose3DQuat base_wrt_right_cam(mrpt::poses::UNINITIALIZED_POSE);
			//base_wrt_right_cam.composeFrom(R2L,mrpt::poses::CPose3DQuat(base_pose_wrt_observer));
			
			double rx,ry,rz; // wrt cam (local coords)
			//base_wrt_right_cam.composePoint(lx,ly,lz, rx,ry,rz);

			// xji_l_right = R2L (+) Xji_l
			R2L.composePoint(lx,ly,lz, rx,ry,rz);
			ASSERT_(rz!=0)

			// Pinhole model: Right camera.
			const mrpt::utils::TCamera &rc = params.camera_calib.rightCamera;
			out_obs[2] = rc.cx() + rc.fx() * rx/rz;
			out_obs[3] = rc.cy() + rc.fy() * ry/rz;
		}

		/** Evaluates the partial Jacobian dh_dx:
		  * \code
		  *            d h(x')
		  * dh_dx = -------------
		  *             d x' 
		  *
		  * \endcode
		  *  With: 
		  *    - x' = x^{j,i}_l  The relative location of the observed landmark wrt to the robot/camera at the instant of observation. (See notation on papers)
		  *    - h(x): Observation model: h(): landmark location --> observation
		  * 
		  * \param[out] dh_dx The output matrix Jacobian. Values at input are undefined (i.e. they cannot be asssumed to be zeros by default).
		  * \param[in]  xji_l The relative location of the observed landmark wrt to the robot/camera at the instant of observation.
		  * \param[in] sensor_params Sensor-specific parameters, as set by the user.
		  *
		  * \return true if the Jacobian is well-defined, false to mark it as ill-defined and ignore it during this step of the optimization
		  */
		static bool eval_jacob_dh_dx(
			TJacobian_dh_dx          & dh_dx,
			const point_t            & xji_l, 
			const TObservationParams & sensor_params)
		{
			// If the point is behind us, mark this Jacobian as invalid. This is probably a temporary situation until we get closer to the optimum.
			if (xji_l.z<=0)
				return false;
			
			// Left camera:
			{
				const double pz_inv = 1.0/xji_l.z;
				const double pz_inv2 = pz_inv*pz_inv;

				const double cam_fx = sensor_params.camera_calib.leftCamera.fx();
				const double cam_fy = sensor_params.camera_calib.leftCamera.fy();

				dh_dx.coeffRef(0,0)=  cam_fx * pz_inv;
				dh_dx.coeffRef(0,1)=  0;
				dh_dx.coeffRef(0,2)=  -cam_fx * xji_l.x * pz_inv2;

				dh_dx.coeffRef(1,0)=  0;
				dh_dx.coeffRef(1,1)=  cam_fy * pz_inv;
				dh_dx.coeffRef(1,2)=  -cam_fy * xji_l.y * pz_inv2;
			}

			// Right camera:
			point_t   xji_l_right; // xji_l_right = R2L (+) Xji_l
			const mrpt::poses::CPose3DQuat R2L = -sensor_params.camera_calib.rightCameraPose; // R2L = (-) Left-to-right_camera_pose
			R2L.composePoint(xji_l, xji_l_right);
			{
				const double pz_inv = 1.0/xji_l_right.z;
				const double pz_inv2 = pz_inv*pz_inv;

				const double cam_fx = sensor_params.camera_calib.rightCamera.fx();
				const double cam_fy = sensor_params.camera_calib.rightCamera.fy();

				dh_dx.coeffRef(2,0)=  cam_fx * pz_inv;
				dh_dx.coeffRef(2,1)=  0;
				dh_dx.coeffRef(2,2)=  -cam_fx * xji_l_right.x * pz_inv2;

				dh_dx.coeffRef(3,0)=  0;
				dh_dx.coeffRef(3,1)=  cam_fy * pz_inv;
				dh_dx.coeffRef(3,2)=  -cam_fy * xji_l_right.y * pz_inv2;
			}

			return true;
		}

		/** Inverse observation model for firs-seen landmarks. Needed to avoid having landmarks at (0,0,0) which 
		  *  leads to undefined Jacobians. This is invoked only when both "unknown_relative_position_init_val" and "is_fixed" are "false" 
		  *  in an observation. 
		  * The LM location must not be exact at all, just make sure it doesn't have an undefined Jacobian.
		  *
		  * \param[out] out_lm_pos The relative landmark position wrt the current observing KF.
		  * \param[in]  obs The observation itself.
		  * \param[in]   params The sensor-specific parameters.
		  */
		static void inverse_sensor_model(
			landmark_traits<LANDMARK_T>::array_landmark_t & out_lm_pos,
			const observation_traits<OBS_T>::obs_data_t   & obs, 
			const OBS_T::TObservationParams               & params)
		{
			//     ¬ +Z 
			//    / 
			//   / 
			//  +----> +X
			//  |
			//  |
			//  V +Y
			//
			const double fxl = params.camera_calib.leftCamera.fx();
			const double fyl = params.camera_calib.leftCamera.fy();
			const double cxl = params.camera_calib.leftCamera.cx();
			const double cyl = params.camera_calib.leftCamera.cy();
			const double disparity = std::max(0.001f, obs.left_px.x - obs.right_px.x);
			const double baseline  = params.camera_calib.rightCameraPose.x();
			const double Z = fxl*baseline/disparity;
			out_lm_pos[0] = (obs.left_px.x - cxl)*Z/fxl;
			out_lm_pos[1] = (obs.left_px.y - cyl)*Z/fyl;
			out_lm_pos[2] = Z;
		}

	};  // end of struct sensor_model<landmark_traits_Euclidean3D,observation_traits_StereoCamera>



	/** @} */

} } // end NS