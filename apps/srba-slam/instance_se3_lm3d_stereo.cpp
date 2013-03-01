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

/* See srba-slam_main.cpp for docs */

#include "srba-run-generic-impl.h"

#include "CDatasetParser_Stereo.h"

template <>
struct InitializerSensorParams<mrpt::srba::observations::StereoCamera>
{
	template <class RBA>
	static void init(RBA &rba, RBASLAM_Params &config)
	{
		// Load params from file:

		if (!config.arg_sensor_params.isSet())
			throw std::runtime_error("Error: --sensor-params-cfg-file is mandatory for this type of observations.");

		const std::string sCfgFile = config.arg_sensor_params.getValue();
		rba.parameters.sensor.camera_calib.loadFromConfigFile("CAMERA",mrpt::utils::CConfigFile(sCfgFile) );
		const double baseline = rba.parameters.sensor.camera_calib.rightCameraPose.x();
		ASSERT_(baseline!=0)

	}
};

// Specializations:
template <>
struct problem_options_traits_t<kf2kf_poses::SE3,landmarks::Euclidean3D,observations::StereoCamera>
{
	// Camera sensors have a different coordinate system wrt the robot (rotated yaw=-90, pitch=0, roll=-90)
	struct srba_options_t
	{
		typedef options::sensor_pose_on_robot_se3     sensor_pose_on_robot_t;
		typedef options::observation_noise_identity   obs_noise_matrix_t;      // The sensor noise matrix is the same for all observations and equal to \sigma * I(identity)
		typedef options::solver_LM_schur_dense_cholesky      solver_t;
	};
};

// Explicit instantiation:
template struct RBA_Run_Factory<kf2kf_poses::SE3,landmarks::Euclidean3D,observations::StereoCamera>;


// Register this RBA problem:
RBA_Run_BasePtr my_creator_se3_lm3d_stereo(RBASLAM_Params &config)
{
	if (config.arg_se3.isSet() && config.arg_lm3d.isSet() && config.arg_obs.getValue()=="StereoCamera")
		return RBA_Run_Factory<kf2kf_poses::SE3,landmarks::Euclidean3D,observations::StereoCamera>::create();

	return RBA_Run_BasePtr();
}

struct TMyRegister_se3_lm3d_stereo
{
	TMyRegister_se3_lm3d_stereo()
	{
		RBA_implemented_registry & reg = RBA_implemented_registry::getInstance();
		reg.doRegister( &my_creator_se3_lm3d_stereo, "--se3 --lm-3d --obs StereoCamera" );
	}
};

static TMyRegister_se3_lm3d_stereo my_initializer_se3_lm3d_stereo;



