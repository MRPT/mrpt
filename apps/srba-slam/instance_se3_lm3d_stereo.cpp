/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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



