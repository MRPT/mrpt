/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2015, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/* See srba-slam_main.cpp for docs */

#include "srba-run-generic-impl.h"

#include "CDatasetParser_Cartesian_3D.h"

// Explicit instantiation:
template struct RBA_Run_Factory<kf2kf_poses::SE3,landmarks::Euclidean3D,observations::Cartesian_3D>;

// Register this RBA problem:
RBA_Run_BasePtr my_creator_se3_lm3d_cart3d(RBASLAM_Params &config)
{
	if (config.arg_se3.isSet() && config.arg_lm3d.isSet() && config.arg_obs.getValue()=="Cartesian_3D")
		return RBA_Run_Factory<kf2kf_poses::SE3,landmarks::Euclidean3D,observations::Cartesian_3D>::create();

	return RBA_Run_BasePtr();
}

struct TMyRegister_se3_lm3d_cart3d
{
	TMyRegister_se3_lm3d_cart3d()
	{
		RBA_implemented_registry & reg = RBA_implemented_registry::getInstance();
		reg.doRegister( &my_creator_se3_lm3d_cart3d, "--se3 --lm-3d --obs Cartesian_3D" );
	}
};

static TMyRegister_se3_lm3d_cart3d my_initializer_se3_lm3d_cart3d;


