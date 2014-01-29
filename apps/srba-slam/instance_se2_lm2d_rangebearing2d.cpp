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

#include "CDatasetParser_RangeBearing2D.h"

// Explicit instantiation:
template struct RBA_Run_Factory<kf2kf_poses::SE2, landmarks::Euclidean2D, observations::RangeBearing_2D>;

// Register this RBA problem:
RBA_Run_BasePtr my_creator_se2_lm2d_rb2d(RBASLAM_Params &config)
{
	if (config.arg_se2.isSet() && config.arg_lm2d.isSet() && config.arg_obs.getValue()=="RangeBearing_2D")
		return RBA_Run_Factory<kf2kf_poses::SE2,landmarks::Euclidean2D,observations::RangeBearing_2D>::create();

	return RBA_Run_BasePtr();
}

struct TMyRegister_se2_lm2d_rb2d
{
	TMyRegister_se2_lm2d_rb2d()
	{
		RBA_implemented_registry & reg = RBA_implemented_registry::getInstance();
		reg.doRegister( &my_creator_se2_lm2d_rb2d, "--se2 --lm-2d --obs RangeBearing_2D" );
	}
};

static TMyRegister_se2_lm2d_rb2d my_initializer_se2_lm2d_rb2d;

