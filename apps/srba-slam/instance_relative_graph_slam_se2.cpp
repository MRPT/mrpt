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

#include "CDatasetParser_RelGraphSLAM2D.h"

template <>
struct problem_options_traits_t<kf2kf_poses::SE2,landmarks::RelativePoses2D,observations::RelativePoses_2D>
{
	struct srba_options_t
	{
		typedef options::sensor_pose_on_robot_none   sensor_pose_on_robot_t;
		typedef options::observation_noise_constant_matrix<observations::RelativePoses_2D> obs_noise_matrix_t;      // The sensor noise matrix is the same for all observations and equal to an arbitrary matrix
		typedef options::solver_LM_no_schur_sparse_cholesky  solver_t;
	};
};

// Explicit instantiation:
template struct RBA_Run_Factory<kf2kf_poses::SE2,landmarks::RelativePoses2D,observations::RelativePoses_2D>;

// Register this RBA problem:
RBA_Run_BasePtr my_creator_rel_graph_slam_se2(RBASLAM_Params &config)
{
	if (config.arg_se2.isSet() && config.arg_graph_slam.isSet())
		return RBA_Run_Factory<kf2kf_poses::SE2,landmarks::RelativePoses2D,observations::RelativePoses_2D>::create();

	return RBA_Run_BasePtr();
}

struct TMyRegister_rel_graph_slam_se2
{
	TMyRegister_rel_graph_slam_se2()
	{
		RBA_implemented_registry & reg = RBA_implemented_registry::getInstance();
		reg.doRegister( &my_creator_rel_graph_slam_se2, "--se2 --graph-slam" );
	}
};

static TMyRegister_rel_graph_slam_se2 my_initializer_rel_graph_slam_se2;

