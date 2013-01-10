
#pragma once

#include <mrpt/srba.h>
#include <mrpt/slam/CSimplePointsMap.h>

int run_demo_simul(int argc, char**argv);

size_t load_simulated_dataset(
	const std::string & FILE_PATH_AND_PREFIX,
	mrpt::math::CMatrixD   &OBS,
	mrpt::aligned_containers<mrpt::poses::CPose3DQuat>::vector_t  &GT_path,
	mrpt::slam::CSimplePointsMap  & GT_MAP,
	mrpt::utils::TCamera          & CAM_CALIB,
	const int verbose_level );


void optimization_feedback(unsigned int iter, const double total_sq_err, const double mean_sqroot_error);
