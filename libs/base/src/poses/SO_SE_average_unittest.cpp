/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/poses/SO_SE_average.h>
#include <mrpt/math/wrap2pi.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

void run_test_so2_avrg(const double *angs, const size_t N, const double ang_correct_avr)
{
	SO_average<2> so_avr;
	for (size_t i=0;i<N;i++)
		so_avr.append(angs[i]);
	const double calc_avr = so_avr.get_average();
	EXPECT_NEAR(  mrpt::math::wrapToPi(ang_correct_avr-calc_avr), .0, 1e-6);
}

TEST(SE2_SE3_avrg,SO2_average)
{
	// Simple tests:
	{
		const double angs[] = {.1};
		const double ang_correct_avr = .1;
		run_test_so2_avrg(angs,sizeof(angs)/sizeof(angs[0]),ang_correct_avr);
	}
	{
		const double angs[] = {.0, M_PI };
		const double ang_correct_avr = .5*M_PI;
		run_test_so2_avrg(angs,sizeof(angs)/sizeof(angs[0]),ang_correct_avr);
	}
	{
		const double angs[] = {-0.75*M_PI, 0.75*M_PI };
		const double ang_correct_avr = 1.0*M_PI;
		run_test_so2_avrg(angs,sizeof(angs)/sizeof(angs[0]),ang_correct_avr);
	}
	{
		const double angs  [] = {-0.75*M_PI, 0.75*M_PI, 0.3*M_PI };
		//const double angs_w[] = {1.0, 1.0, 0.1 };
		const double ang_correct_avr = 2.3668403111754515;
		run_test_so2_avrg(angs,sizeof(angs)/sizeof(angs[0]),ang_correct_avr);
	}
	// Test launching an exception when there is no data:
	{
		const double dummy  [] = {0.};
		try
		{
			run_test_so2_avrg(dummy,0,0);
			GTEST_FAIL() << "An exception should have been raised before this point!!";
		} catch (std::exception &)
		{
			// This error is expected, it's OK.
		}
	}
}

void run_test_so3_avrg(const double *angs, const size_t N, const Eigen::Matrix3d & correct_avr)
{
	SO_average<3> so_avr;
	for (size_t i=0;i<N;i++) {
		mrpt::poses::CPose3D rot(0,0,0, angs[3*i+0],angs[3*i+1],angs[3*i+2]);
		so_avr.append(rot.getRotationMatrix());
	}
	Eigen::Matrix3d calc_avr = so_avr.get_average();
	EXPECT_NEAR(  (correct_avr-calc_avr).array().abs().sum(), .0, 1e-5);
}

TEST(SE2_SE3_avrg,SO3_average)
{
	// Simple tests:
	{
		const double angs[] = { .0,.0,.0 };
		const Eigen::Matrix3d correct_avr = mrpt::poses::CPose3D(0,0,0,  0,0,0).getRotationMatrix();
		run_test_so3_avrg(angs,sizeof(angs)/(3*sizeof(angs[0])),correct_avr);
	}
	{
		const double angs[] = {-.75*M_PI,.0,.0,   .75*M_PI,.0,.0  };
		const Eigen::Matrix3d correct_avr = mrpt::poses::CPose3D(0,0,0, M_PI,0,0).getRotationMatrix();
		run_test_so3_avrg(angs,sizeof(angs)/(3*sizeof(angs[0])),correct_avr);
	}
	{
		const double angs[] = {.0,-0.2,.0,   .0,0.2,.0  };
		const Eigen::Matrix3d correct_avr = mrpt::poses::CPose3D(0,0,0, 0,0,0).getRotationMatrix();
		run_test_so3_avrg(angs,sizeof(angs)/(3*sizeof(angs[0])),correct_avr);
	}
	{
		const double angs[] = {.0,.0,.3,   .0,.0,-.3  };
		const Eigen::Matrix3d correct_avr = mrpt::poses::CPose3D(0,0,0, 0,0,0).getRotationMatrix();
		run_test_so3_avrg(angs,sizeof(angs)/(3*sizeof(angs[0])),correct_avr);
	}
}
