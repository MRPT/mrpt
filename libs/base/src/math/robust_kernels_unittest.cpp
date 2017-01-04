/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/math/robust_kernels.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace std;

// Test data tables:
// Lists of: E2 (squared error) ;  PARAM2 (squared kernel parameter)  ;  R2 (expected robustified output)   ; R' (1st derivative)   ; R'' (2nd derivative)
// =============  Kernel: None (plain least squares)
const double list_test_kernel_none[][5] =  {
	{  0.0          , .0        , 0.0            , 1.0                , 0.0              },
	{  1.0          , .0        , 1.0            , 1.0                , 0.0              },
	{  10.0         , .0        , 10.0           , 1.0                , 0.0              }
};

// =============  Kernel: Pseudo-Huber
const double list_test_kernel_pshb[][5] =  {
	{  0.0          , 1.0     , 0.0              , 1.0                ,-0.5              },
	{  0.0          , 4.0     , 0.0              , 1.0                ,-0.125            },
	{  0.0          , 9.0     , 0.0              , 1.0                ,-0.0555556        },
	{  1.0          , 1.0     , 0.828427         , 0.707107           ,-0.176777         },
	{  1.0          , 4.0     , 0.944272         , 0.894427           ,-0.0894427        },
	{  1.0          , 9.0     , 0.973666         , 0.948683           ,-0.0474342        },
	{  4.0          , 1.0     , 2.47214          , 0.447214           ,-0.0447214        },
	{  4.0          , 4.0     , 3.31371          , 0.707107           ,-0.0441942        },
	{  4.0          , 9.0     , 3.63331          , 0.83205            ,-0.0320019        }
};

template <TRobustKernelType KERNEL_TYPE>
void tester_robust_kernel(const double table[][5], const size_t N)
{
	RobustKernel<KERNEL_TYPE> rKernel;

	for (size_t i=0;i<N;i++)
	{
		const double e2     = table[i][0];
		const double param2 = table[i][1];

		double rhop, rhopp;
		rKernel.param_sq = param2;
		double r2 =  rKernel.eval(e2,rhop,rhopp);

		const double expected_r2    = table[i][2];
		const double expected_rhop  = table[i][3];
		const double expected_rhopp = table[i][4];
		
		EXPECT_NEAR(r2,expected_r2,1e-5);
		EXPECT_NEAR(rhop,expected_rhop,1e-5);
		EXPECT_NEAR(rhopp,expected_rhopp,1e-5);
	}
}

TEST(RobustKernels,PlainLeastSquares)
{
	const size_t N = sizeof(list_test_kernel_none)/sizeof(list_test_kernel_none[0]);
	tester_robust_kernel<rkLeastSquares>(list_test_kernel_none, N);
}

TEST(RobustKernels,PseudoHuber)
{
	const size_t N = sizeof(list_test_kernel_pshb)/sizeof(list_test_kernel_pshb[0]);
	tester_robust_kernel<rkPseudoHuber>(list_test_kernel_pshb, N);
}



