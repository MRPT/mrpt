/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <gtest/gtest.h>
#include <mrpt/math/robust_kernels.h>

#include <algorithm>
#include <cmath>

using namespace mrpt;
using namespace mrpt::math;
using namespace std;

// Test data tables:
// Lists of: E2 (squared error) ;  PARAM2 (squared kernel parameter)  ;  R2
// (expected robustified output)   ; R' (1st derivative)   ; R'' (2nd
// derivative)
// =============  Kernel: None (plain least squares)
const double list_test_kernel_none[][5] = {
    { 0.0, .0,  0.0, 1.0, 0.0},
    { 1.0, .0,  1.0, 1.0, 0.0},
    {10.0, .0, 10.0, 1.0, 0.0}
};

// =============  Kernel: Pseudo-Huber
const double list_test_kernel_pshb[][5] = {
    {0.0, 1.0,      0.0,      1.0,       -0.5},
    {0.0, 4.0,      0.0,      1.0,     -0.125},
    {0.0, 9.0,      0.0,      1.0, -0.0555556},
    {1.0, 1.0, 0.828427, 0.707107,  -0.176777},
    {1.0, 4.0, 0.944272, 0.894427, -0.0894427},
    {1.0, 9.0, 0.973666, 0.948683, -0.0474342},
    {4.0, 1.0,  2.47214, 0.447214, -0.0447214},
    {4.0, 4.0,  3.31371, 0.707107, -0.0441942},
    {4.0, 9.0,  3.63331,  0.83205, -0.0320019}
};

template <TRobustKernelType KERNEL_TYPE>
void tester_robust_kernel(const double table[][5], const size_t N)
{
  RobustKernel<KERNEL_TYPE> rKernel;

  for (size_t i = 0; i < N; i++)
  {
    const double e2 = table[i][0];
    const double param2 = table[i][1];

    double rhop, rhopp;
    rKernel.param_sq = param2;
    double r2 = rKernel.eval(e2, rhop, rhopp);

    const double expected_r2 = table[i][2];
    const double expected_rhop = table[i][3];
    const double expected_rhopp = table[i][4];

    EXPECT_NEAR(r2, expected_r2, 1e-5);
    EXPECT_NEAR(rhop, expected_rhop, 1e-5);
    EXPECT_NEAR(rhopp, expected_rhopp, 1e-5);
  }
}

TEST(RobustKernels, PlainLeastSquares)
{
  const size_t N = sizeof(list_test_kernel_none) / sizeof(list_test_kernel_none[0]);
  tester_robust_kernel<rkLeastSquares>(list_test_kernel_none, N);
}

TEST(RobustKernels, PseudoHuber)
{
  const size_t N = sizeof(list_test_kernel_pshb) / sizeof(list_test_kernel_pshb[0]);
  tester_robust_kernel<rkPseudoHuber>(list_test_kernel_pshb, N);
}

TEST(RobustKernels, robustKernelDerivativesWrtSquaredError)
{
  // eval() returns 2*rho and its 1st/2nd derivatives w.r.t. the squared error
  for (const double param_sq : {0.25, 1.0, 9.0})
  {
    mrpt::math::RobustKernel<mrpt::math::rkPseudoHuber> k;
    k.param_sq = param_sq;
    for (const double r2 : {0.01, 0.5, 2.0, 30.0})
    {
      double d1 = 0;
      double d2 = 0;
      const double f = k.eval(r2, d1, d2);
      EXPECT_NEAR(f, 2 * param_sq * (std::sqrt(1 + r2 / param_sq) - 1), 1e-12);
      const double h = 1e-5 * std::max(1.0, r2);
      double tmp1 = 0;
      double tmp2 = 0;
      const double fp = k.eval(r2 + h, tmp1, tmp2);
      const double fm = k.eval(r2 - h, tmp1, tmp2);
      EXPECT_NEAR(d1, (fp - fm) / (2 * h), 1e-7);
      double d1p = 0;
      double d1m = 0;
      k.eval(r2 + h, d1p, tmp2);
      k.eval(r2 - h, d1m, tmp2);
      EXPECT_NEAR(d2, (d1p - d1m) / (2 * h), 1e-6);
    }
    // Small residuals behave like plain least squares
    double d1 = 0;
    double d2 = 0;
    EXPECT_NEAR(k.eval(1e-10, d1, d2), 1e-10, 1e-13);
    EXPECT_NEAR(d1, 1.0, 1e-9);
  }
}

TEST(RobustKernels, PseudoHuberSmallResidualsKeepPrecision)
{
  // For r2 << param_sq, 2*rho ~ r2 * (1 - r2/(4*param_sq))
  mrpt::math::RobustKernel<mrpt::math::rkPseudoHuber> k;
  k.param_sq = 4.0;
  for (const double r2 : {1e-14, 1e-10, 1e-6})
  {
    double d1 = 0;
    double d2 = 0;
    const double f = k.eval(r2, d1, d2);
    const double ref = r2 * (1 - r2 / (4 * k.param_sq));
    EXPECT_NEAR(f / ref, 1.0, 1e-13) << "r2=" << r2;
  }
}
