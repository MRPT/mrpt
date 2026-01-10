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

// Note: Matrices unit tests have been split in different files since
// building them with eigen3 eats a lot of RAM and may be a problem while
// compiling in small systems.

#include <gtest/gtest.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/random.h>

#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace std;

const double dat_A[] = {4, 5, 8, -2, 1, 3};
const double dat_B[] = {2, 6, 9, 8};
const double dat_Cok[] = {53, 64, -2, 32, 29, 30};

#define CHECK_AND_RET_ERROR(_COND_, _MSG_) EXPECT_FALSE(_COND_) << _MSG_;

TEST(Matrices, DynMat_size)
{
  CMatrixDouble A(3, 2, dat_A);
  EXPECT_EQ(A.rows(), 3);
  EXPECT_EQ(A.cols(), 2);
  EXPECT_EQ(A.size(), 6U);
}

TEST(Matrices, A_times_B_dyn)
{
  // Dyn. size, double.
  CMatrixDouble A(3, 2, dat_A);
  CMatrixDouble B(2, 2, dat_B);
  CMatrixDouble C = CMatrixDouble(A * B);
  CMatrixDouble C_ok(3, 2, dat_Cok);
  CMatrixDouble err = C - C_ok;
  EXPECT_NEAR(0, fabs(err.sum()), 1e-5) << "A:   " << A << "B:   " << B << "A*B: " << C << endl;
}

TEST(Matrices, A_times_B_fix)
{
  // Fix. size, double.
  CMatrixFixed<double, 3, 2> A(dat_A);
  CMatrixFixed<double, 2, 2> B(dat_B);
  CMatrixFixed<double, 3, 2> C, C_ok(dat_Cok), Err;

  C = A * B;
  Err = C.asEigen() - CMatrixFixed<double, 3, 2>(C_ok).asEigen();

  EXPECT_NEAR(0, fabs(Err.asEigen().array().sum()), 1e-5);
}

TEST(Matrices, EigenVal2x2dyn)
{
  const double dat_C1[] = {14.6271, 5.8133, 5.8133, 16.8805};
  CMatrixDouble C1(2, 2, dat_C1);

  CMatrixDouble C1_V;
  std::vector<double> C1_Ds;
  C1.eig(C1_V, C1_Ds);

  CMatrixDouble C1_D;
  C1_D.setDiagonal(C1_Ds);

  CMatrixDouble C1_RR = CMatrixDouble(C1_V.asEigen() * C1_D.asEigen() * C1_V.transpose());
  EXPECT_NEAR((C1_RR - C1).sum_abs(), 0, 1e-4);
}

TEST(Matrices, eig_symmetric)
{
  // Test by looking only at the lower triangular-part:
  {
    const double dat_C[] = {14.6271, 0, 5.8133, 16.8805};
    CMatrixDouble22 C(dat_C);

    CMatrixDouble22 eig_vecs;
    std::vector<double> eig_vals;
    C.eig_symmetric(eig_vecs, eig_vals);

    EXPECT_EQ(eig_vals.size(), 2UL);
    EXPECT_NEAR(eig_vals[0], 9.83232131811656, 1e-4);
    EXPECT_NEAR(eig_vals[1], 21.67527868188344, 1e-4);
  }
  {
    CMatrixDouble22 C({1.0, 0.5, 0.5, 0.25});

    CMatrixDouble22 eig_vecs;
    std::vector<double> eig_vals;
    C.eig_symmetric(eig_vecs, eig_vals);

    EXPECT_EQ(eig_vals.size(), 2UL);
    EXPECT_NEAR(eig_vals[0], 0, 1e-3);
    EXPECT_TRUE(eig_vals[0] >= 0);
    EXPECT_NEAR(eig_vals[1], 1.25, 1e-3);

    EXPECT_NEAR(std::atan(eig_vecs(1, 0) / eig_vecs(0, 0)), std::atan(-0.894427 / 0.447213), 1e-3);
    EXPECT_NEAR(std::atan(eig_vecs(1, 1) / eig_vecs(0, 1)), std::atan(-0.447213 / -0.894427), 1e-3);
  }
}

TEST(Matrices, EigenVal3x3dyn)
{
  const double dat_C1[] = {8, 6, 1, 6, 9, 4, 1, 4, 10};
  CMatrixDouble C1(3, 3, dat_C1);

  CMatrixDouble C1_V;
  std::vector<double> C1_Ds;
  C1.eig(C1_V, C1_Ds);

  CMatrixDouble C1_D;
  C1_D.setDiagonal(C1_Ds);

  CMatrixDouble C1_RR = CMatrixDouble(C1_V.asEigen() * C1_D.asEigen() * C1_V.transpose());
  EXPECT_NEAR((C1_RR - C1).sum_abs(), 0, 1e-4);
}

TEST(Matrices, EigenVal2x2fix)
{
  const double dat_C1[] = {14.6271, 5.8133, 5.8133, 16.8805};
  CMatrixDouble22 C1(dat_C1);

  CMatrixDouble22 C1_V;
  std::vector<double> C1_Ds;
  C1.eig(C1_V, C1_Ds);

  CMatrixDouble22 C1_D;
  C1_D.setDiagonal(C1_Ds);

  CMatrixDouble22 C1_RR = CMatrixDouble22(C1_V.asEigen() * C1_D.asEigen() * C1_V.transpose());
  EXPECT_NEAR((C1_RR - C1).sum_abs(), 0, 1e-4);
}

TEST(Matrices, EigenVal3x3fix)
{
  const double dat_C1[] = {8, 6, 1, 6, 9, 4, 1, 4, 10};
  CMatrixDouble33 C1(dat_C1);

  CMatrixDouble33 C1_V;
  std::vector<double> C1_Ds;
  C1.eig(C1_V, C1_Ds);

  CMatrixDouble33 C1_D;
  C1_D.setDiagonal(C1_Ds);

  CMatrixDouble33 C1_RR = CMatrixDouble33(C1_V.asEigen() * C1_D.asEigen() * C1_V.transpose());
  EXPECT_NEAR((C1_RR - C1).sum_abs(), 0, 1e-4);
}
