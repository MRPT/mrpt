/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

// Note: Matrices unit tests have been split in different files since
// building them with eigen3 eats a lot of RAM and may be a problem while
// compiling in small systems.

#include <gtest/gtest.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorFixed.h>
#include <mrpt/math/geometry.h>
#include <mrpt/math/ops_matrices.h>
#include <mrpt/random.h>
#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace std;

#define CHECK_AND_RET_ERROR(_COND_, _MSG_) EXPECT_FALSE(_COND_) << _MSG_;

TEST(Matrices, inv_4x4_fix)
{
	const double dat_A[] = {
		-0.710681653571291, 0.734469323344333,  -0.656414638791893,
		0.818771495864303,  1.044946492154568,  1.163592359608108,
		-1.069421407670914, 0.307916381104872,  0.185595851677470,
		0.116899590868673,  0.507691343481809,  -3.217842384231890,
		-0.214383515646621, -0.161495561253269, 1.303923696836841,
		0.261535721431038};
	CMatrixDouble44 A(dat_A);
	CMatrixDouble44 C = A.inverse();
	const double dat_AInv[] = {
		-0.741952742824035, 0.493481687552705, -0.134764164880760,
		0.083693424291000,  0.638324207063440, 0.519344439204238,
		0.264483337145361,  0.644307267615193, -0.037800456163779,
		0.131794126194075,  0.070338431705792, 0.828591793299072,
		-0.025568212209135, 0.068123300450057, -0.297834184749986,
		0.158964059763645};
	CMatrixDouble44 AInv(dat_AInv);
	CHECK_AND_RET_ERROR((AInv - C).sum_abs() > 1e-4, "Error in inv, 4x4 fix")
}

TEST(Matrices, inv_LLt_4x4_fix)
{
	const double dat_A[] = {
		// clang-format off
	    5.340050636150691, -5.233909093073407, -0.716270110219391,  2.459687431594893,
	   -5.233909093073407,  9.825652319628178,  5.408354798920869, -0.568364497163277,
	   -0.716270110219391,  5.408354798920869,  7.221045225242777,  1.571113904458352,
	    2.459687431594893, -0.568364497163277,  1.571113904458352,  2.339742034936232
		// clang-format on
	};
	CMatrixDouble44 A(dat_A);
	CMatrixDouble44 C = A.inverse_LLt();
	const double dat_AInv[] = {
		// clang-format off
	    2.1041492047454131,  1.1944136386884630, -0.3135215860855067, -1.7113463206284218,
	    1.1944136386884630,  0.9132606154380595, -0.3988774167033647, -0.7659551448058749,
	   -0.3135215860855067, -0.3988774167033647,  0.4163286875558479, -0.0468609127642805,
	   -1.7113463206284218, -0.7659551448058749, -0.0468609127642805,  2.0718776191839314
		// clang-format on
	};
	CMatrixDouble44 AInv(dat_AInv);
	CHECK_AND_RET_ERROR((AInv - C).sum_abs() > 1e-4, "Error in inv, 4x4 fix")
}

TEST(Matrices, inv_6x6_fix)
{
	const double dat_A[] = {
		363.769989013671875, 0.000000000000000,   316.429992675781250,
		0.000000000000000,   87.266998291015625,  0.000000000000000,
		101.540000915527344, 0.000000000000000,   478.709991455078125,
		0.000000000000000,   504.540008544921875, 0.000000000000000,
		1.000000000000000,   0.000000000000000,   1.000000000000000,
		0.000000000000000,   1.000000000000000,   0.000000000000000,
		0.000000000000000,   363.769989013671875, 0.000000000000000,
		316.429992675781250, 0.000000000000000,   87.266998291015625,
		0.000000000000000,   101.540000915527344, 0.000000000000000,
		478.709991455078125, 0.000000000000000,   504.540008544921875,
		0.000000000000000,   1.000000000000000,   0.000000000000000,
		1.000000000000000,   0.000000000000000,   1.000000000000000};
	CMatrixDouble66 A(dat_A);
	const CMatrixDouble66 C = A.inverse();
	const double dat_AInv[] = {
		-0.000303131460181, -0.002689371550382, 1.383348917627708,
		0.000000000000000,  0.000000000000000,  0.000000000000000,
		0.000000000000000,  0.000000000000000,  0.000000000000000,
		-0.000303131460181, -0.002689371550382, 1.383348917627708,
		0.004729457992255,  0.003244936115630,  -2.049925698035195,
		0.000000000000000,  0.000000000000000,  0.000000000000000,
		0.000000000000000,  0.000000000000000,  0.000000000000000,
		0.004729457992255,  0.003244936115630,  -2.049925698035195,
		-0.004426326532074, -0.000555564565248, 1.666576780407488,
		0.000000000000000,  0.000000000000000,  0.000000000000000,
		0.000000000000000,  0.000000000000000,  0.000000000000000,
		-0.004426326532074, -0.000555564565248, 1.666576780407488};
	CMatrixDouble66 AInv(dat_AInv);
	CHECK_AND_RET_ERROR(
		std::isnan(C(0, 0)) || !std::isfinite(C(0, 0)) ||
			(AInv - C).sum_abs() > 1e-4,
		"Error in inv, 6x6 fix")
}

TEST(Matrices, inv_6x6_dyn)
{
	const double dat_A[] = {
		363.769989013671875, 0.000000000000000,   316.429992675781250,
		0.000000000000000,   87.266998291015625,  0.000000000000000,
		101.540000915527344, 0.000000000000000,   478.709991455078125,
		0.000000000000000,   504.540008544921875, 0.000000000000000,
		1.000000000000000,   0.000000000000000,   1.000000000000000,
		0.000000000000000,   1.000000000000000,   0.000000000000000,
		0.000000000000000,   363.769989013671875, 0.000000000000000,
		316.429992675781250, 0.000000000000000,   87.266998291015625,
		0.000000000000000,   101.540000915527344, 0.000000000000000,
		478.709991455078125, 0.000000000000000,   504.540008544921875,
		0.000000000000000,   1.000000000000000,   0.000000000000000,
		1.000000000000000,   0.000000000000000,   1.000000000000000};
	CMatrixDouble A(6, 6, dat_A);
	CMatrixDouble C = A.inverse();
	const double dat_AInv[] = {
		-0.000303131460181, -0.002689371550382, 1.383348917627708,
		0.000000000000000,  0.000000000000000,  0.000000000000000,
		0.000000000000000,  0.000000000000000,  0.000000000000000,
		-0.000303131460181, -0.002689371550382, 1.383348917627708,
		0.004729457992255,  0.003244936115630,  -2.049925698035195,
		0.000000000000000,  0.000000000000000,  0.000000000000000,
		0.000000000000000,  0.000000000000000,  0.000000000000000,
		0.004729457992255,  0.003244936115630,  -2.049925698035195,
		-0.004426326532074, -0.000555564565248, 1.666576780407488,
		0.000000000000000,  0.000000000000000,  0.000000000000000,
		0.000000000000000,  0.000000000000000,  0.000000000000000,
		-0.004426326532074, -0.000555564565248, 1.666576780407488};
	CMatrixDouble AInv(6, 6, dat_AInv);
	CHECK_AND_RET_ERROR(
		std::isnan(C(0, 0)) || !std::isfinite(C(0, 0)) ||
			(AInv - C).sum_abs() > 1e-4,
		"Error in inv, 6x6 dyn")
}

TEST(Matrices, transpose)
{
	const double dat_A[] = {1, 2, 3, 4, 5, 6};
	const double dat_At[] = {1, 4, 2, 5, 3, 6};
	const CMatrixDouble A(2, 3, dat_A);
	const CMatrixDouble At(3, 2, dat_At);

	EXPECT_EQ(A.transpose(), At.asEigen());
	EXPECT_EQ(A.transpose().transpose(), A.asEigen());
}

TEST(Matrices, multiply_A_skew3)
{
	{
		const double dat_A[] = {1, 2, 3, 4, 5, 6};
		const CMatrixDouble A(2, 3, dat_A);
		const std::vector<double> v{1.0, 2.0, 3.0};
		const CMatrixDouble S = CMatrixDouble(mrpt::math::skew_symmetric3(v));

		CMatrixDouble R;
		mrpt::math::multiply_A_skew3(A, v, R);
		EXPECT_EQ(R.asEigen(), A.asEigen() * S.asEigen());
	}
	{
		const double dat_A[] = {1, 2, 3, 4, 5, 6};
		const double dat_v[] = {1, 2, 3};
		const CMatrixFixed<double, 2, 3> A(dat_A);
		const CVectorFixedDouble<3> v(dat_v);
		const CMatrixFixed<double, 3, 3> S = mrpt::math::skew_symmetric3(v);

		CMatrixFixed<double, 2, 3> R;
		mrpt::math::multiply_A_skew3(A, v, R);
		EXPECT_EQ(R.asEigen(), A.asEigen() * S.asEigen());
	}
}

TEST(Matrices, multiply_skew3_A)
{
	{
		const double dat_A[] = {1, 2, 3, 4, 5, 6};
		const CMatrixDouble A(3, 2, dat_A);
		const std::vector<double> v{1.0, 2.0, 3.0};
		const CMatrixDouble S = CMatrixDouble(mrpt::math::skew_symmetric3(v));

		CMatrixDouble R;
		mrpt::math::multiply_skew3_A(v, A, R);
		EXPECT_EQ(R.asEigen(), S.asEigen() * A.asEigen());
	}
	{
		const double dat_A[] = {1, 2, 3, 4, 5, 6};
		const double dat_v[] = {1, 2, 3};
		const CMatrixFixed<double, 3, 2> A(dat_A);
		const CVectorFixedDouble<3> v(dat_v);
		const CMatrixFixed<double, 3, 3> S = mrpt::math::skew_symmetric3(v);

		CMatrixFixed<double, 3, 2> R;
		mrpt::math::multiply_skew3_A(v, A, R);
		EXPECT_EQ(R.asEigen(), S.asEigen() * A.asEigen());
	}
}

TEST(Matrices, fromMatlabStringFormat)
{
	const char* mat1 = "[1 2 3;-3 -6 -5]";
	const double vals1[] = {1, 2, 3, -3, -6, -5};

	const char* mat2 =
		" [ 	  -8.2	 9.232 ; -2e+2		+6 ; 1.000  7 ] ";  // With tabs and
	// spaces...
	const double vals2[] = {-8.2, 9.232, -2e+2, +6, 1.000, 7};

	const char* mat3 = "[9]";
	const char* mat4 =
		"[1 2 3 4 5 6 7 9 10  ; 1 2 3 4 5 6 7 8 9 10 11]";  // An invalid matrix
	const char* mat5 = "[  ]";  // Empty
	const char* mat6 = "[ -405.200 42.232 ; 1219.600    -98.696 ]";  // M1 * M2

	const char* mat13 = "[9 8 7]";
	const char* mat31 = "[9; 8; 7]";

	CMatrixDouble M1, M2, M3, M4, M5, M6;

	if (!M1.fromMatlabStringFormat(mat1) ||
		(CMatrixFixed<double, 2, 3>(vals1) - M1).array().abs().sum() > 1e-4)
		GTEST_FAIL() << mat1;

	{
		CMatrixDouble M1b;
		if (!M1b.fromMatlabStringFormat(mat1) ||
			(CMatrixFixed<double, 2, 3>(vals1) - M1b).array().abs().sum() >
				1e-4)
			GTEST_FAIL() << mat1;
	}

	if (!M2.fromMatlabStringFormat(mat2) || M2.cols() != 2 || M2.rows() != 3 ||
		(CMatrixFixed<double, 3, 2>(vals2) - M2).array().abs().sum() > 1e-4)
		GTEST_FAIL() << mat2;

	{
		CMatrixDouble M2b;
		if (!M2b.fromMatlabStringFormat(mat2) ||
			(CMatrixDouble(CMatrixFixed<double, 3, 2>(vals2)) - M2b).sum_abs() >
				1e-4)
			GTEST_FAIL() << mat2;
	}

	if (!M3.fromMatlabStringFormat(mat3)) GTEST_FAIL() << mat3;

	{
		CVectorDouble m;
		if (!m.fromMatlabStringFormat(mat3) || m.size() != 1)
			GTEST_FAIL() << "CVectorDouble:" << mat3;
	}
	{
		CVectorDouble m;
		if (!m.fromMatlabStringFormat(mat3))
			GTEST_FAIL() << "CVectorFixedDouble<1>:" << mat3;
	}

	{
		CVectorDouble m;
		if (!m.fromMatlabStringFormat(mat31) || m.size() != 3)
			GTEST_FAIL() << "CVectorDouble:" << mat31;
	}
	{
		CVectorFixedDouble<3> m;
		if (!m.fromMatlabStringFormat(mat31))
			GTEST_FAIL() << "CVectorFixedDouble<3>:" << mat31;
	}

	{
		CMatrixDouble m;
		if (!m.fromMatlabStringFormat(mat13))
			GTEST_FAIL() << "Matrix<double,1,3>:" << mat13;
	}
	{
		CVectorDouble m;
		bool ok = m.fromMatlabStringFormat(mat31);
		const auto len = m.size();
		EXPECT_EQ(m.rows(), 3) << "Matrix<double,1,Dynamic>:" << mat31;
		EXPECT_TRUE(ok) << "Matrix<double,1,Dynamic>:" << mat31;
		EXPECT_EQ(m.rows(), 3) << "Matrix<double,1,Dynamic>:" << mat31;
		EXPECT_EQ(m.cols(), 1) << "Matrix<double,1,Dynamic>:" << mat31;
		EXPECT_EQ(len, 3) << "Matrix<double,1,Dynamic>:" << mat31;
	}

	// This one MUST BE detected as WRONG:
	if (M4.fromMatlabStringFormat(mat4)) GTEST_FAIL() << mat4;

	if (!M5.fromMatlabStringFormat(mat5) || M5.rows() != 0 || M5.cols() != 0)
		GTEST_FAIL() << mat5;

	if (!M6.fromMatlabStringFormat(mat6)) GTEST_FAIL() << mat6;

	// Check correct values loaded:
	CMatrixDouble RES = CMatrixDouble(M1 * M2);

	EXPECT_NEAR(0, (M6 - RES).array().square().sum(), 1e-3);
}
