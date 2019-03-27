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
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/matrix_serialization.h>  // serialization of matrices
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
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
	EXPECT_NEAR(0, fabs(err.sum()), 1e-5)
		<< "A:   " << A << "B:   " << B << "A*B: " << C << endl;
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

TEST(Matrices, SerializeCMatrixD)
{
	CMatrixDouble A(3, 2, dat_A);
	CMatrixFixed<double, 3, 2> fA;

	CMatrixD As = CMatrixD(A);

	mrpt::io::CMemoryStream membuf;
	auto arch = mrpt::serialization::archiveFrom(membuf);
	arch << As;
	membuf.Seek(0);
	arch >> fA;

	EXPECT_NEAR(0, fabs((CMatrixDouble(fA) - A).sum()), 1e-9);

	try
	{
		// Now, if we try to de-serialize into the wrong type, we should get an
		// exception:
		membuf.Seek(0);
		CMatrixFixed<double, 2, 2> fB;
		arch >> fB;  // Wrong size!

		GTEST_FAIL() << "Exception not launched when it was expected!";
	}
	catch (...)
	{  // OK, exception occurred, as expected
	}
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

	CMatrixDouble C1_RR =
		CMatrixDouble(C1_V.asEigen() * C1_D.asEigen() * C1_V.transpose());
	EXPECT_NEAR((C1_RR - C1).sum_abs(), 0, 1e-4);
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

	CMatrixDouble C1_RR =
		CMatrixDouble(C1_V.asEigen() * C1_D.asEigen() * C1_V.transpose());
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

	CMatrixDouble22 C1_RR =
		CMatrixDouble22(C1_V.asEigen() * C1_D.asEigen() * C1_V.transpose());
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

	CMatrixDouble33 C1_RR =
		CMatrixDouble33(C1_V.asEigen() * C1_D.asEigen() * C1_V.transpose());
	EXPECT_NEAR((C1_RR - C1).sum_abs(), 0, 1e-4);
}
