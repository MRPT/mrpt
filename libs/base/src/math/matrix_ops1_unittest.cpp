/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

// Note: Matrices unit tests have been split in different files since
// building them with eigen3 eats a lot of RAM and may be a problem while
// compiling in small systems.

#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/math/CMatrixD.h>
#include <mrpt/math/matrix_serialization.h>  // serialization of matrices
#include <mrpt/math/ops_matrices.h>
#include <mrpt/utils/metaprogramming.h>
#include <mrpt/utils/CMemoryStream.h>
#include <mrpt/random.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace mrpt::utils::metaprogramming;
using namespace std;


const double   dat_A[] = { 4, 5, 8, -2, 1, 3 };
const double   dat_B[] = { 2, 6, 9, 8 };
const double   dat_Cok[] = {53,64, -2,32, 29,30  };


#define CHECK_AND_RET_ERROR(_COND_,_MSG_)    EXPECT_FALSE(_COND_) << _MSG_;

TEST(Matrices, A_times_B_dyn)
{
	// Dyn. size, double.
	CMatrixDouble	A(3,2, dat_A);
	CMatrixDouble	B(2,2, dat_B);
	CMatrixDouble C = A*B;
	CMatrixDouble	C_ok(3,2, dat_Cok);
	CMatrixDouble	err = C - C_ok;
	EXPECT_NEAR(0,fabs(err.sum()), 1e-5) <<
		"A:   " << A <<
		"B:   " << B <<
		"A*B: " << C << endl;
}

TEST(Matrices,A_times_B_fix)
{
	// Fix. size, double.
	CMatrixFixedNumeric<double,3,2>	A(dat_A);
	CMatrixFixedNumeric<double,2,2>	B(dat_B);
	CMatrixFixedNumeric<double,3,2> C, C_ok(dat_Cok), Err;

	C = A * B;
	Err = C - CMatrixFixedNumeric<double,3,2>(C_ok);

	EXPECT_NEAR(0,fabs(Err.sum()), 1e-5) <<
		"A:   " << A <<
		"B:   " << B <<
		"A*B: " << C << endl;
}

TEST(Matrices,SerializeCMatrixD)
{
	CMatrixDouble	A(3,2, dat_A);
	CMatrixFixedNumeric<double,3,2>	fA;

	CMatrixD	As = A;

	CMemoryStream	membuf;
	membuf << As;
	membuf.Seek(0);
	membuf >> fA;

	EXPECT_NEAR(0,fabs((CMatrixDouble(fA) - A).sum()), 1e-9);

	try
	{
		// Now, if we try to de-serialize into the wrong type, we should get an exception:
		membuf.Seek(0);
		CMatrixFixedNumeric<double,2,2>	fB;
		membuf >> fB;  // Wrong size!

		GTEST_FAIL() << "Exception not launched when it was expected!";
	}
	catch(...)
	{ // OK, exception occurred, as expected
	}
}


TEST(Matrices,EigenVal2x2dyn)
{
	const double   dat_C1[] = {  14.6271,  5.8133, 5.8133, 16.8805 };
	CMatrixDouble  C1(2,2, dat_C1);

	Eigen::MatrixXd C1_V,C1_D;
	C1.eigenVectors(C1_V,C1_D);

	CMatrixDouble  C1_RR = C1_V * C1_D * C1_V.transpose();
	EXPECT_NEAR( (C1_RR-C1).array().abs().sum(),0,1e-4);
}

TEST(Matrices,EigenVal3x3dyn)
{
	const double   dat_C1[] = {  8,6,1, 6,9,4, 1,4,10 };
	CMatrixDouble  C1(3,3, dat_C1);

	Eigen::MatrixXd C1_V,C1_D;
	C1.eigenVectors(C1_V,C1_D);

	CMatrixDouble  C1_RR = C1_V*C1_D*C1_V.transpose();
	EXPECT_NEAR( (C1_RR-C1).array().abs().sum(),0,1e-4);
}

TEST(Matrices,EigenVal2x2fix)
{
	const double   dat_C1[] = {  14.6271,  5.8133, 5.8133, 16.8805 };
	CMatrixDouble22  C1(dat_C1);

	Eigen::Matrix2d C1_V,C1_D;
	C1.eigenVectors(C1_V,C1_D);

	CMatrixDouble22  C1_RR = C1_V*C1_D*(~C1_V);
	EXPECT_NEAR( (C1_RR-C1).array().abs().sum(),0,1e-4);
}

TEST(Matrices,EigenVal3x3fix)
{
	const double   dat_C1[] = {  8,6,1, 6,9,4, 1,4,10 };
	CMatrixDouble33  C1(dat_C1);

	CMatrixDouble33  C1_V, C1_D;
	C1.eigenVectors(C1_V,C1_D);

	CMatrixDouble33  C1_RR = C1_V*C1_D*(~C1_V);
	EXPECT_NEAR( (C1_RR-C1).array().abs().sum(),0,1e-4);
}

#if 0 // JL: Disabled since it fails in some PPA build servers. Reported to Eigen list for possible fixes...

// Compare the two ways of computing matrix eigenvectors: generic & for symmetric matrices:
TEST(Matrices,EigenVal4x4_sym_vs_generic)
{
	const double   dat_C1[] = {
		13.737245,10.248641,-5.839599,11.108320,
		10.248641,14.966139,-5.259922,11.662222,
		-5.839599,-5.259922,9.608822,-4.342505,
		11.108320,11.662222,-4.342505,12.121940 };
	const CMatrixDouble44  C1(dat_C1);

	CMatrixDouble44 eigvecs_sym, eigvecs_gen, eigvals_symM, eigvals_genM;
	CVectorDouble   eigvals_sym, eigvals_gen;

	C1.eigenVectorsVec(eigvecs_gen,eigvals_gen);
	C1.eigenVectorsSymmetricVec(eigvecs_sym,eigvals_sym);

	eigvals_symM.setZero();eigvals_symM.diagonal() = eigvals_sym;
	eigvals_genM.setZero();eigvals_genM.diagonal() = eigvals_gen;

	EXPECT_NEAR( (C1-eigvecs_gen*eigvals_genM*(~eigvecs_gen)).array().abs().sum(),0,1e-5)
		<< endl << endl
		<< "eigvecs_gen*eigvals_gen*(~eigvecs_gen):\n" << eigvecs_gen*eigvals_genM*(~eigvecs_gen) << endl
		<< "C1:\n" << C1 << endl
		<< "eigvals_sym:\n" <<  eigvals_sym << endl
		<< "eigvals_gen:\n" << eigvals_gen << endl
		<< "eigvals_symM:\n" <<  eigvals_symM << endl
		<< "eigvals_genM:\n" << eigvals_genM << endl
		<< "eigvecs_gen:\n" << eigvecs_gen << endl
		<< "eigvecs_sym:\n" << eigvecs_sym << endl<< endl;

	EXPECT_NEAR( (C1-eigvecs_sym*eigvals_symM*(~eigvecs_sym)).array().abs().sum(),0,1e-5)
		<< endl << endl
		<< "eigvecs_sym*eigvals_sym*(~eigvecs_sym):\n" << eigvecs_sym*eigvals_symM*(~eigvecs_sym) << endl
		<< "C1:\n" << C1 << endl;

	EXPECT_NEAR( (eigvals_gen-eigvals_sym).array().abs().sum(),0,1e-5)
		<< endl << endl
		<< "eigvals_gen:\n" << eigvals_gen<< endl
		<< "eigvals_sym:\n" << eigvals_sym << endl;
}
#endif

