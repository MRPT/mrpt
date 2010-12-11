/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */


#include <mrpt/base.h>
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


TEST(Matrices, setSize)
{
	{
		CMatrixFixedNumeric<double,6,6>  M;
		EXPECT_TRUE( (M.array() == 0).all() );
	}
	{
		CMatrixDouble  M(5,5);
		EXPECT_TRUE( (M.array() == 0).all() );
	}
	{
		CMatrixDouble  M(5,5);
		M.setSize(6,5);
		EXPECT_TRUE( (M.array() == 0).all() );
	}
	{
		CMatrixDouble  M(5,5);
		M.setSize(10,5);
		EXPECT_TRUE( (M.array() == 0).all() );
	}
	{
		CMatrixDouble  M(5,5);
		M.setSize(5,6);
		EXPECT_TRUE( (M.array() == 0).all() );
	}
	{
		CMatrixDouble  M(5,5);
		M.setSize(6,6);
		EXPECT_TRUE( (M.array() == 0).all() );
	}
	{
		CMatrixDouble  M(5,5);
		M.setSize(10,10);
		EXPECT_TRUE( (M.array() == 0).all() );
	}
}

TEST(Matrices, extractSubmatrixSymmetricalBlocks)
{
	{
		const double vals[] = {
			1,2,3,4,5,6,7,8,
			8,9,10,11,12,13,14,15,
			1,2,3,4,5,6,7,8,
			8,9,10,11,12,13,14,15,
			1,2,3,4,5,6,7,8,
			8,9,10,11,12,13,14,15,
			1,2,3,4,5,6,7,8,
			8,9,10,11,12,13,14,15 };
		const CMatrixFixedNumeric<double,8,8>  M(vals);

		std::vector<size_t> vs;
		vs.push_back(1);
		vs.push_back(3);

		CMatrixDouble E;
		M.extractSubmatrixSymmetricalBlocks(2,vs,E);

		const double valsE[] = {
			3,4,7,8,
			10,11,14,15,
			3,4,7,8,
			10,11,14,15 };
		const CMatrixDouble44  E_expected(valsE);

		EXPECT_TRUE( E_expected == E );
	}
}

TEST(Matrices, extractSubmatrixSymmetrical)
{
	{
		const double vals[] = {
			1,2,3,4,5,6,7,8,
			8,9,10,11,12,13,14,15,
			1,2,3,4,5,6,7,8,
			8,9,10,11,12,13,14,15,
			1,2,3,4,5,6,7,8,
			8,9,10,11,12,13,14,15,
			1,2,3,4,5,6,7,8,
			8,9,10,11,12,13,14,15 };
		const CMatrixFixedNumeric<double,8,8>  M(vals);

		std::vector<size_t> vs;
		vs.push_back(2);
		vs.push_back(3);
		vs.push_back(6);
		vs.push_back(7);

		CMatrixDouble E;
		M.extractSubmatrixSymmetrical(vs,E);

		const double valsE[] = {
			3,4,7,8,
			10,11,14,15,
			3,4,7,8,
			10,11,14,15 };
		const CMatrixDouble44  E_expected(valsE);

		EXPECT_TRUE( E_expected == E );
	}
}


TEST(Matrices, A_times_B_dyn)
{
	// Dyn. size, double.
	CMatrixDouble	A(3,2, dat_A);
	CMatrixDouble	B(2,2, dat_B);
	CMatrixDouble C = A*B;
	CMatrixDouble	C_ok(3,2, dat_Cok);
	CMatrixDouble	err = C - C_ok;
	EXPECT_NEAR(0,fabs(err.sumAll()), 1e-5) <<
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

	EXPECT_NEAR(0,fabs(Err.sumAll()), 1e-5) <<
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

	EXPECT_NEAR(0,fabs((CMatrixDouble(fA) - A).sumAll()), 1e-9);

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
	EXPECT_NEAR( (C1_RR-C1).Abs().sum(),0,1e-4);
}

TEST(Matrices,EigenVal3x3dyn)
{
	const double   dat_C1[] = {  8,6,1, 6,9,4, 1,4,10 };
	CMatrixDouble  C1(3,3, dat_C1);

	Eigen::MatrixXd C1_V,C1_D;
	C1.eigenVectors(C1_V,C1_D);

	CMatrixDouble  C1_RR = C1_V*C1_D*C1_V.transpose();
	EXPECT_NEAR( (C1_RR-C1).Abs().sum(),0,1e-4);
}

TEST(Matrices,EigenVal2x2fix)
{
	const double   dat_C1[] = {  14.6271,  5.8133, 5.8133, 16.8805 };
	CMatrixDouble22  C1(dat_C1);

	Eigen::Matrix2d C1_V,C1_D;
	C1.eigenVectors(C1_V,C1_D);

	CMatrixDouble22  C1_RR = C1_V*C1_D*(~C1_V);
	EXPECT_NEAR( (C1_RR-C1).Abs().sum(),0,1e-4);
}

TEST(Matrices,EigenVal3x3fix)
{
	const double   dat_C1[] = {  8,6,1, 6,9,4, 1,4,10 };
	CMatrixDouble33  C1(dat_C1);

	CMatrixDouble33  C1_V, C1_D;
	C1.eigenVectors(C1_V,C1_D);

	CMatrixDouble33  C1_RR = C1_V*C1_D*(~C1_V);
	EXPECT_NEAR( (C1_RR-C1).Abs().sum(),0,1e-4);
}

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
	vector_double   eigvals_sym, eigvals_gen;

	C1.eigenVectorsVec(eigvecs_gen,eigvals_gen);
	C1.eigenVectorsSymmetricVec(eigvecs_sym,eigvals_sym);

	eigvals_symM.setZero();eigvals_symM.diagonal() = eigvals_sym;
	eigvals_genM.setZero();eigvals_genM.diagonal() = eigvals_gen;

	EXPECT_NEAR( (C1-eigvecs_gen*eigvals_genM*(~eigvecs_gen)).Abs().sum(),0,1e-5)
		<< endl << endl
		<< "eigvecs_gen*eigvals_gen*(~eigvecs_gen):\n" << eigvecs_gen*eigvals_genM*(~eigvecs_gen) << endl
		<< "C1:\n" << C1 << endl
		<< "eigvals_sym:\n" <<  eigvals_sym << endl
		<< "eigvals_gen:\n" << eigvals_gen << endl
		<< "eigvals_symM:\n" <<  eigvals_symM << endl
		<< "eigvals_genM:\n" << eigvals_genM << endl
		<< "eigvecs_gen:\n" << eigvecs_gen << endl
		<< "eigvecs_sym:\n" << eigvecs_sym << endl<< endl;

	EXPECT_NEAR( (C1-eigvecs_sym*eigvals_symM*(~eigvecs_sym)).Abs().sum(),0,1e-5)
		<< endl << endl
		<< "eigvecs_sym*eigvals_sym*(~eigvecs_sym):\n" << eigvecs_sym*eigvals_symM*(~eigvecs_sym) << endl
		<< "C1:\n" << C1 << endl;

	EXPECT_NEAR( (eigvals_gen-eigvals_sym).Abs().sum(),0,1e-5)
		<< endl << endl
		<< "eigvals_gen:\n" << eigvals_gen<< endl
		<< "eigvals_sym:\n" << eigvals_sym << endl;
}

TEST(Matrices,HCHt_3x2_2x2_2x3)
{
	const double   dat_H[] = {  0.2, -0.3,   0.1, 0.9,  -0.07, 1.2  };
	CMatrixDouble  H(3,2, dat_H);

	const double   dat_C[] = {  0.8, -0.1, -0.1, 0.8 };
	CMatrixDouble  C(2,2, dat_C);

	const double   dat_R[] = {  0.11600 , -0.21500,  -0.32530, -0.21500 ,  0.63800  , 0.85270, -0.32530 ,  0.85270 ,  1.17272 };
	CMatrixDouble  R_REAL(3,3, dat_R);

	CMatrixDouble R;
	H.multiply_HCHt(C,R);

	EXPECT_NEAR( (R_REAL-R).Abs().sum(),0,1e-4);
}

TEST(Matrices,HCHt_scalar_1x2_2x2_2x1)
{
	const double   dat_H[] = {  0.2, -0.3 };
	CMatrixDouble H(1,2, dat_H);

	const double   dat_C[] = {  0.8, -0.1, -0.1, 0.8 };
	CMatrixDouble C(2,2, dat_C);

	const double r = H.multiply_HCHt_scalar(C);

	const double r2 = (H*C*H.transpose()).eval()(0,0);
	CHECK_AND_RET_ERROR(fabs(r-r2)>1e-4,  "Error in HCHt_scalar: 1x2 * 2x2 * 2x1")
}

TEST(Matrices,multiply_Ab_2x2_2x1)
{
	const double   dat_A[] = {  0.8, -0.3, -0.7, 0.1 };
	CMatrixDouble  A(2,2, dat_A);

	const vector_double b = make_vector<2,double>(2.0, 3.0);

	const vector_double real_res = make_vector<2,double>(0.7, -1.1);

	vector_double res;
	A.multiply_Ab(b,res);

	CHECK_AND_RET_ERROR((res-real_res).Abs().sumAll() >1e-4,  "Error in multiply_Ab: 2x2 * 2x1")
}

TEST(Matrices,det_2x2_dyn)
{
	const double   dat_A[] = {  0.8, -0.3, -0.7, 0.1 };
	CMatrixDouble  A(2,2, dat_A);

	const double d = A.det();
	const double d_R =-0.13;

	CHECK_AND_RET_ERROR(fabs(d-d_R) >1e-4,  "Error in Determinant: 2x2 dyn")
}

TEST(Matrices,det_2x2_fix)
{
	const double   dat_A[] = {  0.8, -0.3, -0.7, 0.1 };
	CMatrixDouble22  A(dat_A);

	const double d = A.det();
	const double d_R =-0.13;

	CHECK_AND_RET_ERROR(fabs(d-d_R) >1e-4,  "Error in Determinant: 2x2 fix")
}

TEST(Matrices,det_3x3_dyn)
{
	const double   dat_A[] = {  -3.3304e-01, -2.0585e-01, 6.2026e-05,  1.4631e+00,6.0985e-01,2.3746e+00,   -3.6451e-01, 4.8169e-01, -8.4419e-01};
	CMatrixDouble  A(3,3, dat_A);
	const double d = A.det();
	const double d_R =0.476380435871666;
	//cout << "d: " << d << endl;
	CHECK_AND_RET_ERROR(fabs(d-d_R)>1e-4,  "Error in Determinant: 3x3 dyn")
}

TEST(Matrices,det_3x3_fix)
{
	const double   dat_A[] = {  -3.3304e-01, -2.0585e-01, 6.2026e-05,  1.4631e+00,6.0985e-01,2.3746e+00,   -3.6451e-01, 4.8169e-01, -8.4419e-01};
	CMatrixDouble33  A(dat_A);
	const double d = A.det();
	const double d_R =0.476380435871666;
	CHECK_AND_RET_ERROR(fabs(d-d_R) >1e-4,  "Error in Determinant: 3x3 fix")
}

TEST(Matrices,det_4x4_dyn)
{
	const double   dat_A[] = {0.773931,-0.336130,1.131764,0.385890,1.374906,-0.540629,-0.952902,0.659769,-0.387254,-1.557355,0.139683,-2.056635,-0.750078,-0.653811,0.872027,0.217554};
	CMatrixDouble  A(4,4,dat_A);
	const double d = A.det();
	const double d_R = -6.29527837425056;
	CHECK_AND_RET_ERROR(fabs(d-d_R) >1e-4,  "Error in Determinant: 4x4 dyn")
}

TEST(Matrices,det_4x4_fix)
{
	const double   dat_A[] = {0.773931,-0.336130,1.131764,0.385890,1.374906,-0.540629,-0.952902,0.659769,-0.387254,-1.557355,0.139683,-2.056635,-0.750078,-0.653811,0.872027,0.217554};
	CMatrixDouble44  A(dat_A);
	const double d = A.det();
	const double d_R = -6.29527837425056;
	CHECK_AND_RET_ERROR(fabs(d-d_R) >1e-4,  "Error in Determinant: 4x4 fix")
}

TEST(Matrices,det_10x10_dyn)
{
	const double   dat_A[] = {1.2305462976,-0.2944257811,0.8176140437,-0.0487601371,0.4418235581,-0.0088466980,-1.4100223408,-0.6219629815,1.1089237266,-0.6450262619,-2.0862614547,0.2699762709,-0.0705918517,1.1763963161,-0.3461819597,-1.3013222580,-0.3310621595,-0.2595069675,-0.5188213591,1.2261476224,-1.1334297957,2.1452881319,1.7856021357,0.5406722888,0.5497545623,0.4282217402,-1.6175210256,-0.3522824764,0.2773929603,0.8507134453,0.4046854117,-2.1638696195,1.0044939778,0.9755939720,0.9640788301,0.5641138097,0.7382236207,-0.4422212587,0.8507041571,1.3764399072,0.3446492224,1.1681336612,-1.3440052449,1.0120691406,-0.0430604384,0.4823901171,0.0881769800,0.3984805283,-1.9988153178,0.9509748328,0.3202853059,1.9688559025,0.4020581289,-1.5558616735,-0.8753527614,0.1207830427,0.0457715031,-0.1557123759,-0.3161307172,-0.0759276933,-0.0417386037,1.2079564736,-2.5839030155,-0.7648863647,1.1541464803,0.2127569446,-1.4882083860,-0.7630836781,0.8550884427,-0.8440402465,-0.4903597050,-0.1457982930,0.5893448560,-0.2353784687,0.3474655757,2.5874616045,0.6608448038,-1.0105315509,-1.5276853710,-0.1400026815,-1.7630264416,2.4048579514,-0.3111046623,0.7463774799,-0.2800404492,-1.4175124130,-0.5708536580,-1.2085107661,0.8169107561,-1.1659481510,-0.1406355512,2.3507381980,2.6346742737,-1.1028788167,-0.0533115044,0.3752684649,-1.3799576309,-0.7274190037,1.1188847602,-0.6624231096};
	CMatrixDouble  A(10,10,dat_A);
	const double d = A.det();
	const double d_R = 330.498518199239;
	CHECK_AND_RET_ERROR(fabs(d-d_R) >1e-4,  "Error in Determinant: 10x10 dyn")
}

TEST(Matrices,det_10x10_fix)
{
	const double   dat_A[] = {1.2305462976,-0.2944257811,0.8176140437,-0.0487601371,0.4418235581,-0.0088466980,-1.4100223408,-0.6219629815,1.1089237266,-0.6450262619,-2.0862614547,0.2699762709,-0.0705918517,1.1763963161,-0.3461819597,-1.3013222580,-0.3310621595,-0.2595069675,-0.5188213591,1.2261476224,-1.1334297957,2.1452881319,1.7856021357,0.5406722888,0.5497545623,0.4282217402,-1.6175210256,-0.3522824764,0.2773929603,0.8507134453,0.4046854117,-2.1638696195,1.0044939778,0.9755939720,0.9640788301,0.5641138097,0.7382236207,-0.4422212587,0.8507041571,1.3764399072,0.3446492224,1.1681336612,-1.3440052449,1.0120691406,-0.0430604384,0.4823901171,0.0881769800,0.3984805283,-1.9988153178,0.9509748328,0.3202853059,1.9688559025,0.4020581289,-1.5558616735,-0.8753527614,0.1207830427,0.0457715031,-0.1557123759,-0.3161307172,-0.0759276933,-0.0417386037,1.2079564736,-2.5839030155,-0.7648863647,1.1541464803,0.2127569446,-1.4882083860,-0.7630836781,0.8550884427,-0.8440402465,-0.4903597050,-0.1457982930,0.5893448560,-0.2353784687,0.3474655757,2.5874616045,0.6608448038,-1.0105315509,-1.5276853710,-0.1400026815,-1.7630264416,2.4048579514,-0.3111046623,0.7463774799,-0.2800404492,-1.4175124130,-0.5708536580,-1.2085107661,0.8169107561,-1.1659481510,-0.1406355512,2.3507381980,2.6346742737,-1.1028788167,-0.0533115044,0.3752684649,-1.3799576309,-0.7274190037,1.1188847602,-0.6624231096};
	CMatrixFixedNumeric<double,10,10>  A(dat_A);
	const double d = A.det();
	const double d_R = 330.498518199239;
	CHECK_AND_RET_ERROR(fabs(d-d_R) >1e-4,  "Error in Determinant: 10x10 fix")
}

TEST(Matrices,chol_2x2_dyn)
{
	const double   dat_A[] = {1.0727710178,0.6393375593,0.6393375593,0.8262219720};
	CMatrixDouble  A(2,2, dat_A);
	CMatrixDouble C;
	A.chol(C);
	const double   dat_CHOL[] = {1.0357465992,0.6172721781,0.0000000000,0.6672308672};
	CMatrixDouble CHOL(2,2,dat_CHOL);
	CHECK_AND_RET_ERROR( (CHOL-C).Abs().sumAll() >1e-4,  "Error in Choleski, 2x2 dyn")
}

TEST(Matrices,chol_2x2_fix)
{
	const double   dat_A[] = {1.0727710178,0.6393375593,0.6393375593,0.8262219720};
	CMatrixDouble22  A(dat_A);
	CMatrixDouble22  C;
	A.chol(C);
	const double   dat_CHOL[] = {1.0357465992,0.6172721781,0.0000000000,0.6672308672};
	CMatrixDouble CHOL(2,2,dat_CHOL);
	CHECK_AND_RET_ERROR( (CHOL-C).Abs().sumAll() >1e-4,  "Error in Choleski, 2x2 fix")
}


TEST(Matrices,chol_3x3_dyn)
{
	const double   dat_A[] = {0.515479426556448,0.832723636299236,0.249691538245735,0.832723636299236,1.401081397506934,0.385539356127255,0.249691538245735,0.385539356127255,0.128633962591437};
	CMatrixDouble  A(3,3, dat_A);
	CMatrixDouble C;
	A.chol(C);

	const double   dat_CHOL[] = {0.717968959326549,1.159832365288224,0.347774837619643,0.000000000000000,0.236368952988455,-0.075395504153773,0.000000000000000,0.000000000000000,0.044745311077990};
	CMatrixDouble CHOL(3,3,dat_CHOL);
	CHECK_AND_RET_ERROR( (CHOL-C).Abs().sumAll() >1e-4,  "Error in Choleski, 3x3 dyn")
}

TEST(Matrices,chol_3x3_fix)
{
	const double   dat_A[] = {0.515479426556448,0.832723636299236,0.249691538245735,0.832723636299236,1.401081397506934,0.385539356127255,0.249691538245735,0.385539356127255,0.128633962591437};
	CMatrixDouble33  A(dat_A);
	CMatrixDouble33 C;
	A.chol(C);

	const double   dat_CHOL[] = {0.717968959326549,1.159832365288224,0.347774837619643,0.000000000000000,0.236368952988455,-0.075395504153773,0.000000000000000,0.000000000000000,0.044745311077990};
	CMatrixDouble33 CHOL(dat_CHOL);
	CHECK_AND_RET_ERROR( (CHOL-C).Abs().sumAll() >1e-4,  "Error in Choleski, 3x3 fix")
}


TEST(Matrices,chol_10x10_dyn)
{
	const double   dat_A[] = {2.8955668335,2.3041932983,1.9002381085,1.7993158652,1.8456197228,2.9632296740,1.9368565578,2.1988923358,2.0547605617,2.5655678993,2.3041932983,3.8406914364,2.1811218706,3.2312564555,2.4736403918,3.4703311380,1.4874417483,3.1073538218,2.1353324397,2.9541115932,1.9002381085,2.1811218706,2.4942067597,1.6851007198,1.4585872052,2.3015952197,1.0955231591,2.2979627790,1.3918738834,2.1854562572,1.7993158652,3.2312564555,1.6851007198,3.1226161015,1.6779632687,2.7195826381,1.2397348013,2.3757864319,1.6291224768,2.4463194915,1.8456197228,2.4736403918,1.4585872052,1.6779632687,2.8123267839,2.5860688816,1.4131630919,2.1914803135,1.5542420639,2.7170092067,2.9632296740,3.4703311380,2.3015952197,2.7195826381,2.5860688816,4.1669180394,2.1145239023,3.3214801332,2.6694845663,3.0742063088,1.9368565578,1.4874417483,1.0955231591,1.2397348013,1.4131630919,2.1145239023,1.8928811570,1.7097998455,1.7205860530,1.8710847505,2.1988923358,3.1073538218,2.2979627790,2.3757864319,2.1914803135,3.3214801332,1.7097998455,3.4592638415,2.1518695071,2.8907499694,2.0547605617,2.1353324397,1.3918738834,1.6291224768,1.5542420639,2.6694845663,1.7205860530,2.1518695071,2.1110960664,1.6731209980,2.5655678993,2.9541115932,2.1854562572,2.4463194915,2.7170092067,3.0742063088,1.8710847505,2.8907499694,1.6731209980,3.9093678727};
	CMatrixDouble  A(10,10, dat_A);
	CMatrixDouble C;
	A.chol(C);

	const double   dat_CHOL[] = {1.7016365163,1.3541042851,1.1167121124,1.0574031810,1.0846145491,1.7413999087,1.1382316607,1.2922221137,1.2075202560,1.5077061845,0.0000000000,1.4167191047,0.4722017314,1.2701334167,0.7093566960,0.7851196867,-0.0380051491,0.9582353452,0.3530862859,0.6441080558,0.0000000000,0.0000000000,1.0120209201,-0.0943393725,-0.0865342379,-0.0136183214,-0.1557357390,0.3976620401,-0.1218419159,0.1952860421,0.0000000000,0.0000000000,0.0000000000,0.6183654266,-0.6113744707,-0.1944977093,0.1127886805,-0.2752173394,-0.1741275611,0.0847171764,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.8668818973,0.0234194680,0.3011475111,-0.0272963639,-0.1417917925,0.8000162775,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.6924364129,0.2527445784,0.3919505633,0.3715689962,-0.0817608778,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.6358623279,0.4364121485,0.4859857603,-0.0313828244,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.5408375843,-0.1995475524,0.6258606925,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.2213262214,-0.2367037013,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.2838575216};
	CMatrixDouble CHOL(10,10,dat_CHOL);
	CHECK_AND_RET_ERROR( (CHOL-C).Abs().sumAll() >1e-4,  "Error in Choleski, 10x10 dyn")
}

TEST(Matrices,chol_10x10_fix)
{
	const double   dat_A[] = {2.8955668335,2.3041932983,1.9002381085,1.7993158652,1.8456197228,2.9632296740,1.9368565578,2.1988923358,2.0547605617,2.5655678993,2.3041932983,3.8406914364,2.1811218706,3.2312564555,2.4736403918,3.4703311380,1.4874417483,3.1073538218,2.1353324397,2.9541115932,1.9002381085,2.1811218706,2.4942067597,1.6851007198,1.4585872052,2.3015952197,1.0955231591,2.2979627790,1.3918738834,2.1854562572,1.7993158652,3.2312564555,1.6851007198,3.1226161015,1.6779632687,2.7195826381,1.2397348013,2.3757864319,1.6291224768,2.4463194915,1.8456197228,2.4736403918,1.4585872052,1.6779632687,2.8123267839,2.5860688816,1.4131630919,2.1914803135,1.5542420639,2.7170092067,2.9632296740,3.4703311380,2.3015952197,2.7195826381,2.5860688816,4.1669180394,2.1145239023,3.3214801332,2.6694845663,3.0742063088,1.9368565578,1.4874417483,1.0955231591,1.2397348013,1.4131630919,2.1145239023,1.8928811570,1.7097998455,1.7205860530,1.8710847505,2.1988923358,3.1073538218,2.2979627790,2.3757864319,2.1914803135,3.3214801332,1.7097998455,3.4592638415,2.1518695071,2.8907499694,2.0547605617,2.1353324397,1.3918738834,1.6291224768,1.5542420639,2.6694845663,1.7205860530,2.1518695071,2.1110960664,1.6731209980,2.5655678993,2.9541115932,2.1854562572,2.4463194915,2.7170092067,3.0742063088,1.8710847505,2.8907499694,1.6731209980,3.9093678727};
	CMatrixFixedNumeric<double,10,10> A(dat_A);
	CMatrixDouble C;
	A.chol(C);

	const double   dat_CHOL[] = {1.7016365163,1.3541042851,1.1167121124,1.0574031810,1.0846145491,1.7413999087,1.1382316607,1.2922221137,1.2075202560,1.5077061845,0.0000000000,1.4167191047,0.4722017314,1.2701334167,0.7093566960,0.7851196867,-0.0380051491,0.9582353452,0.3530862859,0.6441080558,0.0000000000,0.0000000000,1.0120209201,-0.0943393725,-0.0865342379,-0.0136183214,-0.1557357390,0.3976620401,-0.1218419159,0.1952860421,0.0000000000,0.0000000000,0.0000000000,0.6183654266,-0.6113744707,-0.1944977093,0.1127886805,-0.2752173394,-0.1741275611,0.0847171764,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.8668818973,0.0234194680,0.3011475111,-0.0272963639,-0.1417917925,0.8000162775,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.6924364129,0.2527445784,0.3919505633,0.3715689962,-0.0817608778,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.6358623279,0.4364121485,0.4859857603,-0.0313828244,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.5408375843,-0.1995475524,0.6258606925,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.2213262214,-0.2367037013,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.2838575216};
	CMatrixDouble CHOL(10,10,dat_CHOL);
	CHECK_AND_RET_ERROR( (CHOL-C).Abs().sumAll() >1e-4,  "Error in Choleski, 10x10 fix")
}

TEST(Matrices,inv_4x4_fix)
{
	const double   dat_A[] = {-0.710681653571291,0.734469323344333,-0.656414638791893,0.818771495864303,1.044946492154568,1.163592359608108,-1.069421407670914,0.307916381104872,0.185595851677470,0.116899590868673,0.507691343481809,-3.217842384231890,-0.214383515646621,-0.161495561253269,1.303923696836841,0.261535721431038};
	CMatrixDouble44  A(dat_A);
	CMatrixDouble44  C = A.inv();
	const double   dat_AInv[] = {-0.741952742824035,0.493481687552705,-0.134764164880760,0.083693424291000,0.638324207063440,0.519344439204238,0.264483337145361,0.644307267615193,-0.037800456163779,0.131794126194075,0.070338431705792,0.828591793299072,-0.025568212209135,0.068123300450057,-0.297834184749986,0.158964059763645};
	CMatrixDouble44 AInv(dat_AInv);
	CHECK_AND_RET_ERROR( (AInv-C).Abs().sumAll() >1e-4,  "Error in inv, 4x4 fix")
}

TEST(Matrices,inv_6x6_fix)
{
	const double   dat_A[] = {363.769989013671875,0.000000000000000,316.429992675781250,0.000000000000000,87.266998291015625,0.000000000000000,101.540000915527344,0.000000000000000,478.709991455078125,0.000000000000000,504.540008544921875,0.000000000000000,1.000000000000000,0.000000000000000,1.000000000000000,0.000000000000000,1.000000000000000,0.000000000000000,0.000000000000000,363.769989013671875,0.000000000000000,316.429992675781250,0.000000000000000,87.266998291015625,0.000000000000000,101.540000915527344,0.000000000000000,478.709991455078125,0.000000000000000,504.540008544921875,0.000000000000000,1.000000000000000,0.000000000000000,1.000000000000000,0.000000000000000,1.000000000000000};
	CMatrixDouble66  A(dat_A);
	CMatrixDouble66  C;
	A.inv(C);
	const double   dat_AInv[] = {-0.000303131460181,-0.002689371550382,1.383348917627708,0.000000000000000,0.000000000000000,0.000000000000000,0.000000000000000,0.000000000000000,0.000000000000000,-0.000303131460181,-0.002689371550382,1.383348917627708,0.004729457992255,0.003244936115630,-2.049925698035195,0.000000000000000,0.000000000000000,0.000000000000000,0.000000000000000,0.000000000000000,0.000000000000000,0.004729457992255,0.003244936115630,-2.049925698035195,-0.004426326532074,-0.000555564565248,1.666576780407488,0.000000000000000,0.000000000000000,0.000000000000000,0.000000000000000,0.000000000000000,0.000000000000000,-0.004426326532074,-0.000555564565248,1.666576780407488};
	CMatrixDouble66 AInv(dat_AInv);
	CHECK_AND_RET_ERROR( isNaN(C(0,0)) || !isFinite(C(0,0)) || (AInv-C).Abs().sumAll() >1e-4,  "Error in inv, 6x6 fix")
}

TEST(Matrices,inv_6x6_dyn)
{
	const double   dat_A[] = {363.769989013671875,0.000000000000000,316.429992675781250,0.000000000000000,87.266998291015625,0.000000000000000,101.540000915527344,0.000000000000000,478.709991455078125,0.000000000000000,504.540008544921875,0.000000000000000,1.000000000000000,0.000000000000000,1.000000000000000,0.000000000000000,1.000000000000000,0.000000000000000,0.000000000000000,363.769989013671875,0.000000000000000,316.429992675781250,0.000000000000000,87.266998291015625,0.000000000000000,101.540000915527344,0.000000000000000,478.709991455078125,0.000000000000000,504.540008544921875,0.000000000000000,1.000000000000000,0.000000000000000,1.000000000000000,0.000000000000000,1.000000000000000};
	CMatrixDouble  A(6,6,dat_A);
	CMatrixDouble  C = A.inv();
	const double   dat_AInv[] = {-0.000303131460181,-0.002689371550382,1.383348917627708,0.000000000000000,0.000000000000000,0.000000000000000,0.000000000000000,0.000000000000000,0.000000000000000,-0.000303131460181,-0.002689371550382,1.383348917627708,0.004729457992255,0.003244936115630,-2.049925698035195,0.000000000000000,0.000000000000000,0.000000000000000,0.000000000000000,0.000000000000000,0.000000000000000,0.004729457992255,0.003244936115630,-2.049925698035195,-0.004426326532074,-0.000555564565248,1.666576780407488,0.000000000000000,0.000000000000000,0.000000000000000,0.000000000000000,0.000000000000000,0.000000000000000,-0.004426326532074,-0.000555564565248,1.666576780407488};
	CMatrixDouble AInv(6,6,dat_AInv);
	CHECK_AND_RET_ERROR( isNaN(C(0,0)) || !isFinite(C(0,0)) || (AInv-C).Abs().sumAll() >1e-4,  "Error in inv, 6x6 dyn")
}

TEST(Matrices,transpose)
{
	const double dat_A[] = {
		1,2,3,
		4,5,6 };
	const double dat_At[] = {
		1,4,
		2,5,
		3,6};
	const CMatrixDouble  A(2,3,dat_A);
	const CMatrixDouble  At(3,2,dat_At);

	EXPECT_EQ(A.t(), At);
	EXPECT_EQ(~A, At);
	EXPECT_EQ(A.t().t(), A);
}

TEST(Matrices,multiply_A_skew3)
{
	{
		const double dat_A[] = {
			1,2,3,
			4,5,6 };
		const CMatrixDouble  A(2,3,dat_A);
		const vector_double  v = make_vector<3>(1.0,2.0,3.0);
		const CMatrixDouble  S = CMatrixDouble( mrpt::math::skew_symmetric3(v) );

		CMatrixDouble  R;
		R.multiply_A_skew3(A,v);
		EXPECT_EQ(R, A*S );
	}
	{
		const double dat_A[] = {
			1,2,3,
			4,5,6 };
		const double dat_v[] = { 1,2,3 };
		const CMatrixFixedNumeric<double,2,3>  A(dat_A);
		const CArrayDouble<3> v(dat_v);
		const CMatrixFixedNumeric<double,3,3>  S = mrpt::math::skew_symmetric3(v);

		CMatrixFixedNumeric<double,2,3> R;
		R.multiply_A_skew3(A,v);
		EXPECT_TRUE(R== A*S );
	}
}

TEST(Matrices,multiply_skew3_A)
{
	{
		const double dat_A[] = {
			1,2,
			3,4,
			5,6 };
		const CMatrixDouble  A(3,2,dat_A);
		const vector_double  v = make_vector<3>(1.0,2.0,3.0);
		const CMatrixDouble  S = CMatrixDouble( mrpt::math::skew_symmetric3(v) );

		CMatrixDouble  R;
		R.multiply_skew3_A(v,A);
		EXPECT_TRUE(R == S*A );
	}
	{
		const double dat_A[] = {
			1,2,
			3,4,
			5,6 };
		const double dat_v[] = { 1,2,3 };
		const CMatrixFixedNumeric<double,3,2>  A(dat_A);
		const CArrayDouble<3> v(dat_v);
		const CMatrixFixedNumeric<double,3,3>  S = mrpt::math::skew_symmetric3(v);

		CMatrixFixedNumeric<double,3,2> R;
		R.multiply_skew3_A(v,A);
		EXPECT_TRUE(R == S*A );
	}
}


TEST(Matrices,fromMatlabStringFormat)
{
	const char* mat1 = "[1 2 3;-3 -6 -5]";
	const double vals1[] = {1,2,3,-3,-6,-5};

	const char* mat2 = " [ 	  -8.2	 9.232 ; -2e+2		+6 ; 1.000  7 ] ";    // With tabs and spaces...
	const double vals2[] = {-8.2, 9.232, -2e+2, +6, 1.000 ,7};

	const char* mat3 = "[9]";
	const char* mat4 = "[1 2 3 4 5 6 7 9 10  ; 1 2 3 4 5 6 7 8 9 10 11]";   // An invalid matrix
	const char* mat5 = "[  ]";  // Empty
	const char* mat6 = "[ -405.200 42.232 ; 1219.600    -98.696 ]";  // M1 * M2

	const char* mat13 = "[9 8 7]";
	const char* mat31 = "[9; 8; 7]";

	CMatrixDouble	M1,M2,M3, M4, M5, M6;

	if (! M1.fromMatlabStringFormat(mat1) ||
		(CMatrixFixedNumeric<double,2,3>(vals1)-M1).Abs().sumAll() > 1e-4 )
		GTEST_FAIL() << mat1;

	{
		CMatrixFixedNumeric<double,2,3> M1b;
		if (! M1b.fromMatlabStringFormat(mat1) ||
			(CMatrixFixedNumeric<double,2,3>(vals1)-M1b).Abs().sumAll() > 1e-4 )
			GTEST_FAIL() << mat1;
	}

	if (! M2.fromMatlabStringFormat(mat2) ||
		M2.cols()!=2 || M2.rows()!=3 ||
		(CMatrixFixedNumeric<double,3,2>(vals2)-M2).Abs().sumAll() > 1e-4 )
		GTEST_FAIL() << mat2;

	{
		CMatrixFixedNumeric<double,3,2> M2b;
		if (! M2b.fromMatlabStringFormat(mat2) ||
			(CMatrixFixedNumeric<double,3,2>(vals2)-M2b).Abs().sumAll() > 1e-4 )
			GTEST_FAIL() << mat2;
	}

	if (! M3.fromMatlabStringFormat(mat3) )
		GTEST_FAIL() << mat3;

	{
		vector_double m;
		if (! m.fromMatlabStringFormat(mat3) || m.size()!=1 ) GTEST_FAIL() << "vector_double:" << mat3;
	}
	{
		CArrayDouble<1> m;
		if (! m.fromMatlabStringFormat(mat3) ) GTEST_FAIL() << "CArrayDouble<1>:" << mat3;
	}

	{
		vector_double m;
		if (! m.fromMatlabStringFormat(mat31) || m.size()!=3 ) GTEST_FAIL() << "vector_double:" << mat31;
	}
	{
		CArrayDouble<3> m;
		if (! m.fromMatlabStringFormat(mat31) ) GTEST_FAIL() << "CArrayDouble<3>:" << mat31;
	}

	{
		Eigen::Matrix<double,1,3> m;
		if (! m.fromMatlabStringFormat(mat13) ) GTEST_FAIL() << "Matrix<double,1,3>:" << mat13;
	}
	{
		Eigen::Matrix<double,1,Eigen::Dynamic> m;
		if (! m.fromMatlabStringFormat(mat13) || m.size()!=3 ) GTEST_FAIL() << "Matrix<double,1,Dynamic>:" << mat13;
	}

	// This one MUST BE detected as WRONG:
	if ( M4.fromMatlabStringFormat(mat4, false /*dont dump errors to cerr*/) )
		GTEST_FAIL() << mat4;

	if (! M5.fromMatlabStringFormat(mat5) || size(M5,1)!=0 || size(M5,2)!=0 )
		GTEST_FAIL() << mat5;

	if (! M6.fromMatlabStringFormat(mat6) )
		GTEST_FAIL() << mat6;

	// Check correct values loaded:
	CMatrixDouble RES = M1*M2;

	EXPECT_NEAR(0,(M6 - M1*M2).Square().sumAll(), 1e-3);
}


// Start of auxiliary code/data for text matrix_divide =========================
namespace detail_testMatrixDivide
{
	const size_t matrixSize=10;
	const size_t howManySets=4;
	const double eps=1e-7;

	typedef CMatrixTemplateNumeric<double> MyMat;

	class MatrixGenerator	{
	private:
		CRandomGenerator gen;
		size_t sz;
	public:
		inline MatrixGenerator(size_t s):gen(123),sz(s)	{}
		MyMat operator()()	{
			MyMat res(sz,sz);
			do gen.drawUniformMatrix(res,-10.0,10.0); while (res.isSingular(1e-4));
			return res;
		}
	};

	template<typename It1,typename It2> class DoubleIterator	{
	public:
		typedef pair<typename It1::value_type,typename It2::value_type> value_type;
		typedef value_type *pointer;
		typedef value_type &reference;
		typedef ptrdiff_t difference_type;
		typedef input_iterator_tag iterator_category;
	private:
		It1 base1;
		It2 base2;
	public:
		inline DoubleIterator(const It1 &b1,const It2 &b2):base1(b1),base2(b2)	{}
		inline DoubleIterator<It1,It2> operator++()	{
			++base1;
			++base2;
			return *this;
		}
		inline DoubleIterator<It1,It2> operator++(int)	{
			DoubleIterator<It1,It2> res=*this;
			++(*this);
			return res;
		}
		inline bool operator==(const DoubleIterator<It1,It2> &it) const	{
			return (base1==it.base1)&&(base2==it.base2);
		}
		inline bool operator!=(const DoubleIterator<It1,It2> &it) const	{
			return (base1!=it.base1)||(base2!=it.base2);
		}
		inline value_type operator*() const	{
			return make_pair(*base1,*base2);
		}
	};
	template<typename It1,typename It2> inline DoubleIterator<It1,It2> itDouble(const It1 &i1,const It2 &i2)	{
		return DoubleIterator<It1,It2>(i1,i2);
	}

	inline MyMat prod(const MyMat &l,const MyMat &r)	{
		return l*r;
	}

	int compareMatrices(const MyMat &m1,const MyMat &m2)	{
		if ((m1.getRowCount()!=m2.getRowCount())||(m1.getColCount()!=m2.getColCount())) return 1;
		for (size_t i=0;i<m1.getRowCount();++i) for (size_t j=0;j<m1.getColCount();++j) if (abs(m1.get_unsafe(i,j)-m2.get_unsafe(i,j))>eps) 	return 1;
		return 0;
	}

	template<typename T,typename T2,typename T3,typename ConstIt,typename It> void for_eachEX(ConstIt begin1,const ConstIt &end1,ConstIt leftSource,ConstIt rightSource,It leftDest,It rightDest,void (T::*leftFun)(const T2 &,T3 &) const,void (T::*rightFun)(const T2 &,T3 &) const)	{
		while (begin1!=end1)	{
			//cout << "l" << endl << (*begin1) << endl << *leftSource <<endl;
			//cout << "det b: " << begin1->det() << endl;
			//cout << "det s: " << leftSource->det() << endl;
			((*begin1).*leftFun)(*leftSource,*leftDest);
			((*begin1).*rightFun)(*rightSource,*rightDest);
			begin1++;
			leftSource++;
			rightSource++;
			leftDest++;
			rightDest++;
		}
	}

	inline int compareAndSum(int val,const pair<MyMat,MyMat > &p)	{
		return val+compareMatrices(p.first,p.second);
	}
	// End of auxiliary code/data for text matrix_divide =========================
}

TEST(Matrices,divide)
{
	using namespace detail_testMatrixDivide;

	vector<MyMat > leftOps(howManySets,MyMat(0,0));
	vector<MyMat > rightOps(howManySets,MyMat(0,0));
	vector<MyMat > products(howManySets,MyMat(0,0));
	vector<MyMat > leftCheck(howManySets,MyMat(0,0));
	vector<MyMat > rightCheck(howManySets,MyMat(0,0));
	MatrixGenerator gen(matrixSize);
	generate(leftOps.begin(),leftOps.end(),gen);
	generate(rightOps.begin(),rightOps.end(),gen);
	transform(leftOps.begin(),leftOps.end(),rightOps.begin(),products.begin(),&prod);
	for_eachEX(products.begin(),products.end(),leftOps.begin(),rightOps.begin(),leftCheck.begin(),rightCheck.begin(),
				&MyMat::leftDivideSquare<MyMat,MyMat>,
				&MyMat::rightDivideSquare<MyMat,MyMat> );

	EXPECT_TRUE(0==accumulate(itDouble(leftOps.begin(),leftCheck.begin()),itDouble(leftOps.end(),leftCheck.end()),accumulate(itDouble(rightOps.begin(),rightCheck.begin()),itDouble(rightOps.end(),rightCheck.end()),0,&compareAndSum),&compareAndSum));
}

// JL: Disabled as of porting to Eigen...
#if 0
namespace detail_testMatrixMaha
{
	// Start of auxiliary code/data for text matrix_mahalanobis =========================
	const size_t matrixSize=10;
	const size_t howMany=1000;

	typedef double NumericType;

	const NumericType eps=1e-7;

	template<typename It1,typename It2> class DoubleIterator	{
	public:
		typedef pair<typename It1::value_type,typename It2::value_type> value_type;
		typedef value_type *pointer;
		typedef value_type &reference;
		typedef ptrdiff_t difference_type;
		typedef input_iterator_tag iterator_category;
	private:
		It1 base1;
		It2 base2;
	public:
		inline DoubleIterator(const It1 &b1,const It2 &b2):base1(b1),base2(b2)	{}
		inline DoubleIterator<It1,It2> operator++()	{
			++base1;
			++base2;
			return *this;
		}
		inline DoubleIterator<It1,It2> operator++(int)	{
			DoubleIterator<It1,It2> res=*this;
			++(*this);
			return res;
		}
		inline bool operator==(const DoubleIterator<It1,It2> &it) const	{
			return (base1==it.base1)&&(base2==it.base2);
		}
		inline bool operator!=(const DoubleIterator<It1,It2> &it) const	{
			return (base1!=it.base1)||(base2!=it.base2);
		}
		inline value_type operator*() const	{
			return make_pair(*base1,*base2);
		}
	};
	template<typename It1,typename It2> inline DoubleIterator<It1,It2> itDouble(const It1 &i1,const It2 &i2)	{
		return DoubleIterator<It1,It2>(i1,i2);
	}

	template<typename T> class PositiveSemidefiniteGenerator	{
	private:
		CRandomGenerator gen;
		mutable CMatrixTemplateNumeric<T> tmp;
		mutable CMatrixTemplateNumeric<T> res;
	public:
		inline PositiveSemidefiniteGenerator(size_t s):gen(),tmp(s,s),res(s,s)	{}
		inline CMatrixTemplateNumeric<T> operator()()	{
			do gen.drawUniformMatrix(tmp,T(-3.0),T(3.0)); while (tmp.isSingular(eps));
			res.multiply_AAt(tmp);
			return res;
		}
	};

	template<typename T> class RandomVectorGenerator	{
	private:
		CRandomGenerator gen;
		mutable vector<T> res;
	public:
		inline RandomVectorGenerator(size_t s):gen(),res(s,T(0.0))	{}
		inline vector<T> operator()()	{
			gen.drawUniformVector(res,T(-3.0),T(3.0));
			return res;
		}
	};

	template<typename T> inline T get_xCxT_basic(const pair<vector<T>,CMatrixTemplateNumeric<T> > &p)	{
		return multiply_HCHt_scalar(p.first,p.second);
	}

	template<typename T> class Get_xCxT_cholesky	{
	private:
		mutable CMatrixTemplateNumeric<T> R;
		//mutable vector<T> tmpV;
		const size_t N;
	public:
		typedef pair<vector<T>,CMatrixTemplateNumeric<T> > argument_type;
		typedef T result_type;
		inline Get_xCxT_cholesky(size_t s):R(s,s),/*tmpV(s),*/N(s)	{}
		inline T operator()(const pair<vector<T>,CMatrixTemplateNumeric<T> > &p)	{
			const vector<T> &vec=p.first;
			p.second.chol(R);
			T res=T(0);
			for (size_t i=0;i<N-1;++i)	{
				T accum=T(0.0);
				for (size_t j=i;j<N;++j) accum+=R(i,j)*vec[j];
				//tmpV[i]=accum;
				res+=square(accum);
			}
			//tmpV[N-1]=vec[N-1]*R.get_unsafe(0,0);
			res+=square(vec[N-1]*R.get_unsafe(N-1,N-1));
			return res;
		}
	};

	template<typename T> inline size_t compareAndSum(size_t acc,const pair<T,T> &p)	{
		return acc+((p.first-p.second>=T(eps))?1:0);
	}
	// End of auxiliary code/data for text matrix_mahalanobis =========================
}

TEST(Matrices,mahalanobis)
{
	using namespace detail_testMatrixMaha;

	CTicTac time;
	vector<CMatrixTemplateNumeric<NumericType> > matrices(howMany,CMatrixTemplateNumeric<NumericType>(matrixSize,matrixSize));
	vector<mrpt::dynamicsize_vector<NumericType> > vectors(howMany,mrpt::dynamicsize_vector<NumericType>(matrixSize));
	mrpt::dynamicsize_vector<NumericType> res1(howMany);
	mrpt::dynamicsize_vector<NumericType> res2(howMany);
	generate(matrices.begin(),matrices.end(),PositiveSemidefiniteGenerator<NumericType>(matrixSize));
	generate(vectors.begin(),vectors.end(),RandomVectorGenerator<NumericType>(matrixSize));
	time.Tic();
	transform(itDouble(vectors.begin(),matrices.begin()),itDouble(vectors.end(),matrices.end()),res1.begin(),&get_xCxT_basic<NumericType>);
	double t=time.Tac();
	time.Tic();
	transform(itDouble(vectors.begin(),matrices.begin()),itDouble(vectors.end(),matrices.end()),res2.begin(),Get_xCxT_cholesky<NumericType>(matrixSize));
	t=time.Tac();
	EXPECT_TRUE(0==accumulate(itDouble(res1.begin(),res2.begin()),itDouble(res1.end(),res2.end()),NumericType(0.0),&compareAndSum<NumericType>));
}
#endif


TEST(Matrices,meanAndStd)
{
	MRPT_TODO("Do this test...")
}

TEST(Matrices,meanAndStdAll)
{
	MRPT_TODO("Do this test...")
}

TEST(Matrices,laplacian)
{
	MRPT_TODO("Do this test...")
}

TEST(Matrices,largestEigenvector)
{
	{
		const double   dat_C1[] = {
			13.737245,10.248641,-5.839599,11.108320,
			10.248641,14.966139,-5.259922,11.662222,
			-5.839599,-5.259922,9.608822,-4.342505,
			11.108320,11.662222,-4.342505,12.121940 };
		const CMatrixDouble44  C1(dat_C1);

		const double dat_REAL_EIGVEC[] = { 0.54800  , 0.57167,  -0.29604 ,  0.53409 };
		const Eigen::Matrix<double,4,1>  REAL_EIGVEC(dat_REAL_EIGVEC);
		//const double REAL_LARGEST_EIGENVALUE =  38.40966;

		mrpt::vector_double lev;
		C1.largestEigenvector(lev,1e-3, 20);
		EXPECT_NEAR( (REAL_EIGVEC-lev).array().abs().sum(), 0, 1e-3);
	}
}

TEST(Matrices,powerOperator)
{
	MRPT_TODO("Do this test...")
}

TEST(Matrices,loadFromTextFile)
{
	{
		const std::string s1 =
			"1 2 3\n"
			"4 5 6";
		std::stringstream  s(s1);
		CMatrixDouble M;
		bool retval = false;
		try { M.loadFromTextFile(s); retval=true; } catch(std::exception &e) { std::cerr << e.what() << std::endl; }
		EXPECT_TRUE(retval) << "string:\n" << s1 << endl;
		EXPECT_EQ(M.rows(),2);
		EXPECT_EQ(M.cols(),3);
	}
	{
		const std::string s1 =
			"1 \t 2\n"
			"  4 \t\t 1    ";
		std::stringstream  s(s1);
		CMatrixDouble M;
		bool retval = false;
		try { M.loadFromTextFile(s); retval=true; } catch(std::exception &e) { std::cerr << e.what() << std::endl; }
		EXPECT_TRUE(retval) << "string:\n" << s1 << endl;
		EXPECT_EQ(M.rows(),2);
		EXPECT_EQ(M.cols(),2);
	}
	{
		const std::string s1 =
			"1 2";
		std::stringstream  s(s1);
		CMatrixDouble M;
		bool retval = false;
		try { M.loadFromTextFile(s); retval=true; } catch(std::exception &e) { std::cerr << e.what() << std::endl; }
		EXPECT_TRUE(retval) << "string:\n" << s1 << endl;
		EXPECT_EQ(M.rows(),1);
		EXPECT_EQ(M.cols(),2);
	}
	{
		const std::string s1 =
			"1 2 3\n"
			"4 5 6\n";
		std::stringstream  s(s1);
		CMatrixFixedNumeric<double,2,3> M;
		bool retval = false;
		try { M.loadFromTextFile(s); retval=true; } catch(std::exception &e) { std::cerr << e.what() << std::endl; }
		EXPECT_TRUE(retval) << "string:\n" << s1 << endl;
		EXPECT_EQ(M.rows(),2);
		EXPECT_EQ(M.cols(),3);
	}
	{
		const std::string s1 =
			"1 2 3\n"
			"4 5\n";
		std::stringstream  s(s1);
		CMatrixFixedNumeric<double,2,3> M;
		bool retval = false;
		try { M.loadFromTextFile(s); retval=true; } catch(std::exception &) { }
		EXPECT_FALSE(retval) << "string:\n" << s1 << endl;
	}
	{
		const std::string s1 =
			"1 2 3\n"
			"4 5\n";
		std::stringstream  s(s1);
		CMatrixDouble M;
		bool retval = false;
		try { M.loadFromTextFile(s); retval=true; } catch(std::exception &) { }
		EXPECT_FALSE(retval) << "string:\n" << s1 << endl;
	}
	{
		const std::string s1 =
			"  \n";
		std::stringstream  s(s1);
		CMatrixFixedNumeric<double,2,3> M;
		bool retval = false;
		try { M.loadFromTextFile(s); retval=true; } catch(std::exception &) { }
		EXPECT_FALSE(retval) << "string:\n" << s1 << endl;
	}
	{
		const std::string s1 =
			"1 2 3\n"
			"1 2 3\n"
			"1 2 3";
		std::stringstream  s(s1);
		CMatrixFixedNumeric<double,2,3> M;
		bool retval = false;
		try { M.loadFromTextFile(s); retval=true; } catch(std::exception &) { }
		EXPECT_FALSE(retval) << "string:\n" << s1 << endl;
	}
}


