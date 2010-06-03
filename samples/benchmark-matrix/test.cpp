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

// Minimum dependences for this example, to avoid dependency on the entire MRPT, so this program
//  can be used to test quickly changes in matrix classes!

//#define MRPT_USE_SSE2

#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/math/utils.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/random.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace std;

std::string trash;

// Repetitions
size_t  NTIMES = 1;


CTicTac		tictac;

// ------------------------------------------------------
//				TestHCH
// ------------------------------------------------------
void BenchmarkMatrix_1()
{


	CMatrixTemplateNumeric<float>		C(3,3);
	CMatrixFixedNumeric<float,3,3>		D;

	CMatrixTemplateNumeric<double>		Cd(3,3);
	CMatrixFixedNumeric<double,3,3>		Dd;

	size_t i;

	// TEST 1: UNIT
	// ---------------------
	cout << "TEST #1: Unit 3x3 matrix..."; cout.flush();
	tictac.Tic();
	for (i=0;i<NTIMES;i++)
		C.unit();
	cout << "  Dyn.size float: " << 1e6*tictac.Tac() / NTIMES << " us" << endl; cout.flush();

	cout << "TEST #1: Unit 3x3 matrix..."; cout.flush();
	tictac.Tic();
	for (i=0;i<NTIMES;i++)
		D.unit();
	cout << "  Fix.size float: " << 1e6*tictac.Tac() / NTIMES << " us" << endl; cout.flush();

	cout << "TEST #1: Unit 3x3 matrix..."; cout.flush();
	tictac.Tic();
	for (i=0;i<NTIMES;i++)
		Cd.unit();
	cout << "  Dyn.size double: " << 1e6*tictac.Tac() / NTIMES << " us" << endl; cout.flush();

	cout << "TEST #1: Unit 3x3 matrix..."; cout.flush();
	tictac.Tic();
	for (i=0;i<NTIMES;i++)
		Dd.unit();
	cout << "  Fix.size double: " << 1e6*tictac.Tac() / NTIMES << " us" << endl; cout.flush();
}

void BenchmarkMatrix_2()
{
	size_t  NTIMES = 1000;


#define SIZE1 3
#define SIZE2 4

	const double test_A[] = {
		1,3,4,5,
		-6,7,-1,-3,
		0,9,-3,4 };
	const double test_B[] = {
		-1,2,-1,
		4,2,5,
		-8,7,2,
		1,2,3 };

	CMatrixTemplateNumeric<float>	Adf(SIZE1,SIZE2,test_A);
	CMatrixTemplateNumeric<float>	Bdf(SIZE2,SIZE1,test_B);
	CMatrixTemplateNumeric<float>	Rdf(SIZE1,SIZE1);

	CMatrixFixedNumeric<float,SIZE1,SIZE2>	fAdf(test_A);
	CMatrixFixedNumeric<float,SIZE2,SIZE1>	fBdf(test_B);
	CMatrixFixedNumeric<float,SIZE1,SIZE1>	fRdf;

	CMatrixTemplateNumeric<double>	Ad(SIZE1,SIZE2,test_A);
	CMatrixTemplateNumeric<double>	Bd(SIZE2,SIZE1,test_B);
	CMatrixTemplateNumeric<double>	Rd(SIZE1,SIZE1);

	CMatrixFixedNumeric<double,SIZE1,SIZE2>	fAd(test_A);
	CMatrixFixedNumeric<double,SIZE2,SIZE1>	fBd(test_B);
	CMatrixFixedNumeric<double,SIZE1,SIZE1>	fRd;


	size_t i;

	// TEST 2: MULTIPLY
	// ---------------------
	cout << "TEST #2: Multiply " << SIZE1 << "x" << SIZE2 << " x " << SIZE2 << "x" << SIZE1 << "..."; cout.flush();
	tictac.Tic();
	for (i=0;i<NTIMES;i++)
		Rdf.multiply( Adf, Bdf );
	cout << "  Dyn.size float: " << 1e6*tictac.Tac() / NTIMES << " us" << endl;

	cout << "TEST #2: Multiply " << SIZE1 << "x" << SIZE2 << " x " << SIZE2 << "x" << SIZE1 << "..."; cout.flush();
	tictac.Tic();
	for (i=0;i<NTIMES;i++)
		fRdf.multiply( fAdf, fBdf );
	cout << "  Fix.size float: " << 1e6*tictac.Tac() / NTIMES << " us" << endl;

	cout << "TEST #2: Multiply " << SIZE1 << "x" << SIZE2 << " x " << SIZE2 << "x" << SIZE1 << "..."; cout.flush();
	tictac.Tic();
	for (i=0;i<NTIMES;i++)
		Rd.multiply( Ad, Bd );
	cout << "  Dyn.size double: " << 1e6*tictac.Tac() / NTIMES << " us" << endl;

	cout << "TEST #2: Multiply " << SIZE1 << "x" << SIZE2 << " x " << SIZE2 << "x" << SIZE1 << "..."; cout.flush();
	tictac.Tic();
	for (i=0;i<NTIMES;i++)
		fRd.multiply( fAd, fBd );
	cout << "  Fix.size double: " << 1e6*tictac.Tac() / NTIMES << " us" << endl;

}

void BenchmarkMatrix_2bis()
{


#undef SIZE1
#undef SIZE2

#define SIZE1 4
#define SIZE2 4

	const double test_A[] = {
		1,3,4,5,
		-6,7,-1,-3,
		0,9,-3,4,
		-4,1,9,-2};
	const double test_B[] = {
		-1,2,-1,8,
		4,2,5,4,
		-8,7,2,7,
		1,2,3,-5 };


	CMatrixTemplateNumeric<float>	Adf(SIZE1,SIZE2,test_A);
	CMatrixTemplateNumeric<float>	Bdf(SIZE2,SIZE1,test_B);
	CMatrixTemplateNumeric<float>	Rdf(SIZE1,SIZE1);

	CMatrixFixedNumeric<float,SIZE1,SIZE2>	fAdf(test_A);
	CMatrixFixedNumeric<float,SIZE2,SIZE1>	fBdf(test_B);
	CMatrixFixedNumeric<float,SIZE1,SIZE1>	fRdf;

	CMatrixTemplateNumeric<double>	Ad(SIZE1,SIZE2,test_A);
	CMatrixTemplateNumeric<double>	Bd(SIZE2,SIZE1,test_B);
	CMatrixTemplateNumeric<double>	Rd(SIZE1,SIZE1);

	CMatrixFixedNumeric<double,SIZE1,SIZE2>	fAd(test_A);
	CMatrixFixedNumeric<double,SIZE2,SIZE1>	fBd(test_B);
	CMatrixFixedNumeric<double,SIZE1,SIZE1>	fRd;


	size_t i;

	// TEST 2: MULTIPLY
	// ---------------------
	cout << "TEST #2: Multiply " << SIZE1 << "x" << SIZE2 << " x " << SIZE2 << "x" << SIZE1 << "..."; cout.flush();
	tictac.Tic();
	for (i=0;i<NTIMES;i++)
		Rdf.multiply( Adf, Bdf );
	cout << "  Dyn.size float: " << 1e6*tictac.Tac() / NTIMES << " us" << endl;

	cout << "TEST #2: Multiply " << SIZE1 << "x" << SIZE2 << " x " << SIZE2 << "x" << SIZE1 << "..."; cout.flush();
	tictac.Tic();
	for (i=0;i<NTIMES;i++)
		fRdf.multiply( fAdf, fBdf );
	cout << "  Fix.size float: " << 1e6*tictac.Tac() / NTIMES << " us" << endl;


	cout << "TEST #2: Multiply " << SIZE1 << "x" << SIZE2 << " x " << SIZE2 << "x" << SIZE1 << "..."; cout.flush();
	tictac.Tic();
	for (i=0;i<NTIMES;i++)
		Rd.multiply( Ad, Bd );
	cout << "  Dyn.size double: " << 1e6*tictac.Tac() / NTIMES << " us" << endl;

	cout << "TEST #2: Multiply " << SIZE1 << "x" << SIZE2 << " x " << SIZE2 << "x" << SIZE1 << "..."; cout.flush();
	tictac.Tic();
	for (i=0;i<NTIMES;i++)
		fRd.multiply( fAd, fBd );
	cout << "  Fix.size double: " << 1e6*tictac.Tac() / NTIMES << " us" << endl;

	//cout << Rdf << endl; cout << fRdf << endl;
	//cout << Rd << endl; cout << fRd << endl;
}


void BenchmarkMatrix_3()
{


	CMatrixTemplateNumeric<float>	A(15,4),B(4,4),C,R, GOOD_R;

	// Make A a random matrix:
	randomGenerator.drawGaussian1DMatrix(A);
	randomGenerator.drawGaussian1DMatrix(B);

	B = B * (~B);
	//cout << "B:" << endl << B;


	// R = A * B * At
	C = A * B;
	A = ~A;

	GOOD_R = C*A;

	// Fixed:
	CMatrixFixedNumeric<double,15,4> fA;
	CMatrixFixedNumeric<double,4,4>  fB;

	randomGenerator.drawGaussian1DMatrix(fA);
	randomGenerator.drawGaussian1DMatrix(fB);

	fB = fB * (~fB);
//	cout << "fB:" << endl << fB << endl;


	size_t i;
	double At;

	// TEST 3: MULTIPLY vs MULTIPLY SYM:
	// ------------------------------------
	cout << "TEST #3: Multiply 15x4*4x4 vs. mult.out is sym..."; cout.flush();
	tictac.Tic();
	for (i=0;i<NTIMES;i++)
	{
		R.multiply(C,A);
	}
	At = 1e6*tictac.Tac() / NTIMES;
	//R.saveToTextFile("R_normal.txt");
	cout << "  Normal (float): " << At << " us" << endl;// << R << endl;

	CMatrixTemplateNumeric<double>	Rd; Rd=R;
	CMatrixTemplateNumeric<double>	Cd; Cd=C;
	CMatrixTemplateNumeric<double>	Ad; Ad=A;
	CMatrixTemplateNumeric<double>	GOOD_Rd; GOOD_Rd = C * A;

	cout << "TEST #3: Multiply 15x4*4x4 vs. mult.out is sym..."; cout.flush();
	tictac.Tic();
	for (i=0;i<NTIMES;i++)
	{
		Rd.multiply(Cd,Ad);
	}
	At = 1e6*tictac.Tac() / NTIMES;
	cout << "  Normal (double): " << At << " us" << endl;// << R << endl;

	cout << "TEST #3: Multiply 15x4*4x4 vs. mult.out is sym..."; cout.flush();
	tictac.Tic();
	for (i=0;i<NTIMES;i++)
	{
		R.multiply_result_is_symmetric(C,A);
	}
	At = 1e6*tictac.Tac() / NTIMES;
	//R.saveToTextFile("R_fast.txt");
	cout << "  Fast (float): " << At << " us." << endl;// << R << endl;

	cout << "TEST #3: Multiply 15x4*4x4 vs. mult.out is sym..."; cout.flush();
	tictac.Tic();
	for (i=0;i<NTIMES;i++)
	{
		Rd.multiply_result_is_symmetric(Cd,Ad);
	}
	At = 1e6*tictac.Tac() / NTIMES;
	//Rd.saveToTextFile("Rd_fast.txt");
	cout << "  Fast (double): " << At << " us." << endl;
}

template <typename T,size_t M_>
void BenchmarkMatrix_5f(size_t  NTIMES)
{
	CMatrixFixedNumeric<T,M_,M_>   fA,fAinv;
	cout << "TEST #5: " << M_ << "x" << M_ << " matrix inversion..."; cout.flush();
	randomGenerator.drawGaussian1DMatrix(fA);
	tictac.Tic();
	for (size_t k=0;k<NTIMES;k++)
		fA.inv(fAinv);
	double At = 1e6*tictac.Tac() / NTIMES;
	cout << "Fix. size (" << TTypeName<T>::get() << "): " << At << " us" << endl;
}

void BenchmarkMatrix_4()
{
	size_t  NTIMES_V[] = {100,100,100, 60,50,40 , 20, 10};
	size_t  sizes[]    = { 2,   3,  6, 20,50,100,200, 500};

	for (size_t i=0;i<sizeof(sizes)/sizeof(sizes[0]);i++)
	{
		size_t M = sizes[i];
		size_t  NTIMES = NTIMES_V[i];

		CMatrixTemplateNumeric<float>	A(M,M),Ainv;

		randomGenerator.drawGaussian1DMatrix(A);

		size_t   k;
		double At;

		// TEST 4: MATRIX INVERSION
		// ------------------------------------
		cout << "TEST #4: " << M << "x" << M << " matrix inversion..."; cout.flush();
		tictac.Tic();
		for (k=0;k<NTIMES;k++)
			Ainv = A.inv();
		At = 1e6*tictac.Tac() / NTIMES;
		cout << " Dyn.size (float): " << At << " us" << endl;

		CMatrixTemplateNumeric<double>	Ad,Adinv;
		Ad=A;

		cout << "TEST #4: " << M << "x" << M << " matrix inversion..."; cout.flush();
		tictac.Tic();
		for (k=0;k<NTIMES;k++)
		{
			Adinv = Ad.inv();
		}
		At = 1e6*tictac.Tac() / NTIMES;
		cout << " Dyn.size (double): " << At << " us" << endl;// << R << endl;
	}

	BenchmarkMatrix_5f<float,2>(100); BenchmarkMatrix_5f<double,2>(100);
	BenchmarkMatrix_5f<float,3>(100); BenchmarkMatrix_5f<double,3>(100);
	BenchmarkMatrix_5f<float,6>(100); BenchmarkMatrix_5f<double,6>(100);
	BenchmarkMatrix_5f<float,20>(60); BenchmarkMatrix_5f<double,20>(60);
	BenchmarkMatrix_5f<float,50>(50); BenchmarkMatrix_5f<double,50>(50);
	BenchmarkMatrix_5f<float,100>(40); BenchmarkMatrix_5f<double,100>(40);
	BenchmarkMatrix_5f<float,200>(20); BenchmarkMatrix_5f<double,200>(20);
	BenchmarkMatrix_5f<float,500>(10); BenchmarkMatrix_5f<double,500>(10);

}

#define BENCHMARK_6(M_)  \
void BenchmarkMatrix_6_##M_()\
{ \
	 \
	CMatrixFixedNumeric<float,M_,M_>   fA; \
	cout << "TEST #6: " << M_ << "x" << M_ << " matrix determinant..."; cout.flush(); \
	randomGenerator.drawGaussian1DMatrix(fA); \
	tictac.Tic(); \
	size_t   k; \
	float VAL; \
	for (k=0;k<NTIMES;k++) \
		VAL = fA.det(); \
	double At = 1e6*tictac.Tac() / NTIMES; \
	cout << " Fix.size (float): " << At << " us." << endl; \
	/* cout << " Mat: " << endl << fA << " det= " << VAL << endl; */ \
}

#define BENCHMARK_6d(M_)  \
void BenchmarkMatrix_6d_##M_()\
{ \
	 \
	CMatrixFixedNumeric<double,M_,M_>   fA; \
	cout << "TEST #6: " << M_ << "x" << M_ << " matrix determinant..."; cout.flush(); \
	randomGenerator.drawGaussian1DMatrix(fA); \
	tictac.Tic(); \
	size_t   k; \
	double VAL; \
	for (k=0;k<NTIMES;k++) \
		VAL = fA.det(); \
	double At = 1e6*tictac.Tac() / NTIMES; \
	cout << " Fix.size (double): " << At << " us." << endl; \
	/*cout << " Mat: " << endl << fA << " det= " << VAL << endl;*/ \
}

BENCHMARK_6(2) BENCHMARK_6d(2)
BENCHMARK_6(3) BENCHMARK_6d(3)
BENCHMARK_6(4) BENCHMARK_6d(4)
BENCHMARK_6(5) BENCHMARK_6d(5)


void BenchmarkMatrix_7()
{


	size_t  sizes[] = { 2,3,4,10,50 };

	for (size_t i=0;i<sizeof(sizes)/sizeof(sizes[0]);i++)
	{
		size_t M = sizes[i];
		CMatrixFloat	A(M,M);
		CMatrixDouble	Ad(M,M);

		float val  = 1.08;
		float vald = 1.08;

		randomGenerator.drawGaussian1DMatrix(A);
		randomGenerator.drawGaussian1DMatrix(Ad);

		cout << "TEST #7: Multiply " << M << "x" << M << " by scalar..."; cout.flush();
		tictac.Tic();
		for (size_t k=0;k<NTIMES;k++)
			A*=val;
		double At = 1e6*tictac.Tac() / NTIMES;
		cout << "Dyn. size (float): " << At << " us" << endl;

		cout << "TEST #7: Multiply " << M << "x" << M << " by scalar..."; cout.flush();
		tictac.Tic();
		for (size_t k=0;k<NTIMES;k++)
			Ad*=vald;
		At = 1e6*tictac.Tac() / NTIMES;
		cout << "Dyn. size (double): " << At << " us" << endl;
	}
}

#define BENCHMARK_7(M_)  \
void BenchmarkMatrix_7_##M_()\
{ \
	 \
	CMatrixFixedNumeric<float,M_,M_>   fA; \
	cout << "TEST #7: Multiply " << M_ << "x" << M_ << " by scalar..."; cout.flush(); \
	randomGenerator.drawGaussian1DMatrix(fA); \
	tictac.Tic(); \
	size_t   k; \
	float val  = 1.01; \
	for (k=0;k<NTIMES;k++) \
		fA*=val; \
	double At = 1e6*tictac.Tac() / NTIMES; \
	cout << "Fix. size (float): " << At << " us" << endl; \
}
#define BENCHMARK_7d(M_)  \
void BenchmarkMatrix_7d_##M_()\
{ \
	 \
	CMatrixFixedNumeric<double,M_,M_>   fA; \
	cout << "TEST #7: Multiply " << M_ << "x" << M_ << " by scalar..."; cout.flush(); \
	randomGenerator.drawGaussian1DMatrix(fA); \
	tictac.Tic(); \
	size_t   k; \
	double val  = 1.01; \
	for (k=0;k<NTIMES;k++) \
		fA*=val; \
	double At = 1e6*tictac.Tac() / NTIMES; \
	cout << "Fix. size (double): " << At << " us" << endl; \
}

BENCHMARK_7(2) BENCHMARK_7d(2)
BENCHMARK_7(3) BENCHMARK_7d(3)
BENCHMARK_7(4) BENCHMARK_7d(4)
BENCHMARK_7(10) BENCHMARK_7d(10)
BENCHMARK_7(50) BENCHMARK_7d(50)

void BenchmarkMatrix_8()
{


	size_t  sizes[] = { 2,3,4,10,50 };

	for (size_t i=0;i<sizeof(sizes)/sizeof(sizes[0]);i++)
	{
		size_t M = sizes[i];
		CMatrixFloat	A(M,M);
		CMatrixDouble	Ad(M,M);

		float val  = 1.08;
		float vald = 1.08;

		randomGenerator.drawGaussian1DMatrix(A);
		randomGenerator.drawGaussian1DMatrix(Ad);

		cout << "TEST #8: Add " << M << "x" << M << " a scalar..."; cout.flush();
		tictac.Tic();
		for (size_t k=0;k<NTIMES;k++)
			A+=val;
		double At = 1e6*tictac.Tac() / NTIMES;
		cout << "Dyn. size (float): " << At << " us" << endl;

		cout << "TEST #8: Add " << M << "x" << M << " a scalar..."; cout.flush();
		tictac.Tic();
		for (size_t k=0;k<NTIMES;k++)
			Ad+=vald;
		At = 1e6*tictac.Tac() / NTIMES;
		cout << "Dyn. size (double): " << At << " us" << endl;
	}
}

#define BENCHMARK_8(M_)  \
void BenchmarkMatrix_8_##M_()\
{ \
	 \
	CMatrixFixedNumeric<float,M_,M_>   fA; \
	cout << "TEST #8: Add " << M_ << "x" << M_ << " a scalar..."; cout.flush(); \
	randomGenerator.drawGaussian1DMatrix(fA); \
	tictac.Tic(); \
	size_t   k; \
	float val  = 1.08; \
	for (k=0;k<NTIMES;k++) \
		fA+=val; \
	double At = 1e6*tictac.Tac() / NTIMES; \
	cout << "Fix. size (float): " << At << " us" << endl; \
}
#define BENCHMARK_8d(M_)  \
void BenchmarkMatrix_8d_##M_()\
{ \
	 \
	CMatrixFixedNumeric<double,M_,M_>   fA; \
	cout << "TEST #8: Add " << M_ << "x" << M_ << " a scalar..."; cout.flush(); \
	randomGenerator.drawGaussian1DMatrix(fA); \
	tictac.Tic(); \
	size_t   k; \
	double val  = 1.08; \
	for (k=0;k<NTIMES;k++) \
		fA+=val; \
	double At = 1e6*tictac.Tac() / NTIMES; \
	cout << "Fix. size (double): " << At << " us" << endl; \
}

BENCHMARK_8(2) BENCHMARK_8d(2)
BENCHMARK_8(3) BENCHMARK_8d(3)
BENCHMARK_8(4) BENCHMARK_8d(4)
BENCHMARK_8(10) BENCHMARK_8d(10)
BENCHMARK_8(50) BENCHMARK_8d(50)


void BenchmarkMatrix_9()
{


	size_t  sizes[] = { 2,3,4,10,50 };

	for (size_t i=0;i<sizeof(sizes)/sizeof(sizes[0]);i++)
	{
		size_t M = sizes[i];
		CMatrixFloat	A(M,M),B(M,M);
		CMatrixDouble	Ad(M,M),Bd(M,M);

		randomGenerator.drawGaussian1DMatrix(A);
		randomGenerator.drawGaussian1DMatrix(Ad);
		randomGenerator.drawGaussian1DMatrix(B);
		randomGenerator.drawGaussian1DMatrix(Bd);

		cout << "TEST #9: Sum two " << M << "x" << M << " matrices..."; cout.flush();
		tictac.Tic();
		for (size_t k=0;k<NTIMES;k++)
			A+=B;
		double At = 1e6*tictac.Tac() / NTIMES;
		cout << "Dyn. size (float): " << At << " us" << endl;

		cout << "TEST #9: Sum two " << M << "x" << M << " matrices..."; cout.flush();
		tictac.Tic();
		for (size_t k=0;k<NTIMES;k++)
			Ad+=Bd;
		At = 1e6*tictac.Tac() / NTIMES;
		cout << "Dyn. size (double): " << At << " us" << endl;
	}
}

#define BENCHMARK_9(M_)  \
void BenchmarkMatrix_9_##M_()\
{ \
	 \
	CMatrixFixedNumeric<float,M_,M_>   fA,fB; \
	cout << "TEST #9: Sum two " << M_ << "x" << M_ << " matrices..."; cout.flush(); \
	randomGenerator.drawGaussian1DMatrix(fA); \
	randomGenerator.drawGaussian1DMatrix(fB); \
	tictac.Tic(); \
	size_t   k; \
	for (k=0;k<NTIMES;k++) \
		fA+=fB; \
	double At = 1e6*tictac.Tac() / NTIMES; \
	cout << "Fix. size (float): " << At << " us" << endl; \
}
#define BENCHMARK_9d(M_)  \
void BenchmarkMatrix_9d_##M_()\
{ \
	 \
	CMatrixFixedNumeric<double,M_,M_>   fA,fB; \
	cout << "TEST #9: Sum two " << M_ << "x" << M_ << " matrices..."; cout.flush(); \
	randomGenerator.drawGaussian1DMatrix(fA); \
	randomGenerator.drawGaussian1DMatrix(fB); \
	tictac.Tic(); \
	size_t   k; \
	for (k=0;k<NTIMES;k++) \
		fA+=fB; \
	double At = 1e6*tictac.Tac() / NTIMES; \
	cout << "Fix. size (double): " << At << " us" << endl; \
}

BENCHMARK_9(2) BENCHMARK_9d(2)
BENCHMARK_9(3) BENCHMARK_9d(3)
BENCHMARK_9(4) BENCHMARK_9d(4)
BENCHMARK_9(10) BENCHMARK_9d(10)
BENCHMARK_9(50) BENCHMARK_9d(50)


void BenchmarkMatrix_10()
{


	size_t  sizes[]  = { 2,3,4,10,50 };

	for (size_t i=0;i<sizeof(sizes)/sizeof(sizes[0]);i++)
	{
		const size_t M2 = 4;
		const size_t M  = sizes[i];
		CMatrixFloat	A(M,M),H(M2,M), R(M2,M);
		CMatrixDouble	Ad(M,M),Hd(M2,M), Rd(M2,M);

		randomGenerator.drawGaussian1DMatrix(A);
		randomGenerator.drawGaussian1DMatrix(Ad);
		randomGenerator.drawGaussian1DMatrix(H);
		randomGenerator.drawGaussian1DMatrix(Hd);

		A  = A * ~A;
		Ad = Ad * ~Ad;

		cout << "TEST #10: HCH^t with " << M2 << "x" << M << " * " << M << "x" << M << "..."; cout.flush();
		tictac.Tic();
		for (size_t k=0;k<NTIMES;k++)
			H.multiply_HCHt(A,R);
		double At = 1e6*tictac.Tac() / NTIMES;
		cout << "Dyn. size (float): " << At << " us" << endl;

		cout << "TEST #10: HCH^t with " << M2 << "x" << M << " * " << M << "x" << M << "..."; cout.flush();
		tictac.Tic();
		for (size_t k=0;k<NTIMES;k++)
			Hd.multiply_HCHt(Ad,Rd);
		At = 1e6*tictac.Tac() / NTIMES;
		cout << "Dyn. size (double): " << At << " us" << endl;
	}
}

#define BENCHMARK_10(M_)  \
void BenchmarkMatrix_10_##M_()\
{ \
	 \
	const size_t M2 = 4; \
	CMatrixFixedNumeric<float,M_,M_>   fA; \
	CMatrixFixedNumeric<float,M2,M_>   fH; \
	CMatrixFixedNumeric<float,M2,M2>   fR; \
	cout << "TEST #10: HCH^t with " << M2 << "x" << M_ << " * " << M_ << "x" << M_ << "..."; cout.flush(); \
	randomGenerator.drawGaussian1DMatrix(fA); \
	randomGenerator.drawGaussian1DMatrix(fH); \
	fA  = fA * ~fA; \
	tictac.Tic(); \
	size_t   k; \
	for (k=0;k<NTIMES;k++) \
		fH.multiply_HCHt(fA,fR); \
	double At = 1e6*tictac.Tac() / NTIMES; \
	cout << "Fix. size (float): " << At << " us" << endl; \
}
#define BENCHMARK_10d(M_)  \
void BenchmarkMatrix_10d_##M_()\
{ \
	 \
	const size_t M2 = 4; \
	CMatrixFixedNumeric<double,M_,M_>   fA; \
	CMatrixFixedNumeric<double,M2,M_>   fH; \
	CMatrixFixedNumeric<double,M2,M2>   fR; \
	cout << "TEST #10: HCH^t with " << M2 << "x" << M_ << " * " << M_ << "x" << M_ << "..."; cout.flush(); \
	randomGenerator.drawGaussian1DMatrix(fA); \
	randomGenerator.drawGaussian1DMatrix(fH); \
	fA  = fA * ~fA; \
	fA.saveToTextFile("A.txt"); \
	fH.saveToTextFile("H.txt"); \
	tictac.Tic(); \
	size_t   k; \
	for (k=0;k<NTIMES;k++) \
		fH.multiply_HCHt(fA,fR); \
	double At = 1e6*tictac.Tac() / NTIMES; \
	fR.saveToTextFile("R.txt"); \
	cout << "Fix. size (double): " << At << " us" << endl; \
}

BENCHMARK_10(2) BENCHMARK_10d(2)
BENCHMARK_10(3) BENCHMARK_10d(3)
BENCHMARK_10(4) BENCHMARK_10d(4)
BENCHMARK_10(10) BENCHMARK_10d(10)
BENCHMARK_10(50) BENCHMARK_10d(50)


void BenchmarkMatrix_11()
{


	size_t  sizes[] = { 2,4,10,20,30,50 };

	for (size_t i=0;i<sizeof(sizes)/sizeof(sizes[0]);i++)
	{
		size_t M = sizes[i];
		CMatrixFloat	A(M,M);
		CMatrixDouble	Ad(M,M);

		randomGenerator.drawGaussian1DMatrix(A);
		randomGenerator.drawGaussian1DMatrix(Ad);

		cout << "TEST #11: Sum all elements in " << M << "x" << M << "..."; cout.flush();
		tictac.Tic();
		for (size_t k=0;k<NTIMES;k++)
			A.sumAll();
		double At = 1e6*tictac.Tac() / NTIMES;
		cout << "Dyn. size (float): " << At << " us" << endl;

		cout << "TEST #11: Sum all elements in " << M << "x" << M << "..."; cout.flush();
		tictac.Tic();
		for (size_t k=0;k<NTIMES;k++)
			Ad.sumAll();
		At = 1e6*tictac.Tac() / NTIMES;
		cout << "Dyn. size (double): " << At << " us" << endl;
	}
}

#define BENCHMARK_11(M_)  \
void BenchmarkMatrix_11_##M_()\
{ \
	 \
	CMatrixFixedNumeric<float,M_,M_>   fA; \
	cout << "TEST #11: Sum all elements in " << M_ << "x" << M_ << "..."; cout.flush(); \
	randomGenerator.drawGaussian1DMatrix(fA); \
	tictac.Tic(); \
	size_t   k; \
	float T=0; \
	for (k=0;k<NTIMES;k++) \
		T = fA.sumAll(); \
	double At = 1e6*tictac.Tac() / NTIMES; \
	trash+= format("%f",T); \
	cout << "Fix. size (float): " << At << " us" << endl; \
}
#define BENCHMARK_11d(M_)  \
void BenchmarkMatrix_11d_##M_()\
{ \
	 \
	CMatrixFixedNumeric<double,M_,M_>   fA; \
	cout << "TEST #11: Sum all elements in " << M_ << "x" << M_ << "..."; cout.flush(); \
	randomGenerator.drawGaussian1DMatrix(fA); \
	tictac.Tic(); \
	size_t   k; \
	double T=0; \
	for (k=0;k<NTIMES;k++) \
		T = fA.sumAll(); \
	double At = 1e6*tictac.Tac() / NTIMES; \
	trash+= format("%f",T); \
	cout << "Fix. size (double): " << At << " us" << endl; \
}

BENCHMARK_11(2) BENCHMARK_11d(2)
BENCHMARK_11(4) BENCHMARK_11d(4)
BENCHMARK_11(10) BENCHMARK_11d(10)
BENCHMARK_11(20) BENCHMARK_11d(20)
BENCHMARK_11(30) BENCHMARK_11d(30)
BENCHMARK_11(50) BENCHMARK_11d(50)

template <typename T,size_t M_>
void BenchmarkMatrix_12f()
{
	CMatrixFixedNumeric<T,M_,M_>   fA;
	cout << "TEST #12: Minimum of " << M_ << "x" << M_ << "..."; cout.flush();
	randomGenerator.drawGaussian1DMatrix(fA);
	tictac.Tic();
	for (size_t k=0;k<NTIMES;k++)
		fA.minimum();
	double At = 1e6*tictac.Tac() / NTIMES;
	cout << "Fix. size (" << TTypeName<T>::get() << "): " << At << " us" << endl;
}

void BenchmarkMatrix_12()
{
	size_t  sizes[] = { 2,4,10,20,30,50 };

	for (size_t i=0;i<sizeof(sizes)/sizeof(sizes[0]);i++)
	{
		size_t M = sizes[i];
		CMatrixFloat	A(M,M);
		CMatrixDouble	Ad(M,M);

		randomGenerator.drawGaussian1DMatrix(A);
		randomGenerator.drawGaussian1DMatrix(Ad);

		cout << "TEST #12: Minimum of " << M << "x" << M << "..."; cout.flush();
		tictac.Tic();
		float Q;
		for (size_t k=0;k<NTIMES;k++)
			Q = A.minimum();
		double At = 1e6*tictac.Tac() / NTIMES;
		cout << "Dyn. size (float): " << At << " us" << endl;

		cout << "TEST #12: Minimum of " << M << "x" << M << "..."; cout.flush();
		tictac.Tic();
		double Qd;
		for (size_t k=0;k<NTIMES;k++)
			Qd = Ad.minimum();
		At = 1e6*tictac.Tac() / NTIMES;
		cout << "Dyn. size (double): " << At << " us" << endl;

	}
	BenchmarkMatrix_12f<float,2>(); BenchmarkMatrix_12f<double,2>();
	BenchmarkMatrix_12f<float,4>(); BenchmarkMatrix_12f<double,4>();
	BenchmarkMatrix_12f<float,10>(); BenchmarkMatrix_12f<double,10>();
	BenchmarkMatrix_12f<float,20>(); BenchmarkMatrix_12f<double,20>();
	BenchmarkMatrix_12f<float,30>(); BenchmarkMatrix_12f<double,30>();
	BenchmarkMatrix_12f<float,50>(); BenchmarkMatrix_12f<double,50>();
}


template <typename T,size_t M_>
void BenchmarkMatrix_13f(size_t NTIMES)
{
	CMatrixFixedNumeric<T,M_,M_>   fA,fZ,fD;
	cout << "TEST #13: Eigenvectors/values of " << M_ << "x" << M_ << "..."; cout.flush();
	randomGenerator.drawGaussian1DMatrix(fA);
	fA = fA * ~fA;
	tictac.Tic();
	for (size_t k=0;k<NTIMES;k++)
		fA.eigenVectors(fZ,fD);
	double At = 1e6*tictac.Tac() / NTIMES;
	cout << "Fix. size (" << TTypeName<T>::get() << "): " << At << " us" << endl;
}
void BenchmarkMatrix_13()
{
	size_t  sizes[] =    { 2,3,4,10,30,100 };
	size_t  NTIMES_V[] = {100,100,100,60,30,10 };

	for (size_t i=0;i<sizeof(sizes)/sizeof(sizes[0]);i++)
	{
		size_t M = sizes[i];
		size_t NTIMES = NTIMES_V[i];
		CMatrixFloat	A(M,M), Z,D;
		CMatrixDouble	Ad(M,M), Zd,Dd;

		randomGenerator.drawGaussian1DMatrix(A);
		randomGenerator.drawGaussian1DMatrix(Ad);
		A  = A * ~A;
		Ad = Ad * ~Ad;

		cout << "TEST #13: Eigenvectors/values of " << M << "x" << M << "..."; cout.flush();
		tictac.Tic();
		for (size_t k=0;k<NTIMES;k++)
			A.eigenVectors(Z,D);
		double At = 1e6*tictac.Tac() / NTIMES;
		cout << "Dyn. size (float): " << At << " us" << endl;
		//cout << A << Z << D << endl;

		cout << "TEST #13: Eigenvectors/values of " << M << "x" << M << "..."; cout.flush();
		tictac.Tic();
		for (size_t k=0;k<NTIMES;k++)
			Ad.eigenVectors(Zd,Dd);
		At = 1e6*tictac.Tac() / NTIMES;
		cout << "Dyn. size (double): " << At << " us" << endl;

	}

	BenchmarkMatrix_13f<float,2>(100); BenchmarkMatrix_13f<double,2>(100);
	BenchmarkMatrix_13f<float,3>(100); BenchmarkMatrix_13f<double,3>(100);
	BenchmarkMatrix_13f<float,4>(100); BenchmarkMatrix_13f<double,4>(100);
	BenchmarkMatrix_13f<float,10>(60); BenchmarkMatrix_13f<double,10>(60);
	BenchmarkMatrix_13f<float,30>(30); BenchmarkMatrix_13f<double,30>(30);
	BenchmarkMatrix_13f<float,100>(10); BenchmarkMatrix_13f<double,100>(10);
}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
#if 1
		// Unit
		BenchmarkMatrix_1();

		// Multiply
		BenchmarkMatrix_2();
		BenchmarkMatrix_2bis();

		// Multiply vs. mult. output is sym
		BenchmarkMatrix_3();

		// Inverse
		BenchmarkMatrix_4();

		// Determinant
		BenchmarkMatrix_6_2();
		BenchmarkMatrix_6_3();
		BenchmarkMatrix_6_4();
		BenchmarkMatrix_6_5();
		BenchmarkMatrix_6d_2();
		BenchmarkMatrix_6d_3();
		BenchmarkMatrix_6d_4();
		BenchmarkMatrix_6d_5();

		// *= scalar
		BenchmarkMatrix_7();

		BenchmarkMatrix_7_2();
		BenchmarkMatrix_7d_2();
		BenchmarkMatrix_7_3();
		BenchmarkMatrix_7d_3();
		BenchmarkMatrix_7_4();
		BenchmarkMatrix_7d_4();
		BenchmarkMatrix_7_10();
		BenchmarkMatrix_7d_10();
		BenchmarkMatrix_7_50();
		BenchmarkMatrix_7d_50();

		// += scalar
		BenchmarkMatrix_8();

		BenchmarkMatrix_8_2();
		BenchmarkMatrix_8d_2();
		BenchmarkMatrix_8_3();
		BenchmarkMatrix_8d_3();
		BenchmarkMatrix_8_4();
		BenchmarkMatrix_8d_4();
		BenchmarkMatrix_8_10();
		BenchmarkMatrix_8d_10();
		BenchmarkMatrix_8_50();
		BenchmarkMatrix_8d_50();

		// += matrix
		BenchmarkMatrix_9();

		BenchmarkMatrix_9_2();
		BenchmarkMatrix_9d_2();
		BenchmarkMatrix_9_3();
		BenchmarkMatrix_9d_3();
		BenchmarkMatrix_9_4();
		BenchmarkMatrix_9d_4();
		BenchmarkMatrix_9_10();
		BenchmarkMatrix_9d_10();
		BenchmarkMatrix_9_50();
		BenchmarkMatrix_9d_50();


		// HCH^t
		BenchmarkMatrix_10();

		BenchmarkMatrix_10_2();
		BenchmarkMatrix_10d_2();
		BenchmarkMatrix_10_3();
		BenchmarkMatrix_10d_3();
		BenchmarkMatrix_10_4();
		BenchmarkMatrix_10d_4();
		BenchmarkMatrix_10_10();
		BenchmarkMatrix_10d_10();
		BenchmarkMatrix_10_50();
		BenchmarkMatrix_10d_50();

		// Sum all
		BenchmarkMatrix_11();
		BenchmarkMatrix_11_2();
		BenchmarkMatrix_11d_2();
		BenchmarkMatrix_11_4();
		BenchmarkMatrix_11d_4();
		BenchmarkMatrix_11_10();
		BenchmarkMatrix_11d_10();
		BenchmarkMatrix_11_20();
		BenchmarkMatrix_11d_20();
		BenchmarkMatrix_11_30();
		BenchmarkMatrix_11d_30();
		BenchmarkMatrix_11_50();
		BenchmarkMatrix_11d_50();

#endif
		// Minimum
		BenchmarkMatrix_12();

		// Eigenvectors
		BenchmarkMatrix_13();

		return 0;
	} catch (exception &e)
	{
		cout << "MRPT exception caught: " << e.what() << endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}
