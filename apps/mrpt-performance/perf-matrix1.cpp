/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/math/CMatrixTemplateNumeric.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/random.h>

#include "common.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace std;


// ------------------------------------------------------
// register_tests_matrices:
//   The code for matrices is split in several files
//   to avoid excesive RAM usage by the compiler, which
//   made fail the build in "small" MIPS machines.
// ------------------------------------------------------
void register_tests_matrices1();
void register_tests_matrices2();

void register_tests_matrices()
{
	randomGenerator.randomize(1234);

	register_tests_matrices1();
	register_tests_matrices2();
}

template <typename T>
double matrix_test_unit_dyn(int a1, int a2)
{
	CMatrixTemplateNumeric<T>	C(a1,a1);

	const long N = 1000000;
	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		C.resize(a1,a1);
		C.setIdentity();
	}
	return tictac.Tac()/N;
}

template <typename T,size_t DIM>
double matrix_test_unit_fix(int a1, int a2)
{
	CMatrixFixedNumeric<T,DIM,DIM>	C;

	const long N = 1000000;
	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		C.resize(DIM,DIM);
		C.setIdentity();
	}
	return tictac.Tac()/N;
}


template <typename T,size_t DIM1,size_t DIM2, size_t DIM3>
double matrix_test_mult_dyn(int a1, int a2)
{
	CMatrixTemplateNumeric<T>	A(DIM1,DIM2);
	CMatrixTemplateNumeric<T>	B(DIM2,DIM3);
	CMatrixTemplateNumeric<T>	C(DIM1,DIM3);

	randomGenerator.drawGaussian1DMatrix(A,T(0),T(1));
	randomGenerator.drawGaussian1DMatrix(B,T(0),T(1));

	const long N = 10000;
	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		C.multiply(A,B);
	}
	return tictac.Tac()/N;
}

template <typename T,size_t DIM1,size_t DIM2, size_t DIM3>
double matrix_test_mult_fix(int a1, int a2)
{
	CMatrixFixedNumeric<T,DIM1,DIM2>	A;
	CMatrixFixedNumeric<T,DIM2,DIM3>	B;
	CMatrixFixedNumeric<T,DIM1,DIM3>	C;

	randomGenerator.drawGaussian1DMatrix(A,T(0),T(1));
	randomGenerator.drawGaussian1DMatrix(B,T(0),T(1));

	const long N = 10000;
	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		C.multiply(A,B);
	}
	return tictac.Tac()/N;
}

template <typename T,size_t DIM1>
double matrix_test_inv_dyn(int a1, int a2)
{
	CMatrixTemplateNumeric<T>	A(DIM1,DIM1);
	CMatrixTemplateNumeric<T>	A2(DIM1,DIM1);
	randomGenerator.drawGaussian1DMatrix(A,T(0),T(1));

	const long N = 1000;
	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		A.inv(A2);
	}
	return tictac.Tac()/N;
}

template <typename T,size_t DIM1>
double matrix_test_inv_fix(int a1, int a2)
{
	CMatrixFixedNumeric<T,DIM1,DIM1>	A,A2;
	randomGenerator.drawGaussian1DMatrix(A,T(0),T(1));

	const long N = 1000;
	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		A.inv(A2);
	}
	return tictac.Tac()/N;
}

template <typename T,size_t DIM1>
double matrix_test_det_dyn(int a1, int a2)
{
	CMatrixTemplateNumeric<T>	A(DIM1,DIM1);
	randomGenerator.drawGaussian1DMatrix(A,T(0),T(1));

	const long N = 10000;
	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		A.det();
	}
	return tictac.Tac()/N;
}

template <typename T,size_t DIM1>
double matrix_test_det_fix(int a1, int a2)
{
	CMatrixFixedNumeric<T,DIM1,DIM1>	A;
	randomGenerator.drawGaussian1DMatrix(A,T(0),T(1));

	const long N = 10000;
	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		A.det();
	}
	return tictac.Tac()/N;
}


// ------------------------------------------------------
// register_tests_matrices: Part 1
// ------------------------------------------------------
void register_tests_matrices1()
{
	lstTests.push_back( TestData("matrix: unit, dyn[float], 3x3",matrix_test_unit_dyn<float>,3) );
	lstTests.push_back( TestData("matrix: unit, dyn[double], 3x3",matrix_test_unit_dyn<double>,3) );
	lstTests.push_back( TestData("matrix: unit, dyn[float], 6x6",matrix_test_unit_dyn<float>,6) );
	lstTests.push_back( TestData("matrix: unit, dyn[double], 6x6",matrix_test_unit_dyn<double>,6) );

	lstTests.push_back( TestData("matrix: unit, fix[float,3,3]",matrix_test_unit_fix<float,3>) );
	lstTests.push_back( TestData("matrix: unit, fix[double,3,3]",matrix_test_unit_fix<double,3>) );
	lstTests.push_back( TestData("matrix: unit, fix[float,6,6]",matrix_test_unit_fix<float,6>) );
	lstTests.push_back( TestData("matrix: unit, fix[double,6,6]",matrix_test_unit_fix<double,6>) );

	lstTests.push_back( TestData("matrix: multiply, dyn[float ], 3x3 * 3x3",matrix_test_mult_dyn<float,3,3,3>) );
	lstTests.push_back( TestData("matrix: multiply, fix[float ], 3x3 * 3x3",matrix_test_mult_fix<float,3,3,3>) );
	lstTests.push_back( TestData("matrix: multiply, dyn[double], 3x3 * 3x3",matrix_test_mult_dyn<double,3,3,3>) );
	lstTests.push_back( TestData("matrix: multiply, fix[double], 3x3 * 3x3",matrix_test_mult_fix<double,3,3,3>) );
	lstTests.push_back( TestData("matrix: multiply, dyn[float ], 3x6 * 6x3",matrix_test_mult_dyn<float,3,6,3>) );
	lstTests.push_back( TestData("matrix: multiply, fix[float ], 3x6 * 6x3",matrix_test_mult_fix<float,3,6,3>) );
	lstTests.push_back( TestData("matrix: multiply, dyn[double], 3x6 * 6x3",matrix_test_mult_dyn<double,3,6,3>) );
	lstTests.push_back( TestData("matrix: multiply, fix[double], 3x6 * 6x3",matrix_test_mult_fix<double,3,6,3>) );
	lstTests.push_back( TestData("matrix: multiply, dyn[float ], 10x40 * 40x10",matrix_test_mult_dyn<float,10,40,10>) );
	lstTests.push_back( TestData("matrix: multiply, fix[float ], 10x40 * 40x10",matrix_test_mult_fix<float,10,40,10>) );
	lstTests.push_back( TestData("matrix: multiply, dyn[double], 10x40 * 40x10",matrix_test_mult_dyn<double,10,40,10>) );
	lstTests.push_back( TestData("matrix: multiply, fix[double], 10x40 * 40x10",matrix_test_mult_fix<double,10,40,10>) );

	// Note: All "float" tests below were removed since they produced weird compile errors in MSVC :-(

	lstTests.push_back( TestData("matrix: inv, dyn[double] 3x3",matrix_test_inv_dyn<double,3>) );
	lstTests.push_back( TestData("matrix: inv, fix[double] 3x3",matrix_test_inv_fix<double,3>) );
	lstTests.push_back( TestData("matrix: inv, dyn[double] 6x6",matrix_test_inv_dyn<double,6>) );
	lstTests.push_back( TestData("matrix: inv, fix[double] 6x6",matrix_test_inv_fix<double,6>) );
	lstTests.push_back( TestData("matrix: inv, dyn[double] 20x20",matrix_test_inv_dyn<double,20>) );
	lstTests.push_back( TestData("matrix: inv, fix[double] 20x20",matrix_test_inv_fix<double,20>) );
	lstTests.push_back( TestData("matrix: inv, dyn[double] 40x40",matrix_test_inv_dyn<double,40>) );
	lstTests.push_back( TestData("matrix: inv, fix[double] 40x40",matrix_test_inv_fix<double,40>) );


	lstTests.push_back( TestData("matrix: det, dyn[double] 2x2",matrix_test_det_dyn<double,2>) );
	lstTests.push_back( TestData("matrix: det, fix[double] 2x2",matrix_test_det_fix<double,2>) );
	lstTests.push_back( TestData("matrix: det, dyn[double] 3x3",matrix_test_det_dyn<double,3>) );
	lstTests.push_back( TestData("matrix: det, fix[double] 3x3",matrix_test_det_fix<double,3>) );
	lstTests.push_back( TestData("matrix: det, dyn[double] 6x6",matrix_test_det_dyn<double,6>) );
	lstTests.push_back( TestData("matrix: det, fix[double] 6x6",matrix_test_det_fix<double,6>) );
	lstTests.push_back( TestData("matrix: det, dyn[double] 20x20",matrix_test_det_dyn<double,20>) );
	lstTests.push_back( TestData("matrix: det, fix[double] 20x20",matrix_test_det_fix<double,20>) );
	lstTests.push_back( TestData("matrix: det, dyn[double] 40x40",matrix_test_det_dyn<double,40>) );
	lstTests.push_back( TestData("matrix: det, fix[double] 40x40",matrix_test_det_fix<double,40>) );

}
