/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/math.h>
#include <mrpt/utils/CTicTac.h>
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
