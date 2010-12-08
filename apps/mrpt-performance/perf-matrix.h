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

// ------------------------------------------------------
//				Benchmark Matrices
// ------------------------------------------------------
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


// Cholesky:
template <typename T,size_t DIM1>
double matrix_test_chol_dyn(int a1, int a2)
{
	CMatrixTemplateNumeric<T>	A = randomGenerator.drawDefinitePositiveMatrix(DIM1, 0.2);
	CMatrixTemplateNumeric<T>   chol_U;

	const long N = 100;
	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		A.chol(chol_U);
	}
	return tictac.Tac()/N;
}

double matrix_test_chol_Nx6x6_dyn(int DIM, int nReps)
{
	CMatrixDouble  C(DIM*6,DIM*6);
	for (int i=0;i<DIM;i++)
	{
		CMatrixDouble subCov = randomGenerator.drawDefinitePositiveMatrix(6, 0.2);
		C.insertMatrix(i*6,i*6,subCov);
	}

	CMatrixDouble	chol_U;

	const long N = nReps==0 ?  10 : nReps;
	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		C.chol(chol_U);
	}
	return tictac.Tac()/N;
}


template <typename T,size_t DIM1>
double matrix_test_chol_fix(int a1, int a2)
{
	CMatrixFixedNumeric<T,DIM1,DIM1>	A = randomGenerator.drawDefinitePositiveMatrix(DIM1, 0.2);
	CMatrixFixedNumeric<T,DIM1,DIM1>	chol_U;

	const long N = 100;
	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		A.chol(chol_U);
	}
	return tictac.Tac()/N;
}

template <size_t DIM1, size_t DIM2>
double matrix_test_chol_sparse(int a1, int a2)
{
	CMatrixDouble	A1 = randomGenerator.drawDefinitePositiveMatrix(DIM1, 0.2);
	CMatrixDouble	A2 = randomGenerator.drawDefinitePositiveMatrix(DIM2, 0.2);

	CMatrixDouble	A(DIM1+DIM2,DIM1+DIM2);
	A.insertMatrix(0,0,A1);
	A.insertMatrix(DIM1,DIM1,A2);

	const CSparseMatrix  SM(A);

	const long N = 10;
	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		CSparseMatrix::CholeskyDecomp  CHOL(SM);
	}
	return tictac.Tac()/N;
}

double matrix_test_chol_Nx6x6_sparse(int DIM, int a2)
{
	CMatrixDouble  C(DIM*6,DIM*6);
	for (int i=0;i<DIM;i++)
	{
		CMatrixDouble subCov = randomGenerator.drawDefinitePositiveMatrix(6, 0.2);
		C.insertMatrix(i*6,i*6,subCov);
	}

	const CSparseMatrix  SM(C);

	const long N = 100;
	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		CSparseMatrix::CholeskyDecomp  CHOL(SM);
	}
	return tictac.Tac()/N;
}

// ------------------------------------------------------
// register_tests_matrices
// ------------------------------------------------------
void register_tests_matrices()
{
	randomGenerator.randomize(1234);

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

	lstTests.push_back( TestData("matrix: chol, dyn[double] 4x4",matrix_test_chol_dyn<double,4>) );
	lstTests.push_back( TestData("matrix: chol, fix[double] 4x4",matrix_test_chol_fix<double,4>) );
	lstTests.push_back( TestData("matrix: chol, dyn[double] 40x40",matrix_test_chol_dyn<double,40>) );
	lstTests.push_back( TestData("matrix: chol, fix[double] 40x40",matrix_test_chol_fix<double,40>) );

	lstTests.push_back( TestData("matrix: chol, sparse [2x2;2x2]",matrix_test_chol_sparse<2,2>) );
	lstTests.push_back( TestData("matrix: chol, sparse [30x30;10x10]",matrix_test_chol_sparse<30,10>) );

	lstTests.push_back( TestData("matrix: chol, dyn[double] 10x[6x6]",matrix_test_chol_Nx6x6_dyn,10) );
	lstTests.push_back( TestData("matrix: chol, sparse      10x[6x6]",matrix_test_chol_Nx6x6_sparse,10) );
	lstTests.push_back( TestData("matrix: chol, dyn[double] 20x[6x6]",matrix_test_chol_Nx6x6_dyn,20) );
	lstTests.push_back( TestData("matrix: chol, sparse      20x[6x6]",matrix_test_chol_Nx6x6_sparse,20) );
	lstTests.push_back( TestData("matrix: chol, dyn[double] 50x[6x6]",matrix_test_chol_Nx6x6_dyn,50) );
	lstTests.push_back( TestData("matrix: chol, sparse      50x[6x6]",matrix_test_chol_Nx6x6_sparse,50) );
	lstTests.push_back( TestData("matrix: chol, dyn[double] 100x[6x6]",matrix_test_chol_Nx6x6_dyn,100, 2) );
	lstTests.push_back( TestData("matrix: chol, sparse      100x[6x6]",matrix_test_chol_Nx6x6_sparse,100) );
	lstTests.push_back( TestData("matrix: chol, dyn[double] 120x[6x6]",matrix_test_chol_Nx6x6_dyn,120, 2) );
	lstTests.push_back( TestData("matrix: chol, sparse      120x[6x6]",matrix_test_chol_Nx6x6_sparse,120) );
	lstTests.push_back( TestData("matrix: chol, dyn[double] 140x[6x6]",matrix_test_chol_Nx6x6_dyn,140, 2) );
	lstTests.push_back( TestData("matrix: chol, sparse      140x[6x6]",matrix_test_chol_Nx6x6_sparse,140) );

}
