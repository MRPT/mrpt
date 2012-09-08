/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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

double matrix_test_loadFromArray(int N, int a2)
{
	EIGEN_ALIGN16 double nums[4*4] = {
	 0,1,2,3,
	 4,5,6,7,
	 8,9,10,11,
	 12,13,14,15 };

	CMatrixFixedNumeric<double,4,4> M;

	CTicTac	 tictac;
	M.loadFromArray(nums);
	return tictac.Tac();
}

double matrix_test_loadWithEigenMap(int N, int a2)
{
	EIGEN_ALIGN16 double nums[4*4] = {
	 0,1,2,3,
	 4,5,6,7,
	 8,9,10,11,
	 12,13,14,15 };

	CMatrixFixedNumeric<double,4,4> M;

	CTicTac	 tictac;
	M = Eigen::Map<CMatrixFixedNumeric<double,4,4>::Base,Eigen::Aligned >(nums);
	const double t= tictac.Tac();
	dummy_do_nothing_with_string(mrpt::format("%e",M(0,0)));
	return t;
}

// ------------------------------------------------------
// register_tests_matrices: Part 2
// ------------------------------------------------------
void register_tests_matrices2()
{
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

	lstTests.push_back( TestData("matrix: loadFromArray[double] 4x4",matrix_test_loadFromArray,1e7) );
	lstTests.push_back( TestData("matrix: load Eigen::Map[double] 4x4",matrix_test_loadWithEigenMap,1e7) );

}
