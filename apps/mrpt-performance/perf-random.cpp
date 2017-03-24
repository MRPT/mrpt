/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/random.h>

#include "common.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::random;
using namespace mrpt::math;
using namespace std;


// ------------------------------------------------------
//				Benchmark Random Generators
// ------------------------------------------------------
double random_test_1(int a1, int a2)
{
	CRandomGenerator  rg;

	// test 1: draw uint32
	// ----------------------------------------
	const long N = 100000000;
	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		rg.drawUniform32bit();
	}
	return tictac.Tac()/N;
}
double random_test_2(int a1, int a2)
{
	CRandomGenerator  rg;

	// test 2: drawUniform
	// ----------------------------------------
	const long N = 100000000;
	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		rg.drawUniform(0,1);
	}
	return tictac.Tac()/N;
}
double random_test_3(int a1, int a2)
{
	CRandomGenerator  rg;

	// test 3: drawGaussian1D_normalized
	// ----------------------------------------
	const long N = 10000000;
	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		rg.drawGaussian1D_normalized();
	}
	return tictac.Tac()/N;
}
double random_test_4(int a1, int a2)
{
	CRandomGenerator  rg;

	// test 4: drawGaussian1D
	// ----------------------------------------
	const long N = 10000000;
	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		rg.drawGaussian1D(5.0,3.0);
	}
	return tictac.Tac()/N;
}
double random_test_5(int a1, int a2)
{
	CRandomGenerator  rg;

	// test 5: system rand()
	// ----------------------------------------
	const long N = 10000000;
	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		rand();
	}
	return tictac.Tac()/N;
}
double random_test_6(int a1, int a2)
{
	CRandomGenerator  rg;
	double COVs[] = { 6.0, -5.0, 2.0, -5.0, 3.0, 1.0, 2.0, 1.0, 7.0};

	CMatrixDouble33 COV(COVs);

	// test 6:
	// ----------------------------------------
	const long N = 100000;
	CTicTac	 tictac;
	CVectorDouble res;
	for (long i=0;i<N;i++)
	{
		rg.drawGaussianMultivariate(res,COV);
	}
	return tictac.Tac()/N;
}
double random_test_7(int a1, int a2)
{
	CRandomGenerator  rg;
	double COVs[] = { 6.0, -5.0, 2.0, -5.0, 3.0, 1.0, 2.0, 1.0, 7.0};

	CMatrixDouble COV(3,3,COVs);

	// test 7:
	// ----------------------------------------
	const long N = 100000;
	CTicTac	 tictac;
	CVectorDouble res;
	for (long i=0;i<N;i++)
	{
		rg.drawGaussianMultivariate(res,COV);
	}
	return tictac.Tac()/N;
}

template <size_t DIM>
double random_test_8(int a1, int a2)
{
	CRandomGenerator  rg;

	CMatrixFixedNumeric<double,DIM,DIM> R;
	rg.drawGaussian1DMatrix(R,0.0,1.0);

	CMatrixFixedNumeric<double,DIM,DIM> COV;
	COV.multiply_AAt(R);

	const size_t NSAMPS = 1000;

	// test 8:
	// ----------------------------------------
	const long N = 1000;
	CTicTac	 tictac;
	std::vector<CVectorDouble> res;
	for (long i=0;i<N;i++)
	{
		rg.drawGaussianMultivariateMany(res,NSAMPS,COV);
	}
	return tictac.Tac()/(N*NSAMPS);
}

double random_test_9(int a1, int a2)
{
	CRandomGenerator  rg;

	CMatrixTemplateNumeric<double> R(a1,a1);
	rg.drawGaussian1DMatrix(R,0.0,1.0);

	CMatrixTemplateNumeric<double> COV;
	COV.multiply_AAt(R);
	const size_t NSAMPS = 1000;

	// test 9:
	// ----------------------------------------
	const long N = 1000;
	CTicTac	 tictac;
	std::vector<CVectorDouble> res;
	for (long i=0;i<N;i++)
	{
		rg.drawGaussianMultivariateMany(res,NSAMPS,COV);
	}
	return tictac.Tac()/(N*NSAMPS);
}

double random_test_10(int a1, int a2)
{
	CRandomGenerator  rg;

	CVectorDouble vec(a1);
	rg.drawUniformVector(vec,0.0,1.0);

	// test 10: permute
	// ----------------------------------------
	const long N = 10000;
	CVectorDouble vec2;
	CTicTac	 tictac;
	for (long i=0;i<N;i++)
	{
		rg.permuteVector(vec,vec2);
	}
	return tictac.Tac()/N;
}

// ------------------------------------------------------
// register_tests_random
// ------------------------------------------------------
void register_tests_random()
{
	lstTests.push_back( TestData("random: drawUniform32bit",random_test_1) );
	lstTests.push_back( TestData("random: drawUniform",random_test_2) );
	lstTests.push_back( TestData("random: drawGaussian1D_normalized",random_test_3) );
	lstTests.push_back( TestData("random: drawGaussian1D",random_test_4) );
	lstTests.push_back( TestData("random: system rand()",random_test_5) );

	lstTests.push_back( TestData("random: drawGaussianMultivariate(fixed 3x3)",random_test_6) );
	lstTests.push_back( TestData("random: drawGaussianMultivariate(dyn 3x3)",random_test_7) );

	lstTests.push_back( TestData("random: drawGaussianMultivariateMany(fixed 2x2, 1000)",random_test_8<2>) );
	lstTests.push_back( TestData("random: drawGaussianMultivariateMany(fixed 3x3, 1000)",random_test_8<3>) );
	lstTests.push_back( TestData("random: drawGaussianMultivariateMany(fixed 6x6, 1000)",random_test_8<6>) );
	lstTests.push_back( TestData("random: drawGaussianMultivariateMany(dyn 2x2, 1000)",random_test_9,2) );
	lstTests.push_back( TestData("random: drawGaussianMultivariateMany(dyn 3x3, 1000)",random_test_9,3) );
	lstTests.push_back( TestData("random: drawGaussianMultivariateMany(dyn 6x6, 1000)",random_test_9,6) );

	lstTests.push_back( TestData("random: permuteVector (len=10)",random_test_10,10) );
	lstTests.push_back( TestData("random: permuteVector (len=100)",random_test_10,100) );
	lstTests.push_back( TestData("random: permuteVector (len=1000)",random_test_10,1000) );
}

