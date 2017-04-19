/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/random.h>
#include <mrpt/utils/round.h>
#include <mrpt/utils/bits.h>

#include "common.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace std;


// ------------------------------------------------------
//				Benchmark Misc. Math
// ------------------------------------------------------
double math_test_round(int a1, int a2)
{
	const long N = 100000000;
	CTicTac	 tictac;

	int a;
	double b = 2.3;
	for (long i=0;i<N;i++)
	{
		a=mrpt::utils::round(b);
	}
	double T = tictac.Tac()/N;
	dummy_do_nothing_with_string( mrpt::format("%i",a) );
	return T;
}

template <typename T, typename FUNC>
double math_test_FUNC(int a1, int a2, FUNC func)
{
	const long N = 100000000;
	CTicTac	 tictac;

	T x = 1.0, y = 2.0, r = .0;
	tictac.Tic();
	for (long i = 0; i<N; i++)
	{
		r = func(x, y);
	}
	double t = tictac.Tac() / N;
	dummy_do_nothing_with_string(mrpt::format("%f", r));
	return t;
}

// ------------------------------------------------------
// register_tests_math
// ------------------------------------------------------
void register_tests_math()
{
	randomGenerator.randomize(1234);

	lstTests.push_back( TestData("math: round",math_test_round ) );

	using namespace std::placeholders;
	lstTests.push_back(TestData("math: std::hypot(float)", std::bind(math_test_FUNC<float, decltype(std::hypotf)>, _1, _2, std::hypotf)));
	lstTests.push_back(TestData("math: mrpt::math::hypot_fast(float)", std::bind(math_test_FUNC<float, decltype(mrpt::math::hypot_fast<float>)>, _1, _2, mrpt::math::hypot_fast<float>)));
	lstTests.push_back(TestData("math: mrpt::math::hypot_fast(double)", std::bind(math_test_FUNC<double, decltype(mrpt::math::hypot_fast<double>)>, _1, _2, mrpt::math::hypot_fast<double>)));
}
