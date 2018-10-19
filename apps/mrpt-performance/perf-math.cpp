/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/random.h>
#include <mrpt/core/round.h>

#include "common.h"

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace std;

// ------------------------------------------------------
//				Benchmark Misc. Math
// ------------------------------------------------------
double math_test_round(int a1, int a2)
{
	const long N = 100000000;
	CTicTac tictac;

	int a;
	double b = 2.3;
	for (long i = 0; i < N; i++)
	{
		a = mrpt::round(b);
	}
	double T = tictac.Tac() / N;
	dummy_do_nothing_with_string(mrpt::format("%i", a));
	return T;
}

template <typename T, typename FUNC>
double math_test_FUNC(int a1, int a2, FUNC func)
{
	const long N = 100000000;
	CTicTac tictac;

	T x = 1.0, y = 2.0, r = .0;
	tictac.Tic();
	for (long i = 0; i < N; i++)
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
	getRandomGenerator().randomize(1234);

	lstTests.emplace_back("math: round", math_test_round);

	using namespace std::placeholders;
	lstTests.emplace_back(
		"math: std::hypot(float)", [=](auto&& arg1, auto&& arg2) {
			return math_test_FUNC<float, decltype(std::hypotf)>(
				arg1, arg2, std::hypotf);
		});
	lstTests.emplace_back(
		"math: mrpt::hypot_fast(float)", [=](auto&& arg1, auto&& arg2) {
			return math_test_FUNC<float, decltype(mrpt::hypot_fast<float>)>(
				arg1, arg2, mrpt::hypot_fast<float>);
		});
	lstTests.emplace_back(
		"math: mrpt::hypot_fast(double)", [=](auto&& arg1, auto&& arg2) {
			return math_test_FUNC<double, decltype(mrpt::hypot_fast<double>)>(
				arg1, arg2, mrpt::hypot_fast<double>);
		});
}
