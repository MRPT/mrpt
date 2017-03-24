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

// ------------------------------------------------------
// register_tests_math
// ------------------------------------------------------
void register_tests_math()
{
	randomGenerator.randomize(1234);

	lstTests.push_back( TestData("math: round",math_test_round ) );

}
