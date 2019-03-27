/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/math/CLevenbergMarquardt.h>
#include <mrpt/system/CTicTac.h>
#include <cmath>
#include <iostream>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;

double levmarq_final_error = 1e10;

// The error function F(x):
void myFunction(
	const CVectorDouble& x, const CVectorDouble& y, CVectorDouble& out_f)
{
	out_f.resize(1);
	// 1-cos(x+1) *cos(x*y+1)
	out_f[0] = 1 - cos(x[0] + 1) * cos(x[0] * x[1] + 1);
}

void TestLevMarq()
{
	CVectorDouble optimal_x;
	CVectorDouble initial_x;
	CVectorDouble y;

	CLevenbergMarquardt::TResultInfo info;
	CTicTac tictac;

	initial_x.resize(2);
	initial_x[0] = 1.4;  // x
	initial_x[1] = 2.5;  // y

	CVectorDouble increments_x(2);
	increments_x.fill(0.0001);

	size_t N = 1;

	CLevenbergMarquardt lm;
	tictac.Tic();
	for (size_t k = 0; k < N; k++)
		lm.execute(optimal_x, initial_x, myFunction, increments_x, y, info);

	levmarq_final_error = std::sqrt(info.final_sqr_err);

#ifdef LEVMARQ_EXAMPLE_VERBOSE
	const auto T = tictac.Tac() / N;
	cout << "Iterations: " << info.iterations_executed << endl;
	cout << "Final sqr error: " << info.final_sqr_err << endl;

	cout << endl << "Final optimized position: " << optimal_x << endl;

	cout << "Time: " << T * 1e6 << " us" << endl;

	info.path.saveToTextFile("lm-path.txt");
	cout << "Path saved to 'lm-path.txt'" << endl;
#endif
}
