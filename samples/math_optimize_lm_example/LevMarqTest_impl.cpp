/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/math/CLevenbergMarquardt.h>
#include <mrpt/system/CTicTac.h>

#include <cmath>

#ifdef LEVMARQ_EXAMPLE_VERBOSE
#include <iostream>
#endif

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;

double levmarq_final_error = 1e10;

// The error function F(x):
void myFunction(const CVectorDouble& x, const CVectorDouble& y, CVectorDouble& out_f)  // NOLINT
{
  // The function to be minimized is:
  // 1-cos(x+1) *cos(x*y+1)
  // with x[0] = x, x[1] = y
  // and the output is a vector of size 1.
  // The function is defined in the range [-2,2] for both x and y.

  ASSERT_EQUAL_(x.size(), 2);
  ASSERT_EQUAL_(y.size(), 0);
  {
    out_f.resize(1);
    // 1-cos(x+1) *cos(x*y+1)
    out_f[0] = 1 - cos(x[0] + 1) * cos(x[0] * x[1] + 1);
  }
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
  {
    lm.execute(optimal_x, initial_x, myFunction, increments_x, y, info);
  }

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
