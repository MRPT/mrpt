/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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


#include <mrpt/math.h>
#include <mrpt/utils/CTicTac.h>

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
		a=mrpt::math::round(b);
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
