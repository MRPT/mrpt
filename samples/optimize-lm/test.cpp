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

#include <mrpt/utils.h>
#include <mrpt/math.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;


// The error function F(x):
void myFunction( const vector_double &x, const vector_double &y, vector_double &out_f)
{
	out_f.resize(1);

	// 1-cos(x+1) *cos(x*y+1)
	out_f[0] = 1 - cos(x[0]+1) * cos(x[0]*x[1]+1);
}

// ------------------------------------------------------
//				TestLM
// ------------------------------------------------------
void TestLM()
{
	vector_double		optimal_x;
	vector_double		initial_x;
	vector_double		increments_x;
	vector_double		y;

	CLevenbergMarquardt::TResultInfo	info;
	CTicTac	tictac;

	initial_x.resize(2);
	initial_x[0] = 1.4;	// x
	initial_x[1] = 2.5;	// y

	increments_x.resize(2, 0.0001 );

	double T;
	size_t  N = 1;

	tictac.Tic();
	for (size_t k=0;k<N;k++)
		CLevenbergMarquardt::execute(optimal_x, initial_x, myFunction, increments_x, y, info  );

	T = tictac.Tac() / N;

	cout << "Iterations: " << info.iterations_executed <<  endl;
	cout << "Final sqr error: " << info.final_sqr_err << endl;

	cout << endl << "Final optimized position: " << optimal_x << endl;

	cout << "Time: " << T*1e6 << " us" << endl;

	info.path.saveToTextFile("lm-path.txt");
	cout << "Path saved to 'lm-path.txt'" << endl;

}

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main()
{
	try
	{
		TestLM();

		return 0;
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Untyped exception!!");
		return -1;
	}
}
