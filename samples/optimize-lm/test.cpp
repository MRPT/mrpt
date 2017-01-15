/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils.h>
#include <mrpt/math.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;


// The error function F(x):
void myFunction( const CVectorDouble &x, const CVectorDouble &y, CVectorDouble &out_f)
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
	CVectorDouble		optimal_x;
	CVectorDouble		initial_x;
	CVectorDouble		increments_x;
	CVectorDouble		y;

	CLevenbergMarquardt::TResultInfo	info;
	CTicTac	tictac;

	initial_x.resize(2);
	initial_x[0] = 1.4;	// x
	initial_x[1] = 2.5;	// y

	increments_x.resize(2);
	increments_x.setConstant(0.0001);

	double T;
	size_t  N = 1;

	CLevenbergMarquardt lm;
	tictac.Tic();
	for (size_t k=0;k<N;k++)
		lm.execute(optimal_x, initial_x, myFunction, increments_x, y, info  );

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
