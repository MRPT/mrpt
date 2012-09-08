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
