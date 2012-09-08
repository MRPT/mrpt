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

#include <mrpt/base.h>
#include <mrpt/gui.h>

using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::random;
using namespace std;


// ------------------------------------------------------
//				TestKMeans
// ------------------------------------------------------
void TestKMeans()
{
	typedef CArrayDouble<2>  CPointType;
	//typedef CArrayFloat<2>  CPointType;

	randomGenerator.randomize();
	CTicTac tictac;

	CDisplayWindowPlots	win("k-means results");

	cout << "Close the window to end.\n";

	while (win.isOpen())
	{
		// Generate N clusters of random points:
		mrpt::aligned_containers<CPointType>::vector_t  points;
		const size_t nClusters = 2 + (randomGenerator.drawUniform32bit() % 4);

		for (size_t cl=0;cl<nClusters;cl++)
		{
			const size_t nPts = randomGenerator.drawUniform(5,50);

			TPoint2D  clCenter;
			clCenter.x = randomGenerator.drawUniform(0,10);
			clCenter.y = randomGenerator.drawUniform(0,10);

			for (size_t p=0;p<nPts;p++)
			{
				CPointType v;
				v[0] = clCenter.x + randomGenerator.drawGaussian1D(0,1);
				v[1] = clCenter.y + randomGenerator.drawGaussian1D(0,1);
				points.push_back(v);
			}
		}

		// do k-means
		mrpt::aligned_containers<CPointType>::vector_t	centers;
		vector<int>				assignments;
		tictac.Tic();

		const double cost = mrpt::math::kmeanspp(nClusters, points, assignments, &centers);

		cout << "Took: " << tictac.Tac()*1e3 << " ms.\n";
		cout << "cost: " << cost << endl;

		// Show:
		win.clf();
		win.hold_on();
		static const char colors[6] = {'b','r','k','g','m','c'};

		for (size_t c=0;c<nClusters;c++)
		{
			vector_double xs,ys;

			for (size_t i=0;i<points.size();i++)
			{
				if (size_t(assignments[i])==c)
				{
					xs.push_back( points[i][0] );
					ys.push_back( points[i][1] );
				}
			}
			win.plot(xs,ys,format(".4%c",colors[c%6]));
		}
		win.axis_fit();
		win.axis_equal();

		cout << "Press any key to generate another random dataset...\n";
		win.waitForKey();
	};


}

int main(int argc, char **argv)
{
	try
	{
		TestKMeans();
		return 0;
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}

}
