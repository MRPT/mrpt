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
