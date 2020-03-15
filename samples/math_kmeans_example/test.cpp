/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/math/CVectorFixed.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/kmeans.h>
#include <mrpt/random.h>
#include <mrpt/system/CTicTac.h>
#include <iostream>
#include <vector>

using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::random;
using namespace mrpt::system;
using namespace std;

// ------------------------------------------------------
//				TestKMeans
// ------------------------------------------------------
void TestKMeans()
{
	typedef CVectorFixedDouble<2> CPointType;
	// typedef CVectorFixedFloat<2>  CPointType;

	getRandomGenerator().randomize();
	CTicTac tictac;

	CDisplayWindowPlots win("k-means results");

	cout << "Close the window to end.\n";

	while (win.isOpen())
	{
		// Generate N clusters of random points:
		std::vector<CPointType> points;
		const size_t nClusters =
			2 + (getRandomGenerator().drawUniform32bit() % 4);

		for (size_t cl = 0; cl < nClusters; cl++)
		{
			const size_t nPts = getRandomGenerator().drawUniform<size_t>(5, 50);

			TPoint2D clCenter;
			clCenter.x = getRandomGenerator().drawUniform(0, 10);
			clCenter.y = getRandomGenerator().drawUniform(0, 10);

			for (size_t p = 0; p < nPts; p++)
			{
				CPointType v;
				v[0] = clCenter.x + getRandomGenerator().drawGaussian1D(0, 1);
				v[1] = clCenter.y + getRandomGenerator().drawGaussian1D(0, 1);
				points.push_back(v);
			}
		}

		// do k-means
		std::vector<CPointType> centers;
		vector<int> assignments;
		tictac.Tic();

		const double cost =
			mrpt::math::kmeanspp(nClusters, points, assignments, &centers);

		cout << "Took: " << tictac.Tac() * 1e3 << " ms.\n";
		cout << "cost: " << cost << endl;

		// Show:
		win.clf();
		win.hold_on();
		static const char colors[6] = {'b', 'r', 'k', 'g', 'm', 'c'};

		for (size_t c = 0; c < nClusters; c++)
		{
			CVectorDouble xs, ys;

			for (size_t i = 0; i < points.size(); i++)
			{
				if (size_t(assignments[i]) == c)
				{
					xs.push_back(points[i][0]);
					ys.push_back(points[i][1]);
				}
			}
			win.plot(xs, ys, mrpt::format(".4%c", colors[c % 6]));
		}
		win.axis_fit();
		win.axis_equal();

		cout << "Press any key to generate another random dataset...\n";
		win.waitForKey();
	};
}

int main(int argc, char** argv)
{
	try
	{
		TestKMeans();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}
}
