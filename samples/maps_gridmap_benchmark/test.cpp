/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/config/CConfigFile.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/random.h>
#include <mrpt/system/CTicTac.h>
#include <iostream>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::random;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace std;

#include <mrpt/examples_config.h>
string myDataDir(
	MRPT_EXAMPLES_BASE_DIRECTORY + string("maps_gridmap_benchmark/"));

// Default .ini file:
string iniFile(myDataDir + string("benchmark-options.ini"));

// ------------------------------------------------------
//				BenchmarkGridmaps
// ------------------------------------------------------
void BenchmarkGridmaps()
{
	getRandomGenerator().randomize(333);

	CMultiMetricMap metricMap;
	TSetOfMetricMapInitializers mapInit;

	// Create gridmap:
	mapInit.loadFromConfigFile(
		mrpt::config::CConfigFile(iniFile), "METRIC_MAPS");
	metricMap.setListOfMaps(mapInit);

	// prepare the laser scan:
	CObservation2DRangeScan scan1;
	stock_observations::example2DRangeScan(scan1);

	COccupancyGridMap2D::Ptr gridMap =
		metricMap.mapByClass<COccupancyGridMap2D>();
	ASSERT_(gridMap);
	COccupancyGridMap2D gridMapCopy(*gridMap);

	int i, N;
	CTicTac tictac;

	// test 1: getcell
	// ----------------------------------------
	if (true)
	{
		N = 10000000;

		cout << "Running test #1: getCell... ";
		cout.flush();

		// COccupancyGridMap2D::cellType	cell;
		float p = 0;

		tictac.Tic();
		for (i = 0; i < N; i++)
		{
			p += gridMap->getCell(0, 0);
		}
		double T = tictac.Tac();
		cout << "-> " << 1e9 * T / N << " ns/iter. p=" << p
			 << endl;  // the "p" is to avoid optimizing out the entire loop!
	}

	// test 2: setcell
	// ----------------------------------------
	if (true)
	{
		N = 10000000;

		cout << "Running test #2: setCell... ";
		cout.flush();

		float p = 0.8f;

		tictac.Tic();
		for (i = 0; i < N; i++)
		{
			gridMap->setCell(0, 0, p);
		}
		double T = tictac.Tac();
		cout << "-> " << 1e9 * T / N << " ns/iter."
			 << endl;  // the "p" is to avoid optimizing out the entire loop!
	}

	// test 3: updateCell
	// ----------------------------------------
	if (true)
	{
		N = 1000000;

		cout << "Running test #3: updateCell... ";
		cout.flush();

		float p = 0.57f;

		tictac.Tic();
		for (i = 0; i < N; i++)
		{
			gridMap->updateCell(0, 0, p);
		}
		double T = tictac.Tac();
		cout << "-> " << 1e9 * T / N << " ns/iter."
			 << endl;  // the "p" is to avoid optimizing out the entire loop!
	}

	// test 4: updateCell_fast
	// ----------------------------------------
	if (true)
	{
		N = 10000000;

		cout << "Running test #4: updateCell_fast... ";
		cout.flush();

		float p = 0.57f;
		COccupancyGridMap2D::cellType logodd_obs = COccupancyGridMap2D::p2l(p);
		// float   p_1 = 1-p;

		COccupancyGridMap2D::cellType* theMapArray = gridMap->getRow(0);
		unsigned theMapSize_x = gridMap->getSizeX();
		COccupancyGridMap2D::cellType logodd_thres_occupied =
			COccupancyGridMap2D::OCCGRID_CELLTYPE_MIN + logodd_obs;

		tictac.Tic();
		for (i = 0; i < N; i++)
		{
			COccupancyGridMap2D::updateCell_fast_occupied(
				2, 2, logodd_obs, logodd_thres_occupied, theMapArray,
				theMapSize_x);
		}
		double T = tictac.Tac();
		cout << "-> " << 1e9 * T / N << " ns/iter."
			 << endl;  // the "p" is to avoid optimizing out the entire loop!
	}

#if 0
	for (i=50;i<51;i++)
	{
		CPose3D  pose3D(0.21,0.34,0,-2);
		//scan1.validRange.assign(scan1.validRange.size(), false);
		//scan1.getScanRangeValidity(i)=true;

		gridMap->clear();
		gridMap->resizeGrid(-5,20,-15,15);
		gridMap->insertObservation( &scan1, &pose3D );
		gridMap->saveAsBitmapFile(format("./gridmap_with_widening_%04i.png",i));
	}
#endif

	// test 5: Laser insertion
	// ----------------------------------------
	if (true)
	{
		gridMap->insertionOptions.wideningBeamsWithDistance = false;
		N = 3000;
		cout << "Running test #5: Laser insert. w/o widen... ";
		cout.flush();
		tictac.Tic();
		for (i = 0; i < N; i++)
		{
#if 1
			CPose2D pose(
				getRandomGenerator().drawUniform(-1.0, 1.0),
				getRandomGenerator().drawUniform(-1.0, 1.0),
				getRandomGenerator().drawUniform(-M_PI, M_PI));
			CPose3D pose3D(pose);
#else
			CPose3D pose3D;
#endif

			gridMap->insertObservation(scan1, &pose3D);
		}
		double T = tictac.Tac();
		cout << "-> " << 1000 * T / N << " ms/iter, scans/sec:" << N / T
			 << endl;

		CPose3D pose3D;
		gridMap->clear();
		gridMap->insertObservation(scan1, &pose3D);
		gridMap->saveAsBitmapFile("./gridmap_without_widening.png");
	}

	// test 6: Laser insertion without widening
	// --------------------------------------------------
	if (true)
	{
		gridMap->insertionOptions.wideningBeamsWithDistance = true;
		N = 3000;
		cout << "Running test #6: Laser insert. widen... ";
		cout.flush();
		tictac.Tic();
		for (i = 0; i < N; i++)
		{
#if 1
			CPose2D pose(
				getRandomGenerator().drawUniform(-1.0, 1.0),
				getRandomGenerator().drawUniform(-1.0, 1.0),
				getRandomGenerator().drawUniform(-M_PI, M_PI));
			CPose3D pose3D(pose);
#else
			CPose3D pose3D;
#endif
			gridMap->insertObservation(scan1, &pose3D);
		}
		double T = tictac.Tac();
		cout << "-> " << 1000 * T / N << " ms/iter, scans/sec:" << N / T
			 << endl;

		CPose3D pose3D;
		gridMap->clear();
		gridMap->insertObservation(scan1, &pose3D);
		gridMap->saveAsBitmapFile("./gridmap_with_widening.png");
	}

	// test 7: Grid resize
	// ----------------------------------------
	if (true)
	{
		N = 400;
		cout << "Running test #7: Grid resize... ";
		cout.flush();
		tictac.Tic();
		for (i = 0; i < N; i++)
		{
			*gridMap = gridMapCopy;
			gridMap->resizeGrid(-30, 30, -40, 40);
		}
		double T = tictac.Tac();
		cout << "-> " << 1000 * T / N << " ms/iter" << endl;
	}

	// test 8: Likelihood computation
	// ----------------------------------------
	if (true)
	{
		N = 5000;

		*gridMap = gridMapCopy;
		CPose3D pose3D(0, 0, 0);
		gridMap->insertObservation(scan1, &pose3D);

		cout << "Running test #8: Likelihood... ";
		cout.flush();
		double R = 0;
		tictac.Tic();
		for (i = 0; i < N; i++)
		{
			CPose2D pose(
				getRandomGenerator().drawUniform(-1.0, 1.0),
				getRandomGenerator().drawUniform(-1.0, 1.0),
				getRandomGenerator().drawUniform(-M_PI, M_PI));
			R += gridMap->computeObservationLikelihood(scan1, pose);
		}
		double T = tictac.Tac();
		cout << "-> " << 1000 * T / N << " ms/iter" << endl;
	}
}

int main(int argc, char** argv)
{
	try
	{
		// optional argument: a different ini file
		if (argc > 1) iniFile = string(argv[1]);

		BenchmarkGridmaps();
		return 0;
	}
	catch (exception& e)
	{
		cout << "MRPT exception caught: " << e.what() << endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}
}
