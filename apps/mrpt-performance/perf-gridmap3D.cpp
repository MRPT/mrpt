/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/maps/COccupancyGridMap3D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random.h>

#include "common.h"

double grid3d_test_getcell(int a1, int a2)
{
	mrpt::maps::COccupancyGridMap3D gridMap(
		mrpt::math::TPoint3D(-3.0, -5.0, -2.0),
		mrpt::math::TPoint3D(10.0, 5.0, 2.0), 0.10f);

	const long N = 10000000;
	float p = 0;

	CTicTac tictac;
	for (long i = 0; i < N; i++)
	{
		p += gridMap.getCellFreeness(1, 2, 3);
	}
	return tictac.Tac() / N;
}

double grid3d_test_updateCell(int a1, int a2)
{
	mrpt::maps::COccupancyGridMap3D gridMap(
		mrpt::math::TPoint3D(-3.0, -5.0, -2.0),
		mrpt::math::TPoint3D(10.0, 5.0, 2.0), 0.10f);

	const long N = 1000000;
	float p = 0.57f;

	CTicTac tictac;
	for (long i = 0; i < N; i++)
	{
		gridMap.updateCell(1, 2, 3, p);
	}
	return tictac.Tac() / N;
}

double grid3d_test_insert2DScan(int res_cm, int a2)
{
	auto& rn = mrpt::random::getRandomGenerator();
	rn.randomize(333);

	// prepare the laser scan:
	mrpt::obs::CObservation2DRangeScan scan1;
	mrpt::obs::stock_observations::example2DRangeScan(scan1);

	mrpt::maps::COccupancyGridMap3D gridmap(
		mrpt::math::TPoint3D(-3.0, -5.0, -2.0),
		mrpt::math::TPoint3D(10.0, 5.0, 2.0), 0.01f * res_cm);

	const long N = 1000;
	CTicTac tictac;
	for (long i = 0; i < N; i++)
	{
		mrpt::poses::CPose2D pose(
			rn.drawUniform(-1.0, 1.0), rn.drawUniform(-1.0, 1.0),
			rn.drawUniform(-M_PI, M_PI));
		mrpt::poses::CPose3D pose3D(pose);

		gridmap.insertObservation(&scan1, &pose3D);
	}
	return tictac.Tac() / N;
}

double grid3d_resize(int a1, int a2)
{
	mrpt::maps::COccupancyGridMap3D gridmap(
		mrpt::math::TPoint3D(-20.0, -20.0, -2.0),
		mrpt::math::TPoint3D(20.0, 20.0, 2.0), 0.20f);

	CTicTac tictac;
	gridmap.resizeGrid(
		mrpt::math::TPoint3D(-30.0, -30.0, -3.0),
		mrpt::math::TPoint3D(40.0, 40.0, 3.0));
	return tictac.Tac();
}

void register_tests_grid3D()
{
	lstTests.emplace_back("gridmap3D: getCell", grid3d_test_getcell);
	lstTests.emplace_back("gridmap3D: updateCell", grid3d_test_updateCell);
	lstTests.emplace_back("gridmap3D: resize", grid3d_resize);

	lstTests.emplace_back(
		"gridmap3D: insert 2Dscan (voxels=5cm)", grid3d_test_insert2DScan, 5);
	lstTests.emplace_back(
		"gridmap3D: insert 2Dscan (voxels=10cm)", grid3d_test_insert2DScan, 10);
	lstTests.emplace_back(
		"gridmap3D: insert 2Dscan (voxels=15cm)", grid3d_test_insert2DScan, 15);
	lstTests.emplace_back(
		"gridmap3D: insert 2Dscan (voxels=20cm)", grid3d_test_insert2DScan, 20);
}
