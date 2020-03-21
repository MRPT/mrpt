/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/maps/CRandomFieldGridMap3D.h>
#include <mrpt/system/filesystem.h>

TEST(CRandomFieldGridMap3D, insertCheckMapBounds)
{
	using mrpt::math::TPoint3D;

	mrpt::maps::CRandomFieldGridMap3D::TVoxelInterpolationMethod im =
		mrpt::maps::CRandomFieldGridMap3D::gimNearest;

	mrpt::maps::CRandomFieldGridMap3D grid3d;
	// grid3d.setMinLoggingLevel(mrpt::system::LVL_DEBUG);

	grid3d.setSize(
		-4.0, 4.0, 0.0, 4.0, 0.0, 4.0,
		1.0 /*voxel size*/);  // x:[-10,10] * y:[0,5] * z:[0,4]

	const double val = 10.0, var = 1.0;

	// Inside:
	EXPECT_TRUE(grid3d.insertIndividualReading(
		1.0 * val, var, TPoint3D(2.0, 3.0, 1.0), im, false));
	EXPECT_TRUE(grid3d.insertIndividualReading(
		2.0 * val, var, TPoint3D(-3.0, 0.4, 1.0), im, false));
	EXPECT_TRUE(grid3d.insertIndividualReading(
		3.0 * val, var, TPoint3D(3.0, 3.8, 3.0), im, false));
	// Outside:
	EXPECT_FALSE(grid3d.insertIndividualReading(
		val, var, TPoint3D(-11.0, 2.0, 2.0), im, false));
	EXPECT_FALSE(grid3d.insertIndividualReading(
		val, var, TPoint3D(11.0, 2.0, 3.0), im, false));
	EXPECT_FALSE(grid3d.insertIndividualReading(
		val, var, TPoint3D(2.0, -1.0, 11.0), im, false));
	EXPECT_FALSE(grid3d.insertIndividualReading(
		val, var, TPoint3D(2.0, 6.0, 3.0), im, false));

	grid3d.updateMapEstimation();
	grid3d.saveAsCSV(mrpt::system::getTempFileName());
}

TEST(CRandomFieldGridMap3D, insertPointsAndRead)
{
	using mrpt::math::TPoint3D;

	mrpt::maps::CRandomFieldGridMap3D::TVoxelInterpolationMethod im =
		mrpt::maps::CRandomFieldGridMap3D::gimNearest;
	mrpt::maps::CRandomFieldGridMap3D grid3d;
	// grid3d.setMinLoggingLevel(mrpt::system::LVL_DEBUG);

	grid3d.setSize(
		-4.0, 4.0, 0.0, 4.0, 0.0, 4.0,
		1.0 /*voxel size*/);  // x:[-10,10] * y:[0,5] * z:[0,4]

	const double val = 55.0, var = 1.0;

	EXPECT_TRUE(grid3d.insertIndividualReading(
		val, var, TPoint3D(2.0, 3.0, 1.0), im, false));

	grid3d.insertionOptions.GMRF_skip_variance = true;
	grid3d.updateMapEstimation();

	{
		const double map_value = grid3d.cellByPos(2.0, 3.0, 1.0)->mean_value;
		EXPECT_NEAR(map_value, val, 1e-6);
	}

	// Test after map enlarge:
	grid3d.resize(
		-5.0, 5.0, -1.0, 5.0, -1.0, 5.0, mrpt::maps::TRandomFieldVoxel(), .0);
	{
		const double map_value = grid3d.cellByPos(2.0, 3.0, 1.0)->mean_value;
		EXPECT_NEAR(map_value, val, 1e-6);
	}
}
