/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/maps/CGasConcentrationGridMap2D.h>
#include <mrpt/system/filesystem.h>

const double xMin = -4.0, xMax = 4.0, yMin = -4.0, yMax = 4.0;
const double val = 0.2, sigma = 1.0;
const bool ti = true;  // time invariant
const bool um = false;	// update map. Manual call to updateMapEstimation()

static void test_CGasConcentrationGridMap2D_insertAndRead(
	mrpt::maps::CRandomFieldGridMap2D::TMapRepresentation mapType,
	const double resolution,
	const std::function<double(const mrpt::maps::TRandomFieldCell&)>&
		cellToValue)
{
	mrpt::maps::CGasConcentrationGridMap2D grid(
		mapType, xMin, xMax, yMin, yMax, resolution);

	// Inside:
	grid.insertIndividualReading(1.0 * val, {2.0, 3.0}, um, ti, sigma);
	grid.insertIndividualReading(2.0 * val, {1.0, 0.5}, um, ti, sigma);
	grid.insertIndividualReading(3.0 * val, {-1.0, -2.0}, um, ti, sigma);

	grid.insertionOptions.GMRF_skip_variance = true;
	grid.updateMapEstimation();

	for (int i = 0; i < 2; i++)
	{
		{
			const double map_value = cellToValue(*grid.cellByPos(2.0, 3.0));
			EXPECT_NEAR(map_value, 1.0 * val, 1e-2) << "mapType: " << mapType;
		}
		{
			const double map_value = cellToValue(*grid.cellByPos(1.0, 0.5));
			EXPECT_NEAR(map_value, 2.0 * val, 1e-2) << "mapType: " << mapType;
		}
		{
			const double map_value = cellToValue(*grid.cellByPos(-1.0, -2.0));
			EXPECT_NEAR(map_value, 3.0 * val, 1e-2) << "mapType: " << mapType;

			// Test after map enlarge:
			grid.resize(-5.0, 5.0, -5.0, 5.0, {}, .0);
		}
	}
}

TEST(CGasConcentrationGridMap2D, insertAndRead)
{
	test_CGasConcentrationGridMap2D_insertAndRead(
		mrpt::maps::CRandomFieldGridMap2D::mrKalmanFilter, 1.0,
		[](const mrpt::maps::TRandomFieldCell& c) { return c.kf_mean(); });
	test_CGasConcentrationGridMap2D_insertAndRead(
		mrpt::maps::CRandomFieldGridMap2D::mrKalmanApproximate, 1.0,
		[](const mrpt::maps::TRandomFieldCell& c) { return c.kf_mean(); });
	test_CGasConcentrationGridMap2D_insertAndRead(
		mrpt::maps::CRandomFieldGridMap2D::mrGMRF_SD, 0.5,
		[](const mrpt::maps::TRandomFieldCell& c) { return c.gmrf_mean(); });

#if 0
	test_CGasConcentrationGridMap2D_insertAndRead(
		mrpt::maps::CRandomFieldGridMap2D::mrKernelDM, 0.1,
		[](const mrpt::maps::TRandomFieldCell& c) { return c.dm_mean(); });
	test_CGasConcentrationGridMap2D_insertAndRead(
		mrpt::maps::CRandomFieldGridMap2D::mrKernelDMV, 0.1,
		[](const mrpt::maps::TRandomFieldCell& c) { return c.dm_mean(); });
#endif
}
