/* +---------------------------------------------------------------------------+
|                     Mobile Robot Programming Toolkit (MRPT)               |
|                          http://www.mrpt.org/                             |
|                                                                           |
| Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
| See: http://www.mrpt.org/Authors - All rights reserved.                   |
| Released under BSD License. See details in http://www.mrpt.org/License    |
+---------------------------------------------------------------------------+ */

#include <mrpt/maps/CRandomFieldGridMap3D.h>
#include <gtest/gtest.h>

TEST(CRandomFieldGridMap3D, insertCheckMapBounds)
{
	mrpt::maps::CRandomFieldGridMap3D::TVoxelInterpolationMethod im = mrpt::maps::CRandomFieldGridMap3D::gimNearest;

	mrpt::maps::CRandomFieldGridMap3D grid3d; //mrpt::maps::CHeightGridMap2D grid3d;
	grid3d.setSize(-4.0, 4.0, 0.0, 4.0, 1.0); // x:[-10,10] * y:[0,5]
	// Inside:
	EXPECT_TRUE(grid3d.insertIndividualReading(2.0, 3.0, 56.0, pt_params));
	EXPECT_TRUE(grid3d.insertIndividualReading(-3.0, 0.4, 56.0, pt_params));
	EXPECT_TRUE(grid3d.insertIndividualReading(3.0, 3.8, 56.0, pt_params));
	// Outside:
	EXPECT_FALSE(grid3d.insertIndividualReading(-11.0, 2.0, 56.0, pt_params));
	EXPECT_FALSE(grid3d.insertIndividualReading(11.0, 2.0, 56.0, pt_params));
	EXPECT_FALSE(grid3d.insertIndividualReading(2.0, -1.0, 56.0, pt_params));
	EXPECT_FALSE(grid3d.insertIndividualReading(2.0, 6.0, 56.0, pt_params));
}

TEST(CRandomFieldGridMap3D, insertPointsAndRead)
{
#if 0
	MAP grid3d;
	grid3d.setSize(0.0, 5.0, 0.0, 5.0, 0.5); // x:[-10,10] * y:[0,5]
											 // Inside:
	const double x = 2.1, y = 3.1, z_write = 56.0;
	grid3d.insertIndividualPoint(x, y, z_write);
	double z_read;
	bool res = grid3d.dem_get_z(x, y, z_read);
	EXPECT_TRUE(res);
	EXPECT_NEAR(z_read, z_write, 1e-6);
#endif
}

