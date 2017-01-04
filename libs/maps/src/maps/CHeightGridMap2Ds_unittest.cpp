/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/maps/CHeightGridMap2D_MRF.h>
#include <mrpt/maps/CHeightGridMap2D.h>
#include <gtest/gtest.h>

template <class MAP>
void do_test_insertCheckMapBounds()
{
	mrpt::maps::CHeightGridMap2D_Base::TPointInsertParams pt_params;
	pt_params.update_map_after_insertion = false;

	MAP dem; //mrpt::maps::CHeightGridMap2D dem;
	dem.setSize(-4.0,4.0, 0.0, 4.0,  1.0); // x:[-10,10] * y:[0,5]
	// Inside:
	EXPECT_TRUE( dem.insertIndividualPoint(2.0, 3.0,   56.0, pt_params ) );
	EXPECT_TRUE( dem.insertIndividualPoint(-3.0, 0.4,  56.0, pt_params ) );
	EXPECT_TRUE( dem.insertIndividualPoint( 3.0, 3.8,  56.0, pt_params ) );
	// Outside:
	EXPECT_FALSE(dem.insertIndividualPoint( -11.0, 2.0,  56.0, pt_params ) );
	EXPECT_FALSE(dem.insertIndividualPoint(  11.0, 2.0,  56.0, pt_params ) );
	EXPECT_FALSE(dem.insertIndividualPoint(  2.0, -1.0,  56.0, pt_params ) );
	EXPECT_FALSE(dem.insertIndividualPoint(  2.0,  6.0,  56.0, pt_params ) );
}
TEST(CHeightGridMap2Ds, insertCheckMapBounds)
{
	do_test_insertCheckMapBounds<mrpt::maps::CHeightGridMap2D>();
	do_test_insertCheckMapBounds<mrpt::maps::CHeightGridMap2D_MRF>();
}

template <class MAP>
void do_test_insertPointsAndRead()
{
	MAP dem;
	dem.setSize(0.0,5.0, 0.0, 5.0,  0.5); // x:[-10,10] * y:[0,5]
	// Inside:
	const double x = 2.1, y=3.1, z_write = 56.0;
	dem.insertIndividualPoint(x,y,z_write );
	double z_read;
	bool res = dem.dem_get_z(x,y, z_read);
	EXPECT_TRUE(res);
	EXPECT_NEAR(z_read,z_write, 1e-6);
}
TEST(CHeightGridMap2Ds, insertPointsAndRead)
{
	do_test_insertPointsAndRead<mrpt::maps::CHeightGridMap2D>();
	do_test_insertPointsAndRead<mrpt::maps::CHeightGridMap2D_MRF>();
}

