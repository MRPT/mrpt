/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <CTraitsTest.h>
#include <gtest/gtest.h>
#include <mrpt/poses/CPosePDFGrid.h>

template class mrpt::CTraitsTest<mrpt::poses::CPosePDFGrid>;

TEST(CPosePDFGrid, defaultCtor)
{
	mrpt::poses::CPosePDFGrid pg;
	EXPECT_GE(pg.getXMax(), pg.getXMin());
	EXPECT_GE(pg.getYMax(), pg.getYMin());
	EXPECT_GE(pg.getPhiMax(), pg.getPhiMin());
}

TEST(CPosePDFGrid, resize)
{
	mrpt::poses::CPosePDFGrid pg;
	pg.setSize(0, 10.0, 0.0, 20.0, 1.0 /*res xy*/, 0.1 /*res phi*/);
}

TEST(CPosePDFGrid, basicOps)
{
	mrpt::poses::CPosePDFGrid pg;
	pg.setSize(0, 10.0, 0.0, 20.0, 1.0 /*res xy*/, 0.02 /*res phi*/);

	// Set:
	double* ptr_val = pg.getByPos(1.0, 2.0, 0.7);
	EXPECT_TRUE(ptr_val != nullptr);
	*ptr_val = 10.0;

	double* ptr_val2 = pg.getByPos(4.0, 7.0, -0.7);
	EXPECT_TRUE(ptr_val2 != nullptr);
	*ptr_val2 = 10.0;

	// Get:
	EXPECT_NEAR(*pg.getByPos(1.0, 2.0, 0.7), 10.0, 1e-5);
	EXPECT_NEAR(*pg.getByPos(4.0, 7.0, -0.7), 10.0, 1e-5);

	// Normalize:
	pg.normalize();
	EXPECT_NEAR(*pg.getByPos(1.0, 2.0, 0.7), 1.0 / 2.0, 1e-5);
	EXPECT_NEAR(*pg.getByPos(4.0, 7.0, -0.7), 1.0 / 2.0, 1e-5);

	// mean:
	const auto m = pg.getMeanVal();
	EXPECT_NEAR(m.x(), 2.5, 1e-4);
	EXPECT_NEAR(m.y(), 4.5, 1e-4);
	EXPECT_NEAR(m.phi(), 0.0, 5e-2);
}
