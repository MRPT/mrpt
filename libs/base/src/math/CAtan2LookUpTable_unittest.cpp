/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/math/CAtan2LookUpTable.h>
#include <mrpt/random.h>
#include <gtest/gtest.h>
#include <cmath>

TEST(CAtan2LookUpTable,ValidValidTest)
{
	const double SIZE = 20.0;
	const double RES  = 0.10;

	mrpt::math::CAtan2LookUpTable atan2lut(-.5*SIZE, .5*SIZE, -.5*SIZE, .5*SIZE, RES);

	for (int i=0;i<100;i++)
	{
		const double x = mrpt::random::randomGenerator.drawUniform(-.5*SIZE, .5*SIZE);
		const double y = mrpt::random::randomGenerator.drawUniform(-.5*SIZE, .5*SIZE);

		// Avoid the central part, where accuracy is worse
		if (std::abs(x)<2.0 || std::abs(y)<2.0)
			continue;

		const double atan2_good = ::atan2(y,x);
		double atan2_lut;
		bool atan2_lut_valid = atan2lut.atan2(y,x,atan2_lut);
		
		EXPECT_TRUE(atan2_lut_valid);
		EXPECT_NEAR(atan2_good, atan2_lut, mrpt::utils::DEG2RAD(1) ) << "(x,y): ("<< x << " , " << y << ")";
	}
}

TEST(CAtan2LookUpTable,MultiResTest)
{

}

