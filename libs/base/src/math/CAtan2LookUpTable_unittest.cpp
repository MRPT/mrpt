/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/math/CAtan2LookUpTable.h>
#include <mrpt/random.h>
#include <gtest/gtest.h>
#include <cmath>


template <class LUT_CLASS>
void atan2_lut_test(const LUT_CLASS & atan2lut, const double SIZE, const double max_deg_errors, const double skip_area)
{
	for (int i=0;i<1000;i++)
	{
		const double x = mrpt::random::randomGenerator.drawUniform(-.5*SIZE, .5*SIZE);
		const double y = mrpt::random::randomGenerator.drawUniform(-.5*SIZE, .5*SIZE);

		// Avoid the central part, where accuracy is worse
		if (std::abs(x)<skip_area || std::abs(y)<skip_area)
			continue;

		const double atan2_good = ::atan2(y,x);
		double atan2_lut;
		bool atan2_lut_valid = atan2lut.atan2(y,x,atan2_lut);
		
		EXPECT_TRUE(atan2_lut_valid);
		EXPECT_LT(std::abs(atan2_good-atan2_lut), mrpt::utils::DEG2RAD(max_deg_errors) ) << "(x,y): ("<< x << " , " << y << ")" << "\natan2_good:" << atan2_good << " atan2_lut:" << atan2_lut << std::endl;
	}
}

TEST(CAtan2LookUpTable,ValidValidTest)
{
	const double SIZE = 20.0;
	const double RES  = 0.10;
	mrpt::math::CAtan2LookUpTable atan2lut(-.5*SIZE, .5*SIZE, -.5*SIZE, .5*SIZE, RES);
	atan2_lut_test(atan2lut, SIZE,1.5 /*max error*/, 2.0 /* skip zone */);
}

TEST(CAtan2LookUpTable,MultiResTest)
{
	const double SIZE = 20.0;
	mrpt::math::CAtan2LookUpTableMultiRes atan2lut;

	std::map<double,double> res2extension;
	res2extension[0.001] = 0.8;   // 0.1 cm resolution
	res2extension[0.01]  = 2.0;   // 1.0 cm resolution
	res2extension[0.02] = 5.0;    // 2.0 cm resolution
	res2extension[0.05] = 11.0;   // 5.0 cm resolution
	atan2lut.resize(res2extension);

	atan2_lut_test(atan2lut, SIZE,0.5 /*max error*/, 0.4 /* skip zone */);
}

