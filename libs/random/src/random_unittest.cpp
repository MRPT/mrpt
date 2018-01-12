/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/random/RandomGenerators.h>
#include <gtest/gtest.h>

TEST(Random, Randomize)
{
	using namespace mrpt::random;

	CRandomGenerator rnd;
	rnd.randomize(1);
	auto r1a = rnd.drawUniform32bit();
	rnd.randomize(2);
	auto r2a = rnd.drawUniform32bit();

	EXPECT_NE(r1a, r2a);

	rnd.randomize(1);
	auto r1abis = rnd.drawUniform32bit();
	EXPECT_EQ(r1a, r1abis);
}
