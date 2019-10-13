/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>

TEST(CPoint, toFromString_2D)
{
	mrpt::poses::CPoint2D p;

	p.fromString("[1.0 2.0]");
	EXPECT_DOUBLE_EQ(p.x(), 1.0);
	EXPECT_DOUBLE_EQ(p.y(), 2.0);

	const auto s = p.asString();
	EXPECT_EQ(s, "[1.000000 2.000000]");

	EXPECT_THROW(p.fromString("[]"), std::exception);
	EXPECT_THROW(p.fromString("[1 2;4 5]"), std::exception);
	EXPECT_THROW(p.fromString("[1 2 3]"), std::exception);
	EXPECT_THROW(p.fromString("[1;2]"), std::exception);
}

TEST(CPoint, toFromString_3D)
{
	mrpt::poses::CPoint3D p;

	p.fromString("[1.0 2.0 3.0]");
	EXPECT_DOUBLE_EQ(p.x(), 1.0);
	EXPECT_DOUBLE_EQ(p.y(), 2.0);
	EXPECT_DOUBLE_EQ(p.z(), 3.0);

	const auto s = p.asString();
	EXPECT_EQ(s, "[1.000000 2.000000 3.000000]");

	EXPECT_THROW(p.fromString("[]"), std::exception);
	EXPECT_THROW(p.fromString("[1 2;4 5]"), std::exception);
	EXPECT_THROW(p.fromString("[1 2]"), std::exception);
	EXPECT_THROW(p.fromString("[1;2;3]"), std::exception);
}
