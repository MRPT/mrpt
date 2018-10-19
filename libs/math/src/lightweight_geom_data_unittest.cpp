/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/math/lightweight_geom_data.h>
#include <CTraitsTest.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace std;

template class mrpt::CTraitsTest<mrpt::math::TPoint2D>;
template class mrpt::CTraitsTest<mrpt::math::TPoint3D>;
template class mrpt::CTraitsTest<mrpt::math::TPoint3Df>;
template class mrpt::CTraitsTest<mrpt::math::TPose2D>;
template class mrpt::CTraitsTest<mrpt::math::TPose3D>;
template class mrpt::CTraitsTest<mrpt::math::TPose3DQuat>;

TEST(LightGeomData, PragmaPack)
{
	{
		TPoint2D p;
		EXPECT_TRUE(&p.x == &(p[0]));
		EXPECT_TRUE(&p.y == &(p[1]));
	}
	{
		TPoint3D p;
		EXPECT_TRUE(&p.x == &(p[0]));
		EXPECT_TRUE(&p.y == &(p[1]));
		EXPECT_TRUE(&p.z == &(p[2]));
	}
	{
		TPose2D p;
		EXPECT_TRUE(&p.x == &(p[0]));
		EXPECT_TRUE(&p.y == &(p[1]));
		EXPECT_TRUE(&p.phi == &(p[2]));
	}
	{
		TPose3D p;
		EXPECT_TRUE(&p.x == &(p[0]));
		EXPECT_TRUE(&p.y == &(p[1]));
		EXPECT_TRUE(&p.z == &(p[2]));
		EXPECT_TRUE(&p.yaw == &(p[3]));
		EXPECT_TRUE(&p.pitch == &(p[4]));
		EXPECT_TRUE(&p.roll == &(p[5]));
	}
	{
		TSegment2D s;
		EXPECT_TRUE(&s.point1 == &(s[0]));
		EXPECT_TRUE(&s.point2 == &(s[1]));
	}
}

TEST(LightGeomData, ExpectedMemorySizes)
{
	EXPECT_EQ(sizeof(TPoint2D), 2 * sizeof(double));
	EXPECT_EQ(sizeof(TPoint3D), 3 * sizeof(double));
	EXPECT_EQ(sizeof(TPoint3Df), 3 * sizeof(float));
	EXPECT_EQ(sizeof(TPose2D), 3 * sizeof(double));
	EXPECT_EQ(sizeof(TPose3D), 6 * sizeof(double));
	EXPECT_EQ(sizeof(TPose3DQuat), 7 * sizeof(double));
}

TEST(LightGeomData, ConstExprCtors)
{
	{
		const TPoint2D p(1.0, 2.0);
		EXPECT_EQ(p.x, 1.0);
		EXPECT_EQ(p.y, 2.0);
	}
	{
		constexpr TPoint2D p(1.0, 2.0);
		static_assert(p.x == 1.0, "p.x == 1.0");
		static_assert(p.y == 2.0, "p.y == 2.0");
		static_assert(p[0] == 1.0, "p[0] == 1.0");
		static_assert(p[1] == 2.0, "p[1] == 2.0");
	}
	{
		constexpr TPoint3D p(1.0, 2.0, 3.0);
		static_assert(p.x == 1.0, "p.x == 1.0");
		static_assert(p.y == 2.0, "p.y == 2.0");
		static_assert(p.z == 3.0, "p.z == 3.0");
		static_assert(p[0] == 1.0, "p[0] == 1.0");
		static_assert(p[1] == 2.0, "p[1] == 2.0");
		static_assert(p[2] == 3.0, "p[2] == 3.0");
	}
	{
		constexpr TPose3D p(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
		static_assert(p.x == 1.0, "p.x == 1.0");
		static_assert(p.y == 2.0, "p.y == 2.0");
		static_assert(p.z == 3.0, "p.z == 3.0");
	}
	{
		constexpr TPoint2D p1(4.0, 6.0);
		constexpr TPoint2D p2(2.0, 2.0);
		constexpr TPoint2D p_add = p1 + p2;
		constexpr TPoint2D p_sub = p1 - p2;
		constexpr TPoint2D p_mul = p1 * 2;
		constexpr TPoint2D p_div = p1 / 2;

		static_assert(p_add.x == 6.0, "p_add.x == 6.0");
		static_assert(p_add.y == 8.0, "p_add.y == 8.0");

		static_assert(p_sub.x == 2.0, "p_sub.x == 2.0");
		static_assert(p_sub.y == 4.0, "p_sub.y == 4.0");

		static_assert(p_mul.x == 8.0, "p_mul.x == 8.0");
		static_assert(p_mul.y == 12.0, "p_mul.y == 12.0");

		static_assert(p_div.x == 2.0, "p_div.x == 2.0");
		static_assert(p_div.y == 3.0, "p_div.y == 3.0");
	}
}

TEST(LightGeomData, Conversions)
{
	{
		TPose3D p1{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
		TPoint2D p2(p1);
		EXPECT_EQ(p2.x, 1);
		EXPECT_EQ(p2.y, 2);
	}
	{
		TPoint3D p1{1.0, 2.0, 3.0};
		TPose2D p2(p1);
		EXPECT_EQ(p2.x, 1.0);
		EXPECT_EQ(p2.y, 2.0);
		EXPECT_EQ(p2.phi, 0.0);
	}
}

TEST(LightGeomData, Comparisons)
{
	{
		TPoint2D p1{1, 1};
		TPoint2D p2{1, 1};
		EXPECT_FALSE(p1 < p2);
	}
	{
		TPoint2D p1{1, 1};
		TPoint2D p2{2, 2};
		EXPECT_TRUE(p1 < p2);
		EXPECT_FALSE(p2 < p1);
	}
}

TEST(LightGeomData, Strings)
{
	{
		TPose2D p{1, 2, M_PI};
		std::string s;
		p.asString(s);
		// note this is printed in degrees
		EXPECT_EQ(s, "[1.000000 2.000000 180.000000]");
	}
	{
		TPose2D p;
		p.fromString("[1 2 180]");
		EXPECT_DOUBLE_EQ(p.x, 1);
		EXPECT_DOUBLE_EQ(p.y, 2);
		EXPECT_DOUBLE_EQ(p.phi, M_PI);
	}
}
