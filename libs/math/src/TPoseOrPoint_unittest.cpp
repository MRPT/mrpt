/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <CTraitsTest.h>
#include <gtest/gtest.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/math/TSegment2D.h>

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
		constexpr TPoint2D p_mul2 = 2 * p1;
		constexpr TPoint2D p_mul3 = 2.0f * p1;
		constexpr TPoint2D p_mul4 = 2.0 * p1;
		constexpr TPoint2D p_div = p1 / 2;

		static_assert(p_add == TPoint2D(6.0, 8.0));
		static_assert(p_sub == TPoint2D(2.0, 4.0));
		static_assert(p_mul == TPoint2D(8.0, 12.0));
		static_assert(p_mul2 == TPoint2D(8.0, 12.0));
		static_assert(p_mul3 == TPoint2D(8.0, 12.0));
		static_assert(p_mul4 == TPoint2D(8.0, 12.0));
		static_assert(p_div == TPoint2D(2.0, 3.0));
	}
	{
		constexpr TPoint3D p1(4.0, 6.0, 1.0);
		constexpr TPoint3D p2(2.0, 2.0, 3.0);
		constexpr TPoint3D p_add = p1 + p2;
		constexpr TPoint3D p_sub = p1 - p2;
		constexpr TPoint3D p_mul = p1 * 2;
		constexpr TPoint3D p_mul2 = 2 * p1;
		constexpr TPoint3D p_mul3 = 2.0f * p1;
		constexpr TPoint3D p_mul4 = 2.0 * p1;
		constexpr TPoint3D p_div = p1 / 2;

		static_assert(p_add == TPoint3D(6.0, 8.0, 4.0));
		static_assert(p_sub == TPoint3D(2.0, 4.0, -2.0));
		static_assert(p_mul == TPoint3D(8.0, 12.0, 2.0));
		static_assert(p_mul2 == TPoint3D(8.0, 12.0, 2.0));
		static_assert(p_mul3 == TPoint3D(8.0, 12.0, 2.0));
		static_assert(p_mul4 == TPoint3D(8.0, 12.0, 2.0));
		static_assert(p_div == TPoint3D(2.0, 3.0, 0.5));
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
