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
	}
	{
		constexpr TPoint3D p(1.0, 2.0, 3.0);
		static_assert(p.x == 1.0, "p.x == 1.0");
		static_assert(p.y == 2.0, "p.y == 2.0");
		static_assert(p.z == 3.0, "p.z == 3.0");
	}
	{
		constexpr TPose3D p(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
		static_assert(p.x == 1.0, "p.x == 1.0");
		static_assert(p.y == 2.0, "p.y == 2.0");
		static_assert(p.z == 3.0, "p.z == 3.0");
	}
}
