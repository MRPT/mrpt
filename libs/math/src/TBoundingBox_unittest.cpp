/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TPose3D.h>

template <typename T>
void testDefaultCtor()
{
	mrpt::math::TBoundingBox_<T> bb;

	EXPECT_EQ(bb.min.x, 0);
	EXPECT_EQ(bb.min.y, 0);
	EXPECT_EQ(bb.min.z, 0);

	EXPECT_EQ(bb.max.x, 0);
	EXPECT_EQ(bb.max.y, 0);
	EXPECT_EQ(bb.max.z, 0);
}

TEST(TBoundingBox, defaultCtor)
{
	testDefaultCtor<float>();
	testDefaultCtor<double>();
}

template <typename T>
void testFromCorners()
{
	mrpt::math::TBoundingBox_<T> bb({-1, -2, -3}, {9, 8, 7});
	EXPECT_NEAR(bb.volume(), 10 * 10 * 10, 1e-3);
}

TEST(TBoundingBox, fromCorners)
{
	testFromCorners<float>();
	testFromCorners<double>();
}

template <typename T>
void testInvalidThrows()
{
	EXPECT_ANY_THROW(mrpt::math::TBoundingBox_<T> bb({0, 0, 0}, {-1, 1, 1}));
	EXPECT_ANY_THROW(mrpt::math::TBoundingBox_<T> bb({0, 0, 0}, {1, -1, 1}));
	EXPECT_ANY_THROW(mrpt::math::TBoundingBox_<T> bb({0, 0, 0}, {1, 1, -1}));
}

TEST(TBoundingBox, invalidThrows)
{
	testInvalidThrows<float>();
	testInvalidThrows<double>();
}
template <typename T>
void testIntersections()
{
	{
		mrpt::math::TBoundingBox_<T> bb1({0, 0, 0}, {1, 1, 1});
		mrpt::math::TBoundingBox_<T> bb2({1.1, 0, 0}, {1.2, 1, 1});
		const auto inter = bb1.intersection(bb2);
		EXPECT_FALSE(inter);
	}

	{
		mrpt::math::TBoundingBox_<T> bb1({0, 0, 0}, {1, 1, 0});
		mrpt::math::TBoundingBox_<T> bb2({0.1, 0.2, 0}, {0.8, 0.9, 0});
		const T epsilon = static_cast<T>(0.001);
		const auto inter = bb1.intersection(bb2, epsilon);
		EXPECT_TRUE(inter);
	}
	{
		mrpt::math::TBoundingBox_<T> bb1({0, 0, -1e-8}, {1, 1, -1e-8});
		mrpt::math::TBoundingBox_<T> bb2({0.1, 0.2, -1e-9}, {0.8, 0.9, 2e-9});
		const T epsilon = static_cast<T>(0.001);
		const auto inter = bb1.intersection(bb2, epsilon);
		EXPECT_TRUE(inter);
	}

	{
		mrpt::math::TBoundingBox_<T> bb1({0, 1, 2}, {10, 10, 10});
		mrpt::math::TBoundingBox_<T> bb2({5, 6, 7}, {6, 7, 8});
		const auto inter = bb1.intersection(bb2);
		EXPECT_TRUE(inter);
		EXPECT_EQ(inter->min, (mrpt::math::TPoint3D_<T>{5, 6, 7}));
		EXPECT_EQ(inter->max, (mrpt::math::TPoint3D_<T>{6, 7, 8}));
	}

	{
		mrpt::math::TBoundingBox_<T> bb1({0, 1, 2}, {10, 11, 12});
		mrpt::math::TBoundingBox_<T> bb2({5, 6, 7}, {16, 17, 18});
		const auto inter = bb1.intersection(bb2);
		EXPECT_TRUE(inter);
		EXPECT_EQ(inter->min, (mrpt::math::TPoint3D_<T>{5, 6, 7}));
		EXPECT_EQ(inter->max, (mrpt::math::TPoint3D_<T>{10, 11, 12}));
	}

	{
		mrpt::math::TBoundingBox_<T> bb1({0, 1, 2}, {10, 10, 10});
		mrpt::math::TBoundingBox_<T> bb2({-5, -6, -7}, {6, 7, 8});
		const auto inter = bb1.intersection(bb2);
		EXPECT_TRUE(inter);
		EXPECT_EQ(inter->min, (mrpt::math::TPoint3D_<T>{0, 1, 2}));
		EXPECT_EQ(inter->max, (mrpt::math::TPoint3D_<T>{6, 7, 8}));
	}
}

TEST(TBoundingBox, intersections)
{
	testIntersections<float>();
	testIntersections<double>();
}

TEST(TBoundingBox, compose)
{
	mrpt::math::TBoundingBox bb1({0, 1, 2}, {3, 4, 5});
	const auto bb2 = bb1.compose(mrpt::math::TPose3D(10, 20, 30, 0, 0, 0));

	EXPECT_EQ(bb2.min, mrpt::math::TPoint3D(10, 21, 32));
	EXPECT_EQ(bb2.max, mrpt::math::TPoint3D(13, 24, 35));
}

TEST(TBoundingBox, inverseCompose)
{
	mrpt::math::TBoundingBox bb1({0, 1, 2}, {3, 4, 5});
	const auto bb2 =
		bb1.inverseCompose(mrpt::math::TPose3D(10, 20, 30, 0, 0, 0));

	EXPECT_EQ(bb2.min, mrpt::math::TPoint3D(-10, -19, -28));
	EXPECT_EQ(bb2.max, mrpt::math::TPoint3D(-7, -16, -25));
}
