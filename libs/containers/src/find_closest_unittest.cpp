/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/containers/find_closest.h>
#include <mrpt/core/common.h>

#include <map>

TEST(find_closest, testStdMap)
{
	{
		const std::map<float, char> emptyData = {};

		auto ret = mrpt::containers::find_closest(emptyData, 1.0);
		EXPECT_FALSE(ret.has_value());
	}

	const std::map<float, char> data = {
		{1.0f, 'A'}, {2.0f, 'B'}, {3.0f, 'C'}, {100.0f, 'Z'}};

	{
		auto ret = mrpt::containers::find_closest(data, 1.0);
		EXPECT_TRUE(ret.has_value());
		EXPECT_EQ(ret.value().second, 'A');
	}
	{
		auto ret = mrpt::containers::find_closest(data, 1.4f);
		EXPECT_TRUE(ret.has_value());
		EXPECT_EQ(ret.value().second, 'A');
	}
	EXPECT_EQ(mrpt::containers::find_closest(data, -100.0).value().second, 'A');
	EXPECT_EQ(mrpt::containers::find_closest(data, 2.1).value().second, 'B');
	EXPECT_EQ(mrpt::containers::find_closest(data, 2.45).value().second, 'B');
	EXPECT_EQ(mrpt::containers::find_closest(data, 2.55).value().second, 'C');
	EXPECT_EQ(mrpt::containers::find_closest(data, 90).value().second, 'Z');
	EXPECT_EQ(mrpt::containers::find_closest(data, 1190).value().second, 'Z');
}

TEST(find_closest, testStdMultiMap)
{
	const std::multimap<float, char> data = {
		{1.0f, 'A'}, {2.0f, 'B'}, {3.0f, 'C'}, {100.0f, 'Z'}};

	{
		auto ret = mrpt::containers::find_closest(data, 1.0);
		EXPECT_TRUE(ret.has_value());
		EXPECT_EQ(ret.value().second, 'A');
	}
	{
		auto ret = mrpt::containers::find_closest(data, 1.4f);
		EXPECT_TRUE(ret.has_value());
		EXPECT_EQ(ret.value().second, 'A');
	}
	EXPECT_EQ(mrpt::containers::find_closest(data, -100.0).value().second, 'A');
	EXPECT_EQ(mrpt::containers::find_closest(data, 2.1).value().second, 'B');
	EXPECT_EQ(mrpt::containers::find_closest(data, 2.45).value().second, 'B');
	EXPECT_EQ(mrpt::containers::find_closest(data, 2.55).value().second, 'C');
	EXPECT_EQ(mrpt::containers::find_closest(data, 90).value().second, 'Z');
	EXPECT_EQ(mrpt::containers::find_closest(data, 1190).value().second, 'Z');
}

TEST(find_closest_with_tolerance, testStdMap)
{
	{
		const std::map<float, char> emptyData = {};

		auto ret =
			mrpt::containers::find_closest_with_tolerance(emptyData, 1.0, 0.1);
		EXPECT_FALSE(ret.has_value());
	}

	const std::map<float, char> data = {
		{1.0f, 'A'}, {2.0f, 'B'}, {3.0f, 'C'}, {100.0f, 'Z'}};

	{
		auto ret =
			mrpt::containers::find_closest_with_tolerance(data, 1.0, 0.1);
		EXPECT_TRUE(ret.has_value());
		EXPECT_EQ(ret.value().second, 'A');
	}

	EXPECT_FALSE(mrpt::containers::find_closest_with_tolerance(data, 0.9, 0.09)
					 .has_value());
	EXPECT_EQ(
		mrpt::containers::find_closest_with_tolerance(data, 0.9, 0.11)
			.value()
			.second,
		'A');
	EXPECT_EQ(
		mrpt::containers::find_closest_with_tolerance(data, 0.9f, 0.11f)
			.value()
			.second,
		'A');

	EXPECT_EQ(
		mrpt::containers::find_closest_with_tolerance(data, 1.0, 0.0)
			.value()
			.second,
		'A');

	EXPECT_EQ(
		mrpt::containers::find_closest_with_tolerance(data, 2.4f, 0.5f)
			.value()
			.second,
		'B');
	EXPECT_EQ(
		mrpt::containers::find_closest_with_tolerance(data, 2.6f, 0.5f)
			.value()
			.second,
		'C');
	EXPECT_FALSE(mrpt::containers::find_closest_with_tolerance(data, 2.6f, 0.3f)
					 .has_value());

	EXPECT_FALSE(mrpt::containers::find_closest_with_tolerance(data, 200, 10)
					 .has_value());
	EXPECT_EQ(
		mrpt::containers::find_closest_with_tolerance(data, 200, 110)
			.value()
			.second,
		'Z');
}
