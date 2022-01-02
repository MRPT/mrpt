/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <CTraitsTest.h>
#include <gtest/gtest.h>
#include <mrpt/containers/vector_with_small_size_optimization.h>
#include <mrpt/core/common.h>

using data_t = unsigned int;
constexpr size_t SMALL_LEN = 16;

using vwsso_t =
	mrpt::containers::vector_with_small_size_optimization<data_t, SMALL_LEN>;

template class mrpt::CTraitsTest<vwsso_t>;

TEST(vector_with_small_size_optimization, Empty)
{
	vwsso_t v;

	EXPECT_TRUE(v.empty());
	EXPECT_TRUE(v.size() == 0);
}

TEST(vector_with_small_size_optimization, ResizeSize)
{
	// New one each time:
	for (size_t n = 1; n < SMALL_LEN * 2; n++)
	{
		vwsso_t v;
		v.resize(n);
		EXPECT_FALSE(v.empty());
		EXPECT_EQ(v.size(), n);
	}

	// Reusing object:
	vwsso_t v;
	for (size_t n = 1; n < SMALL_LEN * 2; n++)
	{
		v.resize(n);
		EXPECT_FALSE(v.empty());
		EXPECT_EQ(v.size(), n);
	}
}

TEST(vector_with_small_size_optimization, ResizeWriteRead)
{
	// New one each time:
	for (size_t n = 1; n < SMALL_LEN * 2; n++)
	{
		vwsso_t v;
		v.resize(n);

		for (size_t i = 0; i < n; i++)
			v[i] = i;
		for (size_t i = 0; i < n; i++)
			EXPECT_EQ(v[i], i);
	}

	// Reusing object:
	vwsso_t v;
	for (size_t n = 1; n < SMALL_LEN * 2; n++)
	{
		v.resize(n);
		for (size_t i = 0; i < n; i++)
			v[i] = i;
		for (size_t i = 0; i < n; i++)
			EXPECT_EQ(v[i], i);
	}
}

TEST(vector_with_small_size_optimization, ResizeWriteReadIterators)
{
	// New one each time:
	for (size_t n = 1; n < SMALL_LEN * 2; n++)
	{
		vwsso_t v;
		v.resize(n);

		for (size_t i = 0; i < n; i++)
			v[i] = i;

		size_t i = 0;
		for (const auto& val : v)
		{
			EXPECT_EQ(val, i);
			i++;
		}
	}

	// Reusing object:
	vwsso_t v;
	for (size_t n = 1; n < SMALL_LEN * 2; n++)
	{
		v.resize(n);
		for (size_t i = 0; i < n; i++)
			v[i] = i;
		size_t i = 0;
		for (const auto& val : v)
		{
			EXPECT_EQ(val, i);
			i++;
		}
	}
}

TEST(vector_with_small_size_optimization, GrowCheckContents)
{
	vwsso_t v;
	for (size_t n = 1; n < SMALL_LEN * 2; n++)
	{
		v.resize(n);
		v[n - 1] = n - 1;

		for (size_t i = 0; i < n; i++)
			EXPECT_EQ(v[i], i) << "n=" << n;
	}
}

TEST(vector_with_small_size_optimization, ShrinkCheckContents)
{
	vwsso_t v;
	v.resize(SMALL_LEN * 2);
	for (size_t i = 0; i < v.size(); i++)
		v[i] = i;

	while (!v.empty())
	{
		v.resize(v.size() - 1);

		for (size_t i = 0; i < v.size(); i++)
			EXPECT_EQ(v[i], i) << "size()=" << v.size();
	}
}

TEST(vector_with_small_size_optimization, GrowCheckFrontBack)
{
	vwsso_t v;
	for (size_t n = 1; n < SMALL_LEN * 2; n++)
	{
		v.resize(n);
		EXPECT_EQ(&v[0], &v.front()) << "n=" << n;
		EXPECT_EQ(&v[n - 1], &v.back()) << "n=" << n;
	}
}
TEST(vector_with_small_size_optimization, push_back)
{
	vwsso_t v;
	EXPECT_EQ(v.size(), 0U);

	for (size_t i = 0; i < 2 * SMALL_LEN; i++)
	{
		v.push_back(100 + i);
		EXPECT_EQ(v.size(), i + 1);

		for (size_t j = 0; j <= i; j++)
		{
			EXPECT_EQ(v.at(j), 100 + j);
			EXPECT_EQ(v[j], 100 + j);
		}
	}
}
