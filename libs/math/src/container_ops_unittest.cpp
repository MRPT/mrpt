/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/ops_containers.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace std;

TEST(CVectorDouble, resize)
{
	{
		mrpt::math::CVectorDouble v;
		EXPECT_TRUE(v.size() == 0);
	}

	for (int i = 0; i < 10; i++)
	{
		mrpt::math::CVectorDouble v(i);
		EXPECT_TRUE(v.size() == i);
	}

	for (int i = 0; i < 10; i++)
	{
		mrpt::math::CVectorDouble v;
		v.resize(i);
		EXPECT_TRUE(v.size() == i);
	}

	for (int i = 10; i >= 0; i--)
	{
		mrpt::math::CVectorDouble v;
		v.resize(i);
		EXPECT_TRUE(v.size() == i);
	}

	{
		mrpt::math::CVectorDouble v;
		for (int i = 0; i < 10; i++)
		{
			v.push_back(double(i));
			EXPECT_TRUE(v.size() == (i + 1));
		}
		for (int i = 0; i < 10; i++)
			EXPECT_NEAR(v[i], i, 1e-10);
	}
}

TEST(ops_containers, ncc_vector)
{
	{
		// Error on invalid lengths:
		const std::vector<double> s1 = {1.0, 2.0};
		const std::vector<double> s2 = {1.0, 2.0, 3.0};
		EXPECT_ANY_THROW(mrpt::math::ncc_vector(s1, s2));
	}

	{
		// Error on null mean:
		const std::vector<double> s1 = {0.0, 0.0};
		const std::vector<double> s2 = {0.0, 0.0};
		EXPECT_ANY_THROW(mrpt::math::ncc_vector(s1, s2));
	}

	// Known output test case:
	const std::vector<double> s1 = {
		1.000000, 0.840000, 0.705600, 0.592704, 0.497871, 0.418212,
		0.351298, 0.295090, 0.247876, 0.208216, 0.174901, 0.146917,
		0.123410, 0.103665, 0.087078, 0.073146,
	};
	const std::vector<double> s2 = {
		1.000000, 0.920000, 0.846400, 0.778688, 0.716393, 0.659082,
		0.606355, 0.557847, 0.513219, 0.472161, 0.434388, 0.399637,
		0.367666, 0.338253, 0.311193, 0.286297,
	};
	const double ncc = mrpt::math::ncc_vector(s1, s2);
	const double ncc_gt = 0.9858405444;
	EXPECT_NEAR(ncc, ncc_gt, 1e-4);
}
