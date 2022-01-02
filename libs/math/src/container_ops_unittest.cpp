/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
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

TEST(ops_containers, xcorr)
{
	// Known output test case:
	const std::vector<double> a = {
		1.0000000000, 0.8400000000, 0.7056000000, 0.5927040000,
		0.4978713600, 0.4182119424, 0.3512980316, 0.2950903466,
		0.2478758911, 0.2082157485, 0.1749012288, 0.1469170322,
		0.1234103070, 0.1036646579, 0.0870783126, 0.0731457826,
	};
	const std::vector<double> b = {
		1.0000000000, 0.9200000000, 0.8464000000, 0.7786880000,
		0.7163929600, 0.6590815232, 0.6063550013, 0.5578466012,
		0.5132188731, 0.4721613633, 0.4343884542, 0.3996373779,
		0.3676663877, 0.3382530766, 0.3111928305, 0.2862974041,
	};

	const size_t maxLag = 5;
	const std::vector<double> r = mrpt::math::xcorr(a, b, maxLag);

	// Verify that the peak is at the correct spot:
	const size_t maxCoeffIdx_gt = 5;

	ASSERT_EQUAL_(r.size(), maxLag * 2 + 1);
	for (size_t i = 0; i < r.size(); i++)
	{
		if (i == maxCoeffIdx_gt) continue;
		EXPECT_GT(r[maxCoeffIdx_gt], r[i]);
	}
}
