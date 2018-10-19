/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/maps/CPointCloudFilterByDistance.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <gtest/gtest.h>

void run_pc_filter_test(
	const double map2_x_off, const double map2_tim_off,
	const size_t expected_m1_count, const size_t expected_m2_count)
{
	const double pts1[8][3] = {
		{1, 0, 0}, {1.03, 0, 0},	 {1, 1, 0},  {1.01, 1.02, 0},
		{0, 1, 0}, {-0.01, 1.01, 0}, {-1, 0, 0}, {-1.01, 0.02, 0}};
	const mrpt::poses::CPose3D pts1_pose(0, 0, 0, 0, 0, 0);
	const mrpt::system::TTimeStamp pts1_tim = mrpt::system::now();

	const mrpt::poses::CPose3D pts2_pose(0.5, 0, 0, 0, 0, 0);
	const mrpt::system::TTimeStamp pts2_tim =
		mrpt::system::timestampAdd(pts1_tim, 0.2 + map2_tim_off);

	mrpt::maps::CSimplePointsMap map1, map2;
	for (const auto& i : pts1) map1.insertPoint(i[0], i[1], i[2]);
	for (size_t i = 0; i < sizeof(pts1) / sizeof(pts1[0]); i++)
	{
		double x, y, z;
		pts2_pose.inverseComposePoint(
			pts1[i][0], pts1[i][1], pts1[i][2], x, y, z);
		// Introduce optionally, 1 outlier:
		if (i == 1) x += map2_x_off;
		map2.insertPoint(x, y, z);
	}

	mrpt::maps::CPointCloudFilterByDistance pc_filter;
	mrpt::maps::CPointCloudFilterByDistance::TExtraFilterParams extra_params;
	std::vector<bool> deletion_mask;
	extra_params.out_deletion_mask = &deletion_mask;

	pc_filter.filter(&map1, pts1_tim, pts1_pose, &extra_params);
	EXPECT_EQ(map1.size(), expected_m1_count);

	pc_filter.filter(&map2, pts2_tim, pts2_pose, &extra_params);
	EXPECT_EQ(map2.size(), expected_m2_count);
}

TEST(CPointCloudFilterByDistance, noOutliers)
{
	run_pc_filter_test(
		.0 /*map2_x_off*/, .0 /*map2_tim_off*/, 8 /*expected_m1_count*/,
		8 /*expected_m2_count*/);
}
TEST(CPointCloudFilterByDistance, withOutliers)
{
	run_pc_filter_test(
		.35 /*map2_x_off*/, .0 /*map2_tim_off*/, 8 /*expected_m1_count*/,
		6 /*expected_m2_count*/);
}
TEST(CPointCloudFilterByDistance, tooOldMap)
{
	run_pc_filter_test(
		.35 /*map2_x_off*/, 2.0 /*map2_tim_off*/, 8 /*expected_m1_count*/,
		8 /*expected_m2_count*/);
}
