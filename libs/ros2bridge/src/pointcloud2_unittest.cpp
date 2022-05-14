/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*
 * test_pose_conversions.cpp
 *
 *  Created on: Mar 15, 2012
 *      Author: Pablo Iñigo Blasco
 */

#include <gtest/gtest.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/ros2bridge/point_cloud2.h>

#if HAVE_PCL_CONVERSIONS
#include <pcl/common/common_headers.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

TEST(PointCloud2, basicTest)
{
	pcl::PointCloud<pcl::PointXYZ> point_cloud;

	point_cloud.height = 10;
	point_cloud.width = 10;
	point_cloud.is_dense = true;

	int num_points = point_cloud.height * point_cloud.width;
	point_cloud.points.resize(num_points);

	float i_f = 0;
	for (int i = 0; i < num_points; i++)
	{
		pcl::PointXYZ& point = point_cloud.points[i];
		point.x = i_f;
		point.y = -i_f;
		point.z = -2 * i_f;
		i_f += 1.0;
	}

	// pcl_conversions:
	sensor_msgs::msg::PointCloud2 point_cloud2_msg;
	toROSMsg(point_cloud, point_cloud2_msg);

	mrpt::maps::CSimplePointsMap mrpt_pc;
	mrpt::ros2bridge::fromROS(point_cloud2_msg, mrpt_pc);

	i_f = 0;
	for (int i = 0; i < num_points; i++)
	{
		float mrpt_x, mrpt_y, mrpt_z;
		mrpt_pc.getPoint(i, mrpt_x, mrpt_y, mrpt_z);
		EXPECT_FLOAT_EQ(mrpt_x, i_f);
		EXPECT_FLOAT_EQ(mrpt_y, -i_f);
		EXPECT_FLOAT_EQ(mrpt_z, -2 * i_f);

		i_f += 1.0;
	}
	//;
}
#endif	// HAVE_PCL
