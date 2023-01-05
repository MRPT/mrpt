/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
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
#include <mrpt/ros1bridge/point_cloud2.h>

#if HAVE_PCL
#include <pcl/common/common_headers.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>

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

	sensor_msgs::PointCloud2 point_cloud2_msg;
	pcl::toROSMsg(point_cloud, point_cloud2_msg);

	mrpt::maps::CSimplePointsMap mrpt_pc;

	mrpt::ros1bridge::fromROS(point_cloud2_msg, mrpt_pc);

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

TEST(PointCloud2, toROS)
{
	mrpt::maps::CSimplePointsMap pc1;

	const size_t num_points = 1000;
	pc1.resize(num_points);

	float i_f = 0;
	for (size_t i = 0; i < num_points; i++)
	{
		pc1.setPoint(i, i_f, -i_f, -2 * i_f);
		i_f += 1.0;
	}

	sensor_msgs::PointCloud2 pc_msg;
	std_msgs::Header hdr;
	hdr.frame_id = "map";
	bool ok = mrpt::ros1bridge::toROS(pc1, hdr, pc_msg);
	ASSERT_(ok);

	EXPECT_EQ(pc_msg.header.frame_id, hdr.frame_id);

	//
	mrpt::maps::CSimplePointsMap pc2;
	bool ok2 = mrpt::ros1bridge::fromROS(pc_msg, pc2);
	ASSERT_(ok2);

	EXPECT_EQ(pc1.size(), pc2.size());
	for (size_t i = 0; i < pc1.size(); i++)
	{
		mrpt::math::TPoint3D pt1, pt2;
		pc1.getPoint(i, pt1);
		pc2.getPoint(i, pt2);
		EXPECT_TRUE(pt1 == pt2);
	}
}
