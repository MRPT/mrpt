/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*
 * test_map.cpp
 *
 *  Created on: July 21, 2014
 *      Author: Markus Bader
 */

#include <gtest/gtest.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/ros2bridge/map.h>

#include <nav_msgs/msg/occupancy_grid.hpp>

using mrpt::maps::COccupancyGridMap2D;

void getEmptyRosMsg(nav_msgs::msg::OccupancyGrid& msg)
{
	msg.info.width = 300;
	msg.info.height = 500;
	msg.info.resolution = 0.1;
	msg.info.origin.position.x = rand() % 10 - 5;
	msg.info.origin.position.y = rand() % 10 - 5;
	msg.info.origin.position.z = 0;

	msg.info.origin.orientation.x = 0;
	msg.info.origin.orientation.y = 0;
	msg.info.origin.orientation.z = 0;
	msg.info.origin.orientation.w = 1;

	msg.data.resize(msg.info.width * msg.info.height, -1);
}

TEST(Map, basicTestHeader)
{
	nav_msgs::msg::OccupancyGrid srcRos;
	COccupancyGridMap2D desMrpt;

	getEmptyRosMsg(srcRos);

	srcRos.info.origin.orientation.x = 0;  // fix rotation
	EXPECT_TRUE(mrpt::ros2bridge::fromROS(srcRos, desMrpt));

	EXPECT_EQ(srcRos.info.width, desMrpt.getSizeX());
	EXPECT_EQ(srcRos.info.height, desMrpt.getSizeY());
	EXPECT_EQ(srcRos.info.resolution, desMrpt.getResolution());
	for (uint32_t h = 0; h < srcRos.info.width; h++)
	{
		for (uint32_t w = 0; w < srcRos.info.width; w++)
		{
			EXPECT_EQ(
				desMrpt.getPos(w, h), 0.5);	 // all -1 entreis should map to 0.5
		}
	}
}

TEST(Map, check_ros2mrpt_and_back)
{
	nav_msgs::msg::OccupancyGrid srcRos;
	COccupancyGridMap2D desMrpt;
	nav_msgs::msg::OccupancyGrid desRos;

	// Test empty gridmap:
	getEmptyRosMsg(srcRos);

	ASSERT_TRUE(mrpt::ros2bridge::fromROS(srcRos, desMrpt));
	ASSERT_TRUE(mrpt::ros2bridge::toROS(desMrpt, desRos, desRos.header));
	// all -1 entries should map back to -1
	for (uint32_t h = 0; h < srcRos.info.width; h++)
		for (uint32_t w = 0; w < srcRos.info.width; w++)
			EXPECT_EQ(desRos.data[h * srcRos.info.width + h], -1);

	// Test gridmap with values: 0 to 100
	for (int i = 0; i <= 100; i++)
	{
		// 50 is mid-gray -> unknown = -1 in ROS
		srcRos.data[i] = (std::abs(i - 50) <= 2) ? -1 : i;
	}

	EXPECT_TRUE(mrpt::ros2bridge::fromROS(srcRos, desMrpt));
	EXPECT_TRUE(mrpt::ros2bridge::toROS(desMrpt, desRos, desRos.header));

	for (int i = 0; i <= 100; i++)
	{
		/*printf(
			"%4i, %4.3f = %4.3f,%4i\n", srcRos.data[i], 1.0f - 0.01f * i,
			desMrpt.getCell(i, 0), desRos.data[i]); */
		EXPECT_NEAR(1.0f - 0.01f * i, desMrpt.getCell(i, 0), 0.03f)
			<< "ros to mprt"
			<< "i=" << i;
		EXPECT_NEAR(srcRos.data[i], desRos.data[i], 1) << "ros to mprt to ros";
	}
}
