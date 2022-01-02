/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*
 * test_time.cpp
 *
 *  Created on: July 15, 2014
 *      Author: Markus Bader
 */

#include <gtest/gtest.h>
#include <mrpt/ros1bridge/time.h>

TEST(Time, basicTest)
{
	const auto org_time = mrpt::Clock::now();

	ros::Time ros_tim = mrpt::ros1bridge::toROS(org_time);
	mrpt::system::TTimeStamp mrpt_tim = mrpt::ros1bridge::fromROS(ros_tim);

	EXPECT_NEAR(mrpt::system::timeDifference(org_time, mrpt_tim), .0, 1e-6);
}
