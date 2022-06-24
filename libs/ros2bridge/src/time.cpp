/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/Clock.h>
#include <mrpt/ros2bridge/time.h>

#include <cmath>  // std::fmod

mrpt::system::TTimeStamp mrpt::ros2bridge::fromROS(const rclcpp::Time& src)
{
	return mrpt::Clock::fromDouble(src.seconds());
}

rclcpp::Time mrpt::ros2bridge::toROS(const mrpt::system::TTimeStamp& src)
{
	// Convert to "double-version of time_t", then extract integer and
	// fractional parts:
	const double t = mrpt::Clock::toDouble(src);
	return rclcpp::Time(
		// seconds:
		static_cast<uint64_t>(t),
		// nanoseconds:
		static_cast<uint64_t>(std::fmod(t, 1.0) * 1e9 + 0.5 /*round*/));
}
