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

namespace mrpt::ros2bridge
{
mrpt::system::TTimeStamp fromROS(const rclcpp::Time& src)
{
	// Use the signature of time_tToTimestamp() that accepts "double" with the
	// fractional parts of seconds:
	return mrpt::system::time_tToTimestamp(src.sec + src.nsec * 1e-9);

	const mrpt::Clock::duration time =
		std::chrono::duration_cast<mrpt::Clock::duration>(
			std::chrono::seconds(src.sec) + std::chrono::nanoseconds(src.nsec) +
			std::chrono::seconds(11644473600));

	return mrpt::system::TTimeStamp(time);
}

rclcpp::Time toROS(const mrpt::system::TTimeStamp& src)
{
	// Convert to "double-version of time_t", then extract integer and
	// fractional parts:
	const double t = mrpt::system::timestampTotime_t(src);
	rclcpp::Time des;
	des.sec = static_cast<uint64_t>(t);
	des.nsec = static_cast<uint64_t>(std::fmod(t, 1.0) * 1e9 + 0.5 /*round*/);
	return des;
}
}  // namespace mrpt::ros2bridge
