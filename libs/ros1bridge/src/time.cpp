/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/Clock.h>
#include <mrpt/ros1bridge/time.h>

#include <cmath>  // std::fmod

mrpt::system::TTimeStamp mrpt::ros1bridge::fromROS(const ros::Time& src)
{
	return mrpt::Clock::fromDouble(src.sec + src.nsec * 1e-9);
}

ros::Time mrpt::ros1bridge::toROS(const mrpt::system::TTimeStamp& src)
{
	// Convert to "double-version of time_t", then extract integer and
	// fractional parts:
	const double t = mrpt::Clock::toDouble(src);
	ros::Time des;
	des.sec = static_cast<uint64_t>(t);
	des.nsec = static_cast<uint64_t>(std::fmod(t, 1.0) * 1e9 + 0.5 /*round*/);
	return des;
}
