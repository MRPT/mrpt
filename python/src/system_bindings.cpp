/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#include "bindings.h"

/* MRPT */
#include <mrpt/system/datetime.h>

/* STD */
#include <cstdint>

/* namespaces */
using namespace boost::python;
using namespace mrpt::system;

object system_timestampToParts(long_ timestamp)
{
	// import TTimeParts
	dict locals;
	exec(
		"from mrpt.system import TTimeParts\n"
		"ttimeparts = TTimeParts()\n",
		object(), locals);
	object ttimeparts = locals["ttimeparts"];
	auto t = mrpt::Clock::time_point(
		mrpt::Clock::duration(extract<uint64_t>(timestamp)));
	TTimeParts p;
	timestampToParts(t, p);
	ttimeparts.attr("year") = p.year;
	ttimeparts.attr("month") = p.month;
	ttimeparts.attr("day") = p.day;
	ttimeparts.attr("hour") = p.hour;
	ttimeparts.attr("minute") = p.minute;
	ttimeparts.attr("second") = p.second;
	ttimeparts.attr("day_of_week") = p.day_of_week;
	ttimeparts.attr("daylight_saving") = p.daylight_saving;
	return ttimeparts;
}

long_ system_time_tToTimestamp(const double& t)
{
	return long_(time_tToTimestamp(t));
}

#ifdef ROS_EXTENSIONS
object TTimeStamp_to_ROS_Time(long_ timestamp)
{
	auto t = mrpt::Clock::time_point(
		mrpt::Clock::duration(extract<uint64_t>(timestamp)));
	double secs = timestampTotime_t(t);
	// import rospy.Time
	dict locals;
	locals["secs"] = secs;
	exec(
		"import rospy\n"
		"time = rospy.Time.from_sec(secs)\n",
		object(), locals);
	return locals["time"];
}

long_ TTimeStamp_from_ROS_Time(object ros_time)
{
	return system_time_tToTimestamp(extract<double>(ros_time.attr("to_sec")()));
}
#endif

long_ mrpt_system_now() { return long_(mrpt::system::getCurrentTime()); }
// exporter
void export_system()
{
	// map namespace to be submodule of mrpt package
	MAKE_SUBMODULE(system)

	{
		class_<TTimeParts>("TTimeParts", init<>())
			.def_readwrite("year", &TTimeParts::year)
			.def_readwrite("month", &TTimeParts::month)
			.def_readwrite("day", &TTimeParts::day)
			.def_readwrite("hour", &TTimeParts::hour)
			.def_readwrite("minute", &TTimeParts::minute)
			.def_readwrite("second", &TTimeParts::second)
			.def_readwrite("day_of_week", &TTimeParts::day_of_week)
			.def_readwrite("daylight_saving", &TTimeParts::daylight_saving);
	}

	def("timestampToParts", &system_timestampToParts,
		"Gets the individual parts of a date/time (days, hours, minutes, "
		"seconds) - UTC time or local time");
	def("time_tToTimestamp", &system_time_tToTimestamp,
		"Transform from standard \"time_t\" (actually a double number, it can "
		"contain fractions of seconds) to TTimeStamp.");
	def("now", &mrpt_system_now,
		"Returns the current (local) time as TTimeStamp.");
#ifdef ROS_EXTENSIONS
	def("TTimeStamp_from_ROS_Time", &TTimeStamp_from_ROS_Time,
		"Convert TTimeStamp from ROS Time.");
	def("TTimeStamp_to_ROS_Time", &TTimeStamp_to_ROS_Time,
		"Convert TTimeStamp to ROS Time.");
#endif
}
