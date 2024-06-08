#include <chrono>
#include <iterator>
#include <memory>
#include <mrpt/core/Clock.h>
#include <mrpt/system/datetime.h>
#include <ratio>
#include <sstream> // __str__
#include <string>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mrpt_system_datetime(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::system::InvalidTimeStamp() file:mrpt/system/datetime.h line:46
	M("mrpt::system").def("InvalidTimeStamp", (const mrpt::Clock::time_point & (*)()) &mrpt::system::InvalidTimeStamp, "Required to ensure INVALID_TIMESTAMP returns a \"const T&\"\n  \n\n (New in MRPT 2.3.3)\n\nC++: mrpt::system::InvalidTimeStamp() --> const mrpt::Clock::time_point &", pybind11::return_value_policy::automatic);

	{ // mrpt::system::TTimeParts file:mrpt/system/datetime.h line:59
		pybind11::class_<mrpt::system::TTimeParts, std::shared_ptr<mrpt::system::TTimeParts>> cl(M("mrpt::system"), "TTimeParts", "The parts of a date/time, like the standard `tm` but with fractional\n (`double`) seconds. \n\n TTimeStamp, timestampToParts, buildTimestampFromParts");
		cl.def( pybind11::init( [](){ return new mrpt::system::TTimeParts(); } ) );
		cl.def_readwrite("year", &mrpt::system::TTimeParts::year);
		cl.def_readwrite("month", &mrpt::system::TTimeParts::month);
		cl.def_readwrite("day", &mrpt::system::TTimeParts::day);
		cl.def_readwrite("hour", &mrpt::system::TTimeParts::hour);
		cl.def_readwrite("minute", &mrpt::system::TTimeParts::minute);
		cl.def_readwrite("second", &mrpt::system::TTimeParts::second);
		cl.def_readwrite("day_of_week", &mrpt::system::TTimeParts::day_of_week);
		cl.def_readwrite("daylight_saving", &mrpt::system::TTimeParts::daylight_saving);
	}
	// mrpt::system::buildTimestampFromParts(const struct mrpt::system::TTimeParts &) file:mrpt/system/datetime.h line:74
	M("mrpt::system").def("buildTimestampFromParts", (mrpt::Clock::time_point (*)(const struct mrpt::system::TTimeParts &)) &mrpt::system::buildTimestampFromParts, "Builds a timestamp from the parts (Parts are in UTC)\n \n\n timestampToParts\n\nC++: mrpt::system::buildTimestampFromParts(const struct mrpt::system::TTimeParts &) --> mrpt::Clock::time_point", pybind11::arg("p"));

	// mrpt::system::buildTimestampFromPartsLocalTime(const struct mrpt::system::TTimeParts &) file:mrpt/system/datetime.h line:79
	M("mrpt::system").def("buildTimestampFromPartsLocalTime", (mrpt::Clock::time_point (*)(const struct mrpt::system::TTimeParts &)) &mrpt::system::buildTimestampFromPartsLocalTime, "Builds a timestamp from the parts (Parts are in local time)\n \n\n timestampToParts, buildTimestampFromParts\n\nC++: mrpt::system::buildTimestampFromPartsLocalTime(const struct mrpt::system::TTimeParts &) --> mrpt::Clock::time_point", pybind11::arg("p"));

	// mrpt::system::timestampToParts(mrpt::Clock::time_point, struct mrpt::system::TTimeParts &, bool) file:mrpt/system/datetime.h line:85
	M("mrpt::system").def("timestampToParts", [](mrpt::Clock::time_point const & a0, struct mrpt::system::TTimeParts & a1) -> void { return mrpt::system::timestampToParts(a0, a1); }, "", pybind11::arg("t"), pybind11::arg("p"));
	M("mrpt::system").def("timestampToParts", (void (*)(mrpt::Clock::time_point, struct mrpt::system::TTimeParts &, bool)) &mrpt::system::timestampToParts, "Gets the individual parts of a date/time (days, hours, minutes, seconds) -\n UTC time or local time\n \n\n buildTimestampFromParts\n\nC++: mrpt::system::timestampToParts(mrpt::Clock::time_point, struct mrpt::system::TTimeParts &, bool) --> void", pybind11::arg("t"), pybind11::arg("p"), pybind11::arg("localTime"));

	// mrpt::system::timeDifference(const mrpt::Clock::time_point &, const mrpt::Clock::time_point &) file:mrpt/system/datetime.h line:89
	M("mrpt::system").def("timeDifference", (double (*)(const mrpt::Clock::time_point &, const mrpt::Clock::time_point &)) &mrpt::system::timeDifference, "Returns the time difference from t1 to t2 (positive if t2 is posterior to\n t1), in seconds  \n\nC++: mrpt::system::timeDifference(const mrpt::Clock::time_point &, const mrpt::Clock::time_point &) --> double", pybind11::arg("t_first"), pybind11::arg("t_later"));

	// mrpt::system::timestampAdd(const mrpt::Clock::time_point, const double) file:mrpt/system/datetime.h line:101
	M("mrpt::system").def("timestampAdd", (mrpt::Clock::time_point (*)(const mrpt::Clock::time_point, const double)) &mrpt::system::timestampAdd, "Shifts a timestamp the given amount of seconds (>0: forwards in time, <0:\n backwards)  \n\nC++: mrpt::system::timestampAdd(const mrpt::Clock::time_point, const double) --> mrpt::Clock::time_point", pybind11::arg("tim"), pybind11::arg("num_seconds"));

	// mrpt::system::formatTimeInterval(const double) file:mrpt/system/datetime.h line:111
	M("mrpt::system").def("formatTimeInterval", (std::string (*)(const double)) &mrpt::system::formatTimeInterval, "Returns a formated string with the given time difference (passed as the\n number of seconds), as a string [H]H:MM:SS.MILLISECONDS\n \n\n unitsFormat\n\nC++: mrpt::system::formatTimeInterval(const double) --> std::string", pybind11::arg("timeSeconds"));

	// mrpt::system::dateTimeToString(const mrpt::Clock::time_point) file:mrpt/system/datetime.h line:117
	M("mrpt::system").def("dateTimeToString", (std::string (*)(const mrpt::Clock::time_point)) &mrpt::system::dateTimeToString, "Convert a timestamp into this textual form (UTC time):\n YEAR/MONTH/DAY,HH:MM:SS.MMM\n \n\n dateTimeLocalToString\n\nC++: mrpt::system::dateTimeToString(const mrpt::Clock::time_point) --> std::string", pybind11::arg("t"));

	// mrpt::system::dateTimeLocalToString(const mrpt::Clock::time_point) file:mrpt/system/datetime.h line:123
	M("mrpt::system").def("dateTimeLocalToString", (std::string (*)(const mrpt::Clock::time_point)) &mrpt::system::dateTimeLocalToString, "Convert a timestamp into this textual form (in local time):\n YEAR/MONTH/DAY,HH:MM:SS.MMM\n \n\n dateTimeToString\n\nC++: mrpt::system::dateTimeLocalToString(const mrpt::Clock::time_point) --> std::string", pybind11::arg("t"));

	// mrpt::system::dateToString(const mrpt::Clock::time_point) file:mrpt/system/datetime.h line:127
	M("mrpt::system").def("dateToString", (std::string (*)(const mrpt::Clock::time_point)) &mrpt::system::dateToString, "Convert a timestamp into this textual form: YEAR/MONTH/DAY\n\nC++: mrpt::system::dateToString(const mrpt::Clock::time_point) --> std::string", pybind11::arg("t"));

	// mrpt::system::extractDayTimeFromTimestamp(const mrpt::Clock::time_point) file:mrpt/system/datetime.h line:131
	M("mrpt::system").def("extractDayTimeFromTimestamp", (double (*)(const mrpt::Clock::time_point)) &mrpt::system::extractDayTimeFromTimestamp, "Returns the number of seconds ellapsed from midnight in the given timestamp\n\nC++: mrpt::system::extractDayTimeFromTimestamp(const mrpt::Clock::time_point) --> double", pybind11::arg("t"));

	// mrpt::system::timeToString(const mrpt::Clock::time_point) file:mrpt/system/datetime.h line:135
	M("mrpt::system").def("timeToString", (std::string (*)(const mrpt::Clock::time_point)) &mrpt::system::timeToString, "Convert a timestamp into this textual form (UTC): HH:MM:SS.MMMMMM\n\nC++: mrpt::system::timeToString(const mrpt::Clock::time_point) --> std::string", pybind11::arg("t"));

	// mrpt::system::timeLocalToString(const mrpt::Clock::time_point, unsigned int) file:mrpt/system/datetime.h line:139
	M("mrpt::system").def("timeLocalToString", [](const mrpt::Clock::time_point & a0) -> std::string { return mrpt::system::timeLocalToString(a0); }, "", pybind11::arg("t"));
	M("mrpt::system").def("timeLocalToString", (std::string (*)(const mrpt::Clock::time_point, unsigned int)) &mrpt::system::timeLocalToString, "Convert a timestamp into this textual form (in local time): HH:MM:SS.MMMMMM\n\nC++: mrpt::system::timeLocalToString(const mrpt::Clock::time_point, unsigned int) --> std::string", pybind11::arg("t"), pybind11::arg("secondFractionDigits"));

	// mrpt::system::intervalFormat(const double) file:mrpt/system/datetime.h line:155
	M("mrpt::system").def("intervalFormat", (std::string (*)(const double)) &mrpt::system::intervalFormat, "This function implements time interval formatting: Given a time in seconds,\n it will return a string describing the interval with the most appropriate\n unit.\n E.g.:\n  - \"1 year, 3 days, 4 minutes\"\n  - \"3 days, 8 hours\"\n  - \"9 hours, 4 minutes, 4.3 sec\",\n  - \"3.34 sec\"\n  - \"178.1 ms\"\n  - \"87.1 us\"\n\n \n unitsFormat\n\nC++: mrpt::system::intervalFormat(const double) --> std::string", pybind11::arg("seconds"));

}
