#include <chrono>
#include <mrpt/core/Clock.h>
#include <ratio>
#include <sstream> // __str__

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

void bind_mrpt_core_Clock(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::Clock file:mrpt/core/Clock.h line:21
		pybind11::class_<mrpt::Clock, std::shared_ptr<mrpt::Clock>> cl(M("mrpt"), "Clock", "C++11-clock that is compatible with MRPT TTimeStamp representation\n\n State-aware methods in this class are thread-safe.\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::Clock(); } ) );

		pybind11::enum_<mrpt::Clock::Source>(cl, "Source", pybind11::arithmetic(), "Options for setting the source of all timestamps across MRPT:\n setActiveClock(), now()")
			.value("Realtime", mrpt::Clock::Realtime)
			.value("Monotonic", mrpt::Clock::Monotonic)
			.value("Simulated", mrpt::Clock::Simulated)
			.export_values();

		cl.def_static("now", (mrpt::Clock::time_point (*)()) &mrpt::Clock::now, "Returns the current time using the currently selected Clock source.\n  (Performance: call typ. takes 33 nanoseconds)\n \n\n setActiveClock(), mrpt::Clock::Source, nowDouble()  \n\nC++: mrpt::Clock::now() --> mrpt::Clock::time_point");
		cl.def_static("nowDouble", (double (*)()) &mrpt::Clock::nowDouble, "Equivalent to `Clock::toDouble(Clock::now())`.\n  (Performance: call typ. takes 38 nanoseconds)\n \n\n setActiveClock(), mrpt::Clock::Source, now()\n \n\n (New in MRPT 2.1.5)\n\nC++: mrpt::Clock::nowDouble() --> double");
		cl.def_static("fromDouble", (mrpt::Clock::time_point (*)(const double)) &mrpt::Clock::fromDouble, "Create a timestamp from its double representation. \n toDouble \n\nC++: mrpt::Clock::fromDouble(const double) --> mrpt::Clock::time_point", pybind11::arg("t"));
		cl.def_static("toDouble", (double (*)(const mrpt::Clock::time_point)) &mrpt::Clock::toDouble, "Converts a timestamp to a UNIX time_t-like number, with fractional part.\n  If t is an invalid (default-contructed) time_point, `0.0` will be\n returned.\n \n\n fromDouble \n\nC++: mrpt::Clock::toDouble(const mrpt::Clock::time_point) --> double", pybind11::arg("t"));
		cl.def_static("setActiveClock", (void (*)(const enum mrpt::Clock::Source)) &mrpt::Clock::setActiveClock, "Changes the selected clock to get time from when calling now().\n Default: Realtime.\n\n Monotonic is only available on Linux systems.\n RealTime and Simulated are available on any platform.\n\n It is strongly recommended to call setSimulatedTime()\n before setting the clock source to Simulated to ensure that any\n subsequent call to now(), perhaps in a parallel thread, does not\n return an undefined time_point value.\n\nC++: mrpt::Clock::setActiveClock(const enum mrpt::Clock::Source) --> void", pybind11::arg("s"));
		cl.def_static("getActiveClock", (enum mrpt::Clock::Source (*)()) &mrpt::Clock::getActiveClock, "Returns the currently selected clock. \n\nC++: mrpt::Clock::getActiveClock() --> enum mrpt::Clock::Source");
		cl.def_static("resetMonotonicToRealTimeEpoch", (int64_t (*)()) &mrpt::Clock::resetMonotonicToRealTimeEpoch, "Monotonic clock *might* drift over time with respect to Realtime.\n The first time a time is requested to now() with Monotonic clock enabled,\n MRPT internally saves the current values of both, Realtime and Monotonic\n clocks, then use the difference in all subsequent calls to set Monotonic\n timepoints in the same epoch than Realtime.\n\n By explicitly calling this static method, a user is able to force a\n resynchrnization between the two clocks. Potentially useful for systems\n that run for very long periods of time without interruption.\n\n \n The mismatch between the former and the new estimations of the\n epochs differences between the two clocks, in units of nanoseconds.\n\n \n This method is ignored in non-Linux systems.\n\nC++: mrpt::Clock::resetMonotonicToRealTimeEpoch() --> int64_t");
		cl.def_static("getMonotonicToRealtimeOffset", (uint64_t (*)()) &mrpt::Clock::getMonotonicToRealtimeOffset, "Returns the number of nanoseconds that are added to the output of the\n POSIX `CLOCK_MONOTONIC` to make timestamps match the epoch of POSIX\n `CLOCK_REALTIME`. \n\nC++: mrpt::Clock::getMonotonicToRealtimeOffset() --> uint64_t");
		cl.def_static("setSimulatedTime", (void (*)(const mrpt::Clock::time_point &)) &mrpt::Clock::setSimulatedTime, "When setActiveClock() is set to `Simulated`, sets the simulated time\n that will be returned in subsequent calls to now().\n [New in MRPT 2.1.1]\n\nC++: mrpt::Clock::setSimulatedTime(const mrpt::Clock::time_point &) --> void", pybind11::arg("t"));
	}
}
