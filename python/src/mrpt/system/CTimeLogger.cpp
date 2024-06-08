#include <iterator>
#include <memory>
#include <mrpt/system/CTimeLogger.h>
#include <sstream> // __str__
#include <string>
#include <string_view>

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

void bind_mrpt_system_CTimeLogger(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::system::CTimeLogger file:mrpt/system/CTimeLogger.h line:51
		pybind11::class_<mrpt::system::CTimeLogger, std::shared_ptr<mrpt::system::CTimeLogger>> cl(M("mrpt::system"), "CTimeLogger", "A versatile \"profiler\" that logs the time spent within each pair of calls to\n enter(X)-leave(X), among other stats.\n  The results can be dumped to cout or to Visual Studio's output panel.\n This class can be also used to monitorize min/mean/max/total stats of any\n user-provided parameters via the method CTimeLogger::registerUserMeasure().\n\n Optional recording of **all** data can be enabled via\n enableKeepWholeHistory() (use with caution!).\n\n Cost of the profiler itself (measured on MSVC2015, Windows 10, Intel i5-2310\n 2.9GHz):\n - `enter()`: average 445 ns\n - `leave()`: average 316 ns\n\n  Recursive methods are supported with no problems, that is, calling \"enter(X)\n enter(X) ... leave(X) leave(X)\".\n  `enter()`/`leave()` are thread-safe, in the sense of they being safe to be\n called from different threads. However, calling `enter()`/`leave()` for the\n same user-supplied \"section name\", from different threads, is not allowed. In\n the latter case (and, actually, in general since it's safer against\n exceptions), use the RAII helper class CTimeLoggerEntry.\n\n \n CTimeLoggerEntry\n\n \n The default behavior is dumping all the information at destruction.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::system::CTimeLogger(); } ), "doc" );
		cl.def( pybind11::init( [](bool const & a0){ return new mrpt::system::CTimeLogger(a0); } ), "doc" , pybind11::arg("enabled"));
		cl.def( pybind11::init( [](bool const & a0, const std::string & a1){ return new mrpt::system::CTimeLogger(a0, a1); } ), "doc" , pybind11::arg("enabled"), pybind11::arg("name"));
		cl.def( pybind11::init<bool, const std::string &, const bool>(), pybind11::arg("enabled"), pybind11::arg("name"), pybind11::arg("keep_whole_history") );

		cl.def( pybind11::init( [](mrpt::system::CTimeLogger const &o){ return new mrpt::system::CTimeLogger(o); } ) );
		cl.def("assign", (class mrpt::system::CTimeLogger & (mrpt::system::CTimeLogger::*)(const class mrpt::system::CTimeLogger &)) &mrpt::system::CTimeLogger::operator=, "C++: mrpt::system::CTimeLogger::operator=(const class mrpt::system::CTimeLogger &) --> class mrpt::system::CTimeLogger &", pybind11::return_value_policy::automatic, pybind11::arg("o"));
		cl.def("getStatsAsText", [](mrpt::system::CTimeLogger const &o) -> std::string { return o.getStatsAsText(); }, "");
		cl.def("getStatsAsText", (std::string (mrpt::system::CTimeLogger::*)(size_t) const) &mrpt::system::CTimeLogger::getStatsAsText, "Dump all stats to a multi-line text string. \n dumpAllStats,\n saveToCVSFile \n\nC++: mrpt::system::CTimeLogger::getStatsAsText(size_t) const --> std::string", pybind11::arg("column_width"));
		cl.def("dumpAllStats", [](mrpt::system::CTimeLogger const &o) -> void { return o.dumpAllStats(); }, "");
		cl.def("dumpAllStats", (void (mrpt::system::CTimeLogger::*)(size_t) const) &mrpt::system::CTimeLogger::dumpAllStats, "Dump all stats through the COutputLogger interface. \n getStatsAsText,\n saveToCVSFile \n\nC++: mrpt::system::CTimeLogger::dumpAllStats(size_t) const --> void", pybind11::arg("column_width"));
		cl.def("clear", [](mrpt::system::CTimeLogger &o) -> void { return o.clear(); }, "");
		cl.def("clear", (void (mrpt::system::CTimeLogger::*)(bool)) &mrpt::system::CTimeLogger::clear, "Resets all stats. By default (deep_clear=false), all section names are\n remembered (not freed) so the cost of creating upon the first next call\n is avoided.\n\n \n By design, calling this method is the only one which is not thread\n safe. It's not made thread-safe to save the performance cost. Please,\n ensure that you call `clear()` only while there are no other threads\n registering annotations in the object.\n\nC++: mrpt::system::CTimeLogger::clear(bool) --> void", pybind11::arg("deep_clear"));
		cl.def("enable", [](mrpt::system::CTimeLogger &o) -> void { return o.enable(); }, "");
		cl.def("enable", (void (mrpt::system::CTimeLogger::*)(bool)) &mrpt::system::CTimeLogger::enable, "C++: mrpt::system::CTimeLogger::enable(bool) --> void", pybind11::arg("enabled"));
		cl.def("disable", (void (mrpt::system::CTimeLogger::*)()) &mrpt::system::CTimeLogger::disable, "C++: mrpt::system::CTimeLogger::disable() --> void");
		cl.def("isEnabled", (bool (mrpt::system::CTimeLogger::*)() const) &mrpt::system::CTimeLogger::isEnabled, "C++: mrpt::system::CTimeLogger::isEnabled() const --> bool");
		cl.def("enableKeepWholeHistory", [](mrpt::system::CTimeLogger &o) -> void { return o.enableKeepWholeHistory(); }, "");
		cl.def("enableKeepWholeHistory", (void (mrpt::system::CTimeLogger::*)(bool)) &mrpt::system::CTimeLogger::enableKeepWholeHistory, "C++: mrpt::system::CTimeLogger::enableKeepWholeHistory(bool) --> void", pybind11::arg("enable"));
		cl.def("isEnabledKeepWholeHistory", (bool (mrpt::system::CTimeLogger::*)() const) &mrpt::system::CTimeLogger::isEnabledKeepWholeHistory, "C++: mrpt::system::CTimeLogger::isEnabledKeepWholeHistory() const --> bool");
		cl.def("saveToCSVFile", (void (mrpt::system::CTimeLogger::*)(const std::string &) const) &mrpt::system::CTimeLogger::saveToCSVFile, "Dump all stats to a Comma Separated Values (CSV) file. \n dumpAllStats\n\nC++: mrpt::system::CTimeLogger::saveToCSVFile(const std::string &) const --> void", pybind11::arg("csv_file"));
		cl.def("saveToMFile", (void (mrpt::system::CTimeLogger::*)(const std::string &) const) &mrpt::system::CTimeLogger::saveToMFile, "Dump all stats to a Matlab/Octave (.m) file. \n dumpAllStats \n\nC++: mrpt::system::CTimeLogger::saveToMFile(const std::string &) const --> void", pybind11::arg("m_file"));
		cl.def("getName", (const std::string & (mrpt::system::CTimeLogger::*)() const) &mrpt::system::CTimeLogger::getName, "C++: mrpt::system::CTimeLogger::getName() const --> const std::string &", pybind11::return_value_policy::automatic);
		cl.def("setName", (void (mrpt::system::CTimeLogger::*)(const std::string &)) &mrpt::system::CTimeLogger::setName, "C++: mrpt::system::CTimeLogger::setName(const std::string &) --> void", pybind11::arg("name"));
		cl.def("getMeanTime", (double (mrpt::system::CTimeLogger::*)(const std::string &) const) &mrpt::system::CTimeLogger::getMeanTime, "Return the mean execution time of the given \"section\", or 0 if it hasn't\n ever been called \"enter\" with that section name \n\nC++: mrpt::system::CTimeLogger::getMeanTime(const std::string &) const --> double", pybind11::arg("name"));
		cl.def("getLastTime", (double (mrpt::system::CTimeLogger::*)(const std::string &) const) &mrpt::system::CTimeLogger::getLastTime, "Return the last execution time of the given \"section\", or 0 if it hasn't\n ever been called \"enter\" with that section name \n\nC++: mrpt::system::CTimeLogger::getLastTime(const std::string &) const --> double", pybind11::arg("name"));

		{ // mrpt::system::CTimeLogger::TCallStats file:mrpt/system/CTimeLogger.h line:117
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::system::CTimeLogger::TCallStats, std::shared_ptr<mrpt::system::CTimeLogger::TCallStats>> cl(enclosing_class, "TCallStats", "Data of each call section: # of calls, minimum, maximum, average and\n overall execution time (in seconds) \n\n getStats ");
			cl.def( pybind11::init( [](){ return new mrpt::system::CTimeLogger::TCallStats(); } ) );
			cl.def_readwrite("n_calls", &mrpt::system::CTimeLogger::TCallStats::n_calls);
			cl.def_readwrite("min_t", &mrpt::system::CTimeLogger::TCallStats::min_t);
			cl.def_readwrite("max_t", &mrpt::system::CTimeLogger::TCallStats::max_t);
			cl.def_readwrite("mean_t", &mrpt::system::CTimeLogger::TCallStats::mean_t);
			cl.def_readwrite("total_t", &mrpt::system::CTimeLogger::TCallStats::total_t);
			cl.def_readwrite("last_t", &mrpt::system::CTimeLogger::TCallStats::last_t);
		}

	}
	{ // mrpt::system::CTimeLoggerEntry file:mrpt/system/CTimeLogger.h line:226
		pybind11::class_<mrpt::system::CTimeLoggerEntry, std::shared_ptr<mrpt::system::CTimeLoggerEntry>> cl(M("mrpt::system"), "CTimeLoggerEntry", "A safe way to call enter() and leave() of a mrpt::system::CTimeLogger upon\n construction and destruction of\n this auxiliary object, making sure that leave() will be called upon\n exceptions, etc.\n Usage mode #1 (scoped):\n \n\n\n\n\n\n\n\n\n\n\n\n Usage mode #2 (unscoped):\n \n\n\n\n\n\n\n\n\n\n\n \n\n ");
		cl.def( pybind11::init( [](mrpt::system::CTimeLoggerEntry const &o){ return new mrpt::system::CTimeLoggerEntry(o); } ) );
		cl.def("stop", (void (mrpt::system::CTimeLoggerEntry::*)()) &mrpt::system::CTimeLoggerEntry::stop, "C++: mrpt::system::CTimeLoggerEntry::stop() --> void");
	}
	{ // mrpt::system::CTimeLoggerSaveAtDtor file:mrpt/system/CTimeLogger.h line:245
		pybind11::class_<mrpt::system::CTimeLoggerSaveAtDtor, std::shared_ptr<mrpt::system::CTimeLoggerSaveAtDtor>> cl(M("mrpt::system"), "CTimeLoggerSaveAtDtor", "A helper class to save CSV stats upon self destruction, for example, at the\n end of a program run. The target file will be named after timelogger's name.\n \n\n\n ");
		cl.def( pybind11::init<class mrpt::system::CTimeLogger &>(), pybind11::arg("tm") );

	}
	// mrpt::system::global_profiler_enter(const char *) file:mrpt/system/CTimeLogger.h line:256
	M("mrpt::system").def("global_profiler_enter", (void (*)(const char *)) &mrpt::system::global_profiler_enter, "macros.\n  @{ \n\nC++: mrpt::system::global_profiler_enter(const char *) --> void", pybind11::arg("func_name"));

	// mrpt::system::global_profiler_leave(const char *) file:mrpt/system/CTimeLogger.h line:257
	M("mrpt::system").def("global_profiler_leave", (void (*)(const char *)) &mrpt::system::global_profiler_leave, "C++: mrpt::system::global_profiler_leave(const char *) --> void", pybind11::arg("func_name"));

	// mrpt::system::global_profiler_getref() file:mrpt/system/CTimeLogger.h line:258
	M("mrpt::system").def("global_profiler_getref", (class mrpt::system::CTimeLogger & (*)()) &mrpt::system::global_profiler_getref, "C++: mrpt::system::global_profiler_getref() --> class mrpt::system::CTimeLogger &", pybind11::return_value_policy::automatic);

}
