#include <iterator>
#include <memory>
#include <mrpt/core/backtrace.h>
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

void bind_mrpt_core_backtrace(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::TCallStackEntry file:mrpt/core/backtrace.h line:20
		pybind11::class_<mrpt::TCallStackEntry, std::shared_ptr<mrpt::TCallStackEntry>> cl(M("mrpt"), "TCallStackEntry", "Used in callStackBackTrace() ");
		cl.def( pybind11::init( [](){ return new mrpt::TCallStackEntry(); } ) );
		cl.def( pybind11::init( [](mrpt::TCallStackEntry const &o){ return new mrpt::TCallStackEntry(o); } ) );
		cl.def_readwrite("symbolName", &mrpt::TCallStackEntry::symbolName);
		cl.def_readwrite("symbolNameOriginal", &mrpt::TCallStackEntry::symbolNameOriginal);
		cl.def_readwrite("sourceFileName", &mrpt::TCallStackEntry::sourceFileName);
		cl.def_readwrite("sourceFileFullPath", &mrpt::TCallStackEntry::sourceFileFullPath);
		cl.def_readwrite("sourceFileNumber", &mrpt::TCallStackEntry::sourceFileNumber);
		cl.def("assign", (struct mrpt::TCallStackEntry & (mrpt::TCallStackEntry::*)(const struct mrpt::TCallStackEntry &)) &mrpt::TCallStackEntry::operator=, "C++: mrpt::TCallStackEntry::operator=(const struct mrpt::TCallStackEntry &) --> struct mrpt::TCallStackEntry &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::TCallStackBackTrace file:mrpt/core/backtrace.h line:41
		pybind11::class_<mrpt::TCallStackBackTrace, std::shared_ptr<mrpt::TCallStackBackTrace>> cl(M("mrpt"), "TCallStackBackTrace", "See: callStackBackTrace() ");
		cl.def( pybind11::init( [](){ return new mrpt::TCallStackBackTrace(); } ) );
		cl.def( pybind11::init( [](mrpt::TCallStackBackTrace const &o){ return new mrpt::TCallStackBackTrace(o); } ) );
		cl.def_readwrite("backtrace_levels", &mrpt::TCallStackBackTrace::backtrace_levels);
		cl.def("asString", (std::string (mrpt::TCallStackBackTrace::*)() const) &mrpt::TCallStackBackTrace::asString, "Prints all backtrace entries, one per line in a human-readable format.\n See [environment variables](env-vars.html) that can change the output\n format.\n\nC++: mrpt::TCallStackBackTrace::asString() const --> std::string");
		cl.def("assign", (struct mrpt::TCallStackBackTrace & (mrpt::TCallStackBackTrace::*)(const struct mrpt::TCallStackBackTrace &)) &mrpt::TCallStackBackTrace::operator=, "C++: mrpt::TCallStackBackTrace::operator=(const struct mrpt::TCallStackBackTrace &) --> struct mrpt::TCallStackBackTrace &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	// mrpt::callStackBackTrace(struct mrpt::TCallStackBackTrace &, const unsigned int, const unsigned int) file:mrpt/core/backtrace.h line:63
	M("mrpt").def("callStackBackTrace", [](struct mrpt::TCallStackBackTrace & a0) -> void { return mrpt::callStackBackTrace(a0); }, "", pybind11::arg("out_bt"));
	M("mrpt").def("callStackBackTrace", [](struct mrpt::TCallStackBackTrace & a0, const unsigned int & a1) -> void { return mrpt::callStackBackTrace(a0, a1); }, "", pybind11::arg("out_bt"), pybind11::arg("framesToSkip"));
	M("mrpt").def("callStackBackTrace", (void (*)(struct mrpt::TCallStackBackTrace &, const unsigned int, const unsigned int)) &mrpt::callStackBackTrace, "Returns a list of strings representing the current call stack\n backtrace. If possible, human-readable names are used for\n functions. Source code line numbers will be also recovered\n if code has symbols (`-g` or `-g1` in GCC).\n\n See [environment variables](env-vars.html) that can modify the behavior\n of this function.\n\n \n (Moved from mrpt-system to mrpt-core in MRPT 2.1.5)\n\nC++: mrpt::callStackBackTrace(struct mrpt::TCallStackBackTrace &, const unsigned int, const unsigned int) --> void", pybind11::arg("out_bt"), pybind11::arg("framesToSkip"), pybind11::arg("framesToCapture"));

	// mrpt::callStackBackTrace(const unsigned int, const unsigned int) file:mrpt/core/backtrace.h line:69
	M("mrpt").def("callStackBackTrace", []() -> mrpt::TCallStackBackTrace { return mrpt::callStackBackTrace(); }, "");
	M("mrpt").def("callStackBackTrace", [](const unsigned int & a0) -> mrpt::TCallStackBackTrace { return mrpt::callStackBackTrace(a0); }, "", pybind11::arg("framesToSkip"));
	M("mrpt").def("callStackBackTrace", (struct mrpt::TCallStackBackTrace (*)(const unsigned int, const unsigned int)) &mrpt::callStackBackTrace, "C++: mrpt::callStackBackTrace(const unsigned int, const unsigned int) --> struct mrpt::TCallStackBackTrace", pybind11::arg("framesToSkip"), pybind11::arg("framesToCapture"));

}
