#include <deque>
#include <iterator>
#include <locale>
#include <memory>
#include <mrpt/system/CConsoleRedirector.h>
#include <mrpt/system/CControlledRateTimer.h>
#include <mrpt/system/CDirectoryExplorer.h>
#include <mrpt/system/CFileSystemWatcher.h>
#include <mrpt/system/CRateTimer.h>
#include <mrpt/system/WorkerThreadsPool.h>
#include <sstream> // __str__
#include <streambuf>
#include <string>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <stl_binders.hpp>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

// mrpt::system::CConsoleRedirector file:mrpt/system/CConsoleRedirector.h line:28
struct PyCallBack_mrpt_system_CConsoleRedirector : public mrpt::system::CConsoleRedirector {
	using mrpt::system::CConsoleRedirector::CConsoleRedirector;

	void writeString(const std::string & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::system::CConsoleRedirector *>(this), "writeString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CConsoleRedirector::writeString(a0);
	}
	void imbue(const class std::locale & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::system::CConsoleRedirector *>(this), "imbue");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return basic_streambuf::imbue(a0);
	}
	int sync() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::system::CConsoleRedirector *>(this), "sync");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<int>::value) {
				static pybind11::detail::override_caster_t<int> caster;
				return pybind11::detail::cast_ref<int>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<int>(std::move(o));
		}
		return basic_streambuf::sync();
	}
	long showmanyc() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::system::CConsoleRedirector *>(this), "showmanyc");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<long>::value) {
				static pybind11::detail::override_caster_t<long> caster;
				return pybind11::detail::cast_ref<long>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<long>(std::move(o));
		}
		return basic_streambuf::showmanyc();
	}
	long xsgetn(char * a0, long a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::system::CConsoleRedirector *>(this), "xsgetn");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<long>::value) {
				static pybind11::detail::override_caster_t<long> caster;
				return pybind11::detail::cast_ref<long>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<long>(std::move(o));
		}
		return basic_streambuf::xsgetn(a0, a1);
	}
	int underflow() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::system::CConsoleRedirector *>(this), "underflow");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<int>::value) {
				static pybind11::detail::override_caster_t<int> caster;
				return pybind11::detail::cast_ref<int>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<int>(std::move(o));
		}
		return basic_streambuf::underflow();
	}
	int uflow() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::system::CConsoleRedirector *>(this), "uflow");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<int>::value) {
				static pybind11::detail::override_caster_t<int> caster;
				return pybind11::detail::cast_ref<int>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<int>(std::move(o));
		}
		return basic_streambuf::uflow();
	}
	int pbackfail(int a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::system::CConsoleRedirector *>(this), "pbackfail");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<int>::value) {
				static pybind11::detail::override_caster_t<int> caster;
				return pybind11::detail::cast_ref<int>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<int>(std::move(o));
		}
		return basic_streambuf::pbackfail(a0);
	}
	long xsputn(const char * a0, long a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::system::CConsoleRedirector *>(this), "xsputn");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<long>::value) {
				static pybind11::detail::override_caster_t<long> caster;
				return pybind11::detail::cast_ref<long>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<long>(std::move(o));
		}
		return basic_streambuf::xsputn(a0, a1);
	}
	int overflow(int a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::system::CConsoleRedirector *>(this), "overflow");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<int>::value) {
				static pybind11::detail::override_caster_t<int> caster;
				return pybind11::detail::cast_ref<int>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<int>(std::move(o));
		}
		return basic_streambuf::overflow(a0);
	}
};

void bind_mrpt_system_CConsoleRedirector(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::system::CConsoleRedirector file:mrpt/system/CConsoleRedirector.h line:28
		pybind11::class_<mrpt::system::CConsoleRedirector, std::shared_ptr<mrpt::system::CConsoleRedirector>, PyCallBack_mrpt_system_CConsoleRedirector> cl(M("mrpt::system"), "CConsoleRedirector", "By creating an object of this class, all the output to std::cout (and\n std::cerr) will be redirected to a text file, and optionally also shown on\n the console.\n  Based on code from http://www.devmaster.net/forums/showthread.php?t=7037\n \n\n\n ");
		cl.def( pybind11::init( [](const std::string & a0){ return new mrpt::system::CConsoleRedirector(a0); }, [](const std::string & a0){ return new PyCallBack_mrpt_system_CConsoleRedirector(a0); } ), "doc");
		cl.def( pybind11::init( [](const std::string & a0, bool const & a1){ return new mrpt::system::CConsoleRedirector(a0, a1); }, [](const std::string & a0, bool const & a1){ return new PyCallBack_mrpt_system_CConsoleRedirector(a0, a1); } ), "doc");
		cl.def( pybind11::init( [](const std::string & a0, bool const & a1, bool const & a2){ return new mrpt::system::CConsoleRedirector(a0, a1, a2); }, [](const std::string & a0, bool const & a1, bool const & a2){ return new PyCallBack_mrpt_system_CConsoleRedirector(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](const std::string & a0, bool const & a1, bool const & a2, bool const & a3){ return new mrpt::system::CConsoleRedirector(a0, a1, a2, a3); }, [](const std::string & a0, bool const & a1, bool const & a2, bool const & a3){ return new PyCallBack_mrpt_system_CConsoleRedirector(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init<const std::string &, bool, bool, bool, int>(), pybind11::arg("out_file"), pybind11::arg("also_to_console"), pybind11::arg("also_cerr"), pybind11::arg("append_file"), pybind11::arg("bufferSize") );

		cl.def("flush", (void (mrpt::system::CConsoleRedirector::*)()) &mrpt::system::CConsoleRedirector::flush, "C++: mrpt::system::CConsoleRedirector::flush() --> void");
		cl.def("writeString", (void (mrpt::system::CConsoleRedirector::*)(const std::string &)) &mrpt::system::CConsoleRedirector::writeString, "C++: mrpt::system::CConsoleRedirector::writeString(const std::string &) --> void", pybind11::arg("str"));
	}
	{ // mrpt::system::CRateTimer file:mrpt/system/CRateTimer.h line:19
		pybind11::class_<mrpt::system::CRateTimer, std::shared_ptr<mrpt::system::CRateTimer>> cl(M("mrpt::system"), "CRateTimer", "A class for calling sleep() in a loop, such that the amount of sleep time\n will be computed to make the loop run at the desired rate (in Hz).\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::system::CRateTimer(); } ), "doc" );
		cl.def( pybind11::init<const double>(), pybind11::arg("rate_hz") );

		cl.def( pybind11::init( [](mrpt::system::CRateTimer const &o){ return new mrpt::system::CRateTimer(o); } ) );
		cl.def("setRate", (void (mrpt::system::CRateTimer::*)(const double)) &mrpt::system::CRateTimer::setRate, "Changes the object loop rate (Hz) \n\nC++: mrpt::system::CRateTimer::setRate(const double) --> void", pybind11::arg("rate_hz"));
		cl.def("rate", (double (mrpt::system::CRateTimer::*)() const) &mrpt::system::CRateTimer::rate, "Gets current rate (Hz) \n\nC++: mrpt::system::CRateTimer::rate() const --> double");
		cl.def("sleep", (bool (mrpt::system::CRateTimer::*)()) &mrpt::system::CRateTimer::sleep, "Sleeps for some time, such as the return of this method is 1/rate\n (seconds)\n after the return of the previous call.\n \n\n false if the rate could not be achieved (\"we are already late\"),\n true if all went right. \n\nC++: mrpt::system::CRateTimer::sleep() --> bool");
		cl.def("assign", (class mrpt::system::CRateTimer & (mrpt::system::CRateTimer::*)(const class mrpt::system::CRateTimer &)) &mrpt::system::CRateTimer::operator=, "C++: mrpt::system::CRateTimer::operator=(const class mrpt::system::CRateTimer &) --> class mrpt::system::CRateTimer &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::system::CControlledRateTimer file:mrpt/system/CControlledRateTimer.h line:47
		pybind11::class_<mrpt::system::CControlledRateTimer, std::shared_ptr<mrpt::system::CControlledRateTimer>> cl(M("mrpt::system"), "CControlledRateTimer", "A class for calling sleep() in a loop, such that the amount of sleep time\n will be computed to make the loop run at the desired rate (in Hz).\n This class implements a PI controller on top of a vanilla CRateTimer object,\n ensuring a high accuracy in achieved execution rates. Note that this is done\n by setting a slightly-higher rate (\"control action\") to the internal\n CRateTimer, such that the error between the user-provided expected rate and\n the actual measured rate (low-pass filtered) is decreased by means of a PI\n controller.\n\n Note that rates higher than a few kHz are not attainable in all CPUs and/or\n kernel versions. Find below some graphs illustrating how this class tries to\n achieve a constant setpoint rate (given by the user), reacting to changes in\n the setpoint values:\n\n  ![controlled rate timer plots](CControlledRateTimer_example.png)\n\n This graphs is generated with the example:\n\n `system_control_rate_timer_example --rate1 2000.0 --rate2 4000.0`\n\n All the parameters for the PI controller and low-pass filter (rate estimator)\n are settable by the user to adapt them to specific needs.\n\n \n Control law by [Francisco Jose Mañas\n Alvarez](https://github.com/FranciscoJManasAlvarez)\n\n \n [New in MRPT 2.0.4]\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::system::CControlledRateTimer(); } ), "doc" );
		cl.def( pybind11::init<const double>(), pybind11::arg("rate_hz") );

		cl.def( pybind11::init( [](mrpt::system::CControlledRateTimer const &o){ return new mrpt::system::CControlledRateTimer(o); } ) );
		cl.def("setRate", (void (mrpt::system::CControlledRateTimer::*)(const double)) &mrpt::system::CControlledRateTimer::setRate, "Changes the object loop rate (Hz) \n\nC++: mrpt::system::CControlledRateTimer::setRate(const double) --> void", pybind11::arg("rate_hz"));
		cl.def("sleep", (bool (mrpt::system::CControlledRateTimer::*)()) &mrpt::system::CControlledRateTimer::sleep, "Sleeps for some time, such as the return of this method is 1/rate\n (seconds)\n after the return of the previous call.\n \n\n false if the rate could not be achieved (\"we are already late\"),\n true if all went right. \n\nC++: mrpt::system::CControlledRateTimer::sleep() --> bool");
		cl.def("controllerParam_Kp", (double (mrpt::system::CControlledRateTimer::*)() const) &mrpt::system::CControlledRateTimer::controllerParam_Kp, "PI controller Kp parameter [default=1.0] \n\nC++: mrpt::system::CControlledRateTimer::controllerParam_Kp() const --> double");
		cl.def("controllerParam_Kp", (void (mrpt::system::CControlledRateTimer::*)(double)) &mrpt::system::CControlledRateTimer::controllerParam_Kp, "C++: mrpt::system::CControlledRateTimer::controllerParam_Kp(double) --> void", pybind11::arg("v"));
		cl.def("controllerParam_Ti", (double (mrpt::system::CControlledRateTimer::*)() const) &mrpt::system::CControlledRateTimer::controllerParam_Ti, "PI controller Ti parameter [default=0.0194] \n\nC++: mrpt::system::CControlledRateTimer::controllerParam_Ti() const --> double");
		cl.def("controllerParam_Ti", (void (mrpt::system::CControlledRateTimer::*)(double)) &mrpt::system::CControlledRateTimer::controllerParam_Ti, "C++: mrpt::system::CControlledRateTimer::controllerParam_Ti(double) --> void", pybind11::arg("v"));
		cl.def("lowPassParam_a0", (double (mrpt::system::CControlledRateTimer::*)() const) &mrpt::system::CControlledRateTimer::lowPassParam_a0, "Low-pass filter a0 value [default=0.9]:\n estimation = a0*input + (1-a0)*former_estimation \n\nC++: mrpt::system::CControlledRateTimer::lowPassParam_a0() const --> double");
		cl.def("lowPassParam_a0", (void (mrpt::system::CControlledRateTimer::*)(double)) &mrpt::system::CControlledRateTimer::lowPassParam_a0, "C++: mrpt::system::CControlledRateTimer::lowPassParam_a0(double) --> void", pybind11::arg("v"));
		cl.def("followErrorRatioToRaiseWarning", (double (mrpt::system::CControlledRateTimer::*)() const) &mrpt::system::CControlledRateTimer::followErrorRatioToRaiseWarning, "Get/set ratio threshold for issuing a warning (via COutputLogger\n interface) if the achieved rate is not this close to the set-point\n [Default=0.2, =20%]\n\nC++: mrpt::system::CControlledRateTimer::followErrorRatioToRaiseWarning() const --> double");
		cl.def("followErrorRatioToRaiseWarning", (void (mrpt::system::CControlledRateTimer::*)(double)) &mrpt::system::CControlledRateTimer::followErrorRatioToRaiseWarning, "C++: mrpt::system::CControlledRateTimer::followErrorRatioToRaiseWarning(double) --> void", pybind11::arg("v"));
		cl.def("actualControlledRate", (double (mrpt::system::CControlledRateTimer::*)() const) &mrpt::system::CControlledRateTimer::actualControlledRate, "Gets the actual controller output: the rate (Hz) of the internal\n CRateTimer object. \n\nC++: mrpt::system::CControlledRateTimer::actualControlledRate() const --> double");
		cl.def("estimatedRate", (double (mrpt::system::CControlledRateTimer::*)() const) &mrpt::system::CControlledRateTimer::estimatedRate, "Gets the latest estimated run rate (Hz), which comes from actual period\n measurement, low-pass filtered. \n\nC++: mrpt::system::CControlledRateTimer::estimatedRate() const --> double");
		cl.def("estimatedRateRaw", (double (mrpt::system::CControlledRateTimer::*)() const) &mrpt::system::CControlledRateTimer::estimatedRateRaw, "Last actual execution rate measured (Hz), without low-pass filtering \n\nC++: mrpt::system::CControlledRateTimer::estimatedRateRaw() const --> double");
		cl.def("assign", (class mrpt::system::CControlledRateTimer & (mrpt::system::CControlledRateTimer::*)(const class mrpt::system::CControlledRateTimer &)) &mrpt::system::CControlledRateTimer::operator=, "C++: mrpt::system::CControlledRateTimer::operator=(const class mrpt::system::CControlledRateTimer &) --> class mrpt::system::CControlledRateTimer &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::system::CDirectoryExplorer file:mrpt/system/CDirectoryExplorer.h line:29
		pybind11::class_<mrpt::system::CDirectoryExplorer, std::shared_ptr<mrpt::system::CDirectoryExplorer>> cl(M("mrpt::system"), "CDirectoryExplorer", "This class allows the enumeration of the files/directories that exist into a\n given path.\n  The only existing method is \"explore\" and returns the list of found files &\n directories.\n  Refer to the example in /samples/UTILS/directoryExplorer\n\n  \n CFileSystemWatcher\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::system::CDirectoryExplorer(); } ) );

		{ // mrpt::system::CDirectoryExplorer::TFileInfo file:mrpt/system/CDirectoryExplorer.h line:35
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::system::CDirectoryExplorer::TFileInfo, std::shared_ptr<mrpt::system::CDirectoryExplorer::TFileInfo>> cl(enclosing_class, "TFileInfo", "This represents the information about each file.\n \n\n\n");
			cl.def( pybind11::init( [](){ return new mrpt::system::CDirectoryExplorer::TFileInfo(); } ) );
			cl.def( pybind11::init( [](mrpt::system::CDirectoryExplorer::TFileInfo const &o){ return new mrpt::system::CDirectoryExplorer::TFileInfo(o); } ) );
			cl.def_readwrite("name", &mrpt::system::CDirectoryExplorer::TFileInfo::name);
			cl.def_readwrite("wholePath", &mrpt::system::CDirectoryExplorer::TFileInfo::wholePath);
			cl.def_readwrite("accessTime", &mrpt::system::CDirectoryExplorer::TFileInfo::accessTime);
			cl.def_readwrite("modTime", &mrpt::system::CDirectoryExplorer::TFileInfo::modTime);
			cl.def_readwrite("isDir", &mrpt::system::CDirectoryExplorer::TFileInfo::isDir);
			cl.def_readwrite("isSymLink", &mrpt::system::CDirectoryExplorer::TFileInfo::isSymLink);
			cl.def_readwrite("fileSize", &mrpt::system::CDirectoryExplorer::TFileInfo::fileSize);
			cl.def("assign", (struct mrpt::system::CDirectoryExplorer::TFileInfo & (mrpt::system::CDirectoryExplorer::TFileInfo::*)(const struct mrpt::system::CDirectoryExplorer::TFileInfo &)) &mrpt::system::CDirectoryExplorer::TFileInfo::operator=, "C++: mrpt::system::CDirectoryExplorer::TFileInfo::operator=(const struct mrpt::system::CDirectoryExplorer::TFileInfo &) --> struct mrpt::system::CDirectoryExplorer::TFileInfo &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::system::CFileSystemWatcher file:mrpt/system/CFileSystemWatcher.h line:28
		pybind11::class_<mrpt::system::CFileSystemWatcher, std::shared_ptr<mrpt::system::CFileSystemWatcher>> cl(M("mrpt::system"), "CFileSystemWatcher", "This class subscribes to notifications of file system changes, thus it can\n be used to efficiently stay informed about changes in a directory tree.\n  - Windows: Requires Windows 2000 or newer.\n  - Linux: Requires kernel 2.6.13 or newer.\n  Using this class in an old Linux or other unsoported system (Unix,etc...)\n has no effect, i.e. no notification will be ever received.\n  \n\n CDirectoryExplorer\n \n\n\n ");
		cl.def( pybind11::init<const std::string &>(), pybind11::arg("path") );

		cl.def( pybind11::init( [](mrpt::system::CFileSystemWatcher const &o){ return new mrpt::system::CFileSystemWatcher(o); } ) );
		cl.def("assign", (class mrpt::system::CFileSystemWatcher & (mrpt::system::CFileSystemWatcher::*)(const class mrpt::system::CFileSystemWatcher &)) &mrpt::system::CFileSystemWatcher::operator=, "C++: mrpt::system::CFileSystemWatcher::operator=(const class mrpt::system::CFileSystemWatcher &) --> class mrpt::system::CFileSystemWatcher &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		{ // mrpt::system::CFileSystemWatcher::TFileSystemChange file:mrpt/system/CFileSystemWatcher.h line:33
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::system::CFileSystemWatcher::TFileSystemChange, std::shared_ptr<mrpt::system::CFileSystemWatcher::TFileSystemChange>> cl(enclosing_class, "TFileSystemChange", "Each of the changes detected by mrpt::system::CFileSystemWatcher");
			cl.def( pybind11::init( [](){ return new mrpt::system::CFileSystemWatcher::TFileSystemChange(); } ) );
			cl.def( pybind11::init( [](mrpt::system::CFileSystemWatcher::TFileSystemChange const &o){ return new mrpt::system::CFileSystemWatcher::TFileSystemChange(o); } ) );
			cl.def_readwrite("path", &mrpt::system::CFileSystemWatcher::TFileSystemChange::path);
			cl.def_readwrite("isDir", &mrpt::system::CFileSystemWatcher::TFileSystemChange::isDir);
			cl.def_readwrite("eventModified", &mrpt::system::CFileSystemWatcher::TFileSystemChange::eventModified);
			cl.def_readwrite("eventCloseWrite", &mrpt::system::CFileSystemWatcher::TFileSystemChange::eventCloseWrite);
			cl.def_readwrite("eventDeleted", &mrpt::system::CFileSystemWatcher::TFileSystemChange::eventDeleted);
			cl.def_readwrite("eventMovedTo", &mrpt::system::CFileSystemWatcher::TFileSystemChange::eventMovedTo);
			cl.def_readwrite("eventMovedFrom", &mrpt::system::CFileSystemWatcher::TFileSystemChange::eventMovedFrom);
			cl.def_readwrite("eventCreated", &mrpt::system::CFileSystemWatcher::TFileSystemChange::eventCreated);
			cl.def_readwrite("eventAccessed", &mrpt::system::CFileSystemWatcher::TFileSystemChange::eventAccessed);
			cl.def("assign", (struct mrpt::system::CFileSystemWatcher::TFileSystemChange & (mrpt::system::CFileSystemWatcher::TFileSystemChange::*)(const struct mrpt::system::CFileSystemWatcher::TFileSystemChange &)) &mrpt::system::CFileSystemWatcher::TFileSystemChange::operator=, "C++: mrpt::system::CFileSystemWatcher::TFileSystemChange::operator=(const struct mrpt::system::CFileSystemWatcher::TFileSystemChange &) --> struct mrpt::system::CFileSystemWatcher::TFileSystemChange &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
	{ // mrpt::system::WorkerThreadsPool file:mrpt/system/WorkerThreadsPool.h line:29
		pybind11::class_<mrpt::system::WorkerThreadsPool, std::shared_ptr<mrpt::system::WorkerThreadsPool>> cl(M("mrpt::system"), "WorkerThreadsPool", "A simple thread pool\n\n \n Partly based on: https://github.com/progschj/ThreadPool (ZLib license)");
		cl.def( pybind11::init( [](){ return new mrpt::system::WorkerThreadsPool(); } ) );
		cl.def( pybind11::init( [](unsigned long const & a0){ return new mrpt::system::WorkerThreadsPool(a0); } ), "doc" , pybind11::arg("num_threads"));
		cl.def( pybind11::init<std::size_t, enum mrpt::system::WorkerThreadsPool::queue_policy_t>(), pybind11::arg("num_threads"), pybind11::arg("p") );


		pybind11::enum_<mrpt::system::WorkerThreadsPool::queue_policy_t>(cl, "queue_policy_t", pybind11::arithmetic(), "")
			.value("POLICY_FIFO", mrpt::system::WorkerThreadsPool::POLICY_FIFO)
			.value("POLICY_DROP_OLD", mrpt::system::WorkerThreadsPool::POLICY_DROP_OLD)
			.export_values();

		cl.def("resize", (void (mrpt::system::WorkerThreadsPool::*)(std::size_t)) &mrpt::system::WorkerThreadsPool::resize, "C++: mrpt::system::WorkerThreadsPool::resize(std::size_t) --> void", pybind11::arg("num_threads"));
		cl.def("clear", (void (mrpt::system::WorkerThreadsPool::*)()) &mrpt::system::WorkerThreadsPool::clear, "C++: mrpt::system::WorkerThreadsPool::clear() --> void");
		cl.def("pendingTasks", (std::size_t (mrpt::system::WorkerThreadsPool::*)() const) &mrpt::system::WorkerThreadsPool::pendingTasks, "Returns the number of enqueued tasks, currently waiting for a free\n working thread to process them.  \n\nC++: mrpt::system::WorkerThreadsPool::pendingTasks() const --> std::size_t");
	}
}
