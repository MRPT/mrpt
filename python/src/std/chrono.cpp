#include <chrono>
#include <mrpt/core/Clock.h>
#include <ratio>
#include <sstream> // __str__

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <pybind11/stl.h>
#include <mrpt/system/datetime.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_std_chrono(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // std::chrono::duration file:chrono line:459
		pybind11::class_<std::chrono::duration<int64_t,std::ratio<1, 10000000>>, std::shared_ptr<std::chrono::duration<int64_t,std::ratio<1, 10000000>>>> cl(M("std::chrono"), "duration_long_std_ratio_1_10000000_t", "");
		cl.def( pybind11::init( [](){ return new std::chrono::duration<int64_t,std::ratio<1, 10000000>>(); } ) );
		cl.def( pybind11::init( [](std::chrono::duration<int64_t,std::ratio<1, 10000000>> const &o){ return new std::chrono::duration<int64_t,std::ratio<1, 10000000>>(o); } ) );
		cl.def("assign", (struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> & (std::chrono::duration<int64_t,std::ratio<1, 10000000>>::*)(const struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &)) &std::chrono::duration<int64_t, std::ratio<1, 10000000>>::operator=, "C++: std::chrono::duration<int64_t, std::ratio<1, 10000000>>::operator=(const struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &) --> struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("count", (long (std::chrono::duration<int64_t,std::ratio<1, 10000000>>::*)() const) &std::chrono::duration<int64_t, std::ratio<1, 10000000>>::count, "C++: std::chrono::duration<int64_t, std::ratio<1, 10000000>>::count() const --> long");
		cl.def("__pos__", (struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> (std::chrono::duration<int64_t,std::ratio<1, 10000000>>::*)() const) &std::chrono::duration<int64_t, std::ratio<1, 10000000>>::operator+, "C++: std::chrono::duration<int64_t, std::ratio<1, 10000000>>::operator+() const --> struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>>");
		cl.def("__neg__", (struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> (std::chrono::duration<int64_t,std::ratio<1, 10000000>>::*)() const) &std::chrono::duration<int64_t, std::ratio<1, 10000000>>::operator-, "C++: std::chrono::duration<int64_t, std::ratio<1, 10000000>>::operator-() const --> struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>>");
		cl.def("pre_increment", (struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> & (std::chrono::duration<int64_t,std::ratio<1, 10000000>>::*)()) &std::chrono::duration<int64_t, std::ratio<1, 10000000>>::operator++, "C++: std::chrono::duration<int64_t, std::ratio<1, 10000000>>::operator++() --> struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &", pybind11::return_value_policy::automatic);
		cl.def("post_increment", (struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> (std::chrono::duration<int64_t,std::ratio<1, 10000000>>::*)(int)) &std::chrono::duration<int64_t, std::ratio<1, 10000000>>::operator++, "C++: std::chrono::duration<int64_t, std::ratio<1, 10000000>>::operator++(int) --> struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>>", pybind11::arg(""));
		cl.def("pre_decrement", (struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> & (std::chrono::duration<int64_t,std::ratio<1, 10000000>>::*)()) &std::chrono::duration<int64_t, std::ratio<1, 10000000>>::operator--, "C++: std::chrono::duration<int64_t, std::ratio<1, 10000000>>::operator--() --> struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &", pybind11::return_value_policy::automatic);
		cl.def("post_decrement", (struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> (std::chrono::duration<int64_t,std::ratio<1, 10000000>>::*)(int)) &std::chrono::duration<int64_t, std::ratio<1, 10000000>>::operator--, "C++: std::chrono::duration<int64_t, std::ratio<1, 10000000>>::operator--(int) --> struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>>", pybind11::arg(""));
		cl.def("__iadd__", (struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> & (std::chrono::duration<int64_t,std::ratio<1, 10000000>>::*)(const struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &)) &std::chrono::duration<int64_t, std::ratio<1, 10000000>>::operator+=, "C++: std::chrono::duration<int64_t, std::ratio<1, 10000000>>::operator+=(const struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &) --> struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &", pybind11::return_value_policy::automatic, pybind11::arg("__d"));
		cl.def("__isub__", (struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> & (std::chrono::duration<int64_t,std::ratio<1, 10000000>>::*)(const struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &)) &std::chrono::duration<int64_t, std::ratio<1, 10000000>>::operator-=, "C++: std::chrono::duration<int64_t, std::ratio<1, 10000000>>::operator-=(const struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &) --> struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &", pybind11::return_value_policy::automatic, pybind11::arg("__d"));
		cl.def("__imul__", (struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> & (std::chrono::duration<int64_t,std::ratio<1, 10000000>>::*)(const long &)) &std::chrono::duration<int64_t, std::ratio<1, 10000000>>::operator*=, "C++: std::chrono::duration<int64_t, std::ratio<1, 10000000>>::operator*=(const long &) --> struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &", pybind11::return_value_policy::automatic, pybind11::arg("__rhs"));
		cl.def("__itruediv__", (struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> & (std::chrono::duration<int64_t,std::ratio<1, 10000000>>::*)(const long &)) &std::chrono::duration<int64_t, std::ratio<1, 10000000>>::operator/=, "C++: std::chrono::duration<int64_t, std::ratio<1, 10000000>>::operator/=(const long &) --> struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &", pybind11::return_value_policy::automatic, pybind11::arg("__rhs"));
		cl.def_static("zero", (struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> (*)()) &std::chrono::duration<int64_t, std::ratio<1, 10000000>>::zero, "C++: std::chrono::duration<int64_t, std::ratio<1, 10000000>>::zero() --> struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>>");
		cl.def_static("min", (struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> (*)()) &std::chrono::duration<int64_t, std::ratio<1, 10000000>>::min, "C++: std::chrono::duration<int64_t, std::ratio<1, 10000000>>::min() --> struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>>");
		cl.def_static("max", (struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> (*)()) &std::chrono::duration<int64_t, std::ratio<1, 10000000>>::max, "C++: std::chrono::duration<int64_t, std::ratio<1, 10000000>>::max() --> struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>>");
	}
	{ // std::chrono::time_point file:chrono line:872
		pybind11::class_<mrpt::Clock::time_point, std::shared_ptr<mrpt::Clock::time_point>> cl(M("std::chrono"), "time_point_mrpt_Clock_std_chrono_duration_long_std_ratio_1_10000000_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::Clock::time_point(); } ) );
		cl.def( pybind11::init<const struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &>(), pybind11::arg("__dur") );

		cl.def( pybind11::init( [](mrpt::Clock::time_point const &o){ return new mrpt::Clock::time_point(o); } ) );
		cl.def("time_since_epoch", (struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> (mrpt::Clock::time_point::*)() const) &std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>::time_since_epoch, "C++: std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>::time_since_epoch() const --> struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>>");
		cl.def("__iadd__", (mrpt::Clock::time_point & (mrpt::Clock::time_point::*)(const struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &)) &std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>::operator+=, "C++: std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>::operator+=(const struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &) --> mrpt::Clock::time_point &", pybind11::return_value_policy::automatic, pybind11::arg("__dur"));
		cl.def("__isub__", (mrpt::Clock::time_point & (mrpt::Clock::time_point::*)(const struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &)) &std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>::operator-=, "C++: std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>::operator-=(const struct std::chrono::duration<int64_t,struct std::ratio<1,10000000>> &) --> mrpt::Clock::time_point &", pybind11::return_value_policy::automatic, pybind11::arg("__dur"));
		cl.def_static("min", (mrpt::Clock::time_point (*)()) &std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>::min, "C++: std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>::min() --> mrpt::Clock::time_point");
		cl.def_static("max", (mrpt::Clock::time_point (*)()) &std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>::max, "C++: std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>::max() --> mrpt::Clock::time_point");
		cl.def("assign", (mrpt::Clock::time_point & (mrpt::Clock::time_point::*)(const mrpt::Clock::time_point &)) &std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>::operator=, "C++: std::chrono::time_point<mrpt::Clock, std::chrono::duration<int64_t, std::ratio<1, 10000000>>>::operator=(const mrpt::Clock::time_point &) --> mrpt::Clock::time_point &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("to_double", [](const mrpt::Clock::time_point &t) { return mrpt::Clock::toDouble(t); });
		cl.def("__str__", [](const mrpt::Clock::time_point &t) { return mrpt::system::dateTimeLocalToString(t); }, "Gets the date and time of the given timestamp in local time.");
	}
}
