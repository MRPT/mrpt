#include <mrpt/system/CObservable.h>
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

void bind_mrpt_system_CObservable(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::system::CObservable file:mrpt/system/CObservable.h line:31
		pybind11::class_<mrpt::system::CObservable, std::shared_ptr<mrpt::system::CObservable>> cl(M("mrpt::system"), "CObservable", "Inherit from this class for those objects capable of being observed by a\n CObserver class.\n\n  The only thing to do in your child class is to call\n CObservable::publishEvent() whenever needed and all the\n   observer classes will be notified.\n\n \n The pairs CObservable / CObserver automatically notify each other the\n destruction of any of them, effectively ending the subscription of events.\n\n  \n CObserver, mrptEvent\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::system::CObservable(); } ) );
		cl.def( pybind11::init( [](mrpt::system::CObservable const &o){ return new mrpt::system::CObservable(o); } ) );
		cl.def("assign", (class mrpt::system::CObservable & (mrpt::system::CObservable::*)(const class mrpt::system::CObservable &)) &mrpt::system::CObservable::operator=, "C++: mrpt::system::CObservable::operator=(const class mrpt::system::CObservable &) --> class mrpt::system::CObservable &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
