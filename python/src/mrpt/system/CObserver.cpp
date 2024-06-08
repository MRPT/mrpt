#include <mrpt/system/CObservable.h>
#include <mrpt/system/CObserver.h>
#include <mrpt/system/mrptEvent.h>
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

// mrpt::system::CObserver file:mrpt/system/CObserver.h line:35
struct PyCallBack_mrpt_system_CObserver : public mrpt::system::CObserver {
	using mrpt::system::CObserver::CObserver;

	void OnEvent(const class mrpt::system::mrptEvent & a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::system::CObserver *>(this), "OnEvent");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CObserver::OnEvent\"");
	}
};

void bind_mrpt_system_CObserver(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::system::CObserver file:mrpt/system/CObserver.h line:35
		pybind11::class_<mrpt::system::CObserver, std::shared_ptr<mrpt::system::CObserver>, PyCallBack_mrpt_system_CObserver> cl(M("mrpt::system"), "CObserver", "Inherit from this class to get notified about events from any CObservable\n object after subscribing to it.\n\n  The main methods in this class are:\n   - observeBegin(): To be called to start listening at a given object.\n   - OnEvent(): Virtual functions to be implemented in your child class to\n receive all the notifications.\n\n  Note that if custom (child) mrptEvent classes are used, you can tell\n between them in runtime with \"dynamic_cast<>()\".\n\n \n The pairs CObservable / CObserver automatically notify each other the\n destruction of any of them, effectively ending the subscription of events.\n \n\n\n  \n CObservable, mrptEvent");
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_system_CObserver(); } ) );
		cl.def(pybind11::init<PyCallBack_mrpt_system_CObserver const &>());
		cl.def("observeBegin", (void (mrpt::system::CObserver::*)(class mrpt::system::CObservable &)) &mrpt::system::CObserver::observeBegin, "Starts the subscription of this observer to the given object.  \n\n observeEnd  \n\nC++: mrpt::system::CObserver::observeBegin(class mrpt::system::CObservable &) --> void", pybind11::arg("obj"));
		cl.def("observeEnd", (void (mrpt::system::CObserver::*)(class mrpt::system::CObservable &)) &mrpt::system::CObserver::observeEnd, "Ends the subscription of this observer to the given object (note that\n   there is no need to call this method, since the destruction of the first\n   of observer/observed will put an end to the process\n    \n\n observeBegin  \n\nC++: mrpt::system::CObserver::observeEnd(class mrpt::system::CObservable &) --> void", pybind11::arg("obj"));
		cl.def("assign", (class mrpt::system::CObserver & (mrpt::system::CObserver::*)(const class mrpt::system::CObserver &)) &mrpt::system::CObserver::operator=, "C++: mrpt::system::CObserver::operator=(const class mrpt::system::CObserver &) --> class mrpt::system::CObserver &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
