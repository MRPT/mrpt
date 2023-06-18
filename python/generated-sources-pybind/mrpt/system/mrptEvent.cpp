#include <mrpt/system/CObservable.h>
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

// mrpt::system::mrptEvent file:mrpt/system/mrptEvent.h line:31
struct PyCallBack_mrpt_system_mrptEvent : public mrpt::system::mrptEvent {
	using mrpt::system::mrptEvent::mrptEvent;

	void do_nothing() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::system::mrptEvent *>(this), "do_nothing");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return mrptEvent::do_nothing();
	}
};

// mrpt::system::mrptEventOnDestroy file:mrpt/system/mrptEvent.h line:66
struct PyCallBack_mrpt_system_mrptEventOnDestroy : public mrpt::system::mrptEventOnDestroy {
	using mrpt::system::mrptEventOnDestroy::mrptEventOnDestroy;

	void do_nothing() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::system::mrptEventOnDestroy *>(this), "do_nothing");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return mrptEventOnDestroy::do_nothing();
	}
};

void bind_mrpt_system_mrptEvent(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::system::mrptEvent file:mrpt/system/mrptEvent.h line:31
		pybind11::class_<mrpt::system::mrptEvent, std::shared_ptr<mrpt::system::mrptEvent>, PyCallBack_mrpt_system_mrptEvent> cl(M("mrpt::system"), "mrptEvent", "The basic event type for the observer-observable pattern in MRPT.\n   You can sub-class this base class to create custom event types, then\n    tell between them in runtime with isOfType<T>(), for example:\n \n\n\n\n\n\n\n\n \n CObserver, CObservable\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::system::mrptEvent(); }, [](){ return new PyCallBack_mrpt_system_mrptEvent(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_system_mrptEvent const &o){ return new PyCallBack_mrpt_system_mrptEvent(o); } ) );
		cl.def( pybind11::init( [](mrpt::system::mrptEvent const &o){ return new mrpt::system::mrptEvent(o); } ) );
		cl.def_readwrite("timestamp", &mrpt::system::mrptEvent::timestamp);
		cl.def("assign", (class mrpt::system::mrptEvent & (mrpt::system::mrptEvent::*)(const class mrpt::system::mrptEvent &)) &mrpt::system::mrptEvent::operator=, "C++: mrpt::system::mrptEvent::operator=(const class mrpt::system::mrptEvent &) --> class mrpt::system::mrptEvent &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::system::mrptEventOnDestroy file:mrpt/system/mrptEvent.h line:66
		pybind11::class_<mrpt::system::mrptEventOnDestroy, std::shared_ptr<mrpt::system::mrptEventOnDestroy>, PyCallBack_mrpt_system_mrptEventOnDestroy, mrpt::system::mrptEvent> cl(M("mrpt::system"), "mrptEventOnDestroy", "An event sent by any CObservable object (automatically) just before being\n destroyed and telling its observers to unsubscribe.\n \n\n\n ");
		cl.def( pybind11::init<const class mrpt::system::CObservable *>(), pybind11::arg("obj") );

		cl.def("assign", (class mrpt::system::mrptEventOnDestroy & (mrpt::system::mrptEventOnDestroy::*)(const class mrpt::system::mrptEventOnDestroy &)) &mrpt::system::mrptEventOnDestroy::operator=, "C++: mrpt::system::mrptEventOnDestroy::operator=(const class mrpt::system::mrptEventOnDestroy &) --> class mrpt::system::mrptEventOnDestroy &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
