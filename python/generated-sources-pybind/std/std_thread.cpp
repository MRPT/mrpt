#include <bits/std_thread.h>
#include <sstream> // __str__
#include <thread>

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

// std::thread::_State file:bits/std_thread.h line:68
struct PyCallBack_std_thread__State : public std::thread::_State {
	using std::thread::_State::_State;

	void _M_run() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const std::thread::_State *>(this), "_M_run");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"_State::_M_run\"");
	}
};

void bind_std_std_thread(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // std::thread file:bits/std_thread.h line:62
		pybind11::class_<std::thread, std::shared_ptr<std::thread>> cl(M("std"), "thread", "");
		cl.def( pybind11::init( [](){ return new std::thread(); } ) );
		cl.def("swap", (void (std::thread::*)(class std::thread &)) &std::thread::swap, "C++: std::thread::swap(class std::thread &) --> void", pybind11::arg("__t"));
		cl.def("joinable", (bool (std::thread::*)() const) &std::thread::joinable, "C++: std::thread::joinable() const --> bool");
		cl.def("join", (void (std::thread::*)()) &std::thread::join, "C++: std::thread::join() --> void");
		cl.def("detach", (void (std::thread::*)()) &std::thread::detach, "C++: std::thread::detach() --> void");
		cl.def("get_id", (class std::thread::id (std::thread::*)() const) &std::thread::get_id, "C++: std::thread::get_id() const --> class std::thread::id");
		cl.def("native_handle", (unsigned long (std::thread::*)()) &std::thread::native_handle, "C++: std::thread::native_handle() --> unsigned long");
		cl.def_static("hardware_concurrency", (unsigned int (*)()) &std::thread::hardware_concurrency, "C++: std::thread::hardware_concurrency() --> unsigned int");

		{ // std::thread::_State file:bits/std_thread.h line:68
			auto & enclosing_class = cl;
			pybind11::class_<std::thread::_State, std::shared_ptr<std::thread::_State>, PyCallBack_std_thread__State> cl(enclosing_class, "_State", "");
			cl.def( pybind11::init( [](){ return new PyCallBack_std_thread__State(); } ) );
			cl.def("_M_run", (void (std::thread::_State::*)()) &std::thread::_State::_M_run, "C++: std::thread::_State::_M_run() --> void");
			cl.def("assign", (struct std::thread::_State & (std::thread::_State::*)(const struct std::thread::_State &)) &std::thread::_State::operator=, "C++: std::thread::_State::operator=(const struct std::thread::_State &) --> struct std::thread::_State &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // std::thread::id file:bits/std_thread.h line:81
			auto & enclosing_class = cl;
			pybind11::class_<std::thread::id, std::shared_ptr<std::thread::id>> cl(enclosing_class, "id", "");
			cl.def( pybind11::init( [](){ return new std::thread::id(); } ) );
			cl.def( pybind11::init<unsigned long>(), pybind11::arg("__id") );

			cl.def( pybind11::init( [](std::thread::id const &o){ return new std::thread::id(o); } ) );
			cl.def("assign", (class std::thread::id & (std::thread::id::*)(const class std::thread::id &)) &std::thread::id::operator=, "C++: std::thread::id::operator=(const class std::thread::id &) --> class std::thread::id &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
