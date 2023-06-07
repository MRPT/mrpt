#include <sstream> // __str__
#include <variant>

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

void bind_std_variant(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // std::monostate file:variant line:1220
		pybind11::class_<std::monostate, std::shared_ptr<std::monostate>> cl(M("std"), "monostate", "");
		cl.def( pybind11::init( [](){ return new std::monostate(); } ) );
		cl.def( pybind11::init( [](std::monostate const &o){ return new std::monostate(o); } ) );
		cl.def("assign", (struct std::monostate & (std::monostate::*)(const struct std::monostate &)) &std::monostate::operator=, "C++: std::monostate::operator=(const struct std::monostate &) --> struct std::monostate &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
