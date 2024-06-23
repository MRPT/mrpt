#include <mrpt/typemeta/static_string.h>
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

void bind_mrpt_typemeta_static_string(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::typemeta::internal::make_sequence_ file:mrpt/typemeta/static_string.h line:75
		pybind11::class_<mrpt::typemeta::internal::make_sequence_<0>, std::shared_ptr<mrpt::typemeta::internal::make_sequence_<0>>> cl(M("mrpt::typemeta::internal"), "make_sequence_0_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::typemeta::internal::make_sequence_<0>(); } ) );
	}
}
