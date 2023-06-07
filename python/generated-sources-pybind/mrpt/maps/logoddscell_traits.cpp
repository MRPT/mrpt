#include <mrpt/maps/logoddscell_traits.h>
#include <sstream> // __str__

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

void bind_mrpt_maps_logoddscell_traits(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::detail::logoddscell_traits file:mrpt/maps/logoddscell_traits.h line:20
		pybind11::class_<mrpt::maps::detail::logoddscell_traits<signed char>, std::shared_ptr<mrpt::maps::detail::logoddscell_traits<signed char>>> cl(M("mrpt::maps::detail"), "logoddscell_traits_signed_char_t", "");
		cl.def( pybind11::init( [](mrpt::maps::detail::logoddscell_traits<signed char> const &o){ return new mrpt::maps::detail::logoddscell_traits<signed char>(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::maps::detail::logoddscell_traits<signed char>(); } ) );
		cl.def("assign", (struct mrpt::maps::detail::logoddscell_traits<int8_t> & (mrpt::maps::detail::logoddscell_traits<signed char>::*)(const struct mrpt::maps::detail::logoddscell_traits<int8_t> &)) &mrpt::maps::detail::logoddscell_traits<signed char>::operator=, "C++: mrpt::maps::detail::logoddscell_traits<signed char>::operator=(const struct mrpt::maps::detail::logoddscell_traits<int8_t> &) --> struct mrpt::maps::detail::logoddscell_traits<int8_t> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::maps::detail::logoddscell_traits file:mrpt/maps/logoddscell_traits.h line:28
		pybind11::class_<mrpt::maps::detail::logoddscell_traits<short>, std::shared_ptr<mrpt::maps::detail::logoddscell_traits<short>>> cl(M("mrpt::maps::detail"), "logoddscell_traits_short_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::maps::detail::logoddscell_traits<short>(); } ) );
	}
}
