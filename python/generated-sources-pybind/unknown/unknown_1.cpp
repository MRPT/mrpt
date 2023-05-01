#include <mrpt/obs/gnss_messages.h>
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

void bind_unknown_unknown_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::obs::gnss::nv_oem6_short_header_t file: line:59
		pybind11::class_<mrpt::obs::gnss::nv_oem6_short_header_t, std::shared_ptr<mrpt::obs::gnss::nv_oem6_short_header_t>> cl(M("mrpt::obs::gnss"), "nv_oem6_short_header_t", "Novatel OEM6 short header structure \n mrpt::obs::CObservationGPS  ");
		cl.def( pybind11::init( [](mrpt::obs::gnss::nv_oem6_short_header_t const &o){ return new mrpt::obs::gnss::nv_oem6_short_header_t(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::obs::gnss::nv_oem6_short_header_t(); } ) );
		cl.def_readwrite("msg_len", &mrpt::obs::gnss::nv_oem6_short_header_t::msg_len);
		cl.def_readwrite("msg_id", &mrpt::obs::gnss::nv_oem6_short_header_t::msg_id);
		cl.def_readwrite("week", &mrpt::obs::gnss::nv_oem6_short_header_t::week);
		cl.def_readwrite("ms_in_week", &mrpt::obs::gnss::nv_oem6_short_header_t::ms_in_week);
		cl.def("fixEndianness", (void (mrpt::obs::gnss::nv_oem6_short_header_t::*)()) &mrpt::obs::gnss::nv_oem6_short_header_t::fixEndianness, "C++: mrpt::obs::gnss::nv_oem6_short_header_t::fixEndianness() --> void");
		cl.def("assign", (struct mrpt::obs::gnss::nv_oem6_short_header_t & (mrpt::obs::gnss::nv_oem6_short_header_t::*)(const struct mrpt::obs::gnss::nv_oem6_short_header_t &)) &mrpt::obs::gnss::nv_oem6_short_header_t::operator=, "C++: mrpt::obs::gnss::nv_oem6_short_header_t::operator=(const struct mrpt::obs::gnss::nv_oem6_short_header_t &) --> struct mrpt::obs::gnss::nv_oem6_short_header_t &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
