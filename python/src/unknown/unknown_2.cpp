#include <iterator>
#include <memory>
#include <mrpt/obs/gnss_messages_novatel.h>
#include <string>

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

void bind_unknown_unknown_2(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::obs::gnss::nv_oem6_position_type::nv_position_type_t file: line:88
	pybind11::enum_<mrpt::obs::gnss::nv_oem6_position_type::nv_position_type_t>(M("mrpt::obs::gnss::nv_oem6_position_type"), "nv_position_type_t", pybind11::arithmetic(), "Novatel OEM6 firmware reference, table 84; Novatel SPAN on OEM6 firmware\n manual, table 26. ")
		.value("NONE", mrpt::obs::gnss::nv_oem6_position_type::NONE)
		.value("FIXEDPOS", mrpt::obs::gnss::nv_oem6_position_type::FIXEDPOS)
		.value("FIXEDHEIGHT", mrpt::obs::gnss::nv_oem6_position_type::FIXEDHEIGHT)
		.value("Reserved", mrpt::obs::gnss::nv_oem6_position_type::Reserved)
		.value("FLOATCONV", mrpt::obs::gnss::nv_oem6_position_type::FLOATCONV)
		.value("WIDELANE", mrpt::obs::gnss::nv_oem6_position_type::WIDELANE)
		.value("NARROWLANE", mrpt::obs::gnss::nv_oem6_position_type::NARROWLANE)
		.value("DOPPLER_VELOCITY", mrpt::obs::gnss::nv_oem6_position_type::DOPPLER_VELOCITY)
		.value("SINGLE", mrpt::obs::gnss::nv_oem6_position_type::SINGLE)
		.value("PSRDIFF", mrpt::obs::gnss::nv_oem6_position_type::PSRDIFF)
		.value("WAAS", mrpt::obs::gnss::nv_oem6_position_type::WAAS)
		.value("PROPAGATED", mrpt::obs::gnss::nv_oem6_position_type::PROPAGATED)
		.value("OMNISTAR", mrpt::obs::gnss::nv_oem6_position_type::OMNISTAR)
		.value("L1_FLOAT", mrpt::obs::gnss::nv_oem6_position_type::L1_FLOAT)
		.value("IONOFREE_FLOAT", mrpt::obs::gnss::nv_oem6_position_type::IONOFREE_FLOAT)
		.value("NARROW_FLOAT", mrpt::obs::gnss::nv_oem6_position_type::NARROW_FLOAT)
		.value("L1_INT", mrpt::obs::gnss::nv_oem6_position_type::L1_INT)
		.value("WIDE_INT", mrpt::obs::gnss::nv_oem6_position_type::WIDE_INT)
		.value("NARROW_INT", mrpt::obs::gnss::nv_oem6_position_type::NARROW_INT)
		.value("RTK_DIRECT_INS", mrpt::obs::gnss::nv_oem6_position_type::RTK_DIRECT_INS)
		.value("INS", mrpt::obs::gnss::nv_oem6_position_type::INS)
		.value("INS_PSRSP", mrpt::obs::gnss::nv_oem6_position_type::INS_PSRSP)
		.value("INS_PSRDIFF", mrpt::obs::gnss::nv_oem6_position_type::INS_PSRDIFF)
		.value("INS_RTKFLOAT", mrpt::obs::gnss::nv_oem6_position_type::INS_RTKFLOAT)
		.value("INS_RTKFIXED", mrpt::obs::gnss::nv_oem6_position_type::INS_RTKFIXED)
		.value("OMNISTAR_HP", mrpt::obs::gnss::nv_oem6_position_type::OMNISTAR_HP)
		.value("OMNISTAR_XP", mrpt::obs::gnss::nv_oem6_position_type::OMNISTAR_XP)
		.value("CDGPS", mrpt::obs::gnss::nv_oem6_position_type::CDGPS)
		.export_values();

;

	// mrpt::obs::gnss::nv_oem6_position_type::enum2str(int) file: line:120
	M("mrpt::obs::gnss::nv_oem6_position_type").def("enum2str", (const std::string & (*)(int)) &mrpt::obs::gnss::nv_oem6_position_type::enum2str, "for nv_position_type_t \n\nC++: mrpt::obs::gnss::nv_oem6_position_type::enum2str(int) --> const std::string &", pybind11::return_value_policy::automatic, pybind11::arg("val"));

}
