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

void bind_unknown_unknown_4(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::obs::gnss::nv_oem6_ins_status_type::nv_ins_status_type_t file: line:175
	pybind11::enum_<mrpt::obs::gnss::nv_oem6_ins_status_type::nv_ins_status_type_t>(M("mrpt::obs::gnss::nv_oem6_ins_status_type"), "nv_ins_status_type_t", pybind11::arithmetic(), "Novatel SPAN on OEM6 firmware reference, table 33 ")
		.value("INS_INACTIVE", mrpt::obs::gnss::nv_oem6_ins_status_type::INS_INACTIVE)
		.value("INS_ALIGNING", mrpt::obs::gnss::nv_oem6_ins_status_type::INS_ALIGNING)
		.value("INS_HIGH_VARIANCE", mrpt::obs::gnss::nv_oem6_ins_status_type::INS_HIGH_VARIANCE)
		.value("INS_SOLUTION_GOOD", mrpt::obs::gnss::nv_oem6_ins_status_type::INS_SOLUTION_GOOD)
		.value("INS_SOLUTION_FREE", mrpt::obs::gnss::nv_oem6_ins_status_type::INS_SOLUTION_FREE)
		.value("INS_ALIGNMENT_COMPLETE", mrpt::obs::gnss::nv_oem6_ins_status_type::INS_ALIGNMENT_COMPLETE)
		.value("DETERMINING_ORIENTATION", mrpt::obs::gnss::nv_oem6_ins_status_type::DETERMINING_ORIENTATION)
		.value("WAITING_INITIALPOS", mrpt::obs::gnss::nv_oem6_ins_status_type::WAITING_INITIALPOS)
		.export_values();

;

	// mrpt::obs::gnss::nv_oem6_ins_status_type::enum2str(int) file: line:197
	M("mrpt::obs::gnss::nv_oem6_ins_status_type").def("enum2str", (const std::string & (*)(int)) &mrpt::obs::gnss::nv_oem6_ins_status_type::enum2str, "for nv_ins_status_type_t \n\nC++: mrpt::obs::gnss::nv_oem6_ins_status_type::enum2str(int) --> const std::string &", pybind11::return_value_policy::automatic, pybind11::arg("val"));

}
