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

void bind_unknown_unknown_3(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::obs::gnss::nv_oem6_solution_status::nv_solution_status_t file: line:126
	pybind11::enum_<mrpt::obs::gnss::nv_oem6_solution_status::nv_solution_status_t>(M("mrpt::obs::gnss::nv_oem6_solution_status"), "nv_solution_status_t", pybind11::arithmetic(), "Novatel OEM6 firmware reference, table 85 ")
		.value("SOL_COMPUTED", mrpt::obs::gnss::nv_oem6_solution_status::SOL_COMPUTED)
		.value("INSUFFICIENT_OBS", mrpt::obs::gnss::nv_oem6_solution_status::INSUFFICIENT_OBS)
		.value("NO_CONVERGENCE", mrpt::obs::gnss::nv_oem6_solution_status::NO_CONVERGENCE)
		.value("SINGULARITY", mrpt::obs::gnss::nv_oem6_solution_status::SINGULARITY)
		.value("COV_TRACE", mrpt::obs::gnss::nv_oem6_solution_status::COV_TRACE)
		.value("TEST_DIST", mrpt::obs::gnss::nv_oem6_solution_status::TEST_DIST)
		.value("COLD_START", mrpt::obs::gnss::nv_oem6_solution_status::COLD_START)
		.value("V_H_LIMIT", mrpt::obs::gnss::nv_oem6_solution_status::V_H_LIMIT)
		.value("VARIANCE", mrpt::obs::gnss::nv_oem6_solution_status::VARIANCE)
		.value("RESIDUALS", mrpt::obs::gnss::nv_oem6_solution_status::RESIDUALS)
		.value("DELTA_POS", mrpt::obs::gnss::nv_oem6_solution_status::DELTA_POS)
		.value("NEGATIVE_VAR", mrpt::obs::gnss::nv_oem6_solution_status::NEGATIVE_VAR)
		.value("INTEGRITY_WARNING", mrpt::obs::gnss::nv_oem6_solution_status::INTEGRITY_WARNING)
		.value("INS_INACTIVE", mrpt::obs::gnss::nv_oem6_solution_status::INS_INACTIVE)
		.value("INS_ALIGNING", mrpt::obs::gnss::nv_oem6_solution_status::INS_ALIGNING)
		.value("INS_BAD", mrpt::obs::gnss::nv_oem6_solution_status::INS_BAD)
		.value("IMU_UNPLUGGED", mrpt::obs::gnss::nv_oem6_solution_status::IMU_UNPLUGGED)
		.value("PENDING", mrpt::obs::gnss::nv_oem6_solution_status::PENDING)
		.value("INVALID_FIX", mrpt::obs::gnss::nv_oem6_solution_status::INVALID_FIX)
		.export_values();

;

	// mrpt::obs::gnss::nv_oem6_solution_status::enum2str(int) file: line:170
	M("mrpt::obs::gnss::nv_oem6_solution_status").def("enum2str", (const std::string & (*)(int)) &mrpt::obs::gnss::nv_oem6_solution_status::enum2str, "for nv_solution_status_t \n\nC++: mrpt::obs::gnss::nv_oem6_solution_status::enum2str(int) --> const std::string &", pybind11::return_value_policy::automatic, pybind11::arg("val"));

}
