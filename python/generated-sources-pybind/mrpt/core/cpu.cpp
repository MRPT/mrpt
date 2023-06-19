#include <mrpt/core/cpu.h>

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

void bind_mrpt_core_cpu(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::cpu::feature file:mrpt/core/cpu.h line:22
	pybind11::enum_<mrpt::cpu::feature>(M("mrpt::cpu"), "feature", "OS-portable set of CPU feature definitions, for usage in mrpt::cpu::supports\n \n\n\n ")
		.value("MMX", mrpt::cpu::feature::MMX)
		.value("POPCNT", mrpt::cpu::feature::POPCNT)
		.value("SSE", mrpt::cpu::feature::SSE)
		.value("SSE2", mrpt::cpu::feature::SSE2)
		.value("SSE3", mrpt::cpu::feature::SSE3)
		.value("SSSE3", mrpt::cpu::feature::SSSE3)
		.value("SSE4_1", mrpt::cpu::feature::SSE4_1)
		.value("SSE4_2", mrpt::cpu::feature::SSE4_2)
		.value("AVX", mrpt::cpu::feature::AVX)
		.value("AVX2", mrpt::cpu::feature::AVX2)
		.value("FEATURE_COUNT", mrpt::cpu::feature::FEATURE_COUNT);

;

}
