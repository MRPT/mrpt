#include <iterator>
#include <memory>
#include <mrpt/core/cpu.h>
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

void bind_mrpt_core_cpu_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::cpu::supports(enum mrpt::cpu::feature) file:mrpt/core/cpu.h line:74
	M("mrpt::cpu").def("supports", (bool (*)(enum mrpt::cpu::feature)) &mrpt::cpu::supports, "Returns true if the current CPU (and OS) supports the given CPU feature.\n \n\n\n \n\nC++: mrpt::cpu::supports(enum mrpt::cpu::feature) --> bool", pybind11::arg("f"));

	// mrpt::cpu::overrideDetectedFeature(enum mrpt::cpu::feature, bool) file:mrpt/core/cpu.h line:90
	M("mrpt::cpu").def("overrideDetectedFeature", (void (*)(enum mrpt::cpu::feature, bool)) &mrpt::cpu::overrideDetectedFeature, "Blindly enables/disables a CPU feature flag in the list\n of detected features to be reported in subsequent calls to\n mrpt::cpu::supports(). Could be used to disable a given CPU feature for\n benchmarking dynamically-dispatched functions.\n\n \n Enabling a feature that is not actually supported by the current CPU\n would probably lead to program crashes.\n\n \n\n \n\nC++: mrpt::cpu::overrideDetectedFeature(enum mrpt::cpu::feature, bool) --> void", pybind11::arg("f"), pybind11::arg("newValue"));

	// mrpt::cpu::features_as_string() file:mrpt/core/cpu.h line:99
	M("mrpt::cpu").def("features_as_string", (std::string (*)()) &mrpt::cpu::features_as_string, "Returns a string with detected features: \"MMX:1 SSE2:0 etc.\"\n \n\n\n \n\nC++: mrpt::cpu::features_as_string() --> std::string");

}
