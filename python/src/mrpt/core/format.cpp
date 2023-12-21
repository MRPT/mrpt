#include <iterator>
#include <memory>
#include <mrpt/core/format.h>
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

void bind_mrpt_core_format(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::to_string(double) file:mrpt/core/format.h line:31
	M("mrpt").def("to_string", (std::string (*)(double)) &mrpt::to_string<double>, "C++: mrpt::to_string(double) --> std::string", pybind11::arg("v"));

	// mrpt::to_string(int) file:mrpt/core/format.h line:31
	M("mrpt").def("to_string", (std::string (*)(int)) &mrpt::to_string<int>, "C++: mrpt::to_string(int) --> std::string", pybind11::arg("v"));

	// mrpt::to_string(unsigned long) file:mrpt/core/format.h line:31
	M("mrpt").def("to_string", (std::string (*)(unsigned long)) &mrpt::to_string<unsigned long>, "C++: mrpt::to_string(unsigned long) --> std::string", pybind11::arg("v"));

	// mrpt::to_string(unsigned int) file:mrpt/core/format.h line:31
	M("mrpt").def("to_string", (std::string (*)(unsigned int)) &mrpt::to_string<unsigned int>, "C++: mrpt::to_string(unsigned int) --> std::string", pybind11::arg("v"));

	// mrpt::to_string(float) file:mrpt/core/format.h line:31
	M("mrpt").def("to_string", (std::string (*)(float)) &mrpt::to_string<float>, "C++: mrpt::to_string(float) --> std::string", pybind11::arg("v"));

	// mrpt::to_string(std::string) file:mrpt/core/format.h line:36
	M("mrpt").def("to_string", (std::string (*)(std::string)) &mrpt::to_string<std::string>, "C++: mrpt::to_string(std::string) --> std::string", pybind11::arg("v"));

	// mrpt::to_string(bool) file:mrpt/core/format.h line:41
	M("mrpt").def("to_string", (std::string (*)(bool)) &mrpt::to_string<bool>, "C++: mrpt::to_string(bool) --> std::string", pybind11::arg("v"));

	// mrpt::to_string(const char *) file:mrpt/core/format.h line:46
	M("mrpt").def("to_string", (std::string (*)(const char *)) &mrpt::to_string<const char *>, "C++: mrpt::to_string(const char *) --> std::string", pybind11::arg("s"));

}
