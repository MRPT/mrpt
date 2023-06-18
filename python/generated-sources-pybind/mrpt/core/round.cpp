#include <mrpt/core/round.h>

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

void bind_mrpt_core_round(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::round(const float) file:mrpt/core/round.h line:25
	M("mrpt").def("round", (int (*)(const float)) &mrpt::round<float>, "C++: mrpt::round(const float) --> int", pybind11::arg("value"));

	// mrpt::round(const double) file:mrpt/core/round.h line:25
	M("mrpt").def("round", (int (*)(const double)) &mrpt::round<double>, "C++: mrpt::round(const double) --> int", pybind11::arg("value"));

}
