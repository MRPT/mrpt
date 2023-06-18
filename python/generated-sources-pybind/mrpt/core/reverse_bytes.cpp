#include <mrpt/core/reverse_bytes.h>

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

void bind_mrpt_core_reverse_bytes(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::toNativeEndianness(const unsigned short &) file:mrpt/core/reverse_bytes.h line:54
	M("mrpt").def("toNativeEndianness", (unsigned short (*)(const unsigned short &)) &mrpt::toNativeEndianness<unsigned short>, "C++: mrpt::toNativeEndianness(const unsigned short &) --> unsigned short", pybind11::arg("v_in"));

	// mrpt::toNativeEndianness(const unsigned int &) file:mrpt/core/reverse_bytes.h line:54
	M("mrpt").def("toNativeEndianness", (unsigned int (*)(const unsigned int &)) &mrpt::toNativeEndianness<unsigned int>, "C++: mrpt::toNativeEndianness(const unsigned int &) --> unsigned int", pybind11::arg("v_in"));

}
