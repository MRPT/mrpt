#include <chrono>
#include <mrpt/core/Clock.h>
#include <mrpt/core/reverse_bytes.h>
#include <ratio>

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

void bind_mrpt_core_reverse_bytes(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::reverseBytesInPlace(bool &) file:mrpt/core/reverse_bytes.h line:23
	M("mrpt").def("reverseBytesInPlace", (void (*)(bool &)) &mrpt::reverseBytesInPlace, "Reverse the order of the bytes of a given type (useful for transforming btw\n little/big endian)  \n\nC++: mrpt::reverseBytesInPlace(bool &) --> void", pybind11::arg("v_in_out"));

	// mrpt::reverseBytesInPlace(unsigned char &) file:mrpt/core/reverse_bytes.h line:24
	M("mrpt").def("reverseBytesInPlace", (void (*)(unsigned char &)) &mrpt::reverseBytesInPlace, "C++: mrpt::reverseBytesInPlace(unsigned char &) --> void", pybind11::arg("v_in_out"));

	// mrpt::reverseBytesInPlace(signed char &) file:mrpt/core/reverse_bytes.h line:25
	M("mrpt").def("reverseBytesInPlace", (void (*)(signed char &)) &mrpt::reverseBytesInPlace, "C++: mrpt::reverseBytesInPlace(signed char &) --> void", pybind11::arg("v_in_out"));

	// mrpt::reverseBytesInPlace(unsigned short &) file:mrpt/core/reverse_bytes.h line:26
	M("mrpt").def("reverseBytesInPlace", (void (*)(unsigned short &)) &mrpt::reverseBytesInPlace, "C++: mrpt::reverseBytesInPlace(unsigned short &) --> void", pybind11::arg("v_in_out"));

	// mrpt::reverseBytesInPlace(short &) file:mrpt/core/reverse_bytes.h line:27
	M("mrpt").def("reverseBytesInPlace", (void (*)(short &)) &mrpt::reverseBytesInPlace, "C++: mrpt::reverseBytesInPlace(short &) --> void", pybind11::arg("v_in_out"));

	// mrpt::reverseBytesInPlace(unsigned int &) file:mrpt/core/reverse_bytes.h line:28
	M("mrpt").def("reverseBytesInPlace", (void (*)(unsigned int &)) &mrpt::reverseBytesInPlace, "C++: mrpt::reverseBytesInPlace(unsigned int &) --> void", pybind11::arg("v_in_out"));

	// mrpt::reverseBytesInPlace(int &) file:mrpt/core/reverse_bytes.h line:29
	M("mrpt").def("reverseBytesInPlace", (void (*)(int &)) &mrpt::reverseBytesInPlace, "C++: mrpt::reverseBytesInPlace(int &) --> void", pybind11::arg("v_in_out"));

	// mrpt::reverseBytesInPlace(unsigned long &) file:mrpt/core/reverse_bytes.h line:30
	M("mrpt").def("reverseBytesInPlace", (void (*)(unsigned long &)) &mrpt::reverseBytesInPlace, "C++: mrpt::reverseBytesInPlace(unsigned long &) --> void", pybind11::arg("v_in_out"));

	// mrpt::reverseBytesInPlace(long &) file:mrpt/core/reverse_bytes.h line:31
	M("mrpt").def("reverseBytesInPlace", (void (*)(long &)) &mrpt::reverseBytesInPlace, "C++: mrpt::reverseBytesInPlace(long &) --> void", pybind11::arg("v_in_out"));

	// mrpt::reverseBytesInPlace(float &) file:mrpt/core/reverse_bytes.h line:32
	M("mrpt").def("reverseBytesInPlace", (void (*)(float &)) &mrpt::reverseBytesInPlace, "C++: mrpt::reverseBytesInPlace(float &) --> void", pybind11::arg("v_in_out"));

	// mrpt::reverseBytesInPlace(double &) file:mrpt/core/reverse_bytes.h line:33
	M("mrpt").def("reverseBytesInPlace", (void (*)(double &)) &mrpt::reverseBytesInPlace, "C++: mrpt::reverseBytesInPlace(double &) --> void", pybind11::arg("v_in_out"));

	// mrpt::reverseBytesInPlace(struct std::chrono::time_point<class mrpt::Clock, struct std::chrono::duration<long, struct std::ratio<1, 10000000> > > &) file:mrpt/core/reverse_bytes.h line:34
	M("mrpt").def("reverseBytesInPlace", (void (*)(struct std::chrono::time_point<class mrpt::Clock, struct std::chrono::duration<long, struct std::ratio<1, 10000000> > > &)) &mrpt::reverseBytesInPlace, "C++: mrpt::reverseBytesInPlace(struct std::chrono::time_point<class mrpt::Clock, struct std::chrono::duration<long, struct std::ratio<1, 10000000> > > &) --> void", pybind11::arg("v_in_out"));

	// mrpt::toNativeEndianness(const unsigned short &) file:mrpt/core/reverse_bytes.h line:54
	M("mrpt").def("toNativeEndianness", (unsigned short (*)(const unsigned short &)) &mrpt::toNativeEndianness<unsigned short>, "C++: mrpt::toNativeEndianness(const unsigned short &) --> unsigned short", pybind11::arg("v_in"));

	// mrpt::toNativeEndianness(const unsigned int &) file:mrpt/core/reverse_bytes.h line:54
	M("mrpt").def("toNativeEndianness", (unsigned int (*)(const unsigned int &)) &mrpt::toNativeEndianness<unsigned int>, "C++: mrpt::toNativeEndianness(const unsigned int &) --> unsigned int", pybind11::arg("v_in"));

}
