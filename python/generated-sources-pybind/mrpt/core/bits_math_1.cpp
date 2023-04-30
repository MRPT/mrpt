#include <mrpt/core/bits_math.h>

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

void bind_mrpt_core_bits_math_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::sign(double) file:mrpt/core/bits_math.h line:96
	M("mrpt").def("sign", (int (*)(double)) &mrpt::sign<double>, "C++: mrpt::sign(double) --> int", pybind11::arg("x"));

	// mrpt::signWithZero(double) file:mrpt/core/bits_math.h line:103
	M("mrpt").def("signWithZero", (int (*)(double)) &mrpt::signWithZero<double>, "C++: mrpt::signWithZero(double) --> int", pybind11::arg("x"));

	// mrpt::keep_min(float &, const float) file:mrpt/core/bits_math.h line:141
	M("mrpt").def("keep_min", (void (*)(float &, const float)) &mrpt::keep_min<float,float>, "C++: mrpt::keep_min(float &, const float) --> void", pybind11::arg("var"), pybind11::arg("test_val"));

	// mrpt::keep_min(double &, const double) file:mrpt/core/bits_math.h line:141
	M("mrpt").def("keep_min", (void (*)(double &, const double)) &mrpt::keep_min<double,double>, "C++: mrpt::keep_min(double &, const double) --> void", pybind11::arg("var"), pybind11::arg("test_val"));

	// mrpt::keep_min(double &, const float) file:mrpt/core/bits_math.h line:141
	M("mrpt").def("keep_min", (void (*)(double &, const float)) &mrpt::keep_min<double,float>, "C++: mrpt::keep_min(double &, const float) --> void", pybind11::arg("var"), pybind11::arg("test_val"));

	// mrpt::keep_max(float &, const float) file:mrpt/core/bits_math.h line:148
	M("mrpt").def("keep_max", (void (*)(float &, const float)) &mrpt::keep_max<float,float>, "C++: mrpt::keep_max(float &, const float) --> void", pybind11::arg("var"), pybind11::arg("test_val"));

	// mrpt::keep_max(double &, const double) file:mrpt/core/bits_math.h line:148
	M("mrpt").def("keep_max", (void (*)(double &, const double)) &mrpt::keep_max<double,double>, "C++: mrpt::keep_max(double &, const double) --> void", pybind11::arg("var"), pybind11::arg("test_val"));

	// mrpt::keep_max(double &, const float) file:mrpt/core/bits_math.h line:148
	M("mrpt").def("keep_max", (void (*)(double &, const float)) &mrpt::keep_max<double,float>, "C++: mrpt::keep_max(double &, const float) --> void", pybind11::arg("var"), pybind11::arg("test_val"));

	// mrpt::d2f(const double) file:mrpt/core/bits_math.h line:185
	M("mrpt").def("d2f", (float (*)(const double)) &mrpt::d2f, "shortcut for static_cast<float>(double) \n\nC++: mrpt::d2f(const double) --> float", pybind11::arg("d"));

	// mrpt::f2u8(const float) file:mrpt/core/bits_math.h line:189
	M("mrpt").def("f2u8", (unsigned char (*)(const float)) &mrpt::f2u8, "converts a float [0,1] into an uint8_t [0,255] (without checking for out of\n bounds) \n\n u8tof \n\nC++: mrpt::f2u8(const float) --> unsigned char", pybind11::arg("f"));

	// mrpt::u8tof(const unsigned char) file:mrpt/core/bits_math.h line:192
	M("mrpt").def("u8tof", (float (*)(const unsigned char)) &mrpt::u8tof, "converts a uint8_t [0,255] into a float [0,1] \n f2u8 \n\nC++: mrpt::u8tof(const unsigned char) --> float", pybind11::arg("v"));

}
