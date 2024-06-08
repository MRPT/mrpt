#include <mrpt/core/bits_math.h>

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

void bind_mrpt_core_bits_math(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::square(const double) file:mrpt/core/bits_math.h line:26
	M("mrpt").def("square", (double (*)(const double)) &mrpt::square<double,double>, "C++: mrpt::square(const double) --> double", pybind11::arg("x"));

	// mrpt::square(const float) file:mrpt/core/bits_math.h line:26
	M("mrpt").def("square", (float (*)(const float)) &mrpt::square<float,float>, "C++: mrpt::square(const float) --> float", pybind11::arg("x"));

	// mrpt::hypot_fast(const double, const double) file:mrpt/core/bits_math.h line:34
	M("mrpt").def("hypot_fast", (double (*)(const double, const double)) &mrpt::hypot_fast<double>, "C++: mrpt::hypot_fast(const double, const double) --> double", pybind11::arg("x"), pybind11::arg("y"));

	// mrpt::DEG2RAD(const double) file:mrpt/core/bits_math.h line:50
	M("mrpt").def("DEG2RAD", (double (*)(const double)) &mrpt::DEG2RAD, "Degrees to radians  \n\nC++: mrpt::DEG2RAD(const double) --> double", pybind11::arg("x"));

	// mrpt::DEG2RAD(const float) file:mrpt/core/bits_math.h line:52
	M("mrpt").def("DEG2RAD", (float (*)(const float)) &mrpt::DEG2RAD, "Degrees to radians \n\nC++: mrpt::DEG2RAD(const float) --> float", pybind11::arg("x"));

	// mrpt::DEG2RAD(const int) file:mrpt/core/bits_math.h line:54
	M("mrpt").def("DEG2RAD", (double (*)(const int)) &mrpt::DEG2RAD, "Degrees to radians \n\nC++: mrpt::DEG2RAD(const int) --> double", pybind11::arg("x"));

	// mrpt::RAD2DEG(const double) file:mrpt/core/bits_math.h line:56
	M("mrpt").def("RAD2DEG", (double (*)(const double)) &mrpt::RAD2DEG, "Radians to degrees \n\nC++: mrpt::RAD2DEG(const double) --> double", pybind11::arg("x"));

	// mrpt::RAD2DEG(const float) file:mrpt/core/bits_math.h line:58
	M("mrpt").def("RAD2DEG", (float (*)(const float)) &mrpt::RAD2DEG, "Radians to degrees \n\nC++: mrpt::RAD2DEG(const float) --> float", pybind11::arg("x"));

	// mrpt::DEG2RAD(const long double) file:mrpt/core/bits_math.h line:64
	M("mrpt").def("DEG2RAD", (long double (*)(const long double)) &mrpt::DEG2RAD, "Degrees to radians \n\nC++: mrpt::DEG2RAD(const long double) --> long double", pybind11::arg("x"));

	// mrpt::RAD2DEG(const long double) file:mrpt/core/bits_math.h line:66
	M("mrpt").def("RAD2DEG", (long double (*)(const long double)) &mrpt::RAD2DEG, "Radians to degrees \n\nC++: mrpt::RAD2DEG(const long double) --> long double", pybind11::arg("x"));

}
