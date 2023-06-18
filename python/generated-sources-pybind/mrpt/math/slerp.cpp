#include <iterator>
#include <memory>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/slerp.h>
#include <mrpt/math/utils.h>
#include <string>

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

void bind_mrpt_math_slerp(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::math::slerp_ypr(const struct mrpt::math::TPose3D &, const struct mrpt::math::TPose3D &, const double, struct mrpt::math::TPose3D &) file:mrpt/math/slerp.h line:93
	M("mrpt::math").def("slerp_ypr", (void (*)(const struct mrpt::math::TPose3D &, const struct mrpt::math::TPose3D &, const double, struct mrpt::math::TPose3D &)) &mrpt::math::slerp_ypr, "as mrpt::math::TPose3D\n form as yaw,pitch,roll angles. XYZ are ignored.\n\nC++: mrpt::math::slerp_ypr(const struct mrpt::math::TPose3D &, const struct mrpt::math::TPose3D &, const double, struct mrpt::math::TPose3D &) --> void", pybind11::arg("q0"), pybind11::arg("q1"), pybind11::arg("t"), pybind11::arg("p"));

	// mrpt::math::medianFilter(const int &, int &, int, int) file:mrpt/math/utils.h line:72
	M("mrpt::math").def("medianFilter", [](const int & a0, int & a1, int const & a2) -> void { return mrpt::math::medianFilter(a0, a1, a2); }, "", pybind11::arg("inV"), pybind11::arg("outV"), pybind11::arg("winSize"));
	M("mrpt::math").def("medianFilter", (void (*)(const int &, int &, int, int)) &mrpt::math::medianFilter, "C++: mrpt::math::medianFilter(const int &, int &, int, int) --> void", pybind11::arg("inV"), pybind11::arg("outV"), pybind11::arg("winSize"), pybind11::arg("numberOfSigmas"));

	// mrpt::math::factorial64(unsigned int) file:mrpt/math/utils.h line:168
	M("mrpt::math").def("factorial64", (uint64_t (*)(unsigned int)) &mrpt::math::factorial64, "Computes the factorial of an integer number and returns it as a 64-bit\n integer number.\n\nC++: mrpt::math::factorial64(unsigned int) --> uint64_t", pybind11::arg("n"));

	// mrpt::math::factorial(unsigned int) file:mrpt/math/utils.h line:173
	M("mrpt::math").def("factorial", (double (*)(unsigned int)) &mrpt::math::factorial, "Computes the factorial of an integer number and returns it as a double value\n (internally it uses logarithms for avoiding overflow).\n\nC++: mrpt::math::factorial(unsigned int) --> double", pybind11::arg("n"));

}
