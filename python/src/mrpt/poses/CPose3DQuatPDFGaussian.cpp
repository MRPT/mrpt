#include <mrpt/poses/CPose3DQuatPDFGaussian.h>

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

void bind_mrpt_poses_CPose3DQuatPDFGaussian(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::global_settings::USE_SUT_EULER2QUAT_CONVERSION(bool) file:mrpt/poses/CPose3DQuatPDFGaussian.h line:194
	M("mrpt::global_settings").def("USE_SUT_EULER2QUAT_CONVERSION", (void (*)(bool)) &mrpt::global_settings::USE_SUT_EULER2QUAT_CONVERSION, "If set to true (default), a Scaled Unscented Transform is used instead of a\nlinear approximation with Jacobians.\n Affects to:\n		- CPose3DQuatPDFGaussian::copyFrom(const CPose3DPDFGaussian &o)\n		- CPose3DQuatPDFGaussianInf::copyFrom(const CPose3DPDFGaussianInf &o)\n\nC++: mrpt::global_settings::USE_SUT_EULER2QUAT_CONVERSION(bool) --> void", pybind11::arg("value"));

	// mrpt::global_settings::USE_SUT_EULER2QUAT_CONVERSION() file:mrpt/poses/CPose3DQuatPDFGaussian.h line:195
	M("mrpt::global_settings").def("USE_SUT_EULER2QUAT_CONVERSION", (bool (*)()) &mrpt::global_settings::USE_SUT_EULER2QUAT_CONVERSION, "C++: mrpt::global_settings::USE_SUT_EULER2QUAT_CONVERSION() --> bool");

}
