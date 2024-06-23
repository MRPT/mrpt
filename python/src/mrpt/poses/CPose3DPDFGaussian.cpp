#include <mrpt/poses/CPose3DPDFGaussian.h>

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

void bind_mrpt_poses_CPose3DPDFGaussian(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::global_settings::USE_SUT_QUAT2EULER_CONVERSION(bool) file:mrpt/poses/CPose3DPDFGaussian.h line:235
	M("mrpt::global_settings").def("USE_SUT_QUAT2EULER_CONVERSION", (void (*)(bool)) &mrpt::global_settings::USE_SUT_QUAT2EULER_CONVERSION, "If set to true (false), a Scaled Unscented Transform is used instead of a\nlinear approximation with Jacobians.\n Affects to:\n		- CPose3DPDFGaussian::CPose3DPDFGaussian( const CPose3DQuatPDFGaussian\n&o)\n\nC++: mrpt::global_settings::USE_SUT_QUAT2EULER_CONVERSION(bool) --> void", pybind11::arg("value"));

	// mrpt::global_settings::USE_SUT_QUAT2EULER_CONVERSION() file:mrpt/poses/CPose3DPDFGaussian.h line:236
	M("mrpt::global_settings").def("USE_SUT_QUAT2EULER_CONVERSION", (bool (*)()) &mrpt::global_settings::USE_SUT_QUAT2EULER_CONVERSION, "C++: mrpt::global_settings::USE_SUT_QUAT2EULER_CONVERSION() --> bool");

}
