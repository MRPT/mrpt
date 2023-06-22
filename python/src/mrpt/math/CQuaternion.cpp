#include <mrpt/math/CQuaternion.h>

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

void bind_mrpt_math_CQuaternion(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::math::TConstructorFlags_Quaternions file:mrpt/math/CQuaternion.h line:20
	pybind11::enum_<mrpt::math::TConstructorFlags_Quaternions>(M("mrpt::math"), "TConstructorFlags_Quaternions", pybind11::arithmetic(), "")
		.value("UNINITIALIZED_QUATERNION", mrpt::math::UNINITIALIZED_QUATERNION)
		.export_values();

;

}
