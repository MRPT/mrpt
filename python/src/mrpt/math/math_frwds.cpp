#include <mrpt/math/math_frwds.h>

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

void bind_mrpt_math_math_frwds(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::math::TConstructorFlags_Matrices file:mrpt/math/math_frwds.h line:59
	pybind11::enum_<mrpt::math::TConstructorFlags_Matrices>(M("mrpt::math"), "TConstructorFlags_Matrices", pybind11::arithmetic(), "For usage in one of the constructors of CMatrixFixed or\n   CMatrixDynamic (and derived classes), if it's not required\n   to fill it with zeros at the constructor to save time. ")
		.value("UNINITIALIZED_MATRIX", mrpt::math::UNINITIALIZED_MATRIX)
		.export_values();

;

}
