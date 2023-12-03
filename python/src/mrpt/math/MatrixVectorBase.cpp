#include <mrpt/math/MatrixVectorBase.h>

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

void bind_mrpt_math_MatrixVectorBase(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::math::TMatrixTextFileFormat file:mrpt/math/MatrixVectorBase.h line:35
	pybind11::enum_<mrpt::math::TMatrixTextFileFormat>(M("mrpt::math"), "TMatrixTextFileFormat", pybind11::arithmetic(), "Selection of the number format in MatrixVectorBase::saveToTextFile()\n \n")
		.value("MATRIX_FORMAT_ENG", mrpt::math::MATRIX_FORMAT_ENG)
		.value("MATRIX_FORMAT_FIXED", mrpt::math::MATRIX_FORMAT_FIXED)
		.value("MATRIX_FORMAT_INT", mrpt::math::MATRIX_FORMAT_INT)
		.export_values();

;

}
