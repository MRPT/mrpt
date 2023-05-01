#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <sstream> // __str__

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

void bind_mrpt_math_math_frwds(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::math::TConstructorFlags_Matrices file:mrpt/math/math_frwds.h line:55
	pybind11::enum_<mrpt::math::TConstructorFlags_Matrices>(M("mrpt::math"), "TConstructorFlags_Matrices", pybind11::arithmetic(), "For usage in one of the constructors of CMatrixFixed or\n   CMatrixDynamic (and derived classes), if it's not required\n	 to fill it with zeros at the constructor to save time. ")
		.value("UNINITIALIZED_MATRIX", mrpt::math::UNINITIALIZED_MATRIX)
		.export_values();

;

	{ // mrpt::math::matrix_size_t file:mrpt/math/matrix_size_t.h line:20
		pybind11::class_<mrpt::math::matrix_size_t, std::shared_ptr<mrpt::math::matrix_size_t>, std::array<unsigned long,2>> cl(M("mrpt::math"), "matrix_size_t", "Auxiliary class used in CMatrixDynamic:size(), CMatrixDynamic::resize(),\n CMatrixFixed::size(), CMatrixFixed::resize(), to mimic the\n behavior of STL-containers.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::math::matrix_size_t(); } ) );
		cl.def( pybind11::init<const unsigned long, const unsigned long>(), pybind11::arg("rows"), pybind11::arg("cols") );

		cl.def( pybind11::init( [](mrpt::math::matrix_size_t const &o){ return new mrpt::math::matrix_size_t(o); } ) );
	}
}
