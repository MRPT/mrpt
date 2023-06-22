#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/homog_matrices.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <pybind11/stl.h>

#include <Eigen/Dense>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mrpt_math_homog_matrices(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::math::homogeneousMatrixInverse(const class mrpt::math::CMatrixFixed<double, 3, 3> &, const class mrpt::math::CMatrixFixed<double, 3, 1> &, class mrpt::math::CMatrixFixed<double, 3, 3> &, class mrpt::math::CMatrixFixed<double, 3, 1> &) file:mrpt/math/homog_matrices.h line:74
	M("mrpt::math").def("homogeneousMatrixInverse", (void (*)(const class mrpt::math::CMatrixFixed<double, 3, 3> &, const class mrpt::math::CMatrixFixed<double, 3, 1> &, class mrpt::math::CMatrixFixed<double, 3, 3> &, class mrpt::math::CMatrixFixed<double, 3, 1> &)) &mrpt::math::homogeneousMatrixInverse<mrpt::math::CMatrixFixed<double, 3, 3>,mrpt::math::CMatrixFixed<double, 3, 1>,mrpt::math::CMatrixFixed<double, 3, 3>,mrpt::math::CMatrixFixed<double, 3, 1>>, "C++: mrpt::math::homogeneousMatrixInverse(const class mrpt::math::CMatrixFixed<double, 3, 3> &, const class mrpt::math::CMatrixFixed<double, 3, 1> &, class mrpt::math::CMatrixFixed<double, 3, 3> &, class mrpt::math::CMatrixFixed<double, 3, 1> &) --> void", pybind11::arg("in_R"), pybind11::arg("in_xyz"), pybind11::arg("out_R"), pybind11::arg("out_xyz"));

}
