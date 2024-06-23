#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/poses/Lie/SO.h>
#include <sstream> // __str__

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

void bind_mrpt_poses_Lie_SO(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::Lie::SO file:mrpt/poses/Lie/SO.h line:27
		pybind11::class_<mrpt::poses::Lie::SO<3U>, std::shared_ptr<mrpt::poses::Lie::SO<3U>>> cl(M("mrpt::poses::Lie"), "SO_3U_t", "Traits for SO(3), rotations in R^3 space.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::Lie::SO<3U>(); } ) );
		cl.def_static("exp", (class mrpt::math::CMatrixFixed<double, 3, 3> (*)(const class mrpt::math::CMatrixFixed<double, 3, 1> &)) &mrpt::poses::Lie::SO<3>::exp, "SO(3) exponential map \n.\n This is exactly the same than the Rodrigues formula.\n See 9.4.1 in  for the exponential map\n definition.\n\n - Input: 3-len vector in Lie algebra so(3)\n - Output: 3x3 rotation matrix in SO(3)\n\nC++: mrpt::poses::Lie::SO<3>::exp(const class mrpt::math::CMatrixFixed<double, 3, 1> &) --> class mrpt::math::CMatrixFixed<double, 3, 3>", pybind11::arg("x"));
		cl.def_static("log", (class mrpt::math::CMatrixFixed<double, 3, 1> (*)(const class mrpt::math::CMatrixFixed<double, 3, 3> &)) &mrpt::poses::Lie::SO<3>::log, "SO(3) logarithm map \n.\n See 10.3.1 in \n\n - Input: 3x3 rotation matrix in SO(3)\n - Output: 3-len vector in Lie algebra so(3)\n\nC++: mrpt::poses::Lie::SO<3>::log(const class mrpt::math::CMatrixFixed<double, 3, 3> &) --> class mrpt::math::CMatrixFixed<double, 3, 1>", pybind11::arg("R"));
		cl.def_static("fromYPR", (class mrpt::math::CMatrixFixed<double, 3, 3> (*)(const double, const double, const double)) &mrpt::poses::Lie::SO<3>::fromYPR, "Returns the 3x3 SO(3) rotation matrix from yaw, pitch, roll angles.\n See CPose3D for the axis conventions and a picture. \n\nC++: mrpt::poses::Lie::SO<3>::fromYPR(const double, const double, const double) --> class mrpt::math::CMatrixFixed<double, 3, 3>", pybind11::arg("yaw"), pybind11::arg("pitch"), pybind11::arg("roll"));
		cl.def_static("vee_RmRt", (class mrpt::math::CMatrixFixed<double, 3, 1> (*)(const class mrpt::math::CMatrixFixed<double, 3, 3> &)) &mrpt::poses::Lie::SO<3>::vee_RmRt, "Returns vee(R-R'), which is an approximation to 2*vee(logmat(R)) for\n small rotations. \n\nC++: mrpt::poses::Lie::SO<3>::vee_RmRt(const class mrpt::math::CMatrixFixed<double, 3, 3> &) --> class mrpt::math::CMatrixFixed<double, 3, 1>", pybind11::arg("R"));
	}
	{ // mrpt::poses::Lie::SO file:mrpt/poses/Lie/SO.h line:87
		pybind11::class_<mrpt::poses::Lie::SO<2U>, std::shared_ptr<mrpt::poses::Lie::SO<2U>>> cl(M("mrpt::poses::Lie"), "SO_2U_t", "Traits for SO(2), rotations in R^2 space.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::Lie::SO<2U>(); } ) );
	}
}
