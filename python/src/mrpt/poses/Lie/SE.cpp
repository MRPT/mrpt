#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <sstream> // __str__
#include <string>

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

void bind_mrpt_poses_Lie_SE(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::Lie::SE file:mrpt/poses/Lie/SE.h line:38
		pybind11::class_<mrpt::poses::Lie::SE<3U>, std::shared_ptr<mrpt::poses::Lie::SE<3U>>> cl(M("mrpt::poses::Lie"), "SE_3U_t", "Traits for SE(3), rigid-body transformations in R^3 space.\n See indidual members for documentation, or  for a\n general overview.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::Lie::SE<3U>(); } ) );
		cl.def_static("exp", (class mrpt::poses::CPose3D (*)(const class mrpt::math::CMatrixFixed<double, 6, 1> &)) &mrpt::poses::Lie::SE<3>::exp, "Retraction to SE(3), a **pseudo-exponential** map \n\n\n and its Jacobian.\n - Input: 6-len vector in Lie algebra se(3) [x,y,z, rx,ry,rz]\n - Output: translation and rotation in SE(3) as CPose3D\n Note that this method implements retraction via a **pseudo-exponential**,\n where only the rotational part undergoes a real matrix exponential,\n while the translation is left unmodified. This is done for computational\n efficiency, and does not change the results of optimizations as long\n as the corresponding local coordinates (pseudo-logarithm) are used as\n well.\n\n See section 9.4.2 in \n   \n\nC++: mrpt::poses::Lie::SE<3>::exp(const class mrpt::math::CMatrixFixed<double, 6, 1> &) --> class mrpt::poses::CPose3D", pybind11::arg("x"));
		cl.def_static("log", (class mrpt::math::CMatrixFixed<double, 6, 1> (*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::Lie::SE<3>::log, "SE(3) **pseudo-logarithm** map \n\n\n - Input: translation and rotation in SE(3) as CPose3D\n - Output: 6-len vector in Lie algebra se(3) [x,y,z, rx,ry,rz]\n\n See exp() for the explanation about the \"pseudo\" name.\n For the formulas, see section 9.4.2 in \n   \n\nC++: mrpt::poses::Lie::SE<3>::log(const class mrpt::poses::CPose3D &) --> class mrpt::math::CMatrixFixed<double, 6, 1>", pybind11::arg("P"));
		cl.def_static("asManifoldVector", (class mrpt::math::CMatrixFixed<double, 12, 1> (*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::Lie::SE<3>::asManifoldVector, "Returns a vector with all manifold matrix elements in column-major\n order. For SE(3), it is a 3x4=12 vector. \n\nC++: mrpt::poses::Lie::SE<3>::asManifoldVector(const class mrpt::poses::CPose3D &) --> class mrpt::math::CMatrixFixed<double, 12, 1>", pybind11::arg("pose"));
		cl.def_static("fromManifoldVector", (class mrpt::poses::CPose3D (*)(const class mrpt::math::CMatrixFixed<double, 12, 1> &)) &mrpt::poses::Lie::SE<3>::fromManifoldVector, "The inverse operation of asManifoldVector() \n\nC++: mrpt::poses::Lie::SE<3>::fromManifoldVector(const class mrpt::math::CMatrixFixed<double, 12, 1> &) --> class mrpt::poses::CPose3D", pybind11::arg("v"));
	}
	{ // mrpt::poses::Lie::SE file:mrpt/poses/Lie/SE.h line:164
		pybind11::class_<mrpt::poses::Lie::SE<2U>, std::shared_ptr<mrpt::poses::Lie::SE<2U>>> cl(M("mrpt::poses::Lie"), "SE_2U_t", "Traits for SE(2), rigid-body transformations in R^2 space.\n See indidual members for documentation, or  for a\n general overview.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::Lie::SE<2U>(); } ) );
		cl.def_static("exp", (class mrpt::poses::CPose2D (*)(const class mrpt::math::CMatrixFixed<double, 3, 1> &)) &mrpt::poses::Lie::SE<2>::exp, "Exponential map in SE(2), takes [x,y,phi] and returns a CPose2D \n\nC++: mrpt::poses::Lie::SE<2>::exp(const class mrpt::math::CMatrixFixed<double, 3, 1> &) --> class mrpt::poses::CPose2D", pybind11::arg("x"));
		cl.def_static("log", (class mrpt::math::CMatrixFixed<double, 3, 1> (*)(const class mrpt::poses::CPose2D &)) &mrpt::poses::Lie::SE<2>::log, "Logarithm map in SE(2), takes a CPose2D and returns [X,Y, phi] \n\nC++: mrpt::poses::Lie::SE<2>::log(const class mrpt::poses::CPose2D &) --> class mrpt::math::CMatrixFixed<double, 3, 1>", pybind11::arg("P"));
		cl.def_static("asManifoldVector", (class mrpt::math::CMatrixFixed<double, 3, 1> (*)(const class mrpt::poses::CPose2D &)) &mrpt::poses::Lie::SE<2>::asManifoldVector, "Returns a vector with all manifold matrix elements in column-major\n order. For SE(2), though, it directly returns the vector [x,y,phi] for\n efficiency in comparison to 3x3 homogeneous coordinates. \n\nC++: mrpt::poses::Lie::SE<2>::asManifoldVector(const class mrpt::poses::CPose2D &) --> class mrpt::math::CMatrixFixed<double, 3, 1>", pybind11::arg("pose"));
		cl.def_static("fromManifoldVector", (class mrpt::poses::CPose2D (*)(const class mrpt::math::CMatrixFixed<double, 3, 1> &)) &mrpt::poses::Lie::SE<2>::fromManifoldVector, "The inverse operation of asManifoldVector() \n\nC++: mrpt::poses::Lie::SE<2>::fromManifoldVector(const class mrpt::math::CMatrixFixed<double, 3, 1> &) --> class mrpt::poses::CPose2D", pybind11::arg("v"));
		cl.def_static("jacob_dAB_dA", (class mrpt::math::CMatrixFixed<double, 3, 3> (*)(const class mrpt::poses::CPose2D &, const class mrpt::poses::CPose2D &)) &mrpt::poses::Lie::SE<2>::jacob_dAB_dA, "Jacobian of the pose composition A*B for SE(2) with respect to A.\n \n\n See appendix B of \n   \n\nC++: mrpt::poses::Lie::SE<2>::jacob_dAB_dA(const class mrpt::poses::CPose2D &, const class mrpt::poses::CPose2D &) --> class mrpt::math::CMatrixFixed<double, 3, 3>", pybind11::arg("A"), pybind11::arg("B"));
		cl.def_static("jacob_dAB_dB", (class mrpt::math::CMatrixFixed<double, 3, 3> (*)(const class mrpt::poses::CPose2D &, const class mrpt::poses::CPose2D &)) &mrpt::poses::Lie::SE<2>::jacob_dAB_dB, "Jacobian of the pose composition A*B for SE(2) with respect to B.\n \n\n See appendix B of \n   \n\nC++: mrpt::poses::Lie::SE<2>::jacob_dAB_dB(const class mrpt::poses::CPose2D &, const class mrpt::poses::CPose2D &) --> class mrpt::math::CMatrixFixed<double, 3, 3>", pybind11::arg("A"), pybind11::arg("B"));
		cl.def_static("jacob_dDexpe_de", (class mrpt::math::CMatrixFixed<double, 3, 3> (*)(const class mrpt::poses::CPose2D &)) &mrpt::poses::Lie::SE<2>::jacob_dDexpe_de, "Jacobian d (D * e^eps) / d eps , with eps=increment in Lie Algebra.\n \n\n See appendix B in \n   \n\nC++: mrpt::poses::Lie::SE<2>::jacob_dDexpe_de(const class mrpt::poses::CPose2D &) --> class mrpt::math::CMatrixFixed<double, 3, 3>", pybind11::arg("D"));
	}
}
