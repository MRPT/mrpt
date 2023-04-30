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
#include <stl_binders.hpp>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mrpt_poses_Lie_SE(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::Lie::SE file:mrpt/poses/Lie/SE.h line:38
		pybind11::class_<mrpt::poses::Lie::SE<3>, std::shared_ptr<mrpt::poses::Lie::SE<3>>> cl(M("mrpt::poses::Lie"), "SE_3_t", "Traits for SE(3), rigid-body transformations in R^3 space.\n See indidual members for documentation, or  blanco_se3_tutorial for a\n general overview.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::Lie::SE<3>(); } ) );
	}
	{ // mrpt::poses::Lie::SE file:mrpt/poses/Lie/SE.h line:162
		pybind11::class_<mrpt::poses::Lie::SE<2>, std::shared_ptr<mrpt::poses::Lie::SE<2>>> cl(M("mrpt::poses::Lie"), "SE_2_t", "Traits for SE(2), rigid-body transformations in R^2 space.\n See indidual members for documentation, or  blanco_se3_tutorial for a\n general overview.\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::Lie::SE<2>(); } ) );
	}
}
