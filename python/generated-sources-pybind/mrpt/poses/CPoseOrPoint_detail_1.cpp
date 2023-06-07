#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/poses/CPoseOrPoint_detail.h>
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

void bind_mrpt_poses_CPoseOrPoint_detail_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::detail::T3DTypeHelper file:mrpt/poses/CPoseOrPoint_detail.h line:24
		pybind11::class_<mrpt::poses::detail::T3DTypeHelper<mrpt::poses::CPoint2D>, std::shared_ptr<mrpt::poses::detail::T3DTypeHelper<mrpt::poses::CPoint2D>>> cl(M("mrpt::poses::detail"), "T3DTypeHelper_mrpt_poses_CPoint2D_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::poses::detail::T3DTypeHelper<mrpt::poses::CPoint2D>(); } ) );
	}
	{ // mrpt::poses::detail::T3DTypeHelper file:mrpt/poses/CPoseOrPoint_detail.h line:32
		pybind11::class_<mrpt::poses::detail::T3DTypeHelper<mrpt::poses::CPoint3D>, std::shared_ptr<mrpt::poses::detail::T3DTypeHelper<mrpt::poses::CPoint3D>>> cl(M("mrpt::poses::detail"), "T3DTypeHelper_mrpt_poses_CPoint3D_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::poses::detail::T3DTypeHelper<mrpt::poses::CPoint3D>(); } ) );
	}
	{ // mrpt::poses::detail::T3DTypeHelper file:mrpt/poses/CPoseOrPoint_detail.h line:40
		pybind11::class_<mrpt::poses::detail::T3DTypeHelper<mrpt::poses::CPose2D>, std::shared_ptr<mrpt::poses::detail::T3DTypeHelper<mrpt::poses::CPose2D>>> cl(M("mrpt::poses::detail"), "T3DTypeHelper_mrpt_poses_CPose2D_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::poses::detail::T3DTypeHelper<mrpt::poses::CPose2D>(); } ) );
	}
	{ // mrpt::poses::detail::T3DTypeHelper file:mrpt/poses/CPoseOrPoint_detail.h line:48
		pybind11::class_<mrpt::poses::detail::T3DTypeHelper<mrpt::poses::CPose3D>, std::shared_ptr<mrpt::poses::detail::T3DTypeHelper<mrpt::poses::CPose3D>>> cl(M("mrpt::poses::detail"), "T3DTypeHelper_mrpt_poses_CPose3D_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::poses::detail::T3DTypeHelper<mrpt::poses::CPose3D>(); } ) );
	}
	{ // mrpt::poses::detail::T3DTypeHelper file:mrpt/poses/CPoseOrPoint_detail.h line:56
		pybind11::class_<mrpt::poses::detail::T3DTypeHelper<mrpt::poses::CPose3DQuat>, std::shared_ptr<mrpt::poses::detail::T3DTypeHelper<mrpt::poses::CPose3DQuat>>> cl(M("mrpt::poses::detail"), "T3DTypeHelper_mrpt_poses_CPose3DQuat_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::poses::detail::T3DTypeHelper<mrpt::poses::CPose3DQuat>(); } ) );
	}
}
