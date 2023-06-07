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

void bind_mrpt_poses_CPoseOrPoint_detail(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::detail::pose_point_impl file:mrpt/poses/CPoseOrPoint_detail.h line:91
		pybind11::class_<mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose2D,0>, std::shared_ptr<mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose2D,0>>> cl(M("mrpt::poses::detail"), "pose_point_impl_mrpt_poses_CPose2D_0_t", "");
		cl.def( pybind11::init( [](mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose2D,0> const &o){ return new mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose2D,0>(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose2D,0>(); } ) );
		cl.def("assign", (struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose2D, 0> & (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose2D,0>::*)(const struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose2D, 0> &)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose2D, 0>::operator=, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose2D, 0>::operator=(const struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose2D, 0> &) --> struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose2D, 0> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::poses::detail::pose_point_impl file:mrpt/poses/CPoseOrPoint_detail.h line:69
		pybind11::class_<mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D,1>, std::shared_ptr<mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D,1>>> cl(M("mrpt::poses::detail"), "pose_point_impl_mrpt_poses_CPose3D_1_t", "");
		cl.def( pybind11::init( [](mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D,1> const &o){ return new mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D,1>(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D,1>(); } ) );
		cl.def("z", (double & (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D,1>::*)()) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::z, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::z() --> double &", pybind11::return_value_policy::automatic);
		cl.def("z", (void (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D,1>::*)(const double)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::z, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::z(const double) --> void", pybind11::arg("v"));
		cl.def("z_incr", (void (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D,1>::*)(const double)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::z_incr, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::z_incr(const double) --> void", pybind11::arg("v"));
		cl.def("assign", (struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose3D, 1> & (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D,1>::*)(const struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose3D, 1> &)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::operator=, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::operator=(const struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose3D, 1> &) --> struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose3D, 1> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::poses::detail::pose_point_impl file:mrpt/poses/CPoseOrPoint_detail.h line:69
		pybind11::class_<mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D,1>, std::shared_ptr<mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D,1>>> cl(M("mrpt::poses::detail"), "pose_point_impl_mrpt_poses_CPoint3D_1_t", "");
		cl.def( pybind11::init( [](mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D,1> const &o){ return new mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D,1>(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D,1>(); } ) );
		cl.def("z", (double & (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D,1>::*)()) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D, 1>::z, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D, 1>::z() --> double &", pybind11::return_value_policy::automatic);
		cl.def("z", (void (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D,1>::*)(const double)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D, 1>::z, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D, 1>::z(const double) --> void", pybind11::arg("v"));
		cl.def("z_incr", (void (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D,1>::*)(const double)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D, 1>::z_incr, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D, 1>::z_incr(const double) --> void", pybind11::arg("v"));
		cl.def("assign", (struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPoint3D, 1> & (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D,1>::*)(const struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPoint3D, 1> &)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D, 1>::operator=, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D, 1>::operator=(const struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPoint3D, 1> &) --> struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPoint3D, 1> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::poses::detail::pose_point_impl file:mrpt/poses/CPoseOrPoint_detail.h line:69
		pybind11::class_<mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat,1>, std::shared_ptr<mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat,1>>> cl(M("mrpt::poses::detail"), "pose_point_impl_mrpt_poses_CPose3DQuat_1_t", "");
		cl.def( pybind11::init( [](mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat,1> const &o){ return new mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat,1>(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat,1>(); } ) );
		cl.def("z", (double & (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat,1>::*)()) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat, 1>::z, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat, 1>::z() --> double &", pybind11::return_value_policy::automatic);
		cl.def("z", (void (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat,1>::*)(const double)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat, 1>::z, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat, 1>::z(const double) --> void", pybind11::arg("v"));
		cl.def("z_incr", (void (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat,1>::*)(const double)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat, 1>::z_incr, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat, 1>::z_incr(const double) --> void", pybind11::arg("v"));
		cl.def("assign", (struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose3DQuat, 1> & (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat,1>::*)(const struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose3DQuat, 1> &)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat, 1>::operator=, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat, 1>::operator=(const struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose3DQuat, 1> &) --> struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose3DQuat, 1> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::poses::detail::pose_point_impl file:mrpt/poses/CPoseOrPoint_detail.h line:91
		pybind11::class_<mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint2D,0>, std::shared_ptr<mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint2D,0>>> cl(M("mrpt::poses::detail"), "pose_point_impl_mrpt_poses_CPoint2D_0_t", "");
		cl.def( pybind11::init( [](mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint2D,0> const &o){ return new mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint2D,0>(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint2D,0>(); } ) );
		cl.def("assign", (struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPoint2D, 0> & (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint2D,0>::*)(const struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPoint2D, 0> &)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint2D, 0>::operator=, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint2D, 0>::operator=(const struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPoint2D, 0> &) --> struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPoint2D, 0> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
