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

// mrpt::poses::CPoseOrPoint file:mrpt/poses/CPoseOrPoint.h line:123
struct PyCallBack_mrpt_poses_CPoseOrPoint_mrpt_poses_CPose2D_3_t : public mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3> {
	using mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3>::CPoseOrPoint;

	void setToNaN() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3> *>(this), "setToNaN");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CPoseOrPoint::setToNaN\"");
	}
};

// mrpt::poses::CPoseOrPoint file:mrpt/poses/CPoseOrPoint.h line:123
struct PyCallBack_mrpt_poses_CPoseOrPoint_mrpt_poses_CPose3D_6_t : public mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6> {
	using mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6>::CPoseOrPoint;

	void setToNaN() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6> *>(this), "setToNaN");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CPoseOrPoint::setToNaN\"");
	}
};

// mrpt::poses::CPoseOrPoint file:mrpt/poses/CPoseOrPoint.h line:123
struct PyCallBack_mrpt_poses_CPoseOrPoint_mrpt_poses_CPoint3D_3_t : public mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3> {
	using mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3>::CPoseOrPoint;

	void setToNaN() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3> *>(this), "setToNaN");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		pybind11::pybind11_fail("Tried to call pure virtual function \"CPoseOrPoint::setToNaN\"");
	}
};

void bind_mrpt_poses_CPoseOrPoint(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::poses::TConstructorFlags_Poses file:mrpt/poses/CPoseOrPoint.h line:32
	pybind11::enum_<mrpt::poses::TConstructorFlags_Poses>(M("mrpt::poses"), "TConstructorFlags_Poses", pybind11::arithmetic(), "")
		.value("UNINITIALIZED_POSE", mrpt::poses::UNINITIALIZED_POSE)
		.export_values();

;

	{ // mrpt::poses::CPoseOrPoint file:mrpt/poses/CPoseOrPoint.h line:123
		pybind11::class_<mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3>, std::shared_ptr<mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3>>, PyCallBack_mrpt_poses_CPoseOrPoint_mrpt_poses_CPose2D_3_t, mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose2D,0>> cl(M("mrpt::poses"), "CPoseOrPoint_mrpt_poses_CPose2D_3_t", "");
		cl.def(pybind11::init<PyCallBack_mrpt_poses_CPoseOrPoint_mrpt_poses_CPose2D_3_t const &>());
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_poses_CPoseOrPoint_mrpt_poses_CPose2D_3_t(); } ) );
		cl.def("derived", (class mrpt::poses::CPose2D & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::derived, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::derived() --> class mrpt::poses::CPose2D &", pybind11::return_value_policy::automatic);
		cl.def("x", (double & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::x, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::x() --> double &", pybind11::return_value_policy::automatic);
		cl.def("y", (double & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::y, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::y() --> double &", pybind11::return_value_policy::automatic);
		cl.def("x", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::x, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::x(const double) --> void", pybind11::arg("v"));
		cl.def("y", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::y, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::y(const double) --> void", pybind11::arg("v"));
		cl.def("x_incr", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::x_incr, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::x_incr(const double) --> void", pybind11::arg("v"));
		cl.def("y_incr", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::y_incr, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::y_incr(const double) --> void", pybind11::arg("v"));
		cl.def_static("is3DPoseOrPoint", (bool (*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::is3DPoseOrPoint, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::is3DPoseOrPoint() --> bool");
		cl.def("distance2DToSquare", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3>::*)(double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::distance2DToSquare, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::distance2DToSquare(double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"));
		cl.def("distance3DToSquare", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3>::*)(double, double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::distance3DToSquare, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::distance3DToSquare(double, double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"), pybind11::arg("az"));
		cl.def("distance2DTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3>::*)(double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::distance2DTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::distance2DTo(double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"));
		cl.def("distance3DTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3>::*)(double, double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::distance3DTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::distance3DTo(double, double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"), pybind11::arg("az"));
		cl.def("distanceTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3>::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::distanceTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::distanceTo(const struct mrpt::math::TPoint3D_<double> &) const --> double", pybind11::arg("b"));
		cl.def("norm", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3>::*)() const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::norm, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::norm() const --> double");
		cl.def("setToNaN", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::setToNaN, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::setToNaN() --> void");
		cl.def("assign", (class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPose2D, 3> & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3>::*)(const class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPose2D, 3> &)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::operator=, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::operator=(const class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPose2D, 3> &) --> class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPose2D, 3> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("assign", (struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose2D, 0> & (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose2D,0>::*)(const struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose2D, 0> &)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose2D, 0>::operator=, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose2D, 0>::operator=(const struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose2D, 0> &) --> struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose2D, 0> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::poses::CPoseOrPoint file:mrpt/poses/CPoseOrPoint.h line:123
		pybind11::class_<mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6>, std::shared_ptr<mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6>>, PyCallBack_mrpt_poses_CPoseOrPoint_mrpt_poses_CPose3D_6_t, mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D,1>> cl(M("mrpt::poses"), "CPoseOrPoint_mrpt_poses_CPose3D_6_t", "");
		cl.def(pybind11::init<PyCallBack_mrpt_poses_CPoseOrPoint_mrpt_poses_CPose3D_6_t const &>());
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_poses_CPoseOrPoint_mrpt_poses_CPose3D_6_t(); } ) );
		cl.def("derived", (class mrpt::poses::CPose3D & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::derived, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::derived() --> class mrpt::poses::CPose3D &", pybind11::return_value_policy::automatic);
		cl.def("x", (double & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::x, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::x() --> double &", pybind11::return_value_policy::automatic);
		cl.def("y", (double & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::y, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::y() --> double &", pybind11::return_value_policy::automatic);
		cl.def("x", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::x, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::x(const double) --> void", pybind11::arg("v"));
		cl.def("y", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::y, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::y(const double) --> void", pybind11::arg("v"));
		cl.def("x_incr", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::x_incr, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::x_incr(const double) --> void", pybind11::arg("v"));
		cl.def("y_incr", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::y_incr, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::y_incr(const double) --> void", pybind11::arg("v"));
		cl.def_static("is3DPoseOrPoint", (bool (*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::is3DPoseOrPoint, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::is3DPoseOrPoint() --> bool");
		cl.def("distance2DToSquare", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6>::*)(double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::distance2DToSquare, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::distance2DToSquare(double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"));
		cl.def("distance3DToSquare", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6>::*)(double, double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::distance3DToSquare, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::distance3DToSquare(double, double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"), pybind11::arg("az"));
		cl.def("distance2DTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6>::*)(double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::distance2DTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::distance2DTo(double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"));
		cl.def("distance3DTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6>::*)(double, double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::distance3DTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::distance3DTo(double, double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"), pybind11::arg("az"));
		cl.def("distanceTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6>::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::distanceTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::distanceTo(const struct mrpt::math::TPoint3D_<double> &) const --> double", pybind11::arg("b"));
		cl.def("norm", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6>::*)() const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::norm, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::norm() const --> double");
		cl.def("setToNaN", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::setToNaN, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::setToNaN() --> void");
		cl.def("assign", (class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPose3D, 6> & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6>::*)(const class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPose3D, 6> &)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::operator=, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::operator=(const class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPose3D, 6> &) --> class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPose3D, 6> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("z", (double & (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D,1>::*)()) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::z, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::z() --> double &", pybind11::return_value_policy::automatic);
		cl.def("z", (void (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D,1>::*)(const double)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::z, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::z(const double) --> void", pybind11::arg("v"));
		cl.def("z_incr", (void (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D,1>::*)(const double)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::z_incr, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::z_incr(const double) --> void", pybind11::arg("v"));
		cl.def("assign", (struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose3D, 1> & (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D,1>::*)(const struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose3D, 1> &)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::operator=, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::operator=(const struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose3D, 1> &) --> struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose3D, 1> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::poses::CPoseOrPoint file:mrpt/poses/CPoseOrPoint.h line:123
		pybind11::class_<mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3>, std::shared_ptr<mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3>>, PyCallBack_mrpt_poses_CPoseOrPoint_mrpt_poses_CPoint3D_3_t, mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D,1>> cl(M("mrpt::poses"), "CPoseOrPoint_mrpt_poses_CPoint3D_3_t", "");
		cl.def(pybind11::init<PyCallBack_mrpt_poses_CPoseOrPoint_mrpt_poses_CPoint3D_3_t const &>());
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_poses_CPoseOrPoint_mrpt_poses_CPoint3D_3_t(); } ) );
		cl.def("derived", (class mrpt::poses::CPoint3D & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::derived, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::derived() --> class mrpt::poses::CPoint3D &", pybind11::return_value_policy::automatic);
		cl.def("x", (double & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::x, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::x() --> double &", pybind11::return_value_policy::automatic);
		cl.def("y", (double & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::y, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::y() --> double &", pybind11::return_value_policy::automatic);
		cl.def("x", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::x, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::x(const double) --> void", pybind11::arg("v"));
		cl.def("y", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::y, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::y(const double) --> void", pybind11::arg("v"));
		cl.def("x_incr", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::x_incr, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::x_incr(const double) --> void", pybind11::arg("v"));
		cl.def("y_incr", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::y_incr, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::y_incr(const double) --> void", pybind11::arg("v"));
		cl.def_static("is3DPoseOrPoint", (bool (*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::is3DPoseOrPoint, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::is3DPoseOrPoint() --> bool");
		cl.def("distance2DToSquare", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3>::*)(double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::distance2DToSquare, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::distance2DToSquare(double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"));
		cl.def("distance3DToSquare", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3>::*)(double, double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::distance3DToSquare, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::distance3DToSquare(double, double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"), pybind11::arg("az"));
		cl.def("distance2DTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3>::*)(double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::distance2DTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::distance2DTo(double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"));
		cl.def("distance3DTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3>::*)(double, double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::distance3DTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::distance3DTo(double, double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"), pybind11::arg("az"));
		cl.def("distanceTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3>::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::distanceTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::distanceTo(const struct mrpt::math::TPoint3D_<double> &) const --> double", pybind11::arg("b"));
		cl.def("norm", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3>::*)() const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::norm, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::norm() const --> double");
		cl.def("setToNaN", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::setToNaN, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::setToNaN() --> void");
		cl.def("assign", (class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPoint3D, 3> & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3>::*)(const class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPoint3D, 3> &)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::operator=, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::operator=(const class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPoint3D, 3> &) --> class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPoint3D, 3> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("z", (double & (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D,1>::*)()) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D, 1>::z, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D, 1>::z() --> double &", pybind11::return_value_policy::automatic);
		cl.def("z", (void (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D,1>::*)(const double)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D, 1>::z, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D, 1>::z(const double) --> void", pybind11::arg("v"));
		cl.def("z_incr", (void (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D,1>::*)(const double)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D, 1>::z_incr, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D, 1>::z_incr(const double) --> void", pybind11::arg("v"));
		cl.def("assign", (struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPoint3D, 1> & (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D,1>::*)(const struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPoint3D, 1> &)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D, 1>::operator=, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPoint3D, 1>::operator=(const struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPoint3D, 1> &) --> struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPoint3D, 1> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
