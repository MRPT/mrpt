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
#include <mrpt/poses/CPose.h>
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
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

// mrpt::poses::CPose file:mrpt/poses/CPose.h line:24
struct PyCallBack_mrpt_poses_CPose_mrpt_poses_CPose2D_3UL_t : public mrpt::poses::CPose<mrpt::poses::CPose2D,3UL> {
	using mrpt::poses::CPose<mrpt::poses::CPose2D,3UL>::CPose;

	void setToNaN() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose<mrpt::poses::CPose2D,3UL> *>(this), "setToNaN");
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

// mrpt::poses::CPose file:mrpt/poses/CPose.h line:24
struct PyCallBack_mrpt_poses_CPose_mrpt_poses_CPose3D_6UL_t : public mrpt::poses::CPose<mrpt::poses::CPose3D,6UL> {
	using mrpt::poses::CPose<mrpt::poses::CPose3D,6UL>::CPose;

	void setToNaN() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose<mrpt::poses::CPose3D,6UL> *>(this), "setToNaN");
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

// mrpt::poses::CPose file:mrpt/poses/CPose.h line:24
struct PyCallBack_mrpt_poses_CPose_mrpt_poses_CPose3DQuat_7UL_t : public mrpt::poses::CPose<mrpt::poses::CPose3DQuat,7UL> {
	using mrpt::poses::CPose<mrpt::poses::CPose3DQuat,7UL>::CPose;

	void setToNaN() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose<mrpt::poses::CPose3DQuat,7UL> *>(this), "setToNaN");
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

void bind_mrpt_poses_CPose(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::CPose file:mrpt/poses/CPose.h line:24
		pybind11::class_<mrpt::poses::CPose<mrpt::poses::CPose2D,3UL>, std::shared_ptr<mrpt::poses::CPose<mrpt::poses::CPose2D,3UL>>, PyCallBack_mrpt_poses_CPose_mrpt_poses_CPose2D_3UL_t, mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3UL>> cl(M("mrpt::poses"), "CPose_mrpt_poses_CPose2D_3UL_t", "");
		cl.def(pybind11::init<PyCallBack_mrpt_poses_CPose_mrpt_poses_CPose2D_3UL_t const &>());
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_poses_CPose_mrpt_poses_CPose2D_3UL_t(); } ) );
		cl.def("assign", (class mrpt::poses::CPose<class mrpt::poses::CPose2D, 3> & (mrpt::poses::CPose<mrpt::poses::CPose2D,3UL>::*)(const class mrpt::poses::CPose<class mrpt::poses::CPose2D, 3> &)) &mrpt::poses::CPose<mrpt::poses::CPose2D, 3>::operator=, "C++: mrpt::poses::CPose<mrpt::poses::CPose2D, 3>::operator=(const class mrpt::poses::CPose<class mrpt::poses::CPose2D, 3> &) --> class mrpt::poses::CPose<class mrpt::poses::CPose2D, 3> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("derived", (class mrpt::poses::CPose2D & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3UL>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::derived, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::derived() --> class mrpt::poses::CPose2D &", pybind11::return_value_policy::automatic);
		cl.def("x", (double & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3UL>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::x, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::x() --> double &", pybind11::return_value_policy::automatic);
		cl.def("y", (double & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3UL>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::y, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::y() --> double &", pybind11::return_value_policy::automatic);
		cl.def("x", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3UL>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::x, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::x(const double) --> void", pybind11::arg("v"));
		cl.def("y", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3UL>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::y, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::y(const double) --> void", pybind11::arg("v"));
		cl.def("x_incr", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3UL>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::x_incr, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::x_incr(const double) --> void", pybind11::arg("v"));
		cl.def("y_incr", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3UL>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::y_incr, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::y_incr(const double) --> void", pybind11::arg("v"));
		cl.def_static("is3DPoseOrPoint", (bool (*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::is3DPoseOrPoint, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::is3DPoseOrPoint() --> bool");
		cl.def("distance2DToSquare", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3UL>::*)(double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::distance2DToSquare, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::distance2DToSquare(double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"));
		cl.def("distance3DToSquare", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3UL>::*)(double, double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::distance3DToSquare, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::distance3DToSquare(double, double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"), pybind11::arg("az"));
		cl.def("distance2DTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3UL>::*)(double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::distance2DTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::distance2DTo(double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"));
		cl.def("distance3DTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3UL>::*)(double, double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::distance3DTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::distance3DTo(double, double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"), pybind11::arg("az"));
		cl.def("distanceTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3UL>::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::distanceTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::distanceTo(const struct mrpt::math::TPoint3D_<double> &) const --> double", pybind11::arg("b"));
		cl.def("norm", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3UL>::*)() const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::norm, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::norm() const --> double");
		cl.def("asVectorVal", (class mrpt::math::CMatrixFixed<double, 3, 1> (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3UL>::*)() const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::asVectorVal, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::asVectorVal() const --> class mrpt::math::CMatrixFixed<double, 3, 1>");
		cl.def("setToNaN", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3UL>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::setToNaN, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::setToNaN() --> void");
		cl.def("assign", (class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPose2D, 3> & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D,3UL>::*)(const class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPose2D, 3> &)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::operator=, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose2D, 3>::operator=(const class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPose2D, 3> &) --> class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPose2D, 3> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("assign", (struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose2D, 0> & (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose2D,0>::*)(const struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose2D, 0> &)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose2D, 0>::operator=, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose2D, 0>::operator=(const struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose2D, 0> &) --> struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose2D, 0> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::poses::CPose file:mrpt/poses/CPose.h line:24
		pybind11::class_<mrpt::poses::CPose<mrpt::poses::CPose3D,6UL>, std::shared_ptr<mrpt::poses::CPose<mrpt::poses::CPose3D,6UL>>, PyCallBack_mrpt_poses_CPose_mrpt_poses_CPose3D_6UL_t, mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6UL>> cl(M("mrpt::poses"), "CPose_mrpt_poses_CPose3D_6UL_t", "");
		cl.def(pybind11::init<PyCallBack_mrpt_poses_CPose_mrpt_poses_CPose3D_6UL_t const &>());
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_poses_CPose_mrpt_poses_CPose3D_6UL_t(); } ) );
		cl.def("assign", (class mrpt::poses::CPose<class mrpt::poses::CPose3D, 6> & (mrpt::poses::CPose<mrpt::poses::CPose3D,6UL>::*)(const class mrpt::poses::CPose<class mrpt::poses::CPose3D, 6> &)) &mrpt::poses::CPose<mrpt::poses::CPose3D, 6>::operator=, "C++: mrpt::poses::CPose<mrpt::poses::CPose3D, 6>::operator=(const class mrpt::poses::CPose<class mrpt::poses::CPose3D, 6> &) --> class mrpt::poses::CPose<class mrpt::poses::CPose3D, 6> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("getHomogeneousMatrixVal", (class mrpt::math::CMatrixFixed<double, 4, 4> (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6UL>::*)() const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::getHomogeneousMatrixVal<mrpt::math::CMatrixFixed<double, 4, 4>>, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::getHomogeneousMatrixVal() const --> class mrpt::math::CMatrixFixed<double, 4, 4>");
		cl.def("derived", (class mrpt::poses::CPose3D & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6UL>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::derived, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::derived() --> class mrpt::poses::CPose3D &", pybind11::return_value_policy::automatic);
		cl.def("x", (double & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6UL>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::x, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::x() --> double &", pybind11::return_value_policy::automatic);
		cl.def("y", (double & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6UL>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::y, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::y() --> double &", pybind11::return_value_policy::automatic);
		cl.def("x", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6UL>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::x, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::x(const double) --> void", pybind11::arg("v"));
		cl.def("y", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6UL>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::y, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::y(const double) --> void", pybind11::arg("v"));
		cl.def("x_incr", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6UL>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::x_incr, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::x_incr(const double) --> void", pybind11::arg("v"));
		cl.def("y_incr", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6UL>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::y_incr, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::y_incr(const double) --> void", pybind11::arg("v"));
		cl.def_static("is3DPoseOrPoint", (bool (*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::is3DPoseOrPoint, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::is3DPoseOrPoint() --> bool");
		cl.def("distance2DToSquare", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6UL>::*)(double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::distance2DToSquare, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::distance2DToSquare(double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"));
		cl.def("distance3DToSquare", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6UL>::*)(double, double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::distance3DToSquare, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::distance3DToSquare(double, double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"), pybind11::arg("az"));
		cl.def("distance2DTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6UL>::*)(double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::distance2DTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::distance2DTo(double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"));
		cl.def("distance3DTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6UL>::*)(double, double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::distance3DTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::distance3DTo(double, double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"), pybind11::arg("az"));
		cl.def("distanceTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6UL>::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::distanceTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::distanceTo(const struct mrpt::math::TPoint3D_<double> &) const --> double", pybind11::arg("b"));
		cl.def("norm", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6UL>::*)() const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::norm, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::norm() const --> double");
		cl.def("asVectorVal", (class mrpt::math::CMatrixFixed<double, 6, 1> (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6UL>::*)() const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::asVectorVal, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::asVectorVal() const --> class mrpt::math::CMatrixFixed<double, 6, 1>");
		cl.def("setToNaN", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6UL>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::setToNaN, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::setToNaN() --> void");
		cl.def("assign", (class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPose3D, 6> & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D,6UL>::*)(const class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPose3D, 6> &)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::operator=, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3D, 6>::operator=(const class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPose3D, 6> &) --> class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPose3D, 6> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("z", (double & (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D,1>::*)()) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::z, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::z() --> double &", pybind11::return_value_policy::automatic);
		cl.def("z", (void (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D,1>::*)(const double)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::z, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::z(const double) --> void", pybind11::arg("v"));
		cl.def("z_incr", (void (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D,1>::*)(const double)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::z_incr, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::z_incr(const double) --> void", pybind11::arg("v"));
		cl.def("assign", (struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose3D, 1> & (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D,1>::*)(const struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose3D, 1> &)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::operator=, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3D, 1>::operator=(const struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose3D, 1> &) --> struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose3D, 1> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::poses::CPose file:mrpt/poses/CPose.h line:24
		pybind11::class_<mrpt::poses::CPose<mrpt::poses::CPose3DQuat,7UL>, std::shared_ptr<mrpt::poses::CPose<mrpt::poses::CPose3DQuat,7UL>>, PyCallBack_mrpt_poses_CPose_mrpt_poses_CPose3DQuat_7UL_t, mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat,7UL>> cl(M("mrpt::poses"), "CPose_mrpt_poses_CPose3DQuat_7UL_t", "");
		cl.def(pybind11::init<PyCallBack_mrpt_poses_CPose_mrpt_poses_CPose3DQuat_7UL_t const &>());
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_poses_CPose_mrpt_poses_CPose3DQuat_7UL_t(); } ) );
		cl.def("assign", (class mrpt::poses::CPose<class mrpt::poses::CPose3DQuat, 7> & (mrpt::poses::CPose<mrpt::poses::CPose3DQuat,7UL>::*)(const class mrpt::poses::CPose<class mrpt::poses::CPose3DQuat, 7> &)) &mrpt::poses::CPose<mrpt::poses::CPose3DQuat, 7>::operator=, "C++: mrpt::poses::CPose<mrpt::poses::CPose3DQuat, 7>::operator=(const class mrpt::poses::CPose<class mrpt::poses::CPose3DQuat, 7> &) --> class mrpt::poses::CPose<class mrpt::poses::CPose3DQuat, 7> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("derived", (class mrpt::poses::CPose3DQuat & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat,7UL>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::derived, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::derived() --> class mrpt::poses::CPose3DQuat &", pybind11::return_value_policy::automatic);
		cl.def("x", (double & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat,7UL>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::x, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::x() --> double &", pybind11::return_value_policy::automatic);
		cl.def("y", (double & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat,7UL>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::y, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::y() --> double &", pybind11::return_value_policy::automatic);
		cl.def("x", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat,7UL>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::x, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::x(const double) --> void", pybind11::arg("v"));
		cl.def("y", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat,7UL>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::y, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::y(const double) --> void", pybind11::arg("v"));
		cl.def("x_incr", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat,7UL>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::x_incr, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::x_incr(const double) --> void", pybind11::arg("v"));
		cl.def("y_incr", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat,7UL>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::y_incr, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::y_incr(const double) --> void", pybind11::arg("v"));
		cl.def_static("is3DPoseOrPoint", (bool (*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::is3DPoseOrPoint, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::is3DPoseOrPoint() --> bool");
		cl.def("distance2DToSquare", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat,7UL>::*)(double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::distance2DToSquare, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::distance2DToSquare(double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"));
		cl.def("distance3DToSquare", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat,7UL>::*)(double, double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::distance3DToSquare, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::distance3DToSquare(double, double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"), pybind11::arg("az"));
		cl.def("distance2DTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat,7UL>::*)(double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::distance2DTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::distance2DTo(double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"));
		cl.def("distance3DTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat,7UL>::*)(double, double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::distance3DTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::distance3DTo(double, double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"), pybind11::arg("az"));
		cl.def("distanceTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat,7UL>::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::distanceTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::distanceTo(const struct mrpt::math::TPoint3D_<double> &) const --> double", pybind11::arg("b"));
		cl.def("norm", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat,7UL>::*)() const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::norm, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::norm() const --> double");
		cl.def("asVectorVal", (class mrpt::math::CMatrixFixed<double, 7, 1> (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat,7UL>::*)() const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::asVectorVal, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::asVectorVal() const --> class mrpt::math::CMatrixFixed<double, 7, 1>");
		cl.def("setToNaN", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat,7UL>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::setToNaN, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::setToNaN() --> void");
		cl.def("assign", (class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPose3DQuat, 7> & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat,7UL>::*)(const class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPose3DQuat, 7> &)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::operator=, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPose3DQuat, 7>::operator=(const class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPose3DQuat, 7> &) --> class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPose3DQuat, 7> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("z", (double & (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat,1>::*)()) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat, 1>::z, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat, 1>::z() --> double &", pybind11::return_value_policy::automatic);
		cl.def("z", (void (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat,1>::*)(const double)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat, 1>::z, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat, 1>::z(const double) --> void", pybind11::arg("v"));
		cl.def("z_incr", (void (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat,1>::*)(const double)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat, 1>::z_incr, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat, 1>::z_incr(const double) --> void", pybind11::arg("v"));
		cl.def("assign", (struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose3DQuat, 1> & (mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat,1>::*)(const struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose3DQuat, 1> &)) &mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat, 1>::operator=, "C++: mrpt::poses::detail::pose_point_impl<mrpt::poses::CPose3DQuat, 1>::operator=(const struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose3DQuat, 1> &) --> struct mrpt::poses::detail::pose_point_impl<class mrpt::poses::CPose3DQuat, 1> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
