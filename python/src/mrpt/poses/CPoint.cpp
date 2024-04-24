#include <ios>
#include <iterator>
#include <locale>
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
#include <mrpt/poses/CPoint.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoseOrPoint.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <variant>

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

// mrpt::poses::CPoint file:mrpt/poses/CPoint.h line:25
struct PyCallBack_mrpt_poses_CPoint_mrpt_poses_CPoint3D_3UL_t : public mrpt::poses::CPoint<mrpt::poses::CPoint3D,3UL> {
	using mrpt::poses::CPoint<mrpt::poses::CPoint3D,3UL>::CPoint;

	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint<mrpt::poses::CPoint3D,3UL> *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CPoint::asString();
	}
	void setToNaN() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint<mrpt::poses::CPoint3D,3UL> *>(this), "setToNaN");
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

// mrpt::poses::CPoint file:mrpt/poses/CPoint.h line:25
struct PyCallBack_mrpt_poses_CPoint_mrpt_poses_CPoint2D_2UL_t : public mrpt::poses::CPoint<mrpt::poses::CPoint2D,2UL> {
	using mrpt::poses::CPoint<mrpt::poses::CPoint2D,2UL>::CPoint;

	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint<mrpt::poses::CPoint2D,2UL> *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CPoint::asString();
	}
	void setToNaN() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint<mrpt::poses::CPoint2D,2UL> *>(this), "setToNaN");
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

// mrpt::poses::CPoint3D file:mrpt/poses/CPoint3D.h line:31
struct PyCallBack_mrpt_poses_CPoint3D : public mrpt::poses::CPoint3D {
	using mrpt::poses::CPoint3D::CPoint3D;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint3D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPoint3D::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint3D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPoint3D::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint3D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPoint3D::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint3D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPoint3D::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint3D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPoint3D::serializeFrom(a0, a1);
	}
	void setToNaN() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint3D *>(this), "setToNaN");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPoint3D::setToNaN();
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPoint3D *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CPoint::asString();
	}
};

void bind_mrpt_poses_CPoint(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::CPoint file:mrpt/poses/CPoint.h line:25
		pybind11::class_<mrpt::poses::CPoint<mrpt::poses::CPoint3D,3UL>, std::shared_ptr<mrpt::poses::CPoint<mrpt::poses::CPoint3D,3UL>>, PyCallBack_mrpt_poses_CPoint_mrpt_poses_CPoint3D_3UL_t, mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3UL>, mrpt::Stringifyable> cl(M("mrpt::poses"), "CPoint_mrpt_poses_CPoint3D_3UL_t", "");
		cl.def(pybind11::init<PyCallBack_mrpt_poses_CPoint_mrpt_poses_CPoint3D_3UL_t const &>());
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_poses_CPoint_mrpt_poses_CPoint3D_3UL_t(); } ) );
		cl.def("__imul__", (void (mrpt::poses::CPoint<mrpt::poses::CPoint3D,3UL>::*)(const double)) &mrpt::poses::CPoint<mrpt::poses::CPoint3D, 3>::operator*=, "C++: mrpt::poses::CPoint<mrpt::poses::CPoint3D, 3>::operator*=(const double) --> void", pybind11::arg("s"));
		cl.def("asString", (std::string (mrpt::poses::CPoint<mrpt::poses::CPoint3D,3UL>::*)() const) &mrpt::poses::CPoint<mrpt::poses::CPoint3D, 3>::asString, "C++: mrpt::poses::CPoint<mrpt::poses::CPoint3D, 3>::asString() const --> std::string");
		cl.def("fromString", (void (mrpt::poses::CPoint<mrpt::poses::CPoint3D,3UL>::*)(const std::string &)) &mrpt::poses::CPoint<mrpt::poses::CPoint3D, 3>::fromString, "C++: mrpt::poses::CPoint<mrpt::poses::CPoint3D, 3>::fromString(const std::string &) --> void", pybind11::arg("s"));
		cl.def("__getitem__", (double & (mrpt::poses::CPoint<mrpt::poses::CPoint3D,3UL>::*)(unsigned int)) &mrpt::poses::CPoint<mrpt::poses::CPoint3D, 3>::operator[], "C++: mrpt::poses::CPoint<mrpt::poses::CPoint3D, 3>::operator[](unsigned int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("assign", (class mrpt::poses::CPoint<class mrpt::poses::CPoint3D, 3> & (mrpt::poses::CPoint<mrpt::poses::CPoint3D,3UL>::*)(const class mrpt::poses::CPoint<class mrpt::poses::CPoint3D, 3> &)) &mrpt::poses::CPoint<mrpt::poses::CPoint3D, 3>::operator=, "C++: mrpt::poses::CPoint<mrpt::poses::CPoint3D, 3>::operator=(const class mrpt::poses::CPoint<class mrpt::poses::CPoint3D, 3> &) --> class mrpt::poses::CPoint<class mrpt::poses::CPoint3D, 3> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("derived", (class mrpt::poses::CPoint3D & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3UL>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::derived, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::derived() --> class mrpt::poses::CPoint3D &", pybind11::return_value_policy::automatic);
		cl.def("x", (double & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3UL>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::x, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::x() --> double &", pybind11::return_value_policy::automatic);
		cl.def("y", (double & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3UL>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::y, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::y() --> double &", pybind11::return_value_policy::automatic);
		cl.def("x", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3UL>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::x, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::x(const double) --> void", pybind11::arg("v"));
		cl.def("y", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3UL>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::y, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::y(const double) --> void", pybind11::arg("v"));
		cl.def("x_incr", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3UL>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::x_incr, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::x_incr(const double) --> void", pybind11::arg("v"));
		cl.def("y_incr", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3UL>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::y_incr, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::y_incr(const double) --> void", pybind11::arg("v"));
		cl.def_static("is3DPoseOrPoint", (bool (*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::is3DPoseOrPoint, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::is3DPoseOrPoint() --> bool");
		cl.def("distance2DToSquare", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3UL>::*)(double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::distance2DToSquare, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::distance2DToSquare(double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"));
		cl.def("distance3DToSquare", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3UL>::*)(double, double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::distance3DToSquare, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::distance3DToSquare(double, double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"), pybind11::arg("az"));
		cl.def("distance2DTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3UL>::*)(double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::distance2DTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::distance2DTo(double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"));
		cl.def("distance3DTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3UL>::*)(double, double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::distance3DTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::distance3DTo(double, double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"), pybind11::arg("az"));
		cl.def("distanceTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3UL>::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::distanceTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::distanceTo(const struct mrpt::math::TPoint3D_<double> &) const --> double", pybind11::arg("b"));
		cl.def("norm", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3UL>::*)() const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::norm, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::norm() const --> double");
		cl.def("asVectorVal", (class mrpt::math::CMatrixFixed<double, 3, 1> (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3UL>::*)() const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::asVectorVal, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::asVectorVal() const --> class mrpt::math::CMatrixFixed<double, 3, 1>");
		cl.def("setToNaN", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3UL>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::setToNaN, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::setToNaN() --> void");
		cl.def("assign", (class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPoint3D, 3> & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D,3UL>::*)(const class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPoint3D, 3> &)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::operator=, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint3D, 3>::operator=(const class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPoint3D, 3> &) --> class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPoint3D, 3> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("asString", (std::string (mrpt::Stringifyable::*)() const) &mrpt::Stringifyable::asString, "Returns a human-friendly textual description of the object. For classes\n with a large/complex internal state, only a summary should be returned\n instead of the exhaustive enumeration of all data members.\n\nC++: mrpt::Stringifyable::asString() const --> std::string");
		cl.def("assign", (class mrpt::Stringifyable & (mrpt::Stringifyable::*)(const class mrpt::Stringifyable &)) &mrpt::Stringifyable::operator=, "C++: mrpt::Stringifyable::operator=(const class mrpt::Stringifyable &) --> class mrpt::Stringifyable &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::poses::CPoint file:mrpt/poses/CPoint.h line:25
		pybind11::class_<mrpt::poses::CPoint<mrpt::poses::CPoint2D,2UL>, std::shared_ptr<mrpt::poses::CPoint<mrpt::poses::CPoint2D,2UL>>, PyCallBack_mrpt_poses_CPoint_mrpt_poses_CPoint2D_2UL_t, mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D,2UL>, mrpt::Stringifyable> cl(M("mrpt::poses"), "CPoint_mrpt_poses_CPoint2D_2UL_t", "");
		cl.def(pybind11::init<PyCallBack_mrpt_poses_CPoint_mrpt_poses_CPoint2D_2UL_t const &>());
		cl.def( pybind11::init( [](){ return new PyCallBack_mrpt_poses_CPoint_mrpt_poses_CPoint2D_2UL_t(); } ) );
		cl.def("__imul__", (void (mrpt::poses::CPoint<mrpt::poses::CPoint2D,2UL>::*)(const double)) &mrpt::poses::CPoint<mrpt::poses::CPoint2D, 2>::operator*=, "C++: mrpt::poses::CPoint<mrpt::poses::CPoint2D, 2>::operator*=(const double) --> void", pybind11::arg("s"));
		cl.def("asString", (std::string (mrpt::poses::CPoint<mrpt::poses::CPoint2D,2UL>::*)() const) &mrpt::poses::CPoint<mrpt::poses::CPoint2D, 2>::asString, "C++: mrpt::poses::CPoint<mrpt::poses::CPoint2D, 2>::asString() const --> std::string");
		cl.def("fromString", (void (mrpt::poses::CPoint<mrpt::poses::CPoint2D,2UL>::*)(const std::string &)) &mrpt::poses::CPoint<mrpt::poses::CPoint2D, 2>::fromString, "C++: mrpt::poses::CPoint<mrpt::poses::CPoint2D, 2>::fromString(const std::string &) --> void", pybind11::arg("s"));
		cl.def("__getitem__", (double & (mrpt::poses::CPoint<mrpt::poses::CPoint2D,2UL>::*)(unsigned int)) &mrpt::poses::CPoint<mrpt::poses::CPoint2D, 2>::operator[], "C++: mrpt::poses::CPoint<mrpt::poses::CPoint2D, 2>::operator[](unsigned int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("assign", (class mrpt::poses::CPoint<class mrpt::poses::CPoint2D, 2> & (mrpt::poses::CPoint<mrpt::poses::CPoint2D,2UL>::*)(const class mrpt::poses::CPoint<class mrpt::poses::CPoint2D, 2> &)) &mrpt::poses::CPoint<mrpt::poses::CPoint2D, 2>::operator=, "C++: mrpt::poses::CPoint<mrpt::poses::CPoint2D, 2>::operator=(const class mrpt::poses::CPoint<class mrpt::poses::CPoint2D, 2> &) --> class mrpt::poses::CPoint<class mrpt::poses::CPoint2D, 2> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("derived", (class mrpt::poses::CPoint2D & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D,2UL>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::derived, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::derived() --> class mrpt::poses::CPoint2D &", pybind11::return_value_policy::automatic);
		cl.def("x", (double & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D,2UL>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::x, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::x() --> double &", pybind11::return_value_policy::automatic);
		cl.def("y", (double & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D,2UL>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::y, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::y() --> double &", pybind11::return_value_policy::automatic);
		cl.def("x", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D,2UL>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::x, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::x(const double) --> void", pybind11::arg("v"));
		cl.def("y", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D,2UL>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::y, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::y(const double) --> void", pybind11::arg("v"));
		cl.def("x_incr", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D,2UL>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::x_incr, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::x_incr(const double) --> void", pybind11::arg("v"));
		cl.def("y_incr", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D,2UL>::*)(const double)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::y_incr, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::y_incr(const double) --> void", pybind11::arg("v"));
		cl.def_static("is3DPoseOrPoint", (bool (*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::is3DPoseOrPoint, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::is3DPoseOrPoint() --> bool");
		cl.def("distance2DToSquare", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D,2UL>::*)(double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::distance2DToSquare, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::distance2DToSquare(double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"));
		cl.def("distance3DToSquare", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D,2UL>::*)(double, double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::distance3DToSquare, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::distance3DToSquare(double, double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"), pybind11::arg("az"));
		cl.def("distance2DTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D,2UL>::*)(double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::distance2DTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::distance2DTo(double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"));
		cl.def("distance3DTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D,2UL>::*)(double, double, double) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::distance3DTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::distance3DTo(double, double, double) const --> double", pybind11::arg("ax"), pybind11::arg("ay"), pybind11::arg("az"));
		cl.def("distanceTo", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D,2UL>::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::distanceTo, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::distanceTo(const struct mrpt::math::TPoint3D_<double> &) const --> double", pybind11::arg("b"));
		cl.def("norm", (double (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D,2UL>::*)() const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::norm, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::norm() const --> double");
		cl.def("asVectorVal", (class mrpt::math::CMatrixFixed<double, 2, 1> (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D,2UL>::*)() const) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::asVectorVal, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::asVectorVal() const --> class mrpt::math::CMatrixFixed<double, 2, 1>");
		cl.def("setToNaN", (void (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D,2UL>::*)()) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::setToNaN, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::setToNaN() --> void");
		cl.def("assign", (class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPoint2D, 2> & (mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D,2UL>::*)(const class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPoint2D, 2> &)) &mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::operator=, "C++: mrpt::poses::CPoseOrPoint<mrpt::poses::CPoint2D, 2>::operator=(const class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPoint2D, 2> &) --> class mrpt::poses::CPoseOrPoint<class mrpt::poses::CPoint2D, 2> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("asString", (std::string (mrpt::Stringifyable::*)() const) &mrpt::Stringifyable::asString, "Returns a human-friendly textual description of the object. For classes\n with a large/complex internal state, only a summary should be returned\n instead of the exhaustive enumeration of all data members.\n\nC++: mrpt::Stringifyable::asString() const --> std::string");
		cl.def("assign", (class mrpt::Stringifyable & (mrpt::Stringifyable::*)(const class mrpt::Stringifyable &)) &mrpt::Stringifyable::operator=, "C++: mrpt::Stringifyable::operator=(const class mrpt::Stringifyable &) --> class mrpt::Stringifyable &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::poses::CPoint3D file:mrpt/poses/CPoint3D.h line:31
		pybind11::class_<mrpt::poses::CPoint3D, std::shared_ptr<mrpt::poses::CPoint3D>, PyCallBack_mrpt_poses_CPoint3D, mrpt::poses::CPoint<mrpt::poses::CPoint3D,3UL>, mrpt::serialization::CSerializable> cl(M("mrpt::poses"), "CPoint3D", "A class used to store a 3D point.\n\n  For a complete description of Points/Poses, see mrpt::poses::CPoseOrPoint,\n or refer\n    to the 2D/3D Geometry\n tutorial in the wiki.\n\n  \n   \n  \n\n \n\n \n CPoseOrPoint,CPose, CPoint");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPoint3D(); }, [](){ return new PyCallBack_mrpt_poses_CPoint3D(); } ) );
		cl.def( pybind11::init<const double, const double, const double>(), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z") );

		cl.def( pybind11::init<const class mrpt::math::CMatrixFixed<double, 3, 1> &>(), pybind11::arg("xyz") );

		cl.def( pybind11::init<const class mrpt::poses::CPoint2D &>(), pybind11::arg("p") );

		cl.def( pybind11::init<const class mrpt::poses::CPose3D &>(), pybind11::arg("p") );

		cl.def( pybind11::init<const class mrpt::poses::CPose2D &>(), pybind11::arg("p") );

		cl.def( pybind11::init<const struct mrpt::math::TPoint3D_<double> &>(), pybind11::arg("p") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPoint3D const &o){ return new PyCallBack_mrpt_poses_CPoint3D(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPoint3D const &o){ return new mrpt::poses::CPoint3D(o); } ) );
		cl.def_readwrite("m_coords", &mrpt::poses::CPoint3D::m_coords);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPoint3D::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPoint3D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPoint3D::*)() const) &mrpt::poses::CPoint3D::GetRuntimeClass, "C++: mrpt::poses::CPoint3D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPoint3D::*)() const) &mrpt::poses::CPoint3D::clone, "C++: mrpt::poses::CPoint3D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPoint3D::CreateObject, "C++: mrpt::poses::CPoint3D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("asTPoint", (struct mrpt::math::TPoint3D_<double> (mrpt::poses::CPoint3D::*)() const) &mrpt::poses::CPoint3D::asTPoint, "C++: mrpt::poses::CPoint3D::asTPoint() const --> struct mrpt::math::TPoint3D_<double>");
		cl.def("__sub__", (class mrpt::poses::CPoint3D (mrpt::poses::CPoint3D::*)(const class mrpt::poses::CPose3D &) const) &mrpt::poses::CPoint3D::operator-, "Returns this point as seen from \"b\", i.e. result = this - b \n\nC++: mrpt::poses::CPoint3D::operator-(const class mrpt::poses::CPose3D &) const --> class mrpt::poses::CPoint3D", pybind11::arg("b"));
		cl.def("__sub__", (class mrpt::poses::CPoint3D (mrpt::poses::CPoint3D::*)(const class mrpt::poses::CPoint3D &) const) &mrpt::poses::CPoint3D::operator-, "Returns this point minus point \"b\", i.e. result = this - b \n\nC++: mrpt::poses::CPoint3D::operator-(const class mrpt::poses::CPoint3D &) const --> class mrpt::poses::CPoint3D", pybind11::arg("b"));
		cl.def("__add__", (class mrpt::poses::CPoint3D (mrpt::poses::CPoint3D::*)(const class mrpt::poses::CPoint3D &) const) &mrpt::poses::CPoint3D::operator+, "Returns this point plus point \"b\", i.e. result = this + b \n\nC++: mrpt::poses::CPoint3D::operator+(const class mrpt::poses::CPoint3D &) const --> class mrpt::poses::CPoint3D", pybind11::arg("b"));
		cl.def("__add__", (class mrpt::poses::CPose3D (mrpt::poses::CPoint3D::*)(const class mrpt::poses::CPose3D &) const) &mrpt::poses::CPoint3D::operator+, "Returns this point plus pose \"b\", i.e. result = this + b  \n\nC++: mrpt::poses::CPoint3D::operator+(const class mrpt::poses::CPose3D &) const --> class mrpt::poses::CPose3D", pybind11::arg("b"));
		cl.def("asVector", (void (mrpt::poses::CPoint3D::*)(class mrpt::math::CMatrixFixed<double, 3, 1> &) const) &mrpt::poses::CPoint3D::asVector, "Return the pose or point as a 3x1 vector [x y z]' \n\nC++: mrpt::poses::CPoint3D::asVector(class mrpt::math::CMatrixFixed<double, 3, 1> &) const --> void", pybind11::arg("v"));
		cl.def_static("is_3D", (bool (*)()) &mrpt::poses::CPoint3D::is_3D, "C++: mrpt::poses::CPoint3D::is_3D() --> bool");
		cl.def_static("is_PDF", (bool (*)()) &mrpt::poses::CPoint3D::is_PDF, "C++: mrpt::poses::CPoint3D::is_PDF() --> bool");
		cl.def_static("size", (unsigned long (*)()) &mrpt::poses::CPoint3D::size, "C++: mrpt::poses::CPoint3D::size() --> unsigned long");
		cl.def_static("empty", (bool (*)()) &mrpt::poses::CPoint3D::empty, "C++: mrpt::poses::CPoint3D::empty() --> bool");
		cl.def_static("max_size", (unsigned long (*)()) &mrpt::poses::CPoint3D::max_size, "C++: mrpt::poses::CPoint3D::max_size() --> unsigned long");
		cl.def_static("resize", (void (*)(const unsigned long)) &mrpt::poses::CPoint3D::resize, "C++: mrpt::poses::CPoint3D::resize(const unsigned long) --> void", pybind11::arg("n"));
		cl.def("setToNaN", (void (mrpt::poses::CPoint3D::*)()) &mrpt::poses::CPoint3D::setToNaN, "@} \n\nC++: mrpt::poses::CPoint3D::setToNaN() --> void");
		cl.def("assign", (class mrpt::poses::CPoint3D & (mrpt::poses::CPoint3D::*)(const class mrpt::poses::CPoint3D &)) &mrpt::poses::CPoint3D::operator=, "C++: mrpt::poses::CPoint3D::operator=(const class mrpt::poses::CPoint3D &) --> class mrpt::poses::CPoint3D &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::poses::CPoint3D const &o) -> std::string { std::ostringstream s; using namespace mrpt::poses; s << o; return s.str(); } );
	}
}
