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
#include <mrpt/math/TPose3DQuat.h>
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
#include <mrpt/serialization/CMessage.h>
#include <mrpt/serialization/CSerializable.h>
#include <mrpt/typemeta/static_string.h>
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
#include <string>
#include <variant>
#include <vector>

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

// mrpt::poses::CPose3DQuat file:mrpt/poses/CPose3DQuat.h line:46
struct PyCallBack_mrpt_poses_CPose3DQuat : public mrpt::poses::CPose3DQuat {
	using mrpt::poses::CPose3DQuat::CPose3DQuat;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuat *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPose3DQuat::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuat *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPose3DQuat::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuat *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPose3DQuat::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuat *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DQuat::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuat *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DQuat::serializeFrom(a0, a1);
	}
	void operator*=(const double a0) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuat *>(this), "__imul__");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DQuat::operator*=(a0);
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuat *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CPose3DQuat::asString();
	}
	void setToNaN() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3DQuat *>(this), "setToNaN");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3DQuat::setToNaN();
	}
};

void bind_mrpt_poses_CPoint3D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::CPoint3D file:mrpt/poses/CPoint3D.h line:31
		pybind11::class_<mrpt::poses::CPoint3D, std::shared_ptr<mrpt::poses::CPoint3D>, PyCallBack_mrpt_poses_CPoint3D, mrpt::poses::CPoint<mrpt::poses::CPoint3D,3>, mrpt::serialization::CSerializable> cl(M("mrpt::poses"), "CPoint3D", "A class used to store a 3D point.\n\n  For a complete description of Points/Poses, see mrpt::poses::CPoseOrPoint,\n or refer\n    to the 2D/3D Geometry\n tutorial in the wiki.\n\n  \n   \n  \n\n \n\n \n CPoseOrPoint,CPose, CPoint");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPoint3D(); }, [](){ return new PyCallBack_mrpt_poses_CPoint3D(); } ), "doc");
		cl.def( pybind11::init( [](const double & a0){ return new mrpt::poses::CPoint3D(a0); }, [](const double & a0){ return new PyCallBack_mrpt_poses_CPoint3D(a0); } ), "doc");
		cl.def( pybind11::init( [](const double & a0, const double & a1){ return new mrpt::poses::CPoint3D(a0, a1); }, [](const double & a0, const double & a1){ return new PyCallBack_mrpt_poses_CPoint3D(a0, a1); } ), "doc");
		cl.def( pybind11::init<const double, const double, const double>(), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z") );

		cl.def( pybind11::init<const class mrpt::poses::CPoint2D &>(), pybind11::arg("p") );

		cl.def( pybind11::init<const class mrpt::poses::CPose3D &>(), pybind11::arg("p") );

		cl.def( pybind11::init<const class mrpt::poses::CPose2D &>(), pybind11::arg("p") );

		cl.def( pybind11::init<const struct mrpt::math::TPoint3D_<double> &>(), pybind11::arg("p") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPoint3D const &o){ return new PyCallBack_mrpt_poses_CPoint3D(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPoint3D const &o){ return new mrpt::poses::CPoint3D(o); } ) );
		cl.def_readwrite("m_coords", &mrpt::poses::CPoint3D::m_coords);
		cl.def_static("getClassName", (class mrpt::typemeta::string_literal<21> (*)()) &mrpt::poses::CPoint3D::getClassName, "C++: mrpt::poses::CPoint3D::getClassName() --> class mrpt::typemeta::string_literal<21>");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPoint3D::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPoint3D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPoint3D::*)() const) &mrpt::poses::CPoint3D::GetRuntimeClass, "C++: mrpt::poses::CPoint3D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPoint3D::*)() const) &mrpt::poses::CPoint3D::clone, "C++: mrpt::poses::CPoint3D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPoint3D::CreateObject, "C++: mrpt::poses::CPoint3D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("asTPoint", (struct mrpt::math::TPoint3D_<double> (mrpt::poses::CPoint3D::*)() const) &mrpt::poses::CPoint3D::asTPoint, "C++: mrpt::poses::CPoint3D::asTPoint() const --> struct mrpt::math::TPoint3D_<double>");
		cl.def("__sub__", (class mrpt::poses::CPoint3D (mrpt::poses::CPoint3D::*)(const class mrpt::poses::CPose3D &) const) &mrpt::poses::CPoint3D::operator-, "Returns this point as seen from \"b\", i.e. result = this - b \n\nC++: mrpt::poses::CPoint3D::operator-(const class mrpt::poses::CPose3D &) const --> class mrpt::poses::CPoint3D", pybind11::arg("b"));
		cl.def("__sub__", (class mrpt::poses::CPoint3D (mrpt::poses::CPoint3D::*)(const class mrpt::poses::CPoint3D &) const) &mrpt::poses::CPoint3D::operator-, "Returns this point minus point \"b\", i.e. result = this - b \n\nC++: mrpt::poses::CPoint3D::operator-(const class mrpt::poses::CPoint3D &) const --> class mrpt::poses::CPoint3D", pybind11::arg("b"));
		cl.def("__add__", (class mrpt::poses::CPoint3D (mrpt::poses::CPoint3D::*)(const class mrpt::poses::CPoint3D &) const) &mrpt::poses::CPoint3D::operator+, "Returns this point plus point \"b\", i.e. result = this + b \n\nC++: mrpt::poses::CPoint3D::operator+(const class mrpt::poses::CPoint3D &) const --> class mrpt::poses::CPoint3D", pybind11::arg("b"));
		cl.def("__add__", (class mrpt::poses::CPose3D (mrpt::poses::CPoint3D::*)(const class mrpt::poses::CPose3D &) const) &mrpt::poses::CPoint3D::operator+, "Returns this point plus pose \"b\", i.e. result = this + b  \n\nC++: mrpt::poses::CPoint3D::operator+(const class mrpt::poses::CPose3D &) const --> class mrpt::poses::CPose3D", pybind11::arg("b"));
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
	{ // mrpt::poses::CPose3DQuat file:mrpt/poses/CPose3DQuat.h line:46
		pybind11::class_<mrpt::poses::CPose3DQuat, std::shared_ptr<mrpt::poses::CPose3DQuat>, PyCallBack_mrpt_poses_CPose3DQuat, mrpt::poses::CPose<mrpt::poses::CPose3DQuat,7>, mrpt::serialization::CSerializable, mrpt::Stringifyable> cl(M("mrpt::poses"), "CPose3DQuat", "A class used to store a 3D pose as a translation (x,y,z) and a quaternion\n (qr,qx,qy,qz).\n\n  For a complete description of Points/Poses, see mrpt::poses::CPoseOrPoint,\n or refer\n    to the  2D/3D Geometry\n tutorial in the wiki.\n\n  To access the translation use x(), y() and z(). To access the rotation, use\n CPose3DQuat::quat().\n\n  This class also behaves like a STL container, since it has begin(), end(),\n iterators, and can be accessed with the [] operator\n   with indices running from 0 to 6 to access the  [x y z qr qx qy qz] as if\n they were a vector. Thus, a CPose3DQuat can be used\n   as a 7-vector anywhere the MRPT math functions expect any kind of vector.\n\n  This class and CPose3D are very similar, and they can be converted to the\n each other automatically via transformation constructors.\n\n \n CPose3D (for a class based on a 4x4 matrix instead of a quaternion),\n mrpt::math::TPose3DQuat, mrpt::poses::CPose3DQuatPDF for a probabilistic\n version of this class,  mrpt::math::CQuaternion, CPoseOrPoint\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPose3DQuat(); }, [](){ return new PyCallBack_mrpt_poses_CPose3DQuat(); } ) );
		cl.def( pybind11::init<enum mrpt::math::TConstructorFlags_Quaternions>(), pybind11::arg("") );

		cl.def( pybind11::init<enum mrpt::poses::TConstructorFlags_Poses>(), pybind11::arg("") );

		cl.def( pybind11::init<const double, const double, const double, const class mrpt::math::CQuaternion<double> &>(), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("q") );

		cl.def( pybind11::init<const class mrpt::poses::CPose3D &>(), pybind11::arg("p") );

		cl.def( pybind11::init<const struct mrpt::math::TPose3DQuat &>(), pybind11::arg("p") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPose3DQuat const &o){ return new PyCallBack_mrpt_poses_CPose3DQuat(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPose3DQuat const &o){ return new mrpt::poses::CPose3DQuat(o); } ) );
		cl.def_readwrite("m_coords", &mrpt::poses::CPose3DQuat::m_coords);
		cl.def_readwrite("m_quat", &mrpt::poses::CPose3DQuat::m_quat);
		cl.def_static("getClassName", (class mrpt::typemeta::string_literal<24> (*)()) &mrpt::poses::CPose3DQuat::getClassName, "C++: mrpt::poses::CPose3DQuat::getClassName() --> class mrpt::typemeta::string_literal<24>");
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPose3DQuat::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPose3DQuat::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPose3DQuat::*)() const) &mrpt::poses::CPose3DQuat::GetRuntimeClass, "C++: mrpt::poses::CPose3DQuat::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPose3DQuat::*)() const) &mrpt::poses::CPose3DQuat::clone, "C++: mrpt::poses::CPose3DQuat::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPose3DQuat::CreateObject, "C++: mrpt::poses::CPose3DQuat::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def("quat", (class mrpt::math::CQuaternion<double> & (mrpt::poses::CPose3DQuat::*)()) &mrpt::poses::CPose3DQuat::quat, "Read/Write access to the quaternion representing the 3D rotation. \n\nC++: mrpt::poses::CPose3DQuat::quat() --> class mrpt::math::CQuaternion<double> &", pybind11::return_value_policy::automatic);
		cl.def("asTPose", (struct mrpt::math::TPose3DQuat (mrpt::poses::CPose3DQuat::*)() const) &mrpt::poses::CPose3DQuat::asTPose, "C++: mrpt::poses::CPose3DQuat::asTPose() const --> struct mrpt::math::TPose3DQuat");
		cl.def("composeFrom", (void (mrpt::poses::CPose3DQuat::*)(const class mrpt::poses::CPose3DQuat &, const class mrpt::poses::CPose3DQuat &)) &mrpt::poses::CPose3DQuat::composeFrom, "Makes \n  this method is slightly more efficient\n than \"this= A + B;\" since it avoids the temporary object.\n  \n\n A or B can be \"this\" without problems.\n \n\n inverseComposeFrom, composePoint\n\nC++: mrpt::poses::CPose3DQuat::composeFrom(const class mrpt::poses::CPose3DQuat &, const class mrpt::poses::CPose3DQuat &) --> void", pybind11::arg("A"), pybind11::arg("B"));
		cl.def("inverseComposeFrom", (void (mrpt::poses::CPose3DQuat::*)(const class mrpt::poses::CPose3DQuat &, const class mrpt::poses::CPose3DQuat &)) &mrpt::poses::CPose3DQuat::inverseComposeFrom, "Makes \n this method is slightly more efficient\n than \"this= A - B;\" since it avoids the temporary object.\n  \n\n A or B can be \"this\" without problems.\n \n\n composeFrom, composePoint\n\nC++: mrpt::poses::CPose3DQuat::inverseComposeFrom(const class mrpt::poses::CPose3DQuat &, const class mrpt::poses::CPose3DQuat &) --> void", pybind11::arg("A"), pybind11::arg("B"));
		cl.def("__add__", (class mrpt::poses::CPoint3D (mrpt::poses::CPose3DQuat::*)(const class mrpt::poses::CPoint3D &) const) &mrpt::poses::CPose3DQuat::operator+, "Computes the 3D point G such as \n.  \n\n composePoint    \n\nC++: mrpt::poses::CPose3DQuat::operator+(const class mrpt::poses::CPoint3D &) const --> class mrpt::poses::CPoint3D", pybind11::arg("L"));
		cl.def("__add__", (struct mrpt::math::TPoint3D_<double> (mrpt::poses::CPose3DQuat::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::poses::CPose3DQuat::operator+, "Computes the 3D point G such as \n.  \n\n composePoint    \n\nC++: mrpt::poses::CPose3DQuat::operator+(const struct mrpt::math::TPoint3D_<double> &) const --> struct mrpt::math::TPoint3D_<double>", pybind11::arg("L"));
		cl.def("__imul__", (void (mrpt::poses::CPose3DQuat::*)(const double)) &mrpt::poses::CPose3DQuat::operator*=, "Scalar multiplication (all x y z qr qx qy qz elements are multiplied by\n the scalar). \n\nC++: mrpt::poses::CPose3DQuat::operator*=(const double) --> void", pybind11::arg("s"));
		cl.def("__iadd__", (class mrpt::poses::CPose3DQuat & (mrpt::poses::CPose3DQuat::*)(const class mrpt::poses::CPose3DQuat &)) &mrpt::poses::CPose3DQuat::operator+=, "Make \n  \n\nC++: mrpt::poses::CPose3DQuat::operator+=(const class mrpt::poses::CPose3DQuat &) --> class mrpt::poses::CPose3DQuat &", pybind11::return_value_policy::automatic, pybind11::arg("b"));
		cl.def("__add__", (class mrpt::poses::CPose3DQuat (mrpt::poses::CPose3DQuat::*)(const class mrpt::poses::CPose3DQuat &) const) &mrpt::poses::CPose3DQuat::operator+, "Return the composed pose \n  \n\nC++: mrpt::poses::CPose3DQuat::operator+(const class mrpt::poses::CPose3DQuat &) const --> class mrpt::poses::CPose3DQuat", pybind11::arg("p"));
		cl.def("__isub__", (class mrpt::poses::CPose3DQuat & (mrpt::poses::CPose3DQuat::*)(const class mrpt::poses::CPose3DQuat &)) &mrpt::poses::CPose3DQuat::operator-=, "Make \n  \n\nC++: mrpt::poses::CPose3DQuat::operator-=(const class mrpt::poses::CPose3DQuat &) --> class mrpt::poses::CPose3DQuat &", pybind11::return_value_policy::automatic, pybind11::arg("b"));
		cl.def("__sub__", (class mrpt::poses::CPose3DQuat (mrpt::poses::CPose3DQuat::*)(const class mrpt::poses::CPose3DQuat &) const) &mrpt::poses::CPose3DQuat::operator-, "Return the composed pose \n  \n\nC++: mrpt::poses::CPose3DQuat::operator-(const class mrpt::poses::CPose3DQuat &) const --> class mrpt::poses::CPose3DQuat", pybind11::arg("p"));
		cl.def("inverse", (void (mrpt::poses::CPose3DQuat::*)()) &mrpt::poses::CPose3DQuat::inverse, "Convert this pose into its inverse, saving the result in itself. \n\n operator- \n\nC++: mrpt::poses::CPose3DQuat::inverse() --> void");
		cl.def("asString", (std::string (mrpt::poses::CPose3DQuat::*)() const) &mrpt::poses::CPose3DQuat::asString, "Returns a human-readable textual representation of the object as:\n  `\"[x y z qw qx qy qz]\"`\n \n\n fromString\n\nC++: mrpt::poses::CPose3DQuat::asString() const --> std::string");
		cl.def("fromString", (void (mrpt::poses::CPose3DQuat::*)(const std::string &)) &mrpt::poses::CPose3DQuat::fromString, "Set the current object value from a string generated by 'asString' (eg:\n \"[0.02 1.04 -0.8 1 0 0 0]\" )\n \n\n asString\n \n\n std::exception On invalid format\n\nC++: mrpt::poses::CPose3DQuat::fromString(const std::string &) --> void", pybind11::arg("s"));
		cl.def("fromStringRaw", (void (mrpt::poses::CPose3DQuat::*)(const std::string &)) &mrpt::poses::CPose3DQuat::fromStringRaw, "Same as fromString, but without requiring the square brackets in the\n string \n\nC++: mrpt::poses::CPose3DQuat::fromStringRaw(const std::string &) --> void", pybind11::arg("s"));
		cl.def_static("FromString", (class mrpt::poses::CPose3DQuat (*)(const std::string &)) &mrpt::poses::CPose3DQuat::FromString, "C++: mrpt::poses::CPose3DQuat::FromString(const std::string &) --> class mrpt::poses::CPose3DQuat", pybind11::arg("s"));
		cl.def("__getitem__", (double & (mrpt::poses::CPose3DQuat::*)(unsigned int)) &mrpt::poses::CPose3DQuat::operator[], "Read/write [] operator \n\nC++: mrpt::poses::CPose3DQuat::operator[](unsigned int) --> double &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def_static("is_3D", (bool (*)()) &mrpt::poses::CPose3DQuat::is_3D, "C++: mrpt::poses::CPose3DQuat::is_3D() --> bool");
		cl.def_static("is_PDF", (bool (*)()) &mrpt::poses::CPose3DQuat::is_PDF, "C++: mrpt::poses::CPose3DQuat::is_PDF() --> bool");
		cl.def("getPoseMean", (class mrpt::poses::CPose3DQuat & (mrpt::poses::CPose3DQuat::*)()) &mrpt::poses::CPose3DQuat::getPoseMean, "C++: mrpt::poses::CPose3DQuat::getPoseMean() --> class mrpt::poses::CPose3DQuat &", pybind11::return_value_policy::automatic);
		cl.def_static("size", (unsigned long (*)()) &mrpt::poses::CPose3DQuat::size, "C++: mrpt::poses::CPose3DQuat::size() --> unsigned long");
		cl.def_static("empty", (bool (*)()) &mrpt::poses::CPose3DQuat::empty, "C++: mrpt::poses::CPose3DQuat::empty() --> bool");
		cl.def_static("max_size", (unsigned long (*)()) &mrpt::poses::CPose3DQuat::max_size, "C++: mrpt::poses::CPose3DQuat::max_size() --> unsigned long");
		cl.def_static("resize", (void (*)(const unsigned long)) &mrpt::poses::CPose3DQuat::resize, "C++: mrpt::poses::CPose3DQuat::resize(const unsigned long) --> void", pybind11::arg("n"));
		cl.def("assign", (void (mrpt::poses::CPose3DQuat::*)(const unsigned long, const double)) &mrpt::poses::CPose3DQuat::assign, "C++: mrpt::poses::CPose3DQuat::assign(const unsigned long, const double) --> void", pybind11::arg("N"), pybind11::arg("val"));
		cl.def("begin", (struct mrpt::poses::CPose3DQuat::iterator (mrpt::poses::CPose3DQuat::*)()) &mrpt::poses::CPose3DQuat::begin, "C++: mrpt::poses::CPose3DQuat::begin() --> struct mrpt::poses::CPose3DQuat::iterator");
		cl.def("end", (struct mrpt::poses::CPose3DQuat::iterator (mrpt::poses::CPose3DQuat::*)()) &mrpt::poses::CPose3DQuat::end, "C++: mrpt::poses::CPose3DQuat::end() --> struct mrpt::poses::CPose3DQuat::iterator");
		cl.def("swap", (void (mrpt::poses::CPose3DQuat::*)(class mrpt::poses::CPose3DQuat &)) &mrpt::poses::CPose3DQuat::swap, "C++: mrpt::poses::CPose3DQuat::swap(class mrpt::poses::CPose3DQuat &) --> void", pybind11::arg("o"));
		cl.def("setToNaN", (void (mrpt::poses::CPose3DQuat::*)()) &mrpt::poses::CPose3DQuat::setToNaN, "@} \n\nC++: mrpt::poses::CPose3DQuat::setToNaN() --> void");
		cl.def("assign", (class mrpt::poses::CPose3DQuat & (mrpt::poses::CPose3DQuat::*)(const class mrpt::poses::CPose3DQuat &)) &mrpt::poses::CPose3DQuat::operator=, "C++: mrpt::poses::CPose3DQuat::operator=(const class mrpt::poses::CPose3DQuat &) --> class mrpt::poses::CPose3DQuat &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::poses::CPose3DQuat const &o) -> std::string { std::ostringstream s; using namespace mrpt::poses; s << o; return s.str(); } );

		{ // mrpt::poses::CPose3DQuat::iterator file:mrpt/poses/CPose3DQuat.h line:371
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::poses::CPose3DQuat::iterator, std::shared_ptr<mrpt::poses::CPose3DQuat::iterator>> cl(enclosing_class, "iterator", "");
			cl.def( pybind11::init( [](){ return new mrpt::poses::CPose3DQuat::iterator(); } ) );
			cl.def( pybind11::init<class mrpt::poses::CPose3DQuat &, size_t>(), pybind11::arg("obj"), pybind11::arg("start_idx") );

			cl.def( pybind11::init( [](mrpt::poses::CPose3DQuat::iterator const &o){ return new mrpt::poses::CPose3DQuat::iterator(o); } ) );
			cl.def("dereference", (double & (mrpt::poses::CPose3DQuat::iterator::*)() const) &mrpt::poses::CPose3DQuat::iterator::operator*, "C++: mrpt::poses::CPose3DQuat::iterator::operator*() const --> double &", pybind11::return_value_policy::automatic);
			cl.def("pre_increment", (struct mrpt::poses::CPose3DQuat::iterator & (mrpt::poses::CPose3DQuat::iterator::*)()) &mrpt::poses::CPose3DQuat::iterator::operator++, "C++: mrpt::poses::CPose3DQuat::iterator::operator++() --> struct mrpt::poses::CPose3DQuat::iterator &", pybind11::return_value_policy::automatic);
			cl.def("post_increment", (struct mrpt::poses::CPose3DQuat::iterator (mrpt::poses::CPose3DQuat::iterator::*)(int)) &mrpt::poses::CPose3DQuat::iterator::operator++, "C++: mrpt::poses::CPose3DQuat::iterator::operator++(int) --> struct mrpt::poses::CPose3DQuat::iterator", pybind11::arg(""));
			cl.def("pre_decrement", (struct mrpt::poses::CPose3DQuat::iterator & (mrpt::poses::CPose3DQuat::iterator::*)()) &mrpt::poses::CPose3DQuat::iterator::operator--, "C++: mrpt::poses::CPose3DQuat::iterator::operator--() --> struct mrpt::poses::CPose3DQuat::iterator &", pybind11::return_value_policy::automatic);
			cl.def("post_decrement", (struct mrpt::poses::CPose3DQuat::iterator (mrpt::poses::CPose3DQuat::iterator::*)(int)) &mrpt::poses::CPose3DQuat::iterator::operator--, "C++: mrpt::poses::CPose3DQuat::iterator::operator--(int) --> struct mrpt::poses::CPose3DQuat::iterator", pybind11::arg(""));
			cl.def("__iadd__", (struct mrpt::poses::CPose3DQuat::iterator & (mrpt::poses::CPose3DQuat::iterator::*)(long)) &mrpt::poses::CPose3DQuat::iterator::operator+=, "C++: mrpt::poses::CPose3DQuat::iterator::operator+=(long) --> struct mrpt::poses::CPose3DQuat::iterator &", pybind11::return_value_policy::automatic, pybind11::arg("off"));
			cl.def("__add__", (struct mrpt::poses::CPose3DQuat::iterator (mrpt::poses::CPose3DQuat::iterator::*)(long) const) &mrpt::poses::CPose3DQuat::iterator::operator+, "C++: mrpt::poses::CPose3DQuat::iterator::operator+(long) const --> struct mrpt::poses::CPose3DQuat::iterator", pybind11::arg("off"));
			cl.def("__isub__", (struct mrpt::poses::CPose3DQuat::iterator & (mrpt::poses::CPose3DQuat::iterator::*)(long)) &mrpt::poses::CPose3DQuat::iterator::operator-=, "C++: mrpt::poses::CPose3DQuat::iterator::operator-=(long) --> struct mrpt::poses::CPose3DQuat::iterator &", pybind11::return_value_policy::automatic, pybind11::arg("off"));
			cl.def("__sub__", (struct mrpt::poses::CPose3DQuat::iterator (mrpt::poses::CPose3DQuat::iterator::*)(long) const) &mrpt::poses::CPose3DQuat::iterator::operator-, "C++: mrpt::poses::CPose3DQuat::iterator::operator-(long) const --> struct mrpt::poses::CPose3DQuat::iterator", pybind11::arg("off"));
			cl.def("__sub__", (long (mrpt::poses::CPose3DQuat::iterator::*)(const struct mrpt::poses::CPose3DQuat::iterator &) const) &mrpt::poses::CPose3DQuat::iterator::operator-, "C++: mrpt::poses::CPose3DQuat::iterator::operator-(const struct mrpt::poses::CPose3DQuat::iterator &) const --> long", pybind11::arg("it"));
			cl.def("__getitem__", (double & (mrpt::poses::CPose3DQuat::iterator::*)(long) const) &mrpt::poses::CPose3DQuat::iterator::operator[], "C++: mrpt::poses::CPose3DQuat::iterator::operator[](long) const --> double &", pybind11::return_value_policy::automatic, pybind11::arg("off"));
			cl.def("__eq__", (bool (mrpt::poses::CPose3DQuat::iterator::*)(const struct mrpt::poses::CPose3DQuat::iterator &) const) &mrpt::poses::CPose3DQuat::iterator::operator==, "C++: mrpt::poses::CPose3DQuat::iterator::operator==(const struct mrpt::poses::CPose3DQuat::iterator &) const --> bool", pybind11::arg("it"));
			cl.def("__ne__", (bool (mrpt::poses::CPose3DQuat::iterator::*)(const struct mrpt::poses::CPose3DQuat::iterator &) const) &mrpt::poses::CPose3DQuat::iterator::operator!=, "C++: mrpt::poses::CPose3DQuat::iterator::operator!=(const struct mrpt::poses::CPose3DQuat::iterator &) const --> bool", pybind11::arg("it"));
			cl.def("assign", (struct mrpt::poses::CPose3DQuat::iterator & (mrpt::poses::CPose3DQuat::iterator::*)(const struct mrpt::poses::CPose3DQuat::iterator &)) &mrpt::poses::CPose3DQuat::iterator::operator=, "C++: mrpt::poses::CPose3DQuat::iterator::operator=(const struct mrpt::poses::CPose3DQuat::iterator &) --> struct mrpt::poses::CPose3DQuat::iterator &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{ // mrpt::poses::CPose3DQuat::const_iterator file:mrpt/poses/CPose3DQuat.h line:482
			auto & enclosing_class = cl;
			pybind11::class_<mrpt::poses::CPose3DQuat::const_iterator, std::shared_ptr<mrpt::poses::CPose3DQuat::const_iterator>> cl(enclosing_class, "const_iterator", "");
			cl.def( pybind11::init( [](){ return new mrpt::poses::CPose3DQuat::const_iterator(); } ) );
			cl.def( pybind11::init<const class mrpt::poses::CPose3DQuat &, size_t>(), pybind11::arg("obj"), pybind11::arg("start_idx") );

			cl.def( pybind11::init( [](mrpt::poses::CPose3DQuat::const_iterator const &o){ return new mrpt::poses::CPose3DQuat::const_iterator(o); } ) );
			cl.def("dereference", (double (mrpt::poses::CPose3DQuat::const_iterator::*)() const) &mrpt::poses::CPose3DQuat::const_iterator::operator*, "C++: mrpt::poses::CPose3DQuat::const_iterator::operator*() const --> double");
			cl.def("pre_increment", (struct mrpt::poses::CPose3DQuat::const_iterator & (mrpt::poses::CPose3DQuat::const_iterator::*)()) &mrpt::poses::CPose3DQuat::const_iterator::operator++, "C++: mrpt::poses::CPose3DQuat::const_iterator::operator++() --> struct mrpt::poses::CPose3DQuat::const_iterator &", pybind11::return_value_policy::automatic);
			cl.def("post_increment", (struct mrpt::poses::CPose3DQuat::const_iterator (mrpt::poses::CPose3DQuat::const_iterator::*)(int)) &mrpt::poses::CPose3DQuat::const_iterator::operator++, "C++: mrpt::poses::CPose3DQuat::const_iterator::operator++(int) --> struct mrpt::poses::CPose3DQuat::const_iterator", pybind11::arg(""));
			cl.def("pre_decrement", (struct mrpt::poses::CPose3DQuat::const_iterator & (mrpt::poses::CPose3DQuat::const_iterator::*)()) &mrpt::poses::CPose3DQuat::const_iterator::operator--, "C++: mrpt::poses::CPose3DQuat::const_iterator::operator--() --> struct mrpt::poses::CPose3DQuat::const_iterator &", pybind11::return_value_policy::automatic);
			cl.def("post_decrement", (struct mrpt::poses::CPose3DQuat::const_iterator (mrpt::poses::CPose3DQuat::const_iterator::*)(int)) &mrpt::poses::CPose3DQuat::const_iterator::operator--, "C++: mrpt::poses::CPose3DQuat::const_iterator::operator--(int) --> struct mrpt::poses::CPose3DQuat::const_iterator", pybind11::arg(""));
			cl.def("__iadd__", (struct mrpt::poses::CPose3DQuat::const_iterator & (mrpt::poses::CPose3DQuat::const_iterator::*)(long)) &mrpt::poses::CPose3DQuat::const_iterator::operator+=, "C++: mrpt::poses::CPose3DQuat::const_iterator::operator+=(long) --> struct mrpt::poses::CPose3DQuat::const_iterator &", pybind11::return_value_policy::automatic, pybind11::arg("off"));
			cl.def("__add__", (struct mrpt::poses::CPose3DQuat::const_iterator (mrpt::poses::CPose3DQuat::const_iterator::*)(long) const) &mrpt::poses::CPose3DQuat::const_iterator::operator+, "C++: mrpt::poses::CPose3DQuat::const_iterator::operator+(long) const --> struct mrpt::poses::CPose3DQuat::const_iterator", pybind11::arg("off"));
			cl.def("__isub__", (struct mrpt::poses::CPose3DQuat::const_iterator & (mrpt::poses::CPose3DQuat::const_iterator::*)(long)) &mrpt::poses::CPose3DQuat::const_iterator::operator-=, "C++: mrpt::poses::CPose3DQuat::const_iterator::operator-=(long) --> struct mrpt::poses::CPose3DQuat::const_iterator &", pybind11::return_value_policy::automatic, pybind11::arg("off"));
			cl.def("__sub__", (struct mrpt::poses::CPose3DQuat::const_iterator (mrpt::poses::CPose3DQuat::const_iterator::*)(long) const) &mrpt::poses::CPose3DQuat::const_iterator::operator-, "C++: mrpt::poses::CPose3DQuat::const_iterator::operator-(long) const --> struct mrpt::poses::CPose3DQuat::const_iterator", pybind11::arg("off"));
			cl.def("__sub__", (long (mrpt::poses::CPose3DQuat::const_iterator::*)(const struct mrpt::poses::CPose3DQuat::const_iterator &) const) &mrpt::poses::CPose3DQuat::const_iterator::operator-, "C++: mrpt::poses::CPose3DQuat::const_iterator::operator-(const struct mrpt::poses::CPose3DQuat::const_iterator &) const --> long", pybind11::arg("it"));
			cl.def("__getitem__", (double (mrpt::poses::CPose3DQuat::const_iterator::*)(long) const) &mrpt::poses::CPose3DQuat::const_iterator::operator[], "C++: mrpt::poses::CPose3DQuat::const_iterator::operator[](long) const --> double", pybind11::arg("off"));
			cl.def("__eq__", (bool (mrpt::poses::CPose3DQuat::const_iterator::*)(const struct mrpt::poses::CPose3DQuat::const_iterator &) const) &mrpt::poses::CPose3DQuat::const_iterator::operator==, "C++: mrpt::poses::CPose3DQuat::const_iterator::operator==(const struct mrpt::poses::CPose3DQuat::const_iterator &) const --> bool", pybind11::arg("it"));
			cl.def("__ne__", (bool (mrpt::poses::CPose3DQuat::const_iterator::*)(const struct mrpt::poses::CPose3DQuat::const_iterator &) const) &mrpt::poses::CPose3DQuat::const_iterator::operator!=, "C++: mrpt::poses::CPose3DQuat::const_iterator::operator!=(const struct mrpt::poses::CPose3DQuat::const_iterator &) const --> bool", pybind11::arg("it"));
			cl.def("assign", (struct mrpt::poses::CPose3DQuat::const_iterator & (mrpt::poses::CPose3DQuat::const_iterator::*)(const struct mrpt::poses::CPose3DQuat::const_iterator &)) &mrpt::poses::CPose3DQuat::const_iterator::operator=, "C++: mrpt::poses::CPose3DQuat::const_iterator::operator=(const struct mrpt::poses::CPose3DQuat::const_iterator &) --> struct mrpt::poses::CPose3DQuat::const_iterator &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

	}
}
