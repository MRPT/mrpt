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

// mrpt::poses::CPose2D file:mrpt/poses/CPose2D.h line:42
struct PyCallBack_mrpt_poses_CPose2D : public mrpt::poses::CPose2D {
	using mrpt::poses::CPose2D::CPose2D;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose2D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPose2D::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose2D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPose2D::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose2D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPose2D::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose2D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose2D::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose2D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose2D::serializeFrom(a0, a1);
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose2D *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CPose2D::asString();
	}
	void setToNaN() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose2D *>(this), "setToNaN");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose2D::setToNaN();
	}
};

void bind_mrpt_poses_CPose2D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::CPose2D file:mrpt/poses/CPose2D.h line:42
		pybind11::class_<mrpt::poses::CPose2D, std::shared_ptr<mrpt::poses::CPose2D>, PyCallBack_mrpt_poses_CPose2D, mrpt::poses::CPose<mrpt::poses::CPose2D,3UL>, mrpt::serialization::CSerializable, mrpt::Stringifyable> cl(M("mrpt::poses"), "CPose2D", "A class used to store a 2D pose, including the 2D coordinate point and a\n heading (phi) angle.\n  Use this class instead of lightweight mrpt::math::TPose2D when pose/point\n composition is to be called\n  multiple times with the same pose, since this class caches calls to\n expensive trigronometric functions.\n\n For a complete description of Points/Poses, see mrpt::poses::CPoseOrPoint,\n or refer to [this documentation page]\n(http://www.mrpt.org/tutorials/programming/maths-and-geometry/2d_3d_geometry/)\n\n  \n   \n  \n\n \n Read also: \"A tutorial on SE(3) transformation parameterizations and\n on-manifold optimization\", Jose-Luis Blanco.\n http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf\n \n\n CPoseOrPoint,CPoint2D\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPose2D(); }, [](){ return new PyCallBack_mrpt_poses_CPose2D(); } ) );
		cl.def( pybind11::init<const double, const double, const double>(), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("phi") );

		cl.def( pybind11::init<const class mrpt::poses::CPoint2D &>(), pybind11::arg("") );

		cl.def( pybind11::init<const class mrpt::poses::CPose3D &>(), pybind11::arg("") );

		cl.def( pybind11::init<const struct mrpt::math::TPose2D &>(), pybind11::arg("") );

		cl.def( pybind11::init<const class mrpt::poses::CPoint3D &>(), pybind11::arg("") );

		cl.def( pybind11::init<enum mrpt::poses::TConstructorFlags_Poses>(), pybind11::arg("") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPose2D const &o){ return new PyCallBack_mrpt_poses_CPose2D(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPose2D const &o){ return new mrpt::poses::CPose2D(o); } ) );
		cl.def_readwrite("m_coords", &mrpt::poses::CPose2D::m_coords);
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPose2D::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPose2D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPose2D::*)() const) &mrpt::poses::CPose2D::GetRuntimeClass, "C++: mrpt::poses::CPose2D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPose2D::*)() const) &mrpt::poses::CPose2D::clone, "C++: mrpt::poses::CPose2D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPose2D::CreateObject, "C++: mrpt::poses::CPose2D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def_static("Identity", (class mrpt::poses::CPose2D (*)()) &mrpt::poses::CPose2D::Identity, "Returns the identity transformation \n\nC++: mrpt::poses::CPose2D::Identity() --> class mrpt::poses::CPose2D");
		cl.def("asTPose", (struct mrpt::math::TPose2D (mrpt::poses::CPose2D::*)() const) &mrpt::poses::CPose2D::asTPose, "C++: mrpt::poses::CPose2D::asTPose() const --> struct mrpt::math::TPose2D");
		cl.def("phi", (double & (mrpt::poses::CPose2D::*)()) &mrpt::poses::CPose2D::phi, "C++: mrpt::poses::CPose2D::phi() --> double &", pybind11::return_value_policy::automatic);
		cl.def("phi_cos", (double (mrpt::poses::CPose2D::*)() const) &mrpt::poses::CPose2D::phi_cos, "Get a (cached) value of cos(phi), recomputing it only once when phi\n changes. \n\nC++: mrpt::poses::CPose2D::phi_cos() const --> double");
		cl.def("phi_sin", (double (mrpt::poses::CPose2D::*)() const) &mrpt::poses::CPose2D::phi_sin, "Get a (cached) value of sin(phi), recomputing it only once when phi\n changes. \n\nC++: mrpt::poses::CPose2D::phi_sin() const --> double");
		cl.def("phi", (void (mrpt::poses::CPose2D::*)(double)) &mrpt::poses::CPose2D::phi, "Set the phi angle of the 2D pose (in radians) \n\nC++: mrpt::poses::CPose2D::phi(double) --> void", pybind11::arg("angle"));
		cl.def("phi_incr", (void (mrpt::poses::CPose2D::*)(const double)) &mrpt::poses::CPose2D::phi_incr, "Increment the PHI angle (without checking the 2 PI range, call\n normalizePhi is needed) \n\nC++: mrpt::poses::CPose2D::phi_incr(const double) --> void", pybind11::arg("Aphi"));
		cl.def("asVector", (void (mrpt::poses::CPose2D::*)(class mrpt::math::CMatrixFixed<double, 3, 1> &) const) &mrpt::poses::CPose2D::asVector, "Returns a 1x3 vector with [x y phi] \n\nC++: mrpt::poses::CPose2D::asVector(class mrpt::math::CMatrixFixed<double, 3, 1> &) const --> void", pybind11::arg("v"));
		cl.def("translation", (struct mrpt::math::TPoint2D_<double> (mrpt::poses::CPose2D::*)() const) &mrpt::poses::CPose2D::translation, "Returns the (x,y) translational part of the SE(2) transformation. \n\nC++: mrpt::poses::CPose2D::translation() const --> struct mrpt::math::TPoint2D_<double>");
		cl.def("getHomogeneousMatrix", (void (mrpt::poses::CPose2D::*)(class mrpt::math::CMatrixFixed<double, 4, 4> &) const) &mrpt::poses::CPose2D::getHomogeneousMatrix, "Returns the corresponding 4x4 homogeneous transformation matrix for the\n point(translation) or pose (translation+orientation).\n \n\n getInverseHomogeneousMatrix\n\nC++: mrpt::poses::CPose2D::getHomogeneousMatrix(class mrpt::math::CMatrixFixed<double, 4, 4> &) const --> void", pybind11::arg("out_HM"));
		cl.def("getRotationMatrix", (void (mrpt::poses::CPose2D::*)(class mrpt::math::CMatrixFixed<double, 2, 2> &) const) &mrpt::poses::CPose2D::getRotationMatrix, "Returns the SE(2) 2x2 rotation matrix \n\nC++: mrpt::poses::CPose2D::getRotationMatrix(class mrpt::math::CMatrixFixed<double, 2, 2> &) const --> void", pybind11::arg("R"));
		cl.def("getRotationMatrix", (void (mrpt::poses::CPose2D::*)(class mrpt::math::CMatrixFixed<double, 3, 3> &) const) &mrpt::poses::CPose2D::getRotationMatrix, "Returns the equivalent SE(3) 3x3 rotation matrix, with (2,2)=1. \n\nC++: mrpt::poses::CPose2D::getRotationMatrix(class mrpt::math::CMatrixFixed<double, 3, 3> &) const --> void", pybind11::arg("R"));
		cl.def("__add__", (class mrpt::poses::CPose2D (mrpt::poses::CPose2D::*)(const class mrpt::poses::CPose2D &) const) &mrpt::poses::CPose2D::operator+, "The operator \n is the pose compounding operator.\n\nC++: mrpt::poses::CPose2D::operator+(const class mrpt::poses::CPose2D &) const --> class mrpt::poses::CPose2D", pybind11::arg("D"));
		cl.def("composeFrom", (void (mrpt::poses::CPose2D::*)(const class mrpt::poses::CPose2D &, const class mrpt::poses::CPose2D &)) &mrpt::poses::CPose2D::composeFrom, "Makes \n\n  \n A or B can be \"this\" without problems.  \n\nC++: mrpt::poses::CPose2D::composeFrom(const class mrpt::poses::CPose2D &, const class mrpt::poses::CPose2D &) --> void", pybind11::arg("A"), pybind11::arg("B"));
		cl.def("__add__", (class mrpt::poses::CPose3D (mrpt::poses::CPose2D::*)(const class mrpt::poses::CPose3D &) const) &mrpt::poses::CPose2D::operator+, "The operator \n is the pose compounding operator.\n\nC++: mrpt::poses::CPose2D::operator+(const class mrpt::poses::CPose3D &) const --> class mrpt::poses::CPose3D", pybind11::arg("D"));
		cl.def("__add__", (class mrpt::poses::CPoint2D (mrpt::poses::CPose2D::*)(const class mrpt::poses::CPoint2D &) const) &mrpt::poses::CPose2D::operator+, "The operator \n is the pose/point compounding\n operator.  \n\nC++: mrpt::poses::CPose2D::operator+(const class mrpt::poses::CPoint2D &) const --> class mrpt::poses::CPoint2D", pybind11::arg("u"));
		cl.def("composePoint", (void (mrpt::poses::CPose2D::*)(double, double, double &, double &) const) &mrpt::poses::CPose2D::composePoint, "An alternative, slightly more efficient way of doing \n\n with G and L being 2D points and P this 2D pose.  \n\nC++: mrpt::poses::CPose2D::composePoint(double, double, double &, double &) const --> void", pybind11::arg("lx"), pybind11::arg("ly"), pybind11::arg("gx"), pybind11::arg("gy"));
		cl.def("composePoint", (void (mrpt::poses::CPose2D::*)(const struct mrpt::math::TPoint2D_<double> &, struct mrpt::math::TPoint2D_<double> &) const) &mrpt::poses::CPose2D::composePoint, "overload \n with G and L being 2D points and P this\n 2D pose \n\nC++: mrpt::poses::CPose2D::composePoint(const struct mrpt::math::TPoint2D_<double> &, struct mrpt::math::TPoint2D_<double> &) const --> void", pybind11::arg("l"), pybind11::arg("g"));
		cl.def("composePoint", (struct mrpt::math::TPoint3D_<double> (mrpt::poses::CPose2D::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::poses::CPose2D::composePoint, "C++: mrpt::poses::CPose2D::composePoint(const struct mrpt::math::TPoint3D_<double> &) const --> struct mrpt::math::TPoint3D_<double>", pybind11::arg("l"));
		cl.def("composePoint", (void (mrpt::poses::CPose2D::*)(const struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPoint3D_<double> &) const) &mrpt::poses::CPose2D::composePoint, "overload \n with G and L being 3D points and P this\n 2D pose (the \"z\" coordinate remains unmodified) \n\nC++: mrpt::poses::CPose2D::composePoint(const struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPoint3D_<double> &) const --> void", pybind11::arg("l"), pybind11::arg("g"));
		cl.def("composePoint", (void (mrpt::poses::CPose2D::*)(double, double, double, double &, double &, double &) const) &mrpt::poses::CPose2D::composePoint, "overload (the \"z\" coordinate remains unmodified) \n\nC++: mrpt::poses::CPose2D::composePoint(double, double, double, double &, double &, double &) const --> void", pybind11::arg("lx"), pybind11::arg("ly"), pybind11::arg("lz"), pybind11::arg("gx"), pybind11::arg("gy"), pybind11::arg("gz"));
		cl.def("inverseComposePoint", (void (mrpt::poses::CPose2D::*)(const double, const double, double &, double &) const) &mrpt::poses::CPose2D::inverseComposePoint, "Computes the 2D point L such as \n. \n\n composePoint, composeFrom \n\nC++: mrpt::poses::CPose2D::inverseComposePoint(const double, const double, double &, double &) const --> void", pybind11::arg("gx"), pybind11::arg("gy"), pybind11::arg("lx"), pybind11::arg("ly"));
		cl.def("inverseComposePoint", (void (mrpt::poses::CPose2D::*)(const struct mrpt::math::TPoint2D_<double> &, struct mrpt::math::TPoint2D_<double> &) const) &mrpt::poses::CPose2D::inverseComposePoint, "C++: mrpt::poses::CPose2D::inverseComposePoint(const struct mrpt::math::TPoint2D_<double> &, struct mrpt::math::TPoint2D_<double> &) const --> void", pybind11::arg("g"), pybind11::arg("l"));
		cl.def("inverseComposePoint", (struct mrpt::math::TPoint2D_<double> (mrpt::poses::CPose2D::*)(const struct mrpt::math::TPoint2D_<double> &) const) &mrpt::poses::CPose2D::inverseComposePoint, "C++: mrpt::poses::CPose2D::inverseComposePoint(const struct mrpt::math::TPoint2D_<double> &) const --> struct mrpt::math::TPoint2D_<double>", pybind11::arg("g"));
		cl.def("__add__", (class mrpt::poses::CPoint3D (mrpt::poses::CPose2D::*)(const class mrpt::poses::CPoint3D &) const) &mrpt::poses::CPose2D::operator+, "The operator \n is the pose/point compounding\n operator. \n\nC++: mrpt::poses::CPose2D::operator+(const class mrpt::poses::CPoint3D &) const --> class mrpt::poses::CPoint3D", pybind11::arg("u"));
		cl.def("inverseComposeFrom", (void (mrpt::poses::CPose2D::*)(const class mrpt::poses::CPose2D &, const class mrpt::poses::CPose2D &)) &mrpt::poses::CPose2D::inverseComposeFrom, "Makes \n this method is slightly more efficient\n than \"this= A - B;\" since it avoids the temporary object.\n  \n\n A or B can be \"this\" without problems.\n \n\n composeFrom, composePoint\n\nC++: mrpt::poses::CPose2D::inverseComposeFrom(const class mrpt::poses::CPose2D &, const class mrpt::poses::CPose2D &) --> void", pybind11::arg("A"), pybind11::arg("B"));
		cl.def("inverse", (void (mrpt::poses::CPose2D::*)()) &mrpt::poses::CPose2D::inverse, "Convert this pose into its inverse, saving the result in itself. \n\n operator- \n\nC++: mrpt::poses::CPose2D::inverse() --> void");
		cl.def("__sub__", (class mrpt::poses::CPose2D (mrpt::poses::CPose2D::*)(const class mrpt::poses::CPose2D &) const) &mrpt::poses::CPose2D::operator-, "Compute \n  \n\nC++: mrpt::poses::CPose2D::operator-(const class mrpt::poses::CPose2D &) const --> class mrpt::poses::CPose2D", pybind11::arg("b"));
		cl.def("__sub__", (class mrpt::poses::CPose3D (mrpt::poses::CPose2D::*)(const class mrpt::poses::CPose3D &) const) &mrpt::poses::CPose2D::operator-, "The operator \n is the pose inverse compounding\n operator. \n\nC++: mrpt::poses::CPose2D::operator-(const class mrpt::poses::CPose3D &) const --> class mrpt::poses::CPose3D", pybind11::arg("b"));
		cl.def("AddComponents", (void (mrpt::poses::CPose2D::*)(const class mrpt::poses::CPose2D &)) &mrpt::poses::CPose2D::AddComponents, "Scalar sum of components: This is diferent from poses\n    composition, which is implemented as \"+\" operators in \"CPose\" derived\n classes.\n\nC++: mrpt::poses::CPose2D::AddComponents(const class mrpt::poses::CPose2D &) --> void", pybind11::arg("p"));
		cl.def("__imul__", (void (mrpt::poses::CPose2D::*)(const double)) &mrpt::poses::CPose2D::operator*=, "Scalar multiplication.\n\nC++: mrpt::poses::CPose2D::operator*=(const double) --> void", pybind11::arg("s"));
		cl.def("__iadd__", (class mrpt::poses::CPose2D & (mrpt::poses::CPose2D::*)(const class mrpt::poses::CPose2D &)) &mrpt::poses::CPose2D::operator+=, "Make \n  \n\nC++: mrpt::poses::CPose2D::operator+=(const class mrpt::poses::CPose2D &) --> class mrpt::poses::CPose2D &", pybind11::return_value_policy::automatic, pybind11::arg("b"));
		cl.def("normalizePhi", (void (mrpt::poses::CPose2D::*)()) &mrpt::poses::CPose2D::normalizePhi, "Forces \"phi\" to be in the range [-pi,pi];\n\nC++: mrpt::poses::CPose2D::normalizePhi() --> void");
		cl.def("getOppositeScalar", (class mrpt::poses::CPose2D (mrpt::poses::CPose2D::*)() const) &mrpt::poses::CPose2D::getOppositeScalar, "Return the opposite of the current pose instance by taking the negative\n of all its components \n   \n\nC++: mrpt::poses::CPose2D::getOppositeScalar() const --> class mrpt::poses::CPose2D");
		cl.def("asString", (std::string (mrpt::poses::CPose2D::*)() const) &mrpt::poses::CPose2D::asString, "Returns a human-readable textual representation of the object (eg: \"[x y\n yaw]\", yaw in degrees)\n \n\n fromString\n\nC++: mrpt::poses::CPose2D::asString() const --> std::string");
		cl.def("fromString", (void (mrpt::poses::CPose2D::*)(const std::string &)) &mrpt::poses::CPose2D::fromString, "Set the current object value from a string generated by 'asString' (eg:\n \"[0.02 1.04 -0.8]\" )\n \n\n asString\n \n\n std::exception On invalid format\n\nC++: mrpt::poses::CPose2D::fromString(const std::string &) --> void", pybind11::arg("s"));
		cl.def("fromStringRaw", (void (mrpt::poses::CPose2D::*)(const std::string &)) &mrpt::poses::CPose2D::fromStringRaw, "Same as fromString, but without requiring the square brackets in the\n string \n\nC++: mrpt::poses::CPose2D::fromStringRaw(const std::string &) --> void", pybind11::arg("s"));
		cl.def_static("FromString", (class mrpt::poses::CPose2D (*)(const std::string &)) &mrpt::poses::CPose2D::FromString, "C++: mrpt::poses::CPose2D::FromString(const std::string &) --> class mrpt::poses::CPose2D", pybind11::arg("s"));
		cl.def("__getitem__", (double & (mrpt::poses::CPose2D::*)(unsigned int)) &mrpt::poses::CPose2D::operator[], "C++: mrpt::poses::CPose2D::operator[](unsigned int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPose2D::*)(const class mrpt::poses::CPose2D &)) &mrpt::poses::CPose2D::changeCoordinatesReference, "makes: this = p (+) this \n\nC++: mrpt::poses::CPose2D::changeCoordinatesReference(const class mrpt::poses::CPose2D &) --> void", pybind11::arg("p"));
		cl.def("distance2DFrobeniusTo", (double (mrpt::poses::CPose2D::*)(const class mrpt::poses::CPose2D &) const) &mrpt::poses::CPose2D::distance2DFrobeniusTo, "Returns the 2D distance from this pose/point to a 2D pose using the\n Frobenius distance. \n\nC++: mrpt::poses::CPose2D::distance2DFrobeniusTo(const class mrpt::poses::CPose2D &) const --> double", pybind11::arg("p"));
		cl.def_static("is_3D", (bool (*)()) &mrpt::poses::CPose2D::is_3D, "C++: mrpt::poses::CPose2D::is_3D() --> bool");
		cl.def_static("is_PDF", (bool (*)()) &mrpt::poses::CPose2D::is_PDF, "C++: mrpt::poses::CPose2D::is_PDF() --> bool");
		cl.def("getPoseMean", (class mrpt::poses::CPose2D & (mrpt::poses::CPose2D::*)()) &mrpt::poses::CPose2D::getPoseMean, "C++: mrpt::poses::CPose2D::getPoseMean() --> class mrpt::poses::CPose2D &", pybind11::return_value_policy::automatic);
		cl.def("setToNaN", (void (mrpt::poses::CPose2D::*)()) &mrpt::poses::CPose2D::setToNaN, "C++: mrpt::poses::CPose2D::setToNaN() --> void");
		cl.def_static("size", (unsigned long (*)()) &mrpt::poses::CPose2D::size, "C++: mrpt::poses::CPose2D::size() --> unsigned long");
		cl.def_static("empty", (bool (*)()) &mrpt::poses::CPose2D::empty, "C++: mrpt::poses::CPose2D::empty() --> bool");
		cl.def_static("max_size", (unsigned long (*)()) &mrpt::poses::CPose2D::max_size, "C++: mrpt::poses::CPose2D::max_size() --> unsigned long");
		cl.def_static("resize", (void (*)(size_t)) &mrpt::poses::CPose2D::resize, "C++: mrpt::poses::CPose2D::resize(size_t) --> void", pybind11::arg("n"));
		cl.def("assign", (class mrpt::poses::CPose2D & (mrpt::poses::CPose2D::*)(const class mrpt::poses::CPose2D &)) &mrpt::poses::CPose2D::operator=, "C++: mrpt::poses::CPose2D::operator=(const class mrpt::poses::CPose2D &) --> class mrpt::poses::CPose2D &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::poses::CPose2D const &o) -> std::string { std::ostringstream s; using namespace mrpt::poses; s << o; return s.str(); } );
	}
}
