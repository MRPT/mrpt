#include <ios>
#include <iterator>
#include <locale>
#include <memory>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TPose3DQuat.h>
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

// mrpt::poses::CPose3D file:mrpt/poses/CPose3D.h line:79
struct PyCallBack_mrpt_poses_CPose3D : public mrpt::poses::CPose3D {
	using mrpt::poses::CPose3D::CPose3D;

	const struct mrpt::rtti::TRuntimeClassId * GetRuntimeClass() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3D *>(this), "GetRuntimeClass");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const struct mrpt::rtti::TRuntimeClassId *>::value) {
				static pybind11::detail::override_caster_t<const struct mrpt::rtti::TRuntimeClassId *> caster;
				return pybind11::detail::cast_ref<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<const struct mrpt::rtti::TRuntimeClassId *>(std::move(o));
		}
		return CPose3D::GetRuntimeClass();
	}
	class mrpt::rtti::CObject * clone() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3D *>(this), "clone");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<class mrpt::rtti::CObject *>::value) {
				static pybind11::detail::override_caster_t<class mrpt::rtti::CObject *> caster;
				return pybind11::detail::cast_ref<class mrpt::rtti::CObject *>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<class mrpt::rtti::CObject *>(std::move(o));
		}
		return CPose3D::clone();
	}
	uint8_t serializeGetVersion() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3D *>(this), "serializeGetVersion");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint8_t>::value) {
				static pybind11::detail::override_caster_t<uint8_t> caster;
				return pybind11::detail::cast_ref<uint8_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint8_t>(std::move(o));
		}
		return CPose3D::serializeGetVersion();
	}
	void serializeTo(class mrpt::serialization::CArchive & a0) const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3D *>(this), "serializeTo");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3D::serializeTo(a0);
	}
	void serializeFrom(class mrpt::serialization::CArchive & a0, uint8_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3D *>(this), "serializeFrom");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3D::serializeFrom(a0, a1);
	}
	std::string asString() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3D *>(this), "asString");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CPose3D::asString();
	}
	void setToNaN() override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::poses::CPose3D *>(this), "setToNaN");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<void>::value) {
				static pybind11::detail::override_caster_t<void> caster;
				return pybind11::detail::cast_ref<void>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<void>(std::move(o));
		}
		return CPose3D::setToNaN();
	}
};

void bind_mrpt_poses_CPose3D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::poses::CPose3D file:mrpt/poses/CPose3D.h line:79
		pybind11::class_<mrpt::poses::CPose3D, std::shared_ptr<mrpt::poses::CPose3D>, PyCallBack_mrpt_poses_CPose3D, mrpt::poses::CPose<mrpt::poses::CPose3D,6UL>, mrpt::serialization::CSerializable, mrpt::Stringifyable> cl(M("mrpt::poses"), "CPose3D", "A SE(3) pose, comprising a 3D translation and a 3D rotation.\n\n The transformation is stored in two separate containers:\n - a 3-array for the translation ∈ R³, and\n - a 3x3 rotation matrix ∈ SO(3).\n\n This class allows parameterizing 6D poses as a 6-vector\n `[x y z yaw pitch roll]` (read below for the angles convention).\n Note however, that the yaw/pitch/roll angles are only computed (on-demand and\n transparently) when the user requests them. Normally, rotations and\n transformations are always handled via the 3x3 SO(3) rotation matrix.\n\n Yaw/Pitch/Roll angles are defined as successive rotations around *local*\n (dynamic) axes in the Z/Y/X order:\n\n ![CPose3D](CPose3D.gif)\n\n It can be shown that \"yaw, pitch, roll\" can be also understood as\n rotations around *global* (static) axes. Both conventions lead to exactly\n the same SE(3) transformations, although in it is conventional to write\n the numbers in reverse order.\n That is, the same SO(3) rotation can be described equivalently with any of\n these two parameterizations:\n\n - In local axes Z/Y/X convention: [yaw pitch roll]   (This is the convention\n used in mrpt::poses::CPose3D)\n - In global axes X/Y/Z convention: [roll pitch yaw] (One of the Euler angles\n conventions)\n\n For further descriptions of point & pose classes, see\n mrpt::poses::CPoseOrPoint or refer\n to the [2D/3D Geometry tutorial](http://www.mrpt.org/2D_3D_Geometry) online.\n\n To change the individual components of the pose, use CPose3D::setFromValues.\n This class assures that the internal SO(3) rotation matrix is always\n up-to-date with the \"yaw pitch roll\" members.\n\n Rotations in 3D can be also represented by quaternions. See\n mrpt::math::CQuaternion, and method CPose3D::getAsQuaternion.\n\n This class and CPose3DQuat are very similar, and they can be converted to the\n each other automatically via transformation constructors.\n\n For Lie algebra methods, see mrpt::poses::Lie.\n\n \n Read also: \"A tutorial on SE(3) transformation parameterizations and\n on-manifold optimization\", in \n\n \n\n \n CPoseOrPoint,CPoint3D, mrpt::math::CQuaternion");
		cl.def( pybind11::init( [](){ return new mrpt::poses::CPose3D(); }, [](){ return new PyCallBack_mrpt_poses_CPose3D(); } ) );
		cl.def( pybind11::init( [](const double & a0, const double & a1, const double & a2){ return new mrpt::poses::CPose3D(a0, a1, a2); }, [](const double & a0, const double & a1, const double & a2){ return new PyCallBack_mrpt_poses_CPose3D(a0, a1, a2); } ), "doc");
		cl.def( pybind11::init( [](const double & a0, const double & a1, const double & a2, const double & a3){ return new mrpt::poses::CPose3D(a0, a1, a2, a3); }, [](const double & a0, const double & a1, const double & a2, const double & a3){ return new PyCallBack_mrpt_poses_CPose3D(a0, a1, a2, a3); } ), "doc");
		cl.def( pybind11::init( [](const double & a0, const double & a1, const double & a2, const double & a3, const double & a4){ return new mrpt::poses::CPose3D(a0, a1, a2, a3, a4); }, [](const double & a0, const double & a1, const double & a2, const double & a3, const double & a4){ return new PyCallBack_mrpt_poses_CPose3D(a0, a1, a2, a3, a4); } ), "doc");
		cl.def( pybind11::init<const double, const double, const double, const double, const double, const double>(), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("yaw"), pybind11::arg("pitch"), pybind11::arg("roll") );

		cl.def( pybind11::init<const class mrpt::math::CMatrixDynamic<double> &>(), pybind11::arg("m") );

		cl.def( pybind11::init<const class mrpt::math::CMatrixFixed<double, 4, 4> &>(), pybind11::arg("m") );

		cl.def( pybind11::init<const class mrpt::math::CMatrixFixed<double, 3, 3> &, const class mrpt::math::CMatrixFixed<double, 3, 1> &>(), pybind11::arg("rot"), pybind11::arg("xyz") );

		cl.def( pybind11::init<const class mrpt::poses::CPose2D &>(), pybind11::arg("") );

		cl.def( pybind11::init<const class mrpt::poses::CPoint3D &>(), pybind11::arg("") );

		cl.def( pybind11::init<const struct mrpt::math::TPose3D &>(), pybind11::arg("") );

		cl.def( pybind11::init<const class mrpt::math::CQuaternion<double> &, const double, const double, const double>(), pybind11::arg("q"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z") );

		cl.def( pybind11::init<const class mrpt::poses::CPose3DQuat &>(), pybind11::arg("") );

		cl.def( pybind11::init<enum mrpt::poses::TConstructorFlags_Poses>(), pybind11::arg("") );

		cl.def( pybind11::init<const class mrpt::math::CMatrixFixed<double, 12, 1> &>(), pybind11::arg("vec12") );

		cl.def( pybind11::init( [](PyCallBack_mrpt_poses_CPose3D const &o){ return new PyCallBack_mrpt_poses_CPose3D(o); } ) );
		cl.def( pybind11::init( [](mrpt::poses::CPose3D const &o){ return new mrpt::poses::CPose3D(o); } ) );
		cl.def_readwrite("m_coords", &mrpt::poses::CPose3D::m_coords);
		cl.def("setFrom12Vector", (void (mrpt::poses::CPose3D::*)(const class mrpt::math::CMatrixFixed<double, 12, 1> &)) &mrpt::poses::CPose3D::setFrom12Vector<mrpt::math::CMatrixFixed<double, 12, 1>>, "C++: mrpt::poses::CPose3D::setFrom12Vector(const class mrpt::math::CMatrixFixed<double, 12, 1> &) --> void", pybind11::arg("vec12"));
		cl.def_static("GetRuntimeClassIdStatic", (const struct mrpt::rtti::TRuntimeClassId & (*)()) &mrpt::poses::CPose3D::GetRuntimeClassIdStatic, "C++: mrpt::poses::CPose3D::GetRuntimeClassIdStatic() --> const struct mrpt::rtti::TRuntimeClassId &", pybind11::return_value_policy::automatic);
		cl.def("GetRuntimeClass", (const struct mrpt::rtti::TRuntimeClassId * (mrpt::poses::CPose3D::*)() const) &mrpt::poses::CPose3D::GetRuntimeClass, "C++: mrpt::poses::CPose3D::GetRuntimeClass() const --> const struct mrpt::rtti::TRuntimeClassId *", pybind11::return_value_policy::automatic);
		cl.def("clone", (class mrpt::rtti::CObject * (mrpt::poses::CPose3D::*)() const) &mrpt::poses::CPose3D::clone, "C++: mrpt::poses::CPose3D::clone() const --> class mrpt::rtti::CObject *", pybind11::return_value_policy::automatic);
		cl.def_static("CreateObject", (class std::shared_ptr<class mrpt::rtti::CObject> (*)()) &mrpt::poses::CPose3D::CreateObject, "C++: mrpt::poses::CPose3D::CreateObject() --> class std::shared_ptr<class mrpt::rtti::CObject>");
		cl.def_static("Identity", (class mrpt::poses::CPose3D (*)()) &mrpt::poses::CPose3D::Identity, "Returns the identity transformation \n\nC++: mrpt::poses::CPose3D::Identity() --> class mrpt::poses::CPose3D");
		cl.def_static("FromXYZYawPitchRoll", (class mrpt::poses::CPose3D (*)(double, double, double, double, double, double)) &mrpt::poses::CPose3D::FromXYZYawPitchRoll, "Builds a pose from a translation (x,y,z) in\n meters and (yaw,pitch,roll) angles in radians. \n\n (New in MRPT 2.1.8)\n\nC++: mrpt::poses::CPose3D::FromXYZYawPitchRoll(double, double, double, double, double, double) --> class mrpt::poses::CPose3D", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("yaw"), pybind11::arg("pitch"), pybind11::arg("roll"));
		cl.def_static("FromYawPitchRoll", (class mrpt::poses::CPose3D (*)(double, double, double)) &mrpt::poses::CPose3D::FromYawPitchRoll, "Builds a pose with a null translation and (yaw,pitch,roll) angles in\n radians. \n\n (New in MRPT 2.1.8)\n\nC++: mrpt::poses::CPose3D::FromYawPitchRoll(double, double, double) --> class mrpt::poses::CPose3D", pybind11::arg("yaw"), pybind11::arg("pitch"), pybind11::arg("roll"));
		cl.def_static("FromTranslation", (class mrpt::poses::CPose3D (*)(double, double, double)) &mrpt::poses::CPose3D::FromTranslation, "Builds a pose with a translation without rotation \n (New in\n MRPT 2.1.8)\n\nC++: mrpt::poses::CPose3D::FromTranslation(double, double, double) --> class mrpt::poses::CPose3D", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def_static("FromTranslation", (class mrpt::poses::CPose3D (*)(const struct mrpt::math::TPoint3D_<double> &)) &mrpt::poses::CPose3D::FromTranslation, "C++: mrpt::poses::CPose3D::FromTranslation(const struct mrpt::math::TPoint3D_<double> &) --> class mrpt::poses::CPose3D", pybind11::arg("t"));
		cl.def("asTPose", (struct mrpt::math::TPose3D (mrpt::poses::CPose3D::*)() const) &mrpt::poses::CPose3D::asTPose, "C++: mrpt::poses::CPose3D::asTPose() const --> struct mrpt::math::TPose3D");
		cl.def_static("FromQuaternion", (class mrpt::poses::CPose3D (*)(const class mrpt::math::CQuaternion<double> &)) &mrpt::poses::CPose3D::FromQuaternion, "Builds a pose from a quaternion (and no translation).\n \n\n (New in MRPT 2.1.8)\n\nC++: mrpt::poses::CPose3D::FromQuaternion(const class mrpt::math::CQuaternion<double> &) --> class mrpt::poses::CPose3D", pybind11::arg("q"));
		cl.def_static("FromQuaternionAndTranslation", (class mrpt::poses::CPose3D (*)(const class mrpt::math::CQuaternion<double> &, double, double, double)) &mrpt::poses::CPose3D::FromQuaternionAndTranslation, "Builds a pose from a quaternion and a (x,y,z) translation.\n \n\n (New in MRPT 2.1.8)\n\nC++: mrpt::poses::CPose3D::FromQuaternionAndTranslation(const class mrpt::math::CQuaternion<double> &, double, double, double) --> class mrpt::poses::CPose3D", pybind11::arg("q"), pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"));
		cl.def("getHomogeneousMatrix", (void (mrpt::poses::CPose3D::*)(class mrpt::math::CMatrixFixed<double, 4, 4> &) const) &mrpt::poses::CPose3D::getHomogeneousMatrix, "Returns the corresponding 4x4 homogeneous transformation matrix for the\n point(translation) or pose (translation+orientation).\n \n\n getInverseHomogeneousMatrix, getRotationMatrix\n\nC++: mrpt::poses::CPose3D::getHomogeneousMatrix(class mrpt::math::CMatrixFixed<double, 4, 4> &) const --> void", pybind11::arg("out_HM"));
		cl.def("getRotationMatrix", (void (mrpt::poses::CPose3D::*)(class mrpt::math::CMatrixFixed<double, 3, 3> &) const) &mrpt::poses::CPose3D::getRotationMatrix, "Get the 3x3 rotation matrix \n getHomogeneousMatrix  \n\nC++: mrpt::poses::CPose3D::getRotationMatrix(class mrpt::math::CMatrixFixed<double, 3, 3> &) const --> void", pybind11::arg("ROT"));
		cl.def("getRotationMatrix", (const class mrpt::math::CMatrixFixed<double, 3, 3> & (mrpt::poses::CPose3D::*)() const) &mrpt::poses::CPose3D::getRotationMatrix, "C++: mrpt::poses::CPose3D::getRotationMatrix() const --> const class mrpt::math::CMatrixFixed<double, 3, 3> &", pybind11::return_value_policy::automatic);
		cl.def("setRotationMatrix", (void (mrpt::poses::CPose3D::*)(const class mrpt::math::CMatrixFixed<double, 3, 3> &)) &mrpt::poses::CPose3D::setRotationMatrix, "Sets the 3x3 rotation matrix \n getRotationMatrix, getHomogeneousMatrix\n\nC++: mrpt::poses::CPose3D::setRotationMatrix(const class mrpt::math::CMatrixFixed<double, 3, 3> &) --> void", pybind11::arg("ROT"));
		cl.def("translation", (struct mrpt::math::TPoint3D_<double> (mrpt::poses::CPose3D::*)() const) &mrpt::poses::CPose3D::translation, "Returns the (x,y,z) translational part of the SE(3) transformation. \n\nC++: mrpt::poses::CPose3D::translation() const --> struct mrpt::math::TPoint3D_<double>");
		cl.def("__add__", (class mrpt::poses::CPose3D (mrpt::poses::CPose3D::*)(const class mrpt::poses::CPose3D &) const) &mrpt::poses::CPose3D::operator+, "The operator \n is the pose compounding operator. \n\nC++: mrpt::poses::CPose3D::operator+(const class mrpt::poses::CPose3D &) const --> class mrpt::poses::CPose3D", pybind11::arg("b"));
		cl.def("__add__", (class mrpt::poses::CPoint3D (mrpt::poses::CPose3D::*)(const class mrpt::poses::CPoint3D &) const) &mrpt::poses::CPose3D::operator+, "The operator \n is the pose compounding operator. \n\nC++: mrpt::poses::CPose3D::operator+(const class mrpt::poses::CPoint3D &) const --> class mrpt::poses::CPoint3D", pybind11::arg("b"));
		cl.def("__add__", (class mrpt::poses::CPoint3D (mrpt::poses::CPose3D::*)(const class mrpt::poses::CPoint2D &) const) &mrpt::poses::CPose3D::operator+, "The operator \n is the pose compounding operator. \n\nC++: mrpt::poses::CPose3D::operator+(const class mrpt::poses::CPoint2D &) const --> class mrpt::poses::CPoint3D", pybind11::arg("b"));
		cl.def("sphericalCoordinates", (void (mrpt::poses::CPose3D::*)(const struct mrpt::math::TPoint3D_<double> &, double &, double &, double &) const) &mrpt::poses::CPose3D::sphericalCoordinates, "Computes the spherical coordinates of a 3D point as seen from the 6D\n pose specified by this object. For the coordinate system see the top of\n this page. \n\nC++: mrpt::poses::CPose3D::sphericalCoordinates(const struct mrpt::math::TPoint3D_<double> &, double &, double &, double &) const --> void", pybind11::arg("point"), pybind11::arg("out_range"), pybind11::arg("out_yaw"), pybind11::arg("out_pitch"));
		cl.def("composePoint", (void (mrpt::poses::CPose3D::*)(const struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPoint3D_<double> &) const) &mrpt::poses::CPose3D::composePoint, "An alternative, slightly more efficient way of doing \n\n with G and L being 3D points and P this 6D pose.\n \n\n local_point is passed by value to allow global and local point to\n be the same variable\n\nC++: mrpt::poses::CPose3D::composePoint(const struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPoint3D_<double> &) const --> void", pybind11::arg("local_point"), pybind11::arg("global_point"));
		cl.def("composePoint", (struct mrpt::math::TPoint3D_<double> (mrpt::poses::CPose3D::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::poses::CPose3D::composePoint, "C++: mrpt::poses::CPose3D::composePoint(const struct mrpt::math::TPoint3D_<double> &) const --> struct mrpt::math::TPoint3D_<double>", pybind11::arg("l"));
		cl.def("composePoint", (void (mrpt::poses::CPose3D::*)(const struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPoint2D_<double> &) const) &mrpt::poses::CPose3D::composePoint, "This version of the method assumes that the resulting point has no Z\n component (use with caution!) \n\nC++: mrpt::poses::CPose3D::composePoint(const struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPoint2D_<double> &) const --> void", pybind11::arg("local_point"), pybind11::arg("global_point"));
		cl.def("composePoint", (void (mrpt::poses::CPose3D::*)(double, double, double, float &, float &, float &) const) &mrpt::poses::CPose3D::composePoint, "An alternative, slightly more efficient way of doing \n\n with G and L being 3D points and P this 6D pose.  \n\nC++: mrpt::poses::CPose3D::composePoint(double, double, double, float &, float &, float &) const --> void", pybind11::arg("lx"), pybind11::arg("ly"), pybind11::arg("lz"), pybind11::arg("gx"), pybind11::arg("gy"), pybind11::arg("gz"));
		cl.def("rotateVector", (struct mrpt::math::TPoint3D_<double> (mrpt::poses::CPose3D::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::poses::CPose3D::rotateVector, "Rotates a vector (i.e. like composePoint(), but ignoring translation) \n\nC++: mrpt::poses::CPose3D::rotateVector(const struct mrpt::math::TPoint3D_<double> &) const --> struct mrpt::math::TPoint3D_<double>", pybind11::arg("local"));
		cl.def("inverseRotateVector", (struct mrpt::math::TPoint3D_<double> (mrpt::poses::CPose3D::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::poses::CPose3D::inverseRotateVector, "Inverse of rotateVector(), i.e. using the inverse rotation matrix \n\nC++: mrpt::poses::CPose3D::inverseRotateVector(const struct mrpt::math::TPoint3D_<double> &) const --> struct mrpt::math::TPoint3D_<double>", pybind11::arg("global"));
		cl.def("inverseComposePoint", (void (mrpt::poses::CPose3D::*)(const struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPoint3D_<double> &) const) &mrpt::poses::CPose3D::inverseComposePoint, "C++: mrpt::poses::CPose3D::inverseComposePoint(const struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPoint3D_<double> &) const --> void", pybind11::arg("g"), pybind11::arg("l"));
		cl.def("inverseComposePoint", (struct mrpt::math::TPoint3D_<double> (mrpt::poses::CPose3D::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::poses::CPose3D::inverseComposePoint, "C++: mrpt::poses::CPose3D::inverseComposePoint(const struct mrpt::math::TPoint3D_<double> &) const --> struct mrpt::math::TPoint3D_<double>", pybind11::arg("g"));
		cl.def("inverseComposePoint", [](mrpt::poses::CPose3D const &o, const struct mrpt::math::TPoint2D_<double> & a0, struct mrpt::math::TPoint2D_<double> & a1) -> void { return o.inverseComposePoint(a0, a1); }, "", pybind11::arg("g"), pybind11::arg("l"));
		cl.def("inverseComposePoint", (void (mrpt::poses::CPose3D::*)(const struct mrpt::math::TPoint2D_<double> &, struct mrpt::math::TPoint2D_<double> &, const double) const) &mrpt::poses::CPose3D::inverseComposePoint, "overload for 2D points \n If the z component of the result is\n greater than some epsilon \n\nC++: mrpt::poses::CPose3D::inverseComposePoint(const struct mrpt::math::TPoint2D_<double> &, struct mrpt::math::TPoint2D_<double> &, const double) const --> void", pybind11::arg("g"), pybind11::arg("l"), pybind11::arg("eps"));
		cl.def("composeFrom", (void (mrpt::poses::CPose3D::*)(const class mrpt::poses::CPose3D &, const class mrpt::poses::CPose3D &)) &mrpt::poses::CPose3D::composeFrom, "Makes \"this = A (+) B\"; this method is slightly more efficient than\n \"this= A + B;\" since it avoids the temporary object.\n  \n\n A or B can be \"this\" without problems.\n\nC++: mrpt::poses::CPose3D::composeFrom(const class mrpt::poses::CPose3D &, const class mrpt::poses::CPose3D &) --> void", pybind11::arg("A"), pybind11::arg("B"));
		cl.def("__iadd__", (class mrpt::poses::CPose3D & (mrpt::poses::CPose3D::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPose3D::operator+=, "Make \n  ( can be \"this\" without problems)\n\nC++: mrpt::poses::CPose3D::operator+=(const class mrpt::poses::CPose3D &) --> class mrpt::poses::CPose3D &", pybind11::return_value_policy::automatic, pybind11::arg("b"));
		cl.def("inverseComposeFrom", (void (mrpt::poses::CPose3D::*)(const class mrpt::poses::CPose3D &, const class mrpt::poses::CPose3D &)) &mrpt::poses::CPose3D::inverseComposeFrom, "Makes \n this method is slightly more efficient\n than \"this= A - B;\" since it avoids the temporary object.\n  \n\n A or B can be \"this\" without problems.\n \n\n composeFrom, composePoint\n\nC++: mrpt::poses::CPose3D::inverseComposeFrom(const class mrpt::poses::CPose3D &, const class mrpt::poses::CPose3D &) --> void", pybind11::arg("A"), pybind11::arg("B"));
		cl.def("__sub__", (class mrpt::poses::CPose3D (mrpt::poses::CPose3D::*)(const class mrpt::poses::CPose3D &) const) &mrpt::poses::CPose3D::operator-, "Compute \n  \n\nC++: mrpt::poses::CPose3D::operator-(const class mrpt::poses::CPose3D &) const --> class mrpt::poses::CPose3D", pybind11::arg("b"));
		cl.def("inverse", (void (mrpt::poses::CPose3D::*)()) &mrpt::poses::CPose3D::inverse, "Convert this pose into its inverse, saving the result in itself. \n\n operator- \n\nC++: mrpt::poses::CPose3D::inverse() --> void");
		cl.def("changeCoordinatesReference", (void (mrpt::poses::CPose3D::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPose3D::changeCoordinatesReference, "makes: this = p (+) this \n\nC++: mrpt::poses::CPose3D::changeCoordinatesReference(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("p"));
		cl.def("getOppositeScalar", (class mrpt::poses::CPose3D (mrpt::poses::CPose3D::*)() const) &mrpt::poses::CPose3D::getOppositeScalar, "Return the opposite of the current pose instance by taking the negative\n of all its components \n   \n\nC++: mrpt::poses::CPose3D::getOppositeScalar() const --> class mrpt::poses::CPose3D");
		cl.def("addComponents", (void (mrpt::poses::CPose3D::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPose3D::addComponents, "Scalar sum of all 6 components: This is diferent from poses composition,\n which is implemented as \"+\" operators.\n \n\n normalizeAngles\n\nC++: mrpt::poses::CPose3D::addComponents(const class mrpt::poses::CPose3D &) --> void", pybind11::arg("p"));
		cl.def("normalizeAngles", (void (mrpt::poses::CPose3D::*)()) &mrpt::poses::CPose3D::normalizeAngles, "Rebuild the internal matrix & update the yaw/pitch/roll angles within\n the ]-PI,PI] range (Must be called after using addComponents)\n \n\n addComponents\n\nC++: mrpt::poses::CPose3D::normalizeAngles() --> void");
		cl.def("__imul__", (void (mrpt::poses::CPose3D::*)(const double)) &mrpt::poses::CPose3D::operator*=, "Scalar multiplication of x,y,z,yaw,pitch & roll (angles will be wrapped\n to the ]-pi,pi] interval). \n\nC++: mrpt::poses::CPose3D::operator*=(const double) --> void", pybind11::arg("s"));
		cl.def("setFromValues", [](mrpt::poses::CPose3D &o, const double & a0, const double & a1, const double & a2) -> void { return o.setFromValues(a0, a1, a2); }, "", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("z0"));
		cl.def("setFromValues", [](mrpt::poses::CPose3D &o, const double & a0, const double & a1, const double & a2, const double & a3) -> void { return o.setFromValues(a0, a1, a2, a3); }, "", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("z0"), pybind11::arg("yaw"));
		cl.def("setFromValues", [](mrpt::poses::CPose3D &o, const double & a0, const double & a1, const double & a2, const double & a3, const double & a4) -> void { return o.setFromValues(a0, a1, a2, a3, a4); }, "", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("z0"), pybind11::arg("yaw"), pybind11::arg("pitch"));
		cl.def("setFromValues", (void (mrpt::poses::CPose3D::*)(const double, const double, const double, const double, const double, const double)) &mrpt::poses::CPose3D::setFromValues, "Set the pose from a 3D position (meters) and yaw/pitch/roll angles\n (radians) - This method recomputes the internal rotation matrix.\n \n\n getYawPitchRoll, setYawPitchRoll\n\nC++: mrpt::poses::CPose3D::setFromValues(const double, const double, const double, const double, const double, const double) --> void", pybind11::arg("x0"), pybind11::arg("y0"), pybind11::arg("z0"), pybind11::arg("yaw"), pybind11::arg("pitch"), pybind11::arg("roll"));
		cl.def("setYawPitchRoll", (void (mrpt::poses::CPose3D::*)(const double, const double, const double)) &mrpt::poses::CPose3D::setYawPitchRoll, "Set the 3 angles of the 3D pose (in radians) - This method recomputes\n the internal rotation coordinates matrix.\n \n\n getYawPitchRoll, setFromValues\n\nC++: mrpt::poses::CPose3D::setYawPitchRoll(const double, const double, const double) --> void", pybind11::arg("yaw_"), pybind11::arg("pitch_"), pybind11::arg("roll_"));
		cl.def("getYawPitchRoll", (void (mrpt::poses::CPose3D::*)(double &, double &, double &) const) &mrpt::poses::CPose3D::getYawPitchRoll, "Returns the three angles (yaw, pitch, roll), in radians, from the\n rotation matrix.\n \n\n setFromValues, yaw, pitch, roll\n\nC++: mrpt::poses::CPose3D::getYawPitchRoll(double &, double &, double &) const --> void", pybind11::arg("yaw"), pybind11::arg("pitch"), pybind11::arg("roll"));
		cl.def("yaw", (double (mrpt::poses::CPose3D::*)() const) &mrpt::poses::CPose3D::yaw, "Get the YAW angle (in radians)  \n setFromValues \n\nC++: mrpt::poses::CPose3D::yaw() const --> double");
		cl.def("pitch", (double (mrpt::poses::CPose3D::*)() const) &mrpt::poses::CPose3D::pitch, "Get the PITCH angle (in radians) \n setFromValues \n\nC++: mrpt::poses::CPose3D::pitch() const --> double");
		cl.def("roll", (double (mrpt::poses::CPose3D::*)() const) &mrpt::poses::CPose3D::roll, "Get the ROLL angle (in radians) \n setFromValues \n\nC++: mrpt::poses::CPose3D::roll() const --> double");
		cl.def("asVector", (void (mrpt::poses::CPose3D::*)(class mrpt::math::CMatrixFixed<double, 6, 1> &) const) &mrpt::poses::CPose3D::asVector, "Returns a 6x1 vector with [x y z yaw pitch roll]' \n\nC++: mrpt::poses::CPose3D::asVector(class mrpt::math::CMatrixFixed<double, 6, 1> &) const --> void", pybind11::arg("v"));
		cl.def("jacobian_rodrigues_from_YPR", (class mrpt::math::CMatrixFixed<double, 3, 3> (mrpt::poses::CPose3D::*)() const) &mrpt::poses::CPose3D::jacobian_rodrigues_from_YPR, "C++: mrpt::poses::CPose3D::jacobian_rodrigues_from_YPR() const --> class mrpt::math::CMatrixFixed<double, 3, 3>");
		cl.def("jacobian_pose_rodrigues_from_YPR", (class mrpt::math::CMatrixFixed<double, 6, 6> (mrpt::poses::CPose3D::*)() const) &mrpt::poses::CPose3D::jacobian_pose_rodrigues_from_YPR, "C++: mrpt::poses::CPose3D::jacobian_pose_rodrigues_from_YPR() const --> class mrpt::math::CMatrixFixed<double, 6, 6>");
		cl.def("__getitem__", (double (mrpt::poses::CPose3D::*)(unsigned int) const) &mrpt::poses::CPose3D::operator[], "C++: mrpt::poses::CPose3D::operator[](unsigned int) const --> double", pybind11::arg("i"));
		cl.def("asString", (std::string (mrpt::poses::CPose3D::*)() const) &mrpt::poses::CPose3D::asString, "Returns a human-readable textual representation of the object (eg: \"[x y\n z yaw pitch roll]\", angles in degrees.)\n \n\n fromString\n\nC++: mrpt::poses::CPose3D::asString() const --> std::string");
		cl.def("fromString", (void (mrpt::poses::CPose3D::*)(const std::string &)) &mrpt::poses::CPose3D::fromString, "Set the current object value from a string generated by 'asString' (eg:\n \"[x y z yaw pitch roll]\", angles in deg. )\n \n\n asString\n \n\n std::exception On invalid format\n\nC++: mrpt::poses::CPose3D::fromString(const std::string &) --> void", pybind11::arg("s"));
		cl.def("fromStringRaw", (void (mrpt::poses::CPose3D::*)(const std::string &)) &mrpt::poses::CPose3D::fromStringRaw, "Same as fromString, but without requiring the square brackets in the\n string \n\nC++: mrpt::poses::CPose3D::fromStringRaw(const std::string &) --> void", pybind11::arg("s"));
		cl.def_static("FromString", (class mrpt::poses::CPose3D (*)(const std::string &)) &mrpt::poses::CPose3D::FromString, "C++: mrpt::poses::CPose3D::FromString(const std::string &) --> class mrpt::poses::CPose3D", pybind11::arg("s"));
		cl.def("isHorizontal", [](mrpt::poses::CPose3D const &o) -> bool { return o.isHorizontal(); }, "");
		cl.def("isHorizontal", (bool (mrpt::poses::CPose3D::*)(const double) const) &mrpt::poses::CPose3D::isHorizontal, "Return true if the 6D pose represents a Z axis almost exactly vertical\n (upwards or downwards), with a given tolerance (if set to 0 exact\n horizontality is tested). \n\nC++: mrpt::poses::CPose3D::isHorizontal(const double) const --> bool", pybind11::arg("tolerance"));
		cl.def("distanceEuclidean6D", (double (mrpt::poses::CPose3D::*)(const class mrpt::poses::CPose3D &) const) &mrpt::poses::CPose3D::distanceEuclidean6D, "The euclidean distance between two poses taken as two 6-length vectors\n (angles in radians). \n\nC++: mrpt::poses::CPose3D::distanceEuclidean6D(const class mrpt::poses::CPose3D &) const --> double", pybind11::arg("o"));
		cl.def("setToNaN", (void (mrpt::poses::CPose3D::*)()) &mrpt::poses::CPose3D::setToNaN, "@} \n\nC++: mrpt::poses::CPose3D::setToNaN() --> void");
		cl.def_static("is_3D", (bool (*)()) &mrpt::poses::CPose3D::is_3D, "C++: mrpt::poses::CPose3D::is_3D() --> bool");
		cl.def_static("is_PDF", (bool (*)()) &mrpt::poses::CPose3D::is_PDF, "C++: mrpt::poses::CPose3D::is_PDF() --> bool");
		cl.def("getPoseMean", (class mrpt::poses::CPose3D & (mrpt::poses::CPose3D::*)()) &mrpt::poses::CPose3D::getPoseMean, "C++: mrpt::poses::CPose3D::getPoseMean() --> class mrpt::poses::CPose3D &", pybind11::return_value_policy::automatic);
		cl.def_static("size", (unsigned long (*)()) &mrpt::poses::CPose3D::size, "C++: mrpt::poses::CPose3D::size() --> unsigned long");
		cl.def_static("empty", (bool (*)()) &mrpt::poses::CPose3D::empty, "C++: mrpt::poses::CPose3D::empty() --> bool");
		cl.def_static("max_size", (unsigned long (*)()) &mrpt::poses::CPose3D::max_size, "C++: mrpt::poses::CPose3D::max_size() --> unsigned long");
		cl.def_static("resize", (void (*)(size_t)) &mrpt::poses::CPose3D::resize, "C++: mrpt::poses::CPose3D::resize(size_t) --> void", pybind11::arg("n"));
		cl.def("assign", (class mrpt::poses::CPose3D & (mrpt::poses::CPose3D::*)(const class mrpt::poses::CPose3D &)) &mrpt::poses::CPose3D::operator=, "C++: mrpt::poses::CPose3D::operator=(const class mrpt::poses::CPose3D &) --> class mrpt::poses::CPose3D &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::poses::CPose3D const &o) -> std::string { std::ostringstream s; using namespace mrpt::poses; s << o; return s.str(); } );
	}
}
