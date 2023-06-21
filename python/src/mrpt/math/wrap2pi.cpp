#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <mrpt/math/wrap2pi.h>
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

void bind_mrpt_math_wrap2pi(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::math::wrapTo2PiInPlace(double &) file:mrpt/math/wrap2pi.h line:25
	M("mrpt::math").def("wrapTo2PiInPlace", (void (*)(double &)) &mrpt::math::wrapTo2PiInPlace<double>, "C++: mrpt::math::wrapTo2PiInPlace(double &) --> void", pybind11::arg("a"));

	// mrpt::math::wrapTo2Pi(double) file:mrpt/math/wrap2pi.h line:38
	M("mrpt::math").def("wrapTo2Pi", (double (*)(double)) &mrpt::math::wrapTo2Pi<double>, "C++: mrpt::math::wrapTo2Pi(double) --> double", pybind11::arg("a"));

	// mrpt::math::wrapToPi(double) file:mrpt/math/wrap2pi.h line:50
	M("mrpt::math").def("wrapToPi", (double (*)(double)) &mrpt::math::wrapToPi<double>, "C++: mrpt::math::wrapToPi(double) --> double", pybind11::arg("a"));

	// mrpt::math::wrapToPiInPlace(double &) file:mrpt/math/wrap2pi.h line:61
	M("mrpt::math").def("wrapToPiInPlace", (void (*)(double &)) &mrpt::math::wrapToPiInPlace<double>, "C++: mrpt::math::wrapToPiInPlace(double &) --> void", pybind11::arg("a"));

	// mrpt::math::angDistance(double, double) file:mrpt/math/wrap2pi.h line:95
	M("mrpt::math").def("angDistance", (double (*)(double, double)) &mrpt::math::angDistance<double>, "C++: mrpt::math::angDistance(double, double) --> double", pybind11::arg("from"), pybind11::arg("to"));

	{ // mrpt::math::TPose3D file:mrpt/math/TPose3D.h line:26
		pybind11::class_<mrpt::math::TPose3D, std::shared_ptr<mrpt::math::TPose3D>, mrpt::math::TPoseOrPoint> cl(M("mrpt::math"), "TPose3D", "Lightweight 3D pose (three spatial coordinates, plus three angular\n coordinates). Allows coordinate access using [] operator.\n \n\n mrpt::poses::CPose3D\n \n\n\n ");
		cl.def( pybind11::init<const struct mrpt::math::TPoint2D_<double> &>(), pybind11::arg("p") );

		cl.def( pybind11::init<const struct mrpt::math::TPose2D &>(), pybind11::arg("p") );

		cl.def( pybind11::init<const struct mrpt::math::TPoint3D_<double> &>(), pybind11::arg("p") );

		cl.def( pybind11::init<double, double, double, double, double, double>(), pybind11::arg("_x"), pybind11::arg("_y"), pybind11::arg("_z"), pybind11::arg("_yaw"), pybind11::arg("_pitch"), pybind11::arg("_roll") );

		cl.def( pybind11::init( [](){ return new mrpt::math::TPose3D(); } ) );
		cl.def( pybind11::init( [](mrpt::math::TPose3D const &o){ return new mrpt::math::TPose3D(o); } ) );
		cl.def_readwrite("x", &mrpt::math::TPose3D::x);
		cl.def_readwrite("y", &mrpt::math::TPose3D::y);
		cl.def_readwrite("z", &mrpt::math::TPose3D::z);
		cl.def_readwrite("yaw", &mrpt::math::TPose3D::yaw);
		cl.def_readwrite("pitch", &mrpt::math::TPose3D::pitch);
		cl.def_readwrite("roll", &mrpt::math::TPose3D::roll);
		cl.def_static("Identity", (struct mrpt::math::TPose3D (*)()) &mrpt::math::TPose3D::Identity, "Returns the identity transformation, T=eye(4) \n\nC++: mrpt::math::TPose3D::Identity() --> struct mrpt::math::TPose3D");
		cl.def_static("FromString", (struct mrpt::math::TPose3D (*)(const std::string &)) &mrpt::math::TPose3D::FromString, "See fromString() for a description of the expected string format. \n\nC++: mrpt::math::TPose3D::FromString(const std::string &) --> struct mrpt::math::TPose3D", pybind11::arg("s"));
		cl.def("__getitem__", (double & (mrpt::math::TPose3D::*)(size_t)) &mrpt::math::TPose3D::operator[], "Coordinate access using operator[]. Order: x,y,z,yaw,pitch,roll \n\nC++: mrpt::math::TPose3D::operator[](size_t) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("translation", (struct mrpt::math::TPoint3D_<double> (mrpt::math::TPose3D::*)() const) &mrpt::math::TPose3D::translation, "Returns the (x,y,z) translational part of the SE(3) transformation. \n\nC++: mrpt::math::TPose3D::translation() const --> struct mrpt::math::TPoint3D_<double>");
		cl.def("norm", (double (mrpt::math::TPose3D::*)() const) &mrpt::math::TPose3D::norm, "Pose's spatial coordinates norm.\n\nC++: mrpt::math::TPose3D::norm() const --> double");
		cl.def("asString", (void (mrpt::math::TPose3D::*)(std::string &) const) &mrpt::math::TPose3D::asString, "Returns a human-readable textual representation of the object (eg: \"[x y\n z yaw pitch roll]\", angles in degrees.)\n \n\n fromString\n\nC++: mrpt::math::TPose3D::asString(std::string &) const --> void", pybind11::arg("s"));
		cl.def("asString", (std::string (mrpt::math::TPose3D::*)() const) &mrpt::math::TPose3D::asString, "C++: mrpt::math::TPose3D::asString() const --> std::string");
		cl.def("composePoint", (void (mrpt::math::TPose3D::*)(const struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TPose3D::composePoint, "C++: mrpt::math::TPose3D::composePoint(const struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPoint3D_<double> &) const --> void", pybind11::arg("l"), pybind11::arg("g"));
		cl.def("composePoint", (struct mrpt::math::TPoint3D_<double> (mrpt::math::TPose3D::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TPose3D::composePoint, "C++: mrpt::math::TPose3D::composePoint(const struct mrpt::math::TPoint3D_<double> &) const --> struct mrpt::math::TPoint3D_<double>", pybind11::arg("l"));
		cl.def("inverseComposePoint", (void (mrpt::math::TPose3D::*)(const struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TPose3D::inverseComposePoint, "C++: mrpt::math::TPose3D::inverseComposePoint(const struct mrpt::math::TPoint3D_<double> &, struct mrpt::math::TPoint3D_<double> &) const --> void", pybind11::arg("g"), pybind11::arg("l"));
		cl.def("inverseComposePoint", (struct mrpt::math::TPoint3D_<double> (mrpt::math::TPose3D::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TPose3D::inverseComposePoint, "C++: mrpt::math::TPose3D::inverseComposePoint(const struct mrpt::math::TPoint3D_<double> &) const --> struct mrpt::math::TPoint3D_<double>", pybind11::arg("g"));
		cl.def("composePose", (void (mrpt::math::TPose3D::*)(const struct mrpt::math::TPose3D, struct mrpt::math::TPose3D &) const) &mrpt::math::TPose3D::composePose, "C++: mrpt::math::TPose3D::composePose(const struct mrpt::math::TPose3D, struct mrpt::math::TPose3D &) const --> void", pybind11::arg("other"), pybind11::arg("result"));
		cl.def("__add__", (struct mrpt::math::TPose3D (mrpt::math::TPose3D::*)(const struct mrpt::math::TPose3D &) const) &mrpt::math::TPose3D::operator+, "Operator \"oplus\" pose composition: \"ret=this \\oplus b\"  \n CPose3D\n \n\n [Added in MRPT 2.1.5] \n\nC++: mrpt::math::TPose3D::operator+(const struct mrpt::math::TPose3D &) const --> struct mrpt::math::TPose3D", pybind11::arg("b"));
		cl.def("getRotationMatrix", (void (mrpt::math::TPose3D::*)(class mrpt::math::CMatrixFixed<double, 3, 3> &) const) &mrpt::math::TPose3D::getRotationMatrix, "C++: mrpt::math::TPose3D::getRotationMatrix(class mrpt::math::CMatrixFixed<double, 3, 3> &) const --> void", pybind11::arg("R"));
		cl.def("getRotationMatrix", (class mrpt::math::CMatrixFixed<double, 3, 3> (mrpt::math::TPose3D::*)() const) &mrpt::math::TPose3D::getRotationMatrix, "C++: mrpt::math::TPose3D::getRotationMatrix() const --> class mrpt::math::CMatrixFixed<double, 3, 3>");
		cl.def("getHomogeneousMatrix", (void (mrpt::math::TPose3D::*)(class mrpt::math::CMatrixFixed<double, 4, 4> &) const) &mrpt::math::TPose3D::getHomogeneousMatrix, "C++: mrpt::math::TPose3D::getHomogeneousMatrix(class mrpt::math::CMatrixFixed<double, 4, 4> &) const --> void", pybind11::arg("HG"));
		cl.def("getHomogeneousMatrix", (class mrpt::math::CMatrixFixed<double, 4, 4> (mrpt::math::TPose3D::*)() const) &mrpt::math::TPose3D::getHomogeneousMatrix, "C++: mrpt::math::TPose3D::getHomogeneousMatrix() const --> class mrpt::math::CMatrixFixed<double, 4, 4>");
		cl.def("getInverseHomogeneousMatrix", (void (mrpt::math::TPose3D::*)(class mrpt::math::CMatrixFixed<double, 4, 4> &) const) &mrpt::math::TPose3D::getInverseHomogeneousMatrix, "C++: mrpt::math::TPose3D::getInverseHomogeneousMatrix(class mrpt::math::CMatrixFixed<double, 4, 4> &) const --> void", pybind11::arg("HG"));
		cl.def("getInverseHomogeneousMatrix", (class mrpt::math::CMatrixFixed<double, 4, 4> (mrpt::math::TPose3D::*)() const) &mrpt::math::TPose3D::getInverseHomogeneousMatrix, "C++: mrpt::math::TPose3D::getInverseHomogeneousMatrix() const --> class mrpt::math::CMatrixFixed<double, 4, 4>");
		cl.def("fromHomogeneousMatrix", (void (mrpt::math::TPose3D::*)(const class mrpt::math::CMatrixFixed<double, 4, 4> &)) &mrpt::math::TPose3D::fromHomogeneousMatrix, "C++: mrpt::math::TPose3D::fromHomogeneousMatrix(const class mrpt::math::CMatrixFixed<double, 4, 4> &) --> void", pybind11::arg("HG"));
		cl.def_static("SO3_to_yaw_pitch_roll", (void (*)(const class mrpt::math::CMatrixFixed<double, 3, 3> &, double &, double &, double &)) &mrpt::math::TPose3D::SO3_to_yaw_pitch_roll, "C++: mrpt::math::TPose3D::SO3_to_yaw_pitch_roll(const class mrpt::math::CMatrixFixed<double, 3, 3> &, double &, double &, double &) --> void", pybind11::arg("R"), pybind11::arg("yaw"), pybind11::arg("pitch"), pybind11::arg("roll"));
		cl.def("fromString", (void (mrpt::math::TPose3D::*)(const std::string &)) &mrpt::math::TPose3D::fromString, "Set the current object value from a string generated by 'asString' (eg:\n \"[x y z yaw pitch roll]\" with the three angles given in degrees. )\n \n\n asString\n \n\n std::exception On invalid format\n\nC++: mrpt::math::TPose3D::fromString(const std::string &) --> void", pybind11::arg("s"));
		cl.def("assign", (struct mrpt::math::TPose3D & (mrpt::math::TPose3D::*)(const struct mrpt::math::TPose3D &)) &mrpt::math::TPose3D::operator=, "C++: mrpt::math::TPose3D::operator=(const struct mrpt::math::TPose3D &) --> struct mrpt::math::TPose3D &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
