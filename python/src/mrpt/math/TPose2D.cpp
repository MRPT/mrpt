#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
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

void bind_mrpt_math_TPose2D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::TPose2D file:mrpt/math/TPose2D.h line:25
		pybind11::class_<mrpt::math::TPose2D, std::shared_ptr<mrpt::math::TPose2D>, mrpt::math::TPoseOrPoint> cl(M("mrpt::math"), "TPose2D", "Lightweight 2D pose. Allows coordinate access using [] operator.\n \n\n mrpt::poses::CPose2D\n \n\n\n ");
		cl.def( pybind11::init<const struct mrpt::math::TPoint2D_<double> &>(), pybind11::arg("p") );

		cl.def( pybind11::init<const struct mrpt::math::TPoint3D_<double> &>(), pybind11::arg("p") );

		cl.def( pybind11::init<const struct mrpt::math::TPose3D &>(), pybind11::arg("p") );

		cl.def( pybind11::init<double, double, double>(), pybind11::arg("xx"), pybind11::arg("yy"), pybind11::arg("Phi") );

		cl.def( pybind11::init( [](){ return new mrpt::math::TPose2D(); } ) );
		cl.def( pybind11::init( [](mrpt::math::TPose2D const &o){ return new mrpt::math::TPose2D(o); } ) );
		cl.def_readwrite("x", &mrpt::math::TPose2D::x);
		cl.def_readwrite("y", &mrpt::math::TPose2D::y);
		cl.def_readwrite("phi", &mrpt::math::TPose2D::phi);
		cl.def_static("Identity", (struct mrpt::math::TPose2D (*)()) &mrpt::math::TPose2D::Identity, "Returns the identity transformation \n\nC++: mrpt::math::TPose2D::Identity() --> struct mrpt::math::TPose2D");
		cl.def("__getitem__", (double & (mrpt::math::TPose2D::*)(size_t)) &mrpt::math::TPose2D::operator[], "Coordinate access using operator[]. Order: x,y,phi \n\nC++: mrpt::math::TPose2D::operator[](size_t) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("asString", (void (mrpt::math::TPose2D::*)(std::string &) const) &mrpt::math::TPose2D::asString, "Returns a human-readable textual representation of the object (eg: \"[x y\n yaw]\", yaw in degrees)\n \n\n fromString\n\nC++: mrpt::math::TPose2D::asString(std::string &) const --> void", pybind11::arg("s"));
		cl.def("asString", (std::string (mrpt::math::TPose2D::*)() const) &mrpt::math::TPose2D::asString, "C++: mrpt::math::TPose2D::asString() const --> std::string");
		cl.def("__add__", (struct mrpt::math::TPose2D (mrpt::math::TPose2D::*)(const struct mrpt::math::TPose2D &) const) &mrpt::math::TPose2D::operator+, "Operator \"oplus\" pose composition: \"ret=this \\oplus b\"  \n CPose2D \n\nC++: mrpt::math::TPose2D::operator+(const struct mrpt::math::TPose2D &) const --> struct mrpt::math::TPose2D", pybind11::arg("b"));
		cl.def("__sub__", (struct mrpt::math::TPose2D (mrpt::math::TPose2D::*)(const struct mrpt::math::TPose2D &) const) &mrpt::math::TPose2D::operator-, "Operator \"ominus\" pose composition: \"ret=this \\ominus b\"  \n CPose2D \n\nC++: mrpt::math::TPose2D::operator-(const struct mrpt::math::TPose2D &) const --> struct mrpt::math::TPose2D", pybind11::arg("b"));
		cl.def("composePoint", (struct mrpt::math::TPoint2D_<double> (mrpt::math::TPose2D::*)(const struct mrpt::math::TPoint2D_<double>) const) &mrpt::math::TPose2D::composePoint, "C++: mrpt::math::TPose2D::composePoint(const struct mrpt::math::TPoint2D_<double>) const --> struct mrpt::math::TPoint2D_<double>", pybind11::arg("l"));
		cl.def("__add__", (struct mrpt::math::TPoint2D_<double> (mrpt::math::TPose2D::*)(const struct mrpt::math::TPoint2D_<double> &) const) &mrpt::math::TPose2D::operator+, "C++: mrpt::math::TPose2D::operator+(const struct mrpt::math::TPoint2D_<double> &) const --> struct mrpt::math::TPoint2D_<double>", pybind11::arg("b"));
		cl.def("inverseComposePoint", (struct mrpt::math::TPoint2D_<double> (mrpt::math::TPose2D::*)(const struct mrpt::math::TPoint2D_<double>) const) &mrpt::math::TPose2D::inverseComposePoint, "C++: mrpt::math::TPose2D::inverseComposePoint(const struct mrpt::math::TPoint2D_<double>) const --> struct mrpt::math::TPoint2D_<double>", pybind11::arg("g"));
		cl.def("translation", (struct mrpt::math::TPoint2D_<double> (mrpt::math::TPose2D::*)() const) &mrpt::math::TPose2D::translation, "Returns the (x,y) translational part of the SE(2) transformation. \n\nC++: mrpt::math::TPose2D::translation() const --> struct mrpt::math::TPoint2D_<double>");
		cl.def("norm", (double (mrpt::math::TPose2D::*)() const) &mrpt::math::TPose2D::norm, "Returns the norm of the (x,y) vector (phi is not used) \n\nC++: mrpt::math::TPose2D::norm() const --> double");
		cl.def("normalizePhi", (void (mrpt::math::TPose2D::*)()) &mrpt::math::TPose2D::normalizePhi, "Forces \"phi\" to be in the range [-pi,pi] \n\nC++: mrpt::math::TPose2D::normalizePhi() --> void");
		cl.def("fromString", (void (mrpt::math::TPose2D::*)(const std::string &)) &mrpt::math::TPose2D::fromString, "Set the current object value from a string generated by 'asString' (eg:\n \"[0.02 1.04 -45.0]\" )\n \n\n asString\n \n\n std::exception On invalid format\n\nC++: mrpt::math::TPose2D::fromString(const std::string &) --> void", pybind11::arg("s"));
		cl.def_static("FromString", (struct mrpt::math::TPose2D (*)(const std::string &)) &mrpt::math::TPose2D::FromString, "C++: mrpt::math::TPose2D::FromString(const std::string &) --> struct mrpt::math::TPose2D", pybind11::arg("s"));
		cl.def("assign", (struct mrpt::math::TPose2D & (mrpt::math::TPose2D::*)(const struct mrpt::math::TPose2D &)) &mrpt::math::TPose2D::operator=, "C++: mrpt::math::TPose2D::operator=(const struct mrpt::math::TPose2D &) --> struct mrpt::math::TPose2D &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
