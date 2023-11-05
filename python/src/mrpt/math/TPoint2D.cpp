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

void bind_mrpt_math_TPoint2D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::TPoint2D_ file:mrpt/math/TPoint2D.h line:38
		pybind11::class_<mrpt::math::TPoint2D_<double>, std::shared_ptr<mrpt::math::TPoint2D_<double>>, mrpt::math::TPoseOrPoint, mrpt::math::TPoint2D_data<double>> cl(M("mrpt::math"), "TPoint2D_double_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::TPoint2D_<double>(); } ) );
		cl.def( pybind11::init<double, double>(), pybind11::arg("xx"), pybind11::arg("yy") );

		cl.def( pybind11::init<const struct mrpt::math::TPose2D &>(), pybind11::arg("p") );

		cl.def( pybind11::init<const struct mrpt::math::TPoint3D_<double> &>(), pybind11::arg("p") );

		cl.def( pybind11::init<const struct mrpt::math::TPose3D &>(), pybind11::arg("p") );

		cl.def( pybind11::init( [](mrpt::math::TPoint2D_<double> const &o){ return new mrpt::math::TPoint2D_<double>(o); } ) );
		cl.def("__getitem__", (double & (mrpt::math::TPoint2D_<double>::*)(size_t)) &mrpt::math::TPoint2D_<double>::operator[], "C++: mrpt::math::TPoint2D_<double>::operator[](size_t) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("__iadd__", (struct mrpt::math::TPoint2D_<double> & (mrpt::math::TPoint2D_<double>::*)(const struct mrpt::math::TPoint2D_<double> &)) &mrpt::math::TPoint2D_<double>::operator+=, "C++: mrpt::math::TPoint2D_<double>::operator+=(const struct mrpt::math::TPoint2D_<double> &) --> struct mrpt::math::TPoint2D_<double> &", pybind11::return_value_policy::automatic, pybind11::arg("p"));
		cl.def("__isub__", (struct mrpt::math::TPoint2D_<double> & (mrpt::math::TPoint2D_<double>::*)(const struct mrpt::math::TPoint2D_<double> &)) &mrpt::math::TPoint2D_<double>::operator-=, "C++: mrpt::math::TPoint2D_<double>::operator-=(const struct mrpt::math::TPoint2D_<double> &) --> struct mrpt::math::TPoint2D_<double> &", pybind11::return_value_policy::automatic, pybind11::arg("p"));
		cl.def("__imul__", (struct mrpt::math::TPoint2D_<double> & (mrpt::math::TPoint2D_<double>::*)(double)) &mrpt::math::TPoint2D_<double>::operator*=, "C++: mrpt::math::TPoint2D_<double>::operator*=(double) --> struct mrpt::math::TPoint2D_<double> &", pybind11::return_value_policy::automatic, pybind11::arg("d"));
		cl.def("__itruediv__", (struct mrpt::math::TPoint2D_<double> & (mrpt::math::TPoint2D_<double>::*)(double)) &mrpt::math::TPoint2D_<double>::operator/=, "C++: mrpt::math::TPoint2D_<double>::operator/=(double) --> struct mrpt::math::TPoint2D_<double> &", pybind11::return_value_policy::automatic, pybind11::arg("d"));
		cl.def("__add__", (struct mrpt::math::TPoint2D_<double> (mrpt::math::TPoint2D_<double>::*)(const struct mrpt::math::TPoint2D_<double> &) const) &mrpt::math::TPoint2D_<double>::operator+, "C++: mrpt::math::TPoint2D_<double>::operator+(const struct mrpt::math::TPoint2D_<double> &) const --> struct mrpt::math::TPoint2D_<double>", pybind11::arg("p"));
		cl.def("__sub__", (struct mrpt::math::TPoint2D_<double> (mrpt::math::TPoint2D_<double>::*)(const struct mrpt::math::TPoint2D_<double> &) const) &mrpt::math::TPoint2D_<double>::operator-, "C++: mrpt::math::TPoint2D_<double>::operator-(const struct mrpt::math::TPoint2D_<double> &) const --> struct mrpt::math::TPoint2D_<double>", pybind11::arg("p"));
		cl.def("__mul__", (struct mrpt::math::TPoint2D_<double> (mrpt::math::TPoint2D_<double>::*)(double) const) &mrpt::math::TPoint2D_<double>::operator*, "C++: mrpt::math::TPoint2D_<double>::operator*(double) const --> struct mrpt::math::TPoint2D_<double>", pybind11::arg("d"));
		cl.def("__truediv__", (struct mrpt::math::TPoint2D_<double> (mrpt::math::TPoint2D_<double>::*)(double) const) &mrpt::math::TPoint2D_<double>::operator/, "C++: mrpt::math::TPoint2D_<double>::operator/(double) const --> struct mrpt::math::TPoint2D_<double>", pybind11::arg("d"));
		cl.def("asString", (void (mrpt::math::TPoint2D_<double>::*)(std::string &) const) &mrpt::math::TPoint2D_<double>::asString, "C++: mrpt::math::TPoint2D_<double>::asString(std::string &) const --> void", pybind11::arg("s"));
		cl.def("asString", (std::string (mrpt::math::TPoint2D_<double>::*)() const) &mrpt::math::TPoint2D_<double>::asString, "C++: mrpt::math::TPoint2D_<double>::asString() const --> std::string");
		cl.def("fromString", (void (mrpt::math::TPoint2D_<double>::*)(const std::string &)) &mrpt::math::TPoint2D_<double>::fromString, "C++: mrpt::math::TPoint2D_<double>::fromString(const std::string &) --> void", pybind11::arg("s"));
		cl.def_static("FromString", (struct mrpt::math::TPoint2D_<double> (*)(const std::string &)) &mrpt::math::TPoint2D_<double>::FromString, "C++: mrpt::math::TPoint2D_<double>::FromString(const std::string &) --> struct mrpt::math::TPoint2D_<double>", pybind11::arg("s"));
		cl.def("sqrNorm", (double (mrpt::math::TPoint2D_<double>::*)() const) &mrpt::math::TPoint2D_<double>::sqrNorm, "C++: mrpt::math::TPoint2D_<double>::sqrNorm() const --> double");
		cl.def("norm", (double (mrpt::math::TPoint2D_<double>::*)() const) &mrpt::math::TPoint2D_<double>::norm, "C++: mrpt::math::TPoint2D_<double>::norm() const --> double");
		cl.def("unitarize", (struct mrpt::math::TPoint2D_<double> (mrpt::math::TPoint2D_<double>::*)() const) &mrpt::math::TPoint2D_<double>::unitarize, "C++: mrpt::math::TPoint2D_<double>::unitarize() const --> struct mrpt::math::TPoint2D_<double>");
		cl.def("assign", (struct mrpt::math::TPoint2D_<double> & (mrpt::math::TPoint2D_<double>::*)(const struct mrpt::math::TPoint2D_<double> &)) &mrpt::math::TPoint2D_<double>::operator=, "C++: mrpt::math::TPoint2D_<double>::operator=(const struct mrpt::math::TPoint2D_<double> &) --> struct mrpt::math::TPoint2D_<double> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("assign", (struct mrpt::math::TPoseOrPoint & (mrpt::math::TPoseOrPoint::*)(const struct mrpt::math::TPoseOrPoint &)) &mrpt::math::TPoseOrPoint::operator=, "C++: mrpt::math::TPoseOrPoint::operator=(const struct mrpt::math::TPoseOrPoint &) --> struct mrpt::math::TPoseOrPoint &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def_readwrite("x", &mrpt::math::TPoint2D_data<double>::x);
		cl.def_readwrite("y", &mrpt::math::TPoint2D_data<double>::y);
		cl.def("assign", (struct mrpt::math::TPoint2D_data<double> & (mrpt::math::TPoint2D_data<double>::*)(const struct mrpt::math::TPoint2D_data<double> &)) &mrpt::math::TPoint2D_data<double>::operator=, "C++: mrpt::math::TPoint2D_data<double>::operator=(const struct mrpt::math::TPoint2D_data<double> &) --> struct mrpt::math::TPoint2D_data<double> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::math::TPoint2D_ file:mrpt/math/TPoint2D.h line:38
		pybind11::class_<mrpt::math::TPoint2D_<float>, std::shared_ptr<mrpt::math::TPoint2D_<float>>, mrpt::math::TPoseOrPoint, mrpt::math::TPoint2D_data<float>> cl(M("mrpt::math"), "TPoint2D_float_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::TPoint2D_<float>(); } ) );
		cl.def( pybind11::init<float, float>(), pybind11::arg("xx"), pybind11::arg("yy") );

		cl.def( pybind11::init<const struct mrpt::math::TPose2D &>(), pybind11::arg("p") );

		cl.def( pybind11::init<const struct mrpt::math::TPoint3D_<float> &>(), pybind11::arg("p") );

		cl.def( pybind11::init<const struct mrpt::math::TPose3D &>(), pybind11::arg("p") );

		cl.def( pybind11::init( [](mrpt::math::TPoint2D_<float> const &o){ return new mrpt::math::TPoint2D_<float>(o); } ) );
		cl.def("__getitem__", (float & (mrpt::math::TPoint2D_<float>::*)(size_t)) &mrpt::math::TPoint2D_<float>::operator[], "C++: mrpt::math::TPoint2D_<float>::operator[](size_t) --> float &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("__iadd__", (struct mrpt::math::TPoint2D_<float> & (mrpt::math::TPoint2D_<float>::*)(const struct mrpt::math::TPoint2D_<float> &)) &mrpt::math::TPoint2D_<float>::operator+=, "C++: mrpt::math::TPoint2D_<float>::operator+=(const struct mrpt::math::TPoint2D_<float> &) --> struct mrpt::math::TPoint2D_<float> &", pybind11::return_value_policy::automatic, pybind11::arg("p"));
		cl.def("__isub__", (struct mrpt::math::TPoint2D_<float> & (mrpt::math::TPoint2D_<float>::*)(const struct mrpt::math::TPoint2D_<float> &)) &mrpt::math::TPoint2D_<float>::operator-=, "C++: mrpt::math::TPoint2D_<float>::operator-=(const struct mrpt::math::TPoint2D_<float> &) --> struct mrpt::math::TPoint2D_<float> &", pybind11::return_value_policy::automatic, pybind11::arg("p"));
		cl.def("__imul__", (struct mrpt::math::TPoint2D_<float> & (mrpt::math::TPoint2D_<float>::*)(float)) &mrpt::math::TPoint2D_<float>::operator*=, "C++: mrpt::math::TPoint2D_<float>::operator*=(float) --> struct mrpt::math::TPoint2D_<float> &", pybind11::return_value_policy::automatic, pybind11::arg("d"));
		cl.def("__itruediv__", (struct mrpt::math::TPoint2D_<float> & (mrpt::math::TPoint2D_<float>::*)(float)) &mrpt::math::TPoint2D_<float>::operator/=, "C++: mrpt::math::TPoint2D_<float>::operator/=(float) --> struct mrpt::math::TPoint2D_<float> &", pybind11::return_value_policy::automatic, pybind11::arg("d"));
		cl.def("__add__", (struct mrpt::math::TPoint2D_<float> (mrpt::math::TPoint2D_<float>::*)(const struct mrpt::math::TPoint2D_<float> &) const) &mrpt::math::TPoint2D_<float>::operator+, "C++: mrpt::math::TPoint2D_<float>::operator+(const struct mrpt::math::TPoint2D_<float> &) const --> struct mrpt::math::TPoint2D_<float>", pybind11::arg("p"));
		cl.def("__sub__", (struct mrpt::math::TPoint2D_<float> (mrpt::math::TPoint2D_<float>::*)(const struct mrpt::math::TPoint2D_<float> &) const) &mrpt::math::TPoint2D_<float>::operator-, "C++: mrpt::math::TPoint2D_<float>::operator-(const struct mrpt::math::TPoint2D_<float> &) const --> struct mrpt::math::TPoint2D_<float>", pybind11::arg("p"));
		cl.def("__mul__", (struct mrpt::math::TPoint2D_<float> (mrpt::math::TPoint2D_<float>::*)(float) const) &mrpt::math::TPoint2D_<float>::operator*, "C++: mrpt::math::TPoint2D_<float>::operator*(float) const --> struct mrpt::math::TPoint2D_<float>", pybind11::arg("d"));
		cl.def("__truediv__", (struct mrpt::math::TPoint2D_<float> (mrpt::math::TPoint2D_<float>::*)(float) const) &mrpt::math::TPoint2D_<float>::operator/, "C++: mrpt::math::TPoint2D_<float>::operator/(float) const --> struct mrpt::math::TPoint2D_<float>", pybind11::arg("d"));
		cl.def("asString", (void (mrpt::math::TPoint2D_<float>::*)(std::string &) const) &mrpt::math::TPoint2D_<float>::asString, "C++: mrpt::math::TPoint2D_<float>::asString(std::string &) const --> void", pybind11::arg("s"));
		cl.def("asString", (std::string (mrpt::math::TPoint2D_<float>::*)() const) &mrpt::math::TPoint2D_<float>::asString, "C++: mrpt::math::TPoint2D_<float>::asString() const --> std::string");
		cl.def("fromString", (void (mrpt::math::TPoint2D_<float>::*)(const std::string &)) &mrpt::math::TPoint2D_<float>::fromString, "C++: mrpt::math::TPoint2D_<float>::fromString(const std::string &) --> void", pybind11::arg("s"));
		cl.def_static("FromString", (struct mrpt::math::TPoint2D_<float> (*)(const std::string &)) &mrpt::math::TPoint2D_<float>::FromString, "C++: mrpt::math::TPoint2D_<float>::FromString(const std::string &) --> struct mrpt::math::TPoint2D_<float>", pybind11::arg("s"));
		cl.def("sqrNorm", (float (mrpt::math::TPoint2D_<float>::*)() const) &mrpt::math::TPoint2D_<float>::sqrNorm, "C++: mrpt::math::TPoint2D_<float>::sqrNorm() const --> float");
		cl.def("norm", (float (mrpt::math::TPoint2D_<float>::*)() const) &mrpt::math::TPoint2D_<float>::norm, "C++: mrpt::math::TPoint2D_<float>::norm() const --> float");
		cl.def("unitarize", (struct mrpt::math::TPoint2D_<float> (mrpt::math::TPoint2D_<float>::*)() const) &mrpt::math::TPoint2D_<float>::unitarize, "C++: mrpt::math::TPoint2D_<float>::unitarize() const --> struct mrpt::math::TPoint2D_<float>");
		cl.def("assign", (struct mrpt::math::TPoint2D_<float> & (mrpt::math::TPoint2D_<float>::*)(const struct mrpt::math::TPoint2D_<float> &)) &mrpt::math::TPoint2D_<float>::operator=, "C++: mrpt::math::TPoint2D_<float>::operator=(const struct mrpt::math::TPoint2D_<float> &) --> struct mrpt::math::TPoint2D_<float> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("assign", (struct mrpt::math::TPoseOrPoint & (mrpt::math::TPoseOrPoint::*)(const struct mrpt::math::TPoseOrPoint &)) &mrpt::math::TPoseOrPoint::operator=, "C++: mrpt::math::TPoseOrPoint::operator=(const struct mrpt::math::TPoseOrPoint &) --> struct mrpt::math::TPoseOrPoint &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def_readwrite("x", &mrpt::math::TPoint2D_data<float>::x);
		cl.def_readwrite("y", &mrpt::math::TPoint2D_data<float>::y);
		cl.def("assign", (struct mrpt::math::TPoint2D_data<float> & (mrpt::math::TPoint2D_data<float>::*)(const struct mrpt::math::TPoint2D_data<float> &)) &mrpt::math::TPoint2D_data<float>::operator=, "C++: mrpt::math::TPoint2D_data<float>::operator=(const struct mrpt::math::TPoint2D_data<float> &) --> struct mrpt::math::TPoint2D_data<float> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::math::TPoint3D_ file:mrpt/math/TPoint3D.h line:45
		pybind11::class_<mrpt::math::TPoint3D_<double>, std::shared_ptr<mrpt::math::TPoint3D_<double>>, mrpt::math::TPoseOrPoint, mrpt::math::TPoint3D_data<double>> cl(M("mrpt::math"), "TPoint3D_double_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::TPoint3D_<double>(); } ) );
		cl.def( pybind11::init<double, double, double>(), pybind11::arg("xx"), pybind11::arg("yy"), pybind11::arg("zz") );

		cl.def( pybind11::init<const struct mrpt::math::TPoint2D_<double> &>(), pybind11::arg("p") );

		cl.def( pybind11::init<const struct mrpt::math::TPose2D &>(), pybind11::arg("p") );

		cl.def( pybind11::init<const struct mrpt::math::TPose3D &>(), pybind11::arg("p") );

		cl.def( pybind11::init( [](mrpt::math::TPoint3D_<double> const &o){ return new mrpt::math::TPoint3D_<double>(o); } ) );
		cl.def("cast", (struct mrpt::math::TPoint3D_<float> (mrpt::math::TPoint3D_<double>::*)() const) &mrpt::math::TPoint3D_<double>::cast<float>, "C++: mrpt::math::TPoint3D_<double>::cast() const --> struct mrpt::math::TPoint3D_<float>");
		cl.def("__getitem__", (double & (mrpt::math::TPoint3D_<double>::*)(size_t)) &mrpt::math::TPoint3D_<double>::operator[], "C++: mrpt::math::TPoint3D_<double>::operator[](size_t) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("distanceTo", (double (mrpt::math::TPoint3D_<double>::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TPoint3D_<double>::distanceTo, "C++: mrpt::math::TPoint3D_<double>::distanceTo(const struct mrpt::math::TPoint3D_<double> &) const --> double", pybind11::arg("p"));
		cl.def("sqrDistanceTo", (double (mrpt::math::TPoint3D_<double>::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TPoint3D_<double>::sqrDistanceTo, "C++: mrpt::math::TPoint3D_<double>::sqrDistanceTo(const struct mrpt::math::TPoint3D_<double> &) const --> double", pybind11::arg("p"));
		cl.def("sqrNorm", (double (mrpt::math::TPoint3D_<double>::*)() const) &mrpt::math::TPoint3D_<double>::sqrNorm, "C++: mrpt::math::TPoint3D_<double>::sqrNorm() const --> double");
		cl.def("norm", (double (mrpt::math::TPoint3D_<double>::*)() const) &mrpt::math::TPoint3D_<double>::norm, "C++: mrpt::math::TPoint3D_<double>::norm() const --> double");
		cl.def("unitarize", (struct mrpt::math::TPoint3D_<double> (mrpt::math::TPoint3D_<double>::*)() const) &mrpt::math::TPoint3D_<double>::unitarize, "C++: mrpt::math::TPoint3D_<double>::unitarize() const --> struct mrpt::math::TPoint3D_<double>");
		cl.def("__imul__", (struct mrpt::math::TPoint3D_<double> & (mrpt::math::TPoint3D_<double>::*)(const double)) &mrpt::math::TPoint3D_<double>::operator*=, "C++: mrpt::math::TPoint3D_<double>::operator*=(const double) --> struct mrpt::math::TPoint3D_<double> &", pybind11::return_value_policy::automatic, pybind11::arg("f"));
		cl.def("__iadd__", (struct mrpt::math::TPoint3D_<double> & (mrpt::math::TPoint3D_<double>::*)(const struct mrpt::math::TPoint3D_<double> &)) &mrpt::math::TPoint3D_<double>::operator+=, "C++: mrpt::math::TPoint3D_<double>::operator+=(const struct mrpt::math::TPoint3D_<double> &) --> struct mrpt::math::TPoint3D_<double> &", pybind11::return_value_policy::automatic, pybind11::arg("p"));
		cl.def("__isub__", (struct mrpt::math::TPoint3D_<double> & (mrpt::math::TPoint3D_<double>::*)(const struct mrpt::math::TPoint3D_<double> &)) &mrpt::math::TPoint3D_<double>::operator-=, "C++: mrpt::math::TPoint3D_<double>::operator-=(const struct mrpt::math::TPoint3D_<double> &) --> struct mrpt::math::TPoint3D_<double> &", pybind11::return_value_policy::automatic, pybind11::arg("p"));
		cl.def("__add__", (struct mrpt::math::TPoint3D_<double> (mrpt::math::TPoint3D_<double>::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TPoint3D_<double>::operator+, "C++: mrpt::math::TPoint3D_<double>::operator+(const struct mrpt::math::TPoint3D_<double> &) const --> struct mrpt::math::TPoint3D_<double>", pybind11::arg("p"));
		cl.def("__sub__", (struct mrpt::math::TPoint3D_<double> (mrpt::math::TPoint3D_<double>::*)(const struct mrpt::math::TPoint3D_<double> &) const) &mrpt::math::TPoint3D_<double>::operator-, "C++: mrpt::math::TPoint3D_<double>::operator-(const struct mrpt::math::TPoint3D_<double> &) const --> struct mrpt::math::TPoint3D_<double>", pybind11::arg("p"));
		cl.def("__mul__", (struct mrpt::math::TPoint3D_<double> (mrpt::math::TPoint3D_<double>::*)(double) const) &mrpt::math::TPoint3D_<double>::operator*, "C++: mrpt::math::TPoint3D_<double>::operator*(double) const --> struct mrpt::math::TPoint3D_<double>", pybind11::arg("d"));
		cl.def("__truediv__", (struct mrpt::math::TPoint3D_<double> (mrpt::math::TPoint3D_<double>::*)(double) const) &mrpt::math::TPoint3D_<double>::operator/, "C++: mrpt::math::TPoint3D_<double>::operator/(double) const --> struct mrpt::math::TPoint3D_<double>", pybind11::arg("d"));
		cl.def("asString", (void (mrpt::math::TPoint3D_<double>::*)(std::string &) const) &mrpt::math::TPoint3D_<double>::asString, "C++: mrpt::math::TPoint3D_<double>::asString(std::string &) const --> void", pybind11::arg("s"));
		cl.def("asString", (std::string (mrpt::math::TPoint3D_<double>::*)() const) &mrpt::math::TPoint3D_<double>::asString, "C++: mrpt::math::TPoint3D_<double>::asString() const --> std::string");
		cl.def("fromString", (void (mrpt::math::TPoint3D_<double>::*)(const std::string &)) &mrpt::math::TPoint3D_<double>::fromString, "C++: mrpt::math::TPoint3D_<double>::fromString(const std::string &) --> void", pybind11::arg("s"));
		cl.def_static("FromString", (struct mrpt::math::TPoint3D_<double> (*)(const std::string &)) &mrpt::math::TPoint3D_<double>::FromString, "C++: mrpt::math::TPoint3D_<double>::FromString(const std::string &) --> struct mrpt::math::TPoint3D_<double>", pybind11::arg("s"));
		cl.def("assign", (struct mrpt::math::TPoint3D_<double> & (mrpt::math::TPoint3D_<double>::*)(const struct mrpt::math::TPoint3D_<double> &)) &mrpt::math::TPoint3D_<double>::operator=, "C++: mrpt::math::TPoint3D_<double>::operator=(const struct mrpt::math::TPoint3D_<double> &) --> struct mrpt::math::TPoint3D_<double> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("assign", (struct mrpt::math::TPoseOrPoint & (mrpt::math::TPoseOrPoint::*)(const struct mrpt::math::TPoseOrPoint &)) &mrpt::math::TPoseOrPoint::operator=, "C++: mrpt::math::TPoseOrPoint::operator=(const struct mrpt::math::TPoseOrPoint &) --> struct mrpt::math::TPoseOrPoint &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def_readwrite("x", &mrpt::math::TPoint3D_data<double>::x);
		cl.def_readwrite("y", &mrpt::math::TPoint3D_data<double>::y);
		cl.def_readwrite("z", &mrpt::math::TPoint3D_data<double>::z);
		cl.def("assign", (struct mrpt::math::TPoint3D_data<double> & (mrpt::math::TPoint3D_data<double>::*)(const struct mrpt::math::TPoint3D_data<double> &)) &mrpt::math::TPoint3D_data<double>::operator=, "C++: mrpt::math::TPoint3D_data<double>::operator=(const struct mrpt::math::TPoint3D_data<double> &) --> struct mrpt::math::TPoint3D_data<double> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
