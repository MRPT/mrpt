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

void bind_mrpt_math_TPoint3D_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::TPoint3D_ file:mrpt/math/TPoint3D.h line:45
		pybind11::class_<mrpt::math::TPoint3D_<float>, std::shared_ptr<mrpt::math::TPoint3D_<float>>, mrpt::math::TPoseOrPoint, mrpt::math::TPoint3D_data<float>> cl(M("mrpt::math"), "TPoint3D_float_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::TPoint3D_<float>(); } ) );
		cl.def( pybind11::init<float, float, float>(), pybind11::arg("xx"), pybind11::arg("yy"), pybind11::arg("zz") );

		cl.def( pybind11::init<const struct mrpt::math::TPoint2D_<float> &>(), pybind11::arg("p") );

		cl.def( pybind11::init<const struct mrpt::math::TPose2D &>(), pybind11::arg("p") );

		cl.def( pybind11::init<const struct mrpt::math::TPose3D &>(), pybind11::arg("p") );

		cl.def( pybind11::init( [](mrpt::math::TPoint3D_<float> const &o){ return new mrpt::math::TPoint3D_<float>(o); } ) );
		cl.def("__getitem__", (float & (mrpt::math::TPoint3D_<float>::*)(size_t)) &mrpt::math::TPoint3D_<float>::operator[], "C++: mrpt::math::TPoint3D_<float>::operator[](size_t) --> float &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("distanceTo", (float (mrpt::math::TPoint3D_<float>::*)(const struct mrpt::math::TPoint3D_<float> &) const) &mrpt::math::TPoint3D_<float>::distanceTo, "C++: mrpt::math::TPoint3D_<float>::distanceTo(const struct mrpt::math::TPoint3D_<float> &) const --> float", pybind11::arg("p"));
		cl.def("sqrDistanceTo", (float (mrpt::math::TPoint3D_<float>::*)(const struct mrpt::math::TPoint3D_<float> &) const) &mrpt::math::TPoint3D_<float>::sqrDistanceTo, "C++: mrpt::math::TPoint3D_<float>::sqrDistanceTo(const struct mrpt::math::TPoint3D_<float> &) const --> float", pybind11::arg("p"));
		cl.def("sqrNorm", (float (mrpt::math::TPoint3D_<float>::*)() const) &mrpt::math::TPoint3D_<float>::sqrNorm, "C++: mrpt::math::TPoint3D_<float>::sqrNorm() const --> float");
		cl.def("norm", (float (mrpt::math::TPoint3D_<float>::*)() const) &mrpt::math::TPoint3D_<float>::norm, "C++: mrpt::math::TPoint3D_<float>::norm() const --> float");
		cl.def("unitarize", (struct mrpt::math::TPoint3D_<float> (mrpt::math::TPoint3D_<float>::*)() const) &mrpt::math::TPoint3D_<float>::unitarize, "C++: mrpt::math::TPoint3D_<float>::unitarize() const --> struct mrpt::math::TPoint3D_<float>");
		cl.def("__imul__", (struct mrpt::math::TPoint3D_<float> & (mrpt::math::TPoint3D_<float>::*)(const float)) &mrpt::math::TPoint3D_<float>::operator*=, "C++: mrpt::math::TPoint3D_<float>::operator*=(const float) --> struct mrpt::math::TPoint3D_<float> &", pybind11::return_value_policy::automatic, pybind11::arg("f"));
		cl.def("__iadd__", (struct mrpt::math::TPoint3D_<float> & (mrpt::math::TPoint3D_<float>::*)(const struct mrpt::math::TPoint3D_<float> &)) &mrpt::math::TPoint3D_<float>::operator+=, "C++: mrpt::math::TPoint3D_<float>::operator+=(const struct mrpt::math::TPoint3D_<float> &) --> struct mrpt::math::TPoint3D_<float> &", pybind11::return_value_policy::automatic, pybind11::arg("p"));
		cl.def("__isub__", (struct mrpt::math::TPoint3D_<float> & (mrpt::math::TPoint3D_<float>::*)(const struct mrpt::math::TPoint3D_<float> &)) &mrpt::math::TPoint3D_<float>::operator-=, "C++: mrpt::math::TPoint3D_<float>::operator-=(const struct mrpt::math::TPoint3D_<float> &) --> struct mrpt::math::TPoint3D_<float> &", pybind11::return_value_policy::automatic, pybind11::arg("p"));
		cl.def("__add__", (struct mrpt::math::TPoint3D_<float> (mrpt::math::TPoint3D_<float>::*)(const struct mrpt::math::TPoint3D_<float> &) const) &mrpt::math::TPoint3D_<float>::operator+, "C++: mrpt::math::TPoint3D_<float>::operator+(const struct mrpt::math::TPoint3D_<float> &) const --> struct mrpt::math::TPoint3D_<float>", pybind11::arg("p"));
		cl.def("__sub__", (struct mrpt::math::TPoint3D_<float> (mrpt::math::TPoint3D_<float>::*)(const struct mrpt::math::TPoint3D_<float> &) const) &mrpt::math::TPoint3D_<float>::operator-, "C++: mrpt::math::TPoint3D_<float>::operator-(const struct mrpt::math::TPoint3D_<float> &) const --> struct mrpt::math::TPoint3D_<float>", pybind11::arg("p"));
		cl.def("__mul__", (struct mrpt::math::TPoint3D_<float> (mrpt::math::TPoint3D_<float>::*)(float) const) &mrpt::math::TPoint3D_<float>::operator*, "C++: mrpt::math::TPoint3D_<float>::operator*(float) const --> struct mrpt::math::TPoint3D_<float>", pybind11::arg("d"));
		cl.def("__truediv__", (struct mrpt::math::TPoint3D_<float> (mrpt::math::TPoint3D_<float>::*)(float) const) &mrpt::math::TPoint3D_<float>::operator/, "C++: mrpt::math::TPoint3D_<float>::operator/(float) const --> struct mrpt::math::TPoint3D_<float>", pybind11::arg("d"));
		cl.def("asString", (void (mrpt::math::TPoint3D_<float>::*)(std::string &) const) &mrpt::math::TPoint3D_<float>::asString, "C++: mrpt::math::TPoint3D_<float>::asString(std::string &) const --> void", pybind11::arg("s"));
		cl.def("asString", (std::string (mrpt::math::TPoint3D_<float>::*)() const) &mrpt::math::TPoint3D_<float>::asString, "C++: mrpt::math::TPoint3D_<float>::asString() const --> std::string");
		cl.def("fromString", (void (mrpt::math::TPoint3D_<float>::*)(const std::string &)) &mrpt::math::TPoint3D_<float>::fromString, "C++: mrpt::math::TPoint3D_<float>::fromString(const std::string &) --> void", pybind11::arg("s"));
		cl.def_static("FromString", (struct mrpt::math::TPoint3D_<float> (*)(const std::string &)) &mrpt::math::TPoint3D_<float>::FromString, "C++: mrpt::math::TPoint3D_<float>::FromString(const std::string &) --> struct mrpt::math::TPoint3D_<float>", pybind11::arg("s"));
		cl.def("assign", (struct mrpt::math::TPoint3D_<float> & (mrpt::math::TPoint3D_<float>::*)(const struct mrpt::math::TPoint3D_<float> &)) &mrpt::math::TPoint3D_<float>::operator=, "C++: mrpt::math::TPoint3D_<float>::operator=(const struct mrpt::math::TPoint3D_<float> &) --> struct mrpt::math::TPoint3D_<float> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("assign", (struct mrpt::math::TPoseOrPoint & (mrpt::math::TPoseOrPoint::*)(const struct mrpt::math::TPoseOrPoint &)) &mrpt::math::TPoseOrPoint::operator=, "C++: mrpt::math::TPoseOrPoint::operator=(const struct mrpt::math::TPoseOrPoint &) --> struct mrpt::math::TPoseOrPoint &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def_readwrite("x", &mrpt::math::TPoint3D_data<float>::x);
		cl.def_readwrite("y", &mrpt::math::TPoint3D_data<float>::y);
		cl.def_readwrite("z", &mrpt::math::TPoint3D_data<float>::z);
		cl.def("assign", (struct mrpt::math::TPoint3D_data<float> & (mrpt::math::TPoint3D_data<float>::*)(const struct mrpt::math::TPoint3D_data<float> &)) &mrpt::math::TPoint3D_data<float>::operator=, "C++: mrpt::math::TPoint3D_data<float>::operator=(const struct mrpt::math::TPoint3D_data<float> &) --> struct mrpt::math::TPoint3D_data<float> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::math::CQuaternion file:mrpt/math/CQuaternion.h line:44
		pybind11::class_<mrpt::math::CQuaternion<double>, std::shared_ptr<mrpt::math::CQuaternion<double>>, mrpt::math::CMatrixFixed<double,4UL,1UL>> cl(M("mrpt::math"), "CQuaternion_double_t", "");
		cl.def( pybind11::init<enum mrpt::math::TConstructorFlags_Quaternions>(), pybind11::arg("") );

		cl.def( pybind11::init( [](){ return new mrpt::math::CQuaternion<double>(); } ) );
		cl.def( pybind11::init<const double, const double, const double, const double>(), pybind11::arg("R"), pybind11::arg("X"), pybind11::arg("Y"), pybind11::arg("Z") );

		cl.def( pybind11::init( [](mrpt::math::CQuaternion<double> const &o){ return new mrpt::math::CQuaternion<double>(o); } ) );
		cl.def("ensurePositiveRealPart", (void (mrpt::math::CQuaternion<double>::*)()) &mrpt::math::CQuaternion<double>::ensurePositiveRealPart, "C++: mrpt::math::CQuaternion<double>::ensurePositiveRealPart() --> void");
		cl.def("w", (double (mrpt::math::CQuaternion<double>::*)() const) &mrpt::math::CQuaternion<double>::w, "C++: mrpt::math::CQuaternion<double>::w() const --> double");
		cl.def("r", (void (mrpt::math::CQuaternion<double>::*)(const double)) &mrpt::math::CQuaternion<double>::r, "C++: mrpt::math::CQuaternion<double>::r(const double) --> void", pybind11::arg("r"));
		cl.def("w", (void (mrpt::math::CQuaternion<double>::*)(const double)) &mrpt::math::CQuaternion<double>::w, "C++: mrpt::math::CQuaternion<double>::w(const double) --> void", pybind11::arg("w"));
		cl.def("x", (void (mrpt::math::CQuaternion<double>::*)(const double)) &mrpt::math::CQuaternion<double>::x, "C++: mrpt::math::CQuaternion<double>::x(const double) --> void", pybind11::arg("x"));
		cl.def("y", (void (mrpt::math::CQuaternion<double>::*)(const double)) &mrpt::math::CQuaternion<double>::y, "C++: mrpt::math::CQuaternion<double>::y(const double) --> void", pybind11::arg("y"));
		cl.def("z", (void (mrpt::math::CQuaternion<double>::*)(const double)) &mrpt::math::CQuaternion<double>::z, "C++: mrpt::math::CQuaternion<double>::z(const double) --> void", pybind11::arg("z"));
		cl.def("r", (double & (mrpt::math::CQuaternion<double>::*)()) &mrpt::math::CQuaternion<double>::r, "C++: mrpt::math::CQuaternion<double>::r() --> double &", pybind11::return_value_policy::automatic);
		cl.def("x", (double & (mrpt::math::CQuaternion<double>::*)()) &mrpt::math::CQuaternion<double>::x, "C++: mrpt::math::CQuaternion<double>::x() --> double &", pybind11::return_value_policy::automatic);
		cl.def("y", (double & (mrpt::math::CQuaternion<double>::*)()) &mrpt::math::CQuaternion<double>::y, "C++: mrpt::math::CQuaternion<double>::y() --> double &", pybind11::return_value_policy::automatic);
		cl.def("z", (double & (mrpt::math::CQuaternion<double>::*)()) &mrpt::math::CQuaternion<double>::z, "C++: mrpt::math::CQuaternion<double>::z() --> double &", pybind11::return_value_policy::automatic);
		cl.def("crossProduct", (void (mrpt::math::CQuaternion<double>::*)(const class mrpt::math::CQuaternion<double> &, const class mrpt::math::CQuaternion<double> &)) &mrpt::math::CQuaternion<double>::crossProduct, "C++: mrpt::math::CQuaternion<double>::crossProduct(const class mrpt::math::CQuaternion<double> &, const class mrpt::math::CQuaternion<double> &) --> void", pybind11::arg("q1"), pybind11::arg("q2"));
		cl.def("rotatePoint", (void (mrpt::math::CQuaternion<double>::*)(const double, const double, const double, double &, double &, double &) const) &mrpt::math::CQuaternion<double>::rotatePoint, "C++: mrpt::math::CQuaternion<double>::rotatePoint(const double, const double, const double, double &, double &, double &) const --> void", pybind11::arg("lx"), pybind11::arg("ly"), pybind11::arg("lz"), pybind11::arg("gx"), pybind11::arg("gy"), pybind11::arg("gz"));
		cl.def("inverseRotatePoint", (void (mrpt::math::CQuaternion<double>::*)(const double, const double, const double, double &, double &, double &) const) &mrpt::math::CQuaternion<double>::inverseRotatePoint, "C++: mrpt::math::CQuaternion<double>::inverseRotatePoint(const double, const double, const double, double &, double &, double &) const --> void", pybind11::arg("lx"), pybind11::arg("ly"), pybind11::arg("lz"), pybind11::arg("gx"), pybind11::arg("gy"), pybind11::arg("gz"));
		cl.def("normSqr", (double (mrpt::math::CQuaternion<double>::*)() const) &mrpt::math::CQuaternion<double>::normSqr, "C++: mrpt::math::CQuaternion<double>::normSqr() const --> double");
		cl.def("normalize", (void (mrpt::math::CQuaternion<double>::*)()) &mrpt::math::CQuaternion<double>::normalize, "C++: mrpt::math::CQuaternion<double>::normalize() --> void");
		cl.def("conj", (void (mrpt::math::CQuaternion<double>::*)(class mrpt::math::CQuaternion<double> &) const) &mrpt::math::CQuaternion<double>::conj, "C++: mrpt::math::CQuaternion<double>::conj(class mrpt::math::CQuaternion<double> &) const --> void", pybind11::arg("q_out"));
		cl.def("conj", (class mrpt::math::CQuaternion<double> (mrpt::math::CQuaternion<double>::*)() const) &mrpt::math::CQuaternion<double>::conj, "C++: mrpt::math::CQuaternion<double>::conj() const --> class mrpt::math::CQuaternion<double>");
		cl.def("rpy", (void (mrpt::math::CQuaternion<double>::*)(double &, double &, double &) const) &mrpt::math::CQuaternion<double>::rpy, "C++: mrpt::math::CQuaternion<double>::rpy(double &, double &, double &) const --> void", pybind11::arg("roll"), pybind11::arg("pitch"), pybind11::arg("yaw"));
		cl.def("__mul__", (class mrpt::math::CQuaternion<double> (mrpt::math::CQuaternion<double>::*)(const double &)) &mrpt::math::CQuaternion<double>::operator*, "C++: mrpt::math::CQuaternion<double>::operator*(const double &) --> class mrpt::math::CQuaternion<double>", pybind11::arg("factor"));
		cl.def("jacobian_rodrigues_from_quat", (class mrpt::math::CMatrixFixed<double, 3, 4> (mrpt::math::CQuaternion<double>::*)() const) &mrpt::math::CQuaternion<double>::jacobian_rodrigues_from_quat, "C++: mrpt::math::CQuaternion<double>::jacobian_rodrigues_from_quat() const --> class mrpt::math::CMatrixFixed<double, 3, 4>");
		cl.def("assign", (class mrpt::math::CQuaternion<double> & (mrpt::math::CQuaternion<double>::*)(const class mrpt::math::CQuaternion<double> &)) &mrpt::math::CQuaternion<double>::operator=, "C++: mrpt::math::CQuaternion<double>::operator=(const class mrpt::math::CQuaternion<double> &) --> class mrpt::math::CQuaternion<double> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
		cl.def("loadFromRawPointer", (void (mrpt::math::CMatrixFixed<double,4UL,1UL>::*)(const double *)) &mrpt::math::CMatrixFixed<double, 4, 1>::loadFromRawPointer, "C++: mrpt::math::CMatrixFixed<double, 4, 1>::loadFromRawPointer(const double *) --> void", pybind11::arg("data"));
		cl.def("setSize", [](mrpt::math::CMatrixFixed<double,4UL,1UL> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CMatrixFixed<double,4UL,1UL>::*)(size_t, size_t, bool)) &mrpt::math::CMatrixFixed<double, 4, 1>::setSize, "C++: mrpt::math::CMatrixFixed<double, 4, 1>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("swap", (void (mrpt::math::CMatrixFixed<double,4UL,1UL>::*)(class mrpt::math::CMatrixFixed<double, 4, 1> &)) &mrpt::math::CMatrixFixed<double, 4, 1>::swap, "C++: mrpt::math::CMatrixFixed<double, 4, 1>::swap(class mrpt::math::CMatrixFixed<double, 4, 1> &) --> void", pybind11::arg("o"));
		cl.def("conservativeResize", (void (mrpt::math::CMatrixFixed<double,4UL,1UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<double, 4, 1>::conservativeResize, "C++: mrpt::math::CMatrixFixed<double, 4, 1>::conservativeResize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<double,4UL,1UL>::*)(size_t)) &mrpt::math::CMatrixFixed<double, 4, 1>::resize, "C++: mrpt::math::CMatrixFixed<double, 4, 1>::resize(size_t) --> void", pybind11::arg("n"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<double,4UL,1UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<double, 4, 1>::resize, "C++: mrpt::math::CMatrixFixed<double, 4, 1>::resize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("rows", (int (mrpt::math::CMatrixFixed<double,4UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<double, 4, 1>::rows, "C++: mrpt::math::CMatrixFixed<double, 4, 1>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CMatrixFixed<double,4UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<double, 4, 1>::cols, "C++: mrpt::math::CMatrixFixed<double, 4, 1>::cols() const --> int");
		cl.def("data", (double * (mrpt::math::CMatrixFixed<double,4UL,1UL>::*)()) &mrpt::math::CMatrixFixed<double, 4, 1>::data, "C++: mrpt::math::CMatrixFixed<double, 4, 1>::data() --> double *", pybind11::return_value_policy::automatic);
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,4UL,1UL>::*)(int, int)) &mrpt::math::CMatrixFixed<double, 4, 1>::operator(), "C++: mrpt::math::CMatrixFixed<double, 4, 1>::operator()(int, int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,4UL,1UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 4, 1>::operator(), "C++: mrpt::math::CMatrixFixed<double, 4, 1>::operator()(int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("__getitem__", (double & (mrpt::math::CMatrixFixed<double,4UL,1UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 4, 1>::operator[], "C++: mrpt::math::CMatrixFixed<double, 4, 1>::operator[](int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("cast_float", (class mrpt::math::CMatrixFixed<float, 4, 1> (mrpt::math::CMatrixFixed<double,4UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<double, 4, 1>::cast_float, "C++: mrpt::math::CMatrixFixed<double, 4, 1>::cast_float() const --> class mrpt::math::CMatrixFixed<float, 4, 1>");
		cl.def("sum_At", (void (mrpt::math::CMatrixFixed<double,4UL,1UL>::*)(const class mrpt::math::CMatrixFixed<double, 4, 1> &)) &mrpt::math::CMatrixFixed<double, 4, 1>::sum_At, "C++: mrpt::math::CMatrixFixed<double, 4, 1>::sum_At(const class mrpt::math::CMatrixFixed<double, 4, 1> &) --> void", pybind11::arg("A"));
		cl.def("assign", (class mrpt::math::CMatrixFixed<double, 4, 1> & (mrpt::math::CMatrixFixed<double,4UL,1UL>::*)(const class mrpt::math::CMatrixFixed<double, 4, 1> &)) &mrpt::math::CMatrixFixed<double, 4, 1>::operator=, "C++: mrpt::math::CMatrixFixed<double, 4, 1>::operator=(const class mrpt::math::CMatrixFixed<double, 4, 1> &) --> class mrpt::math::CMatrixFixed<double, 4, 1> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
