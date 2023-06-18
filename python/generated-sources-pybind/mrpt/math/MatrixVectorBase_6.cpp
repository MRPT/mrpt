#include <Eigen/Dense>
#include <ios>
#include <istream>
#include <iterator>
#include <locale>
#include <memory>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/MatrixVectorBase.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <optional>
#include <ostream>
#include <sstream> // __str__
#include <streambuf>
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

void bind_mrpt_math_MatrixVectorBase_6(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::MatrixVectorBase file:mrpt/math/MatrixVectorBase.h line:57
		pybind11::class_<mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>, std::shared_ptr<mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>>> cl(M("mrpt::math"), "MatrixVectorBase_double_mrpt_math_CMatrixFixed_double_7_7_t", "");
		cl.def( pybind11::init( [](mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>> const &o){ return new mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>(); } ) );
		cl.def("fill", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)(const double &)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::fill, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::fill(const double &) --> void", pybind11::arg("val"));
		cl.def("setConstant", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)(const double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::setConstant, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::setConstant(const double) --> void", pybind11::arg("value"));
		cl.def("setConstant", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)(size_t, size_t, const double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::setConstant, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::setConstant(size_t, size_t, const double) --> void", pybind11::arg("nrows"), pybind11::arg("ncols"), pybind11::arg("value"));
		cl.def("setConstant", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)(size_t, const double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::setConstant, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::setConstant(size_t, const double) --> void", pybind11::arg("nrows"), pybind11::arg("value"));
		cl.def_static("Constant", (class mrpt::math::CMatrixFixed<double, 7, 7> (*)(const double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::Constant, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::Constant(const double) --> class mrpt::math::CMatrixFixed<double, 7, 7>", pybind11::arg("value"));
		cl.def_static("Constant", (class mrpt::math::CMatrixFixed<double, 7, 7> (*)(size_t, size_t, const double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::Constant, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::Constant(size_t, size_t, const double) --> class mrpt::math::CMatrixFixed<double, 7, 7>", pybind11::arg("nrows"), pybind11::arg("ncols"), pybind11::arg("value"));
		cl.def("assign", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)(const unsigned long, const double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::assign, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::assign(const unsigned long, const double) --> void", pybind11::arg("N"), pybind11::arg("value"));
		cl.def("setZero", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)()) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::setZero, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::setZero() --> void");
		cl.def("setZero", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)(size_t, size_t)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::setZero, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::setZero(size_t, size_t) --> void", pybind11::arg("nrows"), pybind11::arg("ncols"));
		cl.def("setZero", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)(size_t)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::setZero, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::setZero(size_t) --> void", pybind11::arg("nrows"));
		cl.def_static("Zero", (class mrpt::math::CMatrixFixed<double, 7, 7> (*)()) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::Zero, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::Zero() --> class mrpt::math::CMatrixFixed<double, 7, 7>");
		cl.def_static("Zero", (class mrpt::math::CMatrixFixed<double, 7, 7> (*)(size_t, size_t)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::Zero, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::Zero(size_t, size_t) --> class mrpt::math::CMatrixFixed<double, 7, 7>", pybind11::arg("nrows"), pybind11::arg("ncols"));
		cl.def("coeffRef", (double & (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)(int, int)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::coeffRef, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::coeffRef(int, int) --> double &", pybind11::return_value_policy::automatic, pybind11::arg("r"), pybind11::arg("c"));
		cl.def("coeff", (const double & (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)(int, int) const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::coeff, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::coeff(int, int) const --> const double &", pybind11::return_value_policy::automatic, pybind11::arg("r"), pybind11::arg("c"));
		cl.def("isSquare", (bool (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::isSquare, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::isSquare() const --> bool");
		cl.def("empty", (bool (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::empty, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::empty() const --> bool");
		cl.def("norm_inf", (double (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::norm_inf, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::norm_inf() const --> double");
		cl.def("norm", (double (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::norm, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::norm() const --> double");
		cl.def("__iadd__", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)(double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::operator+=, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::operator+=(double) --> void", pybind11::arg("s"));
		cl.def("__isub__", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)(double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::operator-=, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::operator-=(double) --> void", pybind11::arg("s"));
		cl.def("__imul__", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)(double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::operator*=, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::operator*=(double) --> void", pybind11::arg("s"));
		cl.def("__add__", (class mrpt::math::CMatrixFixed<double, 7, 7> (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)(const class mrpt::math::CMatrixFixed<double, 7, 7> &) const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::operator+, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::operator+(const class mrpt::math::CMatrixFixed<double, 7, 7> &) const --> class mrpt::math::CMatrixFixed<double, 7, 7>", pybind11::arg("m2"));
		cl.def("__iadd__", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)(const class mrpt::math::CMatrixFixed<double, 7, 7> &)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::operator+=, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::operator+=(const class mrpt::math::CMatrixFixed<double, 7, 7> &) --> void", pybind11::arg("m2"));
		cl.def("__isub__", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)(const class mrpt::math::CMatrixFixed<double, 7, 7> &)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::operator-=, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::operator-=(const class mrpt::math::CMatrixFixed<double, 7, 7> &) --> void", pybind11::arg("m2"));
		cl.def("sum", (double (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::sum, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::sum() const --> double");
		cl.def("sum_abs", (double (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::sum_abs, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::sum_abs() const --> double");
		cl.def("asString", (std::string (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::asString, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::asString() const --> std::string");
		cl.def("inMatlabFormat", [](mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>> const &o) -> std::string { return o.inMatlabFormat(); }, "");
		cl.def("inMatlabFormat", (std::string (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)(const unsigned long) const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::inMatlabFormat, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::inMatlabFormat(const unsigned long) const --> std::string", pybind11::arg("decimal_digits"));
		cl.def("saveToTextFile", [](mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>> const &o, const std::string & a0) -> void { return o.saveToTextFile(a0); }, "", pybind11::arg("file"));
		cl.def("saveToTextFile", [](mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>> const &o, const std::string & a0, enum mrpt::math::TMatrixTextFileFormat const & a1) -> void { return o.saveToTextFile(a0, a1); }, "", pybind11::arg("file"), pybind11::arg("fileFormat"));
		cl.def("saveToTextFile", [](mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>> const &o, const std::string & a0, enum mrpt::math::TMatrixTextFileFormat const & a1, bool const & a2) -> void { return o.saveToTextFile(a0, a1, a2); }, "", pybind11::arg("file"), pybind11::arg("fileFormat"), pybind11::arg("appendMRPTHeader"));
		cl.def("saveToTextFile", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)(const std::string &, enum mrpt::math::TMatrixTextFileFormat, bool, const std::string &) const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::saveToTextFile, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::saveToTextFile(const std::string &, enum mrpt::math::TMatrixTextFileFormat, bool, const std::string &) const --> void", pybind11::arg("file"), pybind11::arg("fileFormat"), pybind11::arg("appendMRPTHeader"), pybind11::arg("userHeader"));
		cl.def("loadFromTextFile", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)(const std::string &)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::loadFromTextFile, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::loadFromTextFile(const std::string &) --> void", pybind11::arg("file"));
		cl.def("assign", (class mrpt::math::MatrixVectorBase<double, class mrpt::math::CMatrixFixed<double, 7, 7> > & (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>>::*)(const class mrpt::math::MatrixVectorBase<double, class mrpt::math::CMatrixFixed<double, 7, 7> > &)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::operator=, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 7, 7>>::operator=(const class mrpt::math::MatrixVectorBase<double, class mrpt::math::CMatrixFixed<double, 7, 7> > &) --> class mrpt::math::MatrixVectorBase<double, class mrpt::math::CMatrixFixed<double, 7, 7> > &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 7, 7>> const &o) -> std::string { std::ostringstream s; using namespace mrpt::math; s << o; return s.str(); } );
	}
}
