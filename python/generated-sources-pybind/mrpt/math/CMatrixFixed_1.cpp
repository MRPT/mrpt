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

void bind_mrpt_math_CMatrixFixed_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::CMatrixFixed file:mrpt/math/CMatrixFixed.h line:34
		pybind11::class_<mrpt::math::CMatrixFixed<float,4,1>, std::shared_ptr<mrpt::math::CMatrixFixed<float,4,1>>> cl(M("mrpt::math"), "CMatrixFixed_float_4_1_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixFixed<float,4,1>(); } ) );
		cl.def( pybind11::init<enum mrpt::math::TConstructorFlags_Matrices>(), pybind11::arg("") );

		cl.def( pybind11::init<const float *>(), pybind11::arg("data") );

		cl.def( pybind11::init<const int, const int>(), pybind11::arg("rows"), pybind11::arg("cols") );

		cl.def( pybind11::init( [](mrpt::math::CMatrixFixed<float,4,1> const &o){ return new mrpt::math::CMatrixFixed<float,4,1>(o); } ) );
		cl.def("loadFromRawPointer", (void (mrpt::math::CMatrixFixed<float,4,1>::*)(const float *)) &mrpt::math::CMatrixFixed<float, 4, 1>::loadFromRawPointer, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::loadFromRawPointer(const float *) --> void", pybind11::arg("data"));
		cl.def("setSize", [](mrpt::math::CMatrixFixed<float,4,1> &o, unsigned long const & a0, unsigned long const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CMatrixFixed<float,4,1>::*)(unsigned long, unsigned long, bool)) &mrpt::math::CMatrixFixed<float, 4, 1>::setSize, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::setSize(unsigned long, unsigned long, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("swap", (void (mrpt::math::CMatrixFixed<float,4,1>::*)(class mrpt::math::CMatrixFixed<float, 4, 1> &)) &mrpt::math::CMatrixFixed<float, 4, 1>::swap, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::swap(class mrpt::math::CMatrixFixed<float, 4, 1> &) --> void", pybind11::arg("o"));
		cl.def("conservativeResize", (void (mrpt::math::CMatrixFixed<float,4,1>::*)(unsigned long, unsigned long)) &mrpt::math::CMatrixFixed<float, 4, 1>::conservativeResize, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::conservativeResize(unsigned long, unsigned long) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<float,4,1>::*)(unsigned long)) &mrpt::math::CMatrixFixed<float, 4, 1>::resize, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::resize(unsigned long) --> void", pybind11::arg("n"));
		cl.def("resize", [](mrpt::math::CMatrixFixed<float,4,1> &o, const struct mrpt::math::matrix_size_t & a0) -> void { return o.resize(a0); }, "", pybind11::arg("siz"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<float,4,1>::*)(const struct mrpt::math::matrix_size_t &, bool)) &mrpt::math::CMatrixFixed<float, 4, 1>::resize, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::resize(const struct mrpt::math::matrix_size_t &, bool) --> void", pybind11::arg("siz"), pybind11::arg("zeroNewElements"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<float,4,1>::*)(unsigned long, unsigned long)) &mrpt::math::CMatrixFixed<float, 4, 1>::resize, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::resize(unsigned long, unsigned long) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("rows", (int (mrpt::math::CMatrixFixed<float,4,1>::*)() const) &mrpt::math::CMatrixFixed<float, 4, 1>::rows, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CMatrixFixed<float,4,1>::*)() const) &mrpt::math::CMatrixFixed<float, 4, 1>::cols, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::cols() const --> int");
		cl.def("size", (struct mrpt::math::matrix_size_t (mrpt::math::CMatrixFixed<float,4,1>::*)() const) &mrpt::math::CMatrixFixed<float, 4, 1>::size, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::size() const --> struct mrpt::math::matrix_size_t");
		cl.def("data", (float * (mrpt::math::CMatrixFixed<float,4,1>::*)()) &mrpt::math::CMatrixFixed<float, 4, 1>::data, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::data() --> float *", pybind11::return_value_policy::automatic);
		cl.def("__call__", (float & (mrpt::math::CMatrixFixed<float,4,1>::*)(int, int)) &mrpt::math::CMatrixFixed<float, 4, 1>::operator(), "C++: mrpt::math::CMatrixFixed<float, 4, 1>::operator()(int, int) --> float &", pybind11::return_value_policy::automatic, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__call__", (float & (mrpt::math::CMatrixFixed<float,4,1>::*)(int)) &mrpt::math::CMatrixFixed<float, 4, 1>::operator(), "C++: mrpt::math::CMatrixFixed<float, 4, 1>::operator()(int) --> float &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("__getitem__", (float & (mrpt::math::CMatrixFixed<float,4,1>::*)(int)) &mrpt::math::CMatrixFixed<float, 4, 1>::operator[], "C++: mrpt::math::CMatrixFixed<float, 4, 1>::operator[](int) --> float &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("cast_float", (class mrpt::math::CMatrixFixed<float, 4, 1> (mrpt::math::CMatrixFixed<float,4,1>::*)() const) &mrpt::math::CMatrixFixed<float, 4, 1>::cast_float, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::cast_float() const --> class mrpt::math::CMatrixFixed<float, 4, 1>");
		cl.def("sum_At", (void (mrpt::math::CMatrixFixed<float,4,1>::*)(const class mrpt::math::CMatrixFixed<float, 4, 1> &)) &mrpt::math::CMatrixFixed<float, 4, 1>::sum_At, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::sum_At(const class mrpt::math::CMatrixFixed<float, 4, 1> &) --> void", pybind11::arg("A"));
		cl.def("assign", (class mrpt::math::CMatrixFixed<float, 4, 1> & (mrpt::math::CMatrixFixed<float,4,1>::*)(const class mrpt::math::CMatrixFixed<float, 4, 1> &)) &mrpt::math::CMatrixFixed<float, 4, 1>::operator=, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::operator=(const class mrpt::math::CMatrixFixed<float, 4, 1> &) --> class mrpt::math::CMatrixFixed<float, 4, 1> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	// mrpt::math::TMatrixTextFileFormat file:mrpt/math/MatrixVectorBase.h line:35
	pybind11::enum_<mrpt::math::TMatrixTextFileFormat>(M("mrpt::math"), "TMatrixTextFileFormat", pybind11::arithmetic(), "Selection of the number format in MatrixVectorBase::saveToTextFile()\n \n")
		.value("MATRIX_FORMAT_ENG", mrpt::math::MATRIX_FORMAT_ENG)
		.value("MATRIX_FORMAT_FIXED", mrpt::math::MATRIX_FORMAT_FIXED)
		.value("MATRIX_FORMAT_INT", mrpt::math::MATRIX_FORMAT_INT)
		.export_values();

;

	{ // mrpt::math::MatrixVectorBase file:mrpt/math/MatrixVectorBase.h line:57
		pybind11::class_<mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>, std::shared_ptr<mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>>> cl(M("mrpt::math"), "MatrixVectorBase_double_mrpt_math_CMatrixDynamic_double_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>(); } ) );
		cl.def( pybind11::init( [](mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>> const &o){ return new mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>(o); } ) );
		cl.def("fill", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(const double &)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::fill, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::fill(const double &) --> void", pybind11::arg("val"));
		cl.def("setConstant", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(const double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::setConstant, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::setConstant(const double) --> void", pybind11::arg("value"));
		cl.def("setConstant", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(unsigned long, unsigned long, const double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::setConstant, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::setConstant(unsigned long, unsigned long, const double) --> void", pybind11::arg("nrows"), pybind11::arg("ncols"), pybind11::arg("value"));
		cl.def("setConstant", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(unsigned long, const double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::setConstant, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::setConstant(unsigned long, const double) --> void", pybind11::arg("nrows"), pybind11::arg("value"));
		cl.def_static("Constant", (class mrpt::math::CMatrixDynamic<double> (*)(const double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::Constant, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::Constant(const double) --> class mrpt::math::CMatrixDynamic<double>", pybind11::arg("value"));
		cl.def_static("Constant", (class mrpt::math::CMatrixDynamic<double> (*)(unsigned long, unsigned long, const double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::Constant, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::Constant(unsigned long, unsigned long, const double) --> class mrpt::math::CMatrixDynamic<double>", pybind11::arg("nrows"), pybind11::arg("ncols"), pybind11::arg("value"));
		cl.def("assign", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(const unsigned long, const double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::assign, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::assign(const unsigned long, const double) --> void", pybind11::arg("N"), pybind11::arg("value"));
		cl.def("setZero", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)()) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::setZero, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::setZero() --> void");
		cl.def("setZero", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(unsigned long, unsigned long)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::setZero, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::setZero(unsigned long, unsigned long) --> void", pybind11::arg("nrows"), pybind11::arg("ncols"));
		cl.def("setZero", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(unsigned long)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::setZero, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::setZero(unsigned long) --> void", pybind11::arg("nrows"));
		cl.def_static("Zero", (class mrpt::math::CMatrixDynamic<double> (*)()) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::Zero, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::Zero() --> class mrpt::math::CMatrixDynamic<double>");
		cl.def_static("Zero", (class mrpt::math::CMatrixDynamic<double> (*)(unsigned long, unsigned long)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::Zero, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::Zero(unsigned long, unsigned long) --> class mrpt::math::CMatrixDynamic<double>", pybind11::arg("nrows"), pybind11::arg("ncols"));
		cl.def("coeffRef", (double & (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(int, int)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::coeffRef, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::coeffRef(int, int) --> double &", pybind11::return_value_policy::automatic, pybind11::arg("r"), pybind11::arg("c"));
		cl.def("coeff", (const double & (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(int, int) const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::coeff, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::coeff(int, int) const --> const double &", pybind11::return_value_policy::automatic, pybind11::arg("r"), pybind11::arg("c"));
		cl.def("minCoeff", (double (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::minCoeff, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::minCoeff() const --> double");
		cl.def("minCoeff", (double (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(unsigned long &) const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::minCoeff, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::minCoeff(unsigned long &) const --> double", pybind11::arg("outIndexOfMin"));
		cl.def("minCoeff", (double (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(unsigned long &, unsigned long &) const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::minCoeff, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::minCoeff(unsigned long &, unsigned long &) const --> double", pybind11::arg("rowIdx"), pybind11::arg("colIdx"));
		cl.def("maxCoeff", (double (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::maxCoeff, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::maxCoeff() const --> double");
		cl.def("maxCoeff", (double (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(unsigned long &) const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::maxCoeff, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::maxCoeff(unsigned long &) const --> double", pybind11::arg("outIndexOfMax"));
		cl.def("maxCoeff", (double (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(unsigned long &, unsigned long &) const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::maxCoeff, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::maxCoeff(unsigned long &, unsigned long &) const --> double", pybind11::arg("rowIdx"), pybind11::arg("colIdx"));
		cl.def("isSquare", (bool (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::isSquare, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::isSquare() const --> bool");
		cl.def("empty", (bool (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::empty, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::empty() const --> bool");
		cl.def("norm_inf", (double (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::norm_inf, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::norm_inf() const --> double");
		cl.def("norm", (double (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::norm, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::norm() const --> double");
		cl.def("__iadd__", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::operator+=, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::operator+=(double) --> void", pybind11::arg("s"));
		cl.def("__isub__", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::operator-=, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::operator-=(double) --> void", pybind11::arg("s"));
		cl.def("__imul__", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::operator*=, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::operator*=(double) --> void", pybind11::arg("s"));
		cl.def("__add__", (class mrpt::math::CMatrixDynamic<double> (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(const class mrpt::math::CMatrixDynamic<double> &) const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::operator+, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::operator+(const class mrpt::math::CMatrixDynamic<double> &) const --> class mrpt::math::CMatrixDynamic<double>", pybind11::arg("m2"));
		cl.def("__iadd__", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(const class mrpt::math::CMatrixDynamic<double> &)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::operator+=, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::operator+=(const class mrpt::math::CMatrixDynamic<double> &) --> void", pybind11::arg("m2"));
		cl.def("__isub__", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(const class mrpt::math::CMatrixDynamic<double> &)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::operator-=, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::operator-=(const class mrpt::math::CMatrixDynamic<double> &) --> void", pybind11::arg("m2"));
		cl.def("sum", (double (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::sum, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::sum() const --> double");
		cl.def("sum_abs", (double (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::sum_abs, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::sum_abs() const --> double");
		cl.def("asString", (std::string (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::asString, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::asString() const --> std::string");
		cl.def("inMatlabFormat", [](mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>> const &o) -> std::string { return o.inMatlabFormat(); }, "");
		cl.def("inMatlabFormat", (std::string (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(const unsigned long) const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::inMatlabFormat, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::inMatlabFormat(const unsigned long) const --> std::string", pybind11::arg("decimal_digits"));
		cl.def("saveToTextFile", [](mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>> const &o, const std::string & a0) -> void { return o.saveToTextFile(a0); }, "", pybind11::arg("file"));
		cl.def("saveToTextFile", [](mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>> const &o, const std::string & a0, enum mrpt::math::TMatrixTextFileFormat const & a1) -> void { return o.saveToTextFile(a0, a1); }, "", pybind11::arg("file"), pybind11::arg("fileFormat"));
		cl.def("saveToTextFile", [](mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>> const &o, const std::string & a0, enum mrpt::math::TMatrixTextFileFormat const & a1, bool const & a2) -> void { return o.saveToTextFile(a0, a1, a2); }, "", pybind11::arg("file"), pybind11::arg("fileFormat"), pybind11::arg("appendMRPTHeader"));
		cl.def("saveToTextFile", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(const std::string &, enum mrpt::math::TMatrixTextFileFormat, bool, const std::string &) const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::saveToTextFile, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::saveToTextFile(const std::string &, enum mrpt::math::TMatrixTextFileFormat, bool, const std::string &) const --> void", pybind11::arg("file"), pybind11::arg("fileFormat"), pybind11::arg("appendMRPTHeader"), pybind11::arg("userHeader"));
		cl.def("loadFromTextFile", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(const std::string &)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::loadFromTextFile, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::loadFromTextFile(const std::string &) --> void", pybind11::arg("file"));
		cl.def("assign", (class mrpt::math::MatrixVectorBase<double, class mrpt::math::CMatrixDynamic<double> > & (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>>::*)(const class mrpt::math::MatrixVectorBase<double, class mrpt::math::CMatrixDynamic<double> > &)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::operator=, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixDynamic<double>>::operator=(const class mrpt::math::MatrixVectorBase<double, class mrpt::math::CMatrixDynamic<double> > &) --> class mrpt::math::MatrixVectorBase<double, class mrpt::math::CMatrixDynamic<double> > &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixDynamic<double>> const &o) -> std::string { std::ostringstream s; using namespace mrpt::math; s << o; return s.str(); } );
	}
}
