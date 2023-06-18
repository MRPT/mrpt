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
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mrpt_math_CMatrixFixed_4(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::CMatrixFixed file:mrpt/math/CMatrixFixed.h line:34
		pybind11::class_<mrpt::math::CMatrixFixed<double,12UL,1UL>, std::shared_ptr<mrpt::math::CMatrixFixed<double,12UL,1UL>>> cl(M("mrpt::math"), "CMatrixFixed_double_12UL_1UL_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixFixed<double,12UL,1UL>(); } ) );
		cl.def( pybind11::init<enum mrpt::math::TConstructorFlags_Matrices>(), pybind11::arg("") );

		cl.def( pybind11::init<const double *>(), pybind11::arg("data") );

		cl.def( pybind11::init<const int, const int>(), pybind11::arg("rows"), pybind11::arg("cols") );

		cl.def( pybind11::init( [](mrpt::math::CMatrixFixed<double,12UL,1UL> const &o){ return new mrpt::math::CMatrixFixed<double,12UL,1UL>(o); } ) );
		cl.def("loadFromRawPointer", (void (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)(const double *)) &mrpt::math::CMatrixFixed<double, 12, 1>::loadFromRawPointer, "C++: mrpt::math::CMatrixFixed<double, 12, 1>::loadFromRawPointer(const double *) --> void", pybind11::arg("data"));
		cl.def("setSize", [](mrpt::math::CMatrixFixed<double,12UL,1UL> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)(size_t, size_t, bool)) &mrpt::math::CMatrixFixed<double, 12, 1>::setSize, "C++: mrpt::math::CMatrixFixed<double, 12, 1>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("swap", (void (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)(class mrpt::math::CMatrixFixed<double, 12, 1> &)) &mrpt::math::CMatrixFixed<double, 12, 1>::swap, "C++: mrpt::math::CMatrixFixed<double, 12, 1>::swap(class mrpt::math::CMatrixFixed<double, 12, 1> &) --> void", pybind11::arg("o"));
		cl.def("conservativeResize", (void (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<double, 12, 1>::conservativeResize, "C++: mrpt::math::CMatrixFixed<double, 12, 1>::conservativeResize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)(size_t)) &mrpt::math::CMatrixFixed<double, 12, 1>::resize, "C++: mrpt::math::CMatrixFixed<double, 12, 1>::resize(size_t) --> void", pybind11::arg("n"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<double, 12, 1>::resize, "C++: mrpt::math::CMatrixFixed<double, 12, 1>::resize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("rows", (int (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<double, 12, 1>::rows, "C++: mrpt::math::CMatrixFixed<double, 12, 1>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<double, 12, 1>::cols, "C++: mrpt::math::CMatrixFixed<double, 12, 1>::cols() const --> int");
		cl.def("data", (double * (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)()) &mrpt::math::CMatrixFixed<double, 12, 1>::data, "C++: mrpt::math::CMatrixFixed<double, 12, 1>::data() --> double *", pybind11::return_value_policy::automatic);
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)(int, int)) &mrpt::math::CMatrixFixed<double, 12, 1>::operator(), "C++: mrpt::math::CMatrixFixed<double, 12, 1>::operator()(int, int) --> double &", pybind11::return_value_policy::automatic, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 12, 1>::operator(), "C++: mrpt::math::CMatrixFixed<double, 12, 1>::operator()(int) --> double &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("__getitem__", (double & (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 12, 1>::operator[], "C++: mrpt::math::CMatrixFixed<double, 12, 1>::operator[](int) --> double &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("sum_At", (void (mrpt::math::CMatrixFixed<double,12UL,1UL>::*)(const class mrpt::math::CMatrixFixed<double, 12, 1> &)) &mrpt::math::CMatrixFixed<double, 12, 1>::sum_At, "C++: mrpt::math::CMatrixFixed<double, 12, 1>::sum_At(const class mrpt::math::CMatrixFixed<double, 12, 1> &) --> void", pybind11::arg("A"));
	}
	{ // mrpt::math::CMatrixFixed file:mrpt/math/CMatrixFixed.h line:34
		pybind11::class_<mrpt::math::CMatrixFixed<float,4UL,1UL>, std::shared_ptr<mrpt::math::CMatrixFixed<float,4UL,1UL>>> cl(M("mrpt::math"), "CMatrixFixed_float_4UL_1UL_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixFixed<float,4UL,1UL>(); } ) );
		cl.def( pybind11::init<enum mrpt::math::TConstructorFlags_Matrices>(), pybind11::arg("") );

		cl.def( pybind11::init<const float *>(), pybind11::arg("data") );

		cl.def( pybind11::init<const int, const int>(), pybind11::arg("rows"), pybind11::arg("cols") );

		cl.def( pybind11::init( [](mrpt::math::CMatrixFixed<float,4UL,1UL> const &o){ return new mrpt::math::CMatrixFixed<float,4UL,1UL>(o); } ) );
		cl.def("loadFromRawPointer", (void (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)(const float *)) &mrpt::math::CMatrixFixed<float, 4, 1>::loadFromRawPointer, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::loadFromRawPointer(const float *) --> void", pybind11::arg("data"));
		cl.def("setSize", [](mrpt::math::CMatrixFixed<float,4UL,1UL> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)(size_t, size_t, bool)) &mrpt::math::CMatrixFixed<float, 4, 1>::setSize, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("swap", (void (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)(class mrpt::math::CMatrixFixed<float, 4, 1> &)) &mrpt::math::CMatrixFixed<float, 4, 1>::swap, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::swap(class mrpt::math::CMatrixFixed<float, 4, 1> &) --> void", pybind11::arg("o"));
		cl.def("conservativeResize", (void (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<float, 4, 1>::conservativeResize, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::conservativeResize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)(size_t)) &mrpt::math::CMatrixFixed<float, 4, 1>::resize, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::resize(size_t) --> void", pybind11::arg("n"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<float, 4, 1>::resize, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::resize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("rows", (int (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<float, 4, 1>::rows, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<float, 4, 1>::cols, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::cols() const --> int");
		cl.def("data", (float * (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)()) &mrpt::math::CMatrixFixed<float, 4, 1>::data, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::data() --> float *", pybind11::return_value_policy::automatic);
		cl.def("__call__", (float & (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)(int, int)) &mrpt::math::CMatrixFixed<float, 4, 1>::operator(), "C++: mrpt::math::CMatrixFixed<float, 4, 1>::operator()(int, int) --> float &", pybind11::return_value_policy::automatic, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__call__", (float & (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)(int)) &mrpt::math::CMatrixFixed<float, 4, 1>::operator(), "C++: mrpt::math::CMatrixFixed<float, 4, 1>::operator()(int) --> float &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("__getitem__", (float & (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)(int)) &mrpt::math::CMatrixFixed<float, 4, 1>::operator[], "C++: mrpt::math::CMatrixFixed<float, 4, 1>::operator[](int) --> float &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("cast_float", (class mrpt::math::CMatrixFixed<float, 4, 1> (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<float, 4, 1>::cast_float, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::cast_float() const --> class mrpt::math::CMatrixFixed<float, 4, 1>");
		cl.def("sum_At", (void (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)(const class mrpt::math::CMatrixFixed<float, 4, 1> &)) &mrpt::math::CMatrixFixed<float, 4, 1>::sum_At, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::sum_At(const class mrpt::math::CMatrixFixed<float, 4, 1> &) --> void", pybind11::arg("A"));
		cl.def("assign", (class mrpt::math::CMatrixFixed<float, 4, 1> & (mrpt::math::CMatrixFixed<float,4UL,1UL>::*)(const class mrpt::math::CMatrixFixed<float, 4, 1> &)) &mrpt::math::CMatrixFixed<float, 4, 1>::operator=, "C++: mrpt::math::CMatrixFixed<float, 4, 1>::operator=(const class mrpt::math::CMatrixFixed<float, 4, 1> &) --> class mrpt::math::CMatrixFixed<float, 4, 1> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	// mrpt::math::TMatrixTextFileFormat file:mrpt/math/MatrixVectorBase.h line:35
	pybind11::enum_<mrpt::math::TMatrixTextFileFormat>(M("mrpt::math"), "TMatrixTextFileFormat", pybind11::arithmetic(), "Selection of the number format in MatrixVectorBase::saveToTextFile()\n \n")
		.value("MATRIX_FORMAT_ENG", mrpt::math::MATRIX_FORMAT_ENG)
		.value("MATRIX_FORMAT_FIXED", mrpt::math::MATRIX_FORMAT_FIXED)
		.value("MATRIX_FORMAT_INT", mrpt::math::MATRIX_FORMAT_INT)
		.export_values();

;

	{ // mrpt::math::MatrixVectorBase file:mrpt/math/MatrixVectorBase.h line:57
		pybind11::class_<mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>, std::shared_ptr<mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>>> cl(M("mrpt::math"), "MatrixVectorBase_double_mrpt_math_CMatrixFixed_double_3_3_t", "");
		cl.def( pybind11::init( [](mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>> const &o){ return new mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>(); } ) );
		cl.def("__eq__", (bool (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)(const class mrpt::math::CMatrixFixed<double, 3, 3> &) const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::operator==<mrpt::math::CMatrixFixed<double, 3, 3>>, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::operator==(const class mrpt::math::CMatrixFixed<double, 3, 3> &) const --> bool", pybind11::arg("o"));
		cl.def("fill", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)(const double &)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::fill, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::fill(const double &) --> void", pybind11::arg("val"));
		cl.def("setConstant", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)(const double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::setConstant, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::setConstant(const double) --> void", pybind11::arg("value"));
		cl.def("setConstant", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)(size_t, size_t, const double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::setConstant, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::setConstant(size_t, size_t, const double) --> void", pybind11::arg("nrows"), pybind11::arg("ncols"), pybind11::arg("value"));
		cl.def("setConstant", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)(size_t, const double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::setConstant, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::setConstant(size_t, const double) --> void", pybind11::arg("nrows"), pybind11::arg("value"));
		cl.def_static("Constant", (class mrpt::math::CMatrixFixed<double, 3, 3> (*)(const double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::Constant, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::Constant(const double) --> class mrpt::math::CMatrixFixed<double, 3, 3>", pybind11::arg("value"));
		cl.def_static("Constant", (class mrpt::math::CMatrixFixed<double, 3, 3> (*)(size_t, size_t, const double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::Constant, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::Constant(size_t, size_t, const double) --> class mrpt::math::CMatrixFixed<double, 3, 3>", pybind11::arg("nrows"), pybind11::arg("ncols"), pybind11::arg("value"));
		cl.def("assign", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)(const unsigned long, const double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::assign, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::assign(const unsigned long, const double) --> void", pybind11::arg("N"), pybind11::arg("value"));
		cl.def("setZero", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)()) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::setZero, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::setZero() --> void");
		cl.def("setZero", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)(size_t, size_t)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::setZero, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::setZero(size_t, size_t) --> void", pybind11::arg("nrows"), pybind11::arg("ncols"));
		cl.def("setZero", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)(size_t)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::setZero, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::setZero(size_t) --> void", pybind11::arg("nrows"));
		cl.def_static("Zero", (class mrpt::math::CMatrixFixed<double, 3, 3> (*)()) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::Zero, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::Zero() --> class mrpt::math::CMatrixFixed<double, 3, 3>");
		cl.def_static("Zero", (class mrpt::math::CMatrixFixed<double, 3, 3> (*)(size_t, size_t)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::Zero, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::Zero(size_t, size_t) --> class mrpt::math::CMatrixFixed<double, 3, 3>", pybind11::arg("nrows"), pybind11::arg("ncols"));
		cl.def("coeffRef", (double & (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)(int, int)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::coeffRef, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::coeffRef(int, int) --> double &", pybind11::return_value_policy::automatic, pybind11::arg("r"), pybind11::arg("c"));
		cl.def("coeff", (const double & (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)(int, int) const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::coeff, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::coeff(int, int) const --> const double &", pybind11::return_value_policy::automatic, pybind11::arg("r"), pybind11::arg("c"));
		cl.def("isSquare", (bool (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::isSquare, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::isSquare() const --> bool");
		cl.def("empty", (bool (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::empty, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::empty() const --> bool");
		cl.def("norm_inf", (double (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::norm_inf, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::norm_inf() const --> double");
		cl.def("norm", (double (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::norm, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::norm() const --> double");
		cl.def("__iadd__", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)(double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::operator+=, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::operator+=(double) --> void", pybind11::arg("s"));
		cl.def("__isub__", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)(double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::operator-=, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::operator-=(double) --> void", pybind11::arg("s"));
		cl.def("__imul__", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)(double)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::operator*=, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::operator*=(double) --> void", pybind11::arg("s"));
		cl.def("__add__", (class mrpt::math::CMatrixFixed<double, 3, 3> (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)(const class mrpt::math::CMatrixFixed<double, 3, 3> &) const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::operator+, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::operator+(const class mrpt::math::CMatrixFixed<double, 3, 3> &) const --> class mrpt::math::CMatrixFixed<double, 3, 3>", pybind11::arg("m2"));
		cl.def("__iadd__", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)(const class mrpt::math::CMatrixFixed<double, 3, 3> &)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::operator+=, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::operator+=(const class mrpt::math::CMatrixFixed<double, 3, 3> &) --> void", pybind11::arg("m2"));
		cl.def("__isub__", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)(const class mrpt::math::CMatrixFixed<double, 3, 3> &)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::operator-=, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::operator-=(const class mrpt::math::CMatrixFixed<double, 3, 3> &) --> void", pybind11::arg("m2"));
		cl.def("sum", (double (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::sum, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::sum() const --> double");
		cl.def("sum_abs", (double (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::sum_abs, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::sum_abs() const --> double");
		cl.def("asString", (std::string (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)() const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::asString, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::asString() const --> std::string");
		cl.def("inMatlabFormat", [](mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>> const &o) -> std::string { return o.inMatlabFormat(); }, "");
		cl.def("inMatlabFormat", (std::string (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)(const unsigned long) const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::inMatlabFormat, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::inMatlabFormat(const unsigned long) const --> std::string", pybind11::arg("decimal_digits"));
		cl.def("saveToTextFile", [](mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>> const &o, const std::string & a0) -> void { return o.saveToTextFile(a0); }, "", pybind11::arg("file"));
		cl.def("saveToTextFile", [](mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>> const &o, const std::string & a0, enum mrpt::math::TMatrixTextFileFormat const & a1) -> void { return o.saveToTextFile(a0, a1); }, "", pybind11::arg("file"), pybind11::arg("fileFormat"));
		cl.def("saveToTextFile", [](mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>> const &o, const std::string & a0, enum mrpt::math::TMatrixTextFileFormat const & a1, bool const & a2) -> void { return o.saveToTextFile(a0, a1, a2); }, "", pybind11::arg("file"), pybind11::arg("fileFormat"), pybind11::arg("appendMRPTHeader"));
		cl.def("saveToTextFile", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)(const std::string &, enum mrpt::math::TMatrixTextFileFormat, bool, const std::string &) const) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::saveToTextFile, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::saveToTextFile(const std::string &, enum mrpt::math::TMatrixTextFileFormat, bool, const std::string &) const --> void", pybind11::arg("file"), pybind11::arg("fileFormat"), pybind11::arg("appendMRPTHeader"), pybind11::arg("userHeader"));
		cl.def("loadFromTextFile", (void (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)(const std::string &)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::loadFromTextFile, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::loadFromTextFile(const std::string &) --> void", pybind11::arg("file"));
		cl.def("assign", (class mrpt::math::MatrixVectorBase<double, class mrpt::math::CMatrixFixed<double, 3, 3> > & (mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>>::*)(const class mrpt::math::MatrixVectorBase<double, class mrpt::math::CMatrixFixed<double, 3, 3> > &)) &mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::operator=, "C++: mrpt::math::MatrixVectorBase<double, mrpt::math::CMatrixFixed<double, 3, 3>>::operator=(const class mrpt::math::MatrixVectorBase<double, class mrpt::math::CMatrixFixed<double, 3, 3> > &) --> class mrpt::math::MatrixVectorBase<double, class mrpt::math::CMatrixFixed<double, 3, 3> > &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		cl.def("__str__", [](mrpt::math::MatrixVectorBase<double,mrpt::math::CMatrixFixed<double, 3, 3>> const &o) -> std::string { std::ostringstream s; using namespace mrpt::math; s << o; return s.str(); } );
	}
}
