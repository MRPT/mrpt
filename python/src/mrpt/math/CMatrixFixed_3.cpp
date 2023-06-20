#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
#include <sstream> // __str__

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

void bind_mrpt_math_CMatrixFixed_3(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::CMatrixFixed file:mrpt/math/CMatrixFixed.h line:34
		pybind11::class_<mrpt::math::CMatrixFixed<double,4UL,3UL>, std::shared_ptr<mrpt::math::CMatrixFixed<double,4UL,3UL>>> cl(M("mrpt::math"), "CMatrixFixed_double_4UL_3UL_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixFixed<double,4UL,3UL>(); } ) );
		cl.def( pybind11::init<enum mrpt::math::TConstructorFlags_Matrices>(), pybind11::arg("") );

		cl.def( pybind11::init<const double *>(), pybind11::arg("data") );

		cl.def( pybind11::init<const int, const int>(), pybind11::arg("rows"), pybind11::arg("cols") );

		cl.def( pybind11::init( [](mrpt::math::CMatrixFixed<double,4UL,3UL> const &o){ return new mrpt::math::CMatrixFixed<double,4UL,3UL>(o); } ) );
		cl.def("loadFromRawPointer", (void (mrpt::math::CMatrixFixed<double,4UL,3UL>::*)(const double *)) &mrpt::math::CMatrixFixed<double, 4, 3>::loadFromRawPointer, "C++: mrpt::math::CMatrixFixed<double, 4, 3>::loadFromRawPointer(const double *) --> void", pybind11::arg("data"));
		cl.def("setSize", [](mrpt::math::CMatrixFixed<double,4UL,3UL> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CMatrixFixed<double,4UL,3UL>::*)(size_t, size_t, bool)) &mrpt::math::CMatrixFixed<double, 4, 3>::setSize, "C++: mrpt::math::CMatrixFixed<double, 4, 3>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("swap", (void (mrpt::math::CMatrixFixed<double,4UL,3UL>::*)(class mrpt::math::CMatrixFixed<double, 4, 3> &)) &mrpt::math::CMatrixFixed<double, 4, 3>::swap, "C++: mrpt::math::CMatrixFixed<double, 4, 3>::swap(class mrpt::math::CMatrixFixed<double, 4, 3> &) --> void", pybind11::arg("o"));
		cl.def("conservativeResize", (void (mrpt::math::CMatrixFixed<double,4UL,3UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<double, 4, 3>::conservativeResize, "C++: mrpt::math::CMatrixFixed<double, 4, 3>::conservativeResize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<double,4UL,3UL>::*)(size_t)) &mrpt::math::CMatrixFixed<double, 4, 3>::resize, "C++: mrpt::math::CMatrixFixed<double, 4, 3>::resize(size_t) --> void", pybind11::arg("n"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<double,4UL,3UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<double, 4, 3>::resize, "C++: mrpt::math::CMatrixFixed<double, 4, 3>::resize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("rows", (int (mrpt::math::CMatrixFixed<double,4UL,3UL>::*)() const) &mrpt::math::CMatrixFixed<double, 4, 3>::rows, "C++: mrpt::math::CMatrixFixed<double, 4, 3>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CMatrixFixed<double,4UL,3UL>::*)() const) &mrpt::math::CMatrixFixed<double, 4, 3>::cols, "C++: mrpt::math::CMatrixFixed<double, 4, 3>::cols() const --> int");
		cl.def("data", (double * (mrpt::math::CMatrixFixed<double,4UL,3UL>::*)()) &mrpt::math::CMatrixFixed<double, 4, 3>::data, "C++: mrpt::math::CMatrixFixed<double, 4, 3>::data() --> double *", pybind11::return_value_policy::automatic);
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,4UL,3UL>::*)(int, int)) &mrpt::math::CMatrixFixed<double, 4, 3>::operator(), "C++: mrpt::math::CMatrixFixed<double, 4, 3>::operator()(int, int) --> double &", pybind11::return_value_policy::automatic, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,4UL,3UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 4, 3>::operator(), "C++: mrpt::math::CMatrixFixed<double, 4, 3>::operator()(int) --> double &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("__getitem__", (double & (mrpt::math::CMatrixFixed<double,4UL,3UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 4, 3>::operator[], "C++: mrpt::math::CMatrixFixed<double, 4, 3>::operator[](int) --> double &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("sum_At", (void (mrpt::math::CMatrixFixed<double,4UL,3UL>::*)(const class mrpt::math::CMatrixFixed<double, 4, 3> &)) &mrpt::math::CMatrixFixed<double, 4, 3>::sum_At, "C++: mrpt::math::CMatrixFixed<double, 4, 3>::sum_At(const class mrpt::math::CMatrixFixed<double, 4, 3> &) --> void", pybind11::arg("A"));
	}
	{ // mrpt::math::CMatrixFixed file:mrpt/math/CMatrixFixed.h line:34
		pybind11::class_<mrpt::math::CMatrixFixed<float,3UL,3UL>, std::shared_ptr<mrpt::math::CMatrixFixed<float,3UL,3UL>>> cl(M("mrpt::math"), "CMatrixFixed_float_3UL_3UL_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixFixed<float,3UL,3UL>(); } ) );
		cl.def( pybind11::init<enum mrpt::math::TConstructorFlags_Matrices>(), pybind11::arg("") );

		cl.def( pybind11::init<const float *>(), pybind11::arg("data") );

		cl.def( pybind11::init<const int, const int>(), pybind11::arg("rows"), pybind11::arg("cols") );

		cl.def("loadFromRawPointer", (void (mrpt::math::CMatrixFixed<float,3UL,3UL>::*)(const float *)) &mrpt::math::CMatrixFixed<float, 3, 3>::loadFromRawPointer, "C++: mrpt::math::CMatrixFixed<float, 3, 3>::loadFromRawPointer(const float *) --> void", pybind11::arg("data"));
		cl.def("setSize", [](mrpt::math::CMatrixFixed<float,3UL,3UL> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CMatrixFixed<float,3UL,3UL>::*)(size_t, size_t, bool)) &mrpt::math::CMatrixFixed<float, 3, 3>::setSize, "C++: mrpt::math::CMatrixFixed<float, 3, 3>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("swap", (void (mrpt::math::CMatrixFixed<float,3UL,3UL>::*)(class mrpt::math::CMatrixFixed<float, 3, 3> &)) &mrpt::math::CMatrixFixed<float, 3, 3>::swap, "C++: mrpt::math::CMatrixFixed<float, 3, 3>::swap(class mrpt::math::CMatrixFixed<float, 3, 3> &) --> void", pybind11::arg("o"));
		cl.def("conservativeResize", (void (mrpt::math::CMatrixFixed<float,3UL,3UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<float, 3, 3>::conservativeResize, "C++: mrpt::math::CMatrixFixed<float, 3, 3>::conservativeResize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<float,3UL,3UL>::*)(size_t)) &mrpt::math::CMatrixFixed<float, 3, 3>::resize, "C++: mrpt::math::CMatrixFixed<float, 3, 3>::resize(size_t) --> void", pybind11::arg("n"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<float,3UL,3UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<float, 3, 3>::resize, "C++: mrpt::math::CMatrixFixed<float, 3, 3>::resize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("rows", (int (mrpt::math::CMatrixFixed<float,3UL,3UL>::*)() const) &mrpt::math::CMatrixFixed<float, 3, 3>::rows, "C++: mrpt::math::CMatrixFixed<float, 3, 3>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CMatrixFixed<float,3UL,3UL>::*)() const) &mrpt::math::CMatrixFixed<float, 3, 3>::cols, "C++: mrpt::math::CMatrixFixed<float, 3, 3>::cols() const --> int");
		cl.def("data", (float * (mrpt::math::CMatrixFixed<float,3UL,3UL>::*)()) &mrpt::math::CMatrixFixed<float, 3, 3>::data, "C++: mrpt::math::CMatrixFixed<float, 3, 3>::data() --> float *", pybind11::return_value_policy::automatic);
		cl.def("__call__", (float & (mrpt::math::CMatrixFixed<float,3UL,3UL>::*)(int, int)) &mrpt::math::CMatrixFixed<float, 3, 3>::operator(), "C++: mrpt::math::CMatrixFixed<float, 3, 3>::operator()(int, int) --> float &", pybind11::return_value_policy::automatic, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__call__", (float & (mrpt::math::CMatrixFixed<float,3UL,3UL>::*)(int)) &mrpt::math::CMatrixFixed<float, 3, 3>::operator(), "C++: mrpt::math::CMatrixFixed<float, 3, 3>::operator()(int) --> float &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("__getitem__", (float & (mrpt::math::CMatrixFixed<float,3UL,3UL>::*)(int)) &mrpt::math::CMatrixFixed<float, 3, 3>::operator[], "C++: mrpt::math::CMatrixFixed<float, 3, 3>::operator[](int) --> float &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("cast_float", (class mrpt::math::CMatrixFixed<float, 3, 3> (mrpt::math::CMatrixFixed<float,3UL,3UL>::*)() const) &mrpt::math::CMatrixFixed<float, 3, 3>::cast_float, "C++: mrpt::math::CMatrixFixed<float, 3, 3>::cast_float() const --> class mrpt::math::CMatrixFixed<float, 3, 3>");
		cl.def("sum_At", (void (mrpt::math::CMatrixFixed<float,3UL,3UL>::*)(const class mrpt::math::CMatrixFixed<float, 3, 3> &)) &mrpt::math::CMatrixFixed<float, 3, 3>::sum_At, "C++: mrpt::math::CMatrixFixed<float, 3, 3>::sum_At(const class mrpt::math::CMatrixFixed<float, 3, 3> &) --> void", pybind11::arg("A"));
	}
	{ // mrpt::math::CMatrixFixed file:mrpt/math/CMatrixFixed.h line:34
		pybind11::class_<mrpt::math::CMatrixFixed<float,4UL,4UL>, std::shared_ptr<mrpt::math::CMatrixFixed<float,4UL,4UL>>> cl(M("mrpt::math"), "CMatrixFixed_float_4UL_4UL_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixFixed<float,4UL,4UL>(); } ) );
		cl.def( pybind11::init<enum mrpt::math::TConstructorFlags_Matrices>(), pybind11::arg("") );

		cl.def( pybind11::init<const float *>(), pybind11::arg("data") );

		cl.def( pybind11::init<const int, const int>(), pybind11::arg("rows"), pybind11::arg("cols") );

		cl.def( pybind11::init( [](mrpt::math::CMatrixFixed<float,4UL,4UL> const &o){ return new mrpt::math::CMatrixFixed<float,4UL,4UL>(o); } ) );
		cl.def("loadFromRawPointer", (void (mrpt::math::CMatrixFixed<float,4UL,4UL>::*)(const float *)) &mrpt::math::CMatrixFixed<float, 4, 4>::loadFromRawPointer, "C++: mrpt::math::CMatrixFixed<float, 4, 4>::loadFromRawPointer(const float *) --> void", pybind11::arg("data"));
		cl.def("setSize", [](mrpt::math::CMatrixFixed<float,4UL,4UL> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CMatrixFixed<float,4UL,4UL>::*)(size_t, size_t, bool)) &mrpt::math::CMatrixFixed<float, 4, 4>::setSize, "C++: mrpt::math::CMatrixFixed<float, 4, 4>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("swap", (void (mrpt::math::CMatrixFixed<float,4UL,4UL>::*)(class mrpt::math::CMatrixFixed<float, 4, 4> &)) &mrpt::math::CMatrixFixed<float, 4, 4>::swap, "C++: mrpt::math::CMatrixFixed<float, 4, 4>::swap(class mrpt::math::CMatrixFixed<float, 4, 4> &) --> void", pybind11::arg("o"));
		cl.def("conservativeResize", (void (mrpt::math::CMatrixFixed<float,4UL,4UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<float, 4, 4>::conservativeResize, "C++: mrpt::math::CMatrixFixed<float, 4, 4>::conservativeResize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<float,4UL,4UL>::*)(size_t)) &mrpt::math::CMatrixFixed<float, 4, 4>::resize, "C++: mrpt::math::CMatrixFixed<float, 4, 4>::resize(size_t) --> void", pybind11::arg("n"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<float,4UL,4UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<float, 4, 4>::resize, "C++: mrpt::math::CMatrixFixed<float, 4, 4>::resize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("rows", (int (mrpt::math::CMatrixFixed<float,4UL,4UL>::*)() const) &mrpt::math::CMatrixFixed<float, 4, 4>::rows, "C++: mrpt::math::CMatrixFixed<float, 4, 4>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CMatrixFixed<float,4UL,4UL>::*)() const) &mrpt::math::CMatrixFixed<float, 4, 4>::cols, "C++: mrpt::math::CMatrixFixed<float, 4, 4>::cols() const --> int");
		cl.def("data", (float * (mrpt::math::CMatrixFixed<float,4UL,4UL>::*)()) &mrpt::math::CMatrixFixed<float, 4, 4>::data, "C++: mrpt::math::CMatrixFixed<float, 4, 4>::data() --> float *", pybind11::return_value_policy::automatic);
		cl.def("__call__", (float & (mrpt::math::CMatrixFixed<float,4UL,4UL>::*)(int, int)) &mrpt::math::CMatrixFixed<float, 4, 4>::operator(), "C++: mrpt::math::CMatrixFixed<float, 4, 4>::operator()(int, int) --> float &", pybind11::return_value_policy::automatic, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__call__", (float & (mrpt::math::CMatrixFixed<float,4UL,4UL>::*)(int)) &mrpt::math::CMatrixFixed<float, 4, 4>::operator(), "C++: mrpt::math::CMatrixFixed<float, 4, 4>::operator()(int) --> float &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("__getitem__", (float & (mrpt::math::CMatrixFixed<float,4UL,4UL>::*)(int)) &mrpt::math::CMatrixFixed<float, 4, 4>::operator[], "C++: mrpt::math::CMatrixFixed<float, 4, 4>::operator[](int) --> float &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("cast_float", (class mrpt::math::CMatrixFixed<float, 4, 4> (mrpt::math::CMatrixFixed<float,4UL,4UL>::*)() const) &mrpt::math::CMatrixFixed<float, 4, 4>::cast_float, "C++: mrpt::math::CMatrixFixed<float, 4, 4>::cast_float() const --> class mrpt::math::CMatrixFixed<float, 4, 4>");
		cl.def("sum_At", (void (mrpt::math::CMatrixFixed<float,4UL,4UL>::*)(const class mrpt::math::CMatrixFixed<float, 4, 4> &)) &mrpt::math::CMatrixFixed<float, 4, 4>::sum_At, "C++: mrpt::math::CMatrixFixed<float, 4, 4>::sum_At(const class mrpt::math::CMatrixFixed<float, 4, 4> &) --> void", pybind11::arg("A"));
		cl.def("assign", (class mrpt::math::CMatrixFixed<float, 4, 4> & (mrpt::math::CMatrixFixed<float,4UL,4UL>::*)(const class mrpt::math::CMatrixFixed<float, 4, 4> &)) &mrpt::math::CMatrixFixed<float, 4, 4>::operator=, "C++: mrpt::math::CMatrixFixed<float, 4, 4>::operator=(const class mrpt::math::CMatrixFixed<float, 4, 4> &) --> class mrpt::math::CMatrixFixed<float, 4, 4> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::math::CMatrixFixed file:mrpt/math/CMatrixFixed.h line:34
		pybind11::class_<mrpt::math::CMatrixFixed<float,3UL,1UL>, std::shared_ptr<mrpt::math::CMatrixFixed<float,3UL,1UL>>> cl(M("mrpt::math"), "CMatrixFixed_float_3UL_1UL_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixFixed<float,3UL,1UL>(); } ) );
		cl.def( pybind11::init<enum mrpt::math::TConstructorFlags_Matrices>(), pybind11::arg("") );

		cl.def( pybind11::init<const float *>(), pybind11::arg("data") );

		cl.def( pybind11::init<const int, const int>(), pybind11::arg("rows"), pybind11::arg("cols") );

		cl.def( pybind11::init( [](mrpt::math::CMatrixFixed<float,3UL,1UL> const &o){ return new mrpt::math::CMatrixFixed<float,3UL,1UL>(o); } ) );
		cl.def("loadFromRawPointer", (void (mrpt::math::CMatrixFixed<float,3UL,1UL>::*)(const float *)) &mrpt::math::CMatrixFixed<float, 3, 1>::loadFromRawPointer, "C++: mrpt::math::CMatrixFixed<float, 3, 1>::loadFromRawPointer(const float *) --> void", pybind11::arg("data"));
		cl.def("setSize", [](mrpt::math::CMatrixFixed<float,3UL,1UL> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CMatrixFixed<float,3UL,1UL>::*)(size_t, size_t, bool)) &mrpt::math::CMatrixFixed<float, 3, 1>::setSize, "C++: mrpt::math::CMatrixFixed<float, 3, 1>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("swap", (void (mrpt::math::CMatrixFixed<float,3UL,1UL>::*)(class mrpt::math::CMatrixFixed<float, 3, 1> &)) &mrpt::math::CMatrixFixed<float, 3, 1>::swap, "C++: mrpt::math::CMatrixFixed<float, 3, 1>::swap(class mrpt::math::CMatrixFixed<float, 3, 1> &) --> void", pybind11::arg("o"));
		cl.def("conservativeResize", (void (mrpt::math::CMatrixFixed<float,3UL,1UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<float, 3, 1>::conservativeResize, "C++: mrpt::math::CMatrixFixed<float, 3, 1>::conservativeResize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<float,3UL,1UL>::*)(size_t)) &mrpt::math::CMatrixFixed<float, 3, 1>::resize, "C++: mrpt::math::CMatrixFixed<float, 3, 1>::resize(size_t) --> void", pybind11::arg("n"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<float,3UL,1UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<float, 3, 1>::resize, "C++: mrpt::math::CMatrixFixed<float, 3, 1>::resize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("rows", (int (mrpt::math::CMatrixFixed<float,3UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<float, 3, 1>::rows, "C++: mrpt::math::CMatrixFixed<float, 3, 1>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CMatrixFixed<float,3UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<float, 3, 1>::cols, "C++: mrpt::math::CMatrixFixed<float, 3, 1>::cols() const --> int");
		cl.def("data", (float * (mrpt::math::CMatrixFixed<float,3UL,1UL>::*)()) &mrpt::math::CMatrixFixed<float, 3, 1>::data, "C++: mrpt::math::CMatrixFixed<float, 3, 1>::data() --> float *", pybind11::return_value_policy::automatic);
		cl.def("__call__", (float & (mrpt::math::CMatrixFixed<float,3UL,1UL>::*)(int, int)) &mrpt::math::CMatrixFixed<float, 3, 1>::operator(), "C++: mrpt::math::CMatrixFixed<float, 3, 1>::operator()(int, int) --> float &", pybind11::return_value_policy::automatic, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__call__", (float & (mrpt::math::CMatrixFixed<float,3UL,1UL>::*)(int)) &mrpt::math::CMatrixFixed<float, 3, 1>::operator(), "C++: mrpt::math::CMatrixFixed<float, 3, 1>::operator()(int) --> float &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("__getitem__", (float & (mrpt::math::CMatrixFixed<float,3UL,1UL>::*)(int)) &mrpt::math::CMatrixFixed<float, 3, 1>::operator[], "C++: mrpt::math::CMatrixFixed<float, 3, 1>::operator[](int) --> float &", pybind11::return_value_policy::automatic, pybind11::arg("i"));
		cl.def("cast_float", (class mrpt::math::CMatrixFixed<float, 3, 1> (mrpt::math::CMatrixFixed<float,3UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<float, 3, 1>::cast_float, "C++: mrpt::math::CMatrixFixed<float, 3, 1>::cast_float() const --> class mrpt::math::CMatrixFixed<float, 3, 1>");
		cl.def("sum_At", (void (mrpt::math::CMatrixFixed<float,3UL,1UL>::*)(const class mrpt::math::CMatrixFixed<float, 3, 1> &)) &mrpt::math::CMatrixFixed<float, 3, 1>::sum_At, "C++: mrpt::math::CMatrixFixed<float, 3, 1>::sum_At(const class mrpt::math::CMatrixFixed<float, 3, 1> &) --> void", pybind11::arg("A"));
		cl.def("assign", (class mrpt::math::CMatrixFixed<float, 3, 1> & (mrpt::math::CMatrixFixed<float,3UL,1UL>::*)(const class mrpt::math::CMatrixFixed<float, 3, 1> &)) &mrpt::math::CMatrixFixed<float, 3, 1>::operator=, "C++: mrpt::math::CMatrixFixed<float, 3, 1>::operator=(const class mrpt::math::CMatrixFixed<float, 3, 1> &) --> class mrpt::math::CMatrixFixed<float, 3, 1> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
