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

void bind_mrpt_math_CMatrixFixed_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::CMatrixFixed file:mrpt/math/CMatrixFixed.h line:34
		pybind11::class_<mrpt::math::CMatrixFixed<double,3UL,4UL>, std::shared_ptr<mrpt::math::CMatrixFixed<double,3UL,4UL>>> cl(M("mrpt::math"), "CMatrixFixed_double_3UL_4UL_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixFixed<double,3UL,4UL>(); } ) );
		cl.def( pybind11::init<enum mrpt::math::TConstructorFlags_Matrices>(), pybind11::arg("") );

		cl.def( pybind11::init<const double *>(), pybind11::arg("data") );

		cl.def( pybind11::init<const int, const int>(), pybind11::arg("rows"), pybind11::arg("cols") );

		cl.def( pybind11::init( [](mrpt::math::CMatrixFixed<double,3UL,4UL> const &o){ return new mrpt::math::CMatrixFixed<double,3UL,4UL>(o); } ) );
		cl.def("loadFromRawPointer", (void (mrpt::math::CMatrixFixed<double,3UL,4UL>::*)(const double *)) &mrpt::math::CMatrixFixed<double, 3, 4>::loadFromRawPointer, "C++: mrpt::math::CMatrixFixed<double, 3, 4>::loadFromRawPointer(const double *) --> void", pybind11::arg("data"));
		cl.def("setSize", [](mrpt::math::CMatrixFixed<double,3UL,4UL> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CMatrixFixed<double,3UL,4UL>::*)(size_t, size_t, bool)) &mrpt::math::CMatrixFixed<double, 3, 4>::setSize, "C++: mrpt::math::CMatrixFixed<double, 3, 4>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("swap", (void (mrpt::math::CMatrixFixed<double,3UL,4UL>::*)(class mrpt::math::CMatrixFixed<double, 3, 4> &)) &mrpt::math::CMatrixFixed<double, 3, 4>::swap, "C++: mrpt::math::CMatrixFixed<double, 3, 4>::swap(class mrpt::math::CMatrixFixed<double, 3, 4> &) --> void", pybind11::arg("o"));
		cl.def("conservativeResize", (void (mrpt::math::CMatrixFixed<double,3UL,4UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<double, 3, 4>::conservativeResize, "C++: mrpt::math::CMatrixFixed<double, 3, 4>::conservativeResize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<double,3UL,4UL>::*)(size_t)) &mrpt::math::CMatrixFixed<double, 3, 4>::resize, "C++: mrpt::math::CMatrixFixed<double, 3, 4>::resize(size_t) --> void", pybind11::arg("n"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<double,3UL,4UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<double, 3, 4>::resize, "C++: mrpt::math::CMatrixFixed<double, 3, 4>::resize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("rows", (int (mrpt::math::CMatrixFixed<double,3UL,4UL>::*)() const) &mrpt::math::CMatrixFixed<double, 3, 4>::rows, "C++: mrpt::math::CMatrixFixed<double, 3, 4>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CMatrixFixed<double,3UL,4UL>::*)() const) &mrpt::math::CMatrixFixed<double, 3, 4>::cols, "C++: mrpt::math::CMatrixFixed<double, 3, 4>::cols() const --> int");
		cl.def("data", (double * (mrpt::math::CMatrixFixed<double,3UL,4UL>::*)()) &mrpt::math::CMatrixFixed<double, 3, 4>::data, "C++: mrpt::math::CMatrixFixed<double, 3, 4>::data() --> double *", pybind11::return_value_policy::automatic);
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,3UL,4UL>::*)(int, int)) &mrpt::math::CMatrixFixed<double, 3, 4>::operator(), "C++: mrpt::math::CMatrixFixed<double, 3, 4>::operator()(int, int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,3UL,4UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 3, 4>::operator(), "C++: mrpt::math::CMatrixFixed<double, 3, 4>::operator()(int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("__getitem__", (double & (mrpt::math::CMatrixFixed<double,3UL,4UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 3, 4>::operator[], "C++: mrpt::math::CMatrixFixed<double, 3, 4>::operator[](int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("sum_At", (void (mrpt::math::CMatrixFixed<double,3UL,4UL>::*)(const class mrpt::math::CMatrixFixed<double, 3, 4> &)) &mrpt::math::CMatrixFixed<double, 3, 4>::sum_At, "C++: mrpt::math::CMatrixFixed<double, 3, 4>::sum_At(const class mrpt::math::CMatrixFixed<double, 3, 4> &) --> void", pybind11::arg("A"));
	}
	{ // mrpt::math::CMatrixFixed file:mrpt/math/CMatrixFixed.h line:34
		pybind11::class_<mrpt::math::CMatrixFixed<double,3UL,6UL>, std::shared_ptr<mrpt::math::CMatrixFixed<double,3UL,6UL>>> cl(M("mrpt::math"), "CMatrixFixed_double_3UL_6UL_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixFixed<double,3UL,6UL>(); } ) );
		cl.def( pybind11::init<enum mrpt::math::TConstructorFlags_Matrices>(), pybind11::arg("") );

		cl.def( pybind11::init<const double *>(), pybind11::arg("data") );

		cl.def( pybind11::init<const int, const int>(), pybind11::arg("rows"), pybind11::arg("cols") );

		cl.def( pybind11::init( [](mrpt::math::CMatrixFixed<double,3UL,6UL> const &o){ return new mrpt::math::CMatrixFixed<double,3UL,6UL>(o); } ) );
		cl.def("loadFromRawPointer", (void (mrpt::math::CMatrixFixed<double,3UL,6UL>::*)(const double *)) &mrpt::math::CMatrixFixed<double, 3, 6>::loadFromRawPointer, "C++: mrpt::math::CMatrixFixed<double, 3, 6>::loadFromRawPointer(const double *) --> void", pybind11::arg("data"));
		cl.def("setSize", [](mrpt::math::CMatrixFixed<double,3UL,6UL> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CMatrixFixed<double,3UL,6UL>::*)(size_t, size_t, bool)) &mrpt::math::CMatrixFixed<double, 3, 6>::setSize, "C++: mrpt::math::CMatrixFixed<double, 3, 6>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("swap", (void (mrpt::math::CMatrixFixed<double,3UL,6UL>::*)(class mrpt::math::CMatrixFixed<double, 3, 6> &)) &mrpt::math::CMatrixFixed<double, 3, 6>::swap, "C++: mrpt::math::CMatrixFixed<double, 3, 6>::swap(class mrpt::math::CMatrixFixed<double, 3, 6> &) --> void", pybind11::arg("o"));
		cl.def("conservativeResize", (void (mrpt::math::CMatrixFixed<double,3UL,6UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<double, 3, 6>::conservativeResize, "C++: mrpt::math::CMatrixFixed<double, 3, 6>::conservativeResize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<double,3UL,6UL>::*)(size_t)) &mrpt::math::CMatrixFixed<double, 3, 6>::resize, "C++: mrpt::math::CMatrixFixed<double, 3, 6>::resize(size_t) --> void", pybind11::arg("n"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<double,3UL,6UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<double, 3, 6>::resize, "C++: mrpt::math::CMatrixFixed<double, 3, 6>::resize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("rows", (int (mrpt::math::CMatrixFixed<double,3UL,6UL>::*)() const) &mrpt::math::CMatrixFixed<double, 3, 6>::rows, "C++: mrpt::math::CMatrixFixed<double, 3, 6>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CMatrixFixed<double,3UL,6UL>::*)() const) &mrpt::math::CMatrixFixed<double, 3, 6>::cols, "C++: mrpt::math::CMatrixFixed<double, 3, 6>::cols() const --> int");
		cl.def("data", (double * (mrpt::math::CMatrixFixed<double,3UL,6UL>::*)()) &mrpt::math::CMatrixFixed<double, 3, 6>::data, "C++: mrpt::math::CMatrixFixed<double, 3, 6>::data() --> double *", pybind11::return_value_policy::automatic);
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,3UL,6UL>::*)(int, int)) &mrpt::math::CMatrixFixed<double, 3, 6>::operator(), "C++: mrpt::math::CMatrixFixed<double, 3, 6>::operator()(int, int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,3UL,6UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 3, 6>::operator(), "C++: mrpt::math::CMatrixFixed<double, 3, 6>::operator()(int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("__getitem__", (double & (mrpt::math::CMatrixFixed<double,3UL,6UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 3, 6>::operator[], "C++: mrpt::math::CMatrixFixed<double, 3, 6>::operator[](int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("sum_At", (void (mrpt::math::CMatrixFixed<double,3UL,6UL>::*)(const class mrpt::math::CMatrixFixed<double, 3, 6> &)) &mrpt::math::CMatrixFixed<double, 3, 6>::sum_At, "C++: mrpt::math::CMatrixFixed<double, 3, 6>::sum_At(const class mrpt::math::CMatrixFixed<double, 3, 6> &) --> void", pybind11::arg("A"));
	}
	{ // mrpt::math::CMatrixFixed file:mrpt/math/CMatrixFixed.h line:34
		pybind11::class_<mrpt::math::CMatrixFixed<double,2UL,1UL>, std::shared_ptr<mrpt::math::CMatrixFixed<double,2UL,1UL>>> cl(M("mrpt::math"), "CMatrixFixed_double_2UL_1UL_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixFixed<double,2UL,1UL>(); } ) );
		cl.def( pybind11::init<enum mrpt::math::TConstructorFlags_Matrices>(), pybind11::arg("") );

		cl.def( pybind11::init<const double *>(), pybind11::arg("data") );

		cl.def( pybind11::init<const int, const int>(), pybind11::arg("rows"), pybind11::arg("cols") );

		cl.def( pybind11::init( [](mrpt::math::CMatrixFixed<double,2UL,1UL> const &o){ return new mrpt::math::CMatrixFixed<double,2UL,1UL>(o); } ) );
		cl.def("loadFromRawPointer", (void (mrpt::math::CMatrixFixed<double,2UL,1UL>::*)(const double *)) &mrpt::math::CMatrixFixed<double, 2, 1>::loadFromRawPointer, "C++: mrpt::math::CMatrixFixed<double, 2, 1>::loadFromRawPointer(const double *) --> void", pybind11::arg("data"));
		cl.def("setSize", [](mrpt::math::CMatrixFixed<double,2UL,1UL> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CMatrixFixed<double,2UL,1UL>::*)(size_t, size_t, bool)) &mrpt::math::CMatrixFixed<double, 2, 1>::setSize, "C++: mrpt::math::CMatrixFixed<double, 2, 1>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("swap", (void (mrpt::math::CMatrixFixed<double,2UL,1UL>::*)(class mrpt::math::CMatrixFixed<double, 2, 1> &)) &mrpt::math::CMatrixFixed<double, 2, 1>::swap, "C++: mrpt::math::CMatrixFixed<double, 2, 1>::swap(class mrpt::math::CMatrixFixed<double, 2, 1> &) --> void", pybind11::arg("o"));
		cl.def("conservativeResize", (void (mrpt::math::CMatrixFixed<double,2UL,1UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<double, 2, 1>::conservativeResize, "C++: mrpt::math::CMatrixFixed<double, 2, 1>::conservativeResize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<double,2UL,1UL>::*)(size_t)) &mrpt::math::CMatrixFixed<double, 2, 1>::resize, "C++: mrpt::math::CMatrixFixed<double, 2, 1>::resize(size_t) --> void", pybind11::arg("n"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<double,2UL,1UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<double, 2, 1>::resize, "C++: mrpt::math::CMatrixFixed<double, 2, 1>::resize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("rows", (int (mrpt::math::CMatrixFixed<double,2UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<double, 2, 1>::rows, "C++: mrpt::math::CMatrixFixed<double, 2, 1>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CMatrixFixed<double,2UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<double, 2, 1>::cols, "C++: mrpt::math::CMatrixFixed<double, 2, 1>::cols() const --> int");
		cl.def("data", (double * (mrpt::math::CMatrixFixed<double,2UL,1UL>::*)()) &mrpt::math::CMatrixFixed<double, 2, 1>::data, "C++: mrpt::math::CMatrixFixed<double, 2, 1>::data() --> double *", pybind11::return_value_policy::automatic);
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,2UL,1UL>::*)(int, int)) &mrpt::math::CMatrixFixed<double, 2, 1>::operator(), "C++: mrpt::math::CMatrixFixed<double, 2, 1>::operator()(int, int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,2UL,1UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 2, 1>::operator(), "C++: mrpt::math::CMatrixFixed<double, 2, 1>::operator()(int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("__getitem__", (double & (mrpt::math::CMatrixFixed<double,2UL,1UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 2, 1>::operator[], "C++: mrpt::math::CMatrixFixed<double, 2, 1>::operator[](int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("sum_At", (void (mrpt::math::CMatrixFixed<double,2UL,1UL>::*)(const class mrpt::math::CMatrixFixed<double, 2, 1> &)) &mrpt::math::CMatrixFixed<double, 2, 1>::sum_At, "C++: mrpt::math::CMatrixFixed<double, 2, 1>::sum_At(const class mrpt::math::CMatrixFixed<double, 2, 1> &) --> void", pybind11::arg("A"));
		cl.def("assign", (class mrpt::math::CMatrixFixed<double, 2, 1> & (mrpt::math::CMatrixFixed<double,2UL,1UL>::*)(const class mrpt::math::CMatrixFixed<double, 2, 1> &)) &mrpt::math::CMatrixFixed<double, 2, 1>::operator=, "C++: mrpt::math::CMatrixFixed<double, 2, 1>::operator=(const class mrpt::math::CMatrixFixed<double, 2, 1> &) --> class mrpt::math::CMatrixFixed<double, 2, 1> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::math::CMatrixFixed file:mrpt/math/CMatrixFixed.h line:34
		pybind11::class_<mrpt::math::CMatrixFixed<double,6UL,1UL>, std::shared_ptr<mrpt::math::CMatrixFixed<double,6UL,1UL>>> cl(M("mrpt::math"), "CMatrixFixed_double_6UL_1UL_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixFixed<double,6UL,1UL>(); } ) );
		cl.def( pybind11::init<enum mrpt::math::TConstructorFlags_Matrices>(), pybind11::arg("") );

		cl.def( pybind11::init<const double *>(), pybind11::arg("data") );

		cl.def( pybind11::init<const int, const int>(), pybind11::arg("rows"), pybind11::arg("cols") );

		cl.def( pybind11::init( [](mrpt::math::CMatrixFixed<double,6UL,1UL> const &o){ return new mrpt::math::CMatrixFixed<double,6UL,1UL>(o); } ) );
		cl.def("loadFromRawPointer", (void (mrpt::math::CMatrixFixed<double,6UL,1UL>::*)(const double *)) &mrpt::math::CMatrixFixed<double, 6, 1>::loadFromRawPointer, "C++: mrpt::math::CMatrixFixed<double, 6, 1>::loadFromRawPointer(const double *) --> void", pybind11::arg("data"));
		cl.def("setSize", [](mrpt::math::CMatrixFixed<double,6UL,1UL> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CMatrixFixed<double,6UL,1UL>::*)(size_t, size_t, bool)) &mrpt::math::CMatrixFixed<double, 6, 1>::setSize, "C++: mrpt::math::CMatrixFixed<double, 6, 1>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("swap", (void (mrpt::math::CMatrixFixed<double,6UL,1UL>::*)(class mrpt::math::CMatrixFixed<double, 6, 1> &)) &mrpt::math::CMatrixFixed<double, 6, 1>::swap, "C++: mrpt::math::CMatrixFixed<double, 6, 1>::swap(class mrpt::math::CMatrixFixed<double, 6, 1> &) --> void", pybind11::arg("o"));
		cl.def("conservativeResize", (void (mrpt::math::CMatrixFixed<double,6UL,1UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<double, 6, 1>::conservativeResize, "C++: mrpt::math::CMatrixFixed<double, 6, 1>::conservativeResize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<double,6UL,1UL>::*)(size_t)) &mrpt::math::CMatrixFixed<double, 6, 1>::resize, "C++: mrpt::math::CMatrixFixed<double, 6, 1>::resize(size_t) --> void", pybind11::arg("n"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<double,6UL,1UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<double, 6, 1>::resize, "C++: mrpt::math::CMatrixFixed<double, 6, 1>::resize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("rows", (int (mrpt::math::CMatrixFixed<double,6UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<double, 6, 1>::rows, "C++: mrpt::math::CMatrixFixed<double, 6, 1>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CMatrixFixed<double,6UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<double, 6, 1>::cols, "C++: mrpt::math::CMatrixFixed<double, 6, 1>::cols() const --> int");
		cl.def("data", (double * (mrpt::math::CMatrixFixed<double,6UL,1UL>::*)()) &mrpt::math::CMatrixFixed<double, 6, 1>::data, "C++: mrpt::math::CMatrixFixed<double, 6, 1>::data() --> double *", pybind11::return_value_policy::automatic);
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,6UL,1UL>::*)(int, int)) &mrpt::math::CMatrixFixed<double, 6, 1>::operator(), "C++: mrpt::math::CMatrixFixed<double, 6, 1>::operator()(int, int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,6UL,1UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 6, 1>::operator(), "C++: mrpt::math::CMatrixFixed<double, 6, 1>::operator()(int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("__getitem__", (double & (mrpt::math::CMatrixFixed<double,6UL,1UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 6, 1>::operator[], "C++: mrpt::math::CMatrixFixed<double, 6, 1>::operator[](int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("sum_At", (void (mrpt::math::CMatrixFixed<double,6UL,1UL>::*)(const class mrpt::math::CMatrixFixed<double, 6, 1> &)) &mrpt::math::CMatrixFixed<double, 6, 1>::sum_At, "C++: mrpt::math::CMatrixFixed<double, 6, 1>::sum_At(const class mrpt::math::CMatrixFixed<double, 6, 1> &) --> void", pybind11::arg("A"));
	}
}
