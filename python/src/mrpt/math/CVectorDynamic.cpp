#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/MatrixVectorBase.h>
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

void bind_mrpt_math_CVectorDynamic(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::CVectorDynamic file:mrpt/math/CVectorDynamic.h line:32
		pybind11::class_<mrpt::math::CVectorDynamic<double>, std::shared_ptr<mrpt::math::CVectorDynamic<double>>> cl(M("mrpt::math"), "CVectorDynamic_double_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::CVectorDynamic<double>(); } ) );
		cl.def( pybind11::init( [](size_t const & a0){ return new mrpt::math::CVectorDynamic<double>(a0); } ), "doc" , pybind11::arg("N"));
		cl.def( pybind11::init<size_t, bool>(), pybind11::arg("N"), pybind11::arg("initZero") );

		cl.def( pybind11::init( [](mrpt::math::CVectorDynamic<double> const &o){ return new mrpt::math::CVectorDynamic<double>(o); } ) );
		cl.def("fromVectorLike", (void (mrpt::math::CVectorDynamic<double>::*)(const class mrpt::math::CMatrixFixed<double, 3, 1> &)) &mrpt::math::CVectorDynamic<double>::fromVectorLike<mrpt::math::CMatrixFixed<double, 3, 1>>, "C++: mrpt::math::CVectorDynamic<double>::fromVectorLike(const class mrpt::math::CMatrixFixed<double, 3, 1> &) --> void", pybind11::arg("m"));
		cl.def("fromVectorLike", (void (mrpt::math::CVectorDynamic<double>::*)(const class mrpt::math::CMatrixFixed<double, 6, 1> &)) &mrpt::math::CVectorDynamic<double>::fromVectorLike<mrpt::math::CMatrixFixed<double, 6, 1>>, "C++: mrpt::math::CVectorDynamic<double>::fromVectorLike(const class mrpt::math::CMatrixFixed<double, 6, 1> &) --> void", pybind11::arg("m"));
		cl.def("fromVectorLike", (void (mrpt::math::CVectorDynamic<double>::*)(const class mrpt::math::CMatrixFixed<double, 7, 1> &)) &mrpt::math::CVectorDynamic<double>::fromVectorLike<mrpt::math::CMatrixFixed<double, 7, 1>>, "C++: mrpt::math::CVectorDynamic<double>::fromVectorLike(const class mrpt::math::CMatrixFixed<double, 7, 1> &) --> void", pybind11::arg("m"));
		cl.def("assign", (class mrpt::math::CVectorDynamic<double> & (mrpt::math::CVectorDynamic<double>::*)(const class mrpt::math::CMatrixFixed<double, 3, 1> &)) &mrpt::math::CVectorDynamic<double>::operator=<3UL>, "C++: mrpt::math::CVectorDynamic<double>::operator=(const class mrpt::math::CMatrixFixed<double, 3, 1> &) --> class mrpt::math::CVectorDynamic<double> &", pybind11::return_value_policy::automatic, pybind11::arg("v"));
		cl.def("assign", (class mrpt::math::CVectorDynamic<double> & (mrpt::math::CVectorDynamic<double>::*)(const class mrpt::math::CMatrixFixed<double, 6, 1> &)) &mrpt::math::CVectorDynamic<double>::operator=<6UL>, "C++: mrpt::math::CVectorDynamic<double>::operator=(const class mrpt::math::CMatrixFixed<double, 6, 1> &) --> class mrpt::math::CVectorDynamic<double> &", pybind11::return_value_policy::automatic, pybind11::arg("v"));
		cl.def("assign", (class mrpt::math::CVectorDynamic<double> & (mrpt::math::CVectorDynamic<double>::*)(const class mrpt::math::CMatrixFixed<double, 7, 1> &)) &mrpt::math::CVectorDynamic<double>::operator=<7UL>, "C++: mrpt::math::CVectorDynamic<double>::operator=(const class mrpt::math::CMatrixFixed<double, 7, 1> &) --> class mrpt::math::CVectorDynamic<double> &", pybind11::return_value_policy::automatic, pybind11::arg("v"));
		cl.def("realloc", [](mrpt::math::CVectorDynamic<double> &o, size_t const & a0) -> void { return o.realloc(a0); }, "", pybind11::arg("new_len"));
		cl.def("realloc", (void (mrpt::math::CVectorDynamic<double>::*)(size_t, bool)) &mrpt::math::CVectorDynamic<double>::realloc, "C++: mrpt::math::CVectorDynamic<double>::realloc(size_t, bool) --> void", pybind11::arg("new_len"), pybind11::arg("newElementsToZero"));
		cl.def("swap", (void (mrpt::math::CVectorDynamic<double>::*)(class mrpt::math::CVectorDynamic<double> &)) &mrpt::math::CVectorDynamic<double>::swap, "C++: mrpt::math::CVectorDynamic<double>::swap(class mrpt::math::CVectorDynamic<double> &) --> void", pybind11::arg("o"));
		cl.def("rows", (int (mrpt::math::CVectorDynamic<double>::*)() const) &mrpt::math::CVectorDynamic<double>::rows, "C++: mrpt::math::CVectorDynamic<double>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CVectorDynamic<double>::*)() const) &mrpt::math::CVectorDynamic<double>::cols, "C++: mrpt::math::CVectorDynamic<double>::cols() const --> int");
		cl.def("size", (int (mrpt::math::CVectorDynamic<double>::*)() const) &mrpt::math::CVectorDynamic<double>::size, "C++: mrpt::math::CVectorDynamic<double>::size() const --> int");
		cl.def("empty", (bool (mrpt::math::CVectorDynamic<double>::*)() const) &mrpt::math::CVectorDynamic<double>::empty, "C++: mrpt::math::CVectorDynamic<double>::empty() const --> bool");
		cl.def("setSize", [](mrpt::math::CVectorDynamic<double> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CVectorDynamic<double>::*)(size_t, size_t, bool)) &mrpt::math::CVectorDynamic<double>::setSize, "C++: mrpt::math::CVectorDynamic<double>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("resize", [](mrpt::math::CVectorDynamic<double> &o, std::size_t const & a0) -> void { return o.resize(a0); }, "", pybind11::arg("N"));
		cl.def("resize", (void (mrpt::math::CVectorDynamic<double>::*)(std::size_t, bool)) &mrpt::math::CVectorDynamic<double>::resize, "C++: mrpt::math::CVectorDynamic<double>::resize(std::size_t, bool) --> void", pybind11::arg("N"), pybind11::arg("zeroNewElements"));
		cl.def("push_back", (void (mrpt::math::CVectorDynamic<double>::*)(const double &)) &mrpt::math::CVectorDynamic<double>::push_back, "C++: mrpt::math::CVectorDynamic<double>::push_back(const double &) --> void", pybind11::arg("val"));
		cl.def("segmentCopy", (class mrpt::math::CVectorDynamic<double> (mrpt::math::CVectorDynamic<double>::*)(int, int) const) &mrpt::math::CVectorDynamic<double>::segmentCopy, "C++: mrpt::math::CVectorDynamic<double>::segmentCopy(int, int) const --> class mrpt::math::CVectorDynamic<double>", pybind11::arg("start"), pybind11::arg("LEN"));
		cl.def("__call__", (double & (mrpt::math::CVectorDynamic<double>::*)(size_t, size_t)) &mrpt::math::CVectorDynamic<double>::operator(), "C++: mrpt::math::CVectorDynamic<double>::operator()(size_t, size_t) --> double &", pybind11::return_value_policy::reference, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__getitem__", (double & (mrpt::math::CVectorDynamic<double>::*)(size_t)) &mrpt::math::CVectorDynamic<double>::operator[], "C++: mrpt::math::CVectorDynamic<double>::operator[](size_t) --> double &", pybind11::return_value_policy::reference, pybind11::arg("ith"));
		cl.def("assign", (class mrpt::math::CVectorDynamic<double> & (mrpt::math::CVectorDynamic<double>::*)(const class mrpt::math::CVectorDynamic<double> &)) &mrpt::math::CVectorDynamic<double>::operator=, "C++: mrpt::math::CVectorDynamic<double>::operator=(const class mrpt::math::CVectorDynamic<double> &) --> class mrpt::math::CVectorDynamic<double> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::math::CVectorDynamic file:mrpt/math/CVectorDynamic.h line:32
		pybind11::class_<mrpt::math::CVectorDynamic<float>, std::shared_ptr<mrpt::math::CVectorDynamic<float>>> cl(M("mrpt::math"), "CVectorDynamic_float_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::CVectorDynamic<float>(); } ) );
		cl.def( pybind11::init( [](size_t const & a0){ return new mrpt::math::CVectorDynamic<float>(a0); } ), "doc" , pybind11::arg("N"));
		cl.def( pybind11::init<size_t, bool>(), pybind11::arg("N"), pybind11::arg("initZero") );

		cl.def( pybind11::init( [](mrpt::math::CVectorDynamic<float> const &o){ return new mrpt::math::CVectorDynamic<float>(o); } ) );
		cl.def("realloc", [](mrpt::math::CVectorDynamic<float> &o, size_t const & a0) -> void { return o.realloc(a0); }, "", pybind11::arg("new_len"));
		cl.def("realloc", (void (mrpt::math::CVectorDynamic<float>::*)(size_t, bool)) &mrpt::math::CVectorDynamic<float>::realloc, "C++: mrpt::math::CVectorDynamic<float>::realloc(size_t, bool) --> void", pybind11::arg("new_len"), pybind11::arg("newElementsToZero"));
		cl.def("swap", (void (mrpt::math::CVectorDynamic<float>::*)(class mrpt::math::CVectorDynamic<float> &)) &mrpt::math::CVectorDynamic<float>::swap, "C++: mrpt::math::CVectorDynamic<float>::swap(class mrpt::math::CVectorDynamic<float> &) --> void", pybind11::arg("o"));
		cl.def("rows", (int (mrpt::math::CVectorDynamic<float>::*)() const) &mrpt::math::CVectorDynamic<float>::rows, "C++: mrpt::math::CVectorDynamic<float>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CVectorDynamic<float>::*)() const) &mrpt::math::CVectorDynamic<float>::cols, "C++: mrpt::math::CVectorDynamic<float>::cols() const --> int");
		cl.def("size", (int (mrpt::math::CVectorDynamic<float>::*)() const) &mrpt::math::CVectorDynamic<float>::size, "C++: mrpt::math::CVectorDynamic<float>::size() const --> int");
		cl.def("empty", (bool (mrpt::math::CVectorDynamic<float>::*)() const) &mrpt::math::CVectorDynamic<float>::empty, "C++: mrpt::math::CVectorDynamic<float>::empty() const --> bool");
		cl.def("setSize", [](mrpt::math::CVectorDynamic<float> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CVectorDynamic<float>::*)(size_t, size_t, bool)) &mrpt::math::CVectorDynamic<float>::setSize, "C++: mrpt::math::CVectorDynamic<float>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("resize", [](mrpt::math::CVectorDynamic<float> &o, std::size_t const & a0) -> void { return o.resize(a0); }, "", pybind11::arg("N"));
		cl.def("resize", (void (mrpt::math::CVectorDynamic<float>::*)(std::size_t, bool)) &mrpt::math::CVectorDynamic<float>::resize, "C++: mrpt::math::CVectorDynamic<float>::resize(std::size_t, bool) --> void", pybind11::arg("N"), pybind11::arg("zeroNewElements"));
		cl.def("push_back", (void (mrpt::math::CVectorDynamic<float>::*)(const float &)) &mrpt::math::CVectorDynamic<float>::push_back, "C++: mrpt::math::CVectorDynamic<float>::push_back(const float &) --> void", pybind11::arg("val"));
		cl.def("segmentCopy", (class mrpt::math::CVectorDynamic<float> (mrpt::math::CVectorDynamic<float>::*)(int, int) const) &mrpt::math::CVectorDynamic<float>::segmentCopy, "C++: mrpt::math::CVectorDynamic<float>::segmentCopy(int, int) const --> class mrpt::math::CVectorDynamic<float>", pybind11::arg("start"), pybind11::arg("LEN"));
		cl.def("__call__", (float & (mrpt::math::CVectorDynamic<float>::*)(size_t, size_t)) &mrpt::math::CVectorDynamic<float>::operator(), "C++: mrpt::math::CVectorDynamic<float>::operator()(size_t, size_t) --> float &", pybind11::return_value_policy::reference, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__getitem__", (float & (mrpt::math::CVectorDynamic<float>::*)(size_t)) &mrpt::math::CVectorDynamic<float>::operator[], "C++: mrpt::math::CVectorDynamic<float>::operator[](size_t) --> float &", pybind11::return_value_policy::reference, pybind11::arg("ith"));
		cl.def("assign", (class mrpt::math::CVectorDynamic<float> & (mrpt::math::CVectorDynamic<float>::*)(const class mrpt::math::CVectorDynamic<float> &)) &mrpt::math::CVectorDynamic<float>::operator=, "C++: mrpt::math::CVectorDynamic<float>::operator=(const class mrpt::math::CVectorDynamic<float> &) --> class mrpt::math::CVectorDynamic<float> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
