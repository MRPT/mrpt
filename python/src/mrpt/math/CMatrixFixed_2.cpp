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

void bind_mrpt_math_CMatrixFixed_2(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::CMatrixFixed file:mrpt/math/CMatrixFixed.h line:34
		pybind11::class_<mrpt::math::CMatrixFixed<double,7UL,1UL>, std::shared_ptr<mrpt::math::CMatrixFixed<double,7UL,1UL>>> cl(M("mrpt::math"), "CMatrixFixed_double_7UL_1UL_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixFixed<double,7UL,1UL>(); } ) );
		cl.def( pybind11::init<enum mrpt::math::TConstructorFlags_Matrices>(), pybind11::arg("") );

		cl.def( pybind11::init<const double *>(), pybind11::arg("data") );

		cl.def( pybind11::init<const int, const int>(), pybind11::arg("rows"), pybind11::arg("cols") );

		cl.def( pybind11::init( [](mrpt::math::CMatrixFixed<double,7UL,1UL> const &o){ return new mrpt::math::CMatrixFixed<double,7UL,1UL>(o); } ) );
		cl.def("loadFromRawPointer", (void (mrpt::math::CMatrixFixed<double,7UL,1UL>::*)(const double *)) &mrpt::math::CMatrixFixed<double, 7, 1>::loadFromRawPointer, "C++: mrpt::math::CMatrixFixed<double, 7, 1>::loadFromRawPointer(const double *) --> void", pybind11::arg("data"));
		cl.def("setSize", [](mrpt::math::CMatrixFixed<double,7UL,1UL> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CMatrixFixed<double,7UL,1UL>::*)(size_t, size_t, bool)) &mrpt::math::CMatrixFixed<double, 7, 1>::setSize, "C++: mrpt::math::CMatrixFixed<double, 7, 1>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("swap", (void (mrpt::math::CMatrixFixed<double,7UL,1UL>::*)(class mrpt::math::CMatrixFixed<double, 7, 1> &)) &mrpt::math::CMatrixFixed<double, 7, 1>::swap, "C++: mrpt::math::CMatrixFixed<double, 7, 1>::swap(class mrpt::math::CMatrixFixed<double, 7, 1> &) --> void", pybind11::arg("o"));
		cl.def("conservativeResize", (void (mrpt::math::CMatrixFixed<double,7UL,1UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<double, 7, 1>::conservativeResize, "C++: mrpt::math::CMatrixFixed<double, 7, 1>::conservativeResize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<double,7UL,1UL>::*)(size_t)) &mrpt::math::CMatrixFixed<double, 7, 1>::resize, "C++: mrpt::math::CMatrixFixed<double, 7, 1>::resize(size_t) --> void", pybind11::arg("n"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<double,7UL,1UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<double, 7, 1>::resize, "C++: mrpt::math::CMatrixFixed<double, 7, 1>::resize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("rows", (int (mrpt::math::CMatrixFixed<double,7UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<double, 7, 1>::rows, "C++: mrpt::math::CMatrixFixed<double, 7, 1>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CMatrixFixed<double,7UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<double, 7, 1>::cols, "C++: mrpt::math::CMatrixFixed<double, 7, 1>::cols() const --> int");
		cl.def("data", (double * (mrpt::math::CMatrixFixed<double,7UL,1UL>::*)()) &mrpt::math::CMatrixFixed<double, 7, 1>::data, "C++: mrpt::math::CMatrixFixed<double, 7, 1>::data() --> double *", pybind11::return_value_policy::automatic);
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,7UL,1UL>::*)(int, int)) &mrpt::math::CMatrixFixed<double, 7, 1>::operator(), "C++: mrpt::math::CMatrixFixed<double, 7, 1>::operator()(int, int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,7UL,1UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 7, 1>::operator(), "C++: mrpt::math::CMatrixFixed<double, 7, 1>::operator()(int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("__getitem__", (double & (mrpt::math::CMatrixFixed<double,7UL,1UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 7, 1>::operator[], "C++: mrpt::math::CMatrixFixed<double, 7, 1>::operator[](int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("sum_At", (void (mrpt::math::CMatrixFixed<double,7UL,1UL>::*)(const class mrpt::math::CMatrixFixed<double, 7, 1> &)) &mrpt::math::CMatrixFixed<double, 7, 1>::sum_At, "C++: mrpt::math::CMatrixFixed<double, 7, 1>::sum_At(const class mrpt::math::CMatrixFixed<double, 7, 1> &) --> void", pybind11::arg("A"));
	}
	{ // mrpt::math::CMatrixFixed file:mrpt/math/CMatrixFixed.h line:34
		pybind11::class_<mrpt::math::CMatrixFixed<double,4UL,1UL>, std::shared_ptr<mrpt::math::CMatrixFixed<double,4UL,1UL>>> cl(M("mrpt::math"), "CMatrixFixed_double_4UL_1UL_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixFixed<double,4UL,1UL>(); } ) );
		cl.def( pybind11::init<enum mrpt::math::TConstructorFlags_Matrices>(), pybind11::arg("") );

		cl.def( pybind11::init<const double *>(), pybind11::arg("data") );

		cl.def( pybind11::init<const int, const int>(), pybind11::arg("rows"), pybind11::arg("cols") );

		cl.def( pybind11::init( [](mrpt::math::CMatrixFixed<double,4UL,1UL> const &o){ return new mrpt::math::CMatrixFixed<double,4UL,1UL>(o); } ) );
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
		cl.def("cast_float", (class mrpt::math::CMatrixFixed<float, 4, 1> (mrpt::math::CMatrixFixed<double,4UL,1UL>::*)() const) &mrpt::math::CMatrixFixed<double, 4, 1>::cast_float, "C++: mrpt::math::CMatrixFixed<double, 4, 1>::cast_float() const --> class mrpt::math::CMatrixFixed<float, 4, 1>");
		cl.def("sum_At", (void (mrpt::math::CMatrixFixed<double,4UL,1UL>::*)(const class mrpt::math::CMatrixFixed<double, 4, 1> &)) &mrpt::math::CMatrixFixed<double, 4, 1>::sum_At, "C++: mrpt::math::CMatrixFixed<double, 4, 1>::sum_At(const class mrpt::math::CMatrixFixed<double, 4, 1> &) --> void", pybind11::arg("A"));
		cl.def("assign", (class mrpt::math::CMatrixFixed<double, 4, 1> & (mrpt::math::CMatrixFixed<double,4UL,1UL>::*)(const class mrpt::math::CMatrixFixed<double, 4, 1> &)) &mrpt::math::CMatrixFixed<double, 4, 1>::operator=, "C++: mrpt::math::CMatrixFixed<double, 4, 1>::operator=(const class mrpt::math::CMatrixFixed<double, 4, 1> &) --> class mrpt::math::CMatrixFixed<double, 4, 1> &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		// Manually-added matrix methods:
		using dat_t = double;
		using mat_t = mrpt::math::CMatrixFixed<dat_t,4,1>;
		cl.def("__getitem__", [](const mat_t&self, pybind11::tuple coord) -> dat_t { if (coord.size()==2) return self.coeff(coord[0].cast<size_t>(), coord[1].cast<size_t>()); else if (coord.size()==1) return self[coord[0].cast<size_t>()]; else throw std::invalid_argument("Access with [idx] or [row,col]"); });
		cl.def("__setitem__", [](mat_t&self, pybind11::tuple coord, dat_t val) { if (coord.size()==2) self.coeffRef(coord[0].cast<size_t>(), coord[1].cast<size_t>())=val; else if (coord.size()==1) self[coord[0].cast<size_t>()]=val; else throw std::invalid_argument("Access with [idx] or [row,col]"); });
		cl.def("__str__", [](const mat_t& o) -> std::string { return o.asString(); } );
		cl.def("inMatlabFormat", [](const mat_t& o) -> std::string { return o.inMatlabFormat(); } );
		cl.def("size", [](const mat_t&self) -> pybind11::tuple { return pybind11::make_tuple(self.cols(),self.rows()); });
		cl.def_static("Zero", []() -> mat_t { return mat_t::Zero(); }, "Returns a matrix with zeroes");
		cl.def(pybind11::init( [](pybind11::list vals){ auto m = new mat_t(); const auto nR = vals.size(); if (!nR) return m; const auto nC = vals[0].cast<pybind11::list>().size(); m->setSize(nR,nC); for (size_t r=0;r<nR;r++) { const auto row = vals[r].cast<pybind11::list>(); for (size_t c=0;c<nC;c++) m->coeffRef(r,c) = row[c].cast<dat_t>(); } return m; }));
		cl.def("to_list", [](const mat_t&self) -> pybind11::list { auto l = pybind11::list(); const auto nR = self.rows(), nC = self.cols(); for (size_t r=0;r<nR;r++) { auto row = pybind11::list(); l.append(row); for (size_t c=0;c<nC;c++) row.append(self.coeff(r,c)); } return l; });
	}
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
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,4UL,3UL>::*)(int, int)) &mrpt::math::CMatrixFixed<double, 4, 3>::operator(), "C++: mrpt::math::CMatrixFixed<double, 4, 3>::operator()(int, int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,4UL,3UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 4, 3>::operator(), "C++: mrpt::math::CMatrixFixed<double, 4, 3>::operator()(int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("__getitem__", (double & (mrpt::math::CMatrixFixed<double,4UL,3UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 4, 3>::operator[], "C++: mrpt::math::CMatrixFixed<double, 4, 3>::operator[](int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
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
		cl.def("__call__", (float & (mrpt::math::CMatrixFixed<float,3UL,3UL>::*)(int, int)) &mrpt::math::CMatrixFixed<float, 3, 3>::operator(), "C++: mrpt::math::CMatrixFixed<float, 3, 3>::operator()(int, int) --> float &", pybind11::return_value_policy::reference, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__call__", (float & (mrpt::math::CMatrixFixed<float,3UL,3UL>::*)(int)) &mrpt::math::CMatrixFixed<float, 3, 3>::operator(), "C++: mrpt::math::CMatrixFixed<float, 3, 3>::operator()(int) --> float &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("cast_float", (class mrpt::math::CMatrixFixed<float, 3, 3> (mrpt::math::CMatrixFixed<float,3UL,3UL>::*)() const) &mrpt::math::CMatrixFixed<float, 3, 3>::cast_float, "C++: mrpt::math::CMatrixFixed<float, 3, 3>::cast_float() const --> class mrpt::math::CMatrixFixed<float, 3, 3>");
		cl.def("sum_At", (void (mrpt::math::CMatrixFixed<float,3UL,3UL>::*)(const class mrpt::math::CMatrixFixed<float, 3, 3> &)) &mrpt::math::CMatrixFixed<float, 3, 3>::sum_At, "C++: mrpt::math::CMatrixFixed<float, 3, 3>::sum_At(const class mrpt::math::CMatrixFixed<float, 3, 3> &) --> void", pybind11::arg("A"));

		// Manually-added matrix methods:
		using dat_t = float;
		using mat_t = mrpt::math::CMatrixFixed<dat_t,3,3>;
		cl.def("__getitem__", [](const mat_t&self, pybind11::tuple coord) -> dat_t { if (coord.size()==2) return self.coeff(coord[0].cast<size_t>(), coord[1].cast<size_t>()); else if (coord.size()==1) return self[coord[0].cast<size_t>()]; else throw std::invalid_argument("Access with [idx] or [row,col]"); });
		cl.def("__setitem__", [](mat_t&self, pybind11::tuple coord, dat_t val) { if (coord.size()==2) self.coeffRef(coord[0].cast<size_t>(), coord[1].cast<size_t>())=val; else if (coord.size()==1) self[coord[0].cast<size_t>()]=val; else throw std::invalid_argument("Access with [idx] or [row,col]"); });
		cl.def("__str__", [](const mat_t& o) -> std::string { return o.asString(); } );
		cl.def("inMatlabFormat", [](const mat_t& o) -> std::string { return o.inMatlabFormat(); } );
		cl.def("size", [](const mat_t&self) -> pybind11::tuple { return pybind11::make_tuple(self.cols(),self.rows()); });
		cl.def_static("Identity", []() -> mat_t { return mat_t::Identity(); }, "Returns the NxN identity matrix");
		cl.def_static("Zero", []() -> mat_t { return mat_t::Zero(); }, "Returns a matrix with zeroes");
		cl.def(pybind11::init( [](pybind11::list vals){ auto m = new mat_t(); const auto nR = vals.size(); if (!nR) return m; const auto nC = vals[0].cast<pybind11::list>().size(); m->setSize(nR,nC); for (size_t r=0;r<nR;r++) { const auto row = vals[r].cast<pybind11::list>(); for (size_t c=0;c<nC;c++) m->coeffRef(r,c) = row[c].cast<dat_t>(); } return m; }));
		cl.def("to_list", [](const mat_t&self) -> pybind11::list { auto l = pybind11::list(); const auto nR = self.rows(), nC = self.cols(); for (size_t r=0;r<nR;r++) { auto row = pybind11::list(); l.append(row); for (size_t c=0;c<nC;c++) row.append(self.coeff(r,c)); } return l; });
	}
}
