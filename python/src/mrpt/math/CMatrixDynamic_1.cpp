#include <iterator>
#include <memory>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/math_frwds.h>
#include <mrpt/math/matrix_size_t.h>
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

void bind_mrpt_math_CMatrixDynamic_1(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::CMatrixDynamic file:mrpt/math/CMatrixDynamic.h line:41
		pybind11::class_<mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>, std::shared_ptr<mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>>> cl(M("mrpt::math"), "CMatrixDynamic_mrpt_math_TPoint3D_float_t", "");
		cl.def( pybind11::init( [](mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>> const &o){ return new mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>(); } ), "doc" );
		cl.def( pybind11::init( [](size_t const & a0){ return new mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>(a0); } ), "doc" , pybind11::arg("row"));
		cl.def( pybind11::init<size_t, size_t>(), pybind11::arg("row"), pybind11::arg("col") );

		cl.def( pybind11::init<const class mrpt::math::CMatrixDynamic<struct mrpt::math::TPoint3D_<float> > &, size_t, size_t>(), pybind11::arg("m"), pybind11::arg("cropRowCount"), pybind11::arg("cropColCount") );

		cl.def("assign", (class mrpt::math::CMatrixDynamic<struct mrpt::math::TPoint3D_<float> > & (mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::*)(const class mrpt::math::CMatrixDynamic<struct mrpt::math::TPoint3D_<float> > &)) &mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::operator=<mrpt::math::TPoint3D_<float>>, "C++: mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::operator=(const class mrpt::math::CMatrixDynamic<struct mrpt::math::TPoint3D_<float> > &) --> class mrpt::math::CMatrixDynamic<struct mrpt::math::TPoint3D_<float> > &", pybind11::return_value_policy::automatic, pybind11::arg("m"));
		cl.def("swap", (void (mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::*)(class mrpt::math::CMatrixDynamic<struct mrpt::math::TPoint3D_<float> > &)) &mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::swap, "C++: mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::swap(class mrpt::math::CMatrixDynamic<struct mrpt::math::TPoint3D_<float> > &) --> void", pybind11::arg("o"));
		cl.def("assign", (class mrpt::math::CMatrixDynamic<struct mrpt::math::TPoint3D_<float> > & (mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::*)(const class mrpt::math::CMatrixDynamic<struct mrpt::math::TPoint3D_<float> > &)) &mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::operator=, "C++: mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::operator=(const class mrpt::math::CMatrixDynamic<struct mrpt::math::TPoint3D_<float> > &) --> class mrpt::math::CMatrixDynamic<struct mrpt::math::TPoint3D_<float> > &", pybind11::return_value_policy::automatic, pybind11::arg("m"));
		cl.def("rows", (int (mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::*)() const) &mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::rows, "C++: mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::*)() const) &mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::cols, "C++: mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::cols() const --> int");
		cl.def("setSize", [](mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::*)(size_t, size_t, bool)) &mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::setSize, "C++: mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("resize", (void (mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::*)(size_t, size_t)) &mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::resize, "C++: mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::resize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("resize", (void (mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::*)(size_t)) &mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::resize, "C++: mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::resize(size_t) --> void", pybind11::arg("vectorLen"));
		cl.def("derived", (class mrpt::math::CMatrixDynamic<struct mrpt::math::TPoint3D_<float> > & (mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::*)()) &mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::derived, "C++: mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::derived() --> class mrpt::math::CMatrixDynamic<struct mrpt::math::TPoint3D_<float> > &", pybind11::return_value_policy::automatic);
		cl.def("conservativeResize", (void (mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::*)(size_t, size_t)) &mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::conservativeResize, "C++: mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::conservativeResize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("data", (struct mrpt::math::TPoint3D_<float> * (mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::*)()) &mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::data, "C++: mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::data() --> struct mrpt::math::TPoint3D_<float> *", pybind11::return_value_policy::automatic);
		cl.def("__call__", (struct mrpt::math::TPoint3D_<float> & (mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::*)(size_t, size_t)) &mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::operator(), "C++: mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::operator()(size_t, size_t) --> struct mrpt::math::TPoint3D_<float> &", pybind11::return_value_policy::reference, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__getitem__", (struct mrpt::math::TPoint3D_<float> & (mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::*)(size_t)) &mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::operator[], "C++: mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_<float>>::operator[](size_t) --> struct mrpt::math::TPoint3D_<float> &", pybind11::return_value_policy::reference, pybind11::arg("ith"));
	}
	{ // mrpt::math::CMatrixFixed file:mrpt/math/CMatrixFixed.h line:34
		pybind11::class_<mrpt::math::CMatrixFixed<double,2UL,2UL>, std::shared_ptr<mrpt::math::CMatrixFixed<double,2UL,2UL>>> cl(M("mrpt::math"), "CMatrixFixed_double_2UL_2UL_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixFixed<double,2UL,2UL>(); } ) );
		cl.def( pybind11::init<enum mrpt::math::TConstructorFlags_Matrices>(), pybind11::arg("") );

		cl.def( pybind11::init<const double *>(), pybind11::arg("data") );

		cl.def( pybind11::init<const int, const int>(), pybind11::arg("rows"), pybind11::arg("cols") );

		cl.def( pybind11::init( [](mrpt::math::CMatrixFixed<double,2UL,2UL> const &o){ return new mrpt::math::CMatrixFixed<double,2UL,2UL>(o); } ) );
		cl.def("loadFromRawPointer", (void (mrpt::math::CMatrixFixed<double,2UL,2UL>::*)(const double *)) &mrpt::math::CMatrixFixed<double, 2, 2>::loadFromRawPointer, "C++: mrpt::math::CMatrixFixed<double, 2, 2>::loadFromRawPointer(const double *) --> void", pybind11::arg("data"));
		cl.def("setSize", [](mrpt::math::CMatrixFixed<double,2UL,2UL> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CMatrixFixed<double,2UL,2UL>::*)(size_t, size_t, bool)) &mrpt::math::CMatrixFixed<double, 2, 2>::setSize, "C++: mrpt::math::CMatrixFixed<double, 2, 2>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("swap", (void (mrpt::math::CMatrixFixed<double,2UL,2UL>::*)(class mrpt::math::CMatrixFixed<double, 2, 2> &)) &mrpt::math::CMatrixFixed<double, 2, 2>::swap, "C++: mrpt::math::CMatrixFixed<double, 2, 2>::swap(class mrpt::math::CMatrixFixed<double, 2, 2> &) --> void", pybind11::arg("o"));
		cl.def("conservativeResize", (void (mrpt::math::CMatrixFixed<double,2UL,2UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<double, 2, 2>::conservativeResize, "C++: mrpt::math::CMatrixFixed<double, 2, 2>::conservativeResize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<double,2UL,2UL>::*)(size_t)) &mrpt::math::CMatrixFixed<double, 2, 2>::resize, "C++: mrpt::math::CMatrixFixed<double, 2, 2>::resize(size_t) --> void", pybind11::arg("n"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<double,2UL,2UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<double, 2, 2>::resize, "C++: mrpt::math::CMatrixFixed<double, 2, 2>::resize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("rows", (int (mrpt::math::CMatrixFixed<double,2UL,2UL>::*)() const) &mrpt::math::CMatrixFixed<double, 2, 2>::rows, "C++: mrpt::math::CMatrixFixed<double, 2, 2>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CMatrixFixed<double,2UL,2UL>::*)() const) &mrpt::math::CMatrixFixed<double, 2, 2>::cols, "C++: mrpt::math::CMatrixFixed<double, 2, 2>::cols() const --> int");
		cl.def("data", (double * (mrpt::math::CMatrixFixed<double,2UL,2UL>::*)()) &mrpt::math::CMatrixFixed<double, 2, 2>::data, "C++: mrpt::math::CMatrixFixed<double, 2, 2>::data() --> double *", pybind11::return_value_policy::automatic);
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,2UL,2UL>::*)(int, int)) &mrpt::math::CMatrixFixed<double, 2, 2>::operator(), "C++: mrpt::math::CMatrixFixed<double, 2, 2>::operator()(int, int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,2UL,2UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 2, 2>::operator(), "C++: mrpt::math::CMatrixFixed<double, 2, 2>::operator()(int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("__getitem__", (double & (mrpt::math::CMatrixFixed<double,2UL,2UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 2, 2>::operator[], "C++: mrpt::math::CMatrixFixed<double, 2, 2>::operator[](int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("sum_At", (void (mrpt::math::CMatrixFixed<double,2UL,2UL>::*)(const class mrpt::math::CMatrixFixed<double, 2, 2> &)) &mrpt::math::CMatrixFixed<double, 2, 2>::sum_At, "C++: mrpt::math::CMatrixFixed<double, 2, 2>::sum_At(const class mrpt::math::CMatrixFixed<double, 2, 2> &) --> void", pybind11::arg("A"));
		cl.def("assign", (class mrpt::math::CMatrixFixed<double, 2, 2> & (mrpt::math::CMatrixFixed<double,2UL,2UL>::*)(const class mrpt::math::CMatrixFixed<double, 2, 2> &)) &mrpt::math::CMatrixFixed<double, 2, 2>::operator=, "C++: mrpt::math::CMatrixFixed<double, 2, 2>::operator=(const class mrpt::math::CMatrixFixed<double, 2, 2> &) --> class mrpt::math::CMatrixFixed<double, 2, 2> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::math::CMatrixFixed file:mrpt/math/CMatrixFixed.h line:34
		pybind11::class_<mrpt::math::CMatrixFixed<double,3UL,3UL>, std::shared_ptr<mrpt::math::CMatrixFixed<double,3UL,3UL>>> cl(M("mrpt::math"), "CMatrixFixed_double_3UL_3UL_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixFixed<double,3UL,3UL>(); } ) );
		cl.def( pybind11::init<enum mrpt::math::TConstructorFlags_Matrices>(), pybind11::arg("") );

		cl.def( pybind11::init<const double *>(), pybind11::arg("data") );

		cl.def( pybind11::init<const int, const int>(), pybind11::arg("rows"), pybind11::arg("cols") );

		cl.def( pybind11::init( [](mrpt::math::CMatrixFixed<double,3UL,3UL> const &o){ return new mrpt::math::CMatrixFixed<double,3UL,3UL>(o); } ) );
		cl.def("loadFromRawPointer", (void (mrpt::math::CMatrixFixed<double,3UL,3UL>::*)(const double *)) &mrpt::math::CMatrixFixed<double, 3, 3>::loadFromRawPointer, "C++: mrpt::math::CMatrixFixed<double, 3, 3>::loadFromRawPointer(const double *) --> void", pybind11::arg("data"));
		cl.def("setSize", [](mrpt::math::CMatrixFixed<double,3UL,3UL> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CMatrixFixed<double,3UL,3UL>::*)(size_t, size_t, bool)) &mrpt::math::CMatrixFixed<double, 3, 3>::setSize, "C++: mrpt::math::CMatrixFixed<double, 3, 3>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("swap", (void (mrpt::math::CMatrixFixed<double,3UL,3UL>::*)(class mrpt::math::CMatrixFixed<double, 3, 3> &)) &mrpt::math::CMatrixFixed<double, 3, 3>::swap, "C++: mrpt::math::CMatrixFixed<double, 3, 3>::swap(class mrpt::math::CMatrixFixed<double, 3, 3> &) --> void", pybind11::arg("o"));
		cl.def("conservativeResize", (void (mrpt::math::CMatrixFixed<double,3UL,3UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<double, 3, 3>::conservativeResize, "C++: mrpt::math::CMatrixFixed<double, 3, 3>::conservativeResize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<double,3UL,3UL>::*)(size_t)) &mrpt::math::CMatrixFixed<double, 3, 3>::resize, "C++: mrpt::math::CMatrixFixed<double, 3, 3>::resize(size_t) --> void", pybind11::arg("n"));
		cl.def("resize", (void (mrpt::math::CMatrixFixed<double,3UL,3UL>::*)(size_t, size_t)) &mrpt::math::CMatrixFixed<double, 3, 3>::resize, "C++: mrpt::math::CMatrixFixed<double, 3, 3>::resize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("rows", (int (mrpt::math::CMatrixFixed<double,3UL,3UL>::*)() const) &mrpt::math::CMatrixFixed<double, 3, 3>::rows, "C++: mrpt::math::CMatrixFixed<double, 3, 3>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CMatrixFixed<double,3UL,3UL>::*)() const) &mrpt::math::CMatrixFixed<double, 3, 3>::cols, "C++: mrpt::math::CMatrixFixed<double, 3, 3>::cols() const --> int");
		cl.def("data", (double * (mrpt::math::CMatrixFixed<double,3UL,3UL>::*)()) &mrpt::math::CMatrixFixed<double, 3, 3>::data, "C++: mrpt::math::CMatrixFixed<double, 3, 3>::data() --> double *", pybind11::return_value_policy::automatic);
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,3UL,3UL>::*)(int, int)) &mrpt::math::CMatrixFixed<double, 3, 3>::operator(), "C++: mrpt::math::CMatrixFixed<double, 3, 3>::operator()(int, int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__call__", (double & (mrpt::math::CMatrixFixed<double,3UL,3UL>::*)(int)) &mrpt::math::CMatrixFixed<double, 3, 3>::operator(), "C++: mrpt::math::CMatrixFixed<double, 3, 3>::operator()(int) --> double &", pybind11::return_value_policy::reference, pybind11::arg("i"));
		cl.def("cast_float", (class mrpt::math::CMatrixFixed<float, 3, 3> (mrpt::math::CMatrixFixed<double,3UL,3UL>::*)() const) &mrpt::math::CMatrixFixed<double, 3, 3>::cast_float, "C++: mrpt::math::CMatrixFixed<double, 3, 3>::cast_float() const --> class mrpt::math::CMatrixFixed<float, 3, 3>");
		cl.def("sum_At", (void (mrpt::math::CMatrixFixed<double,3UL,3UL>::*)(const class mrpt::math::CMatrixFixed<double, 3, 3> &)) &mrpt::math::CMatrixFixed<double, 3, 3>::sum_At, "C++: mrpt::math::CMatrixFixed<double, 3, 3>::sum_At(const class mrpt::math::CMatrixFixed<double, 3, 3> &) --> void", pybind11::arg("A"));
		cl.def("assign", (class mrpt::math::CMatrixFixed<double, 3, 3> & (mrpt::math::CMatrixFixed<double,3UL,3UL>::*)(const class mrpt::math::CMatrixFixed<double, 3, 3> &)) &mrpt::math::CMatrixFixed<double, 3, 3>::operator=, "C++: mrpt::math::CMatrixFixed<double, 3, 3>::operator=(const class mrpt::math::CMatrixFixed<double, 3, 3> &) --> class mrpt::math::CMatrixFixed<double, 3, 3> &", pybind11::return_value_policy::automatic, pybind11::arg(""));

		// Manually-added matrix methods:
		using dat_t = double;
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
