#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/CVectorDynamic.h>
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

void bind_mrpt_math_CMatrixDynamic(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::math::CMatrixDynamic file:mrpt/math/CMatrixDynamic.h line:41
		pybind11::class_<mrpt::math::CMatrixDynamic<float>, std::shared_ptr<mrpt::math::CMatrixDynamic<float>>> cl(M("mrpt::math"), "CMatrixDynamic_float_t", "");
		cl.def( pybind11::init( [](mrpt::math::CMatrixDynamic<float> const &o){ return new mrpt::math::CMatrixDynamic<float>(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixDynamic<float>(); } ), "doc" );
		cl.def( pybind11::init( [](size_t const & a0){ return new mrpt::math::CMatrixDynamic<float>(a0); } ), "doc" , pybind11::arg("row"));
		cl.def( pybind11::init<size_t, size_t>(), pybind11::arg("row"), pybind11::arg("col") );

		cl.def( pybind11::init<const class mrpt::math::CMatrixDynamic<float> &, size_t, size_t>(), pybind11::arg("m"), pybind11::arg("cropRowCount"), pybind11::arg("cropColCount") );

		cl.def("setFromMatrixLike", (void (mrpt::math::CMatrixDynamic<float>::*)(const class mrpt::math::CMatrixDynamic<double> &)) &mrpt::math::CMatrixDynamic<float>::setFromMatrixLike<mrpt::math::CMatrixDynamic<double>>, "C++: mrpt::math::CMatrixDynamic<float>::setFromMatrixLike(const class mrpt::math::CMatrixDynamic<double> &) --> void", pybind11::arg("m"));
		cl.def("assign", (class mrpt::math::CMatrixDynamic<float> & (mrpt::math::CMatrixDynamic<float>::*)(const class mrpt::math::CMatrixDynamic<float> &)) &mrpt::math::CMatrixDynamic<float>::operator=<float>, "C++: mrpt::math::CMatrixDynamic<float>::operator=(const class mrpt::math::CMatrixDynamic<float> &) --> class mrpt::math::CMatrixDynamic<float> &", pybind11::return_value_policy::automatic, pybind11::arg("m"));
		cl.def("assign", (class mrpt::math::CMatrixDynamic<float> & (mrpt::math::CMatrixDynamic<float>::*)(const class mrpt::math::CMatrixDynamic<double> &)) &mrpt::math::CMatrixDynamic<float>::operator=<double>, "C++: mrpt::math::CMatrixDynamic<float>::operator=(const class mrpt::math::CMatrixDynamic<double> &) --> class mrpt::math::CMatrixDynamic<float> &", pybind11::return_value_policy::automatic, pybind11::arg("m"));
		cl.def("swap", (void (mrpt::math::CMatrixDynamic<float>::*)(class mrpt::math::CMatrixDynamic<float> &)) &mrpt::math::CMatrixDynamic<float>::swap, "C++: mrpt::math::CMatrixDynamic<float>::swap(class mrpt::math::CMatrixDynamic<float> &) --> void", pybind11::arg("o"));
		cl.def("assign", (class mrpt::math::CMatrixDynamic<float> & (mrpt::math::CMatrixDynamic<float>::*)(const class mrpt::math::CMatrixDynamic<float> &)) &mrpt::math::CMatrixDynamic<float>::operator=, "C++: mrpt::math::CMatrixDynamic<float>::operator=(const class mrpt::math::CMatrixDynamic<float> &) --> class mrpt::math::CMatrixDynamic<float> &", pybind11::return_value_policy::automatic, pybind11::arg("m"));
		cl.def("rows", (int (mrpt::math::CMatrixDynamic<float>::*)() const) &mrpt::math::CMatrixDynamic<float>::rows, "C++: mrpt::math::CMatrixDynamic<float>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CMatrixDynamic<float>::*)() const) &mrpt::math::CMatrixDynamic<float>::cols, "C++: mrpt::math::CMatrixDynamic<float>::cols() const --> int");
		cl.def("setSize", [](mrpt::math::CMatrixDynamic<float> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CMatrixDynamic<float>::*)(size_t, size_t, bool)) &mrpt::math::CMatrixDynamic<float>::setSize, "C++: mrpt::math::CMatrixDynamic<float>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("resize", (void (mrpt::math::CMatrixDynamic<float>::*)(size_t, size_t)) &mrpt::math::CMatrixDynamic<float>::resize, "C++: mrpt::math::CMatrixDynamic<float>::resize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("resize", (void (mrpt::math::CMatrixDynamic<float>::*)(size_t)) &mrpt::math::CMatrixDynamic<float>::resize, "C++: mrpt::math::CMatrixDynamic<float>::resize(size_t) --> void", pybind11::arg("vectorLen"));
		cl.def("derived", (class mrpt::math::CMatrixDynamic<float> & (mrpt::math::CMatrixDynamic<float>::*)()) &mrpt::math::CMatrixDynamic<float>::derived, "C++: mrpt::math::CMatrixDynamic<float>::derived() --> class mrpt::math::CMatrixDynamic<float> &", pybind11::return_value_policy::automatic);
		cl.def("conservativeResize", (void (mrpt::math::CMatrixDynamic<float>::*)(size_t, size_t)) &mrpt::math::CMatrixDynamic<float>::conservativeResize, "C++: mrpt::math::CMatrixDynamic<float>::conservativeResize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("data", (float * (mrpt::math::CMatrixDynamic<float>::*)()) &mrpt::math::CMatrixDynamic<float>::data, "C++: mrpt::math::CMatrixDynamic<float>::data() --> float *", pybind11::return_value_policy::automatic);
		cl.def("__call__", (float & (mrpt::math::CMatrixDynamic<float>::*)(size_t, size_t)) &mrpt::math::CMatrixDynamic<float>::operator(), "C++: mrpt::math::CMatrixDynamic<float>::operator()(size_t, size_t) --> float &", pybind11::return_value_policy::reference, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("cast_float", (class mrpt::math::CMatrixDynamic<float> (mrpt::math::CMatrixDynamic<float>::*)() const) &mrpt::math::CMatrixDynamic<float>::cast_float, "C++: mrpt::math::CMatrixDynamic<float>::cast_float() const --> class mrpt::math::CMatrixDynamic<float>");
		cl.def("cast_double", (class mrpt::math::CMatrixDynamic<double> (mrpt::math::CMatrixDynamic<float>::*)() const) &mrpt::math::CMatrixDynamic<float>::cast_double, "C++: mrpt::math::CMatrixDynamic<float>::cast_double() const --> class mrpt::math::CMatrixDynamic<double>");
		cl.def("llt_solve", (class mrpt::math::CVectorDynamic<float> (mrpt::math::CMatrixDynamic<float>::*)(const class mrpt::math::CVectorDynamic<float> &) const) &mrpt::math::CMatrixDynamic<float>::llt_solve, "C++: mrpt::math::CMatrixDynamic<float>::llt_solve(const class mrpt::math::CVectorDynamic<float> &) const --> class mrpt::math::CVectorDynamic<float>", pybind11::arg("b"));
		cl.def("lu_solve", (class mrpt::math::CVectorDynamic<float> (mrpt::math::CMatrixDynamic<float>::*)(const class mrpt::math::CVectorDynamic<float> &) const) &mrpt::math::CMatrixDynamic<float>::lu_solve, "C++: mrpt::math::CMatrixDynamic<float>::lu_solve(const class mrpt::math::CVectorDynamic<float> &) const --> class mrpt::math::CVectorDynamic<float>", pybind11::arg("b"));

		// Manually-added matrix methods:
		using dat_t = float;
		using mat_t = mrpt::math::CMatrixDynamic<dat_t>;
		cl.def("__getitem__", [](const mat_t&self, pybind11::tuple coord) -> dat_t { if (coord.size()==2) return self.coeff(coord[0].cast<size_t>(), coord[1].cast<size_t>()); else if (coord.size()==1) return self[coord[0].cast<size_t>()]; else throw std::invalid_argument("Access with [idx] or [row,col]"); });
		cl.def("__setitem__", [](mat_t&self, pybind11::tuple coord, dat_t val) { if (coord.size()==2) self.coeffRef(coord[0].cast<size_t>(), coord[1].cast<size_t>())=val; else if (coord.size()==1) self[coord[0].cast<size_t>()]=val; else throw std::invalid_argument("Access with [idx] or [row,col]"); });
		cl.def("__str__", [](const mat_t& o) -> std::string { return o.asString(); } );
		cl.def("inMatlabFormat", [](const mat_t& o) -> std::string { return o.inMatlabFormat(); } );
		cl.def("size", [](const mat_t&self) -> pybind11::tuple { return pybind11::make_tuple(self.cols(),self.rows()); });
		cl.def_static("Identity", [](const size_t N) -> mat_t { return mat_t::Identity(N); }, "Returns the NxN identity matrix");
		cl.def_static("Zero", [](const size_t nRows, const size_t nCols) -> mat_t { return mat_t::Zero(nRows,nCols); }, "Returns a matrix with zeroes");
		cl.def(pybind11::init( [](pybind11::list vals){ auto m = new mat_t(); const auto nR = vals.size(); if (!nR) return m; const auto nC = vals[0].cast<pybind11::list>().size(); m->setSize(nR,nC); for (size_t r=0;r<nR;r++) { const auto row = vals[r].cast<pybind11::list>(); for (size_t c=0;c<nC;c++) m->coeffRef(r,c) = row[c].cast<dat_t>(); } return m; }));
		cl.def("to_list", [](const mat_t&self) -> pybind11::list { auto l = pybind11::list(); const auto nR = self.rows(), nC = self.cols(); for (size_t r=0;r<nR;r++) { auto row = pybind11::list(); l.append(row); for (size_t c=0;c<nC;c++) row.append(self.coeff(r,c)); } return l; });
	}
	{ // mrpt::math::CMatrixDynamic file:mrpt/math/CMatrixDynamic.h line:41
		pybind11::class_<mrpt::math::CMatrixDynamic<unsigned char>, std::shared_ptr<mrpt::math::CMatrixDynamic<unsigned char>>> cl(M("mrpt::math"), "CMatrixDynamic_unsigned_char_t", "");
		cl.def( pybind11::init( [](mrpt::math::CMatrixDynamic<unsigned char> const &o){ return new mrpt::math::CMatrixDynamic<unsigned char>(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixDynamic<unsigned char>(); } ), "doc" );
		cl.def( pybind11::init( [](size_t const & a0){ return new mrpt::math::CMatrixDynamic<unsigned char>(a0); } ), "doc" , pybind11::arg("row"));
		cl.def( pybind11::init<size_t, size_t>(), pybind11::arg("row"), pybind11::arg("col") );

		cl.def( pybind11::init<const class mrpt::math::CMatrixDynamic<unsigned char> &, size_t, size_t>(), pybind11::arg("m"), pybind11::arg("cropRowCount"), pybind11::arg("cropColCount") );

		cl.def("assign", (class mrpt::math::CMatrixDynamic<unsigned char> & (mrpt::math::CMatrixDynamic<unsigned char>::*)(const class mrpt::math::CMatrixDynamic<unsigned char> &)) &mrpt::math::CMatrixDynamic<unsigned char>::operator=<unsigned char>, "C++: mrpt::math::CMatrixDynamic<unsigned char>::operator=(const class mrpt::math::CMatrixDynamic<unsigned char> &) --> class mrpt::math::CMatrixDynamic<unsigned char> &", pybind11::return_value_policy::automatic, pybind11::arg("m"));
		cl.def("swap", (void (mrpt::math::CMatrixDynamic<unsigned char>::*)(class mrpt::math::CMatrixDynamic<unsigned char> &)) &mrpt::math::CMatrixDynamic<unsigned char>::swap, "C++: mrpt::math::CMatrixDynamic<unsigned char>::swap(class mrpt::math::CMatrixDynamic<unsigned char> &) --> void", pybind11::arg("o"));
		cl.def("assign", (class mrpt::math::CMatrixDynamic<unsigned char> & (mrpt::math::CMatrixDynamic<unsigned char>::*)(const class mrpt::math::CMatrixDynamic<unsigned char> &)) &mrpt::math::CMatrixDynamic<unsigned char>::operator=, "C++: mrpt::math::CMatrixDynamic<unsigned char>::operator=(const class mrpt::math::CMatrixDynamic<unsigned char> &) --> class mrpt::math::CMatrixDynamic<unsigned char> &", pybind11::return_value_policy::automatic, pybind11::arg("m"));
		cl.def("rows", (int (mrpt::math::CMatrixDynamic<unsigned char>::*)() const) &mrpt::math::CMatrixDynamic<unsigned char>::rows, "C++: mrpt::math::CMatrixDynamic<unsigned char>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CMatrixDynamic<unsigned char>::*)() const) &mrpt::math::CMatrixDynamic<unsigned char>::cols, "C++: mrpt::math::CMatrixDynamic<unsigned char>::cols() const --> int");
		cl.def("setSize", [](mrpt::math::CMatrixDynamic<unsigned char> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CMatrixDynamic<unsigned char>::*)(size_t, size_t, bool)) &mrpt::math::CMatrixDynamic<unsigned char>::setSize, "C++: mrpt::math::CMatrixDynamic<unsigned char>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("resize", (void (mrpt::math::CMatrixDynamic<unsigned char>::*)(size_t, size_t)) &mrpt::math::CMatrixDynamic<unsigned char>::resize, "C++: mrpt::math::CMatrixDynamic<unsigned char>::resize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("resize", (void (mrpt::math::CMatrixDynamic<unsigned char>::*)(size_t)) &mrpt::math::CMatrixDynamic<unsigned char>::resize, "C++: mrpt::math::CMatrixDynamic<unsigned char>::resize(size_t) --> void", pybind11::arg("vectorLen"));
		cl.def("derived", (class mrpt::math::CMatrixDynamic<unsigned char> & (mrpt::math::CMatrixDynamic<unsigned char>::*)()) &mrpt::math::CMatrixDynamic<unsigned char>::derived, "C++: mrpt::math::CMatrixDynamic<unsigned char>::derived() --> class mrpt::math::CMatrixDynamic<unsigned char> &", pybind11::return_value_policy::automatic);
		cl.def("conservativeResize", (void (mrpt::math::CMatrixDynamic<unsigned char>::*)(size_t, size_t)) &mrpt::math::CMatrixDynamic<unsigned char>::conservativeResize, "C++: mrpt::math::CMatrixDynamic<unsigned char>::conservativeResize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("data", (unsigned char * (mrpt::math::CMatrixDynamic<unsigned char>::*)()) &mrpt::math::CMatrixDynamic<unsigned char>::data, "C++: mrpt::math::CMatrixDynamic<unsigned char>::data() --> unsigned char *", pybind11::return_value_policy::automatic);
		cl.def("__call__", (unsigned char & (mrpt::math::CMatrixDynamic<unsigned char>::*)(size_t, size_t)) &mrpt::math::CMatrixDynamic<unsigned char>::operator(), "C++: mrpt::math::CMatrixDynamic<unsigned char>::operator()(size_t, size_t) --> unsigned char &", pybind11::return_value_policy::reference, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__getitem__", (unsigned char & (mrpt::math::CMatrixDynamic<unsigned char>::*)(size_t)) &mrpt::math::CMatrixDynamic<unsigned char>::operator[], "C++: mrpt::math::CMatrixDynamic<unsigned char>::operator[](size_t) --> unsigned char &", pybind11::return_value_policy::reference, pybind11::arg("ith"));
	}
	{ // mrpt::math::CMatrixDynamic file:mrpt/math/CMatrixDynamic.h line:41
		pybind11::class_<mrpt::math::CMatrixDynamic<unsigned short>, std::shared_ptr<mrpt::math::CMatrixDynamic<unsigned short>>> cl(M("mrpt::math"), "CMatrixDynamic_unsigned_short_t", "");
		cl.def( pybind11::init( [](mrpt::math::CMatrixDynamic<unsigned short> const &o){ return new mrpt::math::CMatrixDynamic<unsigned short>(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::math::CMatrixDynamic<unsigned short>(); } ), "doc" );
		cl.def( pybind11::init( [](size_t const & a0){ return new mrpt::math::CMatrixDynamic<unsigned short>(a0); } ), "doc" , pybind11::arg("row"));
		cl.def( pybind11::init<size_t, size_t>(), pybind11::arg("row"), pybind11::arg("col") );

		cl.def( pybind11::init<const class mrpt::math::CMatrixDynamic<unsigned short> &, size_t, size_t>(), pybind11::arg("m"), pybind11::arg("cropRowCount"), pybind11::arg("cropColCount") );

		cl.def("assign", (class mrpt::math::CMatrixDynamic<unsigned short> & (mrpt::math::CMatrixDynamic<unsigned short>::*)(const class mrpt::math::CMatrixDynamic<unsigned short> &)) &mrpt::math::CMatrixDynamic<unsigned short>::operator=<unsigned short>, "C++: mrpt::math::CMatrixDynamic<unsigned short>::operator=(const class mrpt::math::CMatrixDynamic<unsigned short> &) --> class mrpt::math::CMatrixDynamic<unsigned short> &", pybind11::return_value_policy::automatic, pybind11::arg("m"));
		cl.def("swap", (void (mrpt::math::CMatrixDynamic<unsigned short>::*)(class mrpt::math::CMatrixDynamic<unsigned short> &)) &mrpt::math::CMatrixDynamic<unsigned short>::swap, "C++: mrpt::math::CMatrixDynamic<unsigned short>::swap(class mrpt::math::CMatrixDynamic<unsigned short> &) --> void", pybind11::arg("o"));
		cl.def("assign", (class mrpt::math::CMatrixDynamic<unsigned short> & (mrpt::math::CMatrixDynamic<unsigned short>::*)(const class mrpt::math::CMatrixDynamic<unsigned short> &)) &mrpt::math::CMatrixDynamic<unsigned short>::operator=, "C++: mrpt::math::CMatrixDynamic<unsigned short>::operator=(const class mrpt::math::CMatrixDynamic<unsigned short> &) --> class mrpt::math::CMatrixDynamic<unsigned short> &", pybind11::return_value_policy::automatic, pybind11::arg("m"));
		cl.def("rows", (int (mrpt::math::CMatrixDynamic<unsigned short>::*)() const) &mrpt::math::CMatrixDynamic<unsigned short>::rows, "C++: mrpt::math::CMatrixDynamic<unsigned short>::rows() const --> int");
		cl.def("cols", (int (mrpt::math::CMatrixDynamic<unsigned short>::*)() const) &mrpt::math::CMatrixDynamic<unsigned short>::cols, "C++: mrpt::math::CMatrixDynamic<unsigned short>::cols() const --> int");
		cl.def("setSize", [](mrpt::math::CMatrixDynamic<unsigned short> &o, size_t const & a0, size_t const & a1) -> void { return o.setSize(a0, a1); }, "", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("setSize", (void (mrpt::math::CMatrixDynamic<unsigned short>::*)(size_t, size_t, bool)) &mrpt::math::CMatrixDynamic<unsigned short>::setSize, "C++: mrpt::math::CMatrixDynamic<unsigned short>::setSize(size_t, size_t, bool) --> void", pybind11::arg("row"), pybind11::arg("col"), pybind11::arg("zeroNewElements"));
		cl.def("resize", (void (mrpt::math::CMatrixDynamic<unsigned short>::*)(size_t, size_t)) &mrpt::math::CMatrixDynamic<unsigned short>::resize, "C++: mrpt::math::CMatrixDynamic<unsigned short>::resize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("resize", (void (mrpt::math::CMatrixDynamic<unsigned short>::*)(size_t)) &mrpt::math::CMatrixDynamic<unsigned short>::resize, "C++: mrpt::math::CMatrixDynamic<unsigned short>::resize(size_t) --> void", pybind11::arg("vectorLen"));
		cl.def("derived", (class mrpt::math::CMatrixDynamic<unsigned short> & (mrpt::math::CMatrixDynamic<unsigned short>::*)()) &mrpt::math::CMatrixDynamic<unsigned short>::derived, "C++: mrpt::math::CMatrixDynamic<unsigned short>::derived() --> class mrpt::math::CMatrixDynamic<unsigned short> &", pybind11::return_value_policy::automatic);
		cl.def("conservativeResize", (void (mrpt::math::CMatrixDynamic<unsigned short>::*)(size_t, size_t)) &mrpt::math::CMatrixDynamic<unsigned short>::conservativeResize, "C++: mrpt::math::CMatrixDynamic<unsigned short>::conservativeResize(size_t, size_t) --> void", pybind11::arg("row"), pybind11::arg("col"));
		cl.def("data", (unsigned short * (mrpt::math::CMatrixDynamic<unsigned short>::*)()) &mrpt::math::CMatrixDynamic<unsigned short>::data, "C++: mrpt::math::CMatrixDynamic<unsigned short>::data() --> unsigned short *", pybind11::return_value_policy::automatic);
		cl.def("__call__", (unsigned short & (mrpt::math::CMatrixDynamic<unsigned short>::*)(size_t, size_t)) &mrpt::math::CMatrixDynamic<unsigned short>::operator(), "C++: mrpt::math::CMatrixDynamic<unsigned short>::operator()(size_t, size_t) --> unsigned short &", pybind11::return_value_policy::reference, pybind11::arg("row"), pybind11::arg("col"));
		cl.def("__getitem__", (unsigned short & (mrpt::math::CMatrixDynamic<unsigned short>::*)(size_t)) &mrpt::math::CMatrixDynamic<unsigned short>::operator[], "C++: mrpt::math::CMatrixDynamic<unsigned short>::operator[](size_t) --> unsigned short &", pybind11::return_value_policy::reference, pybind11::arg("ith"));
	}
}
