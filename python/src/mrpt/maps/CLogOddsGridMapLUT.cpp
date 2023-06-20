#include <mrpt/maps/CLogOddsGridMap2D.h>
#include <mrpt/maps/CLogOddsGridMapLUT.h>
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

void bind_mrpt_maps_CLogOddsGridMapLUT(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::CLogOddsGridMapLUT file:mrpt/maps/CLogOddsGridMapLUT.h line:29
		pybind11::class_<mrpt::maps::CLogOddsGridMapLUT<signed char>, std::shared_ptr<mrpt::maps::CLogOddsGridMapLUT<signed char>>> cl(M("mrpt::maps"), "CLogOddsGridMapLUT_signed_char_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::maps::CLogOddsGridMapLUT<signed char>(); } ) );
		cl.def( pybind11::init( [](mrpt::maps::CLogOddsGridMapLUT<signed char> const &o){ return new mrpt::maps::CLogOddsGridMapLUT<signed char>(o); } ) );
		cl.def_readwrite("logoddsTable", &mrpt::maps::CLogOddsGridMapLUT<signed char>::logoddsTable);
		cl.def_readwrite("logoddsTable_255", &mrpt::maps::CLogOddsGridMapLUT<signed char>::logoddsTable_255);
		cl.def_readwrite("p2lTable", &mrpt::maps::CLogOddsGridMapLUT<signed char>::p2lTable);
		cl.def("l2p", (float (mrpt::maps::CLogOddsGridMapLUT<signed char>::*)(const signed char)) &mrpt::maps::CLogOddsGridMapLUT<signed char>::l2p, "C++: mrpt::maps::CLogOddsGridMapLUT<signed char>::l2p(const signed char) --> float", pybind11::arg("l"));
		cl.def("l2p_255", (uint8_t (mrpt::maps::CLogOddsGridMapLUT<signed char>::*)(const signed char)) &mrpt::maps::CLogOddsGridMapLUT<signed char>::l2p_255, "C++: mrpt::maps::CLogOddsGridMapLUT<signed char>::l2p_255(const signed char) --> uint8_t", pybind11::arg("l"));
		cl.def("p2l", (signed char (mrpt::maps::CLogOddsGridMapLUT<signed char>::*)(const float)) &mrpt::maps::CLogOddsGridMapLUT<signed char>::p2l, "C++: mrpt::maps::CLogOddsGridMapLUT<signed char>::p2l(const float) --> signed char", pybind11::arg("p"));
		cl.def("assign", (struct mrpt::maps::CLogOddsGridMapLUT<signed char> & (mrpt::maps::CLogOddsGridMapLUT<signed char>::*)(const struct mrpt::maps::CLogOddsGridMapLUT<signed char> &)) &mrpt::maps::CLogOddsGridMapLUT<signed char>::operator=, "C++: mrpt::maps::CLogOddsGridMapLUT<signed char>::operator=(const struct mrpt::maps::CLogOddsGridMapLUT<signed char> &) --> struct mrpt::maps::CLogOddsGridMapLUT<signed char> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::maps::CLogOddsGridMap2D file:mrpt/maps/CLogOddsGridMap2D.h line:26
		pybind11::class_<mrpt::maps::CLogOddsGridMap2D<signed char>, std::shared_ptr<mrpt::maps::CLogOddsGridMap2D<signed char>>> cl(M("mrpt::maps"), "CLogOddsGridMap2D_signed_char_t", "");
		cl.def( pybind11::init( [](mrpt::maps::CLogOddsGridMap2D<signed char> const &o){ return new mrpt::maps::CLogOddsGridMap2D<signed char>(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::maps::CLogOddsGridMap2D<signed char>(); } ) );
		cl.def("updateCell_fast_occupied", (void (mrpt::maps::CLogOddsGridMap2D<signed char>::*)(const unsigned int, const unsigned int, const signed char, const signed char, signed char *, const unsigned int)) &mrpt::maps::CLogOddsGridMap2D<signed char>::updateCell_fast_occupied, "C++: mrpt::maps::CLogOddsGridMap2D<signed char>::updateCell_fast_occupied(const unsigned int, const unsigned int, const signed char, const signed char, signed char *, const unsigned int) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("logodd_obs"), pybind11::arg("thres"), pybind11::arg("mapArray"), pybind11::arg("_size_x"));
		cl.def("updateCell_fast_occupied", (void (mrpt::maps::CLogOddsGridMap2D<signed char>::*)(signed char *, const signed char, const signed char)) &mrpt::maps::CLogOddsGridMap2D<signed char>::updateCell_fast_occupied, "C++: mrpt::maps::CLogOddsGridMap2D<signed char>::updateCell_fast_occupied(signed char *, const signed char, const signed char) --> void", pybind11::arg("theCell"), pybind11::arg("logodd_obs"), pybind11::arg("thres"));
		cl.def_static("updateCell_fast_free", (void (*)(const unsigned int, const unsigned int, const signed char, const signed char, signed char *, const unsigned int)) &mrpt::maps::CLogOddsGridMap2D<signed char>::updateCell_fast_free, "C++: mrpt::maps::CLogOddsGridMap2D<signed char>::updateCell_fast_free(const unsigned int, const unsigned int, const signed char, const signed char, signed char *, const unsigned int) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("logodd_obs"), pybind11::arg("thres"), pybind11::arg("mapArray"), pybind11::arg("_size_x"));
		cl.def_static("updateCell_fast_free", (void (*)(signed char *, const signed char, const signed char)) &mrpt::maps::CLogOddsGridMap2D<signed char>::updateCell_fast_free, "C++: mrpt::maps::CLogOddsGridMap2D<signed char>::updateCell_fast_free(signed char *, const signed char, const signed char) --> void", pybind11::arg("theCell"), pybind11::arg("logodd_obs"), pybind11::arg("thres"));
		cl.def("assign", (struct mrpt::maps::CLogOddsGridMap2D<signed char> & (mrpt::maps::CLogOddsGridMap2D<signed char>::*)(const struct mrpt::maps::CLogOddsGridMap2D<signed char> &)) &mrpt::maps::CLogOddsGridMap2D<signed char>::operator=, "C++: mrpt::maps::CLogOddsGridMap2D<signed char>::operator=(const struct mrpt::maps::CLogOddsGridMap2D<signed char> &) --> struct mrpt::maps::CLogOddsGridMap2D<signed char> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
