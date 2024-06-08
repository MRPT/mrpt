#include <mrpt/maps/CLogOddsGridMap3D.h>
#include <mrpt/maps/OccupancyGridCellType.h>
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

void bind_mrpt_maps_CLogOddsGridMap3D(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::CLogOddsGridMap3D file:mrpt/maps/CLogOddsGridMap3D.h line:27
		pybind11::class_<mrpt::maps::CLogOddsGridMap3D<signed char>, std::shared_ptr<mrpt::maps::CLogOddsGridMap3D<signed char>>> cl(M("mrpt::maps"), "CLogOddsGridMap3D_signed_char_t", "");
		cl.def( pybind11::init( [](){ return new mrpt::maps::CLogOddsGridMap3D<signed char>(); } ) );
		cl.def( pybind11::init( [](mrpt::maps::CLogOddsGridMap3D<signed char> const &o){ return new mrpt::maps::CLogOddsGridMap3D<signed char>(o); } ) );
		cl.def_readwrite("m_grid", &mrpt::maps::CLogOddsGridMap3D<signed char>::m_grid);
		cl.def("updateCell_fast_occupied", (void (mrpt::maps::CLogOddsGridMap3D<signed char>::*)(signed char *, const signed char, const signed char)) &mrpt::maps::CLogOddsGridMap3D<signed char>::updateCell_fast_occupied, "C++: mrpt::maps::CLogOddsGridMap3D<signed char>::updateCell_fast_occupied(signed char *, const signed char, const signed char) --> void", pybind11::arg("theCell"), pybind11::arg("logodd_obs"), pybind11::arg("thres"));
		cl.def("updateCell_fast_occupied", (void (mrpt::maps::CLogOddsGridMap3D<signed char>::*)(const unsigned int, const unsigned int, const unsigned int, const signed char, const signed char)) &mrpt::maps::CLogOddsGridMap3D<signed char>::updateCell_fast_occupied, "C++: mrpt::maps::CLogOddsGridMap3D<signed char>::updateCell_fast_occupied(const unsigned int, const unsigned int, const unsigned int, const signed char, const signed char) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("logodd_obs"), pybind11::arg("thres"));
		cl.def("updateCell_fast_free", (void (mrpt::maps::CLogOddsGridMap3D<signed char>::*)(signed char *, const signed char, const signed char)) &mrpt::maps::CLogOddsGridMap3D<signed char>::updateCell_fast_free, "C++: mrpt::maps::CLogOddsGridMap3D<signed char>::updateCell_fast_free(signed char *, const signed char, const signed char) --> void", pybind11::arg("theCell"), pybind11::arg("logodd_obs"), pybind11::arg("thres"));
		cl.def("updateCell_fast_free", (void (mrpt::maps::CLogOddsGridMap3D<signed char>::*)(const unsigned int, const unsigned int, const unsigned int, const signed char, const signed char)) &mrpt::maps::CLogOddsGridMap3D<signed char>::updateCell_fast_free, "C++: mrpt::maps::CLogOddsGridMap3D<signed char>::updateCell_fast_free(const unsigned int, const unsigned int, const unsigned int, const signed char, const signed char) --> void", pybind11::arg("x"), pybind11::arg("y"), pybind11::arg("z"), pybind11::arg("logodd_obs"), pybind11::arg("thres"));
		cl.def("assign", (struct mrpt::maps::CLogOddsGridMap3D<signed char> & (mrpt::maps::CLogOddsGridMap3D<signed char>::*)(const struct mrpt::maps::CLogOddsGridMap3D<signed char> &)) &mrpt::maps::CLogOddsGridMap3D<signed char>::operator=, "C++: mrpt::maps::CLogOddsGridMap3D<signed char>::operator=(const struct mrpt::maps::CLogOddsGridMap3D<signed char> &) --> struct mrpt::maps::CLogOddsGridMap3D<signed char> &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::maps::OccGridCellTraits file:mrpt/maps/OccupancyGridCellType.h line:25
		pybind11::class_<mrpt::maps::OccGridCellTraits, std::shared_ptr<mrpt::maps::OccGridCellTraits>> cl(M("mrpt::maps"), "OccGridCellTraits", "");
		cl.def( pybind11::init( [](){ return new mrpt::maps::OccGridCellTraits(); } ) );
	}
}
