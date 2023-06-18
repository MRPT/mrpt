#include <mrpt/maps/OccupancyGridCellType.h>
#include <sstream> // __str__

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

void bind_mrpt_maps_OccupancyGridCellType(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::OccGridCellTraits file:mrpt/maps/OccupancyGridCellType.h line:32
		pybind11::class_<mrpt::maps::OccGridCellTraits, std::shared_ptr<mrpt::maps::OccGridCellTraits>> cl(M("mrpt::maps"), "OccGridCellTraits", "");
		cl.def( pybind11::init( [](){ return new mrpt::maps::OccGridCellTraits(); } ) );
	}
}
