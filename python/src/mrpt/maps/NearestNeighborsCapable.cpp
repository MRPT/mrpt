#include <iterator>
#include <memory>
#include <mrpt/maps/NearestNeighborsCapable.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TPose3D.h>
#include <sstream> // __str__
#include <string>
#include <vector>

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

void bind_mrpt_maps_NearestNeighborsCapable(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::NearestNeighborsCapable file:mrpt/maps/NearestNeighborsCapable.h line:26
		pybind11::class_<mrpt::maps::NearestNeighborsCapable, std::shared_ptr<mrpt::maps::NearestNeighborsCapable>> cl(M("mrpt::maps"), "NearestNeighborsCapable", "Virtual interface for maps having the capability of searching the closest\n neighbor(s) of a given query 2D or 3D point.\n\n Note this is more generic than mrpt::math::KDTreeCapable since it does not\n assume the use of KD-trees, and it is also non templatized, so users can use\n dynamic casting to interact with maps in a generic way.\n\n \n New in MRPT 2.11.3\n \n\n\n ");
		cl.def("nn_single_search", (bool (mrpt::maps::NearestNeighborsCapable::*)(const struct mrpt::math::TPoint3D_<float> &, struct mrpt::math::TPoint3D_<float> &, float &) const) &mrpt::maps::NearestNeighborsCapable::nn_single_search, "Search for the closest 3D point to a given one.\n\n \n The query input point.\n \n\n The found closest point.\n \n\n The square Euclidean distance between the query\n and the returned point.\n\n \n True if successful, false if no point was found.\n\nC++: mrpt::maps::NearestNeighborsCapable::nn_single_search(const struct mrpt::math::TPoint3D_<float> &, struct mrpt::math::TPoint3D_<float> &, float &) const --> bool", pybind11::arg("query"), pybind11::arg("result"), pybind11::arg("out_dist_sqr"));
		cl.def("nn_single_search", (bool (mrpt::maps::NearestNeighborsCapable::*)(const struct mrpt::math::TPoint2D_<float> &, struct mrpt::math::TPoint2D_<float> &, float &) const) &mrpt::maps::NearestNeighborsCapable::nn_single_search, "C++: mrpt::maps::NearestNeighborsCapable::nn_single_search(const struct mrpt::math::TPoint2D_<float> &, struct mrpt::math::TPoint2D_<float> &, float &) const --> bool", pybind11::arg("query"), pybind11::arg("result"), pybind11::arg("out_dist_sqr"));
		cl.def("assign", (class mrpt::maps::NearestNeighborsCapable & (mrpt::maps::NearestNeighborsCapable::*)(const class mrpt::maps::NearestNeighborsCapable &)) &mrpt::maps::NearestNeighborsCapable::operator=, "C++: mrpt::maps::NearestNeighborsCapable::operator=(const class mrpt::maps::NearestNeighborsCapable &) --> class mrpt::maps::NearestNeighborsCapable &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
