#include <mrpt/maps/NearestNeighborsCapable.h>
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

void bind_mrpt_maps_NearestNeighborsCapable(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::maps::NearestNeighborsCapable file:mrpt/maps/NearestNeighborsCapable.h line:28
		pybind11::class_<mrpt::maps::NearestNeighborsCapable, std::shared_ptr<mrpt::maps::NearestNeighborsCapable>> cl(M("mrpt::maps"), "NearestNeighborsCapable", "Virtual interface for maps having the capability of searching the closest\n neighbor(s) of a given query 2D or 3D point.\n\n Note this is more generic than mrpt::math::KDTreeCapable since it does not\n assume the use of KD-trees, and it is also non templatized, so users can use\n dynamic casting to interact with maps in a generic way.\n\n \n New in MRPT 2.11.3\n \n\n\n ");
		cl.def("nn_supports_indices", (bool (mrpt::maps::NearestNeighborsCapable::*)() const) &mrpt::maps::NearestNeighborsCapable::nn_supports_indices, "Returns true if the rest of `nn_*` methods will populate the indices\n std::optional<> return variables, false otherwise. \n\nC++: mrpt::maps::NearestNeighborsCapable::nn_supports_indices() const --> bool");
		cl.def("nn_index_count", (size_t (mrpt::maps::NearestNeighborsCapable::*)() const) &mrpt::maps::NearestNeighborsCapable::nn_index_count, "If nn_supports_indices() returns `true`, this must return the number of\n \"points\" (or whatever entity) the indices correspond to. Otherwise, the\n return value should be ignored.\n\nC++: mrpt::maps::NearestNeighborsCapable::nn_index_count() const --> size_t");
		cl.def("assign", (class mrpt::maps::NearestNeighborsCapable & (mrpt::maps::NearestNeighborsCapable::*)(const class mrpt::maps::NearestNeighborsCapable &)) &mrpt::maps::NearestNeighborsCapable::operator=, "C++: mrpt::maps::NearestNeighborsCapable::operator=(const class mrpt::maps::NearestNeighborsCapable &) --> class mrpt::maps::NearestNeighborsCapable &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
