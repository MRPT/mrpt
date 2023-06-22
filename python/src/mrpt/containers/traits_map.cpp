#include <mrpt/containers/traits_map.h>
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

void bind_mrpt_containers_traits_map(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::containers::map_traits_stdmap file:mrpt/containers/traits_map.h line:24
		pybind11::class_<mrpt::containers::map_traits_stdmap, std::shared_ptr<mrpt::containers::map_traits_stdmap>> cl(M("mrpt::containers"), "map_traits_stdmap", "Traits for using a std::map<> (sparse representation) \n\n map_traits_map_as_vector ");
		cl.def( pybind11::init( [](){ return new mrpt::containers::map_traits_stdmap(); } ) );
	}
	{ // mrpt::containers::map_traits_map_as_vector file:mrpt/containers/traits_map.h line:32
		pybind11::class_<mrpt::containers::map_traits_map_as_vector, std::shared_ptr<mrpt::containers::map_traits_map_as_vector>> cl(M("mrpt::containers"), "map_traits_map_as_vector", "Traits for using a mrpt::containers::map_as_vector<> (dense, fastest\n representation) \n\n map_traits_stdmap  ");
		cl.def( pybind11::init( [](){ return new mrpt::containers::map_traits_map_as_vector(); } ) );
	}
}
