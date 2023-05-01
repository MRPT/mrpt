#include <iterator>
#include <memory>
#include <sstream> // __str__
#include <string>
#include <vector>

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

void bind_std_stl_vector(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// std::vector file:bits/stl_vector.h line:389
	binder::vector_binder<std::string,std::allocator<std::string >>(M("std"), "std_string", "std_allocator_std_string_t");

}
