#include <mrpt/core/aligned_allocator.h>

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

void bind_mrpt_core_aligned_allocator(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::aligned_malloc(size_t, size_t) file:mrpt/core/aligned_allocator.h line:19
	M("mrpt").def("aligned_malloc", (void * (*)(size_t, size_t)) &mrpt::aligned_malloc, "C++: mrpt::aligned_malloc(size_t, size_t) --> void *", pybind11::return_value_policy::automatic, pybind11::arg("size"), pybind11::arg("alignment"));

	// mrpt::aligned_free(void *) file:mrpt/core/aligned_allocator.h line:20
	M("mrpt").def("aligned_free", (void (*)(void *)) &mrpt::aligned_free, "C++: mrpt::aligned_free(void *) --> void", pybind11::arg("ptr"));

	// mrpt::aligned_calloc(size_t, size_t) file:mrpt/core/aligned_allocator.h line:22
	M("mrpt").def("aligned_calloc", (void * (*)(size_t, size_t)) &mrpt::aligned_calloc, "Identical to aligned_malloc, but it zeroes the reserved memory block. \n\nC++: mrpt::aligned_calloc(size_t, size_t) --> void *", pybind11::return_value_policy::automatic, pybind11::arg("bytes"), pybind11::arg("alignment"));

}
