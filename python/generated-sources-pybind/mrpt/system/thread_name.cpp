#include <iterator>
#include <memory>
#include <mrpt/system/thread_name.h>
#include <string>

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

void bind_mrpt_system_thread_name(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::system::thread_name() file:mrpt/system/thread_name.h line:38
	M("mrpt::system").def("thread_name", (std::string (*)()) &mrpt::system::thread_name, "Gets the name of the current thread; useful for debuggers.\n \n\n\n \n New in MRPT 2.0.4\n\nC++: mrpt::system::thread_name() --> std::string");

}
