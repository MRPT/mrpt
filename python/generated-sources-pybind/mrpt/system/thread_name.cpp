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
	// mrpt::system::thread_name(const std::string &) file:mrpt/system/thread_name.h line:26
	M("mrpt::system").def("thread_name", (void (*)(const std::string &)) &mrpt::system::thread_name, "Sets the name of the current thread; useful for debuggers.\n \n\n\n \n New in MRPT 2.0.4\n\nC++: mrpt::system::thread_name(const std::string &) --> void", pybind11::arg("name"));

	// mrpt::system::thread_name(class std::thread &) file:mrpt/system/thread_name.h line:32
	M("mrpt::system").def("thread_name", (std::string (*)(class std::thread &)) &mrpt::system::thread_name, "Gets the name of the given thread; useful for debuggers.\n \n\n\n \n New in MRPT 2.0.4\n\nC++: mrpt::system::thread_name(class std::thread &) --> std::string", pybind11::arg("theThread"));

	// mrpt::system::thread_name() file:mrpt/system/thread_name.h line:38
	M("mrpt::system").def("thread_name", (std::string (*)()) &mrpt::system::thread_name, "Gets the name of the current thread; useful for debuggers.\n \n\n\n \n New in MRPT 2.0.4\n\nC++: mrpt::system::thread_name() --> std::string");

}
