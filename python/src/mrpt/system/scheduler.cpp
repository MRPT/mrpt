#include <iterator>
#include <memory>
#include <mrpt/system/scheduler.h>
#include <mrpt/system/thread_name.h>
#include <string>

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

void bind_mrpt_system_scheduler(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::system::changeCurrentProcessPriority(enum mrpt::system::TProcessPriority) file:mrpt/system/scheduler.h line:66
	M("mrpt::system").def("changeCurrentProcessPriority", (void (*)(enum mrpt::system::TProcessPriority)) &mrpt::system::changeCurrentProcessPriority, "Change the priority of the given process (it applies to all the threads,\n  plus independent modifiers for each thread).\n  - Windows: See\n  [SetPriorityClass](https://msdn.microsoft.com/es-es/library/windows/desktop/ms686219(v=vs.85).aspx)\n  - Linux (pthreads): Requires `root` permissions to increase process\n  priority! Internally it calls [nice()](http://linux.die.net/man/3/nice), so it\n  has no effect if\n  () was called and a SCHED_RR is already active.\n \n\n createThread, changeThreadPriority\n\nC++: mrpt::system::changeCurrentProcessPriority(enum mrpt::system::TProcessPriority) --> void", pybind11::arg("priority"));

	// mrpt::system::thread_name(const std::string &) file:mrpt/system/thread_name.h line:26
	M("mrpt::system").def("thread_name", (void (*)(const std::string &)) &mrpt::system::thread_name, "Sets the name of the current thread; useful for debuggers.\n \n\n\n \n New in MRPT 2.0.4\n\nC++: mrpt::system::thread_name(const std::string &) --> void", pybind11::arg("name"));

	// mrpt::system::thread_name() file:mrpt/system/thread_name.h line:38
	M("mrpt::system").def("thread_name", (std::string (*)()) &mrpt::system::thread_name, "Gets the name of the current thread; useful for debuggers.\n \n\n\n \n New in MRPT 2.0.4\n\nC++: mrpt::system::thread_name() --> std::string");

}
