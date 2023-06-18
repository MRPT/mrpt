#include <iterator>
#include <memory>
#include <mrpt/core/WorkerThreadsPool.h>
#include <sstream> // __str__
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

void bind_mrpt_core_WorkerThreadsPool(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::WorkerThreadsPool file:mrpt/core/WorkerThreadsPool.h line:50
		pybind11::class_<mrpt::WorkerThreadsPool, std::shared_ptr<mrpt::WorkerThreadsPool>> cl(M("mrpt"), "WorkerThreadsPool", "A thread pool: it defines a fixed number of threads, which will remain\n  blocked waiting for jobs to be assigned, via WorkerThreadsPool::enqueue(),\n  which accepts any function-like object with arbitrary parameters and\n  returns a std::future<ReturnType> which can be used to wait and/or retrieve\n  the function output at any moment in time afterwards.\n\n  In case of more tasks assigned than available free threads, two policies\n  exist:\n  - WorkerThreadsPool::POLICY_FIFO: All jobs are enqueued and wait for it turn\n    to be executed.\n  - WorkerThreadsPool::POLICY_DROP_OLD: Old jobs in the waiting queue are\n    discarded. Note that running jobs are never aborted.\n\n \n Partly based on: https://github.com/progschj/ThreadPool (ZLib license)\n\n \n (New in MRPT 2.1.0)");
		cl.def( pybind11::init( [](){ return new mrpt::WorkerThreadsPool(); } ) );
		cl.def( pybind11::init( [](std::size_t const & a0){ return new mrpt::WorkerThreadsPool(a0); } ), "doc" , pybind11::arg("num_threads"));
		cl.def( pybind11::init( [](std::size_t const & a0, enum mrpt::WorkerThreadsPool::queue_policy_t const & a1){ return new mrpt::WorkerThreadsPool(a0, a1); } ), "doc" , pybind11::arg("num_threads"), pybind11::arg("p"));
		cl.def( pybind11::init<std::size_t, enum mrpt::WorkerThreadsPool::queue_policy_t, const std::string &>(), pybind11::arg("num_threads"), pybind11::arg("p"), pybind11::arg("threadsName") );


		pybind11::enum_<mrpt::WorkerThreadsPool::queue_policy_t>(cl, "queue_policy_t", pybind11::arithmetic(), "")
			.value("POLICY_FIFO", mrpt::WorkerThreadsPool::POLICY_FIFO)
			.value("POLICY_DROP_OLD", mrpt::WorkerThreadsPool::POLICY_DROP_OLD)
			.export_values();

		cl.def("resize", (void (mrpt::WorkerThreadsPool::*)(std::size_t)) &mrpt::WorkerThreadsPool::resize, "C++: mrpt::WorkerThreadsPool::resize(std::size_t) --> void", pybind11::arg("num_threads"));
		cl.def("size", (std::size_t (mrpt::WorkerThreadsPool::*)() const) &mrpt::WorkerThreadsPool::size, "Get number of working threads \n (New in MRPT 2.4.2) \n\nC++: mrpt::WorkerThreadsPool::size() const --> std::size_t");
		cl.def("clear", (void (mrpt::WorkerThreadsPool::*)()) &mrpt::WorkerThreadsPool::clear, "Stops and deletes all worker threads \n\nC++: mrpt::WorkerThreadsPool::clear() --> void");
		cl.def("pendingTasks", (std::size_t (mrpt::WorkerThreadsPool::*)() const) &mrpt::WorkerThreadsPool::pendingTasks, "Returns the number of enqueued tasks, currently waiting for a free\n working thread to process them.  \n\nC++: mrpt::WorkerThreadsPool::pendingTasks() const --> std::size_t");
		cl.def("name", (void (mrpt::WorkerThreadsPool::*)(const std::string &)) &mrpt::WorkerThreadsPool::name, "Sets the private thread names of threads in this pool.\n Names can be seen from debuggers, profilers, etc. and will follow\n the format `${name}[i]` with `${name}` the value supplied in this method\n \n\n (Method new in MRPT 2.1.5)\n\nC++: mrpt::WorkerThreadsPool::name(const std::string &) --> void", pybind11::arg("name"));
		cl.def("name", (std::string (mrpt::WorkerThreadsPool::*)() const) &mrpt::WorkerThreadsPool::name, "Returns the base name of threads in this pool \n\nC++: mrpt::WorkerThreadsPool::name() const --> std::string");
	}
}
