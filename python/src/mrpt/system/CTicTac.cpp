#include <mrpt/system/CTicTac.h>
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

void bind_mrpt_system_CTicTac(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::system::CTicTac file:mrpt/system/CTicTac.h line:21
		pybind11::class_<mrpt::system::CTicTac, std::shared_ptr<mrpt::system::CTicTac>> cl(M("mrpt::system"), "CTicTac", "A high-performance stopwatch, with typical resolution of nanoseconds.\n\n This always uses the system MONOTONIC clock, despite the setting in\n mrpt::Clock.\n\n  \n The class is named after the Spanish equivalent of \"Tic-Toc\" ;-)\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::system::CTicTac(); } ) );
		cl.def( pybind11::init( [](mrpt::system::CTicTac const &o){ return new mrpt::system::CTicTac(o); } ) );
		cl.def("Tic", (void (mrpt::system::CTicTac::*)()) &mrpt::system::CTicTac::Tic, "Starts the stopwatch. \n Tac() \n\nC++: mrpt::system::CTicTac::Tic() --> void");
		cl.def("Tac", (double (mrpt::system::CTicTac::*)() const) &mrpt::system::CTicTac::Tac, "Stops the stopwatch.  \n Returns the ellapsed time in seconds.\n \n\n Tic() \n\nC++: mrpt::system::CTicTac::Tac() const --> double");
		cl.def("assign", (class mrpt::system::CTicTac & (mrpt::system::CTicTac::*)(const class mrpt::system::CTicTac &)) &mrpt::system::CTicTac::operator=, "C++: mrpt::system::CTicTac::operator=(const class mrpt::system::CTicTac &) --> class mrpt::system::CTicTac &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
