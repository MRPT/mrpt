#include <iterator>
#include <memory>
#include <mrpt/comms/CClientTCPSocket.h>
#include <mrpt/comms/CServerTCPSocket.h>
#include <mrpt/system/COutputLogger.h>
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

void bind_mrpt_comms_CServerTCPSocket(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::comms::CServerTCPSocket file:mrpt/comms/CServerTCPSocket.h line:25
		pybind11::class_<mrpt::comms::CServerTCPSocket, std::shared_ptr<mrpt::comms::CServerTCPSocket>> cl(M("mrpt::comms"), "CServerTCPSocket", "A TCP socket that can be wait for client connections to enter.\n  Unless otherwise noticed, operations are blocking.\n \n\n\n ");
		cl.def( pybind11::init( [](unsigned short const & a0){ return new mrpt::comms::CServerTCPSocket(a0); } ), "doc" , pybind11::arg("listenPort"));
		cl.def( pybind11::init( [](unsigned short const & a0, const std::string & a1){ return new mrpt::comms::CServerTCPSocket(a0, a1); } ), "doc" , pybind11::arg("listenPort"), pybind11::arg("IPaddress"));
		cl.def( pybind11::init( [](unsigned short const & a0, const std::string & a1, int const & a2){ return new mrpt::comms::CServerTCPSocket(a0, a1, a2); } ), "doc" , pybind11::arg("listenPort"), pybind11::arg("IPaddress"), pybind11::arg("maxConnectionsWaiting"));
		cl.def( pybind11::init<unsigned short, const std::string &, int, enum mrpt::system::VerbosityLevel>(), pybind11::arg("listenPort"), pybind11::arg("IPaddress"), pybind11::arg("maxConnectionsWaiting"), pybind11::arg("verbosityLevel") );

		cl.def( pybind11::init( [](mrpt::comms::CServerTCPSocket const &o){ return new mrpt::comms::CServerTCPSocket(o); } ) );
		cl.def("isListening", (bool (mrpt::comms::CServerTCPSocket::*)()) &mrpt::comms::CServerTCPSocket::isListening, "Returns true if the socket was successfully open and it's bound to the\n desired port. \n\nC++: mrpt::comms::CServerTCPSocket::isListening() --> bool");
		cl.def("accept", [](mrpt::comms::CServerTCPSocket &o) -> std::unique_ptr<class mrpt::comms::CClientTCPSocket> { return o.accept(); }, "");
		cl.def("accept", (class std::unique_ptr<class mrpt::comms::CClientTCPSocket> (mrpt::comms::CServerTCPSocket::*)(int)) &mrpt::comms::CServerTCPSocket::accept, "Waits for an incoming connection (indefinitely, or with a given timeout)\n The returned object represents the new connection, and MUST BE deleted\n by the user when no longer needed.\n \n\n The timeout for the waiting, in milliseconds. Set this\n to \"-1\" to disable timeout (i.e. timeout=infinite)\n \n\n The incoming connection, or nullptr on timeout or error.\n\nC++: mrpt::comms::CServerTCPSocket::accept(int) --> class std::unique_ptr<class mrpt::comms::CClientTCPSocket>", pybind11::arg("timeout_ms"));
		cl.def("assign", (class mrpt::comms::CServerTCPSocket & (mrpt::comms::CServerTCPSocket::*)(const class mrpt::comms::CServerTCPSocket &)) &mrpt::comms::CServerTCPSocket::operator=, "C++: mrpt::comms::CServerTCPSocket::operator=(const class mrpt::comms::CServerTCPSocket &) --> class mrpt::comms::CServerTCPSocket &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
