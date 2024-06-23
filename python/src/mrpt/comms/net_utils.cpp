#include <iterator>
#include <memory>
#include <mrpt/comms/net_utils.h>
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

void bind_mrpt_comms_net_utils(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	// mrpt::comms::net::http_errorcode file:mrpt/comms/net_utils.h line:33
	pybind11::enum_<mrpt::comms::net::http_errorcode>(M("mrpt::comms::net"), "http_errorcode", "Possible returns from a HTTP request. ")
		.value("Ok", mrpt::comms::net::http_errorcode::Ok)
		.value("BadURL", mrpt::comms::net::http_errorcode::BadURL)
		.value("CouldntConnect", mrpt::comms::net::http_errorcode::CouldntConnect)
		.value("NotFound", mrpt::comms::net::http_errorcode::NotFound)
		.value("OtherHTTPError", mrpt::comms::net::http_errorcode::OtherHTTPError);

;

	{ // mrpt::comms::net::HttpRequestOptions file:mrpt/comms/net_utils.h line:42
		pybind11::class_<mrpt::comms::net::HttpRequestOptions, std::shared_ptr<mrpt::comms::net::HttpRequestOptions>> cl(M("mrpt::comms::net"), "HttpRequestOptions", "");
		cl.def( pybind11::init( [](mrpt::comms::net::HttpRequestOptions const &o){ return new mrpt::comms::net::HttpRequestOptions(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::comms::net::HttpRequestOptions(); } ) );
		cl.def_readwrite("port", &mrpt::comms::net::HttpRequestOptions::port);
		cl.def_readwrite("auth_user", &mrpt::comms::net::HttpRequestOptions::auth_user);
		cl.def_readwrite("auth_pass", &mrpt::comms::net::HttpRequestOptions::auth_pass);
		cl.def_readwrite("extra_headers", &mrpt::comms::net::HttpRequestOptions::extra_headers);
		cl.def_readwrite("timeout_ms", &mrpt::comms::net::HttpRequestOptions::timeout_ms);
		cl.def("assign", (struct mrpt::comms::net::HttpRequestOptions & (mrpt::comms::net::HttpRequestOptions::*)(const struct mrpt::comms::net::HttpRequestOptions &)) &mrpt::comms::net::HttpRequestOptions::operator=, "C++: mrpt::comms::net::HttpRequestOptions::operator=(const struct mrpt::comms::net::HttpRequestOptions &) --> struct mrpt::comms::net::HttpRequestOptions &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::comms::net::HttpRequestOutput file:mrpt/comms/net_utils.h line:56
		pybind11::class_<mrpt::comms::net::HttpRequestOutput, std::shared_ptr<mrpt::comms::net::HttpRequestOutput>> cl(M("mrpt::comms::net"), "HttpRequestOutput", "");
		cl.def( pybind11::init( [](mrpt::comms::net::HttpRequestOutput const &o){ return new mrpt::comms::net::HttpRequestOutput(o); } ) );
		cl.def( pybind11::init( [](){ return new mrpt::comms::net::HttpRequestOutput(); } ) );
		cl.def_readwrite("http_responsecode", &mrpt::comms::net::HttpRequestOutput::http_responsecode);
		cl.def_readwrite("errormsg", &mrpt::comms::net::HttpRequestOutput::errormsg);
		cl.def_readwrite("out_headers", &mrpt::comms::net::HttpRequestOutput::out_headers);
		cl.def("assign", (struct mrpt::comms::net::HttpRequestOutput & (mrpt::comms::net::HttpRequestOutput::*)(const struct mrpt::comms::net::HttpRequestOutput &)) &mrpt::comms::net::HttpRequestOutput::operator=, "C++: mrpt::comms::net::HttpRequestOutput::operator=(const struct mrpt::comms::net::HttpRequestOutput &) --> struct mrpt::comms::net::HttpRequestOutput &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	// mrpt::comms::net::DNS_resolve_async(const std::string &, std::string &, const unsigned int) file:mrpt/comms/net_utils.h line:115
	M("mrpt::comms::net").def("DNS_resolve_async", [](const std::string & a0, std::string & a1) -> bool { return mrpt::comms::net::DNS_resolve_async(a0, a1); }, "", pybind11::arg("server_name"), pybind11::arg("out_ip"));
	M("mrpt::comms::net").def("DNS_resolve_async", (bool (*)(const std::string &, std::string &, const unsigned int)) &mrpt::comms::net::DNS_resolve_async, "Resolve a server address by its name, returning its IP address as a\n string - This method has a timeout for the maximum time to wait for\n the DNS server.  For example: server_name=\"www.google.com\" ->\n out_ip=\"209.85.227.99\"\n\n \n true on success, false on timeout or other error.\n\nC++: mrpt::comms::net::DNS_resolve_async(const std::string &, std::string &, const unsigned int) --> bool", pybind11::arg("server_name"), pybind11::arg("out_ip"), pybind11::arg("timeout_ms"));

	// mrpt::comms::net::getLastSocketErrorStr() file:mrpt/comms/net_utils.h line:119
	M("mrpt::comms::net").def("getLastSocketErrorStr", (std::string (*)()) &mrpt::comms::net::getLastSocketErrorStr, "Returns a description of the last Sockets error \n\nC++: mrpt::comms::net::getLastSocketErrorStr() --> std::string");

	// mrpt::comms::net::Ping(const std::string &, const int, std::string *) file:mrpt/comms/net_utils.h line:135
	M("mrpt::comms::net").def("Ping", [](const std::string & a0, const int & a1) -> bool { return mrpt::comms::net::Ping(a0, a1); }, "", pybind11::arg("address"), pybind11::arg("max_attempts"));
	M("mrpt::comms::net").def("Ping", (bool (*)(const std::string &, const int, std::string *)) &mrpt::comms::net::Ping, "Ping an IP address\n\n \n Address to ping.\n \n\n Number of attempts to try and ping.\n \n\n String containing output information\n\n \n True if responsive, false otherwise.\n\n \n { I am redirecting stderr to stdout, so that the overall process\n is simplified.  Otherwise see:\n https://jineshkj.wordpress.com/2006/12/22/how-to-capture-stdin-stdout-and-stderr-of-child-program/\n }\n\n \n\nC++: mrpt::comms::net::Ping(const std::string &, const int, std::string *) --> bool", pybind11::arg("address"), pybind11::arg("max_attempts"), pybind11::arg("output_str"));

}
