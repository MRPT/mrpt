#include <iterator>
#include <memory>
#include <mrpt/comms/CClientTCPSocket.h>
#include <mrpt/comms/CSerialPort.h>
#include <mrpt/io/CStream.h>
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

// mrpt::comms::CClientTCPSocket file:mrpt/comms/CClientTCPSocket.h line:35
struct PyCallBack_mrpt_comms_CClientTCPSocket : public mrpt::comms::CClientTCPSocket {
	using mrpt::comms::CClientTCPSocket::CClientTCPSocket;

	size_t Read(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::comms::CClientTCPSocket *>(this), "Read");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CClientTCPSocket::Read(a0, a1);
	}
	size_t Write(const void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::comms::CClientTCPSocket *>(this), "Write");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CClientTCPSocket::Write(a0, a1);
	}
	uint64_t Seek(int64_t a0, enum mrpt::io::CStream::TSeekOrigin a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::comms::CClientTCPSocket *>(this), "Seek");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CClientTCPSocket::Seek(a0, a1);
	}
	uint64_t getTotalBytesCount() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::comms::CClientTCPSocket *>(this), "getTotalBytesCount");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CClientTCPSocket::getTotalBytesCount();
	}
	uint64_t getPosition() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::comms::CClientTCPSocket *>(this), "getPosition");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CClientTCPSocket::getPosition();
	}
	size_t ReadBufferImmediate(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::comms::CClientTCPSocket *>(this), "ReadBufferImmediate");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CStream::ReadBufferImmediate(a0, a1);
	}
	std::string getStreamDescription() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::comms::CClientTCPSocket *>(this), "getStreamDescription");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CStream::getStreamDescription();
	}
};

// mrpt::comms::CSerialPort file:mrpt/comms/CSerialPort.h line:41
struct PyCallBack_mrpt_comms_CSerialPort : public mrpt::comms::CSerialPort {
	using mrpt::comms::CSerialPort::CSerialPort;

	size_t Read(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::comms::CSerialPort *>(this), "Read");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CSerialPort::Read(a0, a1);
	}
	size_t Write(const void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::comms::CSerialPort *>(this), "Write");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CSerialPort::Write(a0, a1);
	}
	uint64_t Seek(int64_t a0, enum mrpt::io::CStream::TSeekOrigin a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::comms::CSerialPort *>(this), "Seek");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CSerialPort::Seek(a0, a1);
	}
	uint64_t getTotalBytesCount() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::comms::CSerialPort *>(this), "getTotalBytesCount");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CSerialPort::getTotalBytesCount();
	}
	uint64_t getPosition() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::comms::CSerialPort *>(this), "getPosition");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<uint64_t>::value) {
				static pybind11::detail::override_caster_t<uint64_t> caster;
				return pybind11::detail::cast_ref<uint64_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<uint64_t>(std::move(o));
		}
		return CSerialPort::getPosition();
	}
	size_t ReadBufferImmediate(void * a0, size_t a1) override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::comms::CSerialPort *>(this), "ReadBufferImmediate");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>(a0, a1);
			if (pybind11::detail::cast_is_temporary_value_reference<size_t>::value) {
				static pybind11::detail::override_caster_t<size_t> caster;
				return pybind11::detail::cast_ref<size_t>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<size_t>(std::move(o));
		}
		return CStream::ReadBufferImmediate(a0, a1);
	}
	std::string getStreamDescription() const override {
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(static_cast<const mrpt::comms::CSerialPort *>(this), "getStreamDescription");
		if (overload) {
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<std::string>::value) {
				static pybind11::detail::override_caster_t<std::string> caster;
				return pybind11::detail::cast_ref<std::string>(std::move(o), caster);
			}
			else return pybind11::detail::cast_safe<std::string>(std::move(o));
		}
		return CStream::getStreamDescription();
	}
};

void bind_mrpt_comms_CClientTCPSocket(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mrpt::comms::CClientTCPSocket file:mrpt/comms/CClientTCPSocket.h line:35
		pybind11::class_<mrpt::comms::CClientTCPSocket, std::shared_ptr<mrpt::comms::CClientTCPSocket>, PyCallBack_mrpt_comms_CClientTCPSocket, mrpt::io::CStream> cl(M("mrpt::comms"), "CClientTCPSocket", "A TCP socket that can be connected to a TCP server, implementing MRPT's\n CStream interface for passing objects as well as generic read/write methods.\n  Unless otherwise noticed, operations are blocking.\n\n  Note that for convenience, DNS lookup is performed with a timeout\n (default=3000ms), which can be changed by the static member\n CClientTCPSocket::DNS_LOOKUP_TIMEOUT_MS\n \n\n\n ");
		cl.def( pybind11::init( [](){ return new mrpt::comms::CClientTCPSocket(); }, [](){ return new PyCallBack_mrpt_comms_CClientTCPSocket(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_comms_CClientTCPSocket const &o){ return new PyCallBack_mrpt_comms_CClientTCPSocket(o); } ) );
		cl.def( pybind11::init( [](mrpt::comms::CClientTCPSocket const &o){ return new mrpt::comms::CClientTCPSocket(o); } ) );
		cl.def("connect", [](mrpt::comms::CClientTCPSocket &o, const std::string & a0, unsigned short const & a1) -> void { return o.connect(a0, a1); }, "", pybind11::arg("remotePartAddress"), pybind11::arg("remotePartTCPPort"));
		cl.def("connect", (void (mrpt::comms::CClientTCPSocket::*)(const std::string &, unsigned short, unsigned int)) &mrpt::comms::CClientTCPSocket::connect, "Establishes a connection with a remote part.\n \n\n This string can be a host name, like \"server\"\n or \"www.mydomain.org\", or an IP address \"11.22.33.44\".\n \n\n The port on the remote machine to connect to.\n \n\n  The timeout to wait for the connection (0: NO\n TIMEOUT)\n \n\n This method raises an exception if an error is found with a\n textual description of the error.\n\nC++: mrpt::comms::CClientTCPSocket::connect(const std::string &, unsigned short, unsigned int) --> void", pybind11::arg("remotePartAddress"), pybind11::arg("remotePartTCPPort"), pybind11::arg("timeout_ms"));
		cl.def("isConnected", (bool (mrpt::comms::CClientTCPSocket::*)()) &mrpt::comms::CClientTCPSocket::isConnected, "Returns true if this objects represents a successfully connected socket\n\nC++: mrpt::comms::CClientTCPSocket::isConnected() --> bool");
		cl.def("close", (void (mrpt::comms::CClientTCPSocket::*)()) &mrpt::comms::CClientTCPSocket::close, "Closes the connection \n\nC++: mrpt::comms::CClientTCPSocket::close() --> void");
		cl.def("sendString", (void (mrpt::comms::CClientTCPSocket::*)(const std::string &)) &mrpt::comms::CClientTCPSocket::sendString, "Writes a string to the socket.\n \n\n std::exception On communication errors\n\nC++: mrpt::comms::CClientTCPSocket::sendString(const std::string &) --> void", pybind11::arg("str"));
		cl.def("Seek", [](mrpt::comms::CClientTCPSocket &o, int64_t const & a0) -> uint64_t { return o.Seek(a0); }, "", pybind11::arg("off"));
		cl.def("Seek", (uint64_t (mrpt::comms::CClientTCPSocket::*)(int64_t, enum mrpt::io::CStream::TSeekOrigin)) &mrpt::comms::CClientTCPSocket::Seek, "This virtual method has no effect in this implementation over a TCP\n socket, and its use raises an exception \n\nC++: mrpt::comms::CClientTCPSocket::Seek(int64_t, enum mrpt::io::CStream::TSeekOrigin) --> uint64_t", pybind11::arg("off"), pybind11::arg("org"));
		cl.def("getTotalBytesCount", (uint64_t (mrpt::comms::CClientTCPSocket::*)() const) &mrpt::comms::CClientTCPSocket::getTotalBytesCount, "This virtual method has no effect in this implementation over a TCP\n socket, and its use raises an exception \n\nC++: mrpt::comms::CClientTCPSocket::getTotalBytesCount() const --> uint64_t");
		cl.def("getPosition", (uint64_t (mrpt::comms::CClientTCPSocket::*)() const) &mrpt::comms::CClientTCPSocket::getPosition, "This virtual method has no effect in this implementation over a TCP\n socket, and its use raises an exception \n\nC++: mrpt::comms::CClientTCPSocket::getPosition() const --> uint64_t");
		cl.def("readAsync", [](mrpt::comms::CClientTCPSocket &o, void * a0, size_t const & a1) -> size_t { return o.readAsync(a0, a1); }, "", pybind11::arg("Buffer"), pybind11::arg("Count"));
		cl.def("readAsync", [](mrpt::comms::CClientTCPSocket &o, void * a0, size_t const & a1, const int & a2) -> size_t { return o.readAsync(a0, a1, a2); }, "", pybind11::arg("Buffer"), pybind11::arg("Count"), pybind11::arg("timeoutStart_ms"));
		cl.def("readAsync", (size_t (mrpt::comms::CClientTCPSocket::*)(void *, size_t, const int, const int)) &mrpt::comms::CClientTCPSocket::readAsync, "A method for reading from the socket with an optional timeout.\n \n\n The destination of data.\n \n\n The number of bytes to read.\n \n\n The maximum timeout (in milliseconds) to wait for\n the starting of data from the other side.\n \n\n The maximum timeout (in milliseconds) to wait\n for a chunk of data after a previous one.\n  Set timeout's to -1 to block until the desired number of bytes are\n read, or an error happens.\n  \n\n The number of actually read bytes.\n\nC++: mrpt::comms::CClientTCPSocket::readAsync(void *, size_t, const int, const int) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"), pybind11::arg("timeoutStart_ms"), pybind11::arg("timeoutBetween_ms"));
		cl.def("writeAsync", [](mrpt::comms::CClientTCPSocket &o, const void * a0, size_t const & a1) -> size_t { return o.writeAsync(a0, a1); }, "", pybind11::arg("Buffer"), pybind11::arg("Count"));
		cl.def("writeAsync", (size_t (mrpt::comms::CClientTCPSocket::*)(const void *, size_t, const int)) &mrpt::comms::CClientTCPSocket::writeAsync, "A method for writing to the socket with optional timeouts.\n  The method supports writing block by block as the socket allows us to\n write more data.\n \n\n The data.\n \n\n The number of bytes to write.\n \n\n The maximum timeout (in milliseconds) to wait for the\n socket to be available for writing (for each block).\n  Set timeout's to -1 to block until the desired number of bytes are\n written, or an error happens.\n  \n\n The number of actually written bytes.\n\nC++: mrpt::comms::CClientTCPSocket::writeAsync(const void *, size_t, const int) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"), pybind11::arg("timeout_ms"));
		cl.def("getReadPendingBytes", (size_t (mrpt::comms::CClientTCPSocket::*)()) &mrpt::comms::CClientTCPSocket::getReadPendingBytes, "Return the number of bytes already in the receive queue (they can be\n read without waiting) \n\nC++: mrpt::comms::CClientTCPSocket::getReadPendingBytes() --> size_t");
		cl.def("setTCPNoDelay", (int (mrpt::comms::CClientTCPSocket::*)(int)) &mrpt::comms::CClientTCPSocket::setTCPNoDelay, "Set the TCP no delay option of the protocol (Nagle algorithm).\n \n\n New value (0 enable Nagle algorithm, 1 disable).\n \n\n Return a number lower than 0 if any error occurred.\n\nC++: mrpt::comms::CClientTCPSocket::setTCPNoDelay(int) --> int", pybind11::arg("newValue"));
		cl.def("getTCPNoDelay", (int (mrpt::comms::CClientTCPSocket::*)()) &mrpt::comms::CClientTCPSocket::getTCPNoDelay, "Return the value of the TCPNoDelay option. \n\nC++: mrpt::comms::CClientTCPSocket::getTCPNoDelay() --> int");
		cl.def("setSOSendBufffer", (int (mrpt::comms::CClientTCPSocket::*)(int)) &mrpt::comms::CClientTCPSocket::setSOSendBufffer, "Set the size of the SO send buffer. This buffer is used to store data,\n and is sended when is full.\n \n\n New size of the SO send buffer.\n \n\n Return a number lower than 0 if any error occurred.\n\nC++: mrpt::comms::CClientTCPSocket::setSOSendBufffer(int) --> int", pybind11::arg("newValue"));
		cl.def("getSOSendBufffer", (int (mrpt::comms::CClientTCPSocket::*)()) &mrpt::comms::CClientTCPSocket::getSOSendBufffer, "Return the current size of the SO send buffer. \n\nC++: mrpt::comms::CClientTCPSocket::getSOSendBufffer() --> int");
		cl.def("assign", (class mrpt::comms::CClientTCPSocket & (mrpt::comms::CClientTCPSocket::*)(const class mrpt::comms::CClientTCPSocket &)) &mrpt::comms::CClientTCPSocket::operator=, "C++: mrpt::comms::CClientTCPSocket::operator=(const class mrpt::comms::CClientTCPSocket &) --> class mrpt::comms::CClientTCPSocket &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
	{ // mrpt::comms::CSerialPort file:mrpt/comms/CSerialPort.h line:41
		pybind11::class_<mrpt::comms::CSerialPort, std::shared_ptr<mrpt::comms::CSerialPort>, PyCallBack_mrpt_comms_CSerialPort, mrpt::io::CStream> cl(M("mrpt::comms"), "CSerialPort", "A communications serial port implementing the interface mrpt::io::CStream.\n On communication errors (eg. the given port number does not exist,\n timeouts,...), most of the methods will\n raise an exception of the class `std::exception`\n\n  The serial port to open is passed in the constructor in the form of a string\n description, which is platform dependent.\n\n  In Windows they are numbered \"COM1\"-\"COM4\" and \"\\\\.\\COMXXX\" for numbers\n above. It is recomended to always use the prefix \"\\\\.\\\" despite the actual\n port number.\n\n  In Linux the name must refer to the device, for example: \"ttyUSB0\",\"ttyS0\".\n If the name string does not start with \"/\" (an absolute path), the\n constructor will assume the prefix \"/dev/\".\n\n  History:\n    - 1/DEC/2005:  (JLBC) First version\n    - 20/DEC/2006: (JLBC) Integration into the MRPT framework\n    - 12/DEC/2007: (JLBC) Added support for Linux.\n    - 22/AUG/2017: (JLBC) Moved to new module mrpt-comms\n\n \n Add the internal buffer to the Windows implementation also\n \n\n\n ");
		cl.def( pybind11::init( [](const std::string & a0){ return new mrpt::comms::CSerialPort(a0); }, [](const std::string & a0){ return new PyCallBack_mrpt_comms_CSerialPort(a0); } ), "doc");
		cl.def( pybind11::init<const std::string &, bool>(), pybind11::arg("portName"), pybind11::arg("openNow") );

		cl.def( pybind11::init( [](){ return new mrpt::comms::CSerialPort(); }, [](){ return new PyCallBack_mrpt_comms_CSerialPort(); } ) );
		cl.def( pybind11::init( [](PyCallBack_mrpt_comms_CSerialPort const &o){ return new PyCallBack_mrpt_comms_CSerialPort(o); } ) );
		cl.def( pybind11::init( [](mrpt::comms::CSerialPort const &o){ return new mrpt::comms::CSerialPort(o); } ) );
		cl.def("setSerialPortName", (void (mrpt::comms::CSerialPort::*)(const std::string &)) &mrpt::comms::CSerialPort::setSerialPortName, "Sets the serial port to open (it is an error to try to change this while\n open yet).\n \n\n open, close\n\nC++: mrpt::comms::CSerialPort::setSerialPortName(const std::string &) --> void", pybind11::arg("COM_name"));
		cl.def("open", (void (mrpt::comms::CSerialPort::*)()) &mrpt::comms::CSerialPort::open, "Open the port. If is already open results in no action.\n \n\n std::exception On communication errors\n\nC++: mrpt::comms::CSerialPort::open() --> void");
		cl.def("open", (void (mrpt::comms::CSerialPort::*)(const std::string &)) &mrpt::comms::CSerialPort::open, "Open the given serial port. If it is already open and the name does not\n match, an exception is raised.\n \n\n std::exception On communication errors or a different serial\n port already open.\n\nC++: mrpt::comms::CSerialPort::open(const std::string &) --> void", pybind11::arg("COM_name"));
		cl.def("close", (void (mrpt::comms::CSerialPort::*)()) &mrpt::comms::CSerialPort::close, "Close the port. If is already closed, results in no action.\n\nC++: mrpt::comms::CSerialPort::close() --> void");
		cl.def("isOpen", (bool (mrpt::comms::CSerialPort::*)() const) &mrpt::comms::CSerialPort::isOpen, "Returns if port has been correctly open.\n\nC++: mrpt::comms::CSerialPort::isOpen() const --> bool");
		cl.def("purgeBuffers", (void (mrpt::comms::CSerialPort::*)()) &mrpt::comms::CSerialPort::purgeBuffers, "Purge tx and rx buffers.\n \n\n std::exception On communication errors\n\nC++: mrpt::comms::CSerialPort::purgeBuffers() --> void");
		cl.def("setConfig", [](mrpt::comms::CSerialPort &o, int const & a0) -> void { return o.setConfig(a0); }, "", pybind11::arg("baudRate"));
		cl.def("setConfig", [](mrpt::comms::CSerialPort &o, int const & a0, int const & a1) -> void { return o.setConfig(a0, a1); }, "", pybind11::arg("baudRate"), pybind11::arg("parity"));
		cl.def("setConfig", [](mrpt::comms::CSerialPort &o, int const & a0, int const & a1, int const & a2) -> void { return o.setConfig(a0, a1, a2); }, "", pybind11::arg("baudRate"), pybind11::arg("parity"), pybind11::arg("bits"));
		cl.def("setConfig", [](mrpt::comms::CSerialPort &o, int const & a0, int const & a1, int const & a2, int const & a3) -> void { return o.setConfig(a0, a1, a2, a3); }, "", pybind11::arg("baudRate"), pybind11::arg("parity"), pybind11::arg("bits"), pybind11::arg("nStopBits"));
		cl.def("setConfig", (void (mrpt::comms::CSerialPort::*)(int, int, int, int, bool)) &mrpt::comms::CSerialPort::setConfig, "Changes the configuration of the port.\n  \n\n  0:No parity, 1:Odd, 2:Even (WINDOWS ONLY: 3:Mark,\n 4:Space) \n\n The desired baud rate Accepted values: 50 -\n 230400 \n\n Bits per word (typ. 8) Accepted values: 5,6,7,8.\n  \n\n Stop bits (typ. 1) Accepted values: 1,2\n  \n\n Whether to enable the hardware flow control\n (RTS/CTS) (default=no)\n \n\n std::exception On communication errors\n\nC++: mrpt::comms::CSerialPort::setConfig(int, int, int, int, bool) --> void", pybind11::arg("baudRate"), pybind11::arg("parity"), pybind11::arg("bits"), pybind11::arg("nStopBits"), pybind11::arg("enableFlowControl"));
		cl.def("setTimeouts", (void (mrpt::comms::CSerialPort::*)(int, int, int, int, int)) &mrpt::comms::CSerialPort::setTimeouts, "Changes the timeouts of the port, in milliseconds.\n \n\n std::exception On communication errors\n\nC++: mrpt::comms::CSerialPort::setTimeouts(int, int, int, int, int) --> void", pybind11::arg("ReadIntervalTimeout"), pybind11::arg("ReadTotalTimeoutMultiplier"), pybind11::arg("ReadTotalTimeoutConstant"), pybind11::arg("WriteTotalTimeoutMultiplier"), pybind11::arg("WriteTotalTimeoutConstant"));
		cl.def("Read", (size_t (mrpt::comms::CSerialPort::*)(void *, size_t)) &mrpt::comms::CSerialPort::Read, "Implements the virtual method responsible for reading from the stream -\n Unlike CStream::ReadBuffer, this method will not raise an exception on\n zero bytes read, as long as there is not any fatal error in the\n communications.\n \n\n std::exception On communication errors\n\nC++: mrpt::comms::CSerialPort::Read(void *, size_t) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"));
		cl.def("ReadString", [](mrpt::comms::CSerialPort &o) -> std::string { return o.ReadString(); }, "");
		cl.def("ReadString", [](mrpt::comms::CSerialPort &o, const int & a0) -> std::string { return o.ReadString(a0); }, "", pybind11::arg("total_timeout_ms"));
		cl.def("ReadString", [](mrpt::comms::CSerialPort &o, const int & a0, bool * a1) -> std::string { return o.ReadString(a0, a1); }, "", pybind11::arg("total_timeout_ms"), pybind11::arg("out_timeout"));
		cl.def("ReadString", (std::string (mrpt::comms::CSerialPort::*)(const int, bool *, const char *)) &mrpt::comms::CSerialPort::ReadString, "Reads one text line from the serial port in POSIX \"canonical mode\".\n  This method reads from the serial port until one of the characters in\n  are found.\n \n\n A line reception is finished when one of these\n characters is found. Default: LF (10), CR (13).\n \n\n If >0, the maximum number of milliseconds to\n wait.\n \n\n If provided, will hold true on return if a timeout\n ocurred, false on a valid read.\n \n\n The read string, without the final\n \n\n std::exception On communication errors\n\nC++: mrpt::comms::CSerialPort::ReadString(const int, bool *, const char *) --> std::string", pybind11::arg("total_timeout_ms"), pybind11::arg("out_timeout"), pybind11::arg("eol_chars"));
		cl.def("Write", (size_t (mrpt::comms::CSerialPort::*)(const void *, size_t)) &mrpt::comms::CSerialPort::Write, "C++: mrpt::comms::CSerialPort::Write(const void *, size_t) --> size_t", pybind11::arg("Buffer"), pybind11::arg("Count"));
		cl.def("Seek", [](mrpt::comms::CSerialPort &o, int64_t const & a0) -> uint64_t { return o.Seek(a0); }, "", pybind11::arg("off"));
		cl.def("Seek", (uint64_t (mrpt::comms::CSerialPort::*)(int64_t, enum mrpt::io::CStream::TSeekOrigin)) &mrpt::comms::CSerialPort::Seek, "not applicable in a serial port \n\nC++: mrpt::comms::CSerialPort::Seek(int64_t, enum mrpt::io::CStream::TSeekOrigin) --> uint64_t", pybind11::arg("off"), pybind11::arg("o"));
		cl.def("getTotalBytesCount", (uint64_t (mrpt::comms::CSerialPort::*)() const) &mrpt::comms::CSerialPort::getTotalBytesCount, "not applicable in a serial port \n\nC++: mrpt::comms::CSerialPort::getTotalBytesCount() const --> uint64_t");
		cl.def("getPosition", (uint64_t (mrpt::comms::CSerialPort::*)() const) &mrpt::comms::CSerialPort::getPosition, "not applicable in a serial port \n\nC++: mrpt::comms::CSerialPort::getPosition() const --> uint64_t");
		cl.def("assign", (class mrpt::comms::CSerialPort & (mrpt::comms::CSerialPort::*)(const class mrpt::comms::CSerialPort &)) &mrpt::comms::CSerialPort::operator=, "C++: mrpt::comms::CSerialPort::operator=(const class mrpt::comms::CSerialPort &) --> class mrpt::comms::CSerialPort &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
