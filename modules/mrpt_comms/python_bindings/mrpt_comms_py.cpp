/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/comms/CClientTCPSocket.h>
#include <mrpt/comms/CSerialPort.h>
#include <mrpt/io/CStream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <string>
#include <vector>

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(_bindings, m)
{
  m.doc() = "Python bindings for mrpt::comms — TCP sockets and serial ports";

  // -------------------------------------------------------------------------
  // CClientTCPSocket — client-side TCP connection
  // -------------------------------------------------------------------------
  py::class_<mrpt::comms::CClientTCPSocket, mrpt::io::CStream>(m, "CClientTCPSocket")
      .def(py::init<>())
      .def(
          "connect",
          [](mrpt::comms::CClientTCPSocket& s, const std::string& host, unsigned short port,
             unsigned int timeout_ms) { s.connect(host, port, timeout_ms); },
          "remotePartAddress"_a, "remotePartTCPPort"_a, "timeout_ms"_a = 0,
          "Establish a TCP connection to host:port. timeout_ms=0 means no timeout.")
      .def("isConnected", &mrpt::comms::CClientTCPSocket::isConnected)
      .def("close", &mrpt::comms::CClientTCPSocket::close)
      .def(
          "sendString", &mrpt::comms::CClientTCPSocket::sendString, "str"_a,
          "Send a std::string over the TCP connection")
      // Read raw bytes from socket (Read/Write are protected; use the public async variants)
      .def(
          "read",
          [](mrpt::comms::CClientTCPSocket& s, size_t count, int timeout_ms)
          {
            std::vector<uint8_t> buf(count);
            const size_t n = s.readAsync(buf.data(), count, timeout_ms, timeout_ms);
            buf.resize(n);
            return py::bytes(reinterpret_cast<const char*>(buf.data()), n);
          },
          "count"_a, "timeout_ms"_a = -1,
          "Read up to count bytes from the socket, returns bytes object")
      // Write raw bytes to socket
      .def(
          "write",
          [](mrpt::comms::CClientTCPSocket& s, const py::bytes& data, int timeout_ms)
          {
            std::string_view sv = data;
            return s.writeAsync(sv.data(), sv.size(), timeout_ms);
          },
          "data"_a, "timeout_ms"_a = -1,
          "Write bytes to the socket, returns number of bytes written")
      // Context manager support
      .def(
          "__enter__",
          [](mrpt::comms::CClientTCPSocket& s) -> mrpt::comms::CClientTCPSocket& { return s; })
      .def(
          "__exit__",
          [](mrpt::comms::CClientTCPSocket& s, py::object, py::object, py::object)
          {
            if (s.isConnected()) s.close();
          })
      .def(
          "__repr__",
          [](const mrpt::comms::CClientTCPSocket& s)
          {
            return "CClientTCPSocket(connected=" +
                   std::string(
                       const_cast<mrpt::comms::CClientTCPSocket&>(s).isConnected() ? "True"
                                                                                   : "False") +
                   ")";
          });

  // -------------------------------------------------------------------------
  // CSerialPort — RS-232 / USB serial port
  // -------------------------------------------------------------------------
  py::class_<mrpt::comms::CSerialPort, mrpt::io::CStream>(m, "CSerialPort")
      .def(py::init<>(), "Default constructor; call setSerialPortName() + open() before use")
      .def(
          py::init<const std::string&, bool>(), "portName"_a, "openNow"_a = true,
          "Constructor; opens the named port immediately if openNow=True")
      .def(
          "setSerialPortName", &mrpt::comms::CSerialPort::setSerialPortName, "portName"_a,
          "Set the serial port name (e.g. '/dev/ttyUSB0' or 'COM3')")
      .def("open", py::overload_cast<>(&mrpt::comms::CSerialPort::open), "Open the port")
      .def(
          "open", py::overload_cast<const std::string&>(&mrpt::comms::CSerialPort::open),
          "COM_name"_a, "Open the named port")
      .def("close", &mrpt::comms::CSerialPort::close)
      .def("isOpen", &mrpt::comms::CSerialPort::isOpen)
      .def(
          "setConfig", &mrpt::comms::CSerialPort::setConfig, "baudRate"_a, "parity"_a = 0,
          "bits"_a = 8, "nStopBits"_a = 1, "enableFlowControl"_a = false,
          "Configure baud rate and framing (parity: 0=none, 1=odd, 2=even)")
      .def(
          "setTimeouts", &mrpt::comms::CSerialPort::setTimeouts, "ReadIntervalTimeout"_a,
          "ReadTotalTimeoutMultiplier"_a, "ReadTotalTimeoutConstant"_a,
          "WriteTotalTimeoutMultiplier"_a, "WriteTotalTimeoutConstant"_a,
          "Set read/write timeouts in milliseconds")
      .def("purgeBuffers", &mrpt::comms::CSerialPort::purgeBuffers)
      // Read bytes
      .def(
          "read",
          [](mrpt::comms::CSerialPort& s, size_t count)
          {
            std::vector<uint8_t> buf(count);
            const size_t n = s.Read(buf.data(), count);
            buf.resize(n);
            return py::bytes(reinterpret_cast<const char*>(buf.data()), n);
          },
          "count"_a, "Read up to count bytes from the serial port, returns bytes object")
      // Write bytes
      .def(
          "write",
          [](mrpt::comms::CSerialPort& s, const py::bytes& data)
          {
            std::string_view sv = data;
            return s.Write(sv.data(), sv.size());
          },
          "data"_a, "Write bytes to the serial port, returns number of bytes written")
      // Context manager support
      .def("__enter__", [](mrpt::comms::CSerialPort& s) -> mrpt::comms::CSerialPort& { return s; })
      .def(
          "__exit__",
          [](mrpt::comms::CSerialPort& s, py::object, py::object, py::object)
          {
            if (s.isOpen()) s.close();
          })
      .def(
          "__repr__", [](const mrpt::comms::CSerialPort& s)
          { return std::string("CSerialPort(open=") + (s.isOpen() ? "True" : "False") + ")"; });
}
