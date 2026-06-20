# mrpt/comms/__init__.py

"""
mrpt-comms Python API — TCP sockets and RS-232 / USB serial ports.

Classes:
  - CClientTCPSocket: connect to a TCP server, send/receive bytes
  - CSerialPort: open and configure a serial port (RS-232 / FTDI / USB-serial)

Both classes support Python context managers (``with`` statement).

Example — TCP::

    from mrpt.comms import CClientTCPSocket
    with CClientTCPSocket() as sock:
        sock.connect("127.0.0.1", 9000, timeout_ms=3000)
        sock.write(b"hello")
        data = sock.read(1024)

Example — Serial::

    from mrpt.comms import CSerialPort
    with CSerialPort("/dev/ttyUSB0", openNow=True) as port:
        port.setConfig(baudRate=115200)
        port.write(b"AT\r")
        response = port.read(64)
"""

import mrpt.io  # noqa: F401  (CStream base type)

from . import _bindings as _b

CClientTCPSocket = _b.CClientTCPSocket
CSerialPort      = _b.CSerialPort

__all__ = [
    "CClientTCPSocket",
    "CSerialPort",
]
