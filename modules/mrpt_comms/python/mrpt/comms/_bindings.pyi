"""
Python bindings for mrpt::comms — TCP sockets and serial ports
"""
from __future__ import annotations
import mrpt.io
import typing
__all__: list[str] = ['CClientTCPSocket', 'CSerialPort']
class CClientTCPSocket(mrpt.io.CStream):
    """
    A TCP socket that can be connected to a TCP server, implementing MRPT's CStream interface for passing objects as well as generic read/write methods.
    """
    def __enter__(self) -> CClientTCPSocket:
        ...
    def __exit__(self, arg0: typing.Any, arg1: typing.Any, arg2: typing.Any) -> None:
        ...
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __repr__(self) -> str:
        ...
    def close(self) -> None:
        """
        Closes the connection.
        """
    def connect(self, remotePartAddress: str, remotePartTCPPort: int, timeout_ms: int = 0) -> None:
        """
        Establish a TCP connection to host:port. timeout_ms=0 means no timeout.
        """
    def isConnected(self) -> bool:
        """
        Returns true if this objects represents a successfully connected socket.
        """
    def read(self, count: int, timeout_ms: int = -1) -> bytes:
        """
        Read up to count bytes from the socket, returns bytes object
        """
    def sendString(self, str: str) -> None:
        """
        Send a std::string over the TCP connection
        """
    def write(self, data: bytes, timeout_ms: int = -1) -> int:
        """
        Write bytes to the socket, returns number of bytes written
        """
class CSerialPort(mrpt.io.CStream):
    """
    A communications serial port implementing the interface mrpt::io::CStream.
    """
    def __enter__(self) -> CSerialPort:
        ...
    def __exit__(self, arg0: typing.Any, arg1: typing.Any, arg2: typing.Any) -> None:
        ...
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor; call setSerialPortName() + open() before use
        """
    @typing.overload
    def __init__(self, portName: str, openNow: bool = True) -> None:
        """
        Constructor; opens the named port immediately if openNow=True
        """
    def __repr__(self) -> str:
        ...
    def close(self) -> None:
        """
        Close the port. If is already closed, results in no action.
        """
    def isOpen(self) -> bool:
        """
        Returns if port has been correctly open.
        """
    @typing.overload
    def open(self) -> None:
        """
        Open the port
        """
    @typing.overload
    def open(self, COM_name: str) -> None:
        """
        Open the named port
        """
    def purgeBuffers(self) -> None:
        """
        Purge tx and rx buffers.
        """
    def read(self, count: int) -> bytes:
        """
        Read up to count bytes from the serial port, returns bytes object
        """
    def setConfig(self, baudRate: int, parity: int = 0, bits: int = 8, nStopBits: int = 1, enableFlowControl: bool = False) -> None:
        """
        Configure baud rate and framing (parity: 0=none, 1=odd, 2=even)
        """
    def setSerialPortName(self, portName: str) -> None:
        """
        Set the serial port name (e.g. '/dev/ttyUSB0' or 'COM3')
        """
    def setTimeouts(self, ReadIntervalTimeout: int, ReadTotalTimeoutMultiplier: int, ReadTotalTimeoutConstant: int, WriteTotalTimeoutMultiplier: int, WriteTotalTimeoutConstant: int) -> None:
        """
        Set read/write timeouts in milliseconds
        """
    def write(self, data: bytes) -> int:
        """
        Write bytes to the serial port, returns number of bytes written
        """
