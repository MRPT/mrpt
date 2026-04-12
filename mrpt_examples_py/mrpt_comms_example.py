#!/usr/bin/env python3
"""
mrpt_comms_example.py — TCP and serial communication with mrpt.comms.

Demonstrates:
  - CClientTCPSocket: construction, context manager, connection API
  - CSerialPort: construction, setConfig, open/close lifecycle

NOTE: This example does NOT open actual network connections or serial ports —
it demonstrates the API at the Python level without hardware.  Real usage
would call sock.connect() / port.open() against live endpoints.
"""

from mrpt.comms import CClientTCPSocket, CSerialPort

# ---------------------------------------------------------------------------
# CClientTCPSocket — TCP client
# ---------------------------------------------------------------------------
sock = CClientTCPSocket()
print(f"CClientTCPSocket created: {sock}")
print(f"  isConnected = {sock.isConnected()}")

# Demonstrate context-manager support (no actual connection made here)
with CClientTCPSocket() as s:
    print(f"  inside 'with': {s}")
    # In real usage:
    #   s.connect("192.168.1.10", 9000, timeout_ms=3000)
    #   s.write(b"PING\r\n")
    #   response = s.read(64)
    #   s.close()
print("  context manager exited cleanly ✓")

# ---------------------------------------------------------------------------
# CSerialPort — serial port
# ---------------------------------------------------------------------------
port = CSerialPort()   # no port opened yet
print(f"\nCSerialPort created: {port}")
print(f"  isOpen = {port.isOpen()}")

# Configure before opening (in real usage you'd open a real device)
port.setSerialPortName("/dev/ttyUSB0")
# port.open()   ← would raise if /dev/ttyUSB0 doesn't exist

# Context manager (port already closed, just checks API)
with CSerialPort() as p:
    print(f"  inside 'with' (no port opened): {p}")
    # In real usage:
    #   p = CSerialPort("/dev/ttyUSB0", openNow=True)
    #   p.setConfig(baudRate=115200, parity=0, bits=8, nStopBits=1)
    #   p.setTimeouts(100, 0, 500, 0, 0)
    #   data = p.read(64)
    #   p.write(b"AT\r")
print("  serial context manager exited cleanly ✓")

print("""
Typical real-world usage:

  # TCP
  from mrpt.comms import CClientTCPSocket
  with CClientTCPSocket() as s:
      s.connect("192.168.1.100", 5555, timeout_ms=5000)
      s.write(b"HELLO")
      resp = s.read(256)

  # Serial
  from mrpt.comms import CSerialPort
  with CSerialPort("/dev/ttyUSB0") as p:
      p.setConfig(baudRate=115200)
      p.write(b"AT\\r")
      reply = p.read(64)
""")
