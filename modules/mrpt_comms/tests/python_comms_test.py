#!/usr/bin/env python3
"""Smoke tests for mrpt.comms Python bindings (no hardware required)."""
import sys

try:
    from mrpt.comms import CClientTCPSocket, CSerialPort
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.comms bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.comms import error: {e}", file=sys.stderr)
    sys.exit(1)

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("CClientTCPSocket — instantiation only (no network call)")
sock = CClientTCPSocket()
check("CClientTCPSocket created", sock is not None)
check("not connected initially", not sock.isConnected())

print("CSerialPort — instantiation only (no hardware)")
port = CSerialPort()
check("CSerialPort created", port is not None)
check("not open initially", not port.isOpen())

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
