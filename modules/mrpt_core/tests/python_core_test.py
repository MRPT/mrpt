#!/usr/bin/env python3
"""Smoke tests for mrpt.core Python bindings."""
import sys, math

try:
    from mrpt.core import deg2rad, rad2deg, abs_diff_double, reverse_bytes_u32
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.core bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.core import error: {e}", file=sys.stderr)
    sys.exit(1)

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("deg2rad / rad2deg")
check("deg2rad(180) == pi",  abs(deg2rad(180.0) - math.pi) < 1e-12)
check("rad2deg(pi)  == 180", abs(rad2deg(math.pi) - 180.0) < 1e-10)
check("round-trip",          abs(rad2deg(deg2rad(45.0)) - 45.0) < 1e-10)

print("abs_diff")
check("abs_diff_double",     abs(abs_diff_double(3.0, 5.0) - 2.0) < 1e-12)

print("reverse_bytes_u32")
check("reverse 0x01020304",  reverse_bytes_u32(0x01020304) == 0x04030201)

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
