#!/usr/bin/env python3
"""Smoke tests for mrpt.serialization Python bindings."""
import sys

try:
    from mrpt.serialization import CSerializable, objectToBytes, bytesToObject
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.serialization bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.serialization import error: {e}", file=sys.stderr)
    sys.exit(1)

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("CSerializable — round-trip via objectToBytes/bytesToObject")
# Use a concrete serializable class from mrpt.poses
try:
    from mrpt.poses import CPose2D
    obj = CPose2D(1.5, 2.5, 0.3)
    raw = objectToBytes(obj)
    check("objectToBytes returns bytes", isinstance(raw, bytes) and len(raw) > 0,
          f"len={len(raw)}")

    obj2 = bytesToObject(raw)
    check("bytesToObject returns object", obj2 is not None)
    check("round-trip x", abs(obj2.x - 1.5) < 1e-6, f"got x={obj2.x}")
    check("round-trip y", abs(obj2.y - 2.5) < 1e-6, f"got y={obj2.y}")
except ImportError:
    print("  SKIP  round-trip (mrpt.poses not available)")

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
