#!/usr/bin/env python3
"""Smoke tests for mrpt.rtti Python bindings."""
import sys

try:
    from mrpt.rtti import getAllRegisteredClasses, findRegisteredClass, registerAllPendingClasses
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.rtti bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.rtti import error: {e}", file=sys.stderr)
    sys.exit(1)

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("RTTI registry")
registerAllPendingClasses()
classes = getAllRegisteredClasses()
# Note: the registry may be empty if no modules with registered classes are loaded.
# Just verify the functions are callable and return lists.
check("getAllRegisteredClasses returns list", isinstance(classes, list), f"got {type(classes)}")

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
