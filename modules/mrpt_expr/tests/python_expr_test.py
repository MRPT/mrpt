#!/usr/bin/env python3
"""Smoke tests for mrpt.expr Python bindings."""
import sys, math

try:
    from mrpt.expr import CRuntimeCompiledExpression
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.expr bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.expr import error: {e}", file=sys.stderr)
    sys.exit(1)

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("CRuntimeCompiledExpression")
e = CRuntimeCompiledExpression()
e.compile("2 + 3 * 4")
check("2+3*4 == 14", abs(e.eval() - 14.0) < 1e-12)

e2 = CRuntimeCompiledExpression()
e2.compile("x * x", {"x": 5.0})
check("x*x at x=5 == 25", abs(e2.eval() - 25.0) < 1e-12)

# Stress test: the dict passed to compile() is a temporary on the Python side.
# Allocate/free a bunch of garbage dicts afterwards to encourage heap reuse,
# then make sure eval() still returns the correct result (regression test for
# a use-after-free in the pybind11 binding of compile()).
for _ in range(1000):
    _garbage = [{"a": float(i), "b": float(i)} for i in range(50)]
    del _garbage
check("x*x at x=5 == 25 after heap churn", abs(e2.eval() - 25.0) < 1e-12)

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
