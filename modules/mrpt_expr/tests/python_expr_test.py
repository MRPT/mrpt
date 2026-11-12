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

# Recompiling must use the new variables, not those of the first compile():
e3 = CRuntimeCompiledExpression()
e3.compile("x", {"x": 1.0})
e3.compile("x", {"x": 2.0})
for _ in range(1000):
    _garbage = [{"a": float(i), "b": float(i)} for i in range(50)]
    del _garbage
check("recompile uses the new variables", e3.eval() == 2.0, f"got {e3.eval()}")

print("register_function")
k = 2.0
e4 = CRuntimeCompiledExpression()
e4.register_function("seven", lambda: 7.0)
e4.register_function("scale", lambda v: k * v)
e4.register_function("my_avg", lambda a, b: (a + b) / 2.0)
e4.register_function("mid3", lambda a, b, c: (a + b + c) / 3.0)
e4.compile("seven() + scale(3) + my_avg(10, 20) + mid3(1, 2, 6)")
check("Python functions with 0-3 arguments", e4.eval() == 7.0 + 6.0 + 15.0 + 3.0, f"got {e4.eval()}")
k = 3.0
check("Python closures are called upon each eval()", e4.eval() == 7.0 + 9.0 + 15.0 + 3.0,
      f"got {e4.eval()}")

e5 = CRuntimeCompiledExpression()
e5.register_function("avg", lambda a, b: a * b)
try:
    e5.compile("avg(2, 3)")
    check("built-in function names are rejected", False)
except Exception:
    check("built-in function names are rejected", True)

try:
    e5.register_function("f4", lambda a, b, c, d: a)
    check("functions with 4 arguments are rejected", False)
except ValueError:
    check("functions with 4 arguments are rejected", True)

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
