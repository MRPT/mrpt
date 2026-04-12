#!/usr/bin/env python3
"""Smoke tests for mrpt.config Python bindings."""
import sys, tempfile, os

try:
    from mrpt.config import CConfigFileMemory, CConfigFile
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.config bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.config import error: {e}", file=sys.stderr)
    sys.exit(1)

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("CConfigFileMemory")
cfg = CConfigFileMemory()
cfg.write("section1", "key_int", 42)
cfg.write("section1", "key_str", "hello")
cfg.write("section1", "key_float", 3.14)

check("read int",   cfg.read_int("section1", "key_int", 0) == 42)
check("read str",   cfg.read_string("section1", "key_str", "") == "hello")
check("read float", abs(cfg.read_float("section1", "key_float", 0.0) - 3.14) < 1e-5)
check("default",    cfg.read_int("section1", "missing", 99) == 99)

sections = cfg.getAllSections()
check("getAllSections", "section1" in sections, str(sections))

print("CConfigFile (round-trip via file)")
with tempfile.NamedTemporaryFile(suffix=".ini", delete=False, mode="w") as f:
    f.write("[sec]\nval = 123\n")
    fname = f.name
try:
    cfg2 = CConfigFile(fname)
    check("read from file", cfg2.read_int("sec", "val", 0) == 123)
finally:
    os.unlink(fname)

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
