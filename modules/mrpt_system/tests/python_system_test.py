#!/usr/bin/env python3
"""Smoke tests for mrpt.system Python bindings."""
import sys, time

try:
    from mrpt.system import (
        CTicTac, CTimeLogger,
        encodeBase64, decodeBase64,
        compute_CRC32, unitsFormat,
        fileExists, extractFileName, extractFileExtension,
    )
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.system bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.system import error: {e}", file=sys.stderr)
    sys.exit(1)

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("CTicTac")
t = CTicTac()
t.Tic()
elapsed = t.Tac()
check("elapsed >= 0", elapsed >= 0.0, f"got {elapsed}")

print("CTimeLogger")
log = CTimeLogger()
log.enter("section1")
log.leave("section1")
check("mean >= 0", log.getMeanTime("section1") >= 0.0)

print("Base64")
encoded = encodeBase64(b"hello mrpt")
decoded = decodeBase64(encoded)
check("round-trip", decoded == b"hello mrpt", f"got {decoded!r}")

print("CRC32")
crc = compute_CRC32(b"test")
check("CRC32 is int", isinstance(crc, int))
check("CRC32 non-zero", crc != 0)

print("unitsFormat")
s = unitsFormat(1500.0, 2, False)
check("unitsFormat returns str", isinstance(s, str) and len(s) > 0, s)

print("fileExists / extractFileName")
check("fileExists(__file__)", fileExists(__file__))
check("extractFileName", extractFileName(__file__) == "python_system_test")
check("extractFileExtension", extractFileExtension(__file__) == "py")

print("VerbosityLevel")
from mrpt.system import VerbosityLevel, LVL_DEBUG, LVL_ERROR
check("VerbosityLevel values", int(LVL_DEBUG) == 0 and int(VerbosityLevel.LVL_INFO) == 1 and
      int(VerbosityLevel.LVL_WARN) == 2 and int(LVL_ERROR) == 3)

print("Plug-in modules")
import os
import mrpt.system._bindings as _sysb
from mrpt.system import loadPluginModule, unloadPluginModule, loadPluginModules
ok, err = loadPluginModule(_sysb.__file__)
check("loadPluginModule", ok and err == "", err)
ok, err = unloadPluginModule(_sysb.__file__)
check("unloadPluginModule", ok and err == "", err)
missing = os.path.join(os.path.dirname(os.path.abspath(__file__)), "no_such_plugin.so")
ok, err = loadPluginModule(missing)
check("loadPluginModule of a missing file", not ok and "no_such_plugin.so" in err, err)
ok, err = loadPluginModules(_sysb.__file__ + "," + missing)
check("loadPluginModules with one missing file", not ok and "no_such_plugin.so" in err, err)

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
