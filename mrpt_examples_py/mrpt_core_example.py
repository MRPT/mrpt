#!/usr/bin/env python3
"""
mrpt_core_example.py — core utilities with mrpt.core.

Demonstrates:
  - Clock: current time, conversion to/from double
  - WorkerThreadsPool: thread pool for async tasks
  - abs_diff: absolute difference utilities
  - reverse_bytes: endianness helpers
  - deg2rad / rad2deg: angle unit conversion
  - get_env / get_env_int: environment variable access
  - from_string_int / from_string_double: safe string-to-number parsing
  - format1d / format1s: printf-style single-argument formatting
"""

import math
import os
from mrpt.core import (
    Clock,
    WorkerThreadsPool,
    abs_diff_int, abs_diff_double,
    reverse_bytes_u32, reverse_bytes_u64,
    deg2rad, rad2deg,
    get_env, get_env_int,
    from_string_int, from_string_double,
    format1d, format1s,
)

# ── Clock ─────────────────────────────────────────────────────────────────────
print("── Clock ───────────────────────────────────")
now_sec = Clock.nowDouble()
tp = Clock.fromDouble(123.456)
print(f"Current time: {now_sec:.3f} s")
print(f"Clock.fromDouble(123.456) → toDouble: {Clock.toDouble(tp):.3f}")
assert abs(Clock.toDouble(tp) - 123.456) < 1e-9
print("  Clock round-trip ✓")

# ── WorkerThreadsPool ─────────────────────────────────────────────────────────
print("\n── WorkerThreadsPool ───────────────────────")
pool = WorkerThreadsPool(4)
pool.enqueue(lambda: None)   # fire-and-forget task
print(f"WorkerThreadsPool(4): size={pool.size()}, pendingTasks={pool.pendingTasks()}")

# ── abs_diff ──────────────────────────────────────────────────────────────────
print("\n── abs_diff ────────────────────────────────")
print(f"abs_diff_int(10, 3)      = {abs_diff_int(10, 3)}")      # 7
print(f"abs_diff_double(1.5, 4.0)= {abs_diff_double(1.5, 4.0)}")  # 2.5
assert abs_diff_int(10, 3) == 7
assert abs(abs_diff_double(1.5, 4.0) - 2.5) < 1e-9
print("  abs_diff ✓")

# ── reverse_bytes ─────────────────────────────────────────────────────────────
print("\n── reverse_bytes ───────────────────────────")
x = 0x12345678
print(f"reverse_bytes_u32(0x12345678) = {hex(reverse_bytes_u32(x))}")  # 0x78563412
assert reverse_bytes_u32(x) == 0x78563412
print("  reverse_bytes ✓")

# ── deg2rad / rad2deg ────────────────────────────────────────────────────────
print("\n── deg2rad / rad2deg ───────────────────────")
print(f"deg2rad(180) = {deg2rad(180.0):.6f}  (expect {math.pi:.6f})")
print(f"rad2deg(pi)  = {rad2deg(math.pi):.4f}  (expect 180.0000)")
assert abs(deg2rad(180.0) - math.pi) < 1e-9
assert abs(rad2deg(math.pi) - 180.0) < 1e-9
print("  angle conversions ✓")

# ── get_env ───────────────────────────────────────────────────────────────────
print("\n── get_env ─────────────────────────────────")
# Use a well-known var that should exist in most environments
path_val = get_env("PATH", "")
print(f"get_env('PATH', '')   = {path_val[:40]!r}...")
assert path_val != "", "PATH env var should be set"

# Non-existent variable → default value
missing = get_env("MRPT_TEST_NONEXISTENT_VAR_XYZ", "default_val")
print(f"get_env('nonexistent') = {missing!r}")
assert missing == "default_val"

# Integer env var
os.environ["MRPT_TEST_INT"] = "42"
ival = get_env_int("MRPT_TEST_INT", 0)
print(f"get_env_int('MRPT_TEST_INT') = {ival}")
assert ival == 42
del os.environ["MRPT_TEST_INT"]
print("  get_env ✓")

# ── from_string ───────────────────────────────────────────────────────────────
print("\n── from_string ─────────────────────────────")
print(f"from_string_int('42')          = {from_string_int('42')}")
print(f"from_string_int('bad', -1)     = {from_string_int('bad', -1)}")
print(f"from_string_double('3.14')     = {from_string_double('3.14'):.4f}")
print(f"from_string_double('x', 0.0)  = {from_string_double('x', 0.0)}")
assert from_string_int("42") == 42
assert from_string_int("bad", -1) == -1
assert abs(from_string_double("3.14") - 3.14) < 1e-9
print("  from_string ✓")

# ── format1d / format1s ───────────────────────────────────────────────────────
print("\n── format1d / format1s ─────────────────────")
s = format1d("Value = %.4f", 3.14159)
print(f"format1d('Value = %.4f', pi) = {s!r}")
assert "3.1416" in s

s2 = format1s("Hello, %s!", "MRPT")
print(f"format1s('Hello, %s!', 'MRPT') = {s2!r}")
assert s2 == "Hello, MRPT!"
print("  format ✓")

print("\nAll checks ✓")
