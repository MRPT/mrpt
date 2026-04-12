#!/usr/bin/env python3
"""
mrpt_core_example.py — core utilities with mrpt.core.

Demonstrates:
  - Clock: current time, conversion to/from double
  - WorkerThreadsPool: thread pool for async tasks
  - abs_diff: absolute difference utilities
  - reverse_bytes: endianness helpers
  - deg2rad / rad2deg: angle unit conversion
"""

from mrpt.core import (
    Clock,
    WorkerThreadsPool,
    abs_diff_int, abs_diff_double,
    reverse_bytes_u32, reverse_bytes_u64,
    deg2rad, rad2deg,
)
import math

# Clock
now_sec = Clock.nowDouble()
tp = Clock.fromDouble(123.456)
print(f"Current time: {now_sec:.3f} s")
print(f"Clock.fromDouble(123.456) → toDouble: {Clock.toDouble(tp):.3f}")
assert abs(Clock.toDouble(tp) - 123.456) < 1e-9

# WorkerThreadsPool
pool = WorkerThreadsPool(4)
pool.enqueue(lambda: None)   # fire-and-forget task
print(f"WorkerThreadsPool(4): pendingTasks={pool.pendingTasks()}")

# abs_diff
print(f"\nabs_diff_int(10, 3)    = {abs_diff_int(10, 3)}")    # 7
print(f"abs_diff_double(1.5, 4.0) = {abs_diff_double(1.5, 4.0)}")  # 2.5
assert abs_diff_int(10, 3) == 7

# reverse_bytes
x = 0x12345678
print(f"\nreverse_bytes_u32(0x12345678) = {hex(reverse_bytes_u32(x))}")  # 0x78563412
assert reverse_bytes_u32(x) == 0x78563412

# deg2rad / rad2deg
print(f"\ndeg2rad(180) = {deg2rad(180.0):.6f}  (expect {math.pi:.6f})")
print(f"rad2deg(π)   = {rad2deg(math.pi):.4f}  (expect 180.0)")
assert abs(deg2rad(180.0) - math.pi) < 1e-9
assert abs(rad2deg(math.pi) - 180.0) < 1e-9
print("All checks ✓")
