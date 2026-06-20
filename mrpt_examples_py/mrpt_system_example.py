#!/usr/bin/env python3
"""
mrpt_system_example.py — System utilities with mrpt.system.

Demonstrates:
  - CTicTac: stopwatch
  - CTimeLogger: named section profiling
  - compute_CRC16 / compute_CRC32: checksum computation
  - encodeBase64 / decodeBase64: base64 codec
  - unitsFormat: SI metric prefix formatting
  - Filesystem helpers: fileExists, extractFileName, pathJoin, etc.
  - Datetime helpers: TTimeParts, dateTimeToString, timeDifference, etc.
"""

import time as _time
from mrpt.system import (
    CTicTac, CTimeLogger,
    compute_CRC16, compute_CRC32,
    encodeBase64, decodeBase64,
    unitsFormat,
    # filesystem
    fileExists, directoryExists, getTempFileName, getcwd,
    extractFileName, extractFileExtension, extractFileDirectory,
    fileNameChangeExtension, pathJoin, toAbsolutePath,
    # datetime
    TTimeParts, buildTimestampFromParts, buildTimestampFromPartsLocalTime,
    timestampToParts, timeDifference, timestampAdd,
    dateTimeToString, dateToString, timeToString, formatTimeInterval,
)

# ── CTicTac (stopwatch) ──────────────────────────────────────────────────────
print("── CTicTac ─────────────────────────────────")
timer = CTicTac()
_time.sleep(0.05)
elapsed = timer.Tac()
print(f"Elapsed ≈ {elapsed:.3f} s  (expected ~0.05 s)")
assert elapsed > 0.03, "timer too short"

# ── CTimeLogger (named section profiling) ────────────────────────────────────
print("\n── CTimeLogger ─────────────────────────────")
logger = CTimeLogger(name="demo")
logger.enter("step_A")
_time.sleep(0.02)
logger.leave("step_A")
logger.enter("step_B")
_time.sleep(0.01)
logger.leave("step_B")
print(logger.getStatsAsText())
assert logger.getMeanTime("step_A") > 0

# ── CRC checksums ────────────────────────────────────────────────────────────
print("── CRC ─────────────────────────────────────")
crc16 = compute_CRC16(b"hello")
crc32 = compute_CRC32(b"hello")
print(f"CRC16('hello') = {crc16:#06x}")
print(f"CRC32('hello') = {crc32:#010x}")
assert crc16 != 0

# ── Base64 ───────────────────────────────────────────────────────────────────
print("\n── Base64 ──────────────────────────────────")
original = b"MRPT Python bindings"
encoded  = encodeBase64(original)
decoded  = decodeBase64(encoded)
print(f"Original : {original}")
print(f"Encoded  : {encoded}")
print(f"Decoded  : {decoded}")
assert decoded == original, "round-trip mismatch"
print("  base64 round-trip ✓")

# ── unitsFormat (SI prefixes) ─────────────────────────────────────────────────
print("\n── unitsFormat ─────────────────────────────")
for val in [0.001, 1.0, 1234.5, 1_500_000]:
    print(f"  {val:>12} → {unitsFormat(val)!r}")

# ── Filesystem helpers ────────────────────────────────────────────────────────
print("\n── Filesystem ──────────────────────────────")
cwd = getcwd()
print(f"getcwd()  = {cwd}")
assert directoryExists(cwd), "current dir must exist"

tmp = getTempFileName()
print(f"temp file = {tmp}")

full_path = "/home/user/data/sensor.bag.gz"
print(f"\nextractFileName('{full_path}')         = {extractFileName(full_path)!r}")
print(f"extractFileExtension('{full_path}')    = {extractFileExtension(full_path)!r}")
print(f"  (ignore_gz=True)                    = {extractFileExtension(full_path, True)!r}")
print(f"extractFileDirectory('{full_path}')    = {extractFileDirectory(full_path)!r}")
print(f"fileNameChangeExtension(..., 'csv')   = {fileNameChangeExtension(full_path, 'csv')!r}")

joined = pathJoin(["/home", "user", "data", "out.txt"])
print(f"pathJoin(['/home','user','data','out.txt']) = {joined!r}")

abs_cwd = toAbsolutePath(".")
print(f"toAbsolutePath('.')  = {abs_cwd!r}")

assert not fileExists("/this/path/does/not/exist/xyz.txt")
print("  fileExists('/nonexistent') = False ✓")

# ── Datetime helpers ─────────────────────────────────────────────────────────
print("\n── Datetime ────────────────────────────────")
# Build a UTC timestamp from parts
parts = TTimeParts()
parts.year   = 2026
parts.month  = 4
parts.day    = 12
parts.hour   = 10
parts.minute = 30
parts.second = 0.0
ts = buildTimestampFromParts(parts)

dt_str = dateTimeToString(ts)
d_str  = dateToString(ts)
t_str  = timeToString(ts)
print(f"Timestamp: {dt_str}")
print(f"Date only: {d_str}")
print(f"Time only: {t_str}")

# timeDifference
ts2 = timestampAdd(ts, 3661.5)   # +1h 1min 1.5s
diff = timeDifference(ts, ts2)
print(f"timeDifference(ts, ts+3661.5s) = {diff:.1f} s")
assert abs(diff - 3661.5) < 0.01

# Decompose back
parts2 = timestampToParts(ts2, False)
print(f"Decomposed: {parts2.year}-{parts2.month:02d}-{parts2.day:02d} "
      f"{parts2.hour:02d}:{parts2.minute:02d}:{parts2.second:.1f} UTC")

# formatTimeInterval
for secs in [45.0, 125.0, 3723.0, 90061.0]:
    print(f"  formatTimeInterval({secs:7.0f} s) = {formatTimeInterval(secs)!r}")

print("\nAll checks ✓")
