#!/usr/bin/env python3
"""
mrpt_io_example.py — file and memory streams with mrpt.io.

Demonstrates:
  - CMemoryStream: in-memory read/write
  - CFileOutputStream / CFileInputStream: binary file I/O
  - CFileGZOutputStream / CFileGZInputStream: transparent gz compression
  - Context manager (with statement) support
"""

import tempfile, os
from mrpt.io import (
    CMemoryStream,
    CFileInputStream,
    CFileOutputStream,
    CFileGZInputStream,
    CFileGZOutputStream,
    OpenMode,
    SeekOrigin,
)

# ---------------------------------------------------------------------------
# CMemoryStream — in-memory buffer
# ---------------------------------------------------------------------------
ms = CMemoryStream()
payload = b"Hello MRPT streams!"
n = ms.write(payload)
print(f"CMemoryStream: wrote {n} bytes")
print(f"  size: {ms.getTotalBytesCount()} bytes")

ms.Seek(0)  # rewind
data = ms.read(len(payload))
print(f"  read back: {data}")
assert data == payload

ms.clear()
print(f"  after clear: {ms.getTotalBytesCount()} bytes")

# ---------------------------------------------------------------------------
# CFileOutputStream / CFileInputStream — binary file
# ---------------------------------------------------------------------------
with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as f:
    fname = f.name

try:
    with CFileOutputStream(fname, OpenMode.TRUNCATE) as fout:
        fout.write(b"\x01\x02\x03\x04\x05")
        print(f"\nCFileOutputStream: wrote to {fname}, pos={fout.getPosition()}")

    with CFileInputStream(fname) as fin:
        total = fin.getTotalBytesCount()
        chunk = fin.read(total)
        print(f"CFileInputStream: read {len(chunk)} bytes: {list(chunk)}")
        assert list(chunk) == [1, 2, 3, 4, 5]
finally:
    os.unlink(fname)

# ---------------------------------------------------------------------------
# CFileGZOutputStream / CFileGZInputStream — compressed file
# ---------------------------------------------------------------------------
with tempfile.NamedTemporaryFile(suffix=".bin.gz", delete=False) as f:
    gzname = f.name

try:
    with CFileGZOutputStream(gzname) as gz_out:
        gz_out.write(b"compressed content")
        print(f"\nCFileGZOutputStream: wrote to {gzname}")

    with CFileGZInputStream(gzname) as gz_in:
        data_gz = gz_in.read(100)
        print(f"CFileGZInputStream: read back: {data_gz}")
        assert data_gz == b"compressed content"
        print("  gz round-trip OK ✓")
finally:
    os.unlink(gzname)
