#!/usr/bin/env python3
"""Smoke tests for mrpt.io Python bindings."""
import sys, tempfile, os

try:
    from mrpt.io import (
        CMemoryStream, CFileOutputStream, CFileInputStream,
        OpenMode, SeekOrigin,
    )
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.io bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.io import error: {e}", file=sys.stderr)
    sys.exit(1)

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("CMemoryStream")
ms = CMemoryStream()
data = b"hello mrpt io"
ms.write(data)
ms.Seek(0, SeekOrigin.sFromBeginning)
back = ms.read(len(data))
check("round-trip", back == data, f"got {back!r}")
check("getTotalBytesCount", ms.getTotalBytesCount() == len(data))

print("CFileOutputStream / CFileInputStream")
with tempfile.NamedTemporaryFile(delete=False, suffix=".bin") as f:
    fname = f.name
try:
    with CFileOutputStream(fname, OpenMode.TRUNCATE) as fout:
        fout.write(b"\x01\x02\x03\x04")
    with CFileInputStream(fname) as fin:
        buf = fin.read(4)
    check("file round-trip", buf == b"\x01\x02\x03\x04", f"got {buf!r}")
finally:
    os.unlink(fname)

print("Enums")
check("OpenMode.TRUNCATE exists", hasattr(OpenMode, "TRUNCATE"))
check("SeekOrigin.sFromBeginning exists", hasattr(SeekOrigin, "sFromBeginning"))

print("zip / compression")
import mrpt.io
from mrpt.io import CompressionType, CompressionOptions, detect_compression
payload = b"mrpt " * 200
gz = mrpt.io.zip.compress_gz_data_block(payload)
check("gz block smaller", len(gz) < len(payload), f"{len(gz)} vs {len(payload)}")
check("gz block round-trip", mrpt.io.zip.decompress_gz_data_block(gz) == payload)
with tempfile.TemporaryDirectory() as tmpdir:
    gzfile = os.path.join(tmpdir, "data.gz")
    check("compress_gz_file", mrpt.io.zip.compress_gz_file(gzfile, payload))
    check("detect_compression gzip", detect_compression(gzfile) == CompressionType.Gzip)
    check("decompress_gz_file", mrpt.io.zip.decompress_gz_file(gzfile) == payload)
    binfile = os.path.join(tmpdir, "data.bin")
    check("vectorToBinaryFile", mrpt.io.vectorToBinaryFile(b"\x05\x06", binfile))
    back = mrpt.io.loadBinaryFile(binfile)
    check("loadBinaryFile", back is not None and bytes(back) == b"\x05\x06", f"got {back!r}")
    check("decompress_gz_file missing -> None",
          mrpt.io.zip.decompress_gz_file(os.path.join(tmpdir, "nope.gz")) is None)
co = CompressionOptions(CompressionType.Gzip, 5)
check("CompressionOptions", co.type == CompressionType.Gzip and co.level == 5)

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
