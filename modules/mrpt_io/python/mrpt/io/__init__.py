"""
mrpt.io — File and memory stream bindings for MRPT.

Provides:
  - CStream         : Abstract base class for all streams
  - CFileInputStream  : Read-only binary file stream
  - CFileOutputStream : Write-only binary file stream
  - CFileGZInputStream  : Transparent gz-compressed input stream
  - CFileGZOutputStream : Transparent gz-compressed output stream
  - CCompressedInputStream  : Reads plain, gzip or zstd files (auto-detected)
  - CCompressedOutputStream : Writes plain, gzip or zstd files
  - CMemoryStream   : In-memory stream buffer
  - OpenMode        : TRUNCATE / APPEND enum
  - SeekOrigin      : sFromBeginning / sFromCurrent / sFromEnd enum
  - CompressionType, CompressionOptions, detect_compression()
  - archiveFrom()   : CArchive over a stream, to read/write MRPT objects
  - zip             : gzip compression of memory blocks and files
  - loadBinaryFile(), vectorToBinaryFile(), loadTextFile(), file_get_contents()
"""

import mrpt.serialization  # noqa: F401  (CArchive returned by archiveFrom())

from mrpt.io._bindings import (
    CCompressedInputStream,
    CCompressedOutputStream,
    CFileGZInputStream,
    CFileGZOutputStream,
    CFileInputStream,
    CFileOutputStream,
    CMemoryStream,
    CStream,
    OpenMode,
    SeekOrigin,
    CompressionType,
    CompressionOptions,
    archiveFrom,
    detect_compression,
    loadBinaryFile,
    vectorToBinaryFile,
    loadTextFile,
    file_get_contents,
    zip,
)

__all__ = [
    "CCompressedInputStream",
    "CCompressedOutputStream",
    "CStream",
    "CFileInputStream",
    "CFileOutputStream",
    "CFileGZInputStream",
    "CFileGZOutputStream",
    "CMemoryStream",
    "OpenMode",
    "SeekOrigin",
    "CompressionType",
    "CompressionOptions",
    "archiveFrom",
    "detect_compression",
    "loadBinaryFile",
    "vectorToBinaryFile",
    "loadTextFile",
    "file_get_contents",
    # "zip" is not listed: a star-import would shadow the builtin zip().
]
