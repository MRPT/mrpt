"""
mrpt.io — File and memory stream bindings for MRPT.

Provides:
  - CStream         : Abstract base class for all streams
  - CFileInputStream  : Read-only binary file stream
  - CFileOutputStream : Write-only binary file stream
  - CFileGZInputStream  : Transparent gz-compressed input stream
  - CFileGZOutputStream : Transparent gz-compressed output stream
  - CMemoryStream   : In-memory stream buffer
  - OpenMode        : TRUNCATE / APPEND enum
  - SeekOrigin      : sFromBeginning / sFromCurrent / sFromEnd enum
"""

from mrpt.io._bindings import (
    CFileGZInputStream,
    CFileGZOutputStream,
    CFileInputStream,
    CFileOutputStream,
    CMemoryStream,
    CStream,
    OpenMode,
    SeekOrigin,
)

__all__ = [
    "CStream",
    "CFileInputStream",
    "CFileOutputStream",
    "CFileGZInputStream",
    "CFileGZOutputStream",
    "CMemoryStream",
    "OpenMode",
    "SeekOrigin",
]
