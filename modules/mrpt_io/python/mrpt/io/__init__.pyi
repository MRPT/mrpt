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
from __future__ import annotations
import mrpt as mrpt
from mrpt.io._bindings import CCompressedInputStream as CCompressedInputStream
from mrpt.io._bindings import CCompressedOutputStream as CCompressedOutputStream
from mrpt.io._bindings import CFileGZInputStream as CFileGZInputStream
from mrpt.io._bindings import CFileGZOutputStream as CFileGZOutputStream
from mrpt.io._bindings import CFileInputStream as CFileInputStream
from mrpt.io._bindings import CFileOutputStream as CFileOutputStream
from mrpt.io._bindings import CMemoryStream as CMemoryStream
from mrpt.io._bindings import CStream as CStream
from mrpt.io._bindings import CompressionOptions as CompressionOptions
from mrpt.io._bindings import CompressionType as CompressionType
from mrpt.io._bindings import OpenMode as OpenMode
from mrpt.io._bindings import SeekOrigin as SeekOrigin
from mrpt.io._bindings import archiveFrom as archiveFrom
from mrpt.io._bindings import detect_compression as detect_compression
from mrpt.io._bindings import file_get_contents as file_get_contents
from mrpt.io._bindings import loadBinaryFile as loadBinaryFile
from mrpt.io._bindings import loadTextFile as loadTextFile
from mrpt.io._bindings import vectorToBinaryFile as vectorToBinaryFile
from mrpt.io._bindings import zip as zip
from . import _bindings
__all__: list = ['CCompressedInputStream', 'CCompressedOutputStream', 'CStream', 'CFileInputStream', 'CFileOutputStream', 'CFileGZInputStream', 'CFileGZOutputStream', 'CMemoryStream', 'OpenMode', 'SeekOrigin', 'CompressionType', 'CompressionOptions', 'archiveFrom', 'detect_compression', 'loadBinaryFile', 'vectorToBinaryFile', 'loadTextFile', 'file_get_contents']
