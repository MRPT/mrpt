\defgroup mrpt_io_grp [mrpt-io]

C++ library for Input-Output (IO) to raw and compressed files.

[TOC]

# Library mrpt-io


This library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-io-dev

See: \ref mrpt_from_cmake

## Overview

`mrpt-io` defines mrpt::io::CStream, the byte-stream interface used by the
whole of MRPT, and its implementations for files, memory buffers and pipes.
Streams only move raw bytes: to read and write objects (observations, maps,
poses...) wrap a stream in an mrpt::serialization::CArchive (see
\ref mrpt_serialization_grp).

Use it to:
- Read and write files, optionally gzip- or Zstandard-compressed, with
  transparent detection of the compression format on reading.
- Serialize data into memory buffers, e.g. to send them over a network.
- Parse text files line by line, skipping comments.

## Key classes and functions

- mrpt::io::CFileInputStream, mrpt::io::CFileOutputStream: plain binary files.
- mrpt::io::CCompressedInputStream: reads uncompressed, gzip or Zstandard
  files, detecting the format from the file contents.
- mrpt::io::CCompressedOutputStream: writes files with the compression type
  and level given in mrpt::io::CompressionOptions (Zstandard by default).
- mrpt::io::CMemoryStream: a stream over a growing memory buffer.
- mrpt::io::CTextFileLinesParser: reads text files line by line, optionally
  filtering out `#`, `%` or `//` comments.
- mrpt::io::zip::compress_gz_file(), mrpt::io::loadBinaryFile(),
  mrpt::io::file_get_contents(): one-call helpers for whole files.

## Example: compressed text file

```cpp
#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/io/CCompressedOutputStream.h>

// Write a Zstandard-compressed text file:
{
  mrpt::io::CCompressedOutputStream out(
      "log.txt.zst", mrpt::io::OpenMode::TRUNCATE,
      mrpt::io::CompressionOptions(mrpt::io::CompressionType::Zstd, 3));
  out.printf("x=%.3f y=%.3f\n", 1.0, 2.0);
}  // the file is flushed and closed here

// Read it back: the compression format is detected from the file contents.
mrpt::io::CCompressedInputStream in("log.txt.zst");
std::string line;
while (in.getline(line))
{
  std::cout << line << "\n";
}
```

## Examples

- \ref io_compress_example: gzip compression of files and memory blocks.
- \ref io_pipes_example: sending serialized objects between threads through
  an mrpt::io::CPipe.
- Python: [mrpt_io_example.py](pymrpt_example_mrpt_io_example.html).

# Library contents
