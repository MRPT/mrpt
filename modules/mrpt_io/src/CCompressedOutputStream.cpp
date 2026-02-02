/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/core/config.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/io/CCompressedOutputStream.h>

#include <cerrno>
#include <cstring>  // strerror
#include <fstream>

#if HAVE_UNISTD_H
#include <unistd.h>
#endif

#include <zlib.h>
#include <zstd.h>

using namespace mrpt::io;
using namespace std;

struct CCompressedOutputStream::Impl
{
  std::string filename;
  CompressionOptions options;

  // Gzip state
  gzFile gz_file = nullptr;

  // Zstd state
  FILE* rawFile = nullptr;
  ZSTD_CStream* zstdStream = nullptr;
  std::vector<uint8_t> zstdOutBuffer;

  // Plain file state
  std::ofstream plainFile;
};

CCompressedOutputStream::CCompressedOutputStream() :
    m_impl(std::make_unique<CCompressedOutputStream::Impl>())
{
}

CCompressedOutputStream::CCompressedOutputStream(
    const std::string& fileName, const OpenMode mode, const CompressionOptions& options) :
    CCompressedOutputStream()
{
  MRPT_START
  std::string err_msg;
  if (!open(fileName, options, err_msg, mode))
  {
    THROW_EXCEPTION_FMT(
        "Error trying to open file: '%s', error: '%s'", fileName.c_str(), err_msg.c_str());
  }
  MRPT_END
}

bool CCompressedOutputStream::open(
    const string& fileName,
    const CompressionOptions& options,
    mrpt::optional_ref<std::string> error_msg,
    const OpenMode mode)
{
  MRPT_START

  close();

  m_impl->filename = fileName;
  m_impl->options = options;

  switch (options.type)
  {
    case CompressionType::Gzip:
    {
      // Clamp compression level for Gzip (0-9)
      const int level = std::min(9, std::max(0, options.level));

      m_impl->gz_file = gzopen(
          fileName.c_str(), format("%cb%i", mode == OpenMode::APPEND ? 'a' : 'w', level).c_str());

      if (m_impl->gz_file == nullptr)
      {
        if (error_msg)
        {
          error_msg.value().get() = std::string(strerror(errno));
        }
        return false;
      }
      break;
    }

    case CompressionType::Zstd:
    {
      m_impl->rawFile = fopen(fileName.c_str(), mode == OpenMode::APPEND ? "ab" : "wb");
      if (m_impl->rawFile == nullptr)
      {
        if (error_msg)
        {
          error_msg.value().get() = std::string(strerror(errno));
        }
        return false;
      }

      m_impl->zstdStream = ZSTD_createCStream();
      if (m_impl->zstdStream == nullptr)
      {
        fclose(m_impl->rawFile);
        m_impl->rawFile = nullptr;
        if (error_msg)
        {
          error_msg.value().get() = "Failed to create Zstd compression stream";
        }
        return false;
      }

      // Clamp compression level for Zstd (1-22)
      const int level = std::min(22, std::max(1, options.level));

      const size_t initResult = ZSTD_initCStream(m_impl->zstdStream, level);
      if (ZSTD_isError(initResult))
      {
        ZSTD_freeCStream(m_impl->zstdStream);
        m_impl->zstdStream = nullptr;
        fclose(m_impl->rawFile);
        m_impl->rawFile = nullptr;
        if (error_msg)
        {
          error_msg.value().get() =
              std::string("Zstd init error: ") + ZSTD_getErrorName(initResult);
        }
        return false;
      }

      // Allocate output buffer
      m_impl->zstdOutBuffer.resize(ZSTD_CStreamOutSize());
      break;
    }

    case CompressionType::None:
    default:
    {
      ios_base::openmode openMode = ios_base::binary | ios_base::out;
      if (mode == OpenMode::APPEND)
      {
        openMode |= ios_base::app;
      }

      m_impl->plainFile.open(fileName.c_str(), openMode);
      if (!m_impl->plainFile.is_open())
      {
        if (error_msg)
        {
          error_msg.value().get() = std::string(strerror(errno));
        }
        return false;
      }
      break;
    }
  }

  return true;

  MRPT_END
}

CCompressedOutputStream::~CCompressedOutputStream() { close(); }

void CCompressedOutputStream::close()
{
  // Flush Zstd stream before closing
  if (m_impl->zstdStream && m_impl->rawFile)
  {
    ZSTD_outBuffer output = {m_impl->zstdOutBuffer.data(), m_impl->zstdOutBuffer.size(), 0};

    const size_t remaining = ZSTD_endStream(m_impl->zstdStream, &output);
    if (output.pos > 0)
    {
      fwrite(m_impl->zstdOutBuffer.data(), 1, output.pos, m_impl->rawFile);
    }

    // Continue flushing if needed
    while (remaining > 0)
    {
      output.pos = 0;
      const size_t r = ZSTD_endStream(m_impl->zstdStream, &output);
      if (output.pos > 0)
      {
        fwrite(m_impl->zstdOutBuffer.data(), 1, output.pos, m_impl->rawFile);
      }
      if (r == 0)
      {
        break;
      }
    }
  }

  if (m_impl->gz_file)
  {
    gzclose(m_impl->gz_file);
    m_impl->gz_file = nullptr;
  }

  if (m_impl->zstdStream)
  {
    ZSTD_freeCStream(m_impl->zstdStream);
    m_impl->zstdStream = nullptr;
  }

  if (m_impl->rawFile)
  {
    fclose(m_impl->rawFile);
    m_impl->rawFile = nullptr;
  }

  if (m_impl->plainFile.is_open())
  {
    m_impl->plainFile.close();
  }

  m_impl->filename.clear();
}

size_t CCompressedOutputStream::Read([[maybe_unused]] void* Buffer, [[maybe_unused]] size_t Count)
{
  THROW_EXCEPTION("Trying to read from an output file stream.");
}

size_t CCompressedOutputStream::Write(const void* Buffer, size_t Count)
{
  if (!fileOpenCorrectly())
  {
    THROW_EXCEPTION("File is not open.");
  }

  switch (m_impl->options.type)
  {
    case CompressionType::Gzip:
    {
      const int bytes_written =
          gzwrite(m_impl->gz_file, const_cast<void*>(Buffer), static_cast<unsigned int>(Count));
      return bytes_written < 0 ? 0 : static_cast<size_t>(bytes_written);
    }

    case CompressionType::Zstd:
    {
      ZSTD_inBuffer input = {Buffer, Count, 0};

      while (input.pos < input.size)
      {
        ZSTD_outBuffer output = {m_impl->zstdOutBuffer.data(), m_impl->zstdOutBuffer.size(), 0};

        const size_t result = ZSTD_compressStream(m_impl->zstdStream, &output, &input);

        if (ZSTD_isError(result))
        {
          return 0;  // Error
        }

        // Write compressed data to file
        if (output.pos > 0)
        {
          const size_t written =
              fwrite(m_impl->zstdOutBuffer.data(), 1, output.pos, m_impl->rawFile);
          if (written != output.pos)
          {
            return 0;  // Write error
          }
        }
      }

      return Count;
    }

    case CompressionType::None:
    default:
    {
      m_impl->plainFile.write(
          static_cast<const char*>(Buffer), static_cast<std::streamsize>(Count));
      return m_impl->plainFile.fail() ? 0 : Count;
    }
  }
}

uint64_t CCompressedOutputStream::getPosition() const
{
  if (!fileOpenCorrectly())
  {
    THROW_EXCEPTION("File is not open.");
  }

  switch (m_impl->options.type)
  {
    case CompressionType::Gzip:
      return static_cast<uint64_t>(gztell(m_impl->gz_file));

    case CompressionType::Zstd:
      if (m_impl->rawFile)
      {
        return static_cast<uint64_t>(ftell(m_impl->rawFile));
      }
      return 0;

    case CompressionType::None:
    default:
    {
      auto& f = const_cast<std::ofstream&>(m_impl->plainFile);
      if (m_impl->plainFile.is_open())
      {
        return static_cast<uint64_t>(f.tellp());
      }
      return 0;
    }
  }
}

bool CCompressedOutputStream::fileOpenCorrectly() const
{
  switch (m_impl->options.type)
  {
    case CompressionType::Gzip:
      return m_impl->gz_file != nullptr;
    case CompressionType::Zstd:
      return m_impl->rawFile != nullptr && m_impl->zstdStream != nullptr;
    case CompressionType::None:
    default:
      return m_impl->plainFile.is_open();
  }
}

uint64_t CCompressedOutputStream::Seek(
    [[maybe_unused]] int64_t Offset, [[maybe_unused]] CStream::TSeekOrigin Origin)
{
  THROW_EXCEPTION("Method not available in this class.");
}

uint64_t CCompressedOutputStream::getTotalBytesCount() const
{
  THROW_EXCEPTION("Method not available in this class.");
}

std::string CCompressedOutputStream::getStreamDescription() const
{
  const char* typeStr = nullptr;
  switch (m_impl->options.type)
  {
    case CompressionType::None:
      typeStr = "uncompressed";
      break;
    case CompressionType::Gzip:
      typeStr = "gzip";
      break;
    case CompressionType::Zstd:
      typeStr = "zstd";
      break;
    default:
      THROW_EXCEPTION("Invalid stream type");
  }

  return mrpt::format(
      "mrpt::io::CCompressedOutputStream for file '%s' (%s, level=%d)", m_impl->filename.c_str(),
      typeStr, m_impl->options.level);
}

std::string CCompressedOutputStream::filePathAtUse() const { return m_impl->filename; }

CompressionType CCompressedOutputStream::getCompressionType() const { return m_impl->options.type; }
