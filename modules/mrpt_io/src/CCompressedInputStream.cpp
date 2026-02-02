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
#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/system/filesystem.h>

#include <cerrno>
#include <cstring>  // strerror
#include <fstream>
#include <type_traits>
#if HAVE_UNISTD_H
#include <unistd.h>
#endif

#include <zlib.h>
#include <zstd.h>

using namespace mrpt::io;
using namespace std;

static_assert(
    !std::is_copy_constructible_v<CCompressedInputStream> &&
        !std::is_copy_assignable_v<CCompressedInputStream>,
    "Copy Check");

// Magic bytes for file format detection
static constexpr uint8_t GZIP_MAGIC1 = 0x1f;
static constexpr uint8_t GZIP_MAGIC2 = 0x8b;
static constexpr uint32_t ZSTD_MAGIC = 0xFD2FB528;  // Little-endian

struct CCompressedInputStream::Impl
{
  std::string filename;
  CompressionType compressionType = CompressionType::None;

  // Gzip state
  FILE* gz_raw_file = nullptr;  // Underlying FILE* for compressed position tracking
  gzFile gz_file = nullptr;
  uint64_t gzCompressedBytesRead = 0;

  // Zstd state
  FILE* rawFile = nullptr;
  ZSTD_DStream* zstdStream = nullptr;
  std::vector<uint8_t> zstdInBuffer;
  ZSTD_inBuffer zstdInput = {nullptr, 0, 0};
  uint64_t zstdCompressedBytesRead = 0;

  // Plain file state
  std::ifstream plainFile;

  // Stats
  uint64_t bytesReadUncompressed = 0;
  uint64_t uncompressedSizeHint = 0;
};

CCompressedInputStream::CCompressedInputStream() :
    m_impl(std::make_unique<CCompressedInputStream::Impl>())
{
}

CCompressedInputStream::CCompressedInputStream(const string& fileName) : CCompressedInputStream()
{
  MRPT_START
  open(fileName);
  MRPT_END
}

namespace
{
CompressionType detectCompressionType(const std::string& fileName)
{
  std::ifstream f(fileName, std::ios::binary);
  if (!f.is_open())
  {
    return CompressionType::None;
  }

  uint8_t header[4];
  f.read(reinterpret_cast<char*>(header), 4);
  const auto bytesRead = f.gcount();

  if (bytesRead < 2)
  {
    return CompressionType::None;
  }

  // Check for Gzip magic
  if (header[0] == GZIP_MAGIC1 && header[1] == GZIP_MAGIC2)
  {
    return CompressionType::Gzip;
  }

  // Check for Zstd magic
  if (bytesRead >= 4)
  {
    const uint32_t magic =
        static_cast<uint32_t>(header[0]) | (static_cast<uint32_t>(header[1]) << 8) |
        (static_cast<uint32_t>(header[2]) << 16) | (static_cast<uint32_t>(header[3]) << 24);
    if (magic == ZSTD_MAGIC)
    {
      return CompressionType::Zstd;
    }
  }

  return CompressionType::None;
}
}  // namespace

bool CCompressedInputStream::open(
    const std::string& fileName, mrpt::optional_ref<std::string> error_msg)
{
  MRPT_START

  close();

  // Get file size

  try
  {
    m_file_size = mrpt::system::getFileSize(fileName);
  }
  catch (const std::exception& e)
  {
    (void)e;
    m_file_size = uint64_t(-1);
  }

  if (m_file_size == uint64_t(-1))
  {
    if (error_msg)
    {
      error_msg.value().get() = mrpt::format("Couldn't access the file '%s'", fileName.c_str());
    }
    return false;
  }

  m_impl->filename = fileName;
  m_impl->compressionType = detectCompressionType(fileName);

  switch (m_impl->compressionType)
  {
    case CompressionType::Gzip:
    {
      // Open FILE* first to track compressed position
      m_impl->gz_raw_file = fopen(fileName.c_str(), "rb");
      if (m_impl->gz_raw_file == nullptr)
      {
        if (error_msg)
        {
          error_msg.value().get() = std::string(strerror(errno));
        }
        return false;
      }

      // Use gzdopen to wrap the FILE* for decompression
      m_impl->gz_file = gzdopen(fileno(m_impl->gz_raw_file), "rb");
      if (m_impl->gz_file == nullptr)
      {
        fclose(m_impl->gz_raw_file);
        m_impl->gz_raw_file = nullptr;
        if (error_msg)
        {
          error_msg.value().get() = std::string(strerror(errno));
        }
        return false;
      }

      // Try to read uncompressed size from Gzip footer (last 4 bytes)
      if (m_file_size >= 8)
      {
        std::ifstream f(fileName, std::ios::binary);
        f.seekg(-4, std::ios::end);
        uint32_t size32;
        f.read(reinterpret_cast<char*>(&size32), 4);
        if (f.gcount() == 4)
        {
          m_impl->uncompressedSizeHint = size32;
        }
      }
      break;
    }

    case CompressionType::Zstd:
    {
      m_impl->rawFile = fopen(fileName.c_str(), "rb");
      if (m_impl->rawFile == nullptr)
      {
        if (error_msg)
        {
          error_msg.value().get() = std::string(strerror(errno));
        }
        return false;
      }

      m_impl->zstdStream = ZSTD_createDStream();
      if (m_impl->zstdStream == nullptr)
      {
        fclose(m_impl->rawFile);
        m_impl->rawFile = nullptr;
        if (error_msg)
        {
          error_msg.value().get() = "Failed to create Zstd decompression stream";
        }
        return false;
      }

      const size_t initResult = ZSTD_initDStream(m_impl->zstdStream);
      if (ZSTD_isError(initResult))
      {
        ZSTD_freeDStream(m_impl->zstdStream);
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

      // Allocate input buffer
      m_impl->zstdInBuffer.resize(ZSTD_DStreamInSize());
      m_impl->zstdInput = {nullptr, 0, 0};

      // Try to read uncompressed size from Zstd frame header
      std::array<uint8_t, 18> frameHeader;
      const size_t headerRead = fread(frameHeader.data(), 1, frameHeader.size(), m_impl->rawFile);
      if (headerRead >= 6)
      {
        const unsigned long long contentSize =
            ZSTD_getFrameContentSize(frameHeader.data(), headerRead);
        if (contentSize != ZSTD_CONTENTSIZE_UNKNOWN && contentSize != ZSTD_CONTENTSIZE_ERROR)
        {
          m_impl->uncompressedSizeHint = contentSize;
        }
      }
      fseek(m_impl->rawFile, 0, SEEK_SET);
      break;
    }

    case CompressionType::None:
    default:
    {
      m_impl->plainFile.open(fileName.c_str(), ios_base::binary | ios_base::in);
      if (!m_impl->plainFile.is_open())
      {
        if (error_msg)
        {
          error_msg.value().get() = std::string(strerror(errno));
        }
        return false;
      }
      m_impl->uncompressedSizeHint = m_file_size;
      break;
    }
  }

  m_impl->bytesReadUncompressed = 0;
  m_impl->gzCompressedBytesRead = 0;
  m_impl->zstdCompressedBytesRead = 0;
  return true;

  MRPT_END
}

void CCompressedInputStream::close()
{
  if (m_impl->gz_file)
  {
    gzclose(m_impl->gz_file);
    m_impl->gz_file = nullptr;
    // Note: gzclose also closes the underlying FILE*, so don't close gz_raw_file separately
    m_impl->gz_raw_file = nullptr;
  }

  if (m_impl->zstdStream)
  {
    ZSTD_freeDStream(m_impl->zstdStream);
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
  m_impl->compressionType = CompressionType::None;
  m_impl->bytesReadUncompressed = 0;
  m_impl->uncompressedSizeHint = 0;
  m_impl->gzCompressedBytesRead = 0;
  m_impl->zstdCompressedBytesRead = 0;
}

CCompressedInputStream::~CCompressedInputStream() { close(); }

size_t CCompressedInputStream::Read(void* Buffer, size_t Count)
{
  if (!fileOpenCorrectly())
  {
    THROW_EXCEPTION("File is not open.");
  }

  size_t bytesRead = 0;

  switch (m_impl->compressionType)
  {
    case CompressionType::Gzip:
    {
      // Track compressed position before read
      const uint64_t compressedPosBefore =
          m_impl->gz_raw_file ? static_cast<uint64_t>(ftell(m_impl->gz_raw_file)) : 0;

      const int result = gzread(m_impl->gz_file, Buffer, static_cast<unsigned int>(Count));
      bytesRead = (result < 0) ? 0 : static_cast<size_t>(result);

      // Track compressed position after read
      const uint64_t compressedPosAfter =
          m_impl->gz_raw_file ? static_cast<uint64_t>(ftell(m_impl->gz_raw_file)) : 0;

      // Update compressed bytes read based on actual file position change
      if (bytesRead > 0 && compressedPosAfter >= compressedPosBefore)
      {
        const uint64_t compressedBytesConsumed = compressedPosAfter - compressedPosBefore;
        m_impl->gzCompressedBytesRead += compressedBytesConsumed;
      }

      break;
    }

    case CompressionType::Zstd:
    {
      ZSTD_outBuffer output = {Buffer, Count, 0};
      uint64_t compressedBytesConsumed = 0;

      while (output.pos < output.size)
      {
        // Refill input buffer if needed
        if (m_impl->zstdInput.pos >= m_impl->zstdInput.size)
        {
          m_impl->zstdInput.size =
              fread(m_impl->zstdInBuffer.data(), 1, m_impl->zstdInBuffer.size(), m_impl->rawFile);
          m_impl->zstdInput.src = m_impl->zstdInBuffer.data();
          m_impl->zstdInput.pos = 0;

          if (m_impl->zstdInput.size == 0)
          {
            break;  // EOF
          }

          // Track how many compressed bytes we read from file
          compressedBytesConsumed += m_impl->zstdInput.size;
        }

        // const size_t inputPosBefore = m_impl->zstdInput.pos;

        const size_t result =
            ZSTD_decompressStream(m_impl->zstdStream, &output, &m_impl->zstdInput);

        // Track how many bytes from the input buffer were consumed
        // const size_t inputBytesConsumed = m_impl->zstdInput.pos - inputPosBefore;

        if (ZSTD_isError(result))
        {
          break;  // Error
        }

        if (result == 0)
        {
          break;  // End of frame
        }
      }

      bytesRead = output.pos;
      m_impl->zstdCompressedBytesRead += compressedBytesConsumed;
      break;
    }

    case CompressionType::None:
    default:
    {
      m_impl->plainFile.read(static_cast<char*>(Buffer), static_cast<std::streamsize>(Count));
      bytesRead = m_impl->plainFile.fail() ? 0 : Count;
      break;
    }
  }

  m_impl->bytesReadUncompressed += bytesRead;
  return bytesRead;
}

size_t CCompressedInputStream::Write(
    [[maybe_unused]] const void* Buffer, [[maybe_unused]] size_t Count)
{
  THROW_EXCEPTION("Trying to write to an input file stream.");
}

uint64_t CCompressedInputStream::getTotalBytesCount() const
{
  if (!fileOpenCorrectly())
  {
    THROW_EXCEPTION("File is not open.");
  }
  return m_file_size;
}

uint64_t CCompressedInputStream::getPosition() const
{
  if (!fileOpenCorrectly())
  {
    THROW_EXCEPTION("File is not open.");
  }

  switch (m_impl->compressionType)
  {
    case CompressionType::Gzip:
      if (m_impl->gz_raw_file)
      {
        return static_cast<uint64_t>(ftell(m_impl->gz_raw_file));
      }
      return 0;

    case CompressionType::Zstd:
      if (m_impl->rawFile)
      {
        return static_cast<uint64_t>(ftell(m_impl->rawFile));
      }
      return 0;

    case CompressionType::None:
    default:
    {
      auto& f = const_cast<std::ifstream&>(m_impl->plainFile);
      if (m_impl->plainFile.is_open())
      {
        return static_cast<uint64_t>(f.tellg());
      }
      return 0;
    }
  }
}

bool CCompressedInputStream::fileOpenCorrectly() const
{
  switch (m_impl->compressionType)
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

bool CCompressedInputStream::checkEOF()
{
  if (!fileOpenCorrectly())
  {
    return true;
  }

  switch (m_impl->compressionType)
  {
    case CompressionType::Gzip:
      return 0 != gzeof(m_impl->gz_file);

    case CompressionType::Zstd:
      if (m_impl->rawFile)
      {
        return feof(m_impl->rawFile) != 0;
      }
      return true;

    case CompressionType::None:
    default:
      return m_impl->plainFile.eof();
  }
}

uint64_t CCompressedInputStream::Seek(
    [[maybe_unused]] int64_t Offset, [[maybe_unused]] CStream::TSeekOrigin Origin)
{
  THROW_EXCEPTION("Method not available in this class.");
}

std::string CCompressedInputStream::getStreamDescription() const
{
  const char* typeStr = nullptr;
  switch (m_impl->compressionType)
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
      "mrpt::io::CCompressedInputStream for file '%s' (%s)", m_impl->filename.c_str(), typeStr);
}

std::string CCompressedInputStream::filePathAtUse() const { return m_impl->filename; }

CompressionType CCompressedInputStream::getCompressionType() const
{
  return m_impl->compressionType;
}

uint64_t CCompressedInputStream::getUncompressedSize() const
{
  // If we have a stored size hint, return it
  if (m_impl->uncompressedSizeHint > 0)
  {
    return m_impl->uncompressedSizeHint;
  }

  // For uncompressed files, return file size
  if (m_impl->compressionType == CompressionType::None)
  {
    return m_file_size;
  }

  // Otherwise, estimate based on compression ratio observed so far
  uint64_t compressedBytesRead = 0;

  switch (m_impl->compressionType)
  {
    case CompressionType::Gzip:
      compressedBytesRead = m_impl->gzCompressedBytesRead;
      break;

    case CompressionType::Zstd:
      compressedBytesRead = m_impl->zstdCompressedBytesRead;
      break;

    default:
      break;
  }

  // Need some data to have been read to make an estimate
  if (compressedBytesRead == 0 || m_impl->bytesReadUncompressed == 0)
  {
    return 0;
  }

  // Calculate compression ratio: uncompressed / compressed
  const double ratio =
      static_cast<double>(m_impl->bytesReadUncompressed) / static_cast<double>(compressedBytesRead);

  // Extrapolate to total file size
  const uint64_t estimatedTotal = static_cast<uint64_t>(static_cast<double>(m_file_size) * ratio);

  return estimatedTotal;
}

uint64_t CCompressedInputStream::getUncompressedPosition() const
{
  return m_impl->bytesReadUncompressed;
}

double CCompressedInputStream::getCompressionRatio() const
{
  if (m_impl->bytesReadUncompressed == 0)
  {
    return 0.0;
  }

  uint64_t compressedBytesRead = 0;

  switch (m_impl->compressionType)
  {
    case CompressionType::Gzip:
      compressedBytesRead = m_impl->gzCompressedBytesRead;
      break;

    case CompressionType::Zstd:
      compressedBytesRead = m_impl->zstdCompressedBytesRead;
      break;

    case CompressionType::None:
      // For uncompressed, ratio is 1.0
      return 1.0;

    default:
      break;
  }

  if (compressedBytesRead == 0)
  {
    return 0.0;
  }

  return static_cast<double>(m_impl->bytesReadUncompressed) /
         static_cast<double>(compressedBytesRead);
}
