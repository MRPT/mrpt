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

#include <gtest/gtest.h>
#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/io/CCompressedOutputStream.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/io/detect_compression.h>
#include <mrpt/io/vector_loadsave.h>
#include <mrpt/io/zip.h>
#include <mrpt/system/filesystem.h>

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

using mrpt::io::CCompressedInputStream;
using mrpt::io::CCompressedOutputStream;
using mrpt::io::CFileGZInputStream;
using mrpt::io::CFileGZOutputStream;
using mrpt::io::CompressionOptions;
using mrpt::io::CStream;

namespace
{
/** Low-entropy payload, so that compression actually has something to do. */
std::vector<uint8_t> samplePayload(size_t n = 20000)
{
  std::vector<uint8_t> v(n);
  for (size_t i = 0; i < n; i++)
  {
    v[i] = static_cast<uint8_t>(i & 0x3f);
  }
  return v;
}

std::string tempName(const std::string& suffix) { return mrpt::system::getTempFileName() + suffix; }

/** Writes raw bytes into a file, bypassing any compression. */
void writeRawFile(const std::string& fname, const std::vector<uint8_t>& bytes)
{
  mrpt::io::CFileOutputStream f(fname);
  if (!bytes.empty())
  {
    f.Write(bytes.data(), bytes.size());
  }
}
}  // namespace

// --------------------------- detect_compression ----------------------------

TEST(detect_compression, recognizesMagicBytes)
{
  const std::string fname = tempName("_detect");

  writeRawFile(fname, {0x1F, 0x8B, 0x08, 0x00});
  EXPECT_EQ(mrpt::io::detect_compression(fname), mrpt::io::CompressionType::Gzip);

  writeRawFile(fname, {0x28, 0xB5, 0x2F, 0xFD});
  EXPECT_EQ(mrpt::io::detect_compression(fname), mrpt::io::CompressionType::Zstd);

  // Zstd skippable frame:
  writeRawFile(fname, {0x2A, 0x4D, 0x18, 0x50});
  EXPECT_EQ(mrpt::io::detect_compression(fname), mrpt::io::CompressionType::Zstd);
  writeRawFile(fname, {0x2A, 0x4D, 0x18, 0x5F});
  EXPECT_EQ(mrpt::io::detect_compression(fname), mrpt::io::CompressionType::Zstd);
  // ...but 0x60 is outside the skippable-frame range:
  writeRawFile(fname, {0x2A, 0x4D, 0x18, 0x60});
  EXPECT_EQ(mrpt::io::detect_compression(fname), mrpt::io::CompressionType::None);

  writeRawFile(fname, {'p', 'l', 'a', 'i', 'n'});
  EXPECT_EQ(mrpt::io::detect_compression(fname), mrpt::io::CompressionType::None);

  std::remove(fname.c_str());
}

TEST(detect_compression, shortAndMissingFiles)
{
  const std::string fname = tempName("_detect_short");

  // Fewer than 2 bytes: nothing can be recognized.
  writeRawFile(fname, {0x1F});
  EXPECT_EQ(mrpt::io::detect_compression(fname), mrpt::io::CompressionType::None);

  // Two bytes are enough for gzip, but not for the 4-byte zstd magic:
  writeRawFile(fname, {0x28, 0xB5});
  EXPECT_EQ(mrpt::io::detect_compression(fname), mrpt::io::CompressionType::None);

  writeRawFile(fname, {});
  EXPECT_EQ(mrpt::io::detect_compression(fname), mrpt::io::CompressionType::None);

  std::remove(fname.c_str());

  EXPECT_EQ(mrpt::io::detect_compression(fname), mrpt::io::CompressionType::None);
}

// ------------------- CCompressedOutputStream / InputStream -----------------

class CompressedStreamsTest : public ::testing::TestWithParam<mrpt::io::CompressionType>
{
};

TEST_P(CompressedStreamsTest, roundTrip)
{
  const auto type = GetParam();
  const auto payload = samplePayload();
  const std::string fname = tempName("_compressed_roundtrip");

  {
    CCompressedOutputStream out(fname, mrpt::io::OpenMode::TRUNCATE, CompressionOptions(type, 1));
    ASSERT_TRUE(out.fileOpenCorrectly());
    EXPECT_TRUE(out.is_open());
    EXPECT_EQ(out.getCompressionType(), type);
    EXPECT_EQ(out.filePathAtUse(), fname);
    EXPECT_NE(out.getStreamDescription().find(fname), std::string::npos);

    EXPECT_EQ(out.Write(payload.data(), payload.size()), payload.size());
    // Note: with Zstd nothing has necessarily reached the file yet, since the
    // encoder buffers internally until close().
    EXPECT_NO_THROW((void)out.getPosition());

    // Neither seeking nor reading make sense on an output stream:
    EXPECT_ANY_THROW((void)out.Seek(0, CStream::sFromBeginning));
    std::vector<uint8_t> dummy(4);
    EXPECT_ANY_THROW((void)out.Read(dummy.data(), dummy.size()));
  }

  EXPECT_GT(mrpt::system::getFileSize(fname), 0U);

  {
    CCompressedInputStream in(fname);
    ASSERT_TRUE(in.fileOpenCorrectly());
    EXPECT_EQ(in.getCompressionType(), type);
    EXPECT_EQ(in.filePathAtUse(), fname);
    EXPECT_GT(in.getTotalBytesCount(), 0U);

    std::vector<uint8_t> readBack(payload.size());
    EXPECT_EQ(in.Read(readBack.data(), readBack.size()), payload.size());
    EXPECT_EQ(readBack, payload);

    // Nothing left:
    uint8_t extra = 0;
    EXPECT_EQ(in.Read(&extra, 1), 0U);
    EXPECT_TRUE(in.checkEOF());

    EXPECT_GT(in.getUncompressedSize(), 0U);
    EXPECT_GT(in.getCompressionRatio(), 0.0);

    // Writing into an input stream is rejected:
    EXPECT_ANY_THROW((void)in.Write(payload.data(), 1));
  }

  std::remove(fname.c_str());
}

INSTANTIATE_TEST_SUITE_P(
    AllCompressionTypes,
    CompressedStreamsTest,
    ::testing::Values(
        mrpt::io::CompressionType::None,
        mrpt::io::CompressionType::Gzip,
        mrpt::io::CompressionType::Zstd));

TEST(CCompressedOutputStream, unopenedStreamIsInert)
{
  CCompressedOutputStream out;
  EXPECT_FALSE(out.fileOpenCorrectly());
  EXPECT_TRUE(out.filePathAtUse().empty());
  EXPECT_NO_THROW(out.close());

  // Unlike the plain file streams, which report a zero position, the
  // compressed ones reject queries on a closed file:
  EXPECT_ANY_THROW((void)out.getPosition());
}

TEST(CCompressedOutputStream, openingAnUnwritablePathFails)
{
  const std::string badPath = "/this/directory/does/not/exist/file.zst";

  CCompressedOutputStream out;
  std::string err;
  EXPECT_FALSE(out.open(badPath, CompressionOptions(), err));
  EXPECT_FALSE(err.empty());

  EXPECT_ANY_THROW(CCompressedOutputStream{badPath});
}

TEST(CCompressedInputStream, unopenedStreamIsInert)
{
  CCompressedInputStream in;
  EXPECT_FALSE(in.fileOpenCorrectly());
  EXPECT_TRUE(in.filePathAtUse().empty());
  EXPECT_NO_THROW(in.close());

  EXPECT_ANY_THROW((void)in.getTotalBytesCount());
}

TEST(CCompressedInputStream, openingAMissingFileFails)
{
  const std::string missing = tempName("_does_not_exist");

  CCompressedInputStream in;
  std::string err;
  EXPECT_FALSE(in.open(missing, err));
  EXPECT_FALSE(err.empty());

  // Note: the constructor's documented "\exception" is not actually raised;
  // it just leaves the stream closed.
  CCompressedInputStream in2(missing);
  EXPECT_FALSE(in2.fileOpenCorrectly());
}

TEST(CCompressedInputStream, readsAPlainUncompressedFile)
{
  const std::string fname = tempName("_plain");
  const std::vector<uint8_t> payload = {'h', 'e', 'l', 'l', 'o'};
  writeRawFile(fname, payload);

  CCompressedInputStream in(fname);
  ASSERT_TRUE(in.fileOpenCorrectly());
  EXPECT_EQ(in.getCompressionType(), mrpt::io::CompressionType::None);
  EXPECT_EQ(in.getTotalBytesCount(), payload.size());

  std::vector<uint8_t> readBack(payload.size());
  EXPECT_EQ(in.Read(readBack.data(), readBack.size()), payload.size());
  EXPECT_EQ(readBack, payload);

  std::remove(fname.c_str());
}

TEST(CCompressedOutputStream, appendMode)
{
  const std::string fname = tempName("_append");

  {
    CCompressedOutputStream out(
        fname, mrpt::io::OpenMode::TRUNCATE,
        CompressionOptions(mrpt::io::CompressionType::None, 1));
    (void)out.Write("AAA", 3);
  }
  {
    CCompressedOutputStream out(
        fname, mrpt::io::OpenMode::APPEND, CompressionOptions(mrpt::io::CompressionType::None, 1));
    ASSERT_TRUE(out.fileOpenCorrectly());
    (void)out.Write("BBB", 3);
  }

  CCompressedInputStream in(fname);
  ASSERT_TRUE(in.fileOpenCorrectly());
  std::vector<uint8_t> readBack(6);
  EXPECT_EQ(in.Read(readBack.data(), readBack.size()), 6U);
  EXPECT_EQ(std::string(readBack.begin(), readBack.end()), "AAABBB");

  std::remove(fname.c_str());
}

// --------------------------- CFileGZ*Stream --------------------------------

TEST(CFileGZStreams, roundTrip)
{
  const auto payload = samplePayload(5000);
  const std::string fname = tempName("_gz_roundtrip.gz");

  {
    CFileGZOutputStream out(fname);
    ASSERT_TRUE(out.fileOpenCorrectly());
    EXPECT_TRUE(out.is_open());
    EXPECT_EQ(out.filePathAtUse(), fname);
    EXPECT_NE(out.getStreamDescription().find(fname), std::string::npos);

    EXPECT_EQ(out.Write(payload.data(), payload.size()), payload.size());
    EXPECT_EQ(out.getPosition(), payload.size());

    EXPECT_ANY_THROW((void)out.Seek(0));
    EXPECT_ANY_THROW((void)out.getTotalBytesCount());
    std::vector<uint8_t> dummy(4);
    EXPECT_ANY_THROW((void)out.Read(dummy.data(), dummy.size()));
  }

  {
    CFileGZInputStream in(fname);
    ASSERT_TRUE(in.fileOpenCorrectly());
    EXPECT_TRUE(in.is_open());
    EXPECT_EQ(in.filePathAtUse(), fname);
    EXPECT_NE(in.getStreamDescription().find(fname), std::string::npos);
    EXPECT_GT(in.getTotalBytesCount(), 0U);

    std::vector<uint8_t> readBack(payload.size());
    EXPECT_EQ(in.Read(readBack.data(), readBack.size()), payload.size());
    EXPECT_EQ(readBack, payload);

    // gzeof() only reports EOF once a read has actually hit it:
    uint8_t extra = 0;
    EXPECT_EQ(in.Read(&extra, 1), 0U);
    EXPECT_TRUE(in.checkEOF());

    EXPECT_ANY_THROW((void)in.Seek(0));
    EXPECT_ANY_THROW((void)in.Write(payload.data(), 1));
  }

  std::remove(fname.c_str());
}

TEST(CFileGZStreams, unopenedStreamsAreInert)
{
  {
    CFileGZInputStream in;
    EXPECT_FALSE(in.fileOpenCorrectly());
    EXPECT_TRUE(in.checkEOF());
    EXPECT_TRUE(in.filePathAtUse().empty());
    EXPECT_NO_THROW(in.close());
  }
  {
    CFileGZOutputStream out;
    EXPECT_FALSE(out.fileOpenCorrectly());
    EXPECT_TRUE(out.filePathAtUse().empty());
    EXPECT_NO_THROW(out.close());
  }
}

TEST(CFileGZStreams, openFailures)
{
  const std::string missing = tempName("_does_not_exist.gz");

  CFileGZInputStream in;
  std::string err;
  EXPECT_FALSE(in.open(missing, err));
  EXPECT_FALSE(err.empty());

  // Note: the constructor's documented "\exception" is not actually raised;
  // it just leaves the stream closed.
  CFileGZInputStream in2(missing);
  EXPECT_FALSE(in2.fileOpenCorrectly());

  CFileGZOutputStream out;
  EXPECT_FALSE(out.open("/this/directory/does/not/exist/file.gz"));
  // Unlike its input counterpart, this constructor does honor its documented
  // "\exception":
  EXPECT_ANY_THROW(CFileGZOutputStream{"/this/directory/does/not/exist/file.gz"});
}

// ------------------------------- zip:: -------------------------------------

TEST(zip, compressAndDecompressVectors)
{
  const auto payload = samplePayload();
  const std::vector<unsigned char> in(payload.begin(), payload.end());

  std::vector<unsigned char> compressed;
  mrpt::io::zip::compress(in, compressed);
  EXPECT_FALSE(compressed.empty());
  EXPECT_LT(compressed.size(), in.size());

  std::vector<unsigned char> out;
  mrpt::io::zip::decompress(compressed.data(), compressed.size(), out, in.size());
  EXPECT_EQ(out, in);
}

TEST(zip, compressAndDecompressRawBuffers)
{
  auto payload = samplePayload(4096);

  std::vector<unsigned char> compressed;
  mrpt::io::zip::compress(payload.data(), payload.size(), compressed);
  EXPECT_FALSE(compressed.empty());

  std::vector<unsigned char> out(payload.size());
  const size_t n =
      mrpt::io::zip::decompress(compressed.data(), compressed.size(), out.data(), out.size());
  EXPECT_EQ(n, payload.size());
  EXPECT_TRUE(std::equal(payload.begin(), payload.end(), out.begin()));
}

TEST(zip, compressIntoAStreamAndDecompressFromIt)
{
  auto payload = samplePayload(4096);

  {
    // The raw-pointer overload:
    mrpt::io::CMemoryStream buf;
    mrpt::io::zip::compress(payload.data(), payload.size(), buf);
    EXPECT_GT(buf.getTotalBytesCount(), 0U);

    buf.Seek(0);
    std::vector<unsigned char> out(payload.size());
    const size_t n = mrpt::io::zip::decompress(
        buf, static_cast<size_t>(buf.getTotalBytesCount()), out.data(), out.size());
    EXPECT_EQ(n, payload.size());
    EXPECT_TRUE(std::equal(payload.begin(), payload.end(), out.begin()));
  }
  {
    // ...and the std::vector one:
    const std::vector<unsigned char> in(payload.begin(), payload.end());
    mrpt::io::CMemoryStream buf;
    mrpt::io::zip::compress(in, buf);
    EXPECT_GT(buf.getTotalBytesCount(), 0U);
  }
}

TEST(zip, gzFileRoundTrip)
{
  const auto payload = samplePayload(8000);
  const std::string fname = tempName("_zip_gzfile.gz");

  EXPECT_TRUE(mrpt::io::zip::compress_gz_file(fname, payload));

  std::vector<uint8_t> out;
  EXPECT_TRUE(mrpt::io::zip::decompress_gz_file(fname, out));
  EXPECT_EQ(out, payload);

  std::remove(fname.c_str());
}

TEST(zip, gzFileFailures)
{
  std::vector<uint8_t> out;
  EXPECT_FALSE(mrpt::io::zip::decompress_gz_file(tempName("_does_not_exist.gz"), out));

  EXPECT_FALSE(
      mrpt::io::zip::compress_gz_file("/this/directory/does/not/exist/f.gz", samplePayload(10)));
}

TEST(zip, gzDataBlockWithEmptyInput)
{
  std::vector<uint8_t> out;
  EXPECT_TRUE(mrpt::io::zip::compress_gz_data_block({}, out));
  EXPECT_TRUE(out.empty());

  std::vector<uint8_t> out2 = {1, 2, 3};
  EXPECT_TRUE(mrpt::io::zip::decompress_gz_data_block({}, out2));
  EXPECT_TRUE(out2.empty());
}

TEST(zip, compressAnEmptyGzFile)
{
  const std::string fname = tempName("_zip_empty.gz");

  EXPECT_TRUE(mrpt::io::zip::compress_gz_file(fname, {}));

  std::vector<uint8_t> out;
  EXPECT_TRUE(mrpt::io::zip::decompress_gz_file(fname, out));
  EXPECT_TRUE(out.empty());

  std::remove(fname.c_str());
}
