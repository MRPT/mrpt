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
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/system/filesystem.h>

#include <cstdint>
#include <string>
#include <vector>

using namespace mrpt::io;

namespace
{
const CompressionType kAllTypes[] = {
    CompressionType::None, CompressionType::Gzip, CompressionType::Zstd};

std::string tempName(const std::string& suffix) { return mrpt::system::getTempFileName() + suffix; }

std::vector<uint8_t> payload(size_t n)
{
  std::vector<uint8_t> v(n);
  for (size_t i = 0; i < n; i++)
  {
    v[i] = static_cast<uint8_t>((i * 7) & 0x7f);
  }
  return v;
}

void writeRaw(const std::string& fname, const std::string& bytes)
{
  CFileOutputStream f(fname);
  f.Write(bytes.data(), bytes.size());
}
}  // namespace

TEST(CCompressedStreams, RoundTripAndPositionsForEachFormat)
{
  const auto data = payload(200000);
  for (const auto type : kAllTypes)
  {
    const std::string fname = tempName("_cs_" + std::to_string(static_cast<int>(type)));
    {
      CCompressedOutputStream out;
      EXPECT_FALSE(out.fileOpenCorrectly());
      std::string err;
      ASSERT_TRUE(out.open(fname, CompressionOptions(type, 3), err)) << err;
      EXPECT_TRUE(out.is_open());
      EXPECT_EQ(out.getCompressionType(), type);
      EXPECT_EQ(out.filePathAtUse(), fname);

      const auto desc = out.getStreamDescription();
      EXPECT_NE(desc.find(fname), std::string::npos);
      EXPECT_NE(desc.find("level=3"), std::string::npos);

      uint64_t lastPos = 0;
      for (size_t off = 0; off < data.size(); off += 50000)
      {
        const size_t n = std::min<size_t>(50000, data.size() - off);
        ASSERT_EQ(out.Write(data.data() + off, n), n);
        const auto pos = out.getPosition();
        EXPECT_GE(pos, lastPos);
        lastPos = pos;
      }
      // These streams are not seekable:
      EXPECT_ANY_THROW(out.Seek(0, CStream::sFromBeginning));
      EXPECT_ANY_THROW(out.getTotalBytesCount());
      out.close();
      EXPECT_FALSE(out.fileOpenCorrectly());
      // ...and cannot be written once closed:
      EXPECT_ANY_THROW(out.Write(data.data(), 10));
    }

    {
      CCompressedInputStream in(fname);
      ASSERT_TRUE(in.fileOpenCorrectly());
      EXPECT_TRUE(in.is_open());
      EXPECT_EQ(in.getCompressionType(), type);
      EXPECT_EQ(in.filePathAtUse(), fname);
      EXPECT_GT(in.getTotalBytesCount(), 0u);
      EXPECT_NE(in.getStreamDescription().find(fname), std::string::npos);
      EXPECT_EQ(in.getPosition(), 0u);

      std::vector<uint8_t> back(data.size());
      size_t got = 0;
      while (got < back.size())
      {
        const size_t n = in.Read(back.data() + got, std::min<size_t>(30000, back.size() - got));
        ASSERT_GT(n, 0u) << "premature end for type " << static_cast<int>(type);
        got += n;
      }
      EXPECT_EQ(back, data);
      EXPECT_GT(in.getPosition(), 0u);
      EXPECT_GT(in.getUncompressedSize(), 0u);
      EXPECT_GT(in.getCompressionRatio(), 0.0);
      EXPECT_GT(in.getUncompressedPosition(), 0u);

      // Nothing else to read:
      uint8_t extra[4];
      EXPECT_EQ(in.Read(extra, 4), 0u);
      EXPECT_TRUE(in.checkEOF());
      EXPECT_ANY_THROW(in.Seek(0, CStream::sFromBeginning));

      in.close();
      EXPECT_FALSE(in.fileOpenCorrectly());
      EXPECT_ANY_THROW(in.getPosition());
      EXPECT_ANY_THROW(in.Read(extra, 4));
    }
    mrpt::system::deleteFile(fname);
  }
}

TEST(CCompressedStreams, OpenErrors)
{
  std::string err;

  // Missing input file:
  {
    CCompressedInputStream in;
    EXPECT_FALSE(in.open("/nonexistent-dir-for-test/none.bin", err));
    EXPECT_FALSE(err.empty());
    EXPECT_FALSE(in.fileOpenCorrectly());
    EXPECT_ANY_THROW(CCompressedInputStream("/nonexistent-dir-for-test/none.bin"));
  }
  // Output in a directory that does not exist:
  for (const auto type : kAllTypes)
  {
    CCompressedOutputStream out;
    err.clear();
    EXPECT_FALSE(out.open("/nonexistent-dir-for-test/out.bin", CompressionOptions(type), err));
    EXPECT_FALSE(err.empty()) << static_cast<int>(type);
    EXPECT_ANY_THROW(CCompressedOutputStream(
        "/nonexistent-dir-for-test/out.bin", OpenMode::TRUNCATE, CompressionOptions(type)));
  }
}

TEST(CCompressedStreams, CorruptFilesDoNotCrash)
{
  const std::string fname = tempName("_corrupt");
  const std::string truncatedMagic[] = {
      std::string("\x1f\x8b", 2),               // gzip magic, nothing else
      std::string("\x28\xb5\x2f\xfd", 4),       // zstd magic, nothing else
      std::string("\x28\xb5\x2f\xfd garbage"),  // zstd magic and junk
      std::string("\x1f\x8b\x08garbage garbage garbage"),
      std::string(),  // empty file
  };
  for (const auto& bytes : truncatedMagic)
  {
    writeRaw(fname, bytes);
    CCompressedInputStream in;
    std::string err;
    if (in.open(fname, err))
    {
      // Reading a corrupted stream may fail or return nothing, but not crash:
      uint8_t buf[64];
      try
      {
        (void)in.Read(buf, sizeof(buf));
      }
      catch (const std::exception&)
      {
      }
    }
  }
  mrpt::system::deleteFile(fname);
}

TEST(CCompressedStreams, AppendMode)
{
  const std::string fname = tempName("_append");
  const std::vector<uint8_t> a = payload(1000);
  const std::vector<uint8_t> b = payload(500);
  {
    CCompressedOutputStream out(
        fname, OpenMode::TRUNCATE, CompressionOptions(CompressionType::None));
    ASSERT_EQ(out.Write(a.data(), a.size()), a.size());
  }
  {
    CCompressedOutputStream out(fname, OpenMode::APPEND, CompressionOptions(CompressionType::None));
    ASSERT_EQ(out.Write(b.data(), b.size()), b.size());
  }
  CCompressedInputStream in(fname);
  std::vector<uint8_t> back(1500);
  EXPECT_EQ(in.Read(back.data(), back.size()), back.size());
  EXPECT_TRUE(std::equal(a.begin(), a.end(), back.begin()));
  EXPECT_TRUE(std::equal(b.begin(), b.end(), back.begin() + 1000));
  in.close();
  mrpt::system::deleteFile(fname);
}
