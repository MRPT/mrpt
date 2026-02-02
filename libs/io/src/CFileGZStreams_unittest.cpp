/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/core/format.h>
#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/io/CCompressedOutputStream.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/system/filesystem.h>
#include <test_mrpt_common.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace
{
std::string to_hex_string(const uint8_t* buf, size_t len)
{
  std::ostringstream oss;
  for (size_t i = 0; i < len; ++i)
  {
    oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buf[i]);
  }
  return oss.str();
}

constexpr size_t TEST_DATA_LEN = 1000U;

void generate_test_data(std::vector<uint8_t>& tst_data)
{
  constexpr unsigned int RANDOM_SEED = 123U;
  mrpt::random::Generator_MT19937 mersenne_engine;
  mersenne_engine.seed(RANDOM_SEED);

  auto gen = [&mersenne_engine]()
  {
    // low entropy for better compression testing
    return static_cast<uint8_t>(mersenne_engine() % 4);
  };

  tst_data.resize(TEST_DATA_LEN);
  std::generate(std::begin(tst_data), std::end(tst_data), gen);
}

}  // namespace

// ============================================================================
// CFileGZStreams tests (legacy compatibility)
// ============================================================================

TEST(CFileGZStreams, readwriteTmpFileCompressed)
{
  std::vector<uint8_t> tst_data;
  generate_test_data(tst_data);

  // Test for all compress levels:
  for (int compress_level = 1; compress_level <= 9; compress_level++)
  {
    const std::string fil =
        mrpt::system::getTempFileName() + std::string("_") + std::to_string(compress_level) + ".gz";

    // Write
    {
      mrpt::io::CFileGZOutputStream fil_out;
      const bool open_ok = fil_out.open(fil, compress_level);
      EXPECT_TRUE(open_ok);

      const size_t wr_count = fil_out.Write(tst_data.data(), TEST_DATA_LEN);
      EXPECT_EQ(wr_count, TEST_DATA_LEN);
    }

    // Read
    {
      mrpt::io::CFileGZInputStream fil_in;
      const bool open_ok = fil_in.open(fil);
      EXPECT_TRUE(open_ok);

      std::vector<uint8_t> rd_buf(TEST_DATA_LEN + 5);
      const size_t rd_count = fil_in.Read(rd_buf.data(), TEST_DATA_LEN);
      EXPECT_EQ(rd_count, TEST_DATA_LEN);

      EXPECT_TRUE(std::equal(tst_data.begin(), tst_data.end(), rd_buf.begin()));
    }

    // Write for append
    {
      mrpt::io::CFileGZOutputStream fil_out;
      const bool open_ok =
          fil_out.open(fil, compress_level, std::nullopt, mrpt::io::OpenMode::APPEND);
      EXPECT_TRUE(open_ok);

      const size_t wr_count = fil_out.Write(tst_data.data(), TEST_DATA_LEN / 2);
      EXPECT_EQ(wr_count, TEST_DATA_LEN / 2);
    }

    // Read all
    {
      mrpt::io::CFileGZInputStream fil_in;
      const bool open_ok = fil_in.open(fil);
      EXPECT_TRUE(open_ok);

      std::vector<uint8_t> rd_buf(TEST_DATA_LEN + TEST_DATA_LEN / 2);
      const size_t rd_count = fil_in.Read(rd_buf.data(), TEST_DATA_LEN + TEST_DATA_LEN / 2);
      EXPECT_EQ(rd_count, TEST_DATA_LEN + TEST_DATA_LEN / 2);

      EXPECT_TRUE(std::equal(tst_data.begin(), tst_data.end(), rd_buf.begin()));
    }

    // Write and truncate
    {
      mrpt::io::CFileGZOutputStream fil_out;
      const bool open_ok =
          fil_out.open(fil, compress_level, std::nullopt, mrpt::io::OpenMode::TRUNCATE);
      EXPECT_TRUE(open_ok);

      const size_t wr_count = fil_out.Write(tst_data.data(), TEST_DATA_LEN / 4);
      EXPECT_EQ(wr_count, TEST_DATA_LEN / 4);
    }

    // Read all
    {
      mrpt::io::CFileGZInputStream fil_in;
      const bool open_ok = fil_in.open(fil);
      EXPECT_TRUE(open_ok);

      std::vector<uint8_t> rd_buf(TEST_DATA_LEN / 4);
      const size_t rd_count = fil_in.Read(rd_buf.data(), TEST_DATA_LEN / 4);
      EXPECT_EQ(rd_count, TEST_DATA_LEN / 4);

      EXPECT_TRUE(std::equal(rd_buf.begin(), rd_buf.end(), tst_data.begin()))
          << " compress_level: " << compress_level << "\n"
          << " test_data: " << to_hex_string(tst_data.data(), tst_data.size()) << "\n"
          << " read_data: " << to_hex_string(rd_buf.data(), rd_buf.size()) << "\n";

      // next I should find an EOF:
      const auto rdEof = fil_in.Read(rd_buf.data(), 1);
      EXPECT_EQ(rdEof, 0U);
    }

    // Cleanup
    mrpt::system::deleteFile(fil);
  }
}

TEST(CFileGZStreams, compareWithTestGZFiles)
{
  std::vector<uint8_t> tst_data;
  generate_test_data(tst_data);

  // Test for all compress levels:
  for (int compress_level = 1; compress_level <= 9; compress_level++)
  {
    const std::string fil =
        mrpt::format("%s/tests/gz-tests/%i.gz", mrpt::UNITTEST_BASEDIR().c_str(), compress_level);

    if (!mrpt::system::fileExists(fil))
    {
      GTEST_SKIP() << "Skipping test due to missing file: " << fil;
      continue;
    }

    mrpt::io::CFileGZInputStream fil_in;
    const bool open_ok = fil_in.open(fil);
    EXPECT_TRUE(open_ok);

    std::vector<uint8_t> rd_buf(TEST_DATA_LEN + 5);
    const size_t rd_count = fil_in.Read(rd_buf.data(), TEST_DATA_LEN);
    EXPECT_EQ(rd_count, TEST_DATA_LEN);

    EXPECT_TRUE(std::equal(tst_data.begin(), tst_data.end(), rd_buf.begin()))
        << " compress_level:" << compress_level;
  }
}

// ============================================================================
// CCompressedOutputStream tests
// ============================================================================

class CCompressedStreamsTest : public ::testing::Test
{
 protected:
  std::vector<uint8_t> test_data;

  void SetUp() override { generate_test_data(test_data); }
};

TEST_F(CCompressedStreamsTest, writeReadUncompressed)
{
  const std::string fil = mrpt::system::getTempFileName() + "_uncompressed.bin";

  // Write
  {
    const mrpt::io::CompressionOptions opts(mrpt::io::CompressionType::None);
    mrpt::io::CCompressedOutputStream fout(fil, mrpt::io::OpenMode::TRUNCATE, opts);

    EXPECT_TRUE(fout.is_open());
    EXPECT_EQ(fout.getCompressionType(), mrpt::io::CompressionType::None);

    const size_t written = fout.Write(test_data.data(), TEST_DATA_LEN);
    EXPECT_EQ(written, TEST_DATA_LEN);
  }

  // Read
  {
    mrpt::io::CCompressedInputStream fin(fil);

    EXPECT_TRUE(fin.is_open());
    EXPECT_EQ(fin.getCompressionType(), mrpt::io::CompressionType::None);

    std::vector<uint8_t> read_buf(TEST_DATA_LEN);
    const size_t read_count = fin.Read(read_buf.data(), TEST_DATA_LEN);
    EXPECT_EQ(read_count, TEST_DATA_LEN);

    EXPECT_EQ(test_data, read_buf);

    // read past EOF
    const size_t read_count2 = fin.Read(read_buf.data(), 1);
    (void)read_count2;
    EXPECT_TRUE(fin.checkEOF());
  }

  mrpt::system::deleteFile(fil);
}

TEST_F(CCompressedStreamsTest, writeReadGzip)
{
  const std::string fil = mrpt::system::getTempFileName() + "_gzip.gz";

  for (int level = 1; level <= 9; level++)
  {
    // Write
    {
      const mrpt::io::CompressionOptions opts(mrpt::io::CompressionType::Gzip, level);
      mrpt::io::CCompressedOutputStream fout(fil, mrpt::io::OpenMode::TRUNCATE, opts);

      EXPECT_TRUE(fout.is_open());
      EXPECT_EQ(fout.getCompressionType(), mrpt::io::CompressionType::Gzip);

      const size_t written = fout.Write(test_data.data(), TEST_DATA_LEN);
      EXPECT_EQ(written, TEST_DATA_LEN);
    }

    // Read
    {
      mrpt::io::CCompressedInputStream fin(fil);

      EXPECT_TRUE(fin.is_open());
      EXPECT_EQ(fin.getCompressionType(), mrpt::io::CompressionType::Gzip);

      std::vector<uint8_t> read_buf(TEST_DATA_LEN);
      const size_t read_count = fin.Read(read_buf.data(), TEST_DATA_LEN);
      EXPECT_EQ(read_count, TEST_DATA_LEN);

      EXPECT_EQ(test_data, read_buf) << "Failed at compression level " << level;
    }
  }

  mrpt::system::deleteFile(fil);
}

TEST_F(CCompressedStreamsTest, writeReadZstd)
{
  const std::string fil = mrpt::system::getTempFileName() + "_zstd.zst";

  std::array<int, 5> levels = {1, 3, 6, 10, 15};

  for (int level : levels)
  {
    // Write
    {
      const mrpt::io::CompressionOptions opts(mrpt::io::CompressionType::Zstd, level);
      mrpt::io::CCompressedOutputStream fout(fil, mrpt::io::OpenMode::TRUNCATE, opts);

      EXPECT_TRUE(fout.is_open());
      EXPECT_EQ(fout.getCompressionType(), mrpt::io::CompressionType::Zstd);

      const size_t written = fout.Write(test_data.data(), TEST_DATA_LEN);
      EXPECT_EQ(written, TEST_DATA_LEN);
    }

    // Read
    {
      mrpt::io::CCompressedInputStream fin(fil);

      EXPECT_TRUE(fin.is_open());
      EXPECT_EQ(fin.getCompressionType(), mrpt::io::CompressionType::Zstd);

      std::vector<uint8_t> read_buf(TEST_DATA_LEN);
      const size_t read_count = fin.Read(read_buf.data(), TEST_DATA_LEN);
      EXPECT_EQ(read_count, TEST_DATA_LEN);

      EXPECT_EQ(test_data, read_buf) << "Failed at compression level " << level;
    }
  }

  mrpt::system::deleteFile(fil);
}

TEST_F(CCompressedStreamsTest, autoDetection)
{
  const std::string file_none = mrpt::system::getTempFileName() + "_none.bin";
  const std::string file_gzip = mrpt::system::getTempFileName() + "_auto.gz";
  const std::string file_zstd = mrpt::system::getTempFileName() + "_auto.zst";

  // Create files with different compressions
  {
    const mrpt::io::CompressionOptions opts_none(mrpt::io::CompressionType::None);
    mrpt::io::CCompressedOutputStream f1(file_none, mrpt::io::OpenMode::TRUNCATE, opts_none);
    const auto nWrite = f1.Write(test_data.data(), TEST_DATA_LEN);
    EXPECT_EQ(nWrite, TEST_DATA_LEN);
  }
  {
    const mrpt::io::CompressionOptions opts_gzip(mrpt::io::CompressionType::Gzip, 6);
    mrpt::io::CCompressedOutputStream f2(file_gzip, mrpt::io::OpenMode::TRUNCATE, opts_gzip);
    const auto nWrite = f2.Write(test_data.data(), TEST_DATA_LEN);
    EXPECT_EQ(nWrite, TEST_DATA_LEN);
  }
  {
    const mrpt::io::CompressionOptions opts_zstd(mrpt::io::CompressionType::Zstd, 3);
    mrpt::io::CCompressedOutputStream f3(file_zstd, mrpt::io::OpenMode::TRUNCATE, opts_zstd);
    const auto nWrite = f3.Write(test_data.data(), TEST_DATA_LEN);
    EXPECT_EQ(nWrite, TEST_DATA_LEN);
  }

  // Verify auto-detection works
  {
    mrpt::io::CCompressedInputStream fin(file_none);
    EXPECT_EQ(fin.getCompressionType(), mrpt::io::CompressionType::None);
    std::vector<uint8_t> buf(TEST_DATA_LEN);
    const auto nRead = fin.Read(buf.data(), TEST_DATA_LEN);
    EXPECT_EQ(nRead, TEST_DATA_LEN);
    EXPECT_EQ(test_data, buf);
  }
  {
    mrpt::io::CCompressedInputStream fin(file_gzip);
    EXPECT_EQ(fin.getCompressionType(), mrpt::io::CompressionType::Gzip);
    std::vector<uint8_t> buf(TEST_DATA_LEN);
    const auto nRead = fin.Read(buf.data(), TEST_DATA_LEN);
    EXPECT_EQ(nRead, TEST_DATA_LEN);
    EXPECT_EQ(test_data, buf);
  }
  {
    mrpt::io::CCompressedInputStream fin(file_zstd);
    EXPECT_EQ(fin.getCompressionType(), mrpt::io::CompressionType::Zstd);
    std::vector<uint8_t> buf(TEST_DATA_LEN);
    const auto nRead = fin.Read(buf.data(), TEST_DATA_LEN);
    EXPECT_EQ(nRead, TEST_DATA_LEN);
    EXPECT_EQ(test_data, buf);
  }

  mrpt::system::deleteFile(file_none);
  mrpt::system::deleteFile(file_gzip);
  mrpt::system::deleteFile(file_zstd);
}

TEST_F(CCompressedStreamsTest, uncompressedSizeHint)
{
  const std::string file_gzip = mrpt::system::getTempFileName() + "_size.gz";
  const std::string file_zstd = mrpt::system::getTempFileName() + "_size.zst";

  // Write files
  {
    const mrpt::io::CompressionOptions opts_gzip(mrpt::io::CompressionType::Gzip, 6);
    mrpt::io::CCompressedOutputStream fout(file_gzip, mrpt::io::OpenMode::TRUNCATE, opts_gzip);
    const auto nWrite = fout.Write(test_data.data(), TEST_DATA_LEN);
    EXPECT_EQ(nWrite, TEST_DATA_LEN);
  }
  {
    const mrpt::io::CompressionOptions opts_zstd(mrpt::io::CompressionType::Zstd, 3);
    mrpt::io::CCompressedOutputStream fout(file_zstd, mrpt::io::OpenMode::TRUNCATE, opts_zstd);
    const auto nWrite = fout.Write(test_data.data(), TEST_DATA_LEN);
    EXPECT_EQ(nWrite, TEST_DATA_LEN);
  }

  // Check uncompressed size hints
  {
    const mrpt::io::CCompressedInputStream fin(file_gzip);
    const uint64_t size_hint = fin.getUncompressedSize();
    // Gzip stores size as last 4 bytes (modulo 2^32)
    EXPECT_EQ(size_hint, TEST_DATA_LEN % (1ULL << 32));
  }

  // Zstd can store exact size
  // JLBC: But not when streaming...
#if 0
  {
    const mrpt::io::CCompressedInputStream fin(file_zstd);
    const uint64_t size_hint = fin.getUncompressedSize();
    EXPECT_EQ(size_hint, TEST_DATA_LEN);
  }
#endif

  mrpt::system::deleteFile(file_gzip);
  mrpt::system::deleteFile(file_zstd);
}

TEST_F(CCompressedStreamsTest, uncompressedPosition)
{
  const std::string fil = mrpt::system::getTempFileName() + "_pos.gz";

  // Write
  {
    const mrpt::io::CompressionOptions opts(mrpt::io::CompressionType::Gzip, 6);
    mrpt::io::CCompressedOutputStream fout(fil, mrpt::io::OpenMode::TRUNCATE, opts);
    const auto nWrite = fout.Write(test_data.data(), TEST_DATA_LEN);
    EXPECT_EQ(nWrite, TEST_DATA_LEN);
  }

  // Read and check position tracking
  {
    mrpt::io::CCompressedInputStream fin(fil);

    EXPECT_EQ(fin.getUncompressedPosition(), 0U);

    std::vector<uint8_t> buf(TEST_DATA_LEN / 2);
    const auto nRead1 = fin.Read(buf.data(), TEST_DATA_LEN / 4);
    EXPECT_EQ(nRead1, TEST_DATA_LEN / 4);
    EXPECT_EQ(fin.getUncompressedPosition(), TEST_DATA_LEN / 4);

    const auto nRead2 = fin.Read(buf.data(), TEST_DATA_LEN / 4);
    EXPECT_EQ(nRead2, TEST_DATA_LEN / 4);
    EXPECT_EQ(fin.getUncompressedPosition(), TEST_DATA_LEN / 2);

    const auto nRead3 = fin.Read(buf.data(), TEST_DATA_LEN / 2);
    EXPECT_EQ(nRead3, TEST_DATA_LEN / 2);
    EXPECT_EQ(fin.getUncompressedPosition(), TEST_DATA_LEN);
  }

  mrpt::system::deleteFile(fil);
}

TEST_F(CCompressedStreamsTest, errorHandling)
{
  const std::string nonexistent = "/tmp/nonexistent_path_12345/file.gz";

  // Test opening non-existent file
  {
    std::string error_msg;
    mrpt::io::CCompressedInputStream fin;
    const bool opened = fin.open(nonexistent, error_msg);
    EXPECT_FALSE(opened);
    EXPECT_FALSE(error_msg.empty());
    EXPECT_FALSE(fin.is_open());
  }

  // Test writing to read-only stream
  {
    const std::string fil = mrpt::system::getTempFileName() + "_err.gz";
    const mrpt::io::CompressionOptions opts(mrpt::io::CompressionType::Gzip);
    mrpt::io::CCompressedOutputStream fout(fil, mrpt::io::OpenMode::TRUNCATE, opts);
    const auto nWrite = fout.Write(test_data.data(), TEST_DATA_LEN);
    EXPECT_EQ(nWrite, TEST_DATA_LEN);
    fout.close();

    mrpt::io::CCompressedInputStream fin(fil);
    EXPECT_THROW(
        {
          std::vector<uint8_t> buf(10);
          const auto nWrite2 = fin.Write(buf.data(), 10);
          (void)nWrite2;
        },
        std::exception);

    mrpt::system::deleteFile(fil);
  }
}

TEST_F(CCompressedStreamsTest, largeDataGzip)
{
  const std::string fil = mrpt::system::getTempFileName() + "_large.gz";
  constexpr size_t LARGE_SIZE = 100000;

  std::vector<uint8_t> large_data(LARGE_SIZE);
  mrpt::random::Generator_MT19937 rng;
  rng.seed(42);
  for (auto& byte : large_data)
  {
    byte = static_cast<uint8_t>(rng() % 256);
  }

  // Write
  {
    const mrpt::io::CompressionOptions opts(mrpt::io::CompressionType::Gzip, 6);
    mrpt::io::CCompressedOutputStream fout(fil, mrpt::io::OpenMode::TRUNCATE, opts);
    const size_t written = fout.Write(large_data.data(), LARGE_SIZE);
    EXPECT_EQ(written, LARGE_SIZE);
  }

  // Read
  {
    mrpt::io::CCompressedInputStream fin(fil);
    std::vector<uint8_t> read_buf(LARGE_SIZE);
    const size_t read_count = fin.Read(read_buf.data(), LARGE_SIZE);
    EXPECT_EQ(read_count, LARGE_SIZE);
    EXPECT_EQ(large_data, read_buf);
  }

  mrpt::system::deleteFile(fil);
}

TEST_F(CCompressedStreamsTest, largeDataZstd)
{
  const std::string fil = mrpt::system::getTempFileName() + "_large.zst";
  constexpr size_t LARGE_SIZE = 100000;

  std::vector<uint8_t> large_data(LARGE_SIZE);
  mrpt::random::Generator_MT19937 rng;
  rng.seed(42);
  for (auto& byte : large_data)
  {
    byte = static_cast<uint8_t>(rng() % 256);
  }

  // Write
  {
    const mrpt::io::CompressionOptions opts(mrpt::io::CompressionType::Zstd, 3);
    mrpt::io::CCompressedOutputStream fout(fil, mrpt::io::OpenMode::TRUNCATE, opts);
    const size_t written = fout.Write(large_data.data(), LARGE_SIZE);
    EXPECT_EQ(written, LARGE_SIZE);
  }

  // Read
  {
    mrpt::io::CCompressedInputStream fin(fil);
    std::vector<uint8_t> read_buf(LARGE_SIZE);
    const size_t read_count = fin.Read(read_buf.data(), LARGE_SIZE);
    EXPECT_EQ(read_count, LARGE_SIZE);
    EXPECT_EQ(large_data, read_buf);
  }

  mrpt::system::deleteFile(fil);
}
