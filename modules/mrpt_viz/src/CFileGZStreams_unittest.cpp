/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <gtest/gtest.h>
#include <mrpt/core/format.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/system/filesystem.h>
#include <test_mrpt_common.h>

#include <algorithm>  // std::equal
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

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
}  // namespace

const size_t tst_data_len = 1000U;

// Generate random data:
void generate_test_data(std::vector<uint8_t>& tst_data)
{
  const unsigned int random_seed = 123U;
  mrpt::random::Generator_MT19937 mersenne_engine;
  mersenne_engine.seed(random_seed);

  // Was: std::uniform_int_distribution<unsigned short> dist{0, 3};
  // But that class seems not to be reproducible across compilers.
  auto gen = [&mersenne_engine]()
  {
    // low entropy
    return mersenne_engine() % 4;
  };

  tst_data.resize(tst_data_len);
  std::generate(std::begin(tst_data), std::end(tst_data), gen);
}

TEST(CFileGZStreams, readwriteTmpFileCompressed)
{
  std::vector<uint8_t> tst_data;
  generate_test_data(tst_data);

  // Test for all compress levels:
  for (int compress_level = 1; compress_level <= 9; compress_level++)
  {
    const std::string fil =
        mrpt::system::getTempFileName() + std::string("_") + std::to_string(compress_level);
    // Write:
    {
      mrpt::io::CFileGZOutputStream fil_out;
      const bool open_ok = fil_out.open(fil, compress_level);
      EXPECT_TRUE(open_ok);

      const size_t wr_count = fil_out.Write(&tst_data[0], tst_data_len);
      EXPECT_EQ(wr_count, tst_data_len);
    }
    // Read:
    {
      mrpt::io::CFileGZInputStream fil_in;
      const bool open_ok = fil_in.open(fil);
      EXPECT_TRUE(open_ok);

      uint8_t rd_buf[tst_data_len + 5];
      const size_t rd_count = fil_in.Read(rd_buf, tst_data_len);
      EXPECT_EQ(rd_count, tst_data_len);

      EXPECT_TRUE(std::equal(std::begin(tst_data), std::end(tst_data), std::begin(rd_buf)));
    }

    // Write for append =============================
    {
      mrpt::io::CFileGZOutputStream fil_out;
      const bool open_ok =
          fil_out.open(fil, compress_level, std::nullopt, mrpt::io::OpenMode::APPEND);
      EXPECT_TRUE(open_ok);

      const size_t wr_count = fil_out.Write(&tst_data[0], tst_data_len / 2);
      EXPECT_EQ(wr_count, tst_data_len / 2);
    }
    // Read all:
    {
      mrpt::io::CFileGZInputStream fil_in;
      const bool open_ok = fil_in.open(fil);
      EXPECT_TRUE(open_ok);

      uint8_t rd_buf[tst_data_len + tst_data_len / 2];
      const size_t rd_count = fil_in.Read(rd_buf, tst_data_len + tst_data_len / 2);
      EXPECT_EQ(rd_count, tst_data_len + tst_data_len / 2);

      EXPECT_TRUE(std::equal(std::begin(tst_data), std::end(tst_data), std::begin(rd_buf)));
    }
    // Write and truncate =============================
    {
      mrpt::io::CFileGZOutputStream fil_out;
      const bool open_ok =
          fil_out.open(fil, compress_level, std::nullopt, mrpt::io::OpenMode::TRUNCATE);
      EXPECT_TRUE(open_ok);

      const size_t wr_count = fil_out.Write(&tst_data[0], tst_data_len / 4);
      EXPECT_EQ(wr_count, tst_data_len / 4);
    }
    // Read all:
    {
      mrpt::io::CFileGZInputStream fil_in;
      const bool open_ok = fil_in.open(fil);
      EXPECT_TRUE(open_ok);

      uint8_t rd_buf[tst_data_len / 4];
      const size_t rd_count = fil_in.Read(rd_buf, tst_data_len / 4);
      EXPECT_EQ(rd_count, tst_data_len / 4);

      EXPECT_TRUE(std::equal(std::begin(rd_buf), std::end(rd_buf), std::begin(tst_data)))
          << " compress_level: " << compress_level << "\n"
          << " test_data: " << to_hex_string(tst_data.data(), tst_data.size()) << "\n"
          << " read_data: " << to_hex_string(rd_buf, sizeof(rd_buf)) << "\n";

      // next I should find an EOF:
      const auto rdEof = fil_in.Read(rd_buf, 1);
      EXPECT_EQ(rdEof, 0U);
    }
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
      GTEST_FAIL() << "ERROR: test due to missing file: " << fil << "\n";
      continue;
    }
    mrpt::io::CFileGZInputStream fil_in;
    const bool open_ok = fil_in.open(fil);
    EXPECT_TRUE(open_ok);

    uint8_t rd_buf[tst_data_len + 5];
    const size_t rd_count = fil_in.Read(rd_buf, tst_data_len);
    EXPECT_EQ(rd_count, tst_data_len);

    EXPECT_TRUE(std::equal(std::begin(tst_data), std::end(tst_data), std::begin(rd_buf)))
        << " compress_level:" << compress_level;
  }
}
