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

/** Round-trips and error paths for the vector<->file helpers: the binary and
 *  text loaders (both the optional-returning and the deprecated
 *  out-parameter forms), every vectorToTextFile() overload, and the numeric
 *  text parser.
 */

#include <gtest/gtest.h>
#include <mrpt/io/vector_loadsave.h>
#include <mrpt/system/filesystem.h>

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

using namespace mrpt::io;

namespace
{
/** A temp file path that does not exist yet, removed by the destructor. */
class TempFile
{
 public:
  TempFile() : m_path(mrpt::system::getTempFileName()) {}
  ~TempFile() { std::remove(m_path.c_str()); }
  const std::string& path() const { return m_path; }

 private:
  std::string m_path;
};

const char* const kMissing = "/no/such/dir/definitely_missing_file.bin";
}  // namespace

// ---------------------------------------------------------------------------
//  Binary files
// ---------------------------------------------------------------------------
TEST(vector_loadsave, binary_roundtrip)
{
  TempFile f;
  const std::vector<uint8_t> data{0, 1, 2, 250, 255, 42};

  EXPECT_TRUE(vectorToBinaryFile(data, f.path()));

  const auto loaded = loadBinaryFile(f.path());
  ASSERT_TRUE(loaded.has_value());
  EXPECT_EQ(*loaded, data);

  // Deprecated out-parameter form agrees:
  std::vector<uint8_t> viaOutParam;
  EXPECT_TRUE(loadBinaryFile(viaOutParam, f.path()));
  EXPECT_EQ(viaOutParam, data);
}

TEST(vector_loadsave, empty_binary_roundtrip)
{
  TempFile f;
  const std::vector<uint8_t> empty;
  EXPECT_TRUE(vectorToBinaryFile(empty, f.path()));

  const auto loaded = loadBinaryFile(f.path());
  ASSERT_TRUE(loaded.has_value());
  EXPECT_TRUE(loaded->empty());
}

TEST(vector_loadsave, binary_load_of_a_missing_file_fails)
{
  EXPECT_FALSE(loadBinaryFile(kMissing).has_value());

  std::vector<uint8_t> viaOutParam{1, 2, 3};
  EXPECT_FALSE(loadBinaryFile(viaOutParam, kMissing));
}

TEST(vector_loadsave, binary_save_to_an_unwritable_path_fails)
{
  const std::vector<uint8_t> data{1, 2, 3};
  EXPECT_FALSE(vectorToBinaryFile(data, kMissing));
}

// ---------------------------------------------------------------------------
//  Text files
// ---------------------------------------------------------------------------
TEST(vector_loadsave, text_lines_roundtrip)
{
  TempFile f;
  {
    std::ofstream o(f.path());
    o << "first\nsecond\nthird\n";
  }

  const auto lines = loadTextFile(f.path());
  ASSERT_TRUE(lines.has_value());
  ASSERT_EQ(lines->size(), 3U);
  EXPECT_EQ((*lines)[0], "first");
  EXPECT_EQ((*lines)[2], "third");

  std::vector<std::string> viaOutParam;
  EXPECT_TRUE(loadTextFile(viaOutParam, f.path()));
  EXPECT_EQ(viaOutParam, *lines);
}

TEST(vector_loadsave, text_load_of_a_missing_file_fails)
{
  EXPECT_FALSE(loadTextFile(kMissing).has_value());

  std::vector<std::string> viaOutParam{"stale"};
  EXPECT_FALSE(loadTextFile(viaOutParam, kMissing));
}

TEST(vector_loadsave, file_get_contents_returns_the_whole_file)
{
  TempFile f;
  const std::string content = "line one\nline two\n";
  {
    // Binary mode on purpose: file_get_contents() reads the raw bytes, so a
    // text-mode write would translate "\n" to "\r\n" on Windows and the
    // comparison below would be off by one byte per line.
    std::ofstream o(f.path(), std::ios::binary);
    o << content;
  }
  EXPECT_EQ(file_get_contents(f.path()), content);
}

TEST(vector_loadsave, file_get_contents_is_byte_exact)
{
  TempFile f;
  // Bytes that a text-mode round-trip would mangle or truncate:
  const std::string content(
      "a\r\nb\0c\x1a"
      "d",
      8);
  {
    std::ofstream o(f.path(), std::ios::binary);
    o.write(content.data(), static_cast<std::streamsize>(content.size()));
  }

  const std::string back = file_get_contents(f.path());
  EXPECT_EQ(back.size(), content.size());
  EXPECT_EQ(back, content);
}

TEST(vector_loadsave, file_get_contents_throws_for_a_missing_file)
{
  EXPECT_ANY_THROW(file_get_contents(kMissing));
}

// ---------------------------------------------------------------------------
//  vectorToTextFile() overloads
// ---------------------------------------------------------------------------
TEST(vector_loadsave, numeric_text_roundtrip_by_columns_and_rows)
{
  const std::vector<double> v{1.0, 2.5, -3.25};

  {
    TempFile f;
    ASSERT_TRUE(vectorToTextFile(v, f.path()));  // default: one per column

    std::vector<double> back;
    EXPECT_TRUE(vectorNumericFromTextFile(back, f.path()));
    ASSERT_EQ(back.size(), v.size());
    for (size_t i = 0; i < v.size(); i++)
    {
      EXPECT_NEAR(back[i], v[i], 1e-9);
    }
  }
  {
    TempFile f;
    VectorTextFileOptions opts;
    opts.byRows = true;
    ASSERT_TRUE(vectorToTextFile(v, f.path(), opts));

    std::vector<double> back;
    EXPECT_TRUE(vectorNumericFromTextFile(back, f.path(), true /*byRows*/));
    ASSERT_EQ(back.size(), v.size());
    EXPECT_NEAR(back[1], 2.5, 1e-9);
  }
}

TEST(vector_loadsave, append_mode_grows_the_file)
{
  TempFile f;
  const std::vector<int> a{1, 2};
  const std::vector<int> b{3, 4};

  ASSERT_TRUE(vectorToTextFile(a, f.path()));
  const auto afterFirst = loadTextFile(f.path());
  ASSERT_TRUE(afterFirst.has_value());

  VectorTextFileOptions opts;
  opts.append = true;
  ASSERT_TRUE(vectorToTextFile(b, f.path(), opts));

  const auto afterSecond = loadTextFile(f.path());
  ASSERT_TRUE(afterSecond.has_value());
  EXPECT_GT(afterSecond->size(), afterFirst->size());
}

TEST(vector_loadsave, every_numeric_overload_writes_a_file)
{
  const std::vector<float> vf{1.5f, 2.5f};
  const std::vector<double> vd{1.5, 2.5};
  const std::vector<int> vi{1, 2};
  const std::vector<size_t> vs{1U, 2U};

  {
    TempFile f;
    EXPECT_TRUE(vectorToTextFile(vf, f.path()));
    EXPECT_TRUE(mrpt::system::fileExists(f.path()));
  }
  {
    TempFile f;
    EXPECT_TRUE(vectorToTextFile(vd, f.path()));
    EXPECT_TRUE(mrpt::system::fileExists(f.path()));
  }
  {
    TempFile f;
    EXPECT_TRUE(vectorToTextFile(vi, f.path()));
    EXPECT_TRUE(mrpt::system::fileExists(f.path()));
  }
  {
    TempFile f;
    EXPECT_TRUE(vectorToTextFile(vs, f.path()));
    EXPECT_TRUE(mrpt::system::fileExists(f.path()));
  }
}

TEST(vector_loadsave, writing_to_an_unwritable_path_fails_for_every_overload)
{
  EXPECT_FALSE(vectorToTextFile(std::vector<float>{1.f}, kMissing));
  EXPECT_FALSE(vectorToTextFile(std::vector<double>{1.0}, kMissing));
  EXPECT_FALSE(vectorToTextFile(std::vector<int>{1}, kMissing));
  EXPECT_FALSE(vectorToTextFile(std::vector<size_t>{1U}, kMissing));
}

TEST(vector_loadsave, empty_vector_writes_an_empty_file)
{
  TempFile f;
  EXPECT_TRUE(vectorToTextFile(std::vector<double>{}, f.path()));

  std::vector<double> back{9.0};
  EXPECT_TRUE(vectorNumericFromTextFile(back, f.path()));
  EXPECT_TRUE(back.empty());
}

// ---------------------------------------------------------------------------
//  vectorNumericFromTextFile()
// ---------------------------------------------------------------------------
TEST(vector_loadsave, numeric_parse_of_a_missing_file_fails)
{
  std::vector<double> v{1.0};
  EXPECT_FALSE(vectorNumericFromTextFile(v, kMissing));
}

TEST(vector_loadsave, numeric_parse_skips_blank_lines)
{
  TempFile f;
  {
    std::ofstream o(f.path());
    o << "1.0\n\n2.0\n\n\n3.0\n";
  }
  std::vector<double> v;
  EXPECT_TRUE(vectorNumericFromTextFile(v, f.path()));
  ASSERT_EQ(v.size(), 3U);
  EXPECT_NEAR(v[2], 3.0, 1e-9);
}
