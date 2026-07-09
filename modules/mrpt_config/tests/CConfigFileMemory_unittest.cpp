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
#include <mrpt/config/CConfigFile.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/typemeta/TEnumType.h>

#include <cstdlib>
#include <fstream>

TEST(CConfigFileMemory, readwrite)
{
  const std::string a = "check", b = "test", c = "final //comments";
  mrpt::config::CConfigFileMemory first;
  first.write(a, b, c);
  EXPECT_STREQ("final", first.read_string(a, b, b).c_str());
}

TEST(CConfigFileMemory, Sections)
{
  mrpt::config::CConfigFileMemory second;
  second.write("one", "name", "val");
  second.write("two", "names", "value");

  const std::vector<std::string> sections = second.sections();
  EXPECT_EQ(2U, sections.size());
  if (sections.size() == 2)
  {  // avoid potential crash if fails
    EXPECT_STREQ("one", sections[0].c_str());
    EXPECT_STREQ("two", sections[1].c_str());
  }
}

TEST(CConfigFileMemory, Names)
{
  mrpt::config::CConfigFileMemory third;
  third.write("sec", "name", "val");
  third.write("sec", "names", "value");

  const std::vector<std::string> names = third.keys("sec");
  EXPECT_EQ(2U, names.size());
  if (names.size() == 2)
  {  // avoid potential crash if fails
    EXPECT_STREQ("name", names[0].c_str());
    EXPECT_STREQ("names", names[1].c_str());
  }

  EXPECT_TRUE(third.sectionExists("sec"));
  EXPECT_TRUE(third.sectionExists("SEC"));
  EXPECT_FALSE(third.sectionExists("SECXX"));

  EXPECT_TRUE(third.keyExists("sec", "name"));
  EXPECT_TRUE(third.keyExists("SEC", "NAME"));
  EXPECT_FALSE(third.keyExists("SEC", "NAMEX"));
}

TEST(CConfigFileMemory, setFromString)
{
  const std::string sampleCfgTxt =
      "# example config file from std::string\n"
      "[test]\n"
      "key_num = 4\n"
      "key_str = pepe\n";

  mrpt::config::CConfigFileMemory cfg;
  cfg.setContent(sampleCfgTxt);

  EXPECT_EQ(cfg.read_int("test", "key_num", 0), 4);
  EXPECT_EQ(cfg.read_string("test", "key_str", ""), std::string("pepe"));
}

// Being able of read
const std::string sampleCfgTxt =
    "[test]\n"
    "key_str = this is a \\\n"
    "long value that can be \\\n"
    "split into several lines \\\n"
    "but read as a single line. \n";
;
const std::string expectedStr = std::string(
    "this is a long value that can be split into several lines but read as a "
    "single line.");

TEST(CConfigFileMemory, readMultiLineStrings)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.setContent(sampleCfgTxt);

  const std::string readStr = cfg.read_string("test", "key_str", "");
  EXPECT_EQ(readStr, expectedStr);
}

TEST(CConfigFile, readMultiLineStrings)
{
  const std::string tmpFile = mrpt::system::getTempFileName();
  {
    std::ofstream f;
    f.open(tmpFile.c_str(), std::ofstream::out);
    EXPECT_TRUE(f.is_open());
    f << sampleCfgTxt;
    f.close();
  }

  mrpt::config::CConfigFile cfg(tmpFile);

  const std::string readStr = cfg.read_string("test", "key_str", "");
  EXPECT_EQ(readStr, expectedStr);
}

TEST(CConfigFileMemory, parseVariables)
{
#ifdef _MSC_VER
  _putenv_s("ENV_VAR_MULTIPLIER", "2");
#else
  ::setenv("ENV_VAR_MULTIPLIER", "2", 1);
#endif

  const std::string sampleCfgTxt2 =
      "@define MAXSPEED 10\n"
      "@define  MAXOMEGA  -30  \n"
      "[test]\n"
      "var1=5\n"
      "var2=${MAXSPEED}\n"
      "var3=${MAXOMEGA}\n"
      "var4=$eval{5*MAXSPEED+MAXOMEGA}\n"
      "var5 = $eval{ MAXSPEED - MAXOMEGA } \n"
      "var6=$env{ENV_VAR_MULTIPLIER}\n"
      "varstr1=MAXSPEED\n";
  ;
  mrpt::config::CConfigFileMemory cfg;
  cfg.setContent(sampleCfgTxt2);

  EXPECT_EQ(cfg.read_int("test", "var1", 0), 5);
  EXPECT_EQ(cfg.read_int("test", "var2", 0), 10);
  EXPECT_EQ(cfg.read_int("test", "var3", 0), -30);
  EXPECT_NEAR(cfg.read_double("test", "var4", .0), 20.0, 1e-6);
  EXPECT_NEAR(cfg.read_double("test", "var5", .0), 40.0, 1e-6);
  EXPECT_NEAR(cfg.read_double("test", "var6", .0), 2.0, 1e-6);
  EXPECT_EQ(cfg.read_string("test", "varstr1", ""), std::string("MAXSPEED"));
}

TEST(CConfigFileMemory, ConstructFromStringVector)
{
  const std::vector<std::string> lines = {"[sec]", "key = 5"};
  mrpt::config::CConfigFileMemory cfg(lines);
  EXPECT_EQ(cfg.read_int("sec", "key", 0), 5);
}

TEST(CConfigFileMemory, GetContent)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("sec", "key", 1);
  EXPECT_NE(cfg.getContent().find("key"), std::string::npos);
}

TEST(CConfigFileMemory, SectionsAndKeysReturnByValue)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("secA", "k1", 1);

  const auto secs = cfg.sections();
  ASSERT_EQ(secs.size(), 1u);
  EXPECT_EQ(secs[0], "secA");

  const auto ks = cfg.keys("secA");
  ASSERT_EQ(ks.size(), 1u);
  EXPECT_EQ(ks[0], "k1");
}

TEST(CConfigFileMemory, WriteFloat)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("sec", "key", 1.5f);
  EXPECT_NEAR(cfg.read_float("sec", "key", 0.0f), 1.5f, 1e-4f);
}

TEST(CConfigFileMemory, WriteWithCommentButNoValuePadding)
{
  // Exercises the "value_padding_width<1 but comment non-empty" branch of
  // CConfigFileBase::writeString(), where the value is left unpadded.
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("sec", "key", 3, /*name_padding_width=*/-1, /*value_padding_width=*/-1, "a comment");
  EXPECT_EQ(cfg.read_int("sec", "key", 0), 3);
  EXPECT_NE(cfg.getContent().find("a comment"), std::string::npos);
}

TEST(CConfigFileMemory, ReadUint64)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("sec", "key", 123456789ULL);
  EXPECT_EQ(cfg.read_uint64_t("sec", "key", 0), 123456789ULL);
}

TEST(CConfigFileMemory, ReadStringFirstWordAllWhitespaceThrows)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("sec", "key", "   ");
  EXPECT_THROW(
      cfg.read_string_first_word("sec", "key", "", /*failIfNotFound=*/true), std::exception);
}

TEST(CConfigFileMemory, GetContentAsYAML_UnscopedKey)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("", "unscoped", 9);
  const std::string yaml = cfg.getContentAsYAML();
  EXPECT_NE(yaml.find("unscoped"), std::string::npos);
}

namespace
{
/** Minimal stand-in for a matrix type, just enough to exercise
 * CConfigFileBase::read_matrix<>() without pulling in mrpt_math (which
 * itself depends on mrpt_config, so it can't be a test dependency here). */
struct FakeMatrix
{
  bool operator==(const FakeMatrix&) const { return true; }
  bool fromMatlabStringFormat(const std::string& s) { return s.find("bad") == std::string::npos; }
};
}  // namespace

TEST(CConfigFileMemory, ReadMatrixUsesDefaultWhenMissing)
{
  mrpt::config::CConfigFileMemory cfg;
  FakeMatrix out;
  FakeMatrix defaultVal;
  cfg.read_matrix("sec", "missing", out, defaultVal, /*failIfNotFound=*/false);
  EXPECT_EQ(out, defaultVal);
}

TEST(CConfigFileMemory, ReadMatrixParseErrorThrows)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("sec", "key", "this is bad data");
  FakeMatrix out;
  EXPECT_THROW(cfg.read_matrix("sec", "key", out), std::exception);
}

enum TestConfigEnum
{
  TestConfigEnumA = 0,
  TestConfigEnumB
};

MRPT_ENUM_TYPE_BEGIN(TestConfigEnum)
MRPT_FILL_ENUM(TestConfigEnumA);
MRPT_FILL_ENUM(TestConfigEnumB);
MRPT_ENUM_TYPE_END()

TEST(CConfigFileMemory, ReadEnumInvalidNameThrows)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("sec", "key", "NotARealEnumValue");
  EXPECT_THROW(cfg.read_enum<TestConfigEnum>("sec", "key", TestConfigEnumA), std::exception);
}

TEST(CConfigFileMemory, ReadEnumMissingKeyUsesDefault)
{
  mrpt::config::CConfigFileMemory cfg;
  EXPECT_EQ(cfg.read_enum<TestConfigEnum>("sec", "missing", TestConfigEnumB), TestConfigEnumB);
}

TEST(CConfigFileMemory, ReadEnumNumericValue)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("sec", "key", 1);
  EXPECT_EQ(cfg.read_enum<TestConfigEnum>("sec", "key", TestConfigEnumA), TestConfigEnumB);
}

// Regression test: write()'s column-padding (name_padding_width/
// value_padding_width, as used by e.g. MRPT_SAVE_CONFIG_VAR_COMMENT) must
// remain a purely cosmetic text-formatting detail and never change the
// stored key's identity. It previously padded the key itself with trailing
// spaces, so a same-instance read (no text round-trip) using the plain
// variable name silently failed to find the value.
TEST(CConfigFileMemory, writeWithPaddingKeepsKeyReadable)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write(
      "sec", "shortKey", 42, /*name_padding_width=*/41, /*value_padding_width=*/20, "a comment");

  EXPECT_TRUE(cfg.keyExists("sec", "shortKey"));
  EXPECT_EQ(cfg.read_int("sec", "shortKey", 0, /*failIfNotFound=*/true), 42);
}
