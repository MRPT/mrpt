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
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/config/CConfigFilePrefixer.h>

using mrpt::config::CConfigFileMemory;
using mrpt::config::CConfigFilePrefixer;

TEST(CConfigFilePrefixer, UnboundThrows)
{
  CConfigFilePrefixer prefixer;
  EXPECT_THROW(prefixer.write("sec", "key", 1), std::exception);
  EXPECT_EQ(prefixer.getBoundConfigFileBase(), nullptr);
}

TEST(CConfigFilePrefixer, BindConstructorPrefixesKeysAndSections)
{
  CConfigFileMemory cfg;
  CConfigFilePrefixer prefixer(cfg, "sec_", "k_");

  EXPECT_EQ(prefixer.getSectionPrefix(), "sec_");
  EXPECT_EQ(prefixer.getKeyPrefix(), "k_");
  EXPECT_EQ(prefixer.getBoundConfigFileBase(), &cfg);

  prefixer.write("params", "value", 42);
  // The underlying object stores the prefixed section/key:
  EXPECT_EQ(cfg.read_int("sec_params", "k_value", 0), 42);
  // The prefixer transparently reads it back under the unprefixed names:
  EXPECT_EQ(prefixer.read_int("params", "value", 0), 42);
}

TEST(CConfigFilePrefixer, BindAndSetPrefixes)
{
  CConfigFileMemory cfg;
  CConfigFilePrefixer prefixer;
  prefixer.bind(cfg);
  prefixer.setPrefixes("s1_", "k1_");

  prefixer.write("group", "x", 7);
  EXPECT_EQ(cfg.read_int("s1_group", "k1_x", 0), 7);
}

TEST(CConfigFilePrefixer, GetAllSectionsAndKeysArePrefixed)
{
  CConfigFileMemory cfg;
  cfg.write("secA", "k1", 1);

  CConfigFilePrefixer prefixer(cfg, "p_", "q_");
  std::vector<std::string> sections;
  prefixer.getAllSections(sections);
  ASSERT_EQ(sections.size(), 1u);
  EXPECT_EQ(sections[0], "p_secA");

  std::vector<std::string> keys;
  prefixer.getAllKeys("secA", keys);
  ASSERT_EQ(keys.size(), 1u);
  EXPECT_EQ(keys[0], "q_k1");
}

TEST(CConfigFilePrefixer, Clear)
{
  CConfigFileMemory cfg;
  cfg.write("sec", "key", 1);

  CConfigFilePrefixer prefixer(cfg, "", "");
  EXPECT_TRUE(cfg.sectionExists("sec"));
  prefixer.clear();
  EXPECT_FALSE(cfg.sectionExists("sec"));
}
