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
#include <mrpt/system/filesystem.h>

using mrpt::config::CConfigFile;

TEST(CConfigFile, DefaultConstructorThenSetFileName)
{
  const std::string tmpFile = mrpt::system::getTempFileName();
  {
    // Create an initial file with some content:
    CConfigFile cfg(tmpFile);
    cfg.write("sec", "key", 123);
    cfg.writeNow();
  }

  CConfigFile cfg2;
  cfg2.setFileName(tmpFile);
  EXPECT_EQ(cfg2.read_int("sec", "key", 0), 123);
}

TEST(CConfigFile, NonExistentFileToleratedOnConstruction)
{
  // Constructing from a path that doesn't exist yet must not throw: the file
  // is only actually created on the first writeNow().
  const std::string tmpFile = mrpt::system::getTempFileName() + "_does_not_exist";
  CConfigFile cfg(tmpFile);
  EXPECT_EQ(cfg.read_int("sec", "missing", 42), 42);
}

TEST(CConfigFile, WriteStringAndReadBack)
{
  const std::string tmpFile = mrpt::system::getTempFileName();
  CConfigFile cfg(tmpFile);
  cfg.write("sec", "key", "hello");
  EXPECT_EQ(cfg.read_string("sec", "key", ""), "hello");
}

TEST(CConfigFile, WriteNowPersistsToDisk)
{
  const std::string tmpFile = mrpt::system::getTempFileName();
  {
    CConfigFile cfg(tmpFile);
    cfg.write("sec", "key", 99);
    cfg.writeNow();
  }
  // Reopen from scratch and verify the value survived:
  CConfigFile cfg2(tmpFile);
  EXPECT_EQ(cfg2.read_int("sec", "key", 0), 99);
}

TEST(CConfigFile, DestructorAutoSaves)
{
  const std::string tmpFile = mrpt::system::getTempFileName();
  {
    CConfigFile cfg(tmpFile);
    cfg.write("sec", "key", 7);
    // No explicit writeNow(): the destructor must flush pending changes.
  }
  CConfigFile cfg2(tmpFile);
  EXPECT_EQ(cfg2.read_int("sec", "key", 0), 7);
}

TEST(CConfigFile, DiscardSavingChanges)
{
  const std::string tmpFile = mrpt::system::getTempFileName();
  {
    CConfigFile cfg(tmpFile);
    cfg.writeNow();  // create an empty, persisted file
  }
  {
    CConfigFile cfg(tmpFile);
    cfg.write("sec", "key", 5);
    cfg.discardSavingChanges();
    // writeNow() (called by the destructor) must now be a no-op.
  }
  CConfigFile cfg2(tmpFile);
  EXPECT_EQ(cfg2.read_int("sec", "key", -1), -1);
}

TEST(CConfigFile, ReadStringFailIfNotFoundThrows)
{
  const std::string tmpFile = mrpt::system::getTempFileName();
  CConfigFile cfg(tmpFile);
  EXPECT_THROW(cfg.read_string("sec", "missing", "", /*failIfNotFound=*/true), std::exception);
}

TEST(CConfigFile, GetAllSectionsAndKeys)
{
  const std::string tmpFile = mrpt::system::getTempFileName();
  CConfigFile cfg(tmpFile);
  cfg.write("secA", "k1", 1);
  cfg.write("secA", "k2", 2);
  cfg.write("secB", "k3", 3);

  std::vector<std::string> sections;
  cfg.getAllSections(sections);
  EXPECT_EQ(sections.size(), 2u);

  std::vector<std::string> keys;
  cfg.getAllKeys("secA", keys);
  EXPECT_EQ(keys.size(), 2u);
}

TEST(CConfigFile, WriteNowToUnwritablePathThrows)
{
  CConfigFile cfg("/nonexistent_dir_xyz_zzz/nested/file.ini");
  cfg.write("sec", "key", 1);
  EXPECT_THROW(cfg.writeNow(), std::exception);
  cfg.discardSavingChanges();  // avoid the destructor re-throwing/logging again
}

TEST(CConfigFile, DestructorSwallowsWriteNowException)
{
  // The destructor must catch (not propagate) any exception from the
  // implicit writeNow(), only logging it to stderr.
  EXPECT_NO_THROW({
    CConfigFile cfg("/nonexistent_dir_xyz_zzz/nested/file2.ini");
    cfg.write("sec", "key", 1);
  });
}

TEST(CConfigFile, Clear)
{
  const std::string tmpFile = mrpt::system::getTempFileName();
  CConfigFile cfg(tmpFile);
  cfg.write("sec", "key", 1);
  EXPECT_TRUE(cfg.sectionExists("sec"));
  cfg.clear();
  EXPECT_FALSE(cfg.sectionExists("sec"));
}
