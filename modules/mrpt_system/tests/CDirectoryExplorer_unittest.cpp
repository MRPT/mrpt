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
#include <mrpt/system/CDirectoryExplorer.h>
#include <mrpt/system/filesystem.h>

#include <fstream>

using namespace mrpt::system;

namespace
{
class TempDirFixture : public ::testing::Test
{
 protected:
  std::string dir;

  void SetUp() override
  {
    dir = mrpt::system::getTempFileName() + "_dir";
    ASSERT_TRUE(mrpt::system::createDirectory(dir));

    std::ofstream(dir + "/a.txt") << "hello";
    std::ofstream(dir + "/b.log") << "world";
    ASSERT_TRUE(mrpt::system::createDirectory(dir + "/subdir"));
  }

  void TearDown() override
  {
    mrpt::system::deleteFilesInDirectory(dir, /*deleteDirectoryAsWell=*/true);
  }
};
}  // namespace

TEST_F(TempDirFixture, ExploreFilesAndDirectories)
{
  auto files = CDirectoryExplorer::explore(dir, FILE_ATTRIB_ARCHIVE | FILE_ATTRIB_DIRECTORY);

  // a.txt, b.log, subdir
  EXPECT_EQ(files.size(), 3U);

  size_t numFiles = 0;
  size_t numDirs = 0;
  for (const auto& f : files)
  {
    if (f.isDir)
    {
      numDirs++;
      EXPECT_EQ(f.name, "subdir");
    }
    else
    {
      numFiles++;
    }
  }
  EXPECT_EQ(numFiles, 2U);
  EXPECT_EQ(numDirs, 1U);
}

TEST_F(TempDirFixture, ExploreFilesOnly)
{
  auto files = CDirectoryExplorer::explore(dir, FILE_ATTRIB_ARCHIVE);

  EXPECT_EQ(files.size(), 2U);
  for (const auto& f : files)
  {
    EXPECT_FALSE(f.isDir);
    EXPECT_GT(f.fileSize, 0U);
  }
}

TEST_F(TempDirFixture, SortByName)
{
  auto files = CDirectoryExplorer::explore(dir, FILE_ATTRIB_ARCHIVE);
  ASSERT_EQ(files.size(), 2U);

  CDirectoryExplorer::sortByName(files, true);
  EXPECT_TRUE(files.front().wholePath < files.back().wholePath);

  CDirectoryExplorer::sortByName(files, false);
  EXPECT_TRUE(files.front().wholePath > files.back().wholePath);
}

TEST_F(TempDirFixture, FilterByExtension)
{
  auto files = CDirectoryExplorer::explore(dir, FILE_ATTRIB_ARCHIVE);
  ASSERT_EQ(files.size(), 2U);

  CDirectoryExplorer::filterByExtension(files, "txt");
  ASSERT_EQ(files.size(), 1U);
  EXPECT_EQ(files.front().name, "a.txt");
}

TEST(CDirectoryExplorer, ExploreNonExistingDirectoryThrows)
{
  const std::string fname = mrpt::system::getTempFileName() + "_does_not_exist";
  EXPECT_THROW(
      (void)CDirectoryExplorer::explore(fname, FILE_ATTRIB_ARCHIVE | FILE_ATTRIB_DIRECTORY),
      std::exception);
}
