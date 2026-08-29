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
#include <mrpt/system/CFileSystemWatcher.h>
#include <mrpt/system/config.h>  // MRPT_HAS_INOTIFY
#include <mrpt/system/filesystem.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <string>
#include <thread>

using mrpt::system::CFileSystemWatcher;
using namespace std::chrono_literals;

namespace
{
/** A temporary directory that removes itself at the end of the test. */
class TempDir
{
 public:
  TempDir() :
      m_path(mrpt::system::getTempFileName() + "_watched"),
      m_ok(mrpt::system::createDirectory(m_path))
  {
  }
  ~TempDir()
  {
    if (m_ok)
    {
      mrpt::system::deleteFilesInDirectory(m_path, true /*deleteDirectoryAsWell*/);
    }
  }

  TempDir(const TempDir&) = delete;
  TempDir& operator=(const TempDir&) = delete;
  TempDir(TempDir&&) = delete;
  TempDir& operator=(TempDir&&) = delete;

  [[nodiscard]] bool ok() const { return m_ok; }
  [[nodiscard]] const std::string& path() const { return m_path; }

 private:
  std::string m_path;
  bool m_ok;
};

/** Polls the watcher for up to ~2 s, collecting every change reported. */
CFileSystemWatcher::TFileSystemChangeList collectChanges(CFileSystemWatcher& w)
{
  CFileSystemWatcher::TFileSystemChangeList all;
  for (int i = 0; i < 40; i++)
  {
    CFileSystemWatcher::TFileSystemChangeList lst;
    w.getChanges(lst);
    all.insert(all.end(), lst.begin(), lst.end());
    if (!all.empty())
    {
      // Give the remaining events a chance to arrive too:
      std::this_thread::sleep_for(50ms);
      w.getChanges(lst);
      all.insert(all.end(), lst.begin(), lst.end());
      break;
    }
    std::this_thread::sleep_for(50ms);
  }
  return all;
}

bool anyChangeMatches(
    const CFileSystemWatcher::TFileSystemChangeList& lst,
    const std::string& fileName,
    bool CFileSystemWatcher::TFileSystemChange::*event)
{
  return std::any_of(
      lst.begin(), lst.end(),
      [&](const CFileSystemWatcher::TFileSystemChange& c)
      { return c.*event && c.path.find(fileName) != std::string::npos; });
}
}  // namespace

TEST(CFileSystemWatcher, watchingAMissingDirectoryThrows)
{
#if !MRPT_HAS_INOTIFY && !defined(_WIN32)
  // Without notification support the constructor does nothing at all, so it
  // has no chance to notice that the directory is missing:
  GTEST_SKIP() << "This platform has no file system notification support.";
#else
  EXPECT_ANY_THROW(CFileSystemWatcher(mrpt::system::getTempFileName() + "_does_not_exist"));
#endif
}

TEST(CFileSystemWatcher, watchingAnEmptyPathThrows) { EXPECT_ANY_THROW(CFileSystemWatcher("")); }

TEST(CFileSystemWatcher, noChangesOnAQuietDirectory)
{
  TempDir dir;
  ASSERT_TRUE(dir.ok());

  CFileSystemWatcher w(dir.path());

  CFileSystemWatcher::TFileSystemChangeList lst;
  w.getChanges(lst);
  EXPECT_TRUE(lst.empty());
}

TEST(CFileSystemWatcher, detectsFileCreation)
{
#if !MRPT_HAS_INOTIFY
  GTEST_SKIP() << "This platform has no file system notification support.";
#else
  TempDir dir;
  ASSERT_TRUE(dir.ok());

  CFileSystemWatcher w(dir.path());

  const std::string fileName = "created.txt";
  {
    std::ofstream f(dir.path() + "/" + fileName);
    ASSERT_TRUE(f.is_open());
    f << "hello";
  }

  const auto changes = collectChanges(w);
  ASSERT_FALSE(changes.empty());
  EXPECT_TRUE(
      anyChangeMatches(changes, fileName, &CFileSystemWatcher::TFileSystemChange::eventCreated));
  EXPECT_TRUE(
      anyChangeMatches(changes, fileName, &CFileSystemWatcher::TFileSystemChange::eventCloseWrite));
#endif
}

TEST(CFileSystemWatcher, detectsFileDeletion)
{
#if !MRPT_HAS_INOTIFY
  GTEST_SKIP() << "This platform has no file system notification support.";
#else
  TempDir dir;
  ASSERT_TRUE(dir.ok());

  const std::string fileName = "to_delete.txt";
  const std::string fullPath = dir.path() + "/" + fileName;
  {
    std::ofstream f(fullPath);
    ASSERT_TRUE(f.is_open());
  }

  // Only start watching once the file exists, so that the deletion is the
  // first event seen:
  CFileSystemWatcher w(dir.path());
  ASSERT_TRUE(mrpt::system::deleteFile(fullPath));

  const auto changes = collectChanges(w);
  ASSERT_FALSE(changes.empty());
  EXPECT_TRUE(
      anyChangeMatches(changes, fileName, &CFileSystemWatcher::TFileSystemChange::eventDeleted));
#endif
}

TEST(CFileSystemWatcher, detectsSubdirectoryCreation)
{
#if !MRPT_HAS_INOTIFY
  GTEST_SKIP() << "This platform has no file system notification support.";
#else
  TempDir dir;
  ASSERT_TRUE(dir.ok());

  CFileSystemWatcher w(dir.path());

  const std::string subDir = "subdir";
  ASSERT_TRUE(mrpt::system::createDirectory(dir.path() + "/" + subDir));

  const auto changes = collectChanges(w);
  ASSERT_FALSE(changes.empty());

  // Directories are flagged as such:
  EXPECT_TRUE(std::any_of(
      changes.begin(), changes.end(),
      [&](const CFileSystemWatcher::TFileSystemChange& c)
      { return c.isDir && c.path.find(subDir) != std::string::npos; }));
#endif
}
