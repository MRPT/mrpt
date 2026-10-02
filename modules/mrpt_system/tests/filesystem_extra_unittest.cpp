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
#include <mrpt/system/filesystem.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>

using namespace mrpt::system;

namespace
{
/** A fresh empty temporary directory, removed when destroyed. */
class TempDir
{
 public:
  TempDir()
  {
    m_path = getTempFileName() + "_dir";
    createDirectory(m_path);
  }
  ~TempDir() { deleteFilesInDirectory(m_path, true); }
  TempDir(const TempDir&) = delete;
  TempDir& operator=(const TempDir&) = delete;
  [[nodiscard]] std::string path() const { return m_path; }
  [[nodiscard]] std::string file(const std::string& name) const { return m_path + "/" + name; }
  void touch(const std::string& name, const std::string& contents = "x") const
  {
    std::ofstream(file(name)) << contents;
  }

 private:
  std::string m_path;
};
}  // namespace

TEST(filesystem, fileNameStripInvalidChars)
{
  EXPECT_EQ(fileNameStripInvalidChars("normal_name-1.txt"), "normal_name-1.txt");
  EXPECT_EQ(fileNameStripInvalidChars("a/b\\c:d*e?f\"g<h>i|j"), "a_b_c_d_e_f_g_h_i_j");
  EXPECT_EQ(fileNameStripInvalidChars("a/b", '-'), "a-b");
  EXPECT_EQ(fileNameStripInvalidChars(std::string("ctl\x01\x1f") + "chars"), "ctl__chars");
  EXPECT_EQ(fileNameStripInvalidChars(""), "");
}

TEST(filesystem, renameFileReportsErrorsOnlyOnFailure)
{
  TempDir d;
  d.touch("old.txt");

  std::string err = "left over";
  EXPECT_TRUE(renameFile(d.file("old.txt"), d.file("new.txt"), &err));
  EXPECT_TRUE(err.empty()) << "no error message expected on success, got: " << err;
  EXPECT_FALSE(fileExists(d.file("old.txt")));
  EXPECT_TRUE(fileExists(d.file("new.txt")));

  err.clear();
  EXPECT_FALSE(renameFile(d.file("does_not_exist.txt"), d.file("other.txt"), &err));
  EXPECT_FALSE(err.empty()) << "an error message was expected on failure";

  // The message pointer is optional:
  EXPECT_FALSE(renameFile(d.file("does_not_exist.txt"), d.file("other.txt")));
  EXPECT_TRUE(renameFile(d.file("new.txt"), d.file("newer.txt")));
}

TEST(filesystem, copyFile)
{
  TempDir d;
  d.touch("src.txt", "content");

  std::string err;
  EXPECT_TRUE(copyFile(d.file("src.txt"), d.file("dst.txt"), &err));
  EXPECT_TRUE(fileExists(d.file("dst.txt")));
  EXPECT_EQ(getFileSize(d.file("dst.txt")), 7u);

  // Overwrites an existing target:
  d.touch("dst.txt", "a");
  EXPECT_TRUE(copyFile(d.file("src.txt"), d.file("dst.txt")));
  EXPECT_EQ(getFileSize(d.file("dst.txt")), 7u);

  EXPECT_FALSE(copyFile(d.file("missing.txt"), d.file("dst2.txt"), &err));
  EXPECT_FALSE(err.empty());
  EXPECT_FALSE(fileExists(d.file("dst2.txt")));
}

TEST(filesystem, deleteFilesWithWildcards)
{
  TempDir d;
  for (const char* n : {"a1.log", "a2.log", "b1.log", "a1.txt", "keep", ".hidden.log"})
  {
    d.touch(n);
  }
  createDirectory(d.file("subdir.log"));

  deleteFiles(d.file("a?.log"));
  EXPECT_FALSE(fileExists(d.file("a1.log")));
  EXPECT_FALSE(fileExists(d.file("a2.log")));
  EXPECT_TRUE(fileExists(d.file("b1.log")));
  EXPECT_TRUE(fileExists(d.file("a1.txt")));

  // '*' does not match hidden files nor directories:
  deleteFiles(d.file("*.log"));
  EXPECT_FALSE(fileExists(d.file("b1.log")));
  EXPECT_TRUE(fileExists(d.file(".hidden.log")));
  EXPECT_TRUE(directoryExists(d.file("subdir.log")));

  // An exact name, and one that does not exist (only a warning):
  deleteFiles(d.file("keep"));
  EXPECT_FALSE(fileExists(d.file("keep")));
  EXPECT_NO_THROW(deleteFiles(d.file("nothing*matches")));
  EXPECT_NO_THROW(deleteFiles("/nonexistent-dir-for-test/*"));
  EXPECT_TRUE(fileExists(d.file("a1.txt")));
}

#ifndef _WIN32
TEST(filesystem, createDirectoryReportsErrorsWithoutThrowing)
{
  TempDir d;
  const std::string sub = d.file("sub");
  EXPECT_TRUE(createDirectory(sub));
  EXPECT_TRUE(directoryExists(sub));
  // Already existing is not an error:
  EXPECT_TRUE(createDirectory(sub));
  // Missing parent directory, or a file in the way:
  EXPECT_FALSE(createDirectory(d.file("missing/sub")));
  d.touch("a_file");
  EXPECT_FALSE(createDirectory(d.file("a_file")));
}

TEST(filesystem, deleteFilesRemovesSymbolicLinksButNotTheirTargets)
{
  TempDir d;
  createDirectory(d.file("target_dir"));
  d.touch("target_dir/inside.txt");
  d.touch("target_file.txt");
  std::error_code ec;
  std::filesystem::create_directory_symlink(d.file("target_dir"), d.file("link_to_dir.lnk"), ec);
  ASSERT_FALSE(ec);
  std::filesystem::create_symlink(d.file("target_file.txt"), d.file("link_to_file.lnk"), ec);
  ASSERT_FALSE(ec);

  deleteFiles(d.file("*.lnk"));
  EXPECT_FALSE(std::filesystem::is_symlink(d.file("link_to_dir.lnk")));
  EXPECT_FALSE(std::filesystem::is_symlink(d.file("link_to_file.lnk")));
  // ...but what they pointed to is still there:
  EXPECT_TRUE(directoryExists(d.file("target_dir")));
  EXPECT_TRUE(fileExists(d.file("target_dir/inside.txt")));
  EXPECT_TRUE(fileExists(d.file("target_file.txt")));
}
#endif

TEST(filesystem, deleteFilesDoesNotUseTheShell)
{
  // Shell metacharacters in the name are just characters of the file name:
  TempDir d;
  d.touch("victim");
  d.touch("a b");
  deleteFiles(d.file("a b; rm victim"));
  EXPECT_TRUE(fileExists(d.file("victim")));
  EXPECT_TRUE(fileExists(d.file("a b")));
  deleteFiles(d.file("a b"));
  EXPECT_FALSE(fileExists(d.file("a b")));
}

TEST(filesystem, extractFileExtensionAndNames)
{
  EXPECT_EQ(extractFileExtension("dummy.cpp"), "cpp");
  EXPECT_EQ(extractFileExtension("foo.map.gz"), "gz");
  EXPECT_EQ(extractFileExtension("foo.map.gz", true), "map");
  EXPECT_EQ(extractFileExtension("foo.gz", true), "");
  EXPECT_EQ(extractFileExtension("noextension"), "");
  EXPECT_EQ(extractFileExtension("a"), "");
  EXPECT_EQ(extractFileExtension(""), "");
  EXPECT_EQ(extractFileName("/some/dir/name.ext"), "name");
  EXPECT_EQ(extractFileDirectory("/some/dir/name.ext"), "/some/dir");
}

TEST(filesystem, modificationTimeAndSize)
{
  TempDir d;
  d.touch("f.txt", "12345");
  EXPECT_EQ(getFileSize(d.file("f.txt")), 5u);
  const auto t = getFileModificationTime(d.file("f.txt"));
  const auto now = mrpt::Clock::now();
  EXPECT_LT(std::abs(mrpt::Clock::toDouble(now) - mrpt::Clock::toDouble(t)), 3600.0);
  EXPECT_ANY_THROW(getFileModificationTime(d.file("missing")));
}
