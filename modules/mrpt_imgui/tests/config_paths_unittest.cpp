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
#include <mrpt/imgui/config_paths.h>

#include <cstdlib>
#include <filesystem>

using namespace mrpt::imgui;

TEST(config_paths, sanitizeFileName)
{
  EXPECT_EQ(sanitizeFileName("SceneViewer3D"), "SceneViewer3D");
  EXPECT_EQ(sanitizeFileName("a/b\\c:d*e?f\"g<h>i|j"), "a_b_c_d_e_f_g_h_i_j");
  EXPECT_EQ(sanitizeFileName("  .hidden. "), "hidden");
  EXPECT_EQ(sanitizeFileName(""), "app");
  EXPECT_EQ(sanitizeFileName(" .. "), "app");
}

#if !defined(_WIN32)
TEST(config_paths, userConfigDirectoryUsesXdg)
{
  namespace fs = std::filesystem;
  const fs::path tmp = fs::temp_directory_path() / "mrpt_imgui_cfg_test";
  fs::remove_all(tmp);

  const char* old = std::getenv("XDG_CONFIG_HOME");
  const std::string oldValue = old != nullptr ? old : "";
  ::setenv("XDG_CONFIG_HOME", tmp.string().c_str(), 1);

  const auto dir = userConfigDirectory("my/app");
  EXPECT_EQ(dir, (tmp / "my_app").string() + "/");
  EXPECT_TRUE(fs::is_directory(tmp / "my_app"));

  const auto dirNoCreate = userConfigDirectory("other", false);
  EXPECT_EQ(dirNoCreate, (tmp / "other").string() + "/");
  EXPECT_FALSE(fs::exists(tmp / "other"));

  if (old != nullptr)
  {
    ::setenv("XDG_CONFIG_HOME", oldValue.c_str(), 1);
  }
  else
  {
    ::unsetenv("XDG_CONFIG_HOME");
  }
  fs::remove_all(tmp);
}
#endif
