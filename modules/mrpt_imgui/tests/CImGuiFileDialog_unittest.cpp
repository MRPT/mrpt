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
#include <mrpt/imgui/CImGuiFileDialog.h>

using namespace mrpt::imgui;

TEST(CImGuiFileDialog, fileMatchesFilter)
{
  const FileDialogFilter scenes{
      "3D scenes", {"3Dscene", "3Dscene.gz"}
  };
  EXPECT_TRUE(fileMatchesFilter("a.3Dscene", scenes));
  EXPECT_TRUE(fileMatchesFilter("A.3DSCENE", scenes));
  EXPECT_TRUE(fileMatchesFilter("dir.x/a.3Dscene.gz", scenes));
  EXPECT_FALSE(fileMatchesFilter("a.3Dscene.bak", scenes));
  EXPECT_FALSE(fileMatchesFilter("3Dscene", scenes));  // the name is not an extension
  EXPECT_FALSE(fileMatchesFilter(".3Dscene", scenes));

  const FileDialogFilter any{"All", {}};
  EXPECT_TRUE(fileMatchesFilter("whatever", any));
  const FileDialogFilter star{"All", {"*"}};
  EXPECT_TRUE(fileMatchesFilter("whatever", star));
}

TEST(CImGuiFileDialog, toNativeFilterList)
{
  const auto f = toNativeFilterList({
      {"Rawlogs", {"rawlog", "rawlog.gz"}},
      { "Images",                 {"png"}}
  });
  ASSERT_EQ(f.size(), 6u);
  EXPECT_EQ(f[0], "Rawlogs (*.rawlog *.rawlog.gz)");
  EXPECT_EQ(f[1], "*.rawlog *.rawlog.gz");
  EXPECT_EQ(f[2], "Images (*.png)");
  EXPECT_EQ(f[3], "*.png");
  EXPECT_EQ(f[4], "All files (*)");
  EXPECT_EQ(f[5], "*");

  // No duplicated "all files" entry:
  const auto g = toNativeFilterList({
      {"Anything", {}}
  });
  ASSERT_EQ(g.size(), 2u);
  EXPECT_EQ(g[0], "Anything (*)");
  EXPECT_EQ(g[1], "*");

  EXPECT_EQ(toNativeFilterList({}).size(), 2u);
}

TEST(CImGuiFileDialog, ClosedByDefault)
{
  CImGuiFileDialog dlg;
  EXPECT_FALSE(dlg.isOpen());
  EXPECT_FALSE(dlg.poll().has_value());
  EXPECT_TRUE(dlg.lastDirectory().empty());
}
