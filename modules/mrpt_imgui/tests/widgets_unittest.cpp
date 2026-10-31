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
#include <mrpt/imgui/widgets.h>

using namespace mrpt::imgui;

TEST(CImGuiMessageQueue, Queue)
{
  CImGuiMessageQueue q;
  EXPECT_EQ(q.pending(), 0u);
  q.info("a", "b");
  q.warning("c", "d");
  q.error("e", "f");
  EXPECT_EQ(q.pending(), 3u);
  q.clear();
  EXPECT_EQ(q.pending(), 0u);
}

TEST(CImGuiAboutDialog, InformationText)
{
  CImGuiAboutDialog about;
  about.appName = "MyApp";
  about.description = "Does things.";
  const auto s = about.informationText();
  EXPECT_NE(s.find("MyApp"), std::string::npos);
  EXPECT_NE(s.find("Does things."), std::string::npos);
  EXPECT_NE(s.find("MRPT version"), std::string::npos);
  EXPECT_NE(s.find("Dear ImGui"), std::string::npos);
}
