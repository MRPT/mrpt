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
#include <imgui.h>
#include <imgui_internal.h>
#include <mrpt/imgui/CImGuiAppShell.h>
#include <mrpt/imgui/CImGuiBackgroundScene.h>
#include <mrpt/imgui/CImGuiImageView.h>
#include <mrpt/imgui/CImGuiLogConsole.h>
#include <mrpt/imgui/CImGuiPlotPanel.h>
#include <mrpt/imgui/CImGuiTimeline.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CGridPlaneXY.h>

#include <cstdlib>
#include <filesystem>

// A GUI smoke test: needs a display (run under xvfb-run on CI). Asserts that
// frames of plausible size are rendered and the layout is persisted, never
// pixel contents (readback may be black without a compositor).

namespace
{
struct ScopedConfigHome
{
  std::filesystem::path dir;
  std::string old;
  bool hadOld = false;

  ScopedConfigHome()
  {
    dir = std::filesystem::temp_directory_path() / "mrpt_imgui_shell_test";
    std::filesystem::remove_all(dir);
#if !defined(_WIN32)
    if (const char* o = std::getenv("XDG_CONFIG_HOME"); o != nullptr)
    {
      old = o;
      hadOld = true;
    }
    ::setenv("XDG_CONFIG_HOME", dir.string().c_str(), 1);
#endif
  }
  ~ScopedConfigHome()
  {
#if !defined(_WIN32)
    if (hadOld)
    {
      ::setenv("XDG_CONFIG_HOME", old.c_str(), 1);
    }
    else
    {
      ::unsetenv("XDG_CONFIG_HOME");
    }
#endif
    std::filesystem::remove_all(dir);
  }
};

bool skipGuiTests()
{
  const char* s = std::getenv("MRPT_SKIP_GUI_TESTS");
  return s != nullptr && std::string(s) == "1";
}

struct RunStats
{
  bool initOk = false;
  uint64_t frames = 0;
  int menuCalls = 0;
  int beforeDockCalls = 0;
  int layoutCalls = 0;
  int imgW = 0;
  int imgH = 0;
  std::string ini;
};

RunStats runApp(uint64_t frames)
{
  RunStats st;
  mrpt::imgui::CImGuiAppShell app;
  mrpt::imgui::CImGuiAppShell::Options opts;
  opts.width = 640;
  opts.height = 480;
  opts.vsync = false;
  if (!app.init("mrpt_imgui_test_app", opts))
  {
    return st;
  }
  st.initOk = true;
  app.maxFrames = frames;
  st.ini = app.iniFilePath();

  auto scene = mrpt::viz::Scene::Create();
  scene->insert(mrpt::viz::CGridPlaneXY::Create());
  scene->insert(
      mrpt::viz::CBox::Create(mrpt::math::TPoint3D(0, 0, 0), mrpt::math::TPoint3D(1, 1, 1)));
  mrpt::imgui::CImGuiBackgroundScene bg;
  bg.view.setScene(scene);

  mrpt::imgui::CImGuiTimeline timeline;
  timeline.setKeyTimes({0.0, 1.0, 2.0});
  mrpt::imgui::CImGuiPlotPanel plot;
  plot.setSeries("s", {0, 1, 2}, {1, 3, 2});
  mrpt::imgui::CImGuiLogConsole console;
  console.push(mrpt::system::LVL_INFO, "test", "hello");
  mrpt::imgui::CImGuiImageView imageView;
  imageView.setImage(mrpt::img::CImage(64, 48, mrpt::img::CH_RGB));

  app.mainMenuBar = [&]()
  {
    st.menuCalls++;
    if (ImGui::BeginMenu("File"))
    {
      ImGui::EndMenu();
    }
  };
  app.beforeDockspace = [&]()
  {
    st.beforeDockCalls++;
    timeline.renderAsBottomBar();
  };
  app.setupDefaultLayout = [&](unsigned int dockId)
  {
    st.layoutCalls++;
    ImGuiID left = 0;
    ImGuiID rest = 0;
    ImGui::DockBuilderSplitNode(dockId, ImGuiDir_Left, 0.3f, &left, &rest);
    ImGui::DockBuilderDockWindow("Plot", left);
  };

  app.run(
      [&]()
      {
        bg.render(app);
        ImGui::Begin("Plot");
        plot.render("plot");
        ImGui::End();
        ImGui::Begin("Console");
        console.render();
        ImGui::End();
        ImGui::Begin("Image");
        imageView.render("img");
        ImGui::End();
      });
  st.frames = app.frameCount();
  if (auto img = app.grabFramebuffer(); img)
  {
    st.imgW = static_cast<int>(img->getWidth());
    st.imgH = static_cast<int>(img->getHeight());
  }
  return st;
}
}  // namespace

TEST(CImGuiAppShell, RunsFramesAndPersistsLayout)
{
  if (skipGuiTests())
  {
    GTEST_SKIP() << "MRPT_SKIP_GUI_TESTS=1";
  }
  ScopedConfigHome cfg;

  const auto first = runApp(5);
  if (!first.initOk)
  {
    GTEST_SKIP() << "No display available for an OpenGL window";
  }
  EXPECT_EQ(first.frames, 5u);
  EXPECT_EQ(first.menuCalls, 5);
  EXPECT_EQ(first.beforeDockCalls, 5);
  EXPECT_EQ(first.layoutCalls, 1);  // first run: no saved layout
  EXPECT_GE(first.imgW, 100);
  EXPECT_GE(first.imgH, 100);
  ASSERT_FALSE(first.ini.empty());
  EXPECT_TRUE(std::filesystem::exists(first.ini));

  // Second run: the layout is restored, not rebuilt.
  const auto second = runApp(3);
  ASSERT_TRUE(second.initOk);
  EXPECT_EQ(second.frames, 3u);
  EXPECT_EQ(second.layoutCalls, 0);
}

TEST(CImGuiAppShell, NotInitializedIsSafe)
{
  mrpt::imgui::CImGuiAppShell app;
  EXPECT_FALSE(app.isInitialized());
  EXPECT_FALSE(app.grabFramebuffer().has_value());
  app.run([]() {});  // returns at once
  app.requestClose();
  app.shutdown();
  EXPECT_EQ(app.frameCount(), 0u);
}
