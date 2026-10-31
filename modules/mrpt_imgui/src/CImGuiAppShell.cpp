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

// clang-format off
#include <mrpt/opengl/opengl_api.h>  // must go before GLFW
// clang-format on

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <imgui_internal.h>  // DockBuilder API
#include <implot.h>
#include <mrpt/imgui/CImGuiAppShell.h>
#include <mrpt/imgui/config_paths.h>
#include <mrpt/imgui/icons.h>
#include <mrpt/imgui/style.h>
#include <mrpt/imgui_vendor/icon_font.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>

namespace mrpt::imgui
{
namespace
{
// Must outlive the font atlas, which keeps a pointer to it:
const ImWchar kIconRanges[] = {ICON_MIN_MS, ICON_MAX_16_MS, 0};

void glfwErrorCallback(int error, const char* description)
{
  std::cerr << "[CImGuiAppShell] GLFW error " << error << ": " << description << "\n";
}

uint64_t envUInt(const char* name)
{
  const char* v = std::getenv(name);
  if (v == nullptr || *v == '\0')
  {
    return 0;
  }
  return std::strtoull(v, nullptr, 10);
}

CImGuiAppShell* shellOf(GLFWwindow* w)
{
  return static_cast<CImGuiAppShell*>(glfwGetWindowUserPointer(w));
}

void onGlfwDrop(GLFWwindow* w, int count, const char** paths)
{
  auto* shell = shellOf(w);
  if (shell == nullptr || !shell->onFilesDropped)
  {
    return;
  }
  std::vector<std::string> files;
  for (int i = 0; i < count; i++)
  {
    files.emplace_back(paths[i]);
  }
  shell->onFilesDropped(files);
}

void onGlfwClose(GLFWwindow* w)
{
  auto* shell = shellOf(w);
  if (shell != nullptr && shell->onCloseRequest && !shell->onCloseRequest())
  {
    glfwSetWindowShouldClose(w, GLFW_FALSE);
  }
}
}  // namespace

CImGuiAppShell::~CImGuiAppShell() { shutdown(); }

bool CImGuiAppShell::init(const std::string& windowTitle, const Options& options)
{
  if (m_window != nullptr)
  {
    return true;
  }

  maxFrames = envUInt("MRPT_IMGUI_MAX_FRAMES");
  if (const char* shot = std::getenv("MRPT_IMGUI_SCREENSHOT"); shot != nullptr)
  {
    m_exitScreenshot = shot;
  }

  glfwSetErrorCallback(glfwErrorCallback);
  if (!glfwInit())
  {
    std::cerr << "[CImGuiAppShell] glfwInit() failed (no display?)\n";
    return false;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#if defined(__APPLE__)
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
  glfwWindowHint(GLFW_DEPTH_BITS, 24);
  glfwWindowHint(GLFW_SAMPLES, std::max(0, options.msaaSamples));
  glfwWindowHint(GLFW_SRGB_CAPABLE, options.srgbFramebuffer ? GLFW_TRUE : GLFW_FALSE);
  glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);

  m_window = glfwCreateWindow(
      options.width, options.height, windowTitle.c_str(), nullptr /*monitor*/, nullptr);
  if (m_window == nullptr && options.msaaSamples > 0)
  {
    // Retry without multisampling, which some (virtual) displays lack:
    glfwWindowHint(GLFW_SAMPLES, 0);
    m_window =
        glfwCreateWindow(options.width, options.height, windowTitle.c_str(), nullptr, nullptr);
  }
  if (m_window == nullptr)
  {
    std::cerr << "[CImGuiAppShell] glfwCreateWindow() failed\n";
    glfwTerminate();
    return false;
  }
  glfwMakeContextCurrent(m_window);
  glfwSwapInterval(options.vsync ? 1 : 0);
  glfwSetWindowUserPointer(m_window, this);
  glfwSetDropCallback(m_window, &onGlfwDrop);
  glfwSetWindowCloseCallback(m_window, &onGlfwClose);

#if defined(_WIN32) && (MRPT_HAS_OPENGL || MRPT_HAS_EGL)
  glewInit();
#endif

  {
    float xs = 1.0f;
    float ys = 1.0f;
    glfwGetWindowContentScale(m_window, &xs, &ys);
    m_dpiScale = std::max(1.0f, std::max(xs, ys));
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();

  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

  // Layout persisted per application, in its config directory:
  m_iniPath.clear();
  if (options.persistLayout)
  {
    const std::string cfgName = options.configName.empty() ? windowTitle : options.configName;
    if (const auto dir = userConfigDirectory(cfgName); !dir.empty())
    {
      m_iniPath = dir + "imgui.ini";
    }
  }
  io.IniFilename = m_iniPath.empty() ? nullptr : m_iniPath.c_str();

  applyStyle(m_dpiScale);
  ImGuiStyle& style = ImGui::GetStyle();
  style.FontSizeBase = options.fontSize;
  style.FontScaleDpi = m_dpiScale;

  // Default font, plus the embedded Material Symbols icons merged into it:
  {
    ImFontConfig base;
    base.SizePixels = options.fontSize;
    io.Fonts->AddFontDefault(&base);
  }
  {
    ImFontConfig cfg;
    cfg.MergeMode = true;
    cfg.FontDataOwnedByAtlas = false;             // static array, must not be freed
    cfg.GlyphOffset.y = 0.2f * options.fontSize;  // align icons with the text baseline
    io.Fonts->AddFontFromMemoryTTF(
        const_cast<void*>(mrpt::imgui_vendor::iconFontData()),
        static_cast<int>(mrpt::imgui_vendor::iconFontDataSize()), options.fontSize, &cfg,
        kIconRanges);
  }

  ImGui_ImplGlfw_InitForOpenGL(m_window, true);
  ImGui_ImplOpenGL3_Init("#version 330");
  m_imguiInitialized = true;
  m_frameCount = 0;
  m_closeRequested = false;

  return true;
}

void CImGuiAppShell::setWindowTitle(const std::string& title)
{
  if (m_window != nullptr)
  {
    glfwSetWindowTitle(m_window, title.c_str());
  }
}

void CImGuiAppShell::buildDockspace()
{
  ImGuiViewport* viewport = ImGui::GetMainViewport();
  m_dockspaceId = ImGui::GetID("MainDockSpace");

  if (setupDefaultLayout && ImGui::DockBuilderGetNode(m_dockspaceId) == nullptr)
  {
    ImGui::DockBuilderRemoveNode(m_dockspaceId);
    ImGui::DockBuilderAddNode(
        m_dockspaceId, ImGuiDockNodeFlags_PassthruCentralNode | ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(m_dockspaceId, viewport->WorkSize);
    setupDefaultLayout(m_dockspaceId);
    ImGui::DockBuilderFinish(m_dockspaceId);
  }

  ImGui::DockSpaceOverViewport(m_dockspaceId, viewport, ImGuiDockNodeFlags_PassthruCentralNode);

  if (const ImGuiDockNode* central = ImGui::DockBuilderGetCentralNode(m_dockspaceId);
      central != nullptr)
  {
    m_centralRect = {central->Pos.x, central->Pos.y, central->Size.x, central->Size.y};
  }
  else
  {
    m_centralRect = {
        viewport->WorkPos.x, viewport->WorkPos.y, viewport->WorkSize.x, viewport->WorkSize.y};
  }
}

std::optional<mrpt::img::CImage> CImGuiAppShell::grabFramebuffer() const
{
  if (m_window == nullptr)
  {
    return std::nullopt;
  }
  int w = 0;
  int h = 0;
  glfwGetFramebufferSize(m_window, &w, &h);
  if (w <= 0 || h <= 0)
  {
    return std::nullopt;
  }
  mrpt::img::CImage img(w, h, mrpt::img::CH_RGB);
  std::vector<uint8_t> buf(static_cast<size_t>(w) * static_cast<size_t>(h) * 3);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadBuffer(GL_BACK);
  glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, buf.data());
  // GL rows go bottom-up:
  const bool bgr = img.getChannelsOrder() == std::string("BGR");
  for (int y = 0; y < h; y++)
  {
    const uint8_t* src = &buf[static_cast<size_t>(h - 1 - y) * static_cast<size_t>(w) * 3];
    auto* dst = img.ptrLine<uint8_t>(static_cast<unsigned>(y));
    for (int x = 0; x < w; x++)
    {
      dst[3 * x + 0] = src[3 * x + (bgr ? 2 : 0)];
      dst[3 * x + 1] = src[3 * x + 1];
      dst[3 * x + 2] = src[3 * x + (bgr ? 0 : 2)];
    }
  }
  return img;
}

void CImGuiAppShell::handleScreenshotRequests(bool lastFrame)
{
  std::string file;
  if (!m_pendingScreenshot.empty())
  {
    file = m_pendingScreenshot;
    m_pendingScreenshot.clear();
  }
  else if (lastFrame && !m_exitScreenshot.empty())
  {
    file = m_exitScreenshot;
  }
  if (file.empty())
  {
    return;
  }
  if (auto img = grabFramebuffer(); img)
  {
    if (!img->saveToFile(file))
    {
      std::cerr << "[CImGuiAppShell] Could not save screenshot to: " << file << "\n";
    }
  }
}

bool CImGuiAppShell::shouldStop()
{
  return m_window == nullptr || m_closeRequested || glfwWindowShouldClose(m_window) ||
         (maxFrames != 0 && m_frameCount >= maxFrames);
}

void CImGuiAppShell::run(const std::function<void()>& renderFrame)
{
  while (!shouldStop())
  {
    glfwPollEvents();
    if (glfwGetWindowAttrib(m_window, GLFW_ICONIFIED) != 0)
    {
      ImGui_ImplGlfw_Sleep(10);
      continue;
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    if (mainMenuBar && ImGui::BeginMainMenuBar())
    {
      mainMenuBar();
      ImGui::EndMainMenuBar();
    }
    if (beforeDockspace)
    {
      beforeDockspace();
    }
    buildDockspace();

    if (renderFrame)
    {
      renderFrame();
    }

    ImGui::Render();

    int displayW = 0;
    int displayH = 0;
    glfwGetFramebufferSize(m_window, &displayW, &displayH);
    glViewport(0, 0, displayW, displayH);
    glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    m_frameCount++;
    handleScreenshotRequests(shouldStop());

    glfwSwapBuffers(m_window);
  }
}

void CImGuiAppShell::requestClose() { m_closeRequested = true; }

void CImGuiAppShell::shutdown()
{
  if (m_imguiInitialized)
  {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    m_imguiInitialized = false;
  }
  if (m_window != nullptr)
  {
    glfwDestroyWindow(m_window);
    m_window = nullptr;
    glfwTerminate();
  }
}

}  // namespace mrpt::imgui
