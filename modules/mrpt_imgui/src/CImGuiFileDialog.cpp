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

#include <mrpt/imgui/CImGuiFileDialog.h>

#include <cstdlib>
#include <filesystem>
#include <iostream>

// Header-only library, kept out of public headers:
#include <portable-file-dialogs.h>

namespace mrpt::imgui
{
struct CImGuiFileDialog::NativeImpl
{
  std::unique_ptr<pfd::open_file> openDlg;
  std::unique_ptr<pfd::save_file> saveDlg;
  std::unique_ptr<pfd::select_folder> folderDlg;

  [[nodiscard]] bool active() const { return openDlg || saveDlg || folderDlg; }

  void kill()
  {
    if (openDlg)
    {
      openDlg->kill();
    }
    if (saveDlg)
    {
      saveDlg->kill();
    }
    if (folderDlg)
    {
      folderDlg->kill();
    }
    openDlg.reset();
    saveDlg.reset();
    folderDlg.reset();
  }

  /** Non-blocking: returns the result once the dialog is closed. */
  std::optional<std::vector<std::string>> poll()
  {
    if (openDlg && openDlg->ready(0))
    {
      auto r = openDlg->result();
      openDlg.reset();
      return r;
    }
    if (saveDlg && saveDlg->ready(0))
    {
      auto r = saveDlg->result();
      saveDlg.reset();
      return r.empty() ? std::vector<std::string>{} : std::vector<std::string>{r};
    }
    if (folderDlg && folderDlg->ready(0))
    {
      auto r = folderDlg->result();
      folderDlg.reset();
      return r.empty() ? std::vector<std::string>{} : std::vector<std::string>{r};
    }
    return std::nullopt;
  }
};

CImGuiFileDialog::CImGuiFileDialog() : m_native(std::make_unique<NativeImpl>()) {}

CImGuiFileDialog::~CImGuiFileDialog()
{
  if (m_native)
  {
    m_native->kill();
  }
}

bool CImGuiFileDialog::nativeAvailable() { return pfd::settings::available(); }

bool CImGuiFileDialog::useNative() const
{
  Backend b = backend;
  if (b == Backend::Auto)
  {
    if (const char* env = std::getenv("MRPT_IMGUI_FILE_DIALOG"); env != nullptr)
    {
      const std::string e(env);
      if (e == "imgui")
      {
        b = Backend::ImGui;
      }
      else if (e == "native")
      {
        b = Backend::Native;
      }
    }
  }
  switch (b)
  {
    case Backend::ImGui:
      return false;
    case Backend::Native:
      return true;
    case Backend::Auto:
    default:
      return nativeAvailable();
  }
}

void CImGuiFileDialog::open(
    Mode mode,
    const std::string& title,
    const std::vector<FileDialogFilter>& filters,
    const std::string& initialPath)
{
  m_native->kill();
  m_fallback.close();
  m_fallbackActive = false;
  m_mode = mode;

  const std::string startPath = !initialPath.empty() ? initialPath : m_lastDir;

  if (useNative())
  {
    const auto nativeFilters = toNativeFilterList(filters);
    try
    {
      switch (mode)
      {
        case Mode::Open:
          m_native->openDlg = std::make_unique<pfd::open_file>(title, startPath, nativeFilters);
          break;
        case Mode::OpenMultiple:
          m_native->openDlg = std::make_unique<pfd::open_file>(
              title, startPath, nativeFilters, pfd::opt::multiselect);
          break;
        case Mode::Save:
          m_native->saveDlg = std::make_unique<pfd::save_file>(title, startPath, nativeFilters);
          break;
        case Mode::SelectFolder:
          m_native->folderDlg = std::make_unique<pfd::select_folder>(title, startPath);
          break;
      }
      return;
    }
    catch (const std::exception& e)
    {
      std::cerr << "[CImGuiFileDialog] Native dialog failed, using fallback: " << e.what() << "\n";
      m_native->kill();
    }
  }

  const auto fbMode = mode == Mode::Save           ? SimpleFileDialog::Mode::Save
                      : mode == Mode::SelectFolder ? SimpleFileDialog::Mode::SelectFolder
                                                   : SimpleFileDialog::Mode::Open;
  m_fallback.open(fbMode, filters, title, startPath);
  m_fallbackActive = true;
}

bool CImGuiFileDialog::isOpen() const { return m_native->active() || m_fallbackActive; }

void CImGuiFileDialog::rememberDirectory(const std::vector<std::string>& paths)
{
  if (paths.empty())
  {
    return;
  }
  const std::filesystem::path p(paths.front());
  std::error_code ec;
  m_lastDir = std::filesystem::is_directory(p, ec) ? p.string() : p.parent_path().string();
}

std::optional<std::vector<std::string>> CImGuiFileDialog::poll()
{
  if (m_native->active())
  {
    auto r = m_native->poll();
    if (r)
    {
      rememberDirectory(*r);
    }
    return r;
  }
  if (m_fallbackActive)
  {
    if (auto r = m_fallback.render(); r)
    {
      m_fallbackActive = false;
      std::vector<std::string> ret{*r};
      rememberDirectory(ret);
      return ret;
    }
    if (!m_fallback.isOpen())
    {
      m_fallbackActive = false;
      return std::vector<std::string>{};  // cancelled
    }
  }
  return std::nullopt;
}

}  // namespace mrpt::imgui
