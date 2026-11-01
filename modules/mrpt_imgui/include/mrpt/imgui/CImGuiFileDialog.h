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
#pragma once

#include <mrpt/imgui/SimpleFileDialog.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace mrpt::imgui
{
/** Open/save/select-folder dialogs for ImGui applications.
 *
 * Uses the native dialogs of the system (via portable-file-dialogs: zenity,
 * kdialog, Windows or macOS dialogs) when available, and otherwise falls back
 * to SimpleFileDialog, drawn with ImGui. Native dialogs run asynchronously,
 * so the GUI keeps rendering while they are open.
 *
 * The backend can be forced with `backend`, or with the environment variable
 * `MRPT_IMGUI_FILE_DIALOG=native|imgui`.
 *
 * \code
 *   mrpt::imgui::CImGuiFileDialog dlg;
 *   if (ImGui::MenuItem("Open...")) {
 *     dlg.open(CImGuiFileDialog::Mode::Open, "Open scene", {{"3D scenes", {"3Dscene"}}});
 *   }
 *   // Every frame:
 *   if (auto files = dlg.poll(); files && !files->empty()) { load(files->front()); }
 * \endcode
 * \ingroup mrpt_imgui_grp
 */
class CImGuiFileDialog
{
 public:
  enum class Mode
  {
    Open,
    OpenMultiple,
    Save,
    SelectFolder
  };
  enum class Backend
  {
    Auto,
    Native,
    ImGui
  };

  CImGuiFileDialog();
  ~CImGuiFileDialog();

  CImGuiFileDialog(const CImGuiFileDialog&) = delete;
  CImGuiFileDialog& operator=(const CImGuiFileDialog&) = delete;

  /** Opens the dialog. `initialPath` may be a directory or a file name. If a
   * dialog is already open, it is abandoned. */
  void open(
      Mode mode,
      const std::string& title,
      const std::vector<FileDialogFilter>& filters = {},
      const std::string& initialPath = "");

  /** Must be called every frame (it draws the ImGui fallback, if used).
   * Returns the result exactly once when the dialog closes: the selected
   * path(s), or an empty vector if it was cancelled. nullopt otherwise. */
  std::optional<std::vector<std::string>> poll();

  [[nodiscard]] bool isOpen() const;

  /** The mode of the last open() call (to tell results apart when one
   * dialog object serves several commands). */
  [[nodiscard]] Mode mode() const { return m_mode; }

  /** A tag set by the caller in open() to know what the result is for. */
  std::string tag;

  /** Directory of the last selected path, used as the default next time. */
  [[nodiscard]] const std::string& lastDirectory() const { return m_lastDir; }

  /** Whether a native dialog helper is available on this system. */
  [[nodiscard]] static bool nativeAvailable();

  Backend backend = Backend::Auto;

 private:
  struct NativeImpl;
  std::unique_ptr<NativeImpl> m_native;
  SimpleFileDialog m_fallback;
  bool m_fallbackActive = false;
  Mode m_mode = Mode::Open;
  std::string m_lastDir;

  [[nodiscard]] bool useNative() const;
  void rememberDirectory(const std::vector<std::string>& paths);
};

}  // namespace mrpt::imgui
