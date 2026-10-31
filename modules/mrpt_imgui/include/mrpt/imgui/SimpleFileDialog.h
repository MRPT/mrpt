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

#include <optional>
#include <string>
#include <vector>

namespace mrpt::imgui
{
/** One file-type filter of a file dialog, e.g.
 * `{"3D scenes", {"3Dscene", "3Dscene.gz"}}`. Extensions have no leading dot
 * and are matched case-insensitively against the end of file names, so
 * compound ones (`rawlog.gz`) work. No extensions means "any file".
 * \ingroup mrpt_imgui_grp
 */
struct FileDialogFilter
{
  std::string description;
  std::vector<std::string> extensions;
};

/** True if `fileName` matches the filter (see FileDialogFilter). */
[[nodiscard]] bool fileMatchesFilter(const std::string& fileName, const FileDialogFilter& filter);

/** Converts filters into the flat list of portable-file-dialogs: pairs of
 * `"Description (*.a *.b)"`, `"*.a *.b"`. An "All files" entry is appended
 * if no filter accepts any file. */
[[nodiscard]] std::vector<std::string> toNativeFilterList(
    const std::vector<FileDialogFilter>& filters);

/** Minimal file browser drawn with ImGui itself (a modal popup), with no
 * dependency on any system dialog helper. Used by CImGuiFileDialog when no
 * native dialog is available (e.g. remote or minimal sessions).
 *
 * \code
 *   dlg.open(SimpleFileDialog::Mode::Open, {{"Rawlogs", {"rawlog"}}});
 *   // Every frame:
 *   if (auto path = dlg.render(); path) { ... }
 * \endcode
 * \ingroup mrpt_imgui_grp
 */
class SimpleFileDialog
{
 public:
  enum class Mode
  {
    Open,
    Save,
    SelectFolder
  };

  /** Opens the dialog, starting at `initialPath` (a directory, or a file
   * whose directory and name are used), or else the last directory used, or
   * else the current directory. */
  void open(
      Mode mode,
      const std::vector<FileDialogFilter>& filters,
      const std::string& title = "",
      const std::string& initialPath = "");

  /** Must be called every frame while isOpen(). Returns the selected path
   * once the user confirms, nullopt otherwise (still open, or cancelled:
   * check isOpen() to tell them apart). */
  std::optional<std::string> render();

  [[nodiscard]] bool isOpen() const { return m_open; }

  /** Closes the dialog without a result. */
  void close() { m_open = false; }

  [[nodiscard]] const std::string& currentDirectory() const { return m_currentDir; }

 private:
  bool m_open = false;
  bool m_pendingOpen = false;
  Mode m_mode = Mode::Open;
  std::string m_title;
  std::vector<FileDialogFilter> m_filters;
  int m_selectedFilterIdx = 0;

  std::string m_currentDir;
  std::string m_typedName;
  std::string m_errorMsg;

  std::string m_popupId;
  std::string m_overwritePopupId;
  std::string m_pendingOverwritePath;
};

}  // namespace mrpt::imgui
