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

#include <deque>
#include <string>
#include <utility>
#include <vector>

namespace mrpt::imgui
{
/** Modal message boxes for errors, warnings and information, shown one at a
 * time (the replacement for `wxMessageBox()`). Messages can be queued from
 * any place of the frame code; render() must be called once per frame.
 * \ingroup mrpt_imgui_grp
 */
class CImGuiMessageQueue
{
 public:
  enum class Kind
  {
    Info,
    Warning,
    Error
  };

  void info(const std::string& title, const std::string& text) { push(Kind::Info, title, text); }
  void warning(const std::string& title, const std::string& text)
  {
    push(Kind::Warning, title, text);
  }
  void error(const std::string& title, const std::string& text) { push(Kind::Error, title, text); }
  void push(Kind kind, const std::string& title, const std::string& text);

  [[nodiscard]] size_t pending() const { return m_queue.size(); }
  void clear() { m_queue.clear(); }

  /** Shows the oldest queued message, until the user closes it. */
  void render();

 private:
  struct Msg
  {
    Kind kind;
    std::string title;
    std::string text;
  };
  std::deque<Msg> m_queue;
  bool m_popupOpened = false;
};

/** The "About" dialog of an MRPT application: name, description, MRPT,
 * ImGui and OpenGL versions, the MRPT license, and any app-specific tabs
 * (e.g. keyboard and mouse commands).
 * \ingroup mrpt_imgui_grp
 */
class CImGuiAboutDialog
{
 public:
  std::string appName;
  std::string description;
  /** Additional tabs: (title, text). */
  std::vector<std::pair<std::string, std::string>> extraTabs;

  void open() { m_requestOpen = true; }
  /** Call once per frame. */
  void render();

  /** The "Information" tab text. */
  [[nodiscard]] std::string informationText() const;

 private:
  bool m_requestOpen = false;
};

/** Begins a one-line status bar pinned to the bottom of the main window
 * (bars submitted earlier stay closer to the edge). Call it from
 * CImGuiAppShell::beforeDockspace, and always call endStatusBar() after it;
 * separate fields with ImGui::SameLine().
 * \return true if the bar contents must be submitted.
 * \ingroup mrpt_imgui_grp
 */
bool beginStatusBar(const char* name = "##StatusBar");
void endStatusBar();

/** Shows a text table with two columns (e.g. "key", "description"). */
void keyValueTable(const char* id, const std::vector<std::pair<std::string, std::string>>& rows);

}  // namespace mrpt::imgui
