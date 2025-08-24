/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/gui/CGlCanvasBase.h>

// Expose nanogui API to mrpt users, for direct use of nanogui classes.
#include <mrpt/gui/config.h>
#if MRPT_HAS_NANOGUI

#ifdef None  // X11 headers conflict...
#undef None
#endif
#include <nanogui/nanogui.h>

namespace mrpt::gui::internal
{
/** Specialization of CGlCanvasBaseHeadless for nanogui events.
 * Used to keep track of mouse events and update camera parameters.
 * \sa CDisplayWindowGUI
 * \ingroup mrpt_gui_grp
 */
class NanoGUICanvasHeadless : public mrpt::gui::CGlCanvasBaseHeadless
{
 public:
  void mouseMotionEvent(
      const nanogui::Vector2i& p, const nanogui::Vector2i& rel, int button, int modifiers);
  void mouseButtonEvent(const nanogui::Vector2i& p, int button, bool down, int modifiers);
  void scrollEvent(const nanogui::Vector2i& p, const nanogui::Vector2f& rel);

 private:
  int m_lastModifiers = 0;
};
}  // namespace mrpt::gui::internal
#endif  // MRPT_HAS_NANOGUI
