/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/gui/CGlCanvasBase.h>

// Expose nanogui API to mrpt users, for direct use of nanogui classes.
#include <mrpt/config.h>
#if MRPT_HAS_NANOGUI
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
		const nanogui::Vector2i& p, const nanogui::Vector2i& rel, int button,
		int modifiers);
	void mouseButtonEvent(
		const nanogui::Vector2i& p, int button, bool down, int modifiers);
	void scrollEvent(const nanogui::Vector2i& p, const nanogui::Vector2f& rel);

   private:
	int m_lastModifiers = 0;
};
}  // namespace mrpt::gui::internal
#endif	// MRPT_HAS_NANOGUI
