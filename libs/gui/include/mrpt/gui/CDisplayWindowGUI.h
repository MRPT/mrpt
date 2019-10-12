/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/gui/CBaseGUIWindow.h>

#include <mrpt/config.h>
#if MRPT_HAS_NANOGUI
#include <nanogui/nanogui.h>
#endif

namespace mrpt::gui
{
/** A window with powerful GUI capabilities, via the nanogui library.
 *
 * \ingroup mrpt_gui_grp
 */
class CDisplayWindowGUI : public mrpt::gui::CBaseGUIWindow
{
   public:
	using Ptr = std::shared_ptr<CDisplayWindowGUI>;
	using ConstPtr = std::shared_ptr<const CDisplayWindowGUI>;

   protected:
   public:
	/** Constructor */
	CDisplayWindowGUI(
		const std::string& windowCaption = std::string(),
		unsigned int initialWindowWidth = 400,
		unsigned int initialWindowHeight = 300);

	/** Class factory returning a smart pointer */
	static CDisplayWindowGUI::Ptr Create(
		const std::string& windowCaption, unsigned int initialWindowWidth = 400,
		unsigned int initialWindowHeight = 300);

	/** Destructor */
	~CDisplayWindowGUI() override;

	/** Resizes the window */
	void resize(unsigned int width, unsigned int height) override;
	/** Changes the position of the window on the screen. */
	void setPos(int x, int y) override;
	/**  Changes the window title. */
	void setWindowTitle(const std::string& str) override;

};  // End of class def.

}  // namespace mrpt::gui
