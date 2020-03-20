/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/gui/MRPT2NanoguiGLCanvas.h>
#include <mrpt/gui/internal/NanoGUICanvasHeadless.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mutex>
#include <string>

// Expose nanogui API to mrpt users, for direct use of nanogui classes.
#include <mrpt/config.h>
#if MRPT_HAS_NANOGUI
#include <nanogui/nanogui.h>

namespace mrpt::gui
{
/** A window with powerful GUI capabilities, via the nanogui library.
 *
 * You can add a background mrpt::opengl::COpenGLScene object rendered on the
 * background of the entire window by setting an object in field
 * `background_scene`, locking its mutex `background_scene_mtx`.
 *
 * Refer to nanogui API docs or MRPT examples for further usage examples.
 * A typical lifecycle of a GUI app with this class might look like:
 *
 * \rst
 * .. code-block:: cpp
 *
 *    nanogui::init();
 *    {
 *      mrpt::gui::CDisplayWindowGUI win;
 *      win.drawAll();
 *      win.setVisible(true);
 *      nanogui::mainloop();
 *    }
 *    nanogui::shutdown();
 * \endrst
 *
 * \ingroup mrpt_gui_grp
 */
class CDisplayWindowGUI : public nanogui::Screen
{
   public:
	/** @name Ctor and basic window set up
	 * @{ */
	using Ptr = std::shared_ptr<CDisplayWindowGUI>;
	using ConstPtr = std::shared_ptr<const CDisplayWindowGUI>;

	CDisplayWindowGUI(
		const std::string& caption = std::string(), unsigned int width = 400,
		unsigned int height = 300, bool resizable = true,
		bool fullscreen = false);

	virtual ~CDisplayWindowGUI() override;

	/** Class factory returning a smart pointer */
	template <typename... Args>
	static Ptr Create(Args&&... args)
	{
		return std::make_shared<CDisplayWindowGUI>(std::forward<Args>(args)...);
	}

	/** Resizes the window */
	void resize(unsigned int width, unsigned int height);

	/** Changes the position of the window on the screen. */
	void setPos(int x, int y);

	/**  Changes the window title. */
	void setWindowTitle(const std::string& str);

	/** @} */

	/** @name Access to full-window (background) GL scene
	 * @{ */

	mrpt::opengl::COpenGLScene::Ptr background_scene;
	std::mutex background_scene_mtx;

	/** @} */

	/** @name Direct access to underlying nanogui API
	 * @{ */

	nanogui::Window* nanogui_win()
	{
		ASSERT_(m_window);
		return m_window;
	}

	/** @} */

   private:
	CDisplayWindowGUI(const CDisplayWindowGUI&) = delete;
	CDisplayWindowGUI& operator=(const CDisplayWindowGUI&) = delete;

	CDisplayWindowGUI(CDisplayWindowGUI&&) = delete;
	CDisplayWindowGUI& operator=(CDisplayWindowGUI&&) = delete;

	/** the pointer is owned by the parent class Screen, no need to delete
	 * it */
	nanogui::Window* m_window = nullptr;

	virtual bool keyboardEvent(
		int key, int scancode, int action, int modifiers) override;
	virtual void drawContents() override;

	/** @name Internal virtual functions to handle GUI events
	 * @{ */
	virtual bool mouseMotionEvent(
		const nanogui::Vector2i& p, const nanogui::Vector2i& rel, int button,
		int modifiers) override;
	virtual bool mouseButtonEvent(
		const nanogui::Vector2i& p, int button, bool down,
		int modifiers) override;
	virtual bool scrollEvent(
		const nanogui::Vector2i& p, const nanogui::Vector2f& rel) override;
	/** @} */

	/** Used to keep track of mouse events on the camera */
	internal::NanoGUICanvasHeadless m_background_canvas;
};

}  // namespace mrpt::gui

#endif  // MRPT_HAS_NANOGUI
