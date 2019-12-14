/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/opengl/COpenGLScene.h>
#include <mutex>
#include <string>

// Expose nanogui API to mrpt users, for direct use of nanogui classes.
#include <mrpt/config.h>
#if MRPT_HAS_NANOGUI
#include <nanogui/nanogui.h>

namespace mrpt::gui
{
/** An extension of nanogui::GLCanvas to render MRPT OpenGL scenes.
 * \sa CDisplayWindowGUI
 * \ingroup mrpt_gui_grp
 */
class MRPTGLCanvas : public nanogui::GLCanvas
{
   public:
	MRPTGLCanvas(nanogui::Widget* parent);
	~MRPTGLCanvas();

	virtual void drawGL() override;

	/** The scene to render in this control.
	 * \note Users must lock the mutex scene_mtx while modifying this variable.
	 */
	mrpt::opengl::COpenGLScene::Ptr scene;
	std::mutex scene_mtx;
};

/** A window with powerful GUI capabilities, via the nanogui library.
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
	using Ptr = std::shared_ptr<CDisplayWindowGUI>;
	using ConstPtr = std::shared_ptr<const CDisplayWindowGUI>;

	CDisplayWindowGUI(
		const std::string& caption = std::string(), unsigned int width = 400,
		unsigned int height = 300, bool resizable = true,
		bool fullscreen = false);

	~CDisplayWindowGUI();

	/** Class factory returning a smart pointer */
	template <typename... Args>
	static Ptr Create(Args&&... args)
	{
		return std::make_shared<CDisplayWindowGUI>(std::forward<Args>(args)...);
	}

	virtual bool keyboardEvent(
		int key, int scancode, int action, int modifiers) override;
	virtual void draw(NVGcontext* ctx) override;

	/** Resizes the window */
	void resize(unsigned int width, unsigned int height);

	/** Changes the position of the window on the screen. */
	void setPos(int x, int y);

	/**  Changes the window title. */
	void setWindowTitle(const std::string& str);

	nanogui::Window* nanogui_win()
	{
		ASSERT_(m_window);
		return m_window;
	}

   private:
	CDisplayWindowGUI(const CDisplayWindowGUI&) = delete;
	CDisplayWindowGUI& operator=(const CDisplayWindowGUI&) = delete;

	CDisplayWindowGUI(CDisplayWindowGUI&&) = delete;
	CDisplayWindowGUI& operator=(CDisplayWindowGUI&&) = delete;

	/** the pointer is owned by the parent class Screen, no need to delete
	 * it */
	nanogui::Window* m_window;
};

}  // namespace mrpt::gui

#endif  // MRPT_HAS_NANOGUI
