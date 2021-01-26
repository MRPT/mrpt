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
#include <mrpt/gui/internal/NanoGUICanvasHeadless.h>

#include <mutex>

// Expose nanogui API to mrpt users, for direct use of nanogui classes.
#include <mrpt/config.h>
#if MRPT_HAS_NANOGUI

#include <nanogui/nanogui.h>

namespace mrpt::gui
{
/** An extension of nanogui::GLCanvas to render MRPT OpenGL scenes.
 *
 * Directly access `scene` (locking its mutex `scene_mtx` first) to update the
 * scene to be rendered.
 *
 * \sa CDisplayWindowGUI
 * \ingroup mrpt_gui_grp
 */
class MRPT2NanoguiGLCanvas : public nanogui::GLCanvas
{
   public:
	MRPT2NanoguiGLCanvas(nanogui::Widget* parent);
	~MRPT2NanoguiGLCanvas();

	virtual void drawGL() override;

	/** The scene to render in this control.
	 * \note Users must lock the mutex scene_mtx while modifying this variable.
	 */
	mrpt::opengl::COpenGLScene::Ptr scene;
	std::mutex scene_mtx;

	CGlCanvasBase& camera() { return m_headless_canvas; }
	const CGlCanvasBase& camera() const { return m_headless_canvas; }

   protected:
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
	internal::NanoGUICanvasHeadless m_headless_canvas;
};

}  // namespace mrpt::gui

#endif	// MRPT_HAS_NANOGUI
