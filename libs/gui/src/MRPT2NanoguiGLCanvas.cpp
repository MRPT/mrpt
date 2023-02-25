/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "gui-precomp.h"  // Precompiled headers
//
#include <mrpt/core/exceptions.h>
#include <mrpt/gui/MRPT2NanoguiGLCanvas.h>

using namespace mrpt::gui;

#if MRPT_HAS_NANOGUI

MRPT2NanoguiGLCanvas::MRPT2NanoguiGLCanvas(nanogui::Widget* parent)
	: nanogui::GLCanvas(parent)
{
}

MRPT2NanoguiGLCanvas::~MRPT2NanoguiGLCanvas() = default;

void MRPT2NanoguiGLCanvas::drawGL()
{
#if MRPT_HAS_OPENGL_GLUT || MRPT_HAS_EGL
	std::lock_guard<std::mutex> lck(scene_mtx);

	try
	{
		// Set the background color:
		mrpt::img::TColorf m_backgroundColor = {0.7f, 0.7f, 0.7f, 1.0f};
		glClearColor(
			m_backgroundColor.R, m_backgroundColor.G, m_backgroundColor.B,
			m_backgroundColor.A);

		if (!scene) return;	 // No scene -> nothing to render

		// We need the size of the viewport:
		GLint win_dims[4];
		glGetIntegerv(GL_VIEWPORT, win_dims);

		// Set the camera params in the scene:
		mrpt::opengl::Viewport::Ptr view = scene->getViewport("main");
		if (!view)
			THROW_EXCEPTION(
				"Fatal error: there is no 'main' viewport in the 3D scene!");
		mrpt::opengl::CCamera& cam = view->getCamera();
		m_headless_canvas.updateCameraParams(cam);

		for (const auto& m_viewport : scene->viewports())
			m_viewport->render(
				win_dims[2], win_dims[3], win_dims[0], win_dims[1]);
	}
	catch (const std::exception& e)
	{
		std::cerr << "[MRPT2NanoguiGLCanvas::drawGL] Exception:\n"
				  << mrpt::exception_to_str(e);
	}
#endif
}

bool MRPT2NanoguiGLCanvas::mouseMotionEvent(
	const nanogui::Vector2i& p, const nanogui::Vector2i& rel, int button,
	int modifiers)
{
	m_headless_canvas.mouseMotionEvent(p, rel, button, modifiers);
	return true;  // already processed
}

bool MRPT2NanoguiGLCanvas::mouseButtonEvent(
	const nanogui::Vector2i& p, int button, bool down, int modifiers)
{
	m_headless_canvas.mouseButtonEvent(p, button, down, modifiers);
	return true;  // already processed
}
bool MRPT2NanoguiGLCanvas::scrollEvent(
	const nanogui::Vector2i& p, const nanogui::Vector2f& rel)
{
	m_headless_canvas.scrollEvent(p, rel);
	return true;  // already processed
}

#endif	// MRPT_HAS_NANOGUI
