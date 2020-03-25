/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "gui-precomp.h"  // Precompiled headers

#include <mrpt/core/exceptions.h>
#include <mrpt/gui/CDisplayWindowGUI.h>
#include <mrpt/gui/default_mrpt_glfw_icon.h>

using namespace mrpt::gui;

#if MRPT_HAS_NANOGUI

CDisplayWindowGUI::CDisplayWindowGUI(
	const std::string& caption, unsigned int width, unsigned int height,
	const CDisplayWindowGUI_Params& p)
	: nanogui::Screen(
		  Eigen::Vector2i(width, height), caption, p.resizable, p.fullscreen,
		  p.colorBits, p.alphaBits, p.depthBits, p.stencilBits, p.nSamples,
		  p.glMajor, p.glMinor, p.maximized)
{
	// Set MRPT icon:
	GLFWimage images;
	images.width = 64;
	images.height = 64;
	images.pixels = default_mrpt_glfw_icon();

// glfwSetWindowIcon added in glfw 3.2
#if GLFW_VERSION_MAJOR > 3 || \
	(GLFW_VERSION_MAJOR == 3 && GLFW_VERSION_MINOR >= 2)
	glfwSetWindowIcon(screen()->glfwWindow(), 1, &images);
#endif
}

CDisplayWindowGUI::~CDisplayWindowGUI()
{
	// Close window:
	nanogui::Screen::setVisible(false);
}

bool CDisplayWindowGUI::keyboardEvent(
	int key, int scancode, int action, int modifiers)
{
	if (Screen::keyboardEvent(key, scancode, action, modifiers)) return true;
	// Process special key events?
	return false;
}

void CDisplayWindowGUI::drawContents()
{
	// If provided, call the user loop code:
	if (m_loopCallback) m_loopCallback();

	// Optional: render background scene.
	std::lock_guard<std::mutex> lck(background_scene_mtx);
	if (!background_scene) return;

	try
	{
		// We need the size of the viewport:
		GLint win_dims[4];
		glGetIntegerv(GL_VIEWPORT, win_dims);

		// Set the camera params in the scene:
		mrpt::opengl::COpenGLViewport::Ptr view =
			background_scene->getViewport("main");
		if (!view)
			THROW_EXCEPTION(
				"Fatal error: there is no 'main' viewport in the 3D scene!");
		mrpt::opengl::CCamera& cam = view->getCamera();
		m_background_canvas.updateCameraParams(cam);

		for (const auto& m_viewport : background_scene->viewports())
			m_viewport->render(
				win_dims[2], win_dims[3], win_dims[0], win_dims[1]);
	}
	catch (const std::exception& e)
	{
		std::cerr << "[CDisplayWindowGUI::drawContents] Exception:\n"
				  << mrpt::exception_to_str(e);
	}
}

void CDisplayWindowGUI::resize(unsigned int width, unsigned int height)
{
	Screen::setSize({width, height});
}

void CDisplayWindowGUI::setPos(int x, int y) { Screen::setPosition({x, y}); }

void CDisplayWindowGUI::setWindowTitle(const std::string& str)
{
	Screen::setCaption(str);
}

bool CDisplayWindowGUI::mouseButtonEvent(
	const nanogui::Vector2i& p, int button, bool down, int modifiers)
{
	if (!Screen::mouseButtonEvent(p, button, down, modifiers))
		m_background_canvas.mouseButtonEvent(p, button, down, modifiers);

	return true;
}

bool CDisplayWindowGUI::mouseMotionEvent(
	const nanogui::Vector2i& p, const nanogui::Vector2i& rel, int button,
	int modifiers)
{
	if (!Screen::mouseMotionEvent(p, rel, button, modifiers))
		m_background_canvas.mouseMotionEvent(p, rel, button, modifiers);

	return true;
}

bool CDisplayWindowGUI::scrollEvent(
	const nanogui::Vector2i& p, const nanogui::Vector2f& rel)
{
	if (!Screen::scrollEvent(p, rel)) m_background_canvas.scrollEvent(p, rel);

	return true;
}

#endif  // MRPT_HAS_NANOGUI
