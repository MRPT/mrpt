/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "gui-precomp.h"  // Precompiled headers

#include <mrpt/core/exceptions.h>
#include <mrpt/gui/CDisplayWindowGUI.h>

using namespace mrpt::gui;

#if MRPT_HAS_NANOGUI

// -------- MRPTGLCanvas -----------
MRPTGLCanvas::MRPTGLCanvas(nanogui::Widget* parent) : nanogui::GLCanvas(parent)
{
}

MRPTGLCanvas::~MRPTGLCanvas() = default;

void MRPTGLCanvas::drawGL()
{
#if MRPT_HAS_OPENGL_GLUT
	std::lock_guard<std::mutex> lck(scene_mtx);

	try
	{
		// Set the viewport
		// resizeViewport((GLsizei)width, (GLsizei)height);

		// Set the background color:
		mrpt::img::TColorf m_backgroundColor;
		glClearColor(
			m_backgroundColor.R, m_backgroundColor.G, m_backgroundColor.B,
			m_backgroundColor.A);

		if (!scene) return;  // No scene -> nothing to render

		// We need the size of the viewport at the beginning: should be the
		// whole window:
		GLint win_dims[4];
		glGetIntegerv(GL_VIEWPORT, win_dims);
		// CHECK_OPENGL_ERROR();

#if 0
		// Set the camera params in the scene:
		mrpt::opengl::COpenGLViewport::Ptr view =
			m_openGLScene->getViewport("main");
		if (!view)
			THROW_EXCEPTION(
				"Fatal error: there is no 'main' viewport in the 3D scene!");
		mrpt::opengl::CCamera& cam = view->getCamera();
		updateCameraParams(cam);
#endif

		for (const auto& m_viewport : scene->viewports())
			m_viewport->render(
				win_dims[2], win_dims[3], win_dims[0], win_dims[1]);

		//	CHECK_OPENGL_ERROR();
	}
	catch (const std::exception& e)
	{
		std::cerr << "[MRPTGLCanvas::drawGL] Exception:\n"
				  << mrpt::exception_to_str(e);
	}
#endif
}

// -------- CDisplayWindowGUI -----------

CDisplayWindowGUI::CDisplayWindowGUI(
	const std::string& caption, unsigned int width, unsigned int height,
	bool resizable, bool fullscreen)
	: nanogui::Screen(
		  Eigen::Vector2i(width, height), caption, resizable, fullscreen)
{
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

void CDisplayWindowGUI::draw(NVGcontext* ctx)
{
	// Draw the user interface
	Screen::draw(ctx);
}

void CDisplayWindowGUI::resize(unsigned int width, unsigned int height) {}

void CDisplayWindowGUI::setPos(int x, int y) {}

void CDisplayWindowGUI::setWindowTitle(const std::string& str) {}

#endif  // MRPT_HAS_NANOGUI
