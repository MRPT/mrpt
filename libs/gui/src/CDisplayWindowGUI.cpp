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
	std::lock_guard<std::mutex> lck(scene_mtx);
	if (!scene) return;  // No scene -> nothing to render

	try
	{
		glPushAttrib(GL_ALL_ATTRIB_BITS);

		// Set the camera params in the scene:
		auto view = scene->getViewport("main");
		ASSERTMSG_(view, "No 'main' viewport in the 3D scene!");

		mrpt::opengl::CCamera& cam = view->getCamera();
		// updateCameraParams(cam);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		scene->render();

		// m_text_msgs.render_text_messages_public(w, h);

		glPopAttrib();
	}
	catch (const std::exception& e)
	{
		glPopAttrib();
		std::cerr << mrpt::exception_to_str(e);
	}
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
