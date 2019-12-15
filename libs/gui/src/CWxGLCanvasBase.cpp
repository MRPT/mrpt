/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "gui-precomp.h"  // Precompiled headers

#include <mrpt/gui/CWxGLCanvasBase.h>
#include <mrpt/gui/WxSubsystem.h>
#include <mrpt/system/CTicTac.h>

#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT

#if MRPT_HAS_OPENGL_GLUT
#ifdef _WIN32
// Windows:
#include <windows.h>
#endif

#ifdef __APPLE__
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#ifdef HAVE_FREEGLUT_EXT_H
#include <GL/freeglut_ext.h>
#endif
#endif

#endif

#if !wxUSE_GLCANVAS
#error "OpenGL required: set wxUSE_GLCANVAS to 1 and rebuild wxWidgets"
#endif

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace std;

/*----------------------------------------------------------------
  Implementation of Test-GLCanvas
-----------------------------------------------------------------*/

void CWxGLCanvasBase::OnWindowCreation(wxWindowCreateEvent& ev)
{
	if (!m_gl_context) m_gl_context = std::make_unique<wxGLContext>(this);
}

void CWxGLCanvasBase::swapBuffers() { SwapBuffers(); }
void CWxGLCanvasBase::preRender() { OnPreRender(); }
void CWxGLCanvasBase::postRender() { OnPostRender(); }
void CWxGLCanvasBase::renderError(const string& err_msg)
{
	OnRenderError(err_msg.c_str());
}

void CWxGLCanvasBase::OnMouseDown(wxMouseEvent& event)
{
	setMousePos(event.GetX(), event.GetY());
	setMouseClicked(true);
}
void CWxGLCanvasBase::OnMouseUp(wxMouseEvent& /*event*/)
{
	setMouseClicked(false);
}

void CWxGLCanvasBase::OnMouseMove(wxMouseEvent& event)
{
	bool leftIsDown = event.LeftIsDown();

	int X = event.GetX();
	int Y = event.GetY();
	updateLastPos(X, Y);

	if (leftIsDown || event.RightIsDown())
	{
		// Proxy variables to cache the changes:
		CamaraParams params = cameraParams();

		if (leftIsDown)
		{
			if (event.ShiftDown())
				updateZoom(params, X, Y);

			else if (event.ControlDown())
				updateRotate(params, X, Y);

			else
				updateOrbitCamera(params, X, Y);
		}
		else
			updatePan(params, X, Y);

		setMousePos(X, Y);
		setCameraParams(params);

#if wxCHECK_VERSION(2, 9, 5)
		wxTheApp->SafeYieldFor(nullptr, wxEVT_CATEGORY_TIMER);
#endif
		Refresh(false);
	}

	// ensure we have the focus so we get keyboard events:
	this->SetFocus();
}

void CWxGLCanvasBase::OnMouseWheel(wxMouseEvent& event)
{
	CamaraParams params = cameraParams();
	updateZoom(params, event.GetWheelRotation());
	setCameraParams(params);

	Refresh(false);
	// ensure we have the focus so we get keyboard events:
	this->SetFocus();
}

// clang-format off
static int WX_GL_ATTR_LIST[] = {
	WX_GL_DOUBLEBUFFER,    WX_GL_RGBA,
	WX_GL_DEPTH_SIZE,     24,
	WX_GL_MAJOR_VERSION,  3,
	WX_GL_MINOR_VERSION,  1,
	WX_GL_CORE_PROFILE, // do not allow using opengl 1.x deprecated stuff
	0
};
// clang-format on

CWxGLCanvasBase::CWxGLCanvasBase(
	wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size,
	long style, const wxString& name)
	: CGlCanvasBase(),
	  wxGLCanvas(
		  parent, id, WX_GL_ATTR_LIST, pos, size,
		  style | wxFULL_REPAINT_ON_RESIZE, name)
{
	this->Bind(wxEVT_LEFT_DOWN, &CWxGLCanvasBase::OnMouseDown, this);
	this->Bind(wxEVT_RIGHT_DOWN, &CWxGLCanvasBase::OnMouseDown, this);
	this->Bind(wxEVT_LEFT_UP, &CWxGLCanvasBase::OnMouseUp, this);
	this->Bind(wxEVT_RIGHT_UP, &CWxGLCanvasBase::OnMouseUp, this);
	this->Bind(wxEVT_MOTION, &CWxGLCanvasBase::OnMouseMove, this);
	this->Bind(wxEVT_MOUSEWHEEL, &CWxGLCanvasBase::OnMouseWheel, this);
	this->Bind(wxEVT_CHAR, &CWxGLCanvasBase::OnChar, this);
	this->Bind(wxEVT_CHAR_HOOK, &CWxGLCanvasBase::OnChar, this);
	this->Bind(wxEVT_CREATE, &CWxGLCanvasBase::OnWindowCreation, this);

	this->Bind(wxEVT_SIZE, &CWxGLCanvasBase::OnSize, this);
	this->Bind(wxEVT_PAINT, &CWxGLCanvasBase::OnPaint, this);
	this->Bind(
		wxEVT_ERASE_BACKGROUND, &CWxGLCanvasBase::OnEraseBackground, this);
	this->Bind(wxEVT_ENTER_WINDOW, &CWxGLCanvasBase::OnEnterWindow, this);

// JL: There seems to be a problem in MSW we don't receive this event, but
//      in GTK we do and at the right moment to avoid an X server crash.
#ifdef _WIN32
	wxWindowCreateEvent dum;
	OnWindowCreation(dum);
#endif
}

void CWxGLCanvasBase::OnChar(wxKeyEvent& event) { OnCharCustom(event); }
void CWxGLCanvasBase::Render()
{
	wxPaintDC dc(this);

	if (!m_gl_context)
	{ /*cerr << "[CWxGLCanvasBase::Render] No GL Context!" << endl;*/
		return;
	}
	else
		SetCurrent(*m_gl_context);

	// Init OpenGL once, but after SetCurrent
	if (!m_init)
	{
		InitGL();
		m_init = true;
	}

	int width, height;
	GetClientSize(&width, &height);
	double At = renderCanvas(width, height);

	OnPostRenderSwapBuffers(At, dc);
}

void CWxGLCanvasBase::OnEnterWindow(wxMouseEvent& WXUNUSED(event))
{
	SetFocus();
}

void CWxGLCanvasBase::OnPaint(wxPaintEvent& WXUNUSED(event)) { Render(); }
void CWxGLCanvasBase::OnSize(wxSizeEvent& event)
{
	if (!m_parent->IsShown()) return;

	// set GL viewport (not called by wxGLCanvas::OnSize on all platforms...)
	int w, h;
	GetClientSize(&w, &h);

	if (this->IsShownOnScreen())
	{
		if (!m_gl_context)
		{ /*cerr << "[CWxGLCanvasBase::Render] No GL Context!" << endl;*/
			return;
		}
		else
			SetCurrent(*m_gl_context);

		resizeViewport(w, h);
	}
}

void CWxGLCanvasBase::OnEraseBackground(wxEraseEvent& WXUNUSED(event))
{
	// Do nothing, to avoid flashing.
}

void CWxGLCanvasBase::InitGL()
{
	if (!m_gl_context)
	{ /*cerr << "[CWxGLCanvasBase::Render] No GL Context!" << endl;*/
		return;
	}
	else
		SetCurrent(*m_gl_context);

	static bool GLUT_INIT_DONE = false;

	if (!GLUT_INIT_DONE)
	{
		GLUT_INIT_DONE = true;

		int argc = 1;
		char* argv[1] = {nullptr};
		glutInit(&argc, argv);
	}
}

void CWxGLCanvasBase::setCameraPose(const mrpt::poses::CPose3D& camPose)
{
	THROW_EXCEPTION("todo");
}

#endif  // MRPT_HAS_WXWIDGETS
