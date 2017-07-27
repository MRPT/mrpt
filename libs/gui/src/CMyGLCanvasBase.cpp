/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "gui-precomp.h"  // Precompiled headers

#include <mrpt/gui/CMyGLCanvasBase.h>
#include <mrpt/gui/WxSubsystem.h>
#include <mrpt/utils/CTicTac.h>

#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace std;

#if MRPT_HAS_OPENGL_GLUT
#ifdef MRPT_OS_WINDOWS
// Windows:
#include <windows.h>
#endif

#ifdef MRPT_OS_APPLE
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
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

/*----------------------------------------------------------------
  Implementation of Test-GLCanvas
-----------------------------------------------------------------*/

BEGIN_EVENT_TABLE(CMyGLCanvasBase, wxGLCanvas)
EVT_SIZE(CMyGLCanvasBase::OnSize)
EVT_PAINT(CMyGLCanvasBase::OnPaint)
EVT_ERASE_BACKGROUND(CMyGLCanvasBase::OnEraseBackground)
EVT_ENTER_WINDOW(CMyGLCanvasBase::OnEnterWindow)
EVT_WINDOW_CREATE(CMyGLCanvasBase::OnWindowCreation)
END_EVENT_TABLE()

void CMyGLCanvasBase::OnWindowCreation(wxWindowCreateEvent& ev)
{
	if (!m_gl_context) m_gl_context = new wxGLContext(this);
}

void CMyGLCanvasBase::swapBuffers() { SwapBuffers(); }
void CMyGLCanvasBase::preRender() { OnPreRender(); }
void CMyGLCanvasBase::postRender() { OnPostRender(); }
void CMyGLCanvasBase::renderError(const string& err_msg)
{
	OnRenderError(_U(err_msg.c_str()));
}

void CMyGLCanvasBase::OnMouseDown(wxMouseEvent& event)
{
	setMousePos(event.GetX(), event.GetY());
	setMouseClicked(true);
}
void CMyGLCanvasBase::OnMouseUp(wxMouseEvent& /*event*/)
{
	setMouseClicked(false);
}

void CMyGLCanvasBase::OnMouseMove(wxMouseEvent& event)
{
	bool leftIsDown = event.LeftIsDown();

	if (leftIsDown || event.RightIsDown())
	{
		int X = event.GetX();
		int Y = event.GetY();
		updateLastPos(X, Y);

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

void CMyGLCanvasBase::OnMouseWheel(wxMouseEvent& event)
{
	CamaraParams params = cameraParams();
	updateZoom(params, event.GetWheelRotation());
	setCameraParams(params);

	Refresh(false);
	// ensure we have the focus so we get keyboard events:
	this->SetFocus();
}

static int WX_GL_ATTR_LIST[] = {WX_GL_DOUBLEBUFFER, WX_GL_RGBA,
								WX_GL_DEPTH_SIZE, 24, 0};

CMyGLCanvasBase::CMyGLCanvasBase(
	wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size,
	long style, const wxString& name)
	: CGlCanvasBase(),
	  wxGLCanvas(
		  parent, id, WX_GL_ATTR_LIST, pos, size,
		  style | wxFULL_REPAINT_ON_RESIZE, name),
	  m_gl_context(nullptr),
	  m_init(false)
{
	Connect(
		wxID_ANY, wxEVT_LEFT_DOWN,
		(wxObjectEventFunction)&CMyGLCanvasBase::OnMouseDown);
	Connect(
		wxID_ANY, wxEVT_RIGHT_DOWN,
		(wxObjectEventFunction)&CMyGLCanvasBase::OnMouseDown);
	Connect(
		wxID_ANY, wxEVT_LEFT_UP,
		(wxObjectEventFunction)&CMyGLCanvasBase::OnMouseUp);
	Connect(
		wxID_ANY, wxEVT_RIGHT_UP,
		(wxObjectEventFunction)&CMyGLCanvasBase::OnMouseUp);
	Connect(
		wxID_ANY, wxEVT_MOTION,
		(wxObjectEventFunction)&CMyGLCanvasBase::OnMouseMove);
	Connect(
		wxID_ANY, wxEVT_MOUSEWHEEL,
		(wxObjectEventFunction)&CMyGLCanvasBase::OnMouseWheel);

	Connect(
		wxID_ANY, wxEVT_CHAR, (wxObjectEventFunction)&CMyGLCanvasBase::OnChar);
	Connect(
		wxID_ANY, wxEVT_CHAR_HOOK,
		(wxObjectEventFunction)&CMyGLCanvasBase::OnChar);

	Connect(
		wxEVT_CREATE,
		(wxObjectEventFunction)&CMyGLCanvasBase::OnWindowCreation);

// JL: There seems to be a problem in MSW we don't receive this event, but
//      in GTK we do and at the right moment to avoid an X server crash.
#ifdef _WIN32
	wxWindowCreateEvent dum;
	OnWindowCreation(dum);
#endif
}

CMyGLCanvasBase::~CMyGLCanvasBase() { delete_safe(m_gl_context); }
void CMyGLCanvasBase::OnChar(wxKeyEvent& event) { OnCharCustom(event); }
void CMyGLCanvasBase::Render()
{
	wxPaintDC dc(this);

	if (!m_gl_context)
	{ /*cerr << "[CMyGLCanvasBase::Render] No GL Context!" << endl;*/
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

void CMyGLCanvasBase::OnEnterWindow(wxMouseEvent& WXUNUSED(event))
{
	SetFocus();
}

void CMyGLCanvasBase::OnPaint(wxPaintEvent& WXUNUSED(event)) { Render(); }
void CMyGLCanvasBase::OnSize(wxSizeEvent& event)
{
	if (!m_parent->IsShown()) return;

	// set GL viewport (not called by wxGLCanvas::OnSize on all platforms...)
	int w, h;
	GetClientSize(&w, &h);

	if (this->IsShownOnScreen())
	{
		if (!m_gl_context)
		{ /*cerr << "[CMyGLCanvasBase::Render] No GL Context!" << endl;*/
			return;
		}
		else
			SetCurrent(*m_gl_context);

		resizeViewport(w, h);
	}
}

void CMyGLCanvasBase::OnEraseBackground(wxEraseEvent& WXUNUSED(event))
{
	// Do nothing, to avoid flashing.
}

void CMyGLCanvasBase::InitGL()
{
	if (!m_gl_context)
	{ /*cerr << "[CMyGLCanvasBase::Render] No GL Context!" << endl;*/
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

void CMyGLCanvasBase::setCameraPose(const mrpt::poses::CPose3D& camPose)
{
	THROW_EXCEPTION("todo")
}

#endif  // MRPT_HAS_WXWIDGETS
