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
	if (!m_gl_context) m_gl_context.reset(new wxGLContext(this));
}

void CMyGLCanvasBase::swapBuffers() { SwapBuffers(); }
<<<<<<< HEAD
void CMyGLCanvasBase::OnMouseUp(wxMouseEvent& event) { mouseClicked = false; }
void CMyGLCanvasBase::OnMouseMove(wxMouseEvent& event)
=======
void CMyGLCanvasBase::preRender() { OnPreRender(); }
void CMyGLCanvasBase::postRender()
>>>>>>> 9f6c1fc0f7746eb917e8960e450538e0cede9540
{
	OnPostRender();
}

void CMyGLCanvasBase::renderError(const string& err_msg)
{
	OnRenderError(_U(err_msg.c_str()));
}
<<<<<<< HEAD
else if (event.ControlDown())
{
	// Rotate camera pointing direction:
	const float dis = max(0.01f, (cameraZoomDistance));
	float eye_x =
		cameraPointingX +
		dis * cos(DEG2RAD(cameraAzimuthDeg)) * cos(DEG2RAD(cameraElevationDeg));
	float eye_y =
		cameraPointingY +
		dis * sin(DEG2RAD(cameraAzimuthDeg)) * cos(DEG2RAD(cameraElevationDeg));
	float eye_z = cameraPointingZ + dis * sin(DEG2RAD(cameraElevationDeg));

	float A_AzimuthDeg = -SENSIBILITY_DEG_PER_PIXEL * (X - mouseClickX);
	float A_ElevationDeg = SENSIBILITY_DEG_PER_PIXEL * (Y - mouseClickY);

	// Orbit camera:
	cameraAzimuthDeg += A_AzimuthDeg;
	cameraElevationDeg += A_ElevationDeg;
	if (cameraElevationDeg < -90) cameraElevationDeg = -90;
	if (cameraElevationDeg > 90) cameraElevationDeg = 90;

	// Move cameraPointing pos:
	cameraPointingX =
		eye_x -
		dis * cos(DEG2RAD(cameraAzimuthDeg)) * cos(DEG2RAD(cameraElevationDeg));
	cameraPointingY =
		eye_y -
		dis * sin(DEG2RAD(cameraAzimuthDeg)) * cos(DEG2RAD(cameraElevationDeg));
	cameraPointingZ = eye_z - dis * sin(DEG2RAD(cameraElevationDeg));
=======

void CMyGLCanvasBase::OnMouseDown(wxMouseEvent& event)
{
	setMousePos(event.GetX(), event.GetY());
	setMouseClicked(true);
>>>>>>> 9f6c1fc0f7746eb917e8960e450538e0cede9540
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

<<<<<<< HEAD
		// Potential user filter:
		OnUserManuallyMovesCamera(
			cameraPointingX, cameraPointingY, cameraPointingZ,
			cameraZoomDistance, cameraElevationDeg, cameraAzimuthDeg);

#if wxCHECK_VERSION(2, 9, 5)
		wxTheApp->SafeYieldFor(nullptr, wxEVT_CATEGORY_TIMER);
#endif
		Refresh(false);
	}
	else if (event.RightIsDown())
	{
		float Ay = -(X - mouseClickX);
		float Ax = -(Y - mouseClickY);
		float D = 0.001 * cameraZoomDistance;
		cameraPointingX += D * (Ax * cos(DEG2RAD(cameraAzimuthDeg)) -
								Ay * sin(DEG2RAD(cameraAzimuthDeg)));
		cameraPointingY += D * (Ax * sin(DEG2RAD(cameraAzimuthDeg)) +
								Ay * cos(DEG2RAD(cameraAzimuthDeg)));
=======
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
>>>>>>> 9f6c1fc0f7746eb917e8960e450538e0cede9540


<<<<<<< HEAD
		// Potential user filter:
		OnUserManuallyMovesCamera(
			cameraPointingX, cameraPointingY, cameraPointingZ,
			cameraZoomDistance, cameraElevationDeg, cameraAzimuthDeg);
=======
		setMousePos(X, Y);
		setCameraParams(params);
>>>>>>> 9f6c1fc0f7746eb917e8960e450538e0cede9540

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
<<<<<<< HEAD
	float cameraZoomDistance = this->cameraZoomDistance;

	cameraZoomDistance *= 1 - 0.03f * (event.GetWheelRotation() / 120.0f);

	// Potential user filter:
	OnUserManuallyMovesCamera(
		cameraPointingX, cameraPointingY, cameraPointingZ, cameraZoomDistance,
		cameraElevationDeg, cameraAzimuthDeg);
=======
	CamaraParams params = cameraParams();
	updateZoom(params, event.GetWheelRotation());
	setCameraParams(params);
>>>>>>> 9f6c1fc0f7746eb917e8960e450538e0cede9540

	Refresh(false);
	// ensure we have the focus so we get keyboard events:
	this->SetFocus();
}

static int WX_GL_ATTR_LIST[] = {WX_GL_DOUBLEBUFFER, WX_GL_RGBA,
								WX_GL_DEPTH_SIZE, 24, 0};

<<<<<<< HEAD
CMyGLCanvasBase::CMyGLCanvasBase(
	wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size,
	long style, const wxString& name)
	: wxGLCanvas(
		  parent, id, WX_GL_ATTR_LIST, pos, size,
		  style | wxFULL_REPAINT_ON_RESIZE, name),
	  m_init(false),
	  m_mouseLastX(0),
	  m_mouseLastY(0)
{
	m_openGLScene = mrpt::make_aligned_shared<COpenGLScene>();

	mouseClickX = 0;
	mouseClickY = 0;
	mouseClicked = false;

	// Initialize variables:
	cameraPointingX = 0;
	cameraPointingY = 0;
	cameraPointingZ = 0;

	cameraIsProjective = true;

	useCameraFromScene = false;

	cameraZoomDistance = 40;
	cameraFOV = 30;
	cameraElevationDeg = 45;
	cameraAzimuthDeg = 45;

	clearColorR = 0.4f;
	clearColorG = 0.4f;
	clearColorB = 0.4f;

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
=======
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
>>>>>>> 9f6c1fc0f7746eb917e8960e450538e0cede9540

// JL: There seems to be a problem in MSW we don't receive this event, but
//      in GTK we do and at the right moment to avoid an X server crash.
#ifdef _WIN32
	wxWindowCreateEvent dum;
	OnWindowCreation(dum);
#endif
}

<<<<<<< HEAD
CMyGLCanvasBase::~CMyGLCanvasBase() {}
void CMyGLCanvasBase::OnChar(wxKeyEvent& event) { OnCharCustom(event); }
=======

CMyGLCanvasBase::~CMyGLCanvasBase() { delete_safe(m_gl_context); }
void CMyGLCanvasBase::OnChar(wxKeyEvent& event) { OnCharCustom(event); }

>>>>>>> 9f6c1fc0f7746eb917e8960e450538e0cede9540
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

<<<<<<< HEAD
	try
	{
		// Call PreRender user code:
		OnPreRender();

		glPushAttrib(GL_ALL_ATTRIB_BITS);

		// Set static configs:
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_ALPHA_TEST);
		glEnable(GL_TEXTURE_2D);

		// PART 1a: Set the viewport
		// --------------------------------------
		int width, height;
		GetClientSize(&width, &height);
		glViewport(0, 0, (GLsizei)width, (GLsizei)height);

		// Set the background color:
		glClearColor(clearColorR, clearColorG, clearColorB, 1.0);

		if (m_openGLScene)
		{
			// Set the camera params in the scene:
			if (!useCameraFromScene)
			{
				COpenGLViewport::Ptr view = m_openGLScene->getViewport("main");
				if (!view)
				{
					THROW_EXCEPTION(
						"Fatal error: there is no 'main' viewport in the 3D "
						"scene!");
				}

				mrpt::opengl::CCamera& cam = view->getCamera();
				cam.setPointingAt(
					cameraPointingX, cameraPointingY, cameraPointingZ);
				cam.setZoomDistance(cameraZoomDistance);
				cam.setAzimuthDegrees(cameraAzimuthDeg);
				cam.setElevationDegrees(cameraElevationDeg);
				cam.setProjectiveModel(cameraIsProjective);
				cam.setProjectiveFOVdeg(cameraFOV);
			}

			// PART 2: Set the MODELVIEW matrix
			// --------------------------------------
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();

			tictac.Tic();

			// PART 3: Draw primitives:
			// --------------------------------------
			m_openGLScene->render();

		}  // end if "m_openGLScene!=nullptr"

		OnPostRender();

		// Flush & swap buffers to disply new image:
		glFlush();
		SwapBuffers();

		At = tictac.Tac();

		glPopAttrib();
	}
	catch (std::exception& e)
	{
		glPopAttrib();
		const std::string err_msg =
			std::string("[CMyGLCanvasBase::Render] Exception!: ") +
			std::string(e.what());
		std::cerr << err_msg;
		OnRenderError(_U(err_msg.c_str()));
	}
	catch (...)
	{
		glPopAttrib();
		std::cerr << "Runtime error!" << std::endl;
	}
=======
	int width, height;
	GetClientSize(&width, &height);
	double At = renderCanvas(width, height);
>>>>>>> 9f6c1fc0f7746eb917e8960e450538e0cede9540

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
