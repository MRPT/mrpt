/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "gui-precomp.h"   // Precompiled headers


#include <mrpt/gui/CMyGLCanvasBase.h>
#include <mrpt/gui/WxSubsystem.h>
#include <mrpt/utils/CTicTac.h>

#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace std;

float  CMyGLCanvasBase::SENSIBILITY_DEG_PER_PIXEL = 0.1f;

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
    EVT_ENTER_WINDOW( CMyGLCanvasBase::OnEnterWindow )
	EVT_WINDOW_CREATE( CMyGLCanvasBase::OnWindowCreation )
END_EVENT_TABLE()

void CMyGLCanvasBase::OnWindowCreation(wxWindowCreateEvent &ev)
{
	if (!m_gl_context) m_gl_context=new wxGLContext(this);
}

void CMyGLCanvasBase::OnMouseDown(wxMouseEvent& event)
{
	mouseClickX = event.GetX();
	mouseClickY = event.GetY();
	mouseClicked = true;
}
void CMyGLCanvasBase::OnMouseUp(wxMouseEvent& event)
{
	mouseClicked = false;
}

void CMyGLCanvasBase::OnMouseMove(wxMouseEvent& event)
{
	int X = m_mouseLastX = event.GetX();
	int Y = m_mouseLastY = event.GetY();

	// Proxy variables to cache the changes:
	float	cameraPointingX = this->cameraPointingX;
	float	cameraPointingY = this->cameraPointingY;
	float 	cameraPointingZ = this->cameraPointingZ;
	float	cameraZoomDistance = this->cameraZoomDistance;
	float	cameraElevationDeg = this->cameraElevationDeg;
	float 	cameraAzimuthDeg = this->cameraAzimuthDeg;

	if (event.LeftIsDown() )
	{
		if ( event.ShiftDown()  )
		{
			// Zoom:
			cameraZoomDistance *= exp(0.01*(Y - mouseClickY));
			if (cameraZoomDistance<0.01) cameraZoomDistance = 0.01f;

			float	Az = -0.05*(X - mouseClickX);
			float	D = 0.001*cameraZoomDistance;
			cameraPointingZ += D*Az;
		}
		else
		if (event.ControlDown())
		{
			// Rotate camera pointing direction:
			const float dis = max(0.01f,(cameraZoomDistance));
			float	eye_x = cameraPointingX +  dis * cos(DEG2RAD(cameraAzimuthDeg))*cos(DEG2RAD(cameraElevationDeg));
			float	eye_y = cameraPointingY +  dis * sin(DEG2RAD(cameraAzimuthDeg))*cos(DEG2RAD(cameraElevationDeg));
			float	eye_z = cameraPointingZ +  dis * sin(DEG2RAD(cameraElevationDeg));

			float A_AzimuthDeg = -SENSIBILITY_DEG_PER_PIXEL*(X - mouseClickX);
			float A_ElevationDeg = SENSIBILITY_DEG_PER_PIXEL*(Y - mouseClickY);

			// Orbit camera:
			cameraAzimuthDeg += A_AzimuthDeg;
			cameraElevationDeg += A_ElevationDeg;
			if (cameraElevationDeg<-90) cameraElevationDeg = -90;
			if (cameraElevationDeg>90) cameraElevationDeg = 90;

			// Move cameraPointing pos:
			cameraPointingX = eye_x - dis * cos(DEG2RAD(cameraAzimuthDeg))*cos(DEG2RAD(cameraElevationDeg));
			cameraPointingY = eye_y - dis * sin(DEG2RAD(cameraAzimuthDeg))*cos(DEG2RAD(cameraElevationDeg));
			cameraPointingZ = eye_z - dis * sin(DEG2RAD(cameraElevationDeg));
		}
		else
		{
			// Orbit camera:
			cameraAzimuthDeg -= 0.2*(X - mouseClickX);
			cameraElevationDeg += 0.2*(Y - mouseClickY);
			if (cameraElevationDeg<-90) cameraElevationDeg = -90;
			if (cameraElevationDeg>90) cameraElevationDeg = 90;
		}

		mouseClickX = X;
		mouseClickY = Y;

		// Potential user filter:
		OnUserManuallyMovesCamera(cameraPointingX, cameraPointingY, cameraPointingZ, cameraZoomDistance, cameraElevationDeg, cameraAzimuthDeg);

#if wxCHECK_VERSION(2,9,5)
		wxTheApp->SafeYieldFor(NULL,wxEVT_CATEGORY_TIMER);
#endif
		Refresh(false);

	}
	else
	if ( event.RightIsDown() )
	{
		float	Ay = -(X - mouseClickX);
		float	Ax = -(Y - mouseClickY);
		float	D = 0.001*cameraZoomDistance;
		cameraPointingX += D*(Ax*cos(DEG2RAD(cameraAzimuthDeg)) - Ay*sin(DEG2RAD(cameraAzimuthDeg)));
		cameraPointingY += D*(Ax*sin(DEG2RAD(cameraAzimuthDeg)) + Ay*cos(DEG2RAD(cameraAzimuthDeg)));

		mouseClickX = X;
		mouseClickY = Y;

		// Potential user filter:
		OnUserManuallyMovesCamera(cameraPointingX, cameraPointingY, cameraPointingZ, cameraZoomDistance, cameraElevationDeg, cameraAzimuthDeg);

#if wxCHECK_VERSION(2,9,5)
		wxTheApp->SafeYieldFor(NULL,wxEVT_CATEGORY_TIMER);
#endif
		Refresh(false);
	}

	// ensure we have the focus so we get keyboard events:
	this->SetFocus();
}

void CMyGLCanvasBase::OnMouseWheel(wxMouseEvent& event)
{
	float cameraZoomDistance = this->cameraZoomDistance;

	cameraZoomDistance *= 1 - 0.03f*(event.GetWheelRotation()/120.0f);

	// Potential user filter:
	OnUserManuallyMovesCamera(cameraPointingX, cameraPointingY, cameraPointingZ, cameraZoomDistance, cameraElevationDeg, cameraAzimuthDeg);

	Refresh(false);
	// ensure we have the focus so we get keyboard events:
	this->SetFocus();
}

static int WX_GL_ATTR_LIST[] = { WX_GL_DOUBLEBUFFER, WX_GL_RGBA, WX_GL_DEPTH_SIZE, 24,  0 };

CMyGLCanvasBase::CMyGLCanvasBase(wxWindow *parent, wxWindowID id,const wxPoint& pos, const wxSize& size, long style, const wxString& name) :
		wxGLCanvas(parent, id, WX_GL_ATTR_LIST, pos, size, style|wxFULL_REPAINT_ON_RESIZE , name ),
        m_gl_context(NULL),
        m_init(false),
        m_mouseLastX(0),m_mouseLastY(0)
{
    m_openGLScene = COpenGLScene::Create();

    mouseClickX=0;
	mouseClickY=0;
	mouseClicked = false;

    // Initialize variables:
    cameraPointingX 	= 0;
    cameraPointingY 	= 0;
    cameraPointingZ 	= 0;

	cameraIsProjective = true;

	useCameraFromScene	= false;

    cameraZoomDistance	= 40;
	cameraFOV           = 30; 
    cameraElevationDeg 	= 45;
    cameraAzimuthDeg	= 45;

    clearColorR 		= 0.4f;
    clearColorG			= 0.4f;
    clearColorB			= 0.4f;


    Connect(wxID_ANY,wxEVT_LEFT_DOWN,(wxObjectEventFunction)&CMyGLCanvasBase::OnMouseDown);
    Connect(wxID_ANY,wxEVT_RIGHT_DOWN,(wxObjectEventFunction)&CMyGLCanvasBase::OnMouseDown);
    Connect(wxID_ANY,wxEVT_LEFT_UP,(wxObjectEventFunction)&CMyGLCanvasBase::OnMouseUp);
    Connect(wxID_ANY,wxEVT_RIGHT_UP,(wxObjectEventFunction)&CMyGLCanvasBase::OnMouseUp);
    Connect(wxID_ANY,wxEVT_MOTION,(wxObjectEventFunction)&CMyGLCanvasBase::OnMouseMove);
    Connect(wxID_ANY,wxEVT_MOUSEWHEEL,(wxObjectEventFunction)&CMyGLCanvasBase::OnMouseWheel);

	Connect(wxID_ANY,wxEVT_CHAR,(wxObjectEventFunction)&CMyGLCanvasBase::OnChar);
	Connect(wxID_ANY,wxEVT_CHAR_HOOK,(wxObjectEventFunction)&CMyGLCanvasBase::OnChar);

	Connect(wxEVT_CREATE,(wxObjectEventFunction)&CMyGLCanvasBase::OnWindowCreation);

	// JL: There seems to be a problem in MSW we don't receive this event, but
	//      in GTK we do and at the right moment to avoid an X server crash.
#ifdef _WIN32
	wxWindowCreateEvent dum;
	OnWindowCreation(dum);
#endif
}


CMyGLCanvasBase::~CMyGLCanvasBase()
{
	m_openGLScene.clear_unique();
	delete_safe(m_gl_context);
}

void CMyGLCanvasBase::OnChar( wxKeyEvent& event )
{
	OnCharCustom(event);
}

void CMyGLCanvasBase::Render()
{
	CTicTac tictac;
	double	At = 0.1;

    wxPaintDC dc(this);

	if (!m_gl_context) { /*cerr << "[CMyGLCanvasBase::Render] No GL Context!" << endl;*/ return; }
	else SetCurrent(*m_gl_context);

    // Init OpenGL once, but after SetCurrent
    if (!m_init)
    {
        InitGL();
        m_init = true;
    }

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
        glViewport(
            0,
            0,
            (GLsizei)width,
            (GLsizei)height);

        // Set the background color:
        glClearColor(clearColorR,clearColorG,clearColorB,1.0);

        if (m_openGLScene)
        {
			// Set the camera params in the scene:
			if (!useCameraFromScene)
			{
				COpenGLViewportPtr view= m_openGLScene->getViewport("main");
				if (!view)
				{
					THROW_EXCEPTION("Fatal error: there is no 'main' viewport in the 3D scene!");
				}

				mrpt::opengl::CCamera & cam = view->getCamera();
				cam.setPointingAt( cameraPointingX, cameraPointingY, cameraPointingZ );
				cam.setZoomDistance(cameraZoomDistance);
				cam.setAzimuthDegrees( cameraAzimuthDeg );
				cam.setElevationDegrees(cameraElevationDeg);
				cam.setProjectiveModel( cameraIsProjective );
				cam.setProjectiveFOVdeg( cameraFOV );
			}

			// PART 2: Set the MODELVIEW matrix
			// --------------------------------------
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();

			tictac.Tic();

			// PART 3: Draw primitives:
			// --------------------------------------
			m_openGLScene->render();

        } // end if "m_openGLScene!=NULL"

		OnPostRender();

        // Flush & swap buffers to disply new image:
        glFlush();
		SwapBuffers();

		At = tictac.Tac();

        glPopAttrib();
    }
    catch (std::exception &e)
    {
        glPopAttrib();
		const std::string err_msg = std::string("[CMyGLCanvasBase::Render] Exception!: ") +std::string(e.what());
        std::cerr << err_msg;
        OnRenderError( _U(err_msg.c_str()) );
    }
    catch (...)
    {
        glPopAttrib();
        std::cerr << "Runtime error!" << std::endl;
    }

	OnPostRenderSwapBuffers( At, dc );

}

void CMyGLCanvasBase::OnEnterWindow( wxMouseEvent& WXUNUSED(event) )
{
    SetFocus();
}

void CMyGLCanvasBase::OnPaint( wxPaintEvent& WXUNUSED(event) )
{
    Render();
}

void CMyGLCanvasBase::OnSize(wxSizeEvent& event)
{
	if (!m_parent->IsShown()) return;

    // set GL viewport (not called by wxGLCanvas::OnSize on all platforms...)
    int w, h;
    GetClientSize(&w, &h);

	if (this->IsShownOnScreen())
    {
		if (!m_gl_context) { /*cerr << "[CMyGLCanvasBase::Render] No GL Context!" << endl;*/ return; }
		else SetCurrent(*m_gl_context);

        glViewport(0, 0, (GLint) w, (GLint) h);
    }
}

void CMyGLCanvasBase::OnEraseBackground(wxEraseEvent& WXUNUSED(event))
{
    // Do nothing, to avoid flashing.
}

void CMyGLCanvasBase::InitGL()
{
	if (!m_gl_context) { /*cerr << "[CMyGLCanvasBase::Render] No GL Context!" << endl;*/ return; }
	else SetCurrent(*m_gl_context);

    static bool GLUT_INIT_DONE = false;

    if (!GLUT_INIT_DONE)
    {
    	GLUT_INIT_DONE = true;

		int argc=1;
		char *argv[1] = { NULL };
		glutInit( &argc, argv );
    }
}

void CMyGLCanvasBase::setCameraPose(const mrpt::poses::CPose3D &camPose)
{
	THROW_EXCEPTION("todo")
}

#endif // MRPT_HAS_WXWIDGETS

