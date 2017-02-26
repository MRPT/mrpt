/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "gui-precomp.h"   // Precompiled headers

#include <mrpt/config.h>

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/CTicTac.h>

#include <mrpt/gui/WxSubsystem.h>
#include <mrpt/gui/WxUtils.h>

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;


IMPLEMENTS_MRPT_OBJECT(CDisplayWindow3D,CBaseGUIWindow,mrpt::gui)

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

#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT

#if !wxUSE_GLCANVAS
#error "OpenGL required: set wxUSE_GLCANVAS to 1 and rebuild wxWidgets"
#endif

#include <mrpt/gui/CMyGLCanvasBase.h>
#include <mrpt/opengl/CTextMessageCapable.h>

namespace mrpt
{
	namespace gui
	{
		class CMyGLCanvas_DisplayWindow3D : public mrpt::gui::CMyGLCanvasBase
		{
		public:
			CMyGLCanvas_DisplayWindow3D( CDisplayWindow3D *win3D,
						 wxWindow *parent, wxWindowID id = wxID_ANY,
						 const wxPoint& pos = wxDefaultPosition,
						 const wxSize& size = wxDefaultSize,
						 long style = 0, const wxString& name = _T("CMyGLCanvas_DisplayWindow3D") );

			virtual ~CMyGLCanvas_DisplayWindow3D();


			CDisplayWindow3D *m_win3D;

			// The idea is that CMyGLCanvas_DisplayWindow3D was derived from CTextMessageCapable, but
			//  that raises errors in MSVC when converting method pointers to wxObjectEventFunction...
			struct THubClass : public mrpt::opengl::CTextMessageCapable
			{
				void render_text_messages_public(const int w, const int h) const { render_text_messages(w,h); }
			};
			THubClass m_text_msgs;

			void OnCharCustom( wxKeyEvent& event );
			void OnMouseDown(wxMouseEvent& event);

			void OnPreRender();
			void OnPostRender();
			void OnPostRenderSwapBuffers(double At, wxPaintDC &dc);

			static void display3D_processKeyEvent(CDisplayWindow3D *m_win3D, wxKeyEvent&ev);
		};
	}
}

CMyGLCanvas_DisplayWindow3D::CMyGLCanvas_DisplayWindow3D(
    CDisplayWindow3D *win3D,
    wxWindow *parent, wxWindowID id,
    const wxPoint& pos, const wxSize& size, long style, const wxString& name)
    : CMyGLCanvasBase(parent,id,pos,size,style,name)
{
	m_win3D = win3D;
	Connect(wxEVT_CHAR,(wxObjectEventFunction)&CMyGLCanvas_DisplayWindow3D::OnCharCustom);
	Connect(wxEVT_CHAR_HOOK,(wxObjectEventFunction)&CMyGLCanvas_DisplayWindow3D::OnCharCustom);

	Connect(wxEVT_LEFT_DOWN,(wxObjectEventFunction)&CMyGLCanvas_DisplayWindow3D::OnMouseDown);
	Connect(wxEVT_RIGHT_DOWN,(wxObjectEventFunction)&CMyGLCanvas_DisplayWindow3D::OnMouseDown);
}

void CMyGLCanvas_DisplayWindow3D::display3D_processKeyEvent(CDisplayWindow3D *m_win3D, wxKeyEvent&ev)
{
	if (m_win3D)
	{
		if (ev.AltDown() && ev.GetKeyCode()== MRPTK_RETURN)
		{
			if (mrpt::system::timeDifference( m_win3D->m_lastFullScreen, mrpt::system::now() )>0.2)
			{
				m_win3D->m_lastFullScreen = mrpt::system::now();
				cout << "[CDisplayWindow3D] Switching fullscreen...\n";
				C3DWindowDialog *win = (C3DWindowDialog*) m_win3D->m_hwnd.get();
				if (win)
				{
					win->ShowFullScreen( !win->IsFullScreen() );
				}
			}
			// Alt+Enter: Don't notify on this key stroke, since if we're switching to fullscreen
			//  and the user is waiting for a key to close the window, a runtime crash will occur,
			//  so return now:
			return;
		}

		const int 				code = ev.GetKeyCode();
		const mrptKeyModifier 	mod = mrpt::gui::keyEventToMrptKeyModifier(ev);

		m_win3D->m_keyPushedCode = code;
		m_win3D->m_keyPushedModifier = mod;
		m_win3D->m_keyPushed = true;

		// Send the event:
		try {
			m_win3D->publishEvent( mrptEventWindowChar(m_win3D,code,mod));
		} catch(...) {}
	}
	//ev.Skip(); // Pass the event to whoever else.
}

void CMyGLCanvas_DisplayWindow3D::OnCharCustom( wxKeyEvent& ev )
{
	CMyGLCanvas_DisplayWindow3D::display3D_processKeyEvent(m_win3D, ev);
}

void CMyGLCanvas_DisplayWindow3D::OnMouseDown(wxMouseEvent& event)
{
	// Send the event:
	if (m_win3D)
	{
		try {
			m_win3D->publishEvent( mrptEventMouseDown(m_win3D, TPixelCoord(event.GetX(), event.GetY()), event.LeftDown(), event.RightDown() ) );
		} catch(...) { }
	}

	event.Skip(); // so it's processed by the wx system!
}


CMyGLCanvas_DisplayWindow3D::~CMyGLCanvas_DisplayWindow3D()
{
	m_openGLScene.clear_unique();	// Avoid the base class to free this object (it's freed by CDisplayWindow3D)
}

void CMyGLCanvas_DisplayWindow3D::OnPreRender()
{
    if (m_openGLScene) m_openGLScene.clear_unique();

	COpenGLScenePtr ptrScene = m_win3D->get3DSceneAndLock();
	if (ptrScene)
		m_openGLScene = ptrScene;
}

void CMyGLCanvas_DisplayWindow3D::OnPostRender()
{
	// Avoid the base class to free this object (it's freed by CDisplayWindow3D)
	m_openGLScene.clear_unique();
	m_win3D->unlockAccess3DScene();

	// If any, draw the 2D text messages:
	int w,h;
	this->GetSize(&w,&h);

	m_text_msgs.render_text_messages_public(w,h);
}

void CMyGLCanvas_DisplayWindow3D::OnPostRenderSwapBuffers(double At, wxPaintDC &dc)
{
	if (m_win3D) m_win3D->internal_setRenderingFPS(At>0 ? 1.0/At : 1e9);

	// If we are requested to do so, grab images to disk as they are rendered:
	string grabFile; 
	if (m_win3D) 
		grabFile = m_win3D->grabImageGetNextFile();
	if (m_win3D && (!grabFile.empty() || m_win3D->isCapturingImgs()) )
	{
		int w,h;
		dc.GetSize(&w, &h);
			
		//Save image directly from OpenGL - It could also use 4 channels and save with GL_BGRA_EXT
		CImagePtr frame(new CImage(w, h, 3, false));
		glReadBuffer(GL_FRONT);
		glReadPixels(0, 0, w, h, GL_BGR_EXT, GL_UNSIGNED_BYTE, (*frame)(0,0) );

		if (!grabFile.empty())
		{
			frame->saveToFile(grabFile);
			m_win3D->internal_emitGrabImageEvent(grabFile);
		}

		if (m_win3D->isCapturingImgs())
		{
			{
				mrpt::synch::CCriticalSectionLocker	lock(& m_win3D->m_last_captured_img_cs );
				m_win3D->m_last_captured_img = frame;
				frame.clear_unique();
			}
		}
	}
}

#endif // Wx + OpenGL

#if MRPT_HAS_WXWIDGETS

BEGIN_EVENT_TABLE(C3DWindowDialog,wxFrame)

END_EVENT_TABLE()


const long C3DWindowDialog::ID_MENUITEM1 = wxNewId();
const long C3DWindowDialog::ID_MENUITEM2 = wxNewId();

C3DWindowDialog::C3DWindowDialog(
    CDisplayWindow3D *win3D,
    WxSubsystem::CWXMainFrame* parent,
    wxWindowID id,
    const std::string &caption,
    wxSize initialSize ) :
    m_win3D( win3D ),
    m_mainFrame(parent)
{
#if MRPT_HAS_OPENGL_GLUT

    Create(parent, id, _U(caption.c_str()), wxDefaultPosition, initialSize, wxDEFAULT_FRAME_STYLE, _T("id"));

	wxIcon FrameIcon;
	FrameIcon.CopyFromBitmap(mrpt::gui::WxSubsystem::getMRPTDefaultIcon());
	SetIcon(FrameIcon);

    // Create the wxCanvas object:
    m_canvas = new CMyGLCanvas_DisplayWindow3D( win3D, this, wxID_ANY, wxDefaultPosition, wxDefaultSize );


    // Events:
    Connect(wxID_ANY,wxEVT_CLOSE_WINDOW,(wxObjectEventFunction)&C3DWindowDialog::OnClose);
    Connect(ID_MENUITEM1,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&C3DWindowDialog::OnMenuClose);
    Connect(ID_MENUITEM2,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&C3DWindowDialog::OnMenuAbout);

	Connect(wxID_ANY,wxEVT_CHAR,(wxObjectEventFunction)&C3DWindowDialog::OnChar);

	Connect(wxID_ANY,wxEVT_SIZE,(wxObjectEventFunction)&C3DWindowDialog::OnResize);

    // Increment number of windows:
    //int winCount =
    WxSubsystem::CWXMainFrame::notifyWindowCreation();
    //cout << "[C3DWindowDialog] Notifying new window: " << winCount << endl;
#else
	THROW_EXCEPTION("MRPT was compiled without OpenGL support")
#endif
	//this->Iconize(false);
}

// Destructor
C3DWindowDialog::~C3DWindowDialog()
{
//	cout << "[C3DWindowDialog::~C3DWindowDialog]" << endl;
}

// OnClose event:
void C3DWindowDialog::OnClose(wxCloseEvent& event)
{
	// Send the event:
	bool allow_close=true;
	try {
		mrptEventWindowClosed ev(m_win3D, true /* allow close */);
		m_win3D->publishEvent(ev);
		allow_close = ev.allow_close;
	} catch(...){}
	if (!allow_close) return; // Don't process this close event.

	//	cout << "[C3DWindowDialog::OnClose]" << endl;
    // Set the m_hwnd=NULL in our parent object.
    m_win3D->notifyChildWindowDestruction();

    // Decrement number of windows:
    WxSubsystem::CWXMainFrame::notifyWindowDestruction();

	// Signal we are destroyed:
    m_win3D->m_semWindowDestroyed.release();

    event.Skip(); // keep processing by parent classes.
}

// Menu: Close
void C3DWindowDialog::OnMenuClose(wxCommandEvent& event)
{
    Close();
}
// Menu: About
void C3DWindowDialog::OnMenuAbout(wxCommandEvent& event)
{
    ::wxMessageBox(_("3D Scene viewer\n Class gui::CDisplayWindow3D\n MRPT C++ library"),_("About..."));
}

void C3DWindowDialog::OnChar(wxKeyEvent& ev)
{
#if MRPT_HAS_OPENGL_GLUT
	CMyGLCanvas_DisplayWindow3D::display3D_processKeyEvent(m_win3D, ev);
#endif
}

void C3DWindowDialog::OnResize(wxSizeEvent& event)
{
#if MRPT_HAS_OPENGL_GLUT
	// Send the event:
	if (m_win3D)
	{
		try {
			m_win3D->publishEvent( mrptEventWindowResize(m_win3D,event.GetSize().GetWidth(),event.GetSize().GetHeight()));
		} catch(...) {}
	}
	event.Skip(); // so it's processed by the wx system!
#endif
}


void C3DWindowDialog::clearTextMessages()
{
#if MRPT_HAS_OPENGL_GLUT
	m_canvas->m_text_msgs.clearTextMessages();
#endif
}

void C3DWindowDialog::addTextMessage(
	const double x_frac,
	const double y_frac,
	const std::string &text,
	const mrpt::utils::TColorf &color,
	const size_t unique_index,
	const mrpt::opengl::TOpenGLFont font
	)
{
#if MRPT_HAS_OPENGL_GLUT
	m_canvas->m_text_msgs.addTextMessage( x_frac, y_frac, text,color,unique_index,font );
#endif
}

void C3DWindowDialog::addTextMessage(
	const double x_frac,
	const double y_frac,
	const std::string &text,
	const mrpt::utils::TColorf &color,
	const std::string  &font_name,
	const double  font_size,
	const mrpt::opengl::TOpenGLFontStyle font_style,
	const size_t  unique_index,
	const double  font_spacing,
	const double  font_kerning,
	const bool has_shadow,
	const mrpt::utils::TColorf &shadow_color
	)
{
#if MRPT_HAS_OPENGL_GLUT
	m_canvas->m_text_msgs.addTextMessage( x_frac, y_frac, text,color, font_name, font_size, font_style, unique_index, font_spacing, font_kerning,has_shadow,shadow_color );
#endif
}



#endif // MRPT_HAS_WXWIDGETS

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CDisplayWindow3D::CDisplayWindow3D(
	const std::string	&windowCaption,
	unsigned int		initialWindowWidth,
	unsigned int		initialWindowHeight )
	: CBaseGUIWindow(static_cast<void*>(this),300,399, windowCaption),
      m_csAccess3DScene(),
      m_grab_imgs_prefix(),
      m_grab_imgs_idx(0),
      m_is_capturing_imgs(false),
      m_last_captured_img_cs("m_last_captured_img_cs"),
	  m_lastFullScreen (mrpt::system::now()),
	  m_last_FPS(10)
{
//	static mrpt::utils::CStdOutStream oo;
//	m_csAccess3DScene.m_debugOut = &oo;

	m_3Dscene = COpenGLScene::Create();
	CBaseGUIWindow::createWxWindow(initialWindowWidth,initialWindowHeight);
}

CDisplayWindow3DPtr CDisplayWindow3D::Create(
	const std::string	&windowCaption,
	unsigned int		initialWindowWidth,
	unsigned int		initialWindowHeight )
{
	return CDisplayWindow3DPtr(new CDisplayWindow3D(windowCaption,initialWindowWidth,initialWindowHeight));
}
/*---------------------------------------------------------------
					Destructor
 ---------------------------------------------------------------*/
CDisplayWindow3D::~CDisplayWindow3D( )
{
	// get lock so we make sure nobody else is touching the window right now.
	m_csAccess3DScene.enter();
	m_csAccess3DScene.leave();

	CBaseGUIWindow::destroyWxWindow();
}


/*---------------------------------------------------------------
					resize
 ---------------------------------------------------------------*/
void  CDisplayWindow3D::resize(
	unsigned int width,
	unsigned int height )
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	if (!isOpen())
	{
		cerr << "[CDisplayWindow3D::setPos] Window closed!: " << m_caption << endl;
		return;
	}

    // Send a request to destroy this object:
    WxSubsystem::TRequestToWxMainThread *REQ = new WxSubsystem::TRequestToWxMainThread[1];
    REQ->source3D = this;
    REQ->OPCODE   = 303;
    REQ->x        = width;
    REQ->y        = height;
    WxSubsystem::pushPendingWxRequest( REQ );
#else
	MRPT_UNUSED_PARAM(width);
	MRPT_UNUSED_PARAM(height);
#endif
}

/*---------------------------------------------------------------
					setPos
 ---------------------------------------------------------------*/
void  CDisplayWindow3D::setPos( int x, int y )
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	if (!isOpen())
	{
		cerr << "[CDisplayWindow3D::setPos] Window closed!: " << m_caption << endl;
		return;
	}

    // Send a request to destroy this object:
    WxSubsystem::TRequestToWxMainThread *REQ = new WxSubsystem::TRequestToWxMainThread[1];
    REQ->source3D = this;
    REQ->OPCODE   = 302;
    REQ->x        = x;
    REQ->y        = y;
    WxSubsystem::pushPendingWxRequest( REQ );
#else
	MRPT_UNUSED_PARAM(x); MRPT_UNUSED_PARAM(y);
#endif
}

/*---------------------------------------------------------------
					setWindowTitle
 ---------------------------------------------------------------*/
void  CDisplayWindow3D::setWindowTitle( const std::string &str )
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	if (!isOpen())
	{
		cerr << "[CDisplayWindow3D::setWindowTitle] Window closed!: " << m_caption << endl;
		return;
	}

    // Send a request to destroy this object:
    WxSubsystem::TRequestToWxMainThread *REQ = new WxSubsystem::TRequestToWxMainThread[1];
    REQ->source3D = this;
    REQ->OPCODE   = 304;
    REQ->str      = str;
    WxSubsystem::pushPendingWxRequest( REQ );
#else
	MRPT_UNUSED_PARAM(str); 
#endif
}


/*---------------------------------------------------------------
					get3DSceneAndLock
 ---------------------------------------------------------------*/
opengl::COpenGLScenePtr& CDisplayWindow3D::get3DSceneAndLock( )
{
	m_csAccess3DScene.enter();
	return m_3Dscene;
}

/*---------------------------------------------------------------
					unlockAccess3DScene
 ---------------------------------------------------------------*/
void  CDisplayWindow3D::unlockAccess3DScene()
{
	m_csAccess3DScene.leave();
}

/*---------------------------------------------------------------
					forceRepaint
 ---------------------------------------------------------------*/
void  CDisplayWindow3D::forceRepaint()
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	C3DWindowDialog *win = (C3DWindowDialog*) m_hwnd.get();
	if (win)
	{
        //win->Refresh(false); // Do not erase background
        // We must do this from the wx thread!

        // Send refresh request:
        WxSubsystem::TRequestToWxMainThread  *REQ = new WxSubsystem::TRequestToWxMainThread[1];
        REQ->source3D = this;
        REQ->OPCODE   = 350;
        WxSubsystem::pushPendingWxRequest( REQ );
	}
#endif
}

/*---------------------------------------------------------------
					setCameraElevationDeg
 ---------------------------------------------------------------*/
void CDisplayWindow3D::setCameraElevationDeg( float deg )
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	C3DWindowDialog *win = (C3DWindowDialog*) m_hwnd.get();
	if (win)
        win->m_canvas->cameraElevationDeg = deg;
#else
	MRPT_UNUSED_PARAM(deg); 
#endif
}

void CDisplayWindow3D::useCameraFromScene(bool useIt)
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	C3DWindowDialog *win = (C3DWindowDialog*) m_hwnd.get();
	if (win)
        win->m_canvas->useCameraFromScene = useIt;
#else
	MRPT_UNUSED_PARAM(useIt);
#endif
}

/*---------------------------------------------------------------
					setCameraAzimuthDeg
 ---------------------------------------------------------------*/
void CDisplayWindow3D::setCameraAzimuthDeg( float deg )
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	C3DWindowDialog *win = (C3DWindowDialog*) m_hwnd.get();
	if (win)
        win->m_canvas->cameraAzimuthDeg = deg;
#else
	MRPT_UNUSED_PARAM(deg);
#endif
}

/*---------------------------------------------------------------
					setCameraPointingToPoint
 ---------------------------------------------------------------*/
void CDisplayWindow3D::setCameraPointingToPoint( float x,float y, float z )
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	C3DWindowDialog *win = (C3DWindowDialog*) m_hwnd.get();
	if (win)
	{
        win->m_canvas->cameraPointingX = x;
        win->m_canvas->cameraPointingY = y;
        win->m_canvas->cameraPointingZ = z;
	}
#else
	MRPT_UNUSED_PARAM(x); MRPT_UNUSED_PARAM(y); MRPT_UNUSED_PARAM(z);
#endif
}

/*---------------------------------------------------------------
					setCameraZoom
 ---------------------------------------------------------------*/
void CDisplayWindow3D::setCameraZoom( float zoom )
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	C3DWindowDialog *win = (C3DWindowDialog*) m_hwnd.get();
	if (win)
        win->m_canvas->cameraZoomDistance = zoom;
#else
	MRPT_UNUSED_PARAM(zoom);
#endif
}

/*---------------------------------------------------------------
					setCameraProjective
 ---------------------------------------------------------------*/
void CDisplayWindow3D::setCameraProjective( bool isProjective )
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	C3DWindowDialog *win = (C3DWindowDialog*) m_hwnd.get();
	if (win)
		win->m_canvas->cameraIsProjective = isProjective;
#else
	MRPT_UNUSED_PARAM(isProjective);
#endif
}

void CDisplayWindow3D::setMinRange(double new_min)
{
	if (m_3Dscene)
	{
		mrpt::opengl::COpenGLViewportPtr gl_view = m_3Dscene->getViewport("main");
		if (gl_view)
		{
			double m,M;
			gl_view->getViewportClipDistances(m,M);
			gl_view->setViewportClipDistances(new_min,M);
		}
	}
}
void CDisplayWindow3D::setMaxRange(double new_max)
{
	if (m_3Dscene)
	{
		mrpt::opengl::COpenGLViewportPtr gl_view = m_3Dscene->getViewport("main");
		if (gl_view)
		{
			double m,M;
			gl_view->getViewportClipDistances(m,M);
			gl_view->setViewportClipDistances(m,new_max);
		}
	}
}

float CDisplayWindow3D::getFOV() const 
{ 
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	C3DWindowDialog *win = (C3DWindowDialog*) m_hwnd.get();
	if (win)
		return win->m_canvas->cameraFOV;
#endif
	return .0f;
}

void CDisplayWindow3D::setFOV(float v)
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	C3DWindowDialog *win = (C3DWindowDialog*) m_hwnd.get();
	if (win)
		win->m_canvas->cameraFOV = v;
#endif
}

/*---------------------------------------------------------------
					getCameraElevationDeg
 ---------------------------------------------------------------*/
float CDisplayWindow3D::getCameraElevationDeg() const
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	const C3DWindowDialog *win = (const C3DWindowDialog*) m_hwnd.get();
	return win ? win->m_canvas->cameraElevationDeg : 0;
#else
    return 0;
#endif
}

/*---------------------------------------------------------------
					getCameraAzimuthDeg
 ---------------------------------------------------------------*/
float CDisplayWindow3D::getCameraAzimuthDeg() const
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	const C3DWindowDialog *win = (const C3DWindowDialog*) m_hwnd.get();
	return win ? win->m_canvas->cameraAzimuthDeg : 0;
#else
    return 0;
#endif
}

/*---------------------------------------------------------------
					getCameraPointingToPoint
 ---------------------------------------------------------------*/
void CDisplayWindow3D::getCameraPointingToPoint( float &x,float &y, float &z ) const
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	const C3DWindowDialog *win = (const C3DWindowDialog*) m_hwnd.get();
	if (win)
	{
        x = win->m_canvas->cameraPointingX;
        y = win->m_canvas->cameraPointingY;
        z = win->m_canvas->cameraPointingZ;
	}
	else
		x=y=z=0;
#else
	MRPT_UNUSED_PARAM(x); MRPT_UNUSED_PARAM(y); MRPT_UNUSED_PARAM(z);
#endif
}

/*---------------------------------------------------------------
					getCameraZoom
 ---------------------------------------------------------------*/
float CDisplayWindow3D::getCameraZoom() const
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	const C3DWindowDialog *win = (const C3DWindowDialog*) m_hwnd.get();
	return win ? win->m_canvas->cameraZoomDistance : 0;
#else
    return 0;
#endif
}

/*---------------------------------------------------------------
					isCameraProjective
 ---------------------------------------------------------------*/
bool CDisplayWindow3D::isCameraProjective() const
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	const C3DWindowDialog *win = (const C3DWindowDialog*) m_hwnd.get();
	return win ? win->m_canvas->cameraIsProjective : true;
#else
    return true;
#endif
}

/*---------------------------------------------------------------
					getLastMousePosition
 ---------------------------------------------------------------*/
bool CDisplayWindow3D::getLastMousePosition(int &x, int &y) const
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	const C3DWindowDialog *win = (const C3DWindowDialog*) m_hwnd.get();
	if (!win) return false;
	win->m_canvas->getLastMousePosition(x,y);
	return true;
#else
	MRPT_UNUSED_PARAM(x); MRPT_UNUSED_PARAM(y);
    return false;
#endif
}

/*---------------------------------------------------------------
					getLastMousePositionRay
 ---------------------------------------------------------------*/
bool CDisplayWindow3D::getLastMousePositionRay(TLine3D &ray) const
{
	int x,y;
	if (getLastMousePosition(x,y))
	{
		m_csAccess3DScene.enter();
		m_3Dscene->getViewport("main")->get3DRayForPixelCoord(x,y,ray);
		m_csAccess3DScene.leave();
		return true;
	}
	else return false;
}


/*---------------------------------------------------------------
					setCursorCross
 ---------------------------------------------------------------*/
void CDisplayWindow3D::setCursorCross(bool cursorIsCross)
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	const C3DWindowDialog *win = (const C3DWindowDialog*) m_hwnd.get();
	if (!win) return;
	win->m_canvas->SetCursor( *(cursorIsCross ? wxCROSS_CURSOR : wxSTANDARD_CURSOR) );
#else
	MRPT_UNUSED_PARAM(cursorIsCross);
#endif
}


/*---------------------------------------------------------------
					grabImagesStart
 ---------------------------------------------------------------*/
void CDisplayWindow3D::grabImagesStart( const std::string &grab_imgs_prefix )
{
	m_grab_imgs_prefix = grab_imgs_prefix;
	m_grab_imgs_idx = 0;
}

/*---------------------------------------------------------------
					grabImagesStop
 ---------------------------------------------------------------*/
void CDisplayWindow3D::grabImagesStop()
{
	m_grab_imgs_prefix.clear();
}

/*---------------------------------------------------------------
					grabImageGetNextFile
 ---------------------------------------------------------------*/
std::string CDisplayWindow3D::grabImageGetNextFile()
{
	if ( m_grab_imgs_prefix.empty() )
			return string();
	else 	return format( "%s%06u.png",m_grab_imgs_prefix.c_str(), m_grab_imgs_idx++ );
}


/*---------------------------------------------------------------
					captureImagesStart
 ---------------------------------------------------------------*/
void CDisplayWindow3D::captureImagesStart()
{
	m_is_capturing_imgs = true;
}

/*---------------------------------------------------------------
					captureImagesStop
 ---------------------------------------------------------------*/
void CDisplayWindow3D::captureImagesStop()
{
	m_is_capturing_imgs = false;
}

/*---------------------------------------------------------------
					getLastWindowImage
 ---------------------------------------------------------------*/
bool CDisplayWindow3D::getLastWindowImage( mrpt::utils::CImage &out_img ) const
{
	bool ret;

	{
		mrpt::synch::CCriticalSectionLocker	lock(& m_last_captured_img_cs );
		if (m_last_captured_img)
		{
			out_img = *m_last_captured_img;  // Copy the full image
			ret = true;
		}
		else ret = false;
	}
	return ret;
}

/*---------------------------------------------------------------
					getLastWindowImagePtr
 ---------------------------------------------------------------*/
CImagePtr CDisplayWindow3D::getLastWindowImagePtr() const
{
	mrpt::synch::CCriticalSectionLocker	lock(& m_last_captured_img_cs );
	return m_last_captured_img;
}

/*---------------------------------------------------------------
					addTextMessage
 ---------------------------------------------------------------*/
void CDisplayWindow3D::addTextMessage(
	const double x_frac,
	const double y_frac,
	const std::string &text,
	const mrpt::utils::TColorf &color,
	const size_t unique_index,
	const TOpenGLFont font
	)
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	C3DWindowDialog *win = (C3DWindowDialog*) m_hwnd.get();
	if (win)
	{
        // Send request:
        // Add a 2D text message:
        //  vector_x: [0]:x, [1]:y, [2,3,4]:R G B, "x": enum of desired font. "y": unique index, "str": String.
        WxSubsystem::TRequestToWxMainThread  *REQ = new WxSubsystem::TRequestToWxMainThread[1];
        REQ->source3D = this;
        REQ->OPCODE   = 360;
        REQ->str 		= text;
        REQ->vector_x.resize(5);
        REQ->vector_x[0] = x_frac;
        REQ->vector_x[1] = y_frac;
        REQ->vector_x[2] = color.R;
        REQ->vector_x[3] = color.G;
        REQ->vector_x[4] = color.B;
        REQ->x 			= int(font);
        REQ->y			= int(unique_index);

        WxSubsystem::pushPendingWxRequest( REQ );
	}
#else
	MRPT_UNUSED_PARAM(x_frac); MRPT_UNUSED_PARAM(y_frac); MRPT_UNUSED_PARAM(text);
	MRPT_UNUSED_PARAM(color); MRPT_UNUSED_PARAM(unique_index); MRPT_UNUSED_PARAM(font);
#endif
}

/*---------------------------------------------------------------
					addTextMessage
 ---------------------------------------------------------------*/
void CDisplayWindow3D::addTextMessage(
	const double x_frac,
	const double y_frac,
	const std::string &text,
	const mrpt::utils::TColorf &color,
	const std::string  &font_name,
	const double  font_size,
	const mrpt::opengl::TOpenGLFontStyle font_style,
	const size_t  unique_index,
	const double  font_spacing,
	const double  font_kerning,
	const bool draw_shadow,
	const mrpt::utils::TColorf &shadow_color
	)
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	C3DWindowDialog *win = (C3DWindowDialog*) m_hwnd.get();
	if (win)
	{
        // Send request:
        // Add a 2D text message:
        WxSubsystem::TRequestToWxMainThread  *REQ = new WxSubsystem::TRequestToWxMainThread[1];
        REQ->source3D = this;
        REQ->OPCODE   = 362;
        REQ->str 		= text;
        REQ->plotName   = font_name;
        REQ->vector_x.resize(12);
        REQ->vector_x[0] = x_frac;
        REQ->vector_x[1] = y_frac;
        REQ->vector_x[2] = color.R;
        REQ->vector_x[3] = color.G;
        REQ->vector_x[4] = color.B;
        REQ->vector_x[5] = font_size;
        REQ->vector_x[6] = font_spacing;
        REQ->vector_x[7] = font_kerning;
		REQ->vector_x[8] = draw_shadow ? 1:0;
        REQ->vector_x[9] = shadow_color.R;
        REQ->vector_x[10] = shadow_color.G;
        REQ->vector_x[11] = shadow_color.B;

        REQ->x			= int(font_style);
        REQ->y			= int(unique_index);

        WxSubsystem::pushPendingWxRequest( REQ );
	}
#else
	MRPT_UNUSED_PARAM(x_frac);
	MRPT_UNUSED_PARAM(y_frac);
	MRPT_UNUSED_PARAM(text);
	MRPT_UNUSED_PARAM(color);
	MRPT_UNUSED_PARAM(font_name);
	MRPT_UNUSED_PARAM(font_size);
	MRPT_UNUSED_PARAM(font_style);
	MRPT_UNUSED_PARAM(unique_index);
	MRPT_UNUSED_PARAM(font_spacing);
	MRPT_UNUSED_PARAM(font_kerning);
	MRPT_UNUSED_PARAM(draw_shadow);
	MRPT_UNUSED_PARAM(shadow_color);
#endif
}

/*---------------------------------------------------------------
					clearTextMessages
 ---------------------------------------------------------------*/
void CDisplayWindow3D::clearTextMessages()
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	C3DWindowDialog *win = (C3DWindowDialog*) m_hwnd.get();
	if (win)
	{
        // Send request:
        WxSubsystem::TRequestToWxMainThread  *REQ = new WxSubsystem::TRequestToWxMainThread[1];
        REQ->source3D = this;
        REQ->OPCODE   = 361;
        WxSubsystem::pushPendingWxRequest( REQ );
	}
#endif
}

void CDisplayWindow3D::internal_setRenderingFPS(double FPS)
{
	const double ALPHA = 0.99;
	m_last_FPS = ALPHA*m_last_FPS+(1-ALPHA)*FPS;
}

// Called by CMyGLCanvas_DisplayWindow3D::OnPostRenderSwapBuffers
void CDisplayWindow3D::internal_emitGrabImageEvent(const std::string &fil)
{
	const mrptEvent3DWindowGrabImageFile ev(this,fil);
	publishEvent(ev);
}

// Returns the "main" viewport of the scene.
mrpt::opengl::COpenGLViewportPtr CDisplayWindow3D::getDefaultViewport()
{
	m_csAccess3DScene.enter();
	mrpt::opengl::COpenGLViewportPtr view = m_3Dscene->getViewport("main");
	m_csAccess3DScene.leave();
	return view;
}

void CDisplayWindow3D::setImageView(const mrpt::utils::CImage &img)
{
	m_csAccess3DScene.enter();
	mrpt::opengl::COpenGLViewportPtr view = m_3Dscene->getViewport("main");
	view->setImageView(img);
	m_csAccess3DScene.leave();
}

void CDisplayWindow3D::setImageView_fast(mrpt::utils::CImage &img)
{
	m_csAccess3DScene.enter();
	mrpt::opengl::COpenGLViewportPtr view = m_3Dscene->getViewport("main");
	view->setImageView_fast(img);
	m_csAccess3DScene.leave();
}


CDisplayWindow3DLocker::CDisplayWindow3DLocker(CDisplayWindow3D &win, mrpt::opengl::COpenGLScenePtr &out_scene_ptr) :
	m_win(win)
{
	out_scene_ptr = m_win.get3DSceneAndLock();
}
CDisplayWindow3DLocker::CDisplayWindow3DLocker(CDisplayWindow3D &win) : 
	m_win(win)
{
	m_win.get3DSceneAndLock();
}
CDisplayWindow3DLocker::~CDisplayWindow3DLocker()
{
	m_win.unlockAccess3DScene();
}

