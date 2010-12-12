/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/gui.h>  // precompiled header


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
	#endif
	#ifdef HAVE_FREEGLUT_EXT_H
		#include <GL/freeglut_ext.h>
	#endif
#endif

#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT

#if !wxUSE_GLCANVAS
#error "OpenGL required: set wxUSE_GLCANVAS to 1 and rebuild wxWidgets"
#endif

#include <mrpt/gui/CMyGLCanvasBase.h>

namespace mrpt
{
	namespace gui
	{

		class CMyGLCanvas_DisplayWindow3D : public mrpt::gui::CMyGLCanvasBase
		{
		protected:
			struct T2DTextData
			{
				std::string 			text;
				mrpt::utils::TColorf	color;
				mrpt::opengl::TOpenGLFont font;
				double					x,y;
			};
			std::map<size_t,T2DTextData>  m_2D_texts;

		public:
			CMyGLCanvas_DisplayWindow3D( CDisplayWindow3D *win3D,
						 wxWindow *parent, wxWindowID id = wxID_ANY,
						 const wxPoint& pos = wxDefaultPosition,
						 const wxSize& size = wxDefaultSize,
						 long style = 0, const wxString& name = _T("CMyGLCanvas_DisplayWindow3D") );

			virtual ~CMyGLCanvas_DisplayWindow3D();


			CDisplayWindow3D *m_win3D;

			void OnCharCustom( wxKeyEvent& event );
			void OnMouseDown(wxMouseEvent& event);

			void OnPreRender();
			void OnPostRender();
			void OnPostRenderSwapBuffers(double At, wxPaintDC &dc);

			void clearTextMessages()
			{
				m_2D_texts.clear();
			}

			void addTextMessage(
				const double x_frac,
				const double y_frac,
				const std::string &text,
				const mrpt::utils::TColorf &color = mrpt::utils::TColorf(1.0,1.0,1.0),
				const size_t unique_index = 0,
				const mrpt::opengl::TOpenGLFont font = mrpt::opengl::MRPT_GLUT_BITMAP_TIMES_ROMAN_24
				)
			{
				T2DTextData  d;
				d.text = text;
				d.color = color;
				d.x = x_frac;
				d.y = y_frac;
				d.font = font;

				m_2D_texts[unique_index] = d;
			}

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

	for (std::map<size_t,T2DTextData>::const_iterator it=m_2D_texts.begin();it!=m_2D_texts.end();++it)
	{
		// If (x,y) \in [0,1[, it's interpreted as a ratio, otherwise, as an actual coordinate in pixels
		int x = it->second.x>=1 ? int(it->second.x) : (it->second.x<0 ? int(w+it->second.x) : int(it->second.x * w));
		int y = it->second.y>=1 ? int(it->second.y) : (it->second.y<0 ? int(h+it->second.y) : int(it->second.y * h));
		this->renderTextBitmap(
			x,y,
			it->second.text,
			it->second.color.R,it->second.color.G,it->second.color.B,
			it->second.font);
	}
}

void CMyGLCanvas_DisplayWindow3D::OnPostRenderSwapBuffers(double At, wxPaintDC &dc)
{
	if (m_win3D) m_win3D->setRenderingFPS(At>0 ? 1.0/At : 1e9);

	// If we are requested to do so, grab images to disk as they are rendered:
	string grabFile = m_win3D->grabImageGetNextFile();
	if (m_win3D && !grabFile.empty() || m_win3D->isCapturingImgs() )
	{
		int w,h;
		dc.GetSize(&w, &h);

		// create a memory DC and bitmap to capture the DC
		wxMemoryDC memDC;
		wxBitmap memBmp(w, h);
		memDC.SelectObject(memBmp);
		memDC.Blit(0,0, w,h, &dc, 0,0);

		if (!grabFile.empty())
			memBmp.SaveFile( _U(grabFile.c_str()) , wxBITMAP_TYPE_PNG );

		if (m_win3D->isCapturingImgs())
		{
			wxImage img = memBmp.ConvertToImage();
			CImagePtr pimg = mrpt::gui::wxImage2MRPTImagePtr(img);

			{
				mrpt::synch::CCriticalSectionLocker	lock(& m_win3D->m_last_captured_img_cs );
				m_win3D->m_last_captured_img = pimg;
				pimg.clear_unique();
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
	m_canvas->clearTextMessages();
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
	m_canvas->addTextMessage( x_frac, y_frac, text,color,unique_index,font );
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
      m_grab_imgs_prefix(),
      m_grab_imgs_idx(0),
      m_is_capturing_imgs(false),
	  m_lastFullScreen (mrpt::system::now())
{
	m_3Dscene = COpenGLScene::Create();
	CBaseGUIWindow::createWxWindow(initialWindowWidth,initialWindowHeight);
}


/*---------------------------------------------------------------
					Destructor
 ---------------------------------------------------------------*/
CDisplayWindow3D::~CDisplayWindow3D( )
{
	// All done in base class.
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
#endif
}

void CDisplayWindow3D::useCameraFromScene(bool useIt)
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	C3DWindowDialog *win = (C3DWindowDialog*) m_hwnd.get();
	if (win)
        win->m_canvas->useCameraFromScene = useIt;
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
	else x=y=z=0;
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
void CDisplayWindow3D::getLastWindowImage( mrpt::utils::CImage &out_img ) const
{
	mrpt::synch::CCriticalSectionLocker	lock(& m_last_captured_img_cs );

	out_img = *m_last_captured_img;  // Copy the full image
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

void CDisplayWindow3D::setRenderingFPS(double FPS) 
{
	mrpt::synch::CCriticalSectionLocker lock(&m_last_FPSs_cs); 

	m_last_FPSs.push_back(FPS); 
	if (m_last_FPSs.size()>250)
		m_last_FPSs.erase(m_last_FPSs.begin());
}

double CDisplayWindow3D::getRenderingFPS() const 
{ 
	mrpt::synch::CCriticalSectionLocker lock(&m_last_FPSs_cs); 

	if (m_last_FPSs.empty())
	     return 0;
	else return mrpt::math::mean( m_last_FPSs );
}
