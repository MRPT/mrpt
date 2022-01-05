/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "gui-precomp.h"  // Precompiled headers
//
#include <mrpt/config.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/gui/WxSubsystem.h>
#include <mrpt/gui/WxUtils.h>
#include <mrpt/img/CImage.h>
#include <mrpt/system/CTicTac.h>

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

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::img;
using namespace std;

#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT

#if !wxUSE_GLCANVAS
#error "OpenGL required: set wxUSE_GLCANVAS to 1 and rebuild wxWidgets"
#endif

#include <mrpt/gui/CWxGLCanvasBase.h>

namespace mrpt::gui
{
class CMyGLCanvas_DisplayWindow3D : public mrpt::gui::CWxGLCanvasBase
{
   public:
	CMyGLCanvas_DisplayWindow3D(
		CDisplayWindow3D* win3D, wxWindow* parent, wxWindowID id = wxID_ANY,
		const wxPoint& pos = wxDefaultPosition,
		const wxSize& size = wxDefaultSize, long style = 0,
		const wxString& name = _T("CMyGLCanvas_DisplayWindow3D"));

	~CMyGLCanvas_DisplayWindow3D() override;

	CDisplayWindow3D* m_win3D = nullptr;

	void OnCharCustom(wxKeyEvent& event) override;
	void OnMouseDown(wxMouseEvent& event);
	void OnMouseMove(wxMouseEvent& event);

	void OnPreRender() override;
	void OnPostRender() override;
	void OnPostRenderSwapBuffers(double At, wxPaintDC& dc) override;

	static void display3D_processKeyEvent(
		CDisplayWindow3D* m_win3D, wxKeyEvent& ev);
};
}  // namespace mrpt::gui

CMyGLCanvas_DisplayWindow3D::CMyGLCanvas_DisplayWindow3D(
	CDisplayWindow3D* win3D, wxWindow* parent, wxWindowID id,
	const wxPoint& pos, const wxSize& size, long style, const wxString& name)
	: CWxGLCanvasBase(parent, id, pos, size, style, name), m_win3D(win3D)
{
	this->Bind(wxEVT_CHAR, &CMyGLCanvas_DisplayWindow3D::OnCharCustom, this);
	this->Bind(
		wxEVT_CHAR_HOOK, &CMyGLCanvas_DisplayWindow3D::OnCharCustom, this);
	this->Bind(
		wxEVT_LEFT_DOWN, &CMyGLCanvas_DisplayWindow3D::OnMouseDown, this);
	this->Bind(
		wxEVT_RIGHT_DOWN, &CMyGLCanvas_DisplayWindow3D::OnMouseDown, this);
	this->Bind(wxEVT_MOTION, &CMyGLCanvas_DisplayWindow3D::OnMouseMove, this);
}

void CMyGLCanvas_DisplayWindow3D::display3D_processKeyEvent(
	CDisplayWindow3D* m_win3D, wxKeyEvent& ev)
{
	if (m_win3D)
	{
		if (ev.AltDown() && ev.GetKeyCode() == MRPTK_RETURN)
		{
			if (mrpt::system::timeDifference(
					m_win3D->m_lastFullScreen, mrpt::system::now()) > 0.2)
			{
				m_win3D->m_lastFullScreen = mrpt::system::now();
				cout << "[CDisplayWindow3D] Switching fullscreen...\n";
				auto* win = (C3DWindowDialog*)m_win3D->m_hwnd.get();
				if (win) { win->ShowFullScreen(!win->IsFullScreen()); }
			}
			// Alt+Enter: Don't notify on this key stroke, since if we're
			// switching to fullscreen
			//  and the user is waiting for a key to close the window, a runtime
			//  crash will occur,
			//  so return now:
			return;
		}

		const int code = ev.GetKeyCode();
		const mrptKeyModifier mod = mrpt::gui::keyEventToMrptKeyModifier(ev);

		m_win3D->m_keyPushedCode = code;
		m_win3D->m_keyPushedModifier = mod;
		m_win3D->m_keyPushed = true;

		// Send the event:
		try
		{
			m_win3D->publishEvent(mrptEventWindowChar(m_win3D, code, mod));
		}
		catch (...)
		{
		}
	}
	// ev.Skip(); // Pass the event to whoever else.
}

void CMyGLCanvas_DisplayWindow3D::OnCharCustom(wxKeyEvent& ev)
{
	CMyGLCanvas_DisplayWindow3D::display3D_processKeyEvent(m_win3D, ev);
}

void CMyGLCanvas_DisplayWindow3D::OnMouseDown(wxMouseEvent& event)
{
	// Send the event:
	if (m_win3D && m_win3D->hasSubscribers())
	{
		try
		{
			m_win3D->publishEvent(mrptEventMouseDown(
				m_win3D, TPixelCoord(event.GetX(), event.GetY()),
				event.LeftDown(), event.RightDown()));
		}
		catch (...)
		{
		}
	}

	event.Skip();  // so it's processed by the wx system!
}

void CMyGLCanvas_DisplayWindow3D::OnMouseMove(wxMouseEvent& event)
{
	// Send the event:
	if (m_win3D && m_win3D->hasSubscribers())
	{
		try
		{
			m_win3D->publishEvent(mrptEventMouseMove(
				m_win3D, TPixelCoord(event.GetX(), event.GetY()),
				event.LeftDown(), event.RightDown()));
		}
		catch (...)
		{
		}
	}

	event.Skip();  // so it's processed by the wx system!
}

CMyGLCanvas_DisplayWindow3D::~CMyGLCanvas_DisplayWindow3D()
{
	// Ensure all OpenGL resources are freed before the opengl context is gone:
	if (getOpenGLSceneRef()) getOpenGLSceneRef()->unloadShaders();

	// Unbind all objects, free all buffers:
	auto& scene = getOpenGLSceneRef();
	if (scene) scene->freeOpenGLResources();
}

void CMyGLCanvas_DisplayWindow3D::OnPreRender()
{
	auto& openGLSceneRef = getOpenGLSceneRef();
	if (openGLSceneRef) openGLSceneRef.reset();

	COpenGLScene::Ptr& ptrScene = m_win3D->get3DSceneAndLock();
	if (ptrScene) openGLSceneRef = ptrScene;
}

void CMyGLCanvas_DisplayWindow3D::OnPostRender()
{
	m_win3D->unlockAccess3DScene();
}

void CMyGLCanvas_DisplayWindow3D::OnPostRenderSwapBuffers(
	double At, wxPaintDC& dc)
{
	if (m_win3D) m_win3D->internal_setRenderingFPS(At > 0 ? 1.0 / At : 1e9);

	// If we are requested to do so, grab images to disk as they are rendered:
	string grabFile;
	if (m_win3D) grabFile = m_win3D->grabImageGetNextFile();
	if (m_win3D && (!grabFile.empty() || m_win3D->isCapturingImgs()))
	{
		int w, h;
		dc.GetSize(&w, &h);

		// Save image directly from OpenGL - It could also use 4 channels and
		// save with GL_BGRA_EXT
		auto frame = CImage::Create(w, h, mrpt::img::CH_RGB);

		glPixelStorei(GL_PACK_ALIGNMENT, 1);
		glPixelStorei(GL_PACK_ROW_LENGTH, 0);

		glReadBuffer(GL_FRONT);
		glReadPixels(0, 0, w, h, GL_BGR_EXT, GL_UNSIGNED_BYTE, (*frame)(0, 0));
		frame->flipVertical();

		if (!grabFile.empty())
		{
			frame->saveToFile(grabFile);
			m_win3D->internal_emitGrabImageEvent(grabFile);
		}

		if (m_win3D->isCapturingImgs())
		{
			{
				std::lock_guard<std::mutex> lock(
					m_win3D->m_last_captured_img_cs);
				m_win3D->m_last_captured_img = frame;
				frame.reset();
			}
		}
	}
}

#endif	// Wx + OpenGL

#if MRPT_HAS_WXWIDGETS

BEGIN_EVENT_TABLE(C3DWindowDialog, wxFrame)

END_EVENT_TABLE()

const long C3DWindowDialog::ID_MENUITEM1 = wxNewId();
const long C3DWindowDialog::ID_MENUITEM2 = wxNewId();

C3DWindowDialog::C3DWindowDialog(
	CDisplayWindow3D* win3D, WxSubsystem::CWXMainFrame* parent, wxWindowID id,
	const std::string& caption, wxSize initialSize)
	: m_win3D(win3D), m_mainFrame(parent)
{
#if MRPT_HAS_OPENGL_GLUT

	Create(
		parent, id, caption.c_str(), wxDefaultPosition, initialSize,
		wxDEFAULT_FRAME_STYLE, _T("id"));

	wxIcon FrameIcon;
	FrameIcon.CopyFromBitmap(mrpt::gui::WxSubsystem::getMRPTDefaultIcon());
	SetIcon(FrameIcon);

	// Create the wxCanvas object:
	m_canvas = new CMyGLCanvas_DisplayWindow3D(
		win3D, this, wxID_ANY, wxDefaultPosition, wxDefaultSize);

	// Events:
	this->Bind(wxEVT_CLOSE_WINDOW, &C3DWindowDialog::OnClose, this);
	this->Bind(wxEVT_MENU, &C3DWindowDialog::OnMenuClose, this, ID_MENUITEM1);
	this->Bind(wxEVT_MENU, &C3DWindowDialog::OnMenuAbout, this, ID_MENUITEM2);
	this->Bind(wxEVT_CHAR, &C3DWindowDialog::OnChar, this);
	this->Bind(wxEVT_SIZE, &C3DWindowDialog::OnResize, this);

	// Increment number of windows:
	// int winCount =
	WxSubsystem::CWXMainFrame::notifyWindowCreation();
// cout << "[C3DWindowDialog] Notifying new window: " << winCount << endl;
#else
	THROW_EXCEPTION("MRPT was compiled without OpenGL support");
#endif
	// this->Iconize(false);
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
	bool allow_close = true;
	try
	{
		mrptEventWindowClosed ev(m_win3D, true /* allow close */);
		m_win3D->publishEvent(ev);
		allow_close = ev.allow_close;
	}
	catch (...)
	{
	}
	if (!allow_close) return;  // Don't process this close event.

	//	cout << "[C3DWindowDialog::OnClose]" << endl;
	// Set the m_hwnd=nullptr in our parent object.
	m_win3D->notifyChildWindowDestruction();

	// Decrement number of windows:
	WxSubsystem::CWXMainFrame::notifyWindowDestruction();

	// Signal we are destroyed:
	m_win3D->m_windowDestroyed.set_value();

	event.Skip();  // keep processing by parent classes.
}

// Menu: Close
void C3DWindowDialog::OnMenuClose(wxCommandEvent&) { Close(); }
// Menu: About
void C3DWindowDialog::OnMenuAbout(wxCommandEvent&)
{
	::wxMessageBox(
		_("3D Scene viewer\n Class gui::CDisplayWindow3D\n MRPT C++ library"),
		_("About..."));
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
		try
		{
			m_win3D->publishEvent(mrptEventWindowResize(
				m_win3D, event.GetSize().GetWidth(),
				event.GetSize().GetHeight()));
		}
		catch (...)
		{
		}
	}
	event.Skip();  // so it's processed by the wx system!
#endif
}

#endif	// MRPT_HAS_WXWIDGETS

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CDisplayWindow3D::CDisplayWindow3D(
	const std::string& windowCaption, unsigned int initialWindowWidth,
	unsigned int initialWindowHeight)
	: CBaseGUIWindow(static_cast<void*>(this), 300, 399, windowCaption),
	  m_lastFullScreen(mrpt::system::now())
{
	m_3Dscene = COpenGLScene::Create();
	CBaseGUIWindow::createWxWindow(initialWindowWidth, initialWindowHeight);
}

CDisplayWindow3D::Ptr CDisplayWindow3D::Create(
	const std::string& windowCaption, unsigned int initialWindowWidth,
	unsigned int initialWindowHeight)
{
	return std::make_shared<CDisplayWindow3D>(
		windowCaption, initialWindowWidth, initialWindowHeight);
}
/*---------------------------------------------------------------
					Destructor
 ---------------------------------------------------------------*/
CDisplayWindow3D::~CDisplayWindow3D()
{
	// get lock so we make sure nobody else is touching the window right now.
	bool lock_ok = m_csAccess3DScene.try_lock_for(std::chrono::seconds(2));
	m_csAccess3DScene.unlock();

	CBaseGUIWindow::destroyWxWindow();

	if (!lock_ok)
		std::cerr
			<< "[~CDisplayWindow3D] Warning: Timeout acquiring mutex lock.\n";
}

/*---------------------------------------------------------------
					resize
 ---------------------------------------------------------------*/
void CDisplayWindow3D::resize(
	[[maybe_unused]] unsigned int width, [[maybe_unused]] unsigned int height)
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	if (!isOpen())
	{
		cerr << "[CDisplayWindow3D::setPos] Window closed!: " << m_caption
			 << endl;
		return;
	}

	// Send a request to destroy this object:
	auto* REQ = new WxSubsystem::TRequestToWxMainThread[1];
	REQ->source3D = this;
	REQ->OPCODE = 303;
	REQ->x = width;
	REQ->y = height;
	WxSubsystem::pushPendingWxRequest(REQ);
#endif
}

/*---------------------------------------------------------------
					setPos
 ---------------------------------------------------------------*/
void CDisplayWindow3D::setPos([[maybe_unused]] int x, [[maybe_unused]] int y)
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	if (!isOpen())
	{
		cerr << "[CDisplayWindow3D::setPos] Window closed!: " << m_caption
			 << endl;
		return;
	}

	// Send a request to destroy this object:
	auto* REQ = new WxSubsystem::TRequestToWxMainThread[1];
	REQ->source3D = this;
	REQ->OPCODE = 302;
	REQ->x = x;
	REQ->y = y;
	WxSubsystem::pushPendingWxRequest(REQ);
#endif
}

/*---------------------------------------------------------------
					setWindowTitle
 ---------------------------------------------------------------*/
void CDisplayWindow3D::setWindowTitle([[maybe_unused]] const std::string& str)
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	if (!isOpen())
	{
		cerr << "[CDisplayWindow3D::setWindowTitle] Window closed!: "
			 << m_caption << endl;
		return;
	}

	// Send a request to destroy this object:
	auto* REQ = new WxSubsystem::TRequestToWxMainThread[1];
	REQ->source3D = this;
	REQ->OPCODE = 304;
	REQ->str = str;
	WxSubsystem::pushPendingWxRequest(REQ);
#endif
}

opengl::COpenGLScene::Ptr& CDisplayWindow3D::get3DSceneAndLock()
{
	m_csAccess3DScene.lock();
	return m_3Dscene;
}

void CDisplayWindow3D::unlockAccess3DScene() { m_csAccess3DScene.unlock(); }

void CDisplayWindow3D::forceRepaint()
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	if (auto* win = (C3DWindowDialog*)m_hwnd.get(); win)
	{
		// Send refresh request:
		auto* REQ = new WxSubsystem::TRequestToWxMainThread[1];
		REQ->source3D = this;
		REQ->OPCODE = 350;
		WxSubsystem::pushPendingWxRequest(REQ);
	}
#endif
}

/*---------------------------------------------------------------
					setCameraElevationDeg
 ---------------------------------------------------------------*/
void CDisplayWindow3D::setCameraElevationDeg([[maybe_unused]] float deg)
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	auto* win = (C3DWindowDialog*)m_hwnd.get();
	if (win) win->m_canvas->setElevationDegrees(deg);
#endif
}

void CDisplayWindow3D::useCameraFromScene([[maybe_unused]] bool useIt)
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	auto* win = (C3DWindowDialog*)m_hwnd.get();
	if (win) win->m_canvas->setUseCameraFromScene(useIt);
#endif
}

/*---------------------------------------------------------------
					setCameraAzimuthDeg
 ---------------------------------------------------------------*/
void CDisplayWindow3D::setCameraAzimuthDeg([[maybe_unused]] float deg)
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	auto* win = (C3DWindowDialog*)m_hwnd.get();
	if (win) win->m_canvas->setAzimuthDegrees(deg);
#endif
}

/*---------------------------------------------------------------
					setCameraPointingToPoint
 ---------------------------------------------------------------*/
void CDisplayWindow3D::setCameraPointingToPoint(
	[[maybe_unused]] float x, [[maybe_unused]] float y,
	[[maybe_unused]] float z)
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	auto* win = (C3DWindowDialog*)m_hwnd.get();
	if (win) { win->m_canvas->setCameraPointing(x, y, z); }
#endif
}

/*---------------------------------------------------------------
					setCameraZoom
 ---------------------------------------------------------------*/
void CDisplayWindow3D::setCameraZoom([[maybe_unused]] float zoom)
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	auto* win = (C3DWindowDialog*)m_hwnd.get();
	if (win) win->m_canvas->setZoomDistance(zoom);
#endif
}

/*---------------------------------------------------------------
					setCameraProjective
 ---------------------------------------------------------------*/
void CDisplayWindow3D::setCameraProjective([[maybe_unused]] bool isProjective)
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	auto* win = (C3DWindowDialog*)m_hwnd.get();
	if (win) win->m_canvas->setCameraProjective(isProjective);
#endif
}

void CDisplayWindow3D::setMinRange(float new_min)
{
	if (m_3Dscene)
	{
		mrpt::opengl::COpenGLViewport::Ptr gl_view =
			m_3Dscene->getViewport("main");
		if (gl_view)
		{
			float m, M;
			gl_view->getViewportClipDistances(m, M);
			gl_view->setViewportClipDistances(new_min, M);
		}
	}
}
void CDisplayWindow3D::setMaxRange(float new_max)
{
	if (m_3Dscene)
	{
		mrpt::opengl::COpenGLViewport::Ptr gl_view =
			m_3Dscene->getViewport("main");
		if (gl_view)
		{
			float m, M;
			gl_view->getViewportClipDistances(m, M);
			gl_view->setViewportClipDistances(m, new_max);
		}
	}
}

float CDisplayWindow3D::getFOV() const
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	auto* win = (C3DWindowDialog*)m_hwnd.get();
	if (win) return win->m_canvas->cameraFOV();
#endif
	return .0f;
}

void CDisplayWindow3D::setFOV(float v)
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	auto* win = (C3DWindowDialog*)m_hwnd.get();
	if (win) win->m_canvas->setCameraFOV(v);
#endif
}

/*---------------------------------------------------------------
					getCameraElevationDeg
 ---------------------------------------------------------------*/
float CDisplayWindow3D::getCameraElevationDeg() const
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	const auto* win = (const C3DWindowDialog*)m_hwnd.get();
	return win ? win->m_canvas->getElevationDegrees() : 0;
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
	const auto* win = (const C3DWindowDialog*)m_hwnd.get();
	return win ? win->m_canvas->getAzimuthDegrees() : 0;
#else
	return 0;
#endif
}

/*---------------------------------------------------------------
					getCameraPointingToPoint
 ---------------------------------------------------------------*/
void CDisplayWindow3D::getCameraPointingToPoint(
	[[maybe_unused]] float& x, [[maybe_unused]] float& y,
	[[maybe_unused]] float& z) const
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	const auto* win = (const C3DWindowDialog*)m_hwnd.get();
	if (win)
	{
		x = win->m_canvas->getCameraPointingX();
		y = win->m_canvas->getCameraPointingY();
		z = win->m_canvas->getCameraPointingZ();
	}
	else
		x = y = z = 0;
#endif
}

/*---------------------------------------------------------------
					getCameraZoom
 ---------------------------------------------------------------*/
float CDisplayWindow3D::getCameraZoom() const
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	const auto* win = (const C3DWindowDialog*)m_hwnd.get();
	return win ? win->m_canvas->getZoomDistance() : 0;
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
	const auto* win = (const C3DWindowDialog*)m_hwnd.get();
	return win ? win->m_canvas->isCameraProjective() : true;
#else
	return true;
#endif
}

/*---------------------------------------------------------------
					getLastMousePosition
 ---------------------------------------------------------------*/
bool CDisplayWindow3D::getLastMousePosition(
	[[maybe_unused]] int& x, [[maybe_unused]] int& y) const
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	const auto* win = (const C3DWindowDialog*)m_hwnd.get();
	if (!win) return false;
	win->m_canvas->getLastMousePosition(x, y);
	return true;
#else
	return false;
#endif
}

/*---------------------------------------------------------------
					getLastMousePositionRay
 ---------------------------------------------------------------*/
bool CDisplayWindow3D::getLastMousePositionRay(TLine3D& ray) const
{
	int x, y;
	if (getLastMousePosition(x, y))
	{
		std::lock_guard<std::recursive_timed_mutex> lck(m_csAccess3DScene);
		m_3Dscene->getViewport("main")->get3DRayForPixelCoord(x, y, ray);
		return true;
	}
	else
		return false;
}

/*---------------------------------------------------------------
					setCursorCross
 ---------------------------------------------------------------*/
void CDisplayWindow3D::setCursorCross([[maybe_unused]] bool cursorIsCross)
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	const auto* win = (const C3DWindowDialog*)m_hwnd.get();
	if (!win) return;
	win->m_canvas->SetCursor(
		*(cursorIsCross ? wxCROSS_CURSOR : wxSTANDARD_CURSOR));
#endif
}

/*---------------------------------------------------------------
					grabImagesStart
 ---------------------------------------------------------------*/
void CDisplayWindow3D::grabImagesStart(const std::string& grab_imgs_prefix)
{
	m_grab_imgs_prefix = grab_imgs_prefix;
	m_grab_imgs_idx = 0;
}

/*---------------------------------------------------------------
					grabImagesStop
 ---------------------------------------------------------------*/
void CDisplayWindow3D::grabImagesStop() { m_grab_imgs_prefix.clear(); }
/*---------------------------------------------------------------
					grabImageGetNextFile
 ---------------------------------------------------------------*/
std::string CDisplayWindow3D::grabImageGetNextFile()
{
	if (m_grab_imgs_prefix.empty()) return string();
	else
		return format(
			"%s%06u.png", m_grab_imgs_prefix.c_str(), m_grab_imgs_idx++);
}

/*---------------------------------------------------------------
					captureImagesStart
 ---------------------------------------------------------------*/
void CDisplayWindow3D::captureImagesStart() { m_is_capturing_imgs = true; }
/*---------------------------------------------------------------
					captureImagesStop
 ---------------------------------------------------------------*/
void CDisplayWindow3D::captureImagesStop() { m_is_capturing_imgs = false; }
/*---------------------------------------------------------------
					getLastWindowImage
 ---------------------------------------------------------------*/
bool CDisplayWindow3D::getLastWindowImage(mrpt::img::CImage& out_img) const
{
	bool ret;

	{
		std::lock_guard<std::mutex> lock(m_last_captured_img_cs);
		if (m_last_captured_img)
		{
			out_img = *m_last_captured_img;	 // Copy the full image
			ret = true;
		}
		else
			ret = false;
	}
	return ret;
}

/*---------------------------------------------------------------
					getLastWindowImagePtr
 ---------------------------------------------------------------*/
CImage::Ptr CDisplayWindow3D::getLastWindowImagePtr() const
{
	std::lock_guard<std::mutex> lock(m_last_captured_img_cs);
	return m_last_captured_img;
}

void CDisplayWindow3D::internal_setRenderingFPS(double FPS)
{
	const double ALPHA = 0.99;
	m_last_FPS = ALPHA * m_last_FPS + (1 - ALPHA) * FPS;
}

// Called by CMyGLCanvas_DisplayWindow3D::OnPostRenderSwapBuffers
void CDisplayWindow3D::internal_emitGrabImageEvent(const std::string& fil)
{
	const mrptEvent3DWindowGrabImageFile ev(this, fil);
	publishEvent(ev);
}

// Returns the "main" viewport of the scene.
mrpt::opengl::COpenGLViewport::Ptr CDisplayWindow3D::getDefaultViewport()
{
	std::lock_guard<std::recursive_timed_mutex> lck(m_csAccess3DScene);
	return m_3Dscene->getViewport("main");
}

void CDisplayWindow3D::setImageView(const mrpt::img::CImage& img)
{
	std::lock_guard<std::recursive_timed_mutex> lck(m_csAccess3DScene);
	m_3Dscene->getViewport("main")->setImageView(img);
}

void CDisplayWindow3D::setImageView(mrpt::img::CImage&& img)
{
	std::lock_guard<std::recursive_timed_mutex> lck(m_csAccess3DScene);
	m_3Dscene->getViewport("main")->setImageView(std::move(img));
}

CDisplayWindow3DLocker::CDisplayWindow3DLocker(
	CDisplayWindow3D& win, mrpt::opengl::COpenGLScene::Ptr& out_scene_ptr)
	: m_win(win)
{
	out_scene_ptr = m_win.get3DSceneAndLock();
}
CDisplayWindow3DLocker::CDisplayWindow3DLocker(CDisplayWindow3D& win)
	: m_win(win)
{
	m_win.get3DSceneAndLock();
}
CDisplayWindow3DLocker::~CDisplayWindow3DLocker()
{
	m_win.unlockAccess3DScene();
}

void CDisplayWindow3D::sendFunctionToRunOnGUIThread(
	const std::function<void(void)>& f)
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	auto* win = (C3DWindowDialog*)m_hwnd.get();
	if (!win) return;

	// Send refresh request:
	auto* REQ = new WxSubsystem::TRequestToWxMainThread[1];
	REQ->source3D = this;
	REQ->OPCODE = 800;
	REQ->userFunction = f;
	WxSubsystem::pushPendingWxRequest(REQ);

#endif
}

bool CDisplayWindow3D::is_GL_context_created() const
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	auto* win = (C3DWindowDialog*)m_hwnd.get();
	if (!win || !win->m_canvas) return false;
	return win->m_canvas->is_GL_context_created();
#else
	THROW_EXCEPTION("This function requires wxWidgets and OpenGL");
#endif
}

bool CDisplayWindow3D::wait_for_GL_context(const double timeout_seconds) const
{
	const double t0 = mrpt::Clock::nowDouble();
	bool ok = false;
	while (!(ok = is_GL_context_created()) &&
		   mrpt::Clock::nowDouble() - t0 < timeout_seconds)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	return ok;
}
