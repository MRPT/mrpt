/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "gui-precomp.h"   // Precompiled headers

#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/round.h>

#include <mrpt/gui/WxSubsystem.h>
#include <mrpt/gui/WxUtils.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;


IMPLEMENTS_MRPT_OBJECT(CDisplayWindow,CBaseGUIWindow,mrpt::gui)


#if MRPT_HAS_WXWIDGETS


BEGIN_EVENT_TABLE(CWindowDialog,wxFrame)

END_EVENT_TABLE()



const long CWindowDialog::ID_IMAGE_BITMAP = wxNewId();
const long ID_MENUITEM1 = wxNewId();
const long ID_MENUITEM2 = wxNewId();
const long ID_MENUITEM3 = wxNewId();


CWindowDialog::wxMRPTImageControl::wxMRPTImageControl(
	wxWindow *parent,
	wxWindowID winID,
	int x, int y, int width, int height ) : m_img(NULL)
{
	this->Create(parent,winID,wxPoint(x,y),wxSize(width,height));

	Connect(wxEVT_PAINT, wxPaintEventHandler(CWindowDialog::wxMRPTImageControl::OnPaint));
	Connect(wxEVT_MOTION, wxMouseEventHandler(CWindowDialog::wxMRPTImageControl::OnMouseMove));
    Connect(wxID_ANY,wxEVT_LEFT_DOWN,wxMouseEventHandler(CWindowDialog::wxMRPTImageControl::OnMouseClick));

	Connect(wxID_ANY,wxEVT_CHAR,(wxObjectEventFunction)&CWindowDialog::wxMRPTImageControl::OnChar);
	Connect(wxEVT_CHAR,(wxObjectEventFunction)&CWindowDialog::wxMRPTImageControl::OnChar);

//	Connect(wxID_ANY,wxEVT_CHAR,(wxObjectEventFunction)&CWindowDialog::wxMRPTImageControl::OnChar);
//	Connect(wxID_ANY,wxEVT_KEY_DOWN,(wxObjectEventFunction)&CWindowDialog::wxMRPTImageControl::OnChar);
}

CWindowDialog::wxMRPTImageControl::~wxMRPTImageControl()
{
	mrpt::synch::CCriticalSectionLocker	lock(& m_img_cs);
	if (m_img)
	{
		delete m_img;
		m_img=NULL;
	}
}

void CWindowDialog::wxMRPTImageControl::OnMouseMove(wxMouseEvent& ev)
{
	//mrpt::synch::CCriticalSectionLocker	lock(& m_mouse_cs);
	m_last_mouse_point = ev.GetPosition();
}

void CWindowDialog::wxMRPTImageControl::OnMouseClick(wxMouseEvent& ev)
{
	//mrpt::synch::CCriticalSectionLocker	lock(& m_mouse_cs);
	m_last_mouse_click= ev.GetPosition();
}

void CWindowDialog::wxMRPTImageControl::OnChar(wxKeyEvent & ev)
{
}

void CWindowDialog::wxMRPTImageControl::AssignImage(wxBitmap *img)
{
	mrpt::synch::CCriticalSectionLocker	lock(& m_img_cs);
	if (m_img)
	{
		delete m_img;
		m_img=NULL;
	}

	m_img = img;
}

void CWindowDialog::wxMRPTImageControl::OnPaint(wxPaintEvent &ev)
{
	wxPaintDC dc(this);

	mrpt::synch::CCriticalSectionLocker	lock(& m_img_cs);
	if (!m_img)
	{
		// Erase background:
		return;
	}

	dc.DrawBitmap(*m_img,0,0);
}

void CWindowDialog::wxMRPTImageControl::GetBitmap(wxBitmap &bmp)
{
	mrpt::synch::CCriticalSectionLocker	lock(& m_img_cs);
	if (!m_img) return;
	bmp = *m_img;
}


CWindowDialog::CWindowDialog(
    CDisplayWindow *win2D,
    WxSubsystem::CWXMainFrame* parent,
    wxWindowID id,
    const std::string &caption,
    wxSize initialSize ) :
    m_win2D( win2D ),
    m_mainFrame(parent)
{
    Create(
        parent,
        id,
        _U(caption.c_str()),
        wxDefaultPosition,
        initialSize,
        wxDEFAULT_FRAME_STYLE,
        _T("id"));
    SetClientSize(initialSize);

	wxIcon FrameIcon;
	FrameIcon.CopyFromBitmap(mrpt::gui::WxSubsystem::getMRPTDefaultIcon());
	SetIcon(FrameIcon);

    // Create the image object:
	m_image = new CWindowDialog::wxMRPTImageControl(this,ID_IMAGE_BITMAP,0,0,10,10);

	// wxCLIP_CHILDREN seems to avoid flicker
	SetWindowStyle( GetWindowStyle() | wxCLIP_CHILDREN );

    // Menu:
    wxMenuBar *MenuBar1 = new wxMenuBar();

    wxMenu *Menu1 = new wxMenu();
    wxMenuItem *MenuItem3 = new wxMenuItem(Menu1, ID_MENUITEM3, _("Save to file..."), _(""), wxITEM_NORMAL);
    Menu1->Append(MenuItem3);
    wxMenuItem *MenuItem1 = new wxMenuItem(Menu1, ID_MENUITEM1, _("Close"), _(""), wxITEM_NORMAL);
    Menu1->Append(MenuItem1);
    MenuBar1->Append(Menu1, _("&File"));

    wxMenu *Menu2 = new wxMenu();
    wxMenuItem *MenuItem2 = new wxMenuItem(Menu2, ID_MENUITEM2, _("About..."), _(""), wxITEM_NORMAL);
    Menu2->Append(MenuItem2);
    MenuBar1->Append(Menu2, _("&Help"));

    SetMenuBar(MenuBar1);


    // Events:
    Connect(wxID_ANY,wxEVT_CLOSE_WINDOW,(wxObjectEventFunction)&CWindowDialog::OnClose);
    Connect(ID_MENUITEM1,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&CWindowDialog::OnMenuClose);
    Connect(ID_MENUITEM2,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&CWindowDialog::OnMenuAbout);
    Connect(ID_MENUITEM3,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&CWindowDialog::OnMenuSave);

//	Connect(wxID_ANY,wxEVT_CHAR,(wxObjectEventFunction)&CWindowDialog::OnChar);
	Connect(wxID_ANY,wxEVT_KEY_DOWN,(wxObjectEventFunction)&CWindowDialog::OnChar);
	//Connect(wxID_ANY,wxEVT_CHAR,(wxObjectEventFunction)&CWindowDialog::OnChar);
	Connect(wxEVT_CHAR,(wxObjectEventFunction)&CWindowDialog::OnChar);

	m_image->Connect(wxID_ANY,wxEVT_KEY_DOWN,(wxObjectEventFunction)&CWindowDialog::OnChar,NULL,this);
	//m_image->Connect(wxEVT_CHAR,(wxObjectEventFunction)&CWindowDialog::OnChar,NULL,this);
	m_image->Connect(wxEVT_SIZE,(wxObjectEventFunction)&CWindowDialog::OnResize,NULL,this);

	m_image->Connect(wxEVT_LEFT_DOWN,(wxObjectEventFunction)&CWindowDialog::OnMouseDown,NULL,this);
	m_image->Connect(wxEVT_RIGHT_DOWN,(wxObjectEventFunction)&CWindowDialog::OnMouseDown,NULL,this);

    // Increment number of windows:
    //int winCount =
    WxSubsystem::CWXMainFrame::notifyWindowCreation();

	//this->Iconize(false);
}

// Destructor
CWindowDialog::~CWindowDialog()
{
}

// OnClose event:
void CWindowDialog::OnClose(wxCloseEvent& event)
{
	// Send the event:
	bool allow_close=true;
	try {
		mrptEventWindowClosed ev(m_win2D, true /* allow close */);
		m_win2D->publishEvent(ev);
		allow_close = ev.allow_close;
	} catch(...){}
	if (!allow_close) return; // Don't process this close event.

	// Set the m_hwnd=NULL in our parent object.
    m_win2D->notifyChildWindowDestruction();

    // Decrement number of windows:
    WxSubsystem::CWXMainFrame::notifyWindowDestruction();

	// Signal we are destroyed:
    m_win2D->m_semWindowDestroyed.release();

    event.Skip(); // keep processing by parent classes.
}

void CWindowDialog::OnKeyDown(wxKeyEvent& event)
{
	event.Skip(); // So OnChar event is produced.
}

void CWindowDialog::OnChar(wxKeyEvent& event)
{
	if (m_win2D)
	{
		const int 				code = event.GetKeyCode();
		const mrptKeyModifier 	mod = mrpt::gui::keyEventToMrptKeyModifier(event);

		m_win2D->m_keyPushedCode = code;
		m_win2D->m_keyPushedModifier = mod;
		m_win2D->m_keyPushed = true;

		// Send the event:
		try {
			m_win2D->publishEvent( mrptEventWindowChar(m_win2D,code,mod));
		} catch(...){}
	}
	event.Skip();
}

void CWindowDialog::OnResize(wxSizeEvent& event)
{
	// Send the event:
	if (m_win2D)
	{
		try {
			m_win2D->publishEvent( mrptEventWindowResize(m_win2D,event.GetSize().GetWidth(),event.GetSize().GetHeight()));
		} catch(...) { }
	}
	event.Skip(); // so it's processed by the wx system!
}

void CWindowDialog::OnMouseDown(wxMouseEvent& event)
{
	// Send the event:
	if (m_win2D)
	{
		try {
			m_win2D->publishEvent( mrptEventMouseDown(m_win2D, TPixelCoord(event.GetX(), event.GetY()), event.LeftDown(), event.RightDown() ) );
		} catch(...) { }
	}
	event.Skip(); // so it's processed by the wx system!
}


// Menu: Close
void CWindowDialog::OnMenuClose(wxCommandEvent& event)
{
    Close();
}
// Menu: About
void CWindowDialog::OnMenuAbout(wxCommandEvent& event)
{
    ::wxMessageBox(_("Image viewer\n Class gui::CDisplayWindow\n MRPT C++ library"),_("About..."));
}

// Menu: Save to file
void CWindowDialog::OnMenuSave(wxCommandEvent& event)
{
    wxFileDialog dialog(
        this,
        wxT("Save image as..."),
        wxT("."),
        wxT("image.png"),
        wxT("PNG image files (*.png)|*.png"),
#if wxCHECK_VERSION(2, 8, 0)
        wxFD_SAVE | wxFD_OVERWRITE_PROMPT );
#else
        wxFD_SAVE | wxFD_OVERWRITE_PROMPT );
#endif

    if (wxID_OK == dialog.ShowModal())
    {
        try
        {
            wxBitmap  bmp;
			m_image->GetBitmap(bmp);
            bmp.SaveFile( dialog.GetPath(), wxBITMAP_TYPE_PNG );
        }
        catch(...)
        {
        }
    }
}


#endif

CDisplayWindowPtr CDisplayWindow::Create(
	const std::string	&windowCaption,
	unsigned int initWidth,
	unsigned int initHeight )
{
	return CDisplayWindowPtr(new CDisplayWindow(windowCaption,initWidth,initHeight));
}
/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CDisplayWindow::CDisplayWindow( const std::string &windowCaption, unsigned int initWidth, unsigned int initHeight )
	: CBaseGUIWindow(static_cast<void*>(this),200,299, windowCaption),
	  m_enableCursorCoordinates( true )
{
	CBaseGUIWindow::createWxWindow(initWidth, initHeight);
}

/*---------------------------------------------------------------
					Destructor
 ---------------------------------------------------------------*/
CDisplayWindow::~CDisplayWindow( )
{
	CBaseGUIWindow::destroyWxWindow();
}

/** Set cursor style to default (cursorIsCross=false) or to a cross (cursorIsCross=true) */
void CDisplayWindow::setCursorCross(bool cursorIsCross)
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	const CWindowDialog *win = (const CWindowDialog*) m_hwnd.get();
	if (!win) return;
	win->m_image->SetCursor( *(cursorIsCross ? wxCROSS_CURSOR : wxSTANDARD_CURSOR) );
#else
	MRPT_UNUSED_PARAM(cursorIsCross);
#endif
}

/*---------------------------------------------------------------
					getLastMousePosition
 ---------------------------------------------------------------*/
bool CDisplayWindow::getLastMousePosition(int &x, int &y) const
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	const CWindowDialog *win = (const CWindowDialog*) m_hwnd.get();
	if (!win) return false;
	x = win->m_image->m_last_mouse_point.x;
	y = win->m_image->m_last_mouse_point.y;
	return true;
#else
	MRPT_UNUSED_PARAM(x); MRPT_UNUSED_PARAM(y);
    return false;
#endif
}



/*---------------------------------------------------------------
					showImage
 ---------------------------------------------------------------*/
void  CDisplayWindow::showImage( const	CImage& img )
{
#if MRPT_HAS_WXWIDGETS
	MRPT_START

    // Send message of new image:
	wxImage *newImg = mrpt::gui::MRPTImage2wxImage( img );

    // Send a request to destroy this object:
    WxSubsystem::TRequestToWxMainThread *REQ = new WxSubsystem::TRequestToWxMainThread[1];
    REQ->source2D = this;
    REQ->OPCODE   = 201;
    REQ->voidPtr  = m_hwnd.get();
    REQ->voidPtr2 = (void*) newImg;
    WxSubsystem::pushPendingWxRequest( REQ );


	MRPT_END
#else
	MRPT_UNUSED_PARAM(img);
#endif
}

/*---------------------------------------------------------------
					showImageAndPoints
 ---------------------------------------------------------------*/
void  CDisplayWindow::showImageAndPoints( const CImage &img, const CVectorFloat &x_, const CVectorFloat &y_, const TColor &color, const bool &showNumbers )
{
	std::vector<float> x(x_.size()),y(y_.size());
	for (size_t i=0;i<x.size();i++) x[i]=x_[i];
	for (size_t i=0;i<y.size();i++) y[i]=y_[i];
	showImageAndPoints(img,x,y,color,showNumbers);
}


void  CDisplayWindow::showImageAndPoints( const CImage &img, const std::vector<float> &x, const std::vector<float> &y, const TColor &color, const bool &showNumbers )
{
#if MRPT_HAS_WXWIDGETS
	MRPT_START
	ASSERT_( x.size() == y.size() );

	CImage imgColor(1,1,3);
	img.colorImage( imgColor );	// Create a colorimage
	for(size_t i = 0; i < x.size(); i++)
	{
		imgColor.cross(round(x[i]),round(y[i]),color,'+');

		if( showNumbers )
		{
			char buf[15];
			mrpt::system::os::sprintf( buf, 15, "%d", int(i) );
			imgColor.textOut( round(x[i]) - 10, round(y[i]), buf, color );
		}
	} // end-for
	showImage(imgColor);
	MRPT_END
#else
	MRPT_UNUSED_PARAM(img); MRPT_UNUSED_PARAM(x); MRPT_UNUSED_PARAM(y);
	MRPT_UNUSED_PARAM(color); MRPT_UNUSED_PARAM(showNumbers);
#endif
}

/*---------------------------------------------------------------
					plot
 ---------------------------------------------------------------*/
void  CDisplayWindow::plot( const CVectorFloat &x, const CVectorFloat &y )
{
	MRPT_START

	ASSERT_( x.size() == y.size() );

	const int ox = 40;
	const int oy = 40;

	// Suboptimal but...
	CImage imgColor(1,1,3);

	imgColor.resize( 640, 480, 3, 0 );
	// Draw axis:
	imgColor.filledRectangle( 0, 0, 640, 480, TColor(255,255,255) );
	imgColor.line( 40, 40, 560, 40, TColor::black, 3 );
	imgColor.line( 40, 40, 40, 440, TColor::black, 3 );
	imgColor.line( 560, 40, 555, 45, TColor::black, 3 );
	imgColor.line( 560, 40, 555, 35, TColor::black, 3 );
	imgColor.line( 40, 440, 35, 435, TColor::black, 3 );
	imgColor.line( 40, 440, 45, 435, TColor::black, 3 );

	//imgColor.textOut( 550, 25, "x", TColor::black );
	//imgColor.textOut( 25, 430, "y", TColor::black );

	CVectorFloat::const_iterator itx, ity;
	CVectorFloat::const_iterator itymx, itymn;
	itymx = std::max_element( y.begin(), y.end() );
	itymn = std::min_element( y.begin(), y.end() );
	float px = (x[x.size()-1] - x[0])/520;
	float py = (*itymx - *itymn)/400;

	float tpxA=0, tpyA=0;

	for( itx = x.begin(), ity = y.begin(); itx != x.end(); ++itx, ++ity )
	{
		float tpx = (*itx-x[0])/px + ox;
		float tpy = (*ity-*itymn)/py + oy;
		imgColor.cross( tpx, tpy, TColor(255,0,0), 'x' );
		if( itx != x.begin() )
			imgColor.line( tpxA, tpyA, tpx, tpy, TColor(0,0,255), 3 );
		tpxA = tpx;
		tpyA = tpy;
	} // end for

	showImage( imgColor );

	MRPT_END
}

/*---------------------------------------------------------------
					plot
 ---------------------------------------------------------------*/
void  CDisplayWindow::plot( const CVectorFloat &y )
{
	MRPT_START

	ASSERT_( y.size() >= 0 );

	const int ox = 40;
	const int oy = 40;

	// Suboptimal but...
	CImage imgColor(1,1,3);

	imgColor.resize( 640, 480, 3, 0 );
	// Draw axis:
	imgColor.filledRectangle( 0, 0, 640, 480, TColor::white);
	imgColor.line( 40, 40, 560, 40, TColor::black, 3 );
	imgColor.line( 40, 40, 40, 440, TColor::black, 3 );
	imgColor.line( 560, 40, 555, 45, TColor::black, 3 );
	imgColor.line( 560, 40, 555, 35, TColor::black, 3 );
	imgColor.line( 40, 440, 35, 435, TColor::black, 3 );
	imgColor.line( 40, 440, 45, 435, TColor::black, 3 );

	imgColor.textOut( 550, 25, "x", TColor::black );
	imgColor.textOut( 25, 430, "y", TColor::black );

	CVectorFloat::const_iterator ity;
	CVectorFloat::const_iterator itymx, itymn;
	itymx = std::max_element( y.begin(), y.end() );
	itymn = std::min_element( y.begin(), y.end() );
	float px = y.size()/520.0f;
	float py = (*itymx - *itymn)/400.0f;
	float tpxA=0, tpyA=0;

	unsigned int k = 0;

	for( k = 0, ity = y.begin(); ity != y.end(); ++k, ++ity )
	{
		float tpx = k/px + ox;
		float tpy = (*ity-*itymn)/py + oy;
		imgColor.cross( tpx, tpy, TColor::red, 'x' );
		if( k > 0 )
			imgColor.line( tpxA, tpyA, tpx, tpy, TColor::blue, 3 );
		tpxA = tpx;
		tpyA = tpy;
	} // end for

	showImage( imgColor );

	MRPT_END
}

/*---------------------------------------------------------------
					resize
 ---------------------------------------------------------------*/
void  CDisplayWindow::resize(
	unsigned int width,
	unsigned int height )
{
#if MRPT_HAS_WXWIDGETS
	if (!isOpen())
	{
		cerr << "[CDisplayWindow::resize] Window closed!: " << m_caption << endl;
		return;
	}

    // Send a request to destroy this object:
    WxSubsystem::TRequestToWxMainThread *REQ = new WxSubsystem::TRequestToWxMainThread[1];
    REQ->source2D = this;
    REQ->OPCODE   = 203;
    REQ->x        = width;
    REQ->y        = height;
    WxSubsystem::pushPendingWxRequest( REQ );
#else
	MRPT_UNUSED_PARAM(width); MRPT_UNUSED_PARAM(height);
#endif
}

/*---------------------------------------------------------------
					setPos
 ---------------------------------------------------------------*/
void  CDisplayWindow::setPos( int x, int y )
{
#if MRPT_HAS_WXWIDGETS
	if (!isOpen())
	{
		cerr << "[CDisplayWindow::setPos] Window closed!: " << m_caption << endl;
		return;
	}

    // Send a request to destroy this object:
    WxSubsystem::TRequestToWxMainThread *REQ = new WxSubsystem::TRequestToWxMainThread[1];
    REQ->source2D = this;
    REQ->OPCODE   = 202;
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
void  CDisplayWindow::setWindowTitle( const std::string &str )
{
#if MRPT_HAS_WXWIDGETS
	if (!isOpen())
	{
		cerr << "[CDisplayWindow::setWindowTitle] Window closed!: " << m_caption << endl;
		return;
	}

    // Send a request to destroy this object:
    WxSubsystem::TRequestToWxMainThread *REQ = new WxSubsystem::TRequestToWxMainThread[1];
    REQ->source2D = this;
    REQ->OPCODE   = 204;
    REQ->str      = str;
    WxSubsystem::pushPendingWxRequest( REQ );
#else
	MRPT_UNUSED_PARAM(str);
#endif
}
