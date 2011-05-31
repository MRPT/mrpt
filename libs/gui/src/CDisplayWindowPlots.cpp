/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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

#include <mrpt/gui/CDisplayWindowPlots.h>

#include <mrpt/system/os.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/utils/metaprogramming.h>

#include <mrpt/gui/WxSubsystem.h>
#include <mrpt/gui/WxUtils.h>

#include <mrpt/math/utils.h>

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_MRPT_OBJECT(CDisplayWindowPlots,CBaseGUIWindow,mrpt::gui)


#if MRPT_HAS_WXWIDGETS

BEGIN_EVENT_TABLE(CWindowDialogPlots,wxFrame)

END_EVENT_TABLE()


const long CWindowDialogPlots::ID_PLOT = wxNewId();
const long CWindowDialogPlots::ID_MENU_PRINT = wxNewId();
const long ID_MENUITEM1 = wxNewId();
const long ID_MENUITEM2 = wxNewId();


CWindowDialogPlots::CWindowDialogPlots(
    CDisplayWindowPlots *winPlots,
    WxSubsystem::CWXMainFrame* parent,
    wxWindowID id,
    const std::string &caption,
    wxSize initialSize )
    :
    m_winPlots( winPlots ),
    m_mainFrame(parent),
	m_firstSubmenu(true)
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


    // Create the mpWindow object:
    m_plot = new mpWindow( this, ID_PLOT );
    m_plot->AddLayer( new mpScaleX() );
    m_plot->AddLayer( new mpScaleY() );
    m_plot->LockAspect( false );
    m_plot->EnableDoubleBuffer(true);

    m_plot->Fit( -10,10,-10,10 );

    // Menu:
    wxMenuBar *MenuBar1 = new wxMenuBar();

    wxMenu *Menu1 = new wxMenu();
    wxMenuItem *MenuItem1 = new wxMenuItem(Menu1, ID_MENUITEM1, _("Close"), _(""), wxITEM_NORMAL);
    Menu1->Append(MenuItem1);

    wxMenuItem *MenuItemPrint = new wxMenuItem(Menu1, ID_MENU_PRINT, _("Print..."), _(""), wxITEM_NORMAL);
    Menu1->Append(MenuItemPrint);

    MenuBar1->Append(Menu1, _("&File"));

    wxMenu *Menu2 = new wxMenu();
    wxMenuItem *MenuItem2 = new wxMenuItem(Menu2, ID_MENUITEM2, _("About..."), _(""), wxITEM_NORMAL);
    Menu2->Append(MenuItem2);
    MenuBar1->Append(Menu2, _("&Help"));

    SetMenuBar(MenuBar1);


    // Events:
    Connect(wxID_ANY,wxEVT_CLOSE_WINDOW,(wxObjectEventFunction)&CWindowDialogPlots::OnClose);
    Connect(ID_MENUITEM1,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&CWindowDialogPlots::OnMenuClose);
    Connect(ID_MENU_PRINT,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&CWindowDialogPlots::OnMenuPrint);
    Connect(ID_MENUITEM2,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&CWindowDialogPlots::OnMenuAbout);

	Connect(wxID_ANY,wxEVT_SIZE,(wxObjectEventFunction)&CWindowDialogPlots::OnResize);

	Connect(wxID_ANY,wxEVT_CHAR,(wxObjectEventFunction)&CWindowDialogPlots::OnChar);
	m_plot->Connect(wxEVT_CHAR,(wxObjectEventFunction)&CWindowDialogPlots::OnChar,0,this);
	m_plot->Connect(wxEVT_MOTION,(wxObjectEventFunction)&CWindowDialogPlots::OnMouseMove,0,this);

	m_plot->Connect(wxEVT_LEFT_DOWN,(wxObjectEventFunction)&CWindowDialogPlots::OnMouseDown,NULL,this);
	m_plot->Connect(wxEVT_RIGHT_DOWN,(wxObjectEventFunction)&CWindowDialogPlots::OnMouseDown,NULL,this);

    // Increment number of windows:
    //int winCount =
    WxSubsystem::CWXMainFrame::notifyWindowCreation();
    //cout << "[CWindowDialogPlots] Notifying new window: " << winCount << endl;

	//this->Iconize(false);

#if 0
	// JL: TEST CODE: This is the seed of the future new implementation based on wxFreeChart...
	double data[][2] = {
			{ 10, 20, },
			{ 13, 16, },
			{ 7, 30, },
			{ 15, 34, },
			{ 25, 4, },
	};
	// first step: create plot
	XYPlot *plot = new XYPlot();
	// create dataset
	XYSimpleDataset *dataset = new XYSimpleDataset();
	// and add serie to it
	dataset->AddSerie((double *) data, WXSIZEOF(data));
	// set line renderer to dataset
	dataset->SetRenderer(new XYLineRenderer());
	// add our dataset to plot
	plot->AddDataset(dataset);
	// create left and bottom number axes
	NumberAxis *leftAxis = new NumberAxis(AXIS_LEFT);
	NumberAxis *bottomAxis = new NumberAxis(AXIS_BOTTOM);
	// optional: set axis titles
	leftAxis->SetTitle(wxT("X"));
	bottomAxis->SetTitle(wxT("Y"));
	// add axes to plot
	plot->AddAxis(leftAxis);
	plot->AddAxis(bottomAxis);
	// link axes and dataset
	plot->LinkDataVerticalAxis(0, 0);
	plot->LinkDataHorizontalAxis(0, 0);
	// and finally create chart
	Chart* chart = new Chart(plot, wxT("my title"));
	wxChartPanel *m_chartPanel = new wxChartPanel( this ); //, ID_PLOT );
	m_chartPanel->SetChart( chart );
#endif
}

// Destructor
CWindowDialogPlots::~CWindowDialogPlots()
{
}

// OnClose event:
void CWindowDialogPlots::OnClose(wxCloseEvent& event)
{
    // Set the m_hwnd=NULL in our parent object.
    m_winPlots->notifyChildWindowDestruction();

    // Decrement number of windows:
    WxSubsystem::CWXMainFrame::notifyWindowDestruction();

	// Signal we are destroyed:
    m_winPlots->m_semWindowDestroyed.release();

    event.Skip(); // keep processing by parent classes.
}

void CWindowDialogPlots::OnChar(wxKeyEvent& event)
{
	if (m_winPlots)
	{
		const int 				code = event.GetKeyCode();
		const mrptKeyModifier 	mod = mrpt::gui::keyEventToMrptKeyModifier(event);

		m_winPlots->m_keyPushedCode = code;
		m_winPlots->m_keyPushedModifier = mod;
		m_winPlots->m_keyPushed = true;
		// Send the event:
		try {
			m_winPlots->publishEvent( mrptEventWindowChar(m_winPlots,code,mod));
		} catch(...){}
	}
	event.Skip();
}

void CWindowDialogPlots::OnResize(wxSizeEvent& event)
{
	// Send the event:
	if (m_winPlots)
	{
		try {
			m_winPlots->publishEvent( mrptEventWindowResize(m_winPlots,event.GetSize().GetWidth(),event.GetSize().GetHeight()));
		} catch(...) {}
	}
	event.Skip(); // so it's processed by the wx system!
}

void CWindowDialogPlots::OnMouseDown(wxMouseEvent& event)
{
	// Send the event:
	if (m_winPlots)
	{
		try {
			m_winPlots->publishEvent( mrptEventMouseDown(m_winPlots, TPixelCoord(event.GetX(), event.GetY()), event.LeftDown(), event.RightDown() ) );
		} catch(...) { }
	}
	event.Skip(); // so it's processed by the wx system!
}



// Menu: Close
void CWindowDialogPlots::OnMenuClose(wxCommandEvent& event)
{
    Close();
}
// Menu: Print
void CWindowDialogPlots::OnMenuPrint(wxCommandEvent& event)
{
	m_plot->ShowPrintDialog();
}
// Menu: About
void CWindowDialogPlots::OnMenuAbout(wxCommandEvent& event)
{
    ::wxMessageBox(_("Plot viewer\n Class gui::CDisplayWindowPlots\n MRPT C++ & wxMathPlot library"),_("About..."));
}

void CWindowDialogPlots::OnMenuSelected(wxCommandEvent& ev)
{
	std::map<long,long>::const_iterator it = m_ID2ID.find(ev.GetId());
	if (it!=m_ID2ID.end())
	{
		if (m_winPlots && m_winPlots->m_callback)
			m_winPlots->m_callback(it->second,m_curCursorPos.x,m_curCursorPos.y,  m_winPlots->m_callback_param);
	}
}

void CWindowDialogPlots::OnMouseMove(wxMouseEvent& event)
{
	int X, Y;
	event.GetPosition(&X,&Y);
    m_curCursorPos.x = m_plot->p2x(X);
    m_curCursorPos.y = m_plot->p2y(Y);
	m_last_mouse_point.x = X;
	m_last_mouse_point.y = Y;
    event.Skip();
}

// Add / Modify a 2D plot using a MATLAB-like format string
void CWindowDialogPlots::plot(
	const vector_float &x,
	const vector_float &y,
	const std::string  &lineFormat,
	const std::string  &plotName)
{
	mpFXYVector  *theLayer;

	wxString    lyName = _U(plotName.c_str());
	bool        updateAtTheEnd = false; // If we update an existing layer, update manually to refresh the changes!

	// Already existing layer?
	mpLayer* existingLy = m_plot->GetLayerByName( lyName );

	if (existingLy)
	{
		// Assure the class:
		mpFXYVector  *lyPlot2D = static_cast<mpFXYVector*> ( existingLy );

		if (!lyPlot2D)
		{
			cerr << "[CWindowDialogPlots::plot] Plot name '" << plotName << "' is not of expected class mpFXYVector!."<< endl;
			return;
		}

		// Ok:
		theLayer = lyPlot2D;
		updateAtTheEnd = true;
	}
	else
	{
		// Create it:
		theLayer = new mpFXYVector( lyName );
		m_plot->AddLayer( theLayer );
	}

	// Set data:
	{
		std::vector<float> x_(x.size()),y_(x.size());
		::memcpy(&x_[0],&x[0],sizeof(x[0])*x_.size());
		::memcpy(&y_[0],&y[0],sizeof(y[0])*y_.size());
		theLayer->SetData( x_,y_ );
	}

	// Line style:
	// -------------------
	bool isContinuous=true;
	int  lineColor[] = {0,0,255};
	int  lineWidth = 1;
	int  lineStyle = wxSOLID;

	// parse string:
	if ( string::npos != lineFormat.find(".") )		{ isContinuous=false; }
	if ( string::npos != lineFormat.find("-") )		{ isContinuous=true; lineStyle = wxSOLID; }
	if ( string::npos != lineFormat.find(":") )		{ isContinuous=true; lineStyle = wxLONG_DASH; }

	if ( string::npos != lineFormat.find("r") )		{ lineColor[0]=0xFF; lineColor[1]=0x00; lineColor[2]=0x00; }
	if ( string::npos != lineFormat.find("g") )		{ lineColor[0]=0x00; lineColor[1]=0xFF; lineColor[2]=0x00; }
	if ( string::npos != lineFormat.find("b") )		{ lineColor[0]=0x00; lineColor[1]=0x00; lineColor[2]=0xFF; }
	if ( string::npos != lineFormat.find("k") )		{ lineColor[0]=0x00; lineColor[1]=0x00; lineColor[2]=0x00; }
	if ( string::npos != lineFormat.find("m") )		{ lineColor[0]=192; lineColor[1]=0; lineColor[2]=192; }
	if ( string::npos != lineFormat.find("c") )		{ lineColor[0]=0; lineColor[1]=192; lineColor[2]=192; }

	if ( string::npos != lineFormat.find("1") )		{ lineWidth=1; }
	if ( string::npos != lineFormat.find("2") )		{ lineWidth=2; }
	if ( string::npos != lineFormat.find("3") )		{ lineWidth=3; }
	if ( string::npos != lineFormat.find("4") )		{ lineWidth=4; }
	if ( string::npos != lineFormat.find("5") )		{ lineWidth=5; }
	if ( string::npos != lineFormat.find("6") )		{ lineWidth=6; }
	if ( string::npos != lineFormat.find("7") )		{ lineWidth=7; }
	if ( string::npos != lineFormat.find("8") )		{ lineWidth=8; }
	if ( string::npos != lineFormat.find("9") )		{ lineWidth=9; }

	theLayer->SetContinuity(isContinuous);

	wxPen  pen( wxColour(lineColor[0],lineColor[1],lineColor[2]), lineWidth, lineStyle );
	theLayer->SetPen(pen);

	theLayer->ShowName(false);

	if (updateAtTheEnd)
		m_plot->Refresh(false);

}

// Add / Modify a 2D ellipse
// x[0,1]: Mean
// y[0,1,2]: Covariance matrix (0,0),(1,1),(0,1)
void CWindowDialogPlots::plotEllipse(
	const vector_float &x,
	const vector_float &y,
	const std::string  &lineFormat,
	const std::string  &plotName,
	bool showName)
{
	mpCovarianceEllipse  *theLayer;

	if (x.size()!=3 || y.size()!=3)
	{
		cerr << "[CWindowDialogPlots::plotEllipse] vectors do not have expected size!!" << endl;
		return;
	}

	wxString    lyName = _U(plotName.c_str());
	bool        updateAtTheEnd = false; // If we update an existing layer, update manually to refresh the changes!

	// Already existing layer?
	mpLayer* existingLy = m_plot->GetLayerByName( lyName );

	if (existingLy)
	{
		// Assure the class:
		mpCovarianceEllipse  *lyPlotEllipse = static_cast<mpCovarianceEllipse*> ( existingLy );

		if (!lyPlotEllipse)
		{
			cerr << "[CWindowDialogPlots::plotEllipse] Plot name '" << plotName << "' is not of expected class mpCovarianceEllipse!."<< endl;
			return;
		}

		// Ok:
		theLayer = lyPlotEllipse;
		updateAtTheEnd = true;
	}
	else
	{
		// Create it:
		theLayer = new mpCovarianceEllipse( 1,1,0,2,32, lyName );
		m_plot->AddLayer( theLayer );
	}

	// Set data:
	theLayer->SetCovarianceMatrix(y[0],y[2],y[1]);
	theLayer->SetCoordinateBase(x[0],x[1]);
	theLayer->SetQuantiles(x[2]);
	theLayer->ShowName(showName);

	// Line style:
	// -------------------
	bool isContinuous=true;
	int  lineColor[] = {0,0,255};
	int  lineWidth = 1;
	int  lineStyle = wxSOLID;

	// parse string:
	if ( string::npos != lineFormat.find(".") )		{ isContinuous=false; }
	if ( string::npos != lineFormat.find("-") )		{ isContinuous=true; lineStyle = wxSOLID; }
	if ( string::npos != lineFormat.find(":") )		{ isContinuous=true; lineStyle = wxLONG_DASH; }

	if ( string::npos != lineFormat.find("r") )		{ lineColor[0]=0xFF; lineColor[1]=0x00; lineColor[2]=0x00; }
	if ( string::npos != lineFormat.find("g") )		{ lineColor[0]=0x00; lineColor[1]=0xFF; lineColor[2]=0x00; }
	if ( string::npos != lineFormat.find("b") )		{ lineColor[0]=0x00; lineColor[1]=0x00; lineColor[2]=0xFF; }
	if ( string::npos != lineFormat.find("k") )		{ lineColor[0]=0x00; lineColor[1]=0x00; lineColor[2]=0x00; }
	if ( string::npos != lineFormat.find("m") )		{ lineColor[0]=192; lineColor[1]=0; lineColor[2]=192; }
	if ( string::npos != lineFormat.find("c") )		{ lineColor[0]=0; lineColor[1]=192; lineColor[2]=192; }

	if ( string::npos != lineFormat.find("1") )		{ lineWidth=1; }
	if ( string::npos != lineFormat.find("2") )		{ lineWidth=2; }
	if ( string::npos != lineFormat.find("3") )		{ lineWidth=3; }
	if ( string::npos != lineFormat.find("4") )		{ lineWidth=4; }
	if ( string::npos != lineFormat.find("5") )		{ lineWidth=5; }
	if ( string::npos != lineFormat.find("6") )		{ lineWidth=6; }
	if ( string::npos != lineFormat.find("7") )		{ lineWidth=7; }
	if ( string::npos != lineFormat.find("8") )		{ lineWidth=8; }
	if ( string::npos != lineFormat.find("9") )		{ lineWidth=9; }

	theLayer->SetContinuity(isContinuous);

	wxPen  pen( wxColour(lineColor[0],lineColor[1],lineColor[2]), lineWidth, lineStyle );
	theLayer->SetPen(pen);

	if (updateAtTheEnd)
		m_plot->Refresh(false);
}


void CWindowDialogPlots::image(
	void *theWxImage,
	const float &x0,
	const float &y0,
	const float &w,
	const float &h,
	const std::string &plotName)
{
	mpBitmapLayer	*theLayer;

	wxString    lyName = _U(plotName.c_str());
	bool        updateAtTheEnd = false; // If we update an existing layer, update manually to refresh the changes!

	// Already existing layer?
	mpLayer* existingLy = m_plot->GetLayerByName( lyName );

	if (existingLy)
	{
		// Assure the class:
		mpBitmapLayer  *ly = static_cast<mpBitmapLayer*> ( existingLy );

		if (!ly)
		{
			cerr << "[CWindowDialogPlots::image] Plot name '" << plotName << "' is not of expected class mpBitmapLayer!."<< endl;
			return;
		}

		// Ok:
		theLayer = ly;
		updateAtTheEnd = true;
	}
	else
	{
		// Create it:
		theLayer = new mpBitmapLayer();
		m_plot->AddLayer( theLayer );
	}

	// Set data:
	wxImage *ii = static_cast<wxImage *>(theWxImage);
	theLayer->SetBitmap( *ii, x0,y0,w,h );

	delete ii;theWxImage=NULL;

	if (updateAtTheEnd) m_plot->Refresh();
}

#endif

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CDisplayWindowPlots::CDisplayWindowPlots(
	const std::string &windowCaption,
	unsigned int initialWidth,
	unsigned int initialHeight ) :
		CBaseGUIWindow(static_cast<void*>(this),400,499,  windowCaption),
		m_holdon	(false),
		m_holdon_just_disabled(false),
		m_holdon_cnt(0),
		m_callback(NULL),
		m_callback_param (NULL)
{
	CBaseGUIWindow::createWxWindow(initialWidth,initialHeight);
}

/*---------------------------------------------------------------
					Destructor
 ---------------------------------------------------------------*/
CDisplayWindowPlots::~CDisplayWindowPlots( )
{
	CBaseGUIWindow::destroyWxWindow();
}

/** Set cursor style to default (cursorIsCross=false) or to a cross (cursorIsCross=true) */
void CDisplayWindowPlots::setCursorCross(bool cursorIsCross)
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	const CWindowDialogPlots *win = (const CWindowDialogPlots*) m_hwnd.get();
	if (!win) return;
	win->m_plot->SetCursor( *(cursorIsCross ? wxCROSS_CURSOR : wxSTANDARD_CURSOR) );
#endif
}

/*---------------------------------------------------------------
					getLastMousePosition
 ---------------------------------------------------------------*/
bool CDisplayWindowPlots::getLastMousePosition(int &x, int &y) const
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	const CWindowDialogPlots *win = (const CWindowDialogPlots*) m_hwnd.get();
	if (!win) return false;
	x = win->m_last_mouse_point.x;
	y = win->m_last_mouse_point.y;
	return true;
#else
    return false;
#endif
}


/*---------------------------------------------------------------
					resize
 ---------------------------------------------------------------*/
void  CDisplayWindowPlots::resize(
	unsigned int width,
	unsigned int height )
{
#if MRPT_HAS_WXWIDGETS
	if (!isOpen())
	{
		cerr << "[CDisplayWindowPlots::resize] Window closed!: " << m_caption << endl;
		return;
	}

    // Send a request to destroy this object:
    WxSubsystem::TRequestToWxMainThread *REQ = new WxSubsystem::TRequestToWxMainThread[1];
    REQ->sourcePlots = this;
    REQ->OPCODE   = 403;
    REQ->x        = width;
    REQ->y        = height;
    WxSubsystem::pushPendingWxRequest( REQ );
#endif
}

/*---------------------------------------------------------------
					setPos
 ---------------------------------------------------------------*/
void  CDisplayWindowPlots::setPos( int x, int y )
{
#if MRPT_HAS_WXWIDGETS
	if (!isOpen())
	{
		cerr << "[CDisplayWindowPlots::setPos] Window closed!: " << m_caption << endl;
		return;
	}

    // Send a request to destroy this object:
    WxSubsystem::TRequestToWxMainThread *REQ = new WxSubsystem::TRequestToWxMainThread[1];
    REQ->sourcePlots = this;
    REQ->OPCODE   = 402;
    REQ->x        = x;
    REQ->y        = y;
    WxSubsystem::pushPendingWxRequest( REQ );
#endif
}

/*---------------------------------------------------------------
					setWindowTitle
 ---------------------------------------------------------------*/
void  CDisplayWindowPlots::setWindowTitle( const std::string &str )
{
#if MRPT_HAS_WXWIDGETS
	if (!isOpen())
	{
		cerr << "[CDisplayWindowPlots::setWindowTitle] Window closed!: " << m_caption << endl;
		return;
	}

    // Send a request to destroy this object:
    WxSubsystem::TRequestToWxMainThread *REQ = new WxSubsystem::TRequestToWxMainThread[1];
    REQ->sourcePlots = this;
    REQ->OPCODE   = 404;
    REQ->str      = str;
    WxSubsystem::pushPendingWxRequest( REQ );
#endif
}

/*---------------------------------------------------------------
					enableMousePanZoom
 ---------------------------------------------------------------*/
void  CDisplayWindowPlots::enableMousePanZoom( bool enabled )
{
#if MRPT_HAS_WXWIDGETS
	if (!isOpen()) return;

    // Send a request to destroy this object:
    WxSubsystem::TRequestToWxMainThread *REQ = new WxSubsystem::TRequestToWxMainThread[1];
    REQ->sourcePlots = this;
    REQ->OPCODE   = 410;
    REQ->boolVal  = enabled;
    WxSubsystem::pushPendingWxRequest( REQ );
#endif
}

/*---------------------------------------------------------------
					axis_equal
 ---------------------------------------------------------------*/
void  CDisplayWindowPlots::axis_equal( bool enabled )
{
#if MRPT_HAS_WXWIDGETS
	if (!isOpen()) return;

    // Send a request to destroy this object:
    WxSubsystem::TRequestToWxMainThread *REQ = new WxSubsystem::TRequestToWxMainThread[1];
    REQ->sourcePlots = this;
    REQ->OPCODE   = 411;
    REQ->boolVal  = enabled;
    WxSubsystem::pushPendingWxRequest( REQ );
#endif
}

/*---------------------------------------------------------------
					axis
 ---------------------------------------------------------------*/
void CDisplayWindowPlots::axis( float x_min, float x_max, float y_min, float y_max, bool aspectRatioFix )
{
#if MRPT_HAS_WXWIDGETS
	if (!isOpen()) return;

    // Send a request to destroy this object:
    WxSubsystem::TRequestToWxMainThread *REQ = new WxSubsystem::TRequestToWxMainThread[1];
    REQ->sourcePlots = this;
    REQ->OPCODE   = 412;
	REQ->vector_x.resize(2);
	REQ->vector_x[0] = x_min;
	REQ->vector_x[1] = x_max;
	REQ->vector_y.resize(2);
	REQ->vector_y[0] = y_min;
	REQ->vector_y[1] = y_max;
	REQ->boolVal  = aspectRatioFix;
    WxSubsystem::pushPendingWxRequest( REQ );
#endif
}

/*---------------------------------------------------------------
					axis_fit
 ---------------------------------------------------------------*/
void CDisplayWindowPlots::axis_fit(bool aspectRatioFix)
{
#if MRPT_HAS_WXWIDGETS
	if (!isOpen()) return;

    // Send a request to destroy this object:
    WxSubsystem::TRequestToWxMainThread *REQ = new WxSubsystem::TRequestToWxMainThread[1];
    REQ->sourcePlots = this;
    REQ->OPCODE   = 413;
	REQ->boolVal  = aspectRatioFix;
    WxSubsystem::pushPendingWxRequest( REQ );
#endif
}



/*---------------------------------------------------------------
					plotEllipse
 ---------------------------------------------------------------*/
template <typename T>
void CDisplayWindowPlots::plotEllipse(
	const T mean_x,
	const T mean_y,
	const CMatrixTemplateNumeric<T> &cov22,
	const float quantiles,
	const std::string  &lineFormat,
	const std::string  &plotName,
	bool showName)
{
	MRPT_START
#if MRPT_HAS_WXWIDGETS
	if (!isOpen()) return;

	ASSERT_(cov22.getColCount()==2 && cov22.getRowCount()==2);
	ASSERT_(cov22(0,0)>=0);
	ASSERT_(cov22(1,1)>=0);
	ASSERT_(cov22(0,1) == cov22(1,0) );

	if (m_holdon_just_disabled)
	{
		m_holdon_just_disabled=false;
		this->clf();
	}
	std::string holdon_post;
	if (m_holdon)
		holdon_post = format("_fig_%u",static_cast<unsigned int>(m_holdon_cnt++));

    // Send a request to destroy this object:
    WxSubsystem::TRequestToWxMainThread *REQ = new WxSubsystem::TRequestToWxMainThread[1];
    REQ->sourcePlots = this;
    REQ->OPCODE   = 421;
    // 421: Add/update a 2D ellipse: format string=str, plot name =plotName, vector_x[0,1]:X/Y center,  vector_y[0,1,2]: Covariance matrix entries 00,11,01.
    REQ->str      = lineFormat;
    REQ->plotName = plotName+holdon_post;

    REQ->vector_x.resize(3);
    REQ->vector_x[0]=mean_x;
    REQ->vector_x[1]=mean_y;
    REQ->vector_x[2]=quantiles;

    REQ->vector_y.resize(3);
    REQ->vector_y[0] = cov22(0,0);
    REQ->vector_y[1] = cov22(1,1);
    REQ->vector_y[2] = cov22(0,1);

	REQ->boolVal = showName;

    WxSubsystem::pushPendingWxRequest( REQ );
#endif
	MRPT_END
}

// Explicit instantations:
template void GUI_IMPEXP CDisplayWindowPlots::plotEllipse(
	const float mean_x,
	const float mean_y,
	const CMatrixTemplateNumeric<float> &cov22,
	const float quantiles,
	const std::string  &lineFormat,
	const std::string  &plotName,
	bool showName);
template void GUI_IMPEXP CDisplayWindowPlots::plotEllipse(
	const double mean_x,
	const double mean_y,
	const CMatrixTemplateNumeric<double> &cov22,
	const float quantiles,
	const std::string  &lineFormat,
	const std::string  &plotName,
	bool showName);

/*---------------------------------------------------------------
					plotEllipse
 ---------------------------------------------------------------*/
template <typename T>
void CDisplayWindowPlots::plotEllipse(
	const T mean_x,
	const T mean_y,
	const CMatrixFixedNumeric<T,2,2> &cov22,
	const float quantiles,
	const std::string  &lineFormat,
	const std::string  &plotName,
	bool showName)
{
	MRPT_START
#if MRPT_HAS_WXWIDGETS
	if (!isOpen()) return;

	ASSERT_(cov22(0,0)>=0);
	ASSERT_(cov22(1,1)>=0);
	ASSERT_(cov22(0,1) == cov22(1,0) );

	if (m_holdon_just_disabled)
	{
		m_holdon_just_disabled=false;
		this->clf();
	}
	std::string holdon_post;
	if (m_holdon)
		holdon_post = format("_fig_%u",static_cast<unsigned int>(m_holdon_cnt++));

    // Send a request to destroy this object:
    WxSubsystem::TRequestToWxMainThread *REQ = new WxSubsystem::TRequestToWxMainThread[1];
    REQ->sourcePlots = this;
    REQ->OPCODE   = 421;
    // 421: Add/update a 2D ellipse: format string=str, plot name =plotName, vector_x[0,1]:X/Y center,  vector_y[0,1,2]: Covariance matrix entries 00,11,01.
    REQ->str      = lineFormat;
    REQ->plotName = plotName+holdon_post;

    REQ->vector_x.resize(3);
    REQ->vector_x[0]=mean_x;
    REQ->vector_x[1]=mean_y;
    REQ->vector_x[2]=quantiles;

    REQ->vector_y.resize(3);
    REQ->vector_y[0] = cov22(0,0);
    REQ->vector_y[1] = cov22(1,1);
    REQ->vector_y[2] = cov22(0,1);

	REQ->boolVal = showName;

    WxSubsystem::pushPendingWxRequest( REQ );
#endif
	MRPT_END
}

// Explicit instantations:
template
void GUI_IMPEXP CDisplayWindowPlots::plotEllipse(
	const float mean_x,
	const float mean_y,
	const CMatrixFixedNumeric<float,2,2> &cov22,
	const float quantiles,
	const std::string  &lineFormat,
	const std::string  &plotName,
	bool showName);
template
void GUI_IMPEXP CDisplayWindowPlots::plotEllipse(
	const double mean_x,
	const double mean_y,
	const CMatrixFixedNumeric<double ,2,2> &cov22,
	const float quantiles,
	const std::string  &lineFormat,
	const std::string  &plotName,
	bool showName);


/*---------------------------------------------------------------
					image
 ---------------------------------------------------------------*/
void CDisplayWindowPlots::image(
	const utils::CImage &img,
	const float &x_left,
	const float &y_bottom,
	const float &x_width,
	const float &y_height,
	const std::string  &plotName  )
{
	MRPT_START
#if MRPT_HAS_WXWIDGETS
	if (!isOpen()) return;

	if (m_holdon_just_disabled)
	{
		m_holdon_just_disabled=false;
		this->clf();
	}
	std::string holdon_post;
	if (m_holdon)
		holdon_post = format("_fig_%u",static_cast<unsigned int>(m_holdon_cnt++));

    // Send a request to destroy this object:
    WxSubsystem::TRequestToWxMainThread *REQ = new WxSubsystem::TRequestToWxMainThread[1];
    REQ->sourcePlots = this;
    REQ->OPCODE   = 422;

    // 422: Add/update a bitmap: plot name =plotName, vector_x[0,1]:X/Y corner, vector_x[2,3]: X/Y widths, voidPtr2: pointer to a newly created wxImage with the bitmap.
    REQ->plotName = plotName+holdon_post;

    REQ->vector_x.resize(4);
    REQ->vector_x[0]=x_left;
    REQ->vector_x[1]=y_bottom;
    REQ->vector_x[2]=x_width;
    REQ->vector_x[3]=y_height;

	REQ->voidPtr2 = mrpt::gui::MRPTImage2wxImage(img);

    WxSubsystem::pushPendingWxRequest( REQ );
#endif
	MRPT_END
}

/*---------------------------------------------------------------
					internal_plot
 ---------------------------------------------------------------*/
void CDisplayWindowPlots::internal_plot(
	vector_float &x,
	vector_float &y,
	const std::string  &lineFormat,
	const std::string  &plotName)
{
	MRPT_START
#if MRPT_HAS_WXWIDGETS
	if (!isOpen()) return;

	ASSERT_EQUAL_(x.size(),y.size());

	if (m_holdon_just_disabled)
	{
		m_holdon_just_disabled=false;
		this->clf();
	}

	if (x.empty()) return;

	std::string holdon_post;
	if (m_holdon)
		holdon_post = format("_fig_%u",static_cast<unsigned int>(m_holdon_cnt++));

    // Send a request to destroy this object:
    WxSubsystem::TRequestToWxMainThread *REQ = new WxSubsystem::TRequestToWxMainThread[1];
    REQ->sourcePlots = this;
    REQ->OPCODE   = 420;
    REQ->str      = lineFormat;
    REQ->plotName = plotName + holdon_post;
    REQ->vector_x.swap(x);
    REQ->vector_y.swap(y);

	WxSubsystem::pushPendingWxRequest( REQ );
#endif
	MRPT_END
}



/*---------------------------------------------------------------
					clear
 ---------------------------------------------------------------*/
void CDisplayWindowPlots::clear()
{
	MRPT_START
#if MRPT_HAS_WXWIDGETS
	if (!isOpen()) return;

    // Send a request to destroy this object:
    WxSubsystem::TRequestToWxMainThread *REQ = new WxSubsystem::TRequestToWxMainThread[1];
    REQ->sourcePlots = this;
    REQ->OPCODE   = 414;

	// 414: Clear all plot objects.

    WxSubsystem::pushPendingWxRequest( REQ );
#endif
	MRPT_END
}

/*---------------------------------------------------------------
					hold_on
 ---------------------------------------------------------------*/
void CDisplayWindowPlots::hold_on()
{
	m_holdon = true;
}

/*---------------------------------------------------------------
					hold_off
 ---------------------------------------------------------------*/
void CDisplayWindowPlots::hold_off()
{
	if (m_holdon)
	{
		m_holdon = false;
		m_holdon_just_disabled = true;
	}
}




/*---------------------------------------------------------------
					addPopupMenuEntry
 ---------------------------------------------------------------*/
void CDisplayWindowPlots::addPopupMenuEntry(
	const std::string &label,
	int menuID
	)
{
	MRPT_START
#if MRPT_HAS_WXWIDGETS
	if (!isOpen()) return;

    WxSubsystem::TRequestToWxMainThread *REQ = new WxSubsystem::TRequestToWxMainThread[1];
    REQ->sourcePlots = this;
    REQ->OPCODE   = 440;
	REQ->plotName = label;
	REQ->x = menuID;
	// 440: Inser submenu in the popup menu.

    WxSubsystem::pushPendingWxRequest( REQ );
#endif
	MRPT_END
}

/*---------------------------------------------------------------
					setMenuCallback
 ---------------------------------------------------------------*/
void CDisplayWindowPlots::setMenuCallback(TCallbackMenu userFunction,void* userParam )
{
	ASSERT_(userFunction!=NULL)
	m_callback = userFunction;
	m_callback_param =userParam;
}
