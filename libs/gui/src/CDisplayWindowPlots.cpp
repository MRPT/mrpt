/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "gui-precomp.h"  // Precompiled headers
//
#include <mrpt/config.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/gui/WxSubsystem.h>
#include <mrpt/gui/WxUtils.h>
#include <mrpt/img/CImage.h>
#include <mrpt/math/utils.h>
#include <mrpt/system/os.h>

#include <cstring>

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;
using mrpt::img::TPixelCoord;

#if MRPT_HAS_WXWIDGETS

BEGIN_EVENT_TABLE(CWindowDialogPlots, wxFrame)

END_EVENT_TABLE()

const long CWindowDialogPlots::ID_PLOT = wxNewId();
const long CWindowDialogPlots::ID_MENU_PRINT = wxNewId();
const long ID_MENUITEM1 = wxNewId();
const long ID_MENUITEM2 = wxNewId();

CWindowDialogPlots::CWindowDialogPlots(
	CDisplayWindowPlots* winPlots, WxSubsystem::CWXMainFrame* parent,
	wxWindowID id, const std::string& caption, wxSize initialSize)
	: m_winPlots(winPlots), m_mainFrame(parent), m_firstSubmenu(true)
{
	Create(
		parent, id, caption.c_str(), wxDefaultPosition, initialSize,
		wxDEFAULT_FRAME_STYLE, _T("id"));

	SetClientSize(initialSize);

	wxIcon FrameIcon;
	FrameIcon.CopyFromBitmap(mrpt::gui::WxSubsystem::getMRPTDefaultIcon());
	SetIcon(FrameIcon);

	// Create the mpWindow object:
	m_plot = new mpWindow(this, ID_PLOT);
	m_plot->AddLayer(new mpScaleX());
	m_plot->AddLayer(new mpScaleY());
	m_plot->LockAspect(false);

	m_plot->Fit(-10, 10, -10, 10);

	// Menu:
	auto* MenuBar1 = new wxMenuBar();

	auto* Menu1 = new wxMenu();
	wxMenuItem* MenuItem1 =
		new wxMenuItem(Menu1, ID_MENUITEM1, _("Close"), _(""), wxITEM_NORMAL);
	Menu1->Append(MenuItem1);

	wxMenuItem* MenuItemPrint = new wxMenuItem(
		Menu1, ID_MENU_PRINT, _("Print..."), _(""), wxITEM_NORMAL);
	Menu1->Append(MenuItemPrint);

	MenuBar1->Append(Menu1, _("&File"));

	auto* Menu2 = new wxMenu();
	wxMenuItem* MenuItem2 = new wxMenuItem(
		Menu2, ID_MENUITEM2, _("About..."), _(""), wxITEM_NORMAL);
	Menu2->Append(MenuItem2);
	MenuBar1->Append(Menu2, _("&Help"));

	SetMenuBar(MenuBar1);

	// Events:
	Bind(wxEVT_CLOSE_WINDOW, &CWindowDialogPlots::OnClose, this, wxID_ANY);
	Bind(wxEVT_MENU, &CWindowDialogPlots::OnMenuClose, this, ID_MENUITEM1);
	Bind(wxEVT_MENU, &CWindowDialogPlots::OnMenuPrint, this, ID_MENU_PRINT);
	Bind(wxEVT_MENU, &CWindowDialogPlots::OnMenuAbout, this, ID_MENUITEM2);

	Bind(wxEVT_SIZE, &CWindowDialogPlots::OnResize, this, wxID_ANY);

	Bind(wxEVT_CHAR, &CWindowDialogPlots::OnChar, this, wxID_ANY);
	m_plot->Bind(wxEVT_CHAR, &CWindowDialogPlots::OnChar, this);
	m_plot->Bind(wxEVT_MOTION, &CWindowDialogPlots::OnMouseMove, this);

	m_plot->Bind(wxEVT_LEFT_DOWN, &CWindowDialogPlots::OnMouseDown, this);
	m_plot->Bind(wxEVT_RIGHT_DOWN, &CWindowDialogPlots::OnMouseDown, this);

	// Increment number of windows:
	// int winCount =
	WxSubsystem::CWXMainFrame::notifyWindowCreation();
	// this->Iconize(false);

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
CWindowDialogPlots::~CWindowDialogPlots() = default;
// OnClose event:
void CWindowDialogPlots::OnClose(wxCloseEvent& event)
{
	// Send the event:
	bool allow_close = true;
	try
	{
		mrptEventWindowClosed ev(m_winPlots, true /* allow close */);
		m_winPlots->publishEvent(ev);
		allow_close = ev.allow_close;
	}
	catch (...)
	{
	}
	if (!allow_close) return;  // Don't process this close event.

	// Set the m_hwnd=nullptr in our parent object.
	m_winPlots->notifyChildWindowDestruction();

	// Decrement number of windows:
	WxSubsystem::CWXMainFrame::notifyWindowDestruction();

	// Signal we are destroyed:
	m_winPlots->m_windowDestroyed.set_value();

	event.Skip();  // keep processing by parent classes.
}

void CWindowDialogPlots::OnChar(wxKeyEvent& event)
{
	if (m_winPlots)
	{
		const int code = event.GetKeyCode();
		const mrptKeyModifier mod = mrpt::gui::keyEventToMrptKeyModifier(event);

		auto lck = mrpt::lockHelper(m_winPlots->m_mtx);
		m_winPlots->m_keyPushedCode = code;
		m_winPlots->m_keyPushedModifier = mod;
		m_winPlots->m_keyPushed = true;
		// Send the event:
		try
		{
			m_winPlots->publishEvent(
				mrptEventWindowChar(m_winPlots, code, mod));
		}
		catch (...)
		{
		}
	}
	event.Skip();
}

void CWindowDialogPlots::OnResize(wxSizeEvent& event)
{
	// Send the event:
	if (m_winPlots)
	{
		try
		{
			m_winPlots->publishEvent(mrptEventWindowResize(
				m_winPlots, event.GetSize().GetWidth(),
				event.GetSize().GetHeight()));
		}
		catch (...)
		{
		}
	}
	event.Skip();  // so it's processed by the wx system!
}

void CWindowDialogPlots::OnMouseDown(wxMouseEvent& event)
{
	// Send the event:
	if (m_winPlots)
	{
		try
		{
			m_winPlots->publishEvent(mrptEventMouseDown(
				m_winPlots, TPixelCoord(event.GetX(), event.GetY()),
				event.LeftDown(), event.RightDown()));
		}
		catch (...)
		{
		}
	}
	event.Skip();  // so it's processed by the wx system!
}

// Menu: Close
void CWindowDialogPlots::OnMenuClose(wxCommandEvent& event) { Close(); }
// Menu: Print
void CWindowDialogPlots::OnMenuPrint(wxCommandEvent& event)
{
	m_plot->ShowPrintDialog();
}
// Menu: About
void CWindowDialogPlots::OnMenuAbout(wxCommandEvent& event)
{
	::wxMessageBox(
		_("Plot viewer\n Class gui::CDisplayWindowPlots\n MRPT C++ & "
		  "wxMathPlot library"),
		_("About..."));
}

void CWindowDialogPlots::OnMenuSelected(wxCommandEvent& ev)
{
	auto it = m_ID2ID.find(ev.GetId());
	if (it != m_ID2ID.end())
	{
		if (m_winPlots && m_winPlots->m_callback)
			m_winPlots->m_callback(
				it->second, d2f(m_curCursorPos.x), d2f(m_curCursorPos.y),
				m_winPlots->m_callback_param);
	}
}

void CWindowDialogPlots::OnMouseMove(wxMouseEvent& event)
{
	int X, Y;
	event.GetPosition(&X, &Y);
	m_curCursorPos.x = m_plot->p2x(X);
	m_curCursorPos.y = m_plot->p2y(Y);
	m_last_mouse_point.x = X;
	m_last_mouse_point.y = Y;

	// Send the event:
	if (m_winPlots && m_winPlots->hasSubscribers())
	{
		try
		{
			m_winPlots->publishEvent(mrptEventMouseMove(
				m_winPlots, mrpt::img::TPixelCoord(event.GetX(), event.GetY()),
				event.LeftDown(), event.RightDown()));
		}
		catch (...)
		{
		}
	}
	event.Skip();  // so it's processed by the wx system!
}

// Add / Modify a 2D plot using a MATLAB-like format string
void CWindowDialogPlots::plot(
	const CVectorFloat& x, const CVectorFloat& y, const std::string& lineFormat,
	const std::string& plotName)
{
	mpFXYVector* theLayer;

	wxString lyName = plotName.c_str();
	bool updateAtTheEnd = false;  // If we update an existing layer, update
	// manually to refresh the changes!

	// Already existing layer?
	mpLayer* existingLy = m_plot->GetLayerByName(lyName);

	if (existingLy)
	{
		// Assure the class:
		auto* lyPlot2D = dynamic_cast<mpFXYVector*>(existingLy);

		if (!lyPlot2D)
		{
			cerr << "[CWindowDialogPlots::plot] Plot name '" << plotName
				 << "' is not of expected class mpFXYVector!." << endl;
			return;
		}

		// Ok:
		theLayer = lyPlot2D;
		updateAtTheEnd = true;
	}
	else
	{
		// Create it:
		theLayer = new mpFXYVector(lyName);
		m_plot->AddLayer(theLayer);
	}

	// Set data:
	{
		std::vector<float> x_(x.size()), y_(x.size());
		std::memcpy(&x_[0], &x[0], sizeof(x[0]) * x_.size());
		std::memcpy(&y_[0], &y[0], sizeof(y[0]) * y_.size());
		theLayer->SetData(x_, y_);
	}

	// Line style:
	// -------------------
	bool isContinuous = true;
	int lineColor[] = {0, 0, 255};
	int lineWidth = 1;
	wxPenStyle lineStyle = wxPENSTYLE_SOLID;

	// parse string:
	if (string::npos != lineFormat.find(".")) { isContinuous = false; }
	if (string::npos != lineFormat.find("-"))
	{
		isContinuous = true;
		lineStyle = wxPENSTYLE_SOLID;
	}
	if (string::npos != lineFormat.find(":"))
	{
		isContinuous = true;
		lineStyle = wxPENSTYLE_LONG_DASH;
	}

	if (string::npos != lineFormat.find("r"))
	{
		lineColor[0] = 0xFF;
		lineColor[1] = 0x00;
		lineColor[2] = 0x00;
	}
	if (string::npos != lineFormat.find("g"))
	{
		lineColor[0] = 0x00;
		lineColor[1] = 0xFF;
		lineColor[2] = 0x00;
	}
	if (string::npos != lineFormat.find("b"))
	{
		lineColor[0] = 0x00;
		lineColor[1] = 0x00;
		lineColor[2] = 0xFF;
	}
	if (string::npos != lineFormat.find("k"))
	{
		lineColor[0] = 0x00;
		lineColor[1] = 0x00;
		lineColor[2] = 0x00;
	}
	if (string::npos != lineFormat.find("m"))
	{
		lineColor[0] = 192;
		lineColor[1] = 0;
		lineColor[2] = 192;
	}
	if (string::npos != lineFormat.find("c"))
	{
		lineColor[0] = 0;
		lineColor[1] = 192;
		lineColor[2] = 192;
	}

	if (string::npos != lineFormat.find("1")) { lineWidth = 1; }
	if (string::npos != lineFormat.find("2")) { lineWidth = 2; }
	if (string::npos != lineFormat.find("3")) { lineWidth = 3; }
	if (string::npos != lineFormat.find("4")) { lineWidth = 4; }
	if (string::npos != lineFormat.find("5")) { lineWidth = 5; }
	if (string::npos != lineFormat.find("6")) { lineWidth = 6; }
	if (string::npos != lineFormat.find("7")) { lineWidth = 7; }
	if (string::npos != lineFormat.find("8")) { lineWidth = 8; }
	if (string::npos != lineFormat.find("9")) { lineWidth = 9; }

	theLayer->SetContinuity(isContinuous);

	wxPen pen(
		wxColour(lineColor[0], lineColor[1], lineColor[2]), lineWidth,
		lineStyle);
	theLayer->SetPen(pen);

	theLayer->ShowName(false);

	if (updateAtTheEnd) m_plot->Refresh(false);
}

// Add / Modify a 2D ellipse
// x[0,1]: Mean
// y[0,1,2]: Covariance matrix (0,0),(1,1),(0,1)
void CWindowDialogPlots::plotEllipse(
	const CVectorFloat& x, const CVectorFloat& y, const std::string& lineFormat,
	const std::string& plotName, bool showName)
{
	mpCovarianceEllipse* theLayer;

	if (x.size() != 3 || y.size() != 3)
	{
		cerr << "[CWindowDialogPlots::plotEllipse] vectors do not have "
				"expected size!!"
			 << endl;
		return;
	}

	wxString lyName = plotName.c_str();
	bool updateAtTheEnd = false;  // If we update an existing layer, update
	// manually to refresh the changes!

	// Already existing layer?
	mpLayer* existingLy = m_plot->GetLayerByName(lyName);

	if (existingLy)
	{
		// Assure the class:
		auto* lyPlotEllipse = dynamic_cast<mpCovarianceEllipse*>(existingLy);

		if (!lyPlotEllipse)
		{
			cerr << "[CWindowDialogPlots::plotEllipse] Plot name '" << plotName
				 << "' is not of expected class mpCovarianceEllipse!." << endl;
			return;
		}

		// Ok:
		theLayer = lyPlotEllipse;
		updateAtTheEnd = true;
	}
	else
	{
		// Create it:
		theLayer = new mpCovarianceEllipse(1, 1, 0, 2, 32, lyName);
		m_plot->AddLayer(theLayer);
	}

	// Set data:
	theLayer->SetCovarianceMatrix(y[0], y[2], y[1]);
	theLayer->SetCoordinateBase(x[0], x[1]);
	theLayer->SetQuantiles(x[2]);
	theLayer->ShowName(showName);

	// Line style:
	// -------------------
	bool isContinuous = true;
	int lineColor[] = {0, 0, 255};
	int lineWidth = 1;
	wxPenStyle lineStyle = wxPENSTYLE_SOLID;

	// parse string:
	if (string::npos != lineFormat.find(".")) { isContinuous = false; }
	if (string::npos != lineFormat.find("-"))
	{
		isContinuous = true;
		lineStyle = wxPENSTYLE_SOLID;
	}
	if (string::npos != lineFormat.find(":"))
	{
		isContinuous = true;
		lineStyle = wxPENSTYLE_LONG_DASH;
	}

	if (string::npos != lineFormat.find("r"))
	{
		lineColor[0] = 0xFF;
		lineColor[1] = 0x00;
		lineColor[2] = 0x00;
	}
	if (string::npos != lineFormat.find("g"))
	{
		lineColor[0] = 0x00;
		lineColor[1] = 0xFF;
		lineColor[2] = 0x00;
	}
	if (string::npos != lineFormat.find("b"))
	{
		lineColor[0] = 0x00;
		lineColor[1] = 0x00;
		lineColor[2] = 0xFF;
	}
	if (string::npos != lineFormat.find("k"))
	{
		lineColor[0] = 0x00;
		lineColor[1] = 0x00;
		lineColor[2] = 0x00;
	}
	if (string::npos != lineFormat.find("m"))
	{
		lineColor[0] = 192;
		lineColor[1] = 0;
		lineColor[2] = 192;
	}
	if (string::npos != lineFormat.find("c"))
	{
		lineColor[0] = 0;
		lineColor[1] = 192;
		lineColor[2] = 192;
	}

	if (string::npos != lineFormat.find("1")) { lineWidth = 1; }
	if (string::npos != lineFormat.find("2")) { lineWidth = 2; }
	if (string::npos != lineFormat.find("3")) { lineWidth = 3; }
	if (string::npos != lineFormat.find("4")) { lineWidth = 4; }
	if (string::npos != lineFormat.find("5")) { lineWidth = 5; }
	if (string::npos != lineFormat.find("6")) { lineWidth = 6; }
	if (string::npos != lineFormat.find("7")) { lineWidth = 7; }
	if (string::npos != lineFormat.find("8")) { lineWidth = 8; }
	if (string::npos != lineFormat.find("9")) { lineWidth = 9; }

	theLayer->SetContinuity(isContinuous);

	wxPen pen(
		wxColour(lineColor[0], lineColor[1], lineColor[2]), lineWidth,
		lineStyle);
	theLayer->SetPen(pen);

	if (updateAtTheEnd) m_plot->Refresh(false);
}

void CWindowDialogPlots::image(
	void* theWxImage, float x0, float y0, float w, float h,
	const std::string& plotName)
{
	mpBitmapLayer* theLayer;

	wxString lyName = plotName.c_str();
	bool updateAtTheEnd = false;  // If we update an existing layer, update
	// manually to refresh the changes!

	// Already existing layer?
	mpLayer* existingLy = m_plot->GetLayerByName(lyName);

	if (existingLy)
	{
		// Assure the class:
		auto* ly = dynamic_cast<mpBitmapLayer*>(existingLy);

		if (!ly)
		{
			cerr << "[CWindowDialogPlots::image] Plot name '" << plotName
				 << "' is not of expected class mpBitmapLayer!." << endl;
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
		m_plot->AddLayer(theLayer);
	}

	// Set data:
	auto* ii = static_cast<wxImage*>(theWxImage);
	theLayer->SetBitmap(*ii, x0, y0, w, h);

	delete ii;
	theWxImage = nullptr;

	if (updateAtTheEnd) m_plot->Refresh();
}

#endif

CDisplayWindowPlots::Ptr CDisplayWindowPlots::Create(
	const std::string& windowCaption, unsigned int initialWindowWidth,
	unsigned int initialWindowHeight)
{
	return std::make_shared<CDisplayWindowPlots>(
		windowCaption, initialWindowWidth, initialWindowHeight);
}
/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CDisplayWindowPlots::CDisplayWindowPlots(
	const std::string& windowCaption, unsigned int initialWidth,
	unsigned int initialHeight)
	: CBaseGUIWindow(static_cast<void*>(this), 400, 499, windowCaption)

{
	CBaseGUIWindow::createWxWindow(initialWidth, initialHeight);
}

/*---------------------------------------------------------------
					Destructor
 ---------------------------------------------------------------*/
CDisplayWindowPlots::~CDisplayWindowPlots()
{
	CBaseGUIWindow::destroyWxWindow();
}

/** Set cursor style to default (cursorIsCross=false) or to a cross
 * (cursorIsCross=true) */
void CDisplayWindowPlots::setCursorCross([[maybe_unused]] bool cursorIsCross)
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	const auto* win = (const CWindowDialogPlots*)m_hwnd.get();
	if (!win) return;
	win->m_plot->SetCursor(
		*(cursorIsCross ? wxCROSS_CURSOR : wxSTANDARD_CURSOR));
#endif
}

/*---------------------------------------------------------------
					getLastMousePosition
 ---------------------------------------------------------------*/
bool CDisplayWindowPlots::getLastMousePosition(
	[[maybe_unused]] int& x, [[maybe_unused]] int& y) const
{
#if MRPT_HAS_WXWIDGETS && MRPT_HAS_OPENGL_GLUT
	const auto* win = (const CWindowDialogPlots*)m_hwnd.get();
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
void CDisplayWindowPlots::resize(
	[[maybe_unused]] unsigned int width, [[maybe_unused]] unsigned int height)
{
#if MRPT_HAS_WXWIDGETS
	if (!isOpen())
	{
		cerr << "[CDisplayWindowPlots::resize] Window closed!: " << m_caption
			 << endl;
		return;
	}

	// Send a request to destroy this object:
	auto* REQ = new WxSubsystem::TRequestToWxMainThread[1];
	REQ->sourcePlots = this;
	REQ->OPCODE = 403;
	REQ->x = width;
	REQ->y = height;
	WxSubsystem::pushPendingWxRequest(REQ);
#endif
}

/*---------------------------------------------------------------
					setPos
 ---------------------------------------------------------------*/
void CDisplayWindowPlots::setPos([[maybe_unused]] int x, [[maybe_unused]] int y)
{
#if MRPT_HAS_WXWIDGETS
	if (!isOpen())
	{
		cerr << "[CDisplayWindowPlots::setPos] Window closed!: " << m_caption
			 << endl;
		return;
	}

	// Send a request to destroy this object:
	auto* REQ = new WxSubsystem::TRequestToWxMainThread[1];
	REQ->sourcePlots = this;
	REQ->OPCODE = 402;
	REQ->x = x;
	REQ->y = y;
	WxSubsystem::pushPendingWxRequest(REQ);
#endif
}

/*---------------------------------------------------------------
					setWindowTitle
 ---------------------------------------------------------------*/
void CDisplayWindowPlots::setWindowTitle(
	[[maybe_unused]] const std::string& str)
{
#if MRPT_HAS_WXWIDGETS
	if (!isOpen())
	{
		cerr << "[CDisplayWindowPlots::setWindowTitle] Window closed!: "
			 << m_caption << endl;
		return;
	}

	// Send a request to destroy this object:
	auto* REQ = new WxSubsystem::TRequestToWxMainThread[1];
	REQ->sourcePlots = this;
	REQ->OPCODE = 404;
	REQ->str = str;
	WxSubsystem::pushPendingWxRequest(REQ);
#endif
}

/*---------------------------------------------------------------
					enableMousePanZoom
 ---------------------------------------------------------------*/
void CDisplayWindowPlots::enableMousePanZoom([[maybe_unused]] bool enabled)
{
#if MRPT_HAS_WXWIDGETS
	if (!isOpen()) return;

	// Send a request to destroy this object:
	auto* REQ = new WxSubsystem::TRequestToWxMainThread[1];
	REQ->sourcePlots = this;
	REQ->OPCODE = 410;
	REQ->boolVal = enabled;
	WxSubsystem::pushPendingWxRequest(REQ);
#endif
}

/*---------------------------------------------------------------
					axis_equal
 ---------------------------------------------------------------*/
void CDisplayWindowPlots::axis_equal([[maybe_unused]] bool enabled)
{
#if MRPT_HAS_WXWIDGETS
	if (!isOpen()) return;

	// Send a request to destroy this object:
	auto* REQ = new WxSubsystem::TRequestToWxMainThread[1];
	REQ->sourcePlots = this;
	REQ->OPCODE = 411;
	REQ->boolVal = enabled;
	WxSubsystem::pushPendingWxRequest(REQ);
#endif
}

/*---------------------------------------------------------------
					axis
 ---------------------------------------------------------------*/
void CDisplayWindowPlots::axis(
	[[maybe_unused]] float x_min, [[maybe_unused]] float x_max,
	[[maybe_unused]] float y_min, [[maybe_unused]] float y_max,
	[[maybe_unused]] bool aspectRatioFix)
{
#if MRPT_HAS_WXWIDGETS
	if (!isOpen()) return;

	// Send a request to destroy this object:
	auto* REQ = new WxSubsystem::TRequestToWxMainThread[1];
	REQ->sourcePlots = this;
	REQ->OPCODE = 412;
	REQ->vector_x.resize(2);
	REQ->vector_x[0] = x_min;
	REQ->vector_x[1] = x_max;
	REQ->vector_y.resize(2);
	REQ->vector_y[0] = y_min;
	REQ->vector_y[1] = y_max;
	REQ->boolVal = aspectRatioFix;
	WxSubsystem::pushPendingWxRequest(REQ);
#endif
}

/*---------------------------------------------------------------
					axis_fit
 ---------------------------------------------------------------*/
void CDisplayWindowPlots::axis_fit([[maybe_unused]] bool aspectRatioFix)
{
#if MRPT_HAS_WXWIDGETS
	if (!isOpen()) return;

	// Send a request to destroy this object:
	auto* REQ = new WxSubsystem::TRequestToWxMainThread[1];
	REQ->sourcePlots = this;
	REQ->OPCODE = 413;
	REQ->boolVal = aspectRatioFix;
	WxSubsystem::pushPendingWxRequest(REQ);
#endif
}

/*---------------------------------------------------------------
					plotEllipse
 ---------------------------------------------------------------*/
template <typename T>
void CDisplayWindowPlots::plotEllipse(
	[[maybe_unused]] const T mean_x, [[maybe_unused]] const T mean_y,
	[[maybe_unused]] const CMatrixDynamic<T>& cov22,
	[[maybe_unused]] const float quantiles,
	[[maybe_unused]] const std::string& lineFormat,
	[[maybe_unused]] const std::string& plotName,
	[[maybe_unused]] bool showName)
{
#if MRPT_HAS_WXWIDGETS
	MRPT_START
	if (!isOpen()) return;

	ASSERT_(cov22.cols() == 2 && cov22.rows() == 2);
	ASSERT_(cov22(0, 0) >= 0);
	ASSERT_(cov22(1, 1) >= 0);
	ASSERT_(cov22(0, 1) == cov22(1, 0));

	if (m_holdon_just_disabled)
	{
		m_holdon_just_disabled = false;
		this->clf();
	}
	std::string holdon_post;
	if (m_holdon)
		holdon_post =
			format("_fig_%u", static_cast<unsigned int>(m_holdon_cnt++));

	// Send a request to destroy this object:
	auto* REQ = new WxSubsystem::TRequestToWxMainThread[1];
	REQ->sourcePlots = this;
	REQ->OPCODE = 421;
	// 421: Add/update a 2D ellipse: format string=str, plot name =plotName,
	// vector_x[0,1]:X/Y center,  vector_y[0,1,2]: Covariance matrix entries
	// 00,11,01.
	REQ->str = lineFormat;
	REQ->plotName = plotName + holdon_post;

	REQ->vector_x.resize(3);
	REQ->vector_x[0] = d2f(mean_x);
	REQ->vector_x[1] = d2f(mean_y);
	REQ->vector_x[2] = quantiles;

	REQ->vector_y.resize(3);
	REQ->vector_y[0] = d2f(cov22(0, 0));
	REQ->vector_y[1] = d2f(cov22(1, 1));
	REQ->vector_y[2] = d2f(cov22(0, 1));

	REQ->boolVal = showName;

	WxSubsystem::pushPendingWxRequest(REQ);
	MRPT_END
#endif
}

// Explicit instantations:
template void CDisplayWindowPlots::plotEllipse(
	const float mean_x, const float mean_y, const CMatrixDynamic<float>& cov22,
	const float quantiles, const std::string& lineFormat,
	const std::string& plotName, bool showName);
template void CDisplayWindowPlots::plotEllipse(
	const double mean_x, const double mean_y,
	const CMatrixDynamic<double>& cov22, const float quantiles,
	const std::string& lineFormat, const std::string& plotName, bool showName);

/*---------------------------------------------------------------
					plotEllipse
 ---------------------------------------------------------------*/
template <typename T>
void CDisplayWindowPlots::plotEllipse(
	[[maybe_unused]] const T mean_x, [[maybe_unused]] const T mean_y,
	[[maybe_unused]] const CMatrixFixed<T, 2, 2>& cov22,
	[[maybe_unused]] const float quantiles,
	[[maybe_unused]] const std::string& lineFormat,
	[[maybe_unused]] const std::string& plotName,
	[[maybe_unused]] bool showName)
{
#if MRPT_HAS_WXWIDGETS
	MRPT_START
	if (!isOpen()) return;

	ASSERT_(cov22(0, 0) >= 0);
	ASSERT_(cov22(1, 1) >= 0);
	ASSERT_(cov22(0, 1) == cov22(1, 0));

	if (m_holdon_just_disabled)
	{
		m_holdon_just_disabled = false;
		this->clf();
	}
	std::string holdon_post;
	if (m_holdon)
		holdon_post =
			format("_fig_%u", static_cast<unsigned int>(m_holdon_cnt++));

	// Send a request to destroy this object:
	auto* REQ = new WxSubsystem::TRequestToWxMainThread[1];
	REQ->sourcePlots = this;
	REQ->OPCODE = 421;
	// 421: Add/update a 2D ellipse: format string=str, plot name =plotName,
	// vector_x[0,1]:X/Y center,  vector_y[0,1,2]: Covariance matrix entries
	// 00,11,01.
	REQ->str = lineFormat;
	REQ->plotName = plotName + holdon_post;

	REQ->vector_x.resize(3);
	REQ->vector_x[0] = d2f(mean_x);
	REQ->vector_x[1] = d2f(mean_y);
	REQ->vector_x[2] = quantiles;

	REQ->vector_y.resize(3);
	REQ->vector_y[0] = d2f(cov22(0, 0));
	REQ->vector_y[1] = d2f(cov22(1, 1));
	REQ->vector_y[2] = d2f(cov22(0, 1));

	REQ->boolVal = showName;

	WxSubsystem::pushPendingWxRequest(REQ);
	MRPT_END
#endif
}

// Explicit instantations:
template void CDisplayWindowPlots::plotEllipse(
	const float mean_x, const float mean_y,
	const CMatrixFixed<float, 2, 2>& cov22, const float quantiles,
	const std::string& lineFormat, const std::string& plotName, bool showName);
template void CDisplayWindowPlots::plotEllipse(
	const double mean_x, const double mean_y,
	const CMatrixFixed<double, 2, 2>& cov22, const float quantiles,
	const std::string& lineFormat, const std::string& plotName, bool showName);

/*---------------------------------------------------------------
					image
 ---------------------------------------------------------------*/
void CDisplayWindowPlots::image(
	[[maybe_unused]] const mrpt::img::CImage& img,
	[[maybe_unused]] float x_left, [[maybe_unused]] float y_bottom,
	[[maybe_unused]] float x_width, [[maybe_unused]] float y_height,
	[[maybe_unused]] const std::string& plotName)
{
#if MRPT_HAS_WXWIDGETS
	MRPT_START
	if (!isOpen()) return;

	if (m_holdon_just_disabled)
	{
		m_holdon_just_disabled = false;
		this->clf();
	}
	std::string holdon_post;
	if (m_holdon)
		holdon_post =
			format("_fig_%u", static_cast<unsigned int>(m_holdon_cnt++));

	// Send a request to destroy this object:
	auto* REQ = new WxSubsystem::TRequestToWxMainThread[1];
	REQ->sourcePlots = this;
	REQ->OPCODE = 422;

	// 422: Add/update a bitmap: plot name =plotName, vector_x[0,1]:X/Y corner,
	// vector_x[2,3]: X/Y widths, voidPtr2: pointer to a newly created wxImage
	// with the bitmap.
	REQ->plotName = plotName + holdon_post;

	REQ->vector_x.resize(4);
	REQ->vector_x[0] = x_left;
	REQ->vector_x[1] = y_bottom;
	REQ->vector_x[2] = x_width;
	REQ->vector_x[3] = y_height;

	REQ->voidPtr2 = mrpt::gui::MRPTImage2wxImage(img);

	WxSubsystem::pushPendingWxRequest(REQ);
	MRPT_END
#endif
}

/*---------------------------------------------------------------
					internal_plot
 ---------------------------------------------------------------*/
void CDisplayWindowPlots::internal_plot(
	[[maybe_unused]] CVectorFloat& x, [[maybe_unused]] CVectorFloat& y,
	[[maybe_unused]] const std::string& lineFormat,
	[[maybe_unused]] const std::string& plotName)
{
#if MRPT_HAS_WXWIDGETS
	MRPT_START
	if (!isOpen()) return;

	ASSERT_EQUAL_(x.size(), y.size());

	if (m_holdon_just_disabled)
	{
		m_holdon_just_disabled = false;
		this->clf();
	}

	if (x.empty()) return;

	std::string holdon_post;
	if (m_holdon)
		holdon_post =
			format("_fig_%u", static_cast<unsigned int>(m_holdon_cnt++));

	// Send a request to destroy this object:
	auto* REQ = new WxSubsystem::TRequestToWxMainThread[1];
	REQ->sourcePlots = this;
	REQ->OPCODE = 420;
	REQ->str = lineFormat;
	REQ->plotName = plotName + holdon_post;
	REQ->vector_x.swap(x);
	REQ->vector_y.swap(y);

	WxSubsystem::pushPendingWxRequest(REQ);
	MRPT_END
#endif
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
	auto* REQ = new WxSubsystem::TRequestToWxMainThread[1];
	REQ->sourcePlots = this;
	REQ->OPCODE = 414;

	// 414: Clear all plot objects.

	WxSubsystem::pushPendingWxRequest(REQ);
#endif
	MRPT_END
}

/*---------------------------------------------------------------
					hold_on
 ---------------------------------------------------------------*/
void CDisplayWindowPlots::hold_on() { m_holdon = true; }
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
	[[maybe_unused]] const std::string& label, [[maybe_unused]] int menuID)
{
#if MRPT_HAS_WXWIDGETS
	MRPT_START
	if (!isOpen()) return;

	auto* REQ = new WxSubsystem::TRequestToWxMainThread[1];
	REQ->sourcePlots = this;
	REQ->OPCODE = 440;
	REQ->plotName = label;
	REQ->x = menuID;
	// 440: Inser submenu in the popup menu.

	WxSubsystem::pushPendingWxRequest(REQ);
	MRPT_END
#endif
}

/*---------------------------------------------------------------
					setMenuCallback
 ---------------------------------------------------------------*/
void CDisplayWindowPlots::setMenuCallback(
	TCallbackMenu userFunction, void* userParam)
{
	ASSERT_(userFunction != nullptr);
	m_callback = userFunction;
	m_callback_param = userParam;
}
