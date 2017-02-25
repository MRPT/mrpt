/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "gridmapSimulMain.h"
#include <wx/msgdlg.h>

//(*InternalHeaders(gridmapSimulFrame)
#include <wx/string.h>
#include <wx/intl.h>
#include <wx/bitmap.h>
#include <wx/icon.h>
#include <wx/image.h>
#include <wx/artprov.h>
//*)

#include <wx/msgdlg.h>
#include <wx/filedlg.h>
#include <wx/progdlg.h>
#include <wx/imaglist.h>
#include <wx/busyinfo.h>
#include <wx/log.h>
#include <wx/textdlg.h>
#include <wx/dirdlg.h>
#include <wx/colordlg.h>
#include <wx/dcmemory.h>
#include <wx/image.h>
#include <wx/artprov.h>

#include "../wx-common/mrpt_logo.xpm"
#include "imgs/app_icon_gridmapsimul.xpm"

#include <mrpt/gui/CMyRedirector.h>
#include "CAboutBox.h"

#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/utils/CTimeLogger.h>

//#define DO_SCAN_LIKELIHOOD_DEBUG

#ifdef DO_SCAN_LIKELIHOOD_DEBUG
#	include <mrpt/gui/CDisplayWindowPlots.h>
#	include <mrpt/math/data_utils.h>
#endif

#if !wxUSE_GLCANVAS
#error "OpenGL required: set wxUSE_GLCANVAS to 1 and rebuild wxWidgets"
#endif


#define TIMER_MS 10

#include "virtual_map1.xpm"

// A custom Art provider for customizing the icons:
class MyArtProvider : public wxArtProvider
{
protected:
	virtual wxBitmap CreateBitmap(const wxArtID& id,
								  const wxArtClient& client,
								  const wxSize& size);
};

#define RETURN_BITMAP(artid,xpm) \
	if (id == artid) \
	{ \
		if (client==wxART_MENU) \
		{	wxBitmap	b(xpm); \
			return wxBitmap( b.ConvertToImage().Scale(16,16, wxIMAGE_QUALITY_HIGH ) ); \
		} else \
		{  return wxBitmap(xpm); } \
	} \

// CreateBitmap function
wxBitmap MyArtProvider::CreateBitmap(const wxArtID& id,
									 const wxArtClient& client,
									 const wxSize& size)
{
	RETURN_BITMAP( wxART_MAKE_ART_ID(IMG_MRPT_LOGO),	mrpt_logo_xpm );
	RETURN_BITMAP( wxART_MAKE_ART_ID(MAIN_ICON),	    app_icon_gridmapsimul_xpm );

	// Any wxWidgets icons not implemented here
	// will be provided by the default art provider.
	return wxNullBitmap;
}

#include <mrpt/gui/CMyGLCanvasBase.h>

#include <mrpt/utils/CTicTac.h>
#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/hwdrivers/CJoystick.h>

#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/stock_objects.h>

#include <mrpt/gui/WxUtils.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;


mrpt::kinematics::CVehicleSimul_DiffDriven  the_robot;
COccupancyGridMap2D          the_grid;
CJoystick                    joystick;

StdVector_CPose2D		robot_path_GT, robot_path_ODO;
CPose2D					lastOdo, pose_start;
bool					we_are_closing = false;
long					decimation = 1;

mrpt::opengl::CSetOfObjectsPtr		gl_grid;
mrpt::opengl::CSetOfObjectsPtr		gl_robot;
mrpt::opengl::CPlanarLaserScanPtr 	gl_scan;
mrpt::opengl::CPointCloudPtr 		gl_path_GT;
mrpt::opengl::CPointCloudPtr 		gl_path_ODO;


int		last_pressed_key = 0;

long LASER_N_RANGES     = 361;
double LASER_APERTURE  = M_PI;
double LASER_STD_ERROR = 0.01;
double LASER_BEARING_STD_ERROR = DEG2RAD(0.05);


class CMyGLCanvas : public CMyGLCanvasBase
{
public:
    CMyGLCanvas( wxWindow *parent, wxWindowID id = wxID_ANY,
                 const wxPoint& pos = wxDefaultPosition,
                 const wxSize& size = wxDefaultSize,
                 long style = 0, const wxString& name = _T("CMyGLCanvasBase") )
		: CMyGLCanvasBase(parent,id,pos,size,style,name)
	{
	}

	virtual ~CMyGLCanvas()
	{ }

	void OnCharCustom( wxKeyEvent& event );

	void OnPreRender();
	void OnPostRender();
	void OnPostRenderSwapBuffers(double At, wxPaintDC &dc);
	void OnRenderError( const wxString &str );

};

void CMyGLCanvas::OnRenderError( const wxString &str )
{
}

void CMyGLCanvas::OnPreRender()
{
}

void CMyGLCanvas::OnPostRenderSwapBuffers(double At, wxPaintDC &dc)
{
}


void CMyGLCanvas::OnPostRender()
{
	CPose2D p = the_robot.getCurrentGTPose();

	string s = format("Pose: (%.03f,%.03f,%.02fdeg)", p.x(),p.y(), RAD2DEG(p.phi()) );
	mrpt::opengl::CRenderizable::renderTextBitmap( 20,20, s.c_str(), 1,0,0 , MRPT_GLUT_BITMAP_HELVETICA_18);


	const mrpt::math::TTwist2D vel_local = the_robot.getCurrentGTVelLocal();
	s = format("V=%.03fm/s  W=%.02fdeg/s", vel_local.vx, RAD2DEG(vel_local.omega) );
	mrpt::opengl::CRenderizable::renderTextBitmap( 20,45, s.c_str(), 1,0,0 , MRPT_GLUT_BITMAP_HELVETICA_18);
}

void CMyGLCanvas::OnCharCustom( wxKeyEvent& event )
{
	last_pressed_key = event.GetKeyCode();
}

//(*IdInit(gridmapSimulFrame)
const long gridmapSimulFrame::ID_STATICTEXT1 = wxNewId();
const long gridmapSimulFrame::ID_TEXTCTRL1 = wxNewId();
const long gridmapSimulFrame::ID_STATICTEXT2 = wxNewId();
const long gridmapSimulFrame::ID_TEXTCTRL2 = wxNewId();
const long gridmapSimulFrame::ID_STATICTEXT3 = wxNewId();
const long gridmapSimulFrame::ID_TEXTCTRL3 = wxNewId();
const long gridmapSimulFrame::ID_STATICTEXT8 = wxNewId();
const long gridmapSimulFrame::ID_TEXTCTRL5 = wxNewId();
const long gridmapSimulFrame::ID_BUTTON5 = wxNewId();
const long gridmapSimulFrame::ID_STATICTEXT16 = wxNewId();
const long gridmapSimulFrame::ID_SPINCTRL1 = wxNewId();
const long gridmapSimulFrame::ID_STATICTEXT9 = wxNewId();
const long gridmapSimulFrame::ID_TEXTCTRL6 = wxNewId();
const long gridmapSimulFrame::ID_STATICTEXT13 = wxNewId();
const long gridmapSimulFrame::ID_TEXTCTRL7 = wxNewId();
const long gridmapSimulFrame::ID_STATICTEXT10 = wxNewId();
const long gridmapSimulFrame::ID_TEXTCTRL8 = wxNewId();
const long gridmapSimulFrame::ID_STATICTEXT11 = wxNewId();
const long gridmapSimulFrame::ID_TEXTCTRL9 = wxNewId();
const long gridmapSimulFrame::ID_STATICTEXT12 = wxNewId();
const long gridmapSimulFrame::ID_TEXTCTRL10 = wxNewId();
const long gridmapSimulFrame::ID_STATICTEXT14 = wxNewId();
const long gridmapSimulFrame::ID_TEXTCTRL11 = wxNewId();
const long gridmapSimulFrame::ID_STATICTEXT4 = wxNewId();
const long gridmapSimulFrame::ID_BUTTON4 = wxNewId();
const long gridmapSimulFrame::ID_CHECKBOX1 = wxNewId();
const long gridmapSimulFrame::ID_CHECKBOX_RAWLOG_FORMAT = wxNewId();
const long gridmapSimulFrame::ID_STATICTEXT6 = wxNewId();
const long gridmapSimulFrame::ID_STATICTEXT7 = wxNewId();
const long gridmapSimulFrame::ID_BUTTON6 = wxNewId();
const long gridmapSimulFrame::ID_PANEL3 = wxNewId();
const long gridmapSimulFrame::ID_BUTTON1 = wxNewId();
const long gridmapSimulFrame::ID_BUTTON2 = wxNewId();
const long gridmapSimulFrame::ID_STATICTEXT5 = wxNewId();
const long gridmapSimulFrame::ID_TEXTCTRL4 = wxNewId();
const long gridmapSimulFrame::ID_BUTTON3 = wxNewId();
const long gridmapSimulFrame::ID_STATICTEXT15 = wxNewId();
const long gridmapSimulFrame::ID_TEXTCTRL12 = wxNewId();
const long gridmapSimulFrame::ID_PANEL6 = wxNewId();
const long gridmapSimulFrame::ID_PANEL4 = wxNewId();
const long gridmapSimulFrame::ID_PANEL2 = wxNewId();
const long gridmapSimulFrame::ID_PANEL1 = wxNewId();
const long gridmapSimulFrame::ID_PANEL5 = wxNewId();
const long gridmapSimulFrame::ID_SPLITTERWINDOW1 = wxNewId();
const long gridmapSimulFrame::ID_TIMER1 = wxNewId();
const long gridmapSimulFrame::ID_MENUITEM1 = wxNewId();
const long gridmapSimulFrame::ID_MENUITEM_LOADMAP = wxNewId();
const long gridmapSimulFrame::ID_MENUITEM2 = wxNewId();
const long gridmapSimulFrame::ID_MENUITEM3 = wxNewId();
//*)
const long gridmapSimulFrame::ID_TEXTCTRL_INPUT = wxNewId();


BEGIN_EVENT_TABLE(gridmapSimulFrame,wxFrame)
    //(*EventTable(gridmapSimulFrame)
    //*)
END_EVENT_TABLE()

#ifdef _MSC_VER
#	define _MYTT(X) L##X
#else
#	define _MYTT(X) _T(X)
#endif

gridmapSimulFrame::gridmapSimulFrame(wxWindow* parent,wxWindowID id)
{
	// Load my custom icons:
	wxArtProvider::Push(new MyArtProvider);

    //(*Initialize(gridmapSimulFrame)
    wxMenuItem* MenuItem2;
    wxStaticBoxSizer* StaticBoxSizer2;
    wxFlexGridSizer* FlexGridSizer8;
    wxFlexGridSizer* FlexGridSizer1;
    wxFlexGridSizer* FlexGridSizer2;
    wxStaticBoxSizer* StaticBoxSizer5;
    wxFlexGridSizer* FlexGridSizer11;
    wxFlexGridSizer* FlexGridSizer7;
    wxFlexGridSizer* FlexGridSizer4;
    wxFlexGridSizer* FlexGridSizer9;
    wxStaticBoxSizer* StaticBoxSizer3;
    wxFlexGridSizer* FlexGridSizer6;
    wxMenuItem* MenuItem3;
    wxFlexGridSizer* FlexGridSizer3;
    wxFlexGridSizer* flexGL;
    wxStaticBoxSizer* StaticBoxSizer4;
    wxStaticBoxSizer* StaticBoxSizer6;
    wxFlexGridSizer* FlexGridSizer10;
    wxFlexGridSizer* FlexGridSizer13;
    wxFlexGridSizer* FlexGridSizer12;
    wxFlexGridSizer* FlexGridSizer5;
    wxStaticBoxSizer* StaticBoxSizer1;

    Create(parent, wxID_ANY, _("Gridmap navigation simulator - Part of the MRPT project"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("wxID_ANY"));
    SetMinSize(wxSize(400,400));
    {
    wxIcon FrameIcon;
    FrameIcon.CopyFromBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("MAIN_ICON")),wxART_FRAME_ICON));
    SetIcon(FrameIcon);
    }
    FlexGridSizer1 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer1->AddGrowableCol(0);
    FlexGridSizer1->AddGrowableRow(0);
    SplitterWindow1 = new wxSplitterWindow(this, ID_SPLITTERWINDOW1, wxDefaultPosition, wxDefaultSize, wxSP_3D, _T("ID_SPLITTERWINDOW1"));
    SplitterWindow1->SetMinSize(wxSize(200,200));
    SplitterWindow1->SetMinimumPaneSize(200);
    Panel1 = new wxPanel(SplitterWindow1, ID_PANEL1, wxDefaultPosition, wxSize(983,291), wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    Panel1->SetMinSize(wxSize(400,250));
    FlexGridSizer2 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer2->AddGrowableCol(0);
    //FlexGridSizer2->AddGrowableRow();
    Panel2 = new wxPanel(Panel1, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
    FlexGridSizer3 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer3->AddGrowableCol(0);
    Panel3 = new wxPanel(Panel2, ID_PANEL3, wxDefaultPosition, wxSize(622,67), wxTAB_TRAVERSAL, _T("ID_PANEL3"));
    FlexGridSizer4 = new wxFlexGridSizer(1, 4, 0, 0);
    FlexGridSizer4->AddGrowableCol(2);
    FlexGridSizer12 = new wxFlexGridSizer(0, 1, 0, 0);
    StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, Panel3, _("Laser sensor"));
    FlexGridSizer5 = new wxFlexGridSizer(0, 2, 0, 0);
    StaticText1 = new wxStaticText(Panel3, ID_STATICTEXT1, _("Range count:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    FlexGridSizer5->Add(StaticText1, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edCount = new wxTextCtrl(Panel3, ID_TEXTCTRL1, _("361"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    FlexGridSizer5->Add(edCount, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText2 = new wxStaticText(Panel3, ID_STATICTEXT2, _("Span (deg):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    FlexGridSizer5->Add(StaticText2, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edSpan = new wxTextCtrl(Panel3, ID_TEXTCTRL2, _("180"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL2"));
    FlexGridSizer5->Add(edSpan, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText3 = new wxStaticText(Panel3, ID_STATICTEXT3, _("Range noise std. (m):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
    FlexGridSizer5->Add(StaticText3, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edStdNoise = new wxTextCtrl(Panel3, ID_TEXTCTRL3, _("0.01"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL3"));
    FlexGridSizer5->Add(edStdNoise, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText8 = new wxStaticText(Panel3, ID_STATICTEXT8, _("Bearing noise std. (deg):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT8"));
    FlexGridSizer5->Add(StaticText8, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edStdNoiseAng = new wxTextCtrl(Panel3, ID_TEXTCTRL5, _("0.05"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL5"));
    FlexGridSizer5->Add(edStdNoiseAng, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnSetLaser = new wxButton(Panel3, ID_BUTTON5, _("Apply"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON5"));
    FlexGridSizer5->Add(btnSetLaser, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer1->Add(FlexGridSizer5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer12->Add(StaticBoxSizer1, 1, wxALL|wxEXPAND|wxALIGN_TOP|wxALIGN_CENTER_HORIZONTAL, 5);
    StaticBoxSizer6 = new wxStaticBoxSizer(wxHORIZONTAL, Panel3, _(" Decimation: "));
    FlexGridSizer13 = new wxFlexGridSizer(0, 2, 0, 0);
    FlexGridSizer13->AddGrowableCol(0);
    StaticText16 = new wxStaticText(Panel3, ID_STATICTEXT16, _("Decimation:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT16"));
    FlexGridSizer13->Add(StaticText16, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edDecimate = new wxSpinCtrl(Panel3, ID_SPINCTRL1, _T("1"), wxDefaultPosition, wxDefaultSize, 0, 1, 100, 1, _T("ID_SPINCTRL1"));
    edDecimate->SetValue(_T("1"));
    FlexGridSizer13->Add(edDecimate, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer6->Add(FlexGridSizer13, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer12->Add(StaticBoxSizer6, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer4->Add(FlexGridSizer12, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer7 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer7->AddGrowableCol(0);
    StaticBoxSizer3 = new wxStaticBoxSizer(wxHORIZONTAL, Panel3, _("Odometry errors"));
    FlexGridSizer9 = new wxFlexGridSizer(0, 4, 0, 0);
    StaticText9 = new wxStaticText(Panel3, ID_STATICTEXT9, _MYTT("\u0394x bias:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT9"));
    FlexGridSizer9->Add(StaticText9, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edAxb = new wxTextCtrl(Panel3, ID_TEXTCTRL6, _("1e-5"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL6"));
    FlexGridSizer9->Add(edAxb, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText13 = new wxStaticText(Panel3, ID_STATICTEXT13, _MYTT("\u0394x std:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT13"));
    FlexGridSizer9->Add(StaticText13, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edAxs = new wxTextCtrl(Panel3, ID_TEXTCTRL7, _("10e-4"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL7"));
    FlexGridSizer9->Add(edAxs, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText10 = new wxStaticText(Panel3, ID_STATICTEXT10, _MYTT("\u0394y bias:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT10"));
    FlexGridSizer9->Add(StaticText10, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edAyb = new wxTextCtrl(Panel3, ID_TEXTCTRL8, _("1e-5"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL8"));
    FlexGridSizer9->Add(edAyb, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText11 = new wxStaticText(Panel3, ID_STATICTEXT11, _MYTT("\u0394y std:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT11"));
    FlexGridSizer9->Add(StaticText11, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edAys = new wxTextCtrl(Panel3, ID_TEXTCTRL9, _("10e-4"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL9"));
    FlexGridSizer9->Add(edAys, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText12 = new wxStaticText(Panel3, ID_STATICTEXT12, _MYTT("\u0394\u03C6 bias:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT12"));
    FlexGridSizer9->Add(StaticText12, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edApb = new wxTextCtrl(Panel3, ID_TEXTCTRL10, _("5e-4"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL10"));
    FlexGridSizer9->Add(edApb, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText14 = new wxStaticText(Panel3, ID_STATICTEXT14, _MYTT("\u0394\u03C6 std:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT14"));
    FlexGridSizer9->Add(StaticText14, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edAps = new wxTextCtrl(Panel3, ID_TEXTCTRL11, _("10e-4"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL11"));
    FlexGridSizer9->Add(edAps, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer3->Add(FlexGridSizer9, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer7->Add(StaticBoxSizer3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer5 = new wxStaticBoxSizer(wxHORIZONTAL, Panel3, _(" Re-simulate "));
    FlexGridSizer11 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer11->AddGrowableCol(0);
    StaticText4 = new wxStaticText(Panel3, ID_STATICTEXT4, _("Select an existing ground truth (GT) file and \nthe corresponding scans will be simulated \nagain with the current laser sensor errors."), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT4"));
    FlexGridSizer11->Add(StaticText4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnResimulate = new wxButton(Panel3, ID_BUTTON4, _("Re-simulate..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON4"));
    FlexGridSizer11->Add(btnResimulate, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer5->Add(FlexGridSizer11, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer7->Add(StaticBoxSizer5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer4->Add(FlexGridSizer7, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    StaticBoxSizer2 = new wxStaticBoxSizer(wxHORIZONTAL, Panel3, _("Controls"));
    FlexGridSizer8 = new wxFlexGridSizer(0, 1, 0, 0);
    cbJoy = new wxCheckBox(Panel3, ID_CHECKBOX1, _("Use joystick"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
    cbJoy->SetValue(true);
    FlexGridSizer8->Add(cbJoy, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    StaticText6 = new wxStaticText(Panel3, ID_STATICTEXT6, _("Use arrow keys for inc/decrement speed and turn."), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT6"));
    FlexGridSizer8->Add(StaticText6, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    StaticText7 = new wxStaticText(Panel3, ID_STATICTEXT7, _("Brake: Space or joystick\'s first button."), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT7"));
    FlexGridSizer8->Add(StaticText7, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);

	FlexGridSizer8->Add(new wxStaticText(Panel3, ID_STATICTEXT6, _("Enter keystrokes in this box:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT6")), 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    wxTextCtrl *edInput = new wxTextCtrl(Panel3, ID_TEXTCTRL_INPUT, _(""), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL_INPUT"));
    FlexGridSizer8->Add(edInput, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);

    StaticBoxSizer2->Add(FlexGridSizer8, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer4->Add(StaticBoxSizer2, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnExit = new wxButton(Panel3, ID_BUTTON6, _("Exit"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON6"));
    FlexGridSizer4->Add(btnExit, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
    Panel3->SetSizer(FlexGridSizer4);
    FlexGridSizer4->SetSizeHints(Panel3);
    FlexGridSizer3->Add(Panel3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel4 = new wxPanel(Panel2, ID_PANEL4, wxDefaultPosition, wxSize(622,55), wxTAB_TRAVERSAL, _T("ID_PANEL4"));
    FlexGridSizer6 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer6->AddGrowableCol(0);
    StaticBoxSizer4 = new wxStaticBoxSizer(wxHORIZONTAL, Panel4, _("Rawlog generation"));
    Panel5 = new wxPanel(Panel4, ID_PANEL6, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL6"));
    FlexGridSizer10 = new wxFlexGridSizer(0, 5, 0, 0);
    FlexGridSizer10->AddGrowableCol(3);
    btnStart = new wxButton(Panel5, ID_BUTTON1, _("START"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
    FlexGridSizer10->Add(btnStart, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnEnd = new wxButton(Panel5, ID_BUTTON2, _("STOP"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
    btnEnd->Disable();
    FlexGridSizer10->Add(btnEnd, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText5 = new wxStaticText(Panel5, ID_STATICTEXT5, _("Output rawlog:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT5"));
    FlexGridSizer10->Add(StaticText5, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edOutFile = new wxTextCtrl(Panel5, ID_TEXTCTRL4, _("./simul.rawlog"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL4"));
    FlexGridSizer10->Add(edOutFile, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnExplore = new wxButton(Panel5, ID_BUTTON3, _("Browse..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON3"));
    FlexGridSizer10->Add(btnExplore, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);

    //FlexGridSizer10->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	cbRawlogSFformat = new wxCheckBox(Panel5, ID_CHECKBOX_RAWLOG_FORMAT, _("Act/SF format"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX_RAWLOG_FORMAT"));
    cbRawlogSFformat->SetValue(false);
    FlexGridSizer10->Add(cbRawlogSFformat, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);


    FlexGridSizer10->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText15 = new wxStaticText(Panel5, ID_STATICTEXT15, _("Output groundtruth:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT15"));
    FlexGridSizer10->Add(StaticText15, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edOutGT = new wxTextCtrl(Panel5, ID_TEXTCTRL12, _("./simul.rawlog.GT.txt"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL12"));
    edOutGT->Disable();
    FlexGridSizer10->Add(edOutGT, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    Panel5->SetSizer(FlexGridSizer10);
    FlexGridSizer10->Fit(Panel5);
    FlexGridSizer10->SetSizeHints(Panel5);
    StaticBoxSizer4->Add(Panel5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer6->Add(StaticBoxSizer4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    Panel4->SetSizer(FlexGridSizer6);
    FlexGridSizer6->SetSizeHints(Panel4);
    FlexGridSizer3->Add(Panel4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel2->SetSizer(FlexGridSizer3);
    FlexGridSizer3->Fit(Panel2);
    FlexGridSizer3->SetSizeHints(Panel2);
    FlexGridSizer2->Add(Panel2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel1->SetSizer(FlexGridSizer2);
    FlexGridSizer2->SetSizeHints(Panel1);
    panelGL = new wxPanel(SplitterWindow1, ID_PANEL5, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL5"));
    flexGL = new wxFlexGridSizer(0, 1, 0, 0);
    flexGL->AddGrowableCol(0);
    flexGL->AddGrowableRow(0);
    panelGL->SetSizer(flexGL);
    flexGL->Fit(panelGL);
    flexGL->SetSizeHints(panelGL);
    SplitterWindow1->SplitHorizontally(Panel1, panelGL);
    FlexGridSizer1->Add(SplitterWindow1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    SetSizer(FlexGridSizer1);
    timRun.SetOwner(this, ID_TIMER1);
    timRun.Start(10, true);
    MenuBar1 = new wxMenuBar();
    Menu1 = new wxMenu();
    MenuItem1 = new wxMenuItem(Menu1, ID_MENUITEM1, _("Set &output rawlog file..."), wxEmptyString, wxITEM_NORMAL);
    MenuItem1->SetBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FILE_SAVE")),wxART_MENU));
    Menu1->Append(MenuItem1);

    MenuItemLoadMap = new wxMenuItem(Menu1, ID_MENUITEM_LOADMAP, _("Load &gridmap..."), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(MenuItemLoadMap);

    MenuItem2 = new wxMenuItem(Menu1, ID_MENUITEM2, _("&Quit"), wxEmptyString, wxITEM_NORMAL);
    MenuItem2->SetBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_QUIT")),wxART_MENU));
    Menu1->Append(MenuItem2);
    MenuBar1->Append(Menu1, _("&File"));
    Menu2 = new wxMenu();
    MenuItem3 = new wxMenuItem(Menu2, ID_MENUITEM3, _("&About..."), wxEmptyString, wxITEM_NORMAL);
    MenuItem3->SetBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_HELP_BOOK")),wxART_MENU));
    Menu2->Append(MenuItem3);
    MenuBar1->Append(Menu2, _("&Help"));
    SetMenuBar(MenuBar1);
    FlexGridSizer1->Fit(this);
    FlexGridSizer1->SetSizeHints(this);

    Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&gridmapSimulFrame::OnbtnSetLaserClick);
    Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&gridmapSimulFrame::OnbtnResimulateClick);
    Connect(ID_BUTTON6,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&gridmapSimulFrame::OnbtnQuitClick);
    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&gridmapSimulFrame::OnbtnStartClick);
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&gridmapSimulFrame::OnbtnEndClick);
    Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&gridmapSimulFrame::OnbtnExploreClick);
    Connect(ID_TIMER1,wxEVT_TIMER,(wxObjectEventFunction)&gridmapSimulFrame::OntimRunTrigger);
    Connect(ID_MENUITEM1,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&gridmapSimulFrame::OnbtnExploreClick);
    Connect(ID_MENUITEM_LOADMAP,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&gridmapSimulFrame::OnMenuLoadMap);
    Connect(ID_MENUITEM2,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&gridmapSimulFrame::OnbtnQuitClick);
    Connect(ID_MENUITEM3,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&gridmapSimulFrame::OnAbout);
    //*)

    // Import grid map:
    {
		wxImage img(virtual_map1_xpm);
		CImage *myImg = wxImage2MRPTImage(img);
		the_grid.loadFromBitmap( *myImg, 0.03f );
		delete myImg;
    }

    // Create GL canvas:
    // -------------------------------
	m_canvas = new CMyGLCanvas( panelGL, wxID_ANY, wxDefaultPosition, wxDefaultSize );
	panelGL->SetMinSize( wxSize(200,200) );
	m_canvas->SetMinSize( wxSize(200,200) );
	flexGL->Add(m_canvas, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);

	m_canvas->cameraPointingX = 0;
	m_canvas->cameraPointingY = 0;
	m_canvas->cameraPointingZ = 0;
	m_canvas->cameraElevationDeg = 60;
	m_canvas->cameraZoomDistance = 25;

	// Populate scene:
	m_canvas->m_openGLScene->insert( mrpt::opengl::CGridPlaneXY::Create(-100,100,-100,100,0,5) );

	update_grid_map_3d();
	m_canvas->m_openGLScene->insert( gl_grid );

	// paths:
	gl_path_GT = mrpt::opengl::CPointCloud::Create();
	gl_path_GT->setColor(0,0,0, 0.7);
	gl_path_GT->setLocation(0,0, 0.01);
	gl_path_GT->setPointSize(3);

	m_canvas->m_openGLScene->insert( gl_path_GT );


	gl_path_ODO = mrpt::opengl::CPointCloud::Create();
	gl_path_ODO->setColor(0,1,0, 0.7);
	gl_path_ODO->setLocation(0,0, 0.01);
	gl_path_ODO->setPointSize(2);

	m_canvas->m_openGLScene->insert( gl_path_ODO );

	// Robot & scan:
	gl_robot = mrpt::opengl::stock_objects::RobotPioneer();
	m_canvas->m_openGLScene->insert( gl_robot );

	gl_scan = mrpt::opengl::CPlanarLaserScan::Create();
	gl_robot->insert(gl_scan);

	// Redirect all keystrokes in this box to the gl canvas:
	edInput->Connect(wxEVT_CHAR,(wxObjectEventFunction)&CMyGLCanvas::OnCharCustom, NULL, m_canvas );

	// fix sizes:
	SplitterWindow1->SetMinSize(wxSize(200,200));
	SplitterWindow1->SetSashPosition(200);
	Fit();
    Maximize();

}

gridmapSimulFrame::~gridmapSimulFrame()
{
    //(*Destroy(gridmapSimulFrame)
    //*)
}

void gridmapSimulFrame::update_grid_map_3d()
{
	if (!gl_grid) gl_grid = CSetOfObjects::Create();
	gl_grid->clear();
	the_grid.getAs3DObject( gl_grid );
}

void gridmapSimulFrame::OnbtnQuitClick(wxCommandEvent& event)
{
	we_are_closing = true;
	Close();
}

void gridmapSimulFrame::OntimRunTrigger(wxTimerEvent& event)
{
	static CTicTac	tictac;

	if (we_are_closing) return;

	double At = tictac.Tac();
	tictac.Tic();

	// Simul robot:
	the_robot.simulateOneTimeStep(At);

	try
	{
		// Use joystick:
		if (cbJoy->GetValue())
		{
			float       x,y,z;
			vector_bool btns;

			if (joystick.getJoystickPosition(0,x,y,z,btns))
			{
				const float V_MAX_ACC = 1;
				const float W_MAX_ACC =
						DEG2RAD(45.0f);

				const mrpt::math::TTwist2D vel_local = the_robot.getCurrentGTVelLocal();
				double v = vel_local.vx, w = vel_local.omega;

				v -= y* V_MAX_ACC*At;
				w -= x* W_MAX_ACC*At;

				if (!btns.empty() && btns[0])
				{
					v *= 0.95;
					w *= 0.95;
				}

				if (btns.size()>=2 && btns[1])
					m_canvas->cameraZoomDistance *= 0.99f;

				if (btns.size()>=3 && btns[2])
					m_canvas->cameraZoomDistance *= 1.01f;

				the_robot.movementCommand(v,w);
			}
		}

		// Use keyboard:
		{
			const float V_MAX_ACC = 1;
			const float W_MAX_ACC = DEG2RAD(45.0f);

			const mrpt::math::TTwist2D vel_local = the_robot.getCurrentGTVelLocal();
			double v = vel_local.vx, w = vel_local.omega;

			float x=0,y=0;

			switch(last_pressed_key)
			{
				case WXK_UP:    y=-1; break;
				case WXK_DOWN:  y= 1; break;
				case WXK_LEFT:  x=-1; break;
				case WXK_RIGHT: x= 1; break;

			}

			if (x!=0 || y!=0)
			{
				v -= y* V_MAX_ACC*At;
				w -= x* W_MAX_ACC*At;
				the_robot.movementCommand(v,w);
			}
			else
			if (last_pressed_key==WXK_SPACE)
			{
				v *= 0.95;
				w *= 0.95;
				the_robot.movementCommand(v,w);
			}

			last_pressed_key=0;
		}

		CPose2D p = the_robot.getCurrentGTPose();

		// Simulate scan:
		mrpt::obs::CObservation2DRangeScan the_scan;
		the_scan.sensorLabel = "LASER_SIM";
		the_scan.sensorPose.setFromValues(0.20,0,0.10);
		the_scan.maxRange = 80; //LASER_MAX_RANGE;
		the_scan.aperture = LASER_APERTURE;
		the_scan.stdError = LASER_STD_ERROR;

		static mrpt::utils::CTimeLogger timlog;

		timlog.enter("laserScanSimulator");
		the_grid.laserScanSimulator( the_scan, p, 0.6f, LASER_N_RANGES, LASER_STD_ERROR, 1, LASER_BEARING_STD_ERROR );
		timlog.leave("laserScanSimulator");

#ifdef DO_SCAN_LIKELIHOOD_DEBUG
		{
			timlog.enter("laserScanSimulatorWithUncertainty");
			COccupancyGridMap2D::TLaserSimulUncertaintyParams  ssu_params;
			COccupancyGridMap2D::TLaserSimulUncertaintyResult  ssu_out;

			ssu_params.robotPose.mean = p;
			ssu_params.robotPose.cov << square(0.25) , 0.0 , 0.0 , 0.0 , square(0.25) , 0.0 , 0.0 , 0.0 , square(DEG2RAD(5.0));

			the_grid.laserScanSimulatorWithUncertainty(ssu_params, ssu_out);
			timlog.leave("laserScanSimulatorWithUncertainty");

#if 1
			static mrpt::gui::CDisplayWindowPlots win;

			win.plot(ssu_out.scanWithUncert.rangeScan.scan, "3k-", "mean");
			win.plot(the_scan.scan, "r-", "obs");

			Eigen::VectorXd ci1 = ssu_out.scanWithUncert.rangesMean + 3*ssu_out.scanWithUncert.rangesCovar.diagonal().array().sqrt().matrix();
			Eigen::VectorXd ci2 = ssu_out.scanWithUncert.rangesMean - 3*ssu_out.scanWithUncert.rangesCovar.diagonal().array().sqrt().matrix();
			win.plot(ci1, "k-", "CI+");
			win.plot(ci2, "k-", "CI-");

			const double LIK = ssu_out.scanWithUncert.evaluateScanLikelihood(the_scan);
			win.setWindowTitle( mrpt::format("LIK: %.05f",LIK) );

			win.axis_fit();
#endif
		}
#endif

		// Save rawlog?
		// ----------------------------
		static CFileGZOutputStream	outs;
		static CFileOutputStream	out_GT;

		if (!btnStart->IsEnabled())
		{
			// Recording
			if (!outs.fileOpenCorrectly())
			{
				if (!outs.open( string(edOutFile->GetValue().mb_str()) ))
				{
					wxCommandEvent dum;
					OnbtnEndClick(dum);
					wxMessageBox( _("Cannot open output rawlog file."), _("Error"), wxOK, this);
				}

				if (!out_GT.open( string(edOutGT->GetValue().mb_str()) ))
				{
					outs.close();
					wxCommandEvent dum;
					OnbtnEndClick(dum);
					wxMessageBox( _("Cannot open output GT file."), _("Error"), wxOK, this);
				}

				// Save also the gridmap:
				CFileGZOutputStream	out_grid;
				string grid_file = string(edOutFile->GetValue().mb_str())+string("_grid.gridmap.gz");
				if (!out_grid.open( grid_file ) )
				{
					outs.close();
					out_GT.close();
					wxCommandEvent dum;
					OnbtnEndClick(dum);
					wxMessageBox( _("Cannot open output file for the reference gridmap!."), _("Error"), wxOK, this);
				}

				out_grid << the_grid; // save it
			}

			static long   decimation_count = 0;

			if (outs.fileOpenCorrectly() && 0==(decimation_count++ % decimation) )
			{
				// Desired rawlog format?
				const bool  is_sf_format = cbRawlogSFformat->GetValue();

				const TTimeStamp	tim_now = mrpt::system::now();
				CPose2D  odo_now = the_robot.getCurrentOdometricPose();

				if(is_sf_format)
				{
					// Action:
					CActionCollection acts;
					CActionRobotMovement2D	act;
					CActionRobotMovement2D::TMotionModelOptions	opts;


					opts.modelSelection = CActionRobotMovement2D::mmGaussian;

					CPose2D  Aodom = odo_now - lastOdo;
					lastOdo = odo_now;

					act.computeFromOdometry(Aodom, opts);
					act.timestamp = tim_now;

					acts.insert(act);
					outs << acts;

					// Observation:
					CSensoryFrame	sf;
					the_scan.timestamp = act.timestamp;
					sf.insert( CObservation2DRangeScanPtr( new CObservation2DRangeScan(the_scan)) );
					outs << sf;
				}
				else
				{
					// Observations only format:
					mrpt::obs::CObservationOdometry odo_obs;
					odo_obs.timestamp = tim_now;
					odo_obs.sensorLabel = "odometry";

					odo_obs.odometry = odo_now;

					odo_obs.hasVelocities = true;
					odo_obs.velocityLocal = the_robot.getCurrentGTVelLocal();

					outs << odo_obs << the_scan;
				}


				// And save to a text file the GT robot pose:
				CPose2D  cur_pose_relative = p - pose_start;

				out_GT.printf("%f %.03f %.03f %.03f\n",
					mrpt::system::timestampTotime_t(tim_now),
					cur_pose_relative.x(),
					cur_pose_relative.y(),
					cur_pose_relative.phi() );

				// save to lists:
				robot_path_GT.push_back( p );
				robot_path_ODO.push_back( odo_now );
			}
		}
		else
		{
			// Not recording
			if (outs.fileOpenCorrectly())
			{
				// Close it:
				outs.close();
				out_GT.close();
			}
		}

		// Update scan 3D:
		gl_scan->setScan(the_scan);

		// Update robot pose in the 3D scene:
		gl_robot->setPose( CPose3D( p ) );

		// Update robot path GT/odo:
		if (!robot_path_GT.empty())
		{
			gl_path_GT->insertPoint( robot_path_GT.rbegin()->x(), robot_path_GT.rbegin()->y(), 0 );
			CPose2D  this_odo = pose_start + *robot_path_ODO.rbegin();
			gl_path_ODO->insertPoint( this_odo.x(), this_odo.y(), 0 );
		}

		m_canvas->cameraPointingX = p.x();
		m_canvas->cameraPointingY = p.y();

		m_canvas->Refresh();


		// Prepare next interval
		if (!we_are_closing)
			timRun.Start(TIMER_MS, true);
    }
	catch(std::exception &e)
    {
    	wxCommandEvent dum;
    	OnbtnEndClick(dum);
        wxMessageBox( wxString(e.what(),wxConvUTF8), wxT("Exception"), wxOK, this);
    }
    catch(...)
    {
    	wxCommandEvent dum;
    	OnbtnEndClick(dum);
        wxMessageBox( _("Untyped exception!"), _("Exception"), wxOK, this);
    }
}

void gridmapSimulFrame::OnbtnStartClick(wxCommandEvent& event)
{
	btnStart->Enable(false);
	btnEnd->Enable(true);

	robot_path_GT.clear();
	robot_path_ODO.clear();

	gl_path_GT->clear();
	gl_path_ODO->clear();

	pose_start = the_robot.getCurrentGTPose( );
	the_robot.setCurrentOdometricPose( TPose2D(0,0,0) );
	lastOdo = CPose2D(0,0,0);


	// odo errors:
	double Ax_err_bias;
	double Ax_err_std;
	double Ay_err_bias;
	double Ay_err_std;
	double Aphi_err_bias;
	double Aphi_err_std;

	edAxb->GetValue().ToDouble( &Ax_err_bias );
	edAxs->GetValue().ToDouble( &Ax_err_std );

	edAyb->GetValue().ToDouble( &Ay_err_bias );
	edAys->GetValue().ToDouble( &Ay_err_std );

	edApb->GetValue().ToDouble( &Aphi_err_bias );
	edAps->GetValue().ToDouble( &Aphi_err_std );
	Aphi_err_bias = DEG2RAD( Aphi_err_bias );
	Aphi_err_std = DEG2RAD( Aphi_err_std );

	the_robot.setOdometryErrors(true,
		Ax_err_bias, Ax_err_std,
		Ay_err_bias, Ay_err_std,
		Aphi_err_bias, Aphi_err_std );

	// Get decimation:
	decimation = edDecimate->GetValue();
}

void gridmapSimulFrame::OnbtnEndClick(wxCommandEvent& event)
{
	btnStart->Enable(true);
	btnEnd->Enable(false);
}

void gridmapSimulFrame::OnMenuLoadMap(wxCommandEvent& event)
{
	WX_START_TRY

	wxFileDialog dlg(
		this,
		_("Select grid map to load"),
		_("."),
		_("grid.png"),
		wxT("Image files (*.png,*.jpg,*.gif)|*.png;*.jpg;*.gif|Binary gridmap files (*.gridmap,*.gridmap.gz)|*.gridmap;*.gridmap.gz|All files (*.*)|*.*"),
		wxFD_OPEN | wxFD_FILE_MUST_EXIST );

	if (dlg.ShowModal() != wxID_OK)
		return;

	const wxString sFil =  dlg.GetPath();
	const std::string fil = std::string(sFil.mb_str());

	const std::string fil_ext = mrpt::system::extractFileExtension(fil,true);

	if (mrpt::system::lowerCase(fil_ext)=="gridmap")
	{
		CFileGZInputStream f(fil);
		f >> the_grid;
		update_grid_map_3d();
	}
	else
	{
		// Try loading the image:
		CImage img;
		if (!img.loadFromFile(fil, 0 /* force grayscale */ ))
		{
			wxMessageBox(_("Error"),_("Can't load the image file (check its format)."));
		}
		else
		{
			// We also need the size of each pixel:
			double cx =-1;
			double cy =-1;
			double cell_size = 0.05;

			const wxString sCellSize = wxGetTextFromUser(_("Enter the size (in meters) of each pixel:"),_("Grid parameters"),_("0.05"), this);
			const wxString sCX = wxGetTextFromUser(_("Enter the central pixel (x-coordinate), or -1 = the image center:"),_("Grid parameters"),_("-1"), this);
			const wxString sCY = wxGetTextFromUser(_("Enter the central pixel (y-coordinate), or -1 = the image center:"),_("Grid parameters"),_("-1"), this);

			if (sCellSize.ToDouble(&cell_size) && sCX.ToDouble(&cx) && sCY.ToDouble(&cy) )
			{
				if (the_grid.loadFromBitmap(img,cell_size,cx,cy))
				{
					update_grid_map_3d();
					wxMessageBox(_("OK"),_("Map loaded!"));
				}
				else
					wxMessageBox(_("Error"),_("Can't load the image file into the gridmap..."));
			}
			else
				wxMessageBox(_("Error"),_("Error parsing the numbers you entered..."));
		}
	}

	WX_END_TRY
}

void gridmapSimulFrame::OnbtnExploreClick(wxCommandEvent& event)
{
	wxFileDialog dialog(this, _("Rawlog to create.."), _("."), _("simul.rawlog"),wxT("RawLog files (*.rawlog)|*.rawlog|All files (*.*)|*.*"), wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

	if (dialog.ShowModal() == wxID_OK)
	{
		edOutFile->SetValue( dialog.GetPath() );
		edOutGT->SetValue( dialog.GetPath() + _(".GT.txt") );
	}
}

void gridmapSimulFrame::OnbtnSetLaserClick(wxCommandEvent& event)
{
	edSpan->GetValue().ToDouble( &LASER_APERTURE );
	LASER_APERTURE=DEG2RAD(LASER_APERTURE);

	edCount->GetValue().ToLong(  &LASER_N_RANGES );

	edStdNoise->GetValue().ToDouble( &LASER_STD_ERROR );

	edStdNoiseAng->GetValue().ToDouble( &LASER_BEARING_STD_ERROR );
	LASER_BEARING_STD_ERROR = DEG2RAD(LASER_BEARING_STD_ERROR);
}

void gridmapSimulFrame::OnAbout(wxCommandEvent& event)
{
	CAboutBox	dialog(this);
	dialog.ShowModal();
}

void gridmapSimulFrame::OnbtnResimulateClick(wxCommandEvent& event)
{
	WX_START_TRY

    string gt_file;
    {
        wxFileDialog dialog(this, _("Existing GT file..."), _("."), _("GT.txt"),wxT("Text files (*.txt)|*.txt|All files (*.*)|*.*"), wxFD_OPEN | wxFD_FILE_MUST_EXIST );
        if (dialog.ShowModal() != wxID_OK) return;
        gt_file = string( dialog.GetPath().mb_str() );
    }

    string raw_file;
    {
        wxFileDialog dialog(this, _("Corresponding rawlog..."), _U( mrpt::system::extractFileDirectory(gt_file).c_str() ), _("simul.rawlog"),wxT("Rawlog files (*.rawlog)|*.rawlog|All files (*.*)|*.*"), wxFD_OPEN | wxFD_FILE_MUST_EXIST );
        if (dialog.ShowModal() != wxID_OK) return;
        raw_file = string( dialog.GetPath().mb_str() );
    }


	// GT file rows are:
	//  time x y z
	CMatrixDouble	GT;
	GT.loadFromTextFile(gt_file);

	// Rawlog:
	CRawlog  rawlog;
	rawlog.loadFromRawLogFile(raw_file);

	//cout << "rawlog entries: " << rawlog.size() << endl;

	// Assert sizes:
    ASSERT_( rawlog.size()>0 );
    ASSERT_( GT.getColCount() >= 4 );
    ASSERT_( rawlog.getType(0) == CRawlog::etActionCollection );
	ASSERT_( rawlog.size()/2 == GT.getRowCount() );

	// Ask for the output:
    string out_raw_file, out_GT_file;
    {
        wxFileDialog dialog(this, _("New rawlog to create..."), _U( mrpt::system::extractFileDirectory(gt_file).c_str() ), _("simul_new.rawlog"),wxT("Rawlog files (*.rawlog)|*.rawlog|All files (*.*)|*.*"), wxFD_SAVE | wxFD_OVERWRITE_PROMPT );
        if (dialog.ShowModal() != wxID_OK) return;
        out_raw_file = string( dialog.GetPath().mb_str() );
        out_GT_file  = out_raw_file  + string(".GT.txt");
    }

    // ------------------------------------------
    // Resimulate scans:
    // ------------------------------------------
    wxCommandEvent	ee;
    OnbtnSetLaserClick(ee);

    for (size_t i=1;i<rawlog.size();i+=2)
    {
    	ASSERT_( rawlog.getType(i) == CRawlog::etSensoryFrame );

    	CSensoryFramePtr sf = rawlog.getAsObservations(i);
    	CPose2D  gt_pose( GT(i/2,1),GT(i/2,2),GT(i/2,3) );

		CObservation2DRangeScanPtr the_scan = sf->getObservationByClass<CObservation2DRangeScan>();
		the_grid.laserScanSimulator( *the_scan, gt_pose, 0.5f, LASER_N_RANGES, LASER_STD_ERROR, 1, LASER_BEARING_STD_ERROR );
    }

    // Save the new rawlog:
    CFileGZOutputStream(out_raw_file) << rawlog;
    //rawlog.saveToRawLogFile(out_raw_file);

    // The GT file is the same:
    GT.saveToTextFile(out_GT_file,MATRIX_FORMAT_FIXED);

	WX_END_TRY
}
