/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "holonomic_navigator_demoMain.h"
#include <wx/msgdlg.h>
#include "CAboutBox.h"

//(*InternalHeaders(holonomic_navigator_demoFrame)
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/tglbtn.h>
#include <wx/settings.h>
#include <wx/font.h>
#include <wx/intl.h>
#include <wx/image.h>
#include <wx/string.h>
//*)

#include <mrpt/gui/WxUtils.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include "imgs/main_icon.xpm"
#include "../wx-common/mrpt_logo.xpm"

#include "../ReactiveNavigationDemo/DEFAULT_GRIDMAP_DATA.h"

// A custom Art provider for customizing the icons:
class MyArtProvider : public wxArtProvider
{
protected:
    virtual wxBitmap CreateBitmap(const wxArtID& id,
                                  const wxArtClient& client,
                                  const wxSize& size);
};

// CreateBitmap function
wxBitmap MyArtProvider::CreateBitmap(const wxArtID& id,
                                     const wxArtClient& client,
                                     const wxSize& size)
{
    if (id == wxART_MAKE_ART_ID(MAIN_ICON))   return wxBitmap(main_icon_xpm);
    if (id == wxART_MAKE_ART_ID(IMG_MRPT_LOGO))  return wxBitmap(mrpt_logo_xpm);

    // Any wxWidgets icons not implemented here
    // will be provided by the default art provider.
    return wxNullBitmap;
}

#include <mrpt/nav.h>
#include <mrpt/gui.h>
#include <mrpt/opengl.h>
#include <mrpt/system/filesystem.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::nav;
using namespace mrpt::poses;
using namespace std;


//(*IdInit(holonomic_navigator_demoFrame)
const long holonomic_navigator_demoFrame::ID_BUTTON1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_BUTTON2 = wxNewId();
const long holonomic_navigator_demoFrame::ID_BUTTON3 = wxNewId();
const long holonomic_navigator_demoFrame::ID_BUTTON6 = wxNewId();
const long holonomic_navigator_demoFrame::ID_BUTTON7 = wxNewId();
const long holonomic_navigator_demoFrame::ID_BUTTON4 = wxNewId();
const long holonomic_navigator_demoFrame::ID_BUTTON5 = wxNewId();
const long holonomic_navigator_demoFrame::ID_RADIOBOX1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_TEXTCTRL1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_PANEL1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_PANEL2 = wxNewId();
const long holonomic_navigator_demoFrame::ID_NOTEBOOK1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_STATICTEXT2 = wxNewId();
const long holonomic_navigator_demoFrame::ID_STATICTEXT1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_XY_GLCANVAS = wxNewId();
const long holonomic_navigator_demoFrame::ID_CUSTOM1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_TEXTCTRL2 = wxNewId();
const long holonomic_navigator_demoFrame::ID_MENUITEM4 = wxNewId();
const long holonomic_navigator_demoFrame::idMenuQuit = wxNewId();
const long holonomic_navigator_demoFrame::ID_MENUITEM1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_MENUITEM2 = wxNewId();
const long holonomic_navigator_demoFrame::ID_MENUITEM3 = wxNewId();
const long holonomic_navigator_demoFrame::idMenuAbout = wxNewId();
const long holonomic_navigator_demoFrame::ID_STATUSBAR1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_TIMER1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(holonomic_navigator_demoFrame,wxFrame)
    //(*EventTable(holonomic_navigator_demoFrame)
    //*)
END_EVENT_TABLE()

holonomic_navigator_demoFrame::holonomic_navigator_demoFrame(wxWindow* parent,wxWindowID id) :
	m_holonomicMethod   ( NULL ),
	m_gridMap(),
	m_targetPoint(-5,-7),
	m_robotPose(0,0,0),
	m_curCursorPos(0,0),
	m_cursorPickState(cpsNone)
{
    // Load my custom icons:
#if wxCHECK_VERSION(2, 8, 0)
    wxArtProvider::Push(new MyArtProvider);
#else
    wxArtProvider::PushProvider(new MyArtProvider);
#endif


    //(*Initialize(holonomic_navigator_demoFrame)
    wxFlexGridSizer* FlexGridSizer4;
    wxMenuItem* MenuItem2;
    wxFlexGridSizer* FlexGridSizer3;
    wxMenuItem* MenuItem1;
    wxFlexGridSizer* FlexGridSizer5;
    wxFlexGridSizer* FlexGridSizer2;
    wxBoxSizer* BoxSizer2;
    wxMenu* Menu1;
    wxFlexGridSizer* FlexGridSizer7;
    wxBoxSizer* BoxSizer1;
    wxMenuBar* MenuBar1;
    wxFlexGridSizer* FlexGridSizer1;
    wxMenu* Menu2;

    Create(parent, id, _("Holonomic Navigation Demo - Part of MRPT"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("id"));
    FlexGridSizer1 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer1->AddGrowableCol(0);
    FlexGridSizer1->AddGrowableRow(1);
    FlexGridSizer2 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer2->AddGrowableCol(1);
    FlexGridSizer4 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer4->AddGrowableCol(0);
    FlexGridSizer4->AddGrowableRow(0);
    FlexGridSizer4->AddGrowableRow(1);
    BoxSizer1 = new wxBoxSizer(wxHORIZONTAL);
    btnLoadMap = new wxCustomButton(this,ID_BUTTON1,_("Load map..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FOLDER")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON1"));
    btnLoadMap->SetBitmapDisabled(btnLoadMap->CreateBitmapDisabled(btnLoadMap->GetBitmapLabel()));
    btnLoadMap->SetBitmapMargin(wxSize(2,4));
    BoxSizer1->Add(btnLoadMap, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    btnHelp = new wxCustomButton(this,ID_BUTTON2,_("About..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_QUESTION")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON2"));
    btnHelp->SetBitmapDisabled(btnHelp->CreateBitmapDisabled(btnHelp->GetBitmapLabel()));
    btnHelp->SetBitmapMargin(wxSize(5,4));
    BoxSizer1->Add(btnHelp, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    btnQuit = new wxCustomButton(this,ID_BUTTON3,_("Exit"),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_QUIT")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON3"));
    btnQuit->SetBitmapDisabled(btnQuit->CreateBitmapDisabled(btnQuit->GetBitmapLabel()));
    BoxSizer1->Add(btnQuit, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    FlexGridSizer4->Add(BoxSizer1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    BoxSizer2 = new wxBoxSizer(wxHORIZONTAL);
    btnPlaceRobot = new wxCustomButton(this,ID_BUTTON6,_("Place robot..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FIND")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON6"));
    btnPlaceRobot->SetBitmapDisabled(btnPlaceRobot->CreateBitmapDisabled(btnPlaceRobot->GetBitmapLabel()));
    btnPlaceRobot->SetBitmapMargin(wxSize(2,4));
    BoxSizer2->Add(btnPlaceRobot, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    btnPlaceTarget = new wxCustomButton(this,ID_BUTTON7,_("Set target..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FIND")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON7"));
    btnPlaceTarget->SetBitmapDisabled(btnPlaceTarget->CreateBitmapDisabled(btnPlaceTarget->GetBitmapLabel()));
    btnPlaceTarget->SetBitmapMargin(wxSize(2,4));
    BoxSizer2->Add(btnPlaceTarget, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    btnStart = new wxCustomButton(this,ID_BUTTON4,_("START"),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_GO_FORWARD")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON4"));
    btnStart->SetBitmapDisabled(btnStart->CreateBitmapDisabled(btnStart->GetBitmapLabel()));
    btnStart->SetBitmapMargin(wxSize(2,4));
    BoxSizer2->Add(btnStart, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    btnStop = new wxCustomButton(this,ID_BUTTON5,_("STOP"),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_CROSS_MARK")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON5"));
    btnStop->SetBitmapDisabled(btnStop->CreateBitmapDisabled(btnStop->GetBitmapLabel()));
    btnStop->SetBitmapMargin(wxSize(2,4));
    BoxSizer2->Add(btnStop, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    FlexGridSizer4->Add(BoxSizer2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer2->Add(FlexGridSizer4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Notebook1 = new wxNotebook(this, ID_NOTEBOOK1, wxDefaultPosition, wxDefaultSize, 0, _T("ID_NOTEBOOK1"));
    Panel1 = new wxPanel(Notebook1, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    FlexGridSizer7 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer7->AddGrowableCol(1);
    FlexGridSizer7->AddGrowableRow(0);
    wxString __wxRadioBoxChoices_1[2] =
    {
    	_("VFF (Virtual Force Field)"),
    	_("ND (Nearness  Diagram)")
    };
    rbHoloMethod = new wxRadioBox(Panel1, ID_RADIOBOX1, _(" Holonomic method "), wxDefaultPosition, wxDefaultSize, 2, __wxRadioBoxChoices_1, 1, wxRA_HORIZONTAL, wxDefaultValidator, _T("ID_RADIOBOX1"));
    FlexGridSizer7->Add(rbHoloMethod, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edHoloParams = new wxTextCtrl(Panel1, ID_TEXTCTRL1, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER|wxTE_PROCESS_TAB|wxTE_MULTILINE|wxVSCROLL|wxHSCROLL|wxALWAYS_SHOW_SB, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    edHoloParams->SetMinSize(wxSize(-1,100));
    wxFont edHoloParamsFont(8,wxFONTFAMILY_TELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    edHoloParams->SetFont(edHoloParamsFont);
    FlexGridSizer7->Add(edHoloParams, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    Panel1->SetSizer(FlexGridSizer7);
    FlexGridSizer7->Fit(Panel1);
    FlexGridSizer7->SetSizeHints(Panel1);
    Panel2 = new wxPanel(Notebook1, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
    Notebook1->AddPage(Panel1, _("Configuration"), true);
    Notebook1->AddPage(Panel2, _("Stats"), false);
    FlexGridSizer2->Add(Notebook1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    FlexGridSizer1->Add(FlexGridSizer2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer3 = new wxFlexGridSizer(2, 2, 0, 0);
    FlexGridSizer3->AddGrowableCol(0);
    FlexGridSizer3->AddGrowableCol(1);
    FlexGridSizer3->AddGrowableRow(1);
    StaticText2 = new wxStaticText(this, ID_STATICTEXT2, _("[3D view of the simulated world]"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    FlexGridSizer3->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText1 = new wxStaticText(this, ID_STATICTEXT1, _("[Local view]"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    FlexGridSizer3->Add(StaticText1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    m_plot3D = new CMyGLCanvas(this,ID_XY_GLCANVAS,wxDefaultPosition,wxSize(450,350),wxTAB_TRAVERSAL,_T("ID_XY_GLCANVAS"));
    FlexGridSizer3->Add(m_plot3D, 1, wxALL|wxEXPAND|wxFIXED_MINSIZE|wxALIGN_LEFT|wxALIGN_TOP, 1);
    FlexGridSizer5 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer5->AddGrowableCol(0);
    FlexGridSizer5->AddGrowableRow(0);
    m_plotScan = new CMyGLCanvas(this,ID_CUSTOM1,wxDefaultPosition,wxSize(150,150),wxTAB_TRAVERSAL,_T("ID_CUSTOM1"));
    FlexGridSizer5->Add(m_plotScan, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    edInfoLocalView = new wxTextCtrl(this, ID_TEXTCTRL2, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE|wxTE_READONLY|wxTE_DONTWRAP|wxALWAYS_SHOW_SB, wxDefaultValidator, _T("ID_TEXTCTRL2"));
    edInfoLocalView->SetMinSize(wxSize(-1,50));
    edInfoLocalView->SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BTNFACE));
    wxFont edInfoLocalViewFont(8,wxFONTFAMILY_TELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    edInfoLocalView->SetFont(edInfoLocalViewFont);
    FlexGridSizer5->Add(edInfoLocalView, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    FlexGridSizer3->Add(FlexGridSizer5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer1->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    SetSizer(FlexGridSizer1);
    MenuBar1 = new wxMenuBar();
    Menu1 = new wxMenu();
    MenuItem3 = new wxMenuItem(Menu1, ID_MENUITEM4, _("Load map..."), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(MenuItem3);
    MenuItem1 = new wxMenuItem(Menu1, idMenuQuit, _("Quit\tAlt-F4"), _("Quit the application"), wxITEM_NORMAL);
    Menu1->Append(MenuItem1);
    MenuBar1->Append(Menu1, _("&File"));
    Menu3 = new wxMenu();
    mnuViewMaxRange = new wxMenuItem(Menu3, ID_MENUITEM1, _("Maximum sensor range"), wxEmptyString, wxITEM_CHECK);
    Menu3->Append(mnuViewMaxRange);
    mnuViewRobotPath = new wxMenuItem(Menu3, ID_MENUITEM2, _("Robot path"), wxEmptyString, wxITEM_CHECK);
    Menu3->Append(mnuViewRobotPath);
    Menu3->AppendSeparator();
    MenuItem5 = new wxMenuItem(Menu3, ID_MENUITEM3, _("Clear robot path"), wxEmptyString, wxITEM_NORMAL);
    Menu3->Append(MenuItem5);
    MenuBar1->Append(Menu3, _("&View"));
    Menu2 = new wxMenu();
    MenuItem2 = new wxMenuItem(Menu2, idMenuAbout, _("About\tF1"), _("Show info about this application"), wxITEM_NORMAL);
    Menu2->Append(MenuItem2);
    MenuBar1->Append(Menu2, _("Help"));
    SetMenuBar(MenuBar1);
    StatusBar1 = new wxStatusBar(this, ID_STATUSBAR1, 0, _T("ID_STATUSBAR1"));
    int __wxStatusBarWidths_1[3] = { -2, -2, -3 };
    int __wxStatusBarStyles_1[3] = { wxSB_NORMAL, wxSB_NORMAL, wxSB_NORMAL };
    StatusBar1->SetFieldsCount(3,__wxStatusBarWidths_1);
    StatusBar1->SetStatusStyles(3,__wxStatusBarStyles_1);
    SetStatusBar(StatusBar1);
    timRunSimul.SetOwner(this, ID_TIMER1);
    timRunSimul.Start(10, false);
    FlexGridSizer1->Fit(this);
    FlexGridSizer1->SetSizeHints(this);
    Center();

    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnbtnLoadMapClick);
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnAbout);
    Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnQuit);
    Connect(ID_BUTTON6,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnbtnPlaceRobotClick);
    Connect(ID_BUTTON7,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnbtnPlaceTargetClick);
    Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnbtnStartClick);
    Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnbtnStopClick);
    Connect(ID_MENUITEM4,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnbtnLoadMapClick);
    Connect(idMenuQuit,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnQuit);
    Connect(ID_MENUITEM1,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnMenuItemChangeVisibleStuff);
    Connect(ID_MENUITEM2,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnMenuItemChangeVisibleStuff);
    Connect(ID_MENUITEM3,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnMenuItemClearRobotPath);
    Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnAbout);
    Connect(ID_TIMER1,wxEVT_TIMER,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OntimRunSimulTrigger);
    //*)

	m_plot3D->Connect(wxEVT_MOTION,(wxObjectEventFunction)&holonomic_navigator_demoFrame::Onplot3DMouseMove,0,this);
	m_plot3D->Connect(wxEVT_LEFT_DOWN,(wxObjectEventFunction)&holonomic_navigator_demoFrame::Onplot3DMouseClick,0,this);

	mnuViewMaxRange->Check(true);
	mnuViewRobotPath->Check(true);

	btnStart->Enable(true);
	btnStop->Enable(false);

	WX_START_TRY

	// Initialize gridmap:
	// -------------------------------
	CMemoryStream  s( DEFAULT_GRIDMAP_DATA, sizeof(DEFAULT_GRIDMAP_DATA) );
	s >> m_gridMap;

	// Populate 3D views:
	// -------------------------------
	{
		mrpt::opengl::CGridPlaneXYPtr obj = mrpt::opengl::CGridPlaneXY::Create(-50,50, -50,50, 0, 1);
		obj->setColor_u8(TColor(30,30,30,50));
		m_plot3D->m_openGLScene->insert( obj );
	}

	gl_grid = mrpt::opengl::CSetOfObjects::Create();
	m_plot3D->m_openGLScene->insert(gl_grid);
	this->updateMap3DView();

	gl_robot = mrpt::opengl::CSetOfObjects::Create();
	{
		mrpt::opengl::CCylinderPtr obj = mrpt::opengl::CCylinder::Create(0.2f,0.1f,0.9f);
		obj->setColor_u8( TColor::red );
		gl_robot->insert( obj );
	}
	m_plot3D->m_openGLScene->insert(gl_robot);

	gl_scan3D = mrpt::opengl::CPlanarLaserScan::Create();
	gl_scan3D->enableLine(false);
	gl_scan3D->setPointsWidth(3.0);
	gl_robot->insert(gl_scan3D);

	gl_robot_sensor_range = mrpt::opengl::CDisk::Create(0,0);
	gl_robot_sensor_range->setColor_u8( TColor(0,0,255, 90) );
	gl_robot_sensor_range->setLocation(0,0,0.01);
	gl_robot->insert(gl_robot_sensor_range);

	gl_robot_path = mrpt::opengl::CSetOfLines::Create();
	gl_robot_path->setLineWidth(1);
	gl_robot_path->setColor_u8( TColor(40,40,40, 200));
	m_plot3D->m_openGLScene->insert(gl_robot_path);

	gl_target = mrpt::opengl::CSetOfObjects::Create();
	{
		mrpt::opengl::CArrowPtr obj;
		obj = mrpt::opengl::CArrow::Create( 1,0,0,  0.2f,0,0, 0.4f,0.05f, 0.15f ); obj->setColor_u8( TColor(0,0,255) ); gl_target->insert(obj);
		obj = mrpt::opengl::CArrow::Create(-1,0,0, -0.2f,0,0, 0.4f,0.05f, 0.15f ); obj->setColor_u8( TColor(0,0,255) ); gl_target->insert(obj);
		obj = mrpt::opengl::CArrow::Create( 0,1,0,  0,0.2f,0, 0.4f,0.05f, 0.15f ); obj->setColor_u8( TColor(0,0,255) ); gl_target->insert(obj);
		obj = mrpt::opengl::CArrow::Create(0,-1,0,  0,-0.2f,0, 0.4f,0.05f, 0.15f ); obj->setColor_u8( TColor(0,0,255) ); gl_target->insert(obj);
		m_plot3D->m_openGLScene->insert(gl_target);
	}

	{	// Sign of "picking a navigation target":
		m_gl_placing_nav_target = opengl::CSetOfObjects::Create();

		mrpt::opengl::CArrowPtr obj;
		obj = mrpt::opengl::CArrow::Create( 1,0,0,  0.2f,0,0, 0.4f,0.05f, 0.15f ); obj->setColor_u8( TColor(0,0,255) ); m_gl_placing_nav_target->insert(obj);
		obj = mrpt::opengl::CArrow::Create(-1,0,0, -0.2f,0,0, 0.4f,0.05f, 0.15f ); obj->setColor_u8( TColor(0,0,255) ); m_gl_placing_nav_target->insert(obj);
		obj = mrpt::opengl::CArrow::Create( 0,1,0,  0,0.2f,0, 0.4f,0.05f, 0.15f ); obj->setColor_u8( TColor(0,0,255) ); m_gl_placing_nav_target->insert(obj);
		obj = mrpt::opengl::CArrow::Create(0,-1,0,  0,-0.2f,0, 0.4f,0.05f, 0.15f ); obj->setColor_u8( TColor(0,0,255) ); m_gl_placing_nav_target->insert(obj);
		m_gl_placing_nav_target->setVisibility(false); // Start invisible.
		m_plot3D->m_openGLScene->insert(m_gl_placing_nav_target);
	}
	{	// Sign of "replacing the robot":
		m_gl_placing_robot = opengl::CSetOfObjects::Create();
		mrpt::opengl::CCylinderPtr obj = mrpt::opengl::CCylinder::Create(0.2f,0.1f,0.9f);
		obj->setColor_u8( TColor(255,0,0, 120) );
		m_gl_placing_robot->insert(obj);

		m_gl_placing_robot->setVisibility(false); // Start invisible.
		m_plot3D->m_openGLScene->insert(m_gl_placing_robot);
	}

	m_plot3D->m_openGLScene->insert( mrpt::opengl::stock_objects::CornerXYZ(1) );

	// Set camera:
	m_plot3D->cameraPointingX=0;
	m_plot3D->cameraPointingY=0;
	m_plot3D->cameraPointingZ=0;
	m_plot3D->cameraZoomDistance = 40;
	m_plot3D->cameraElevationDeg = 70;
	m_plot3D->cameraAzimuthDeg = -100;
	m_plot3D->cameraIsProjective = true;


	// 2D view ==============
	{
		mrpt::opengl::CGridPlaneXYPtr obj = mrpt::opengl::CGridPlaneXY::Create(-1,1.001f, -1,1.001f, 0, 1);
		obj->setColor_u8(TColor(30,30,30,50));
		m_plotScan->m_openGLScene->insert( obj );
	}

	gl_scan2D = mrpt::opengl::CPlanarLaserScan::Create();
	gl_scan2D->enableLine(false);
	gl_scan2D->enableSurface(false);
	gl_scan2D->setPointsWidth(3.0);
	m_plotScan->m_openGLScene->insert(gl_scan2D);

	gl_line_direction = mrpt::opengl::CSimpleLine::Create();
	gl_line_direction->setLineWidth(4);
	gl_line_direction->setColor_u8(TColor(0,0,0));
	m_plotScan->m_openGLScene->insert(gl_line_direction);

	gl_rel_target = mrpt::opengl::CPointCloud::Create();
	gl_rel_target->setPointSize(7);
	gl_rel_target->setColor_u8(TColor(0,0,255));
	gl_rel_target->insertPoint(0,0,0);
	m_plotScan->m_openGLScene->insert(gl_rel_target);

	m_plotScan->m_openGLScene->insert( mrpt::opengl::stock_objects::CornerXYSimple(0.1f,2) );

	gl_nd_gaps = mrpt::opengl::CSetOfLines::Create();
	gl_nd_gaps->setLineWidth(2);
	gl_nd_gaps->setColor_u8( TColor(204,102,51) );
	m_plotScan->m_openGLScene->insert(gl_nd_gaps);

	m_plotScan->clearColorR = 0.9f;
	m_plotScan->clearColorG = 0.9f;
	m_plotScan->clearColorB = 0.9f;

	// Set camera:
	m_plotScan->cameraPointingX=0;
	m_plotScan->cameraPointingY=0;
	m_plotScan->cameraPointingZ=0;
	m_plotScan->cameraZoomDistance = 2.2f;
	m_plotScan->cameraElevationDeg = 90;
	m_plotScan->cameraAzimuthDeg = -90;
	m_plotScan->cameraIsProjective = false;


	// Update positions of stuff:
	this->updateViewsDynamicObjects();


	// Retrieve default parameters for holonomic methods:
	// ------------------------------------------------------
	{
		mrpt::utils::CConfigFileMemory cfg;

		m_simul_options.saveToConfigFile(cfg,"SIMULATOR");

		mrpt::nav::CHolonomicVFF holo_VFF;
		holo_VFF.options.saveToConfigFile(cfg,"VFF_CONFIG");

		mrpt::nav::CHolonomicND holo_ND;
		holo_ND.options.saveToConfigFile(cfg,"ND_CONFIG");

		this->edHoloParams->SetValue( _U( cfg.getContent().c_str() ) );
	}

	WX_END_TRY


	this->Maximize();
}

holonomic_navigator_demoFrame::~holonomic_navigator_demoFrame()
{
    //(*Destroy(holonomic_navigator_demoFrame)
    //*)
}

void holonomic_navigator_demoFrame::OnQuit(wxCommandEvent& event)
{
    Close();
}

void holonomic_navigator_demoFrame::OnAbout(wxCommandEvent& event)
{
	CAboutBox dlg(this);
	dlg.ShowModal();
}

void holonomic_navigator_demoFrame::updateMap3DView()
{
	gl_grid->clear();
	m_gridMap.getAs3DObject(gl_grid);
}


void holonomic_navigator_demoFrame::OnbtnPlaceRobotClick(wxCommandEvent& event)
{
	if (m_cursorPickState!=cpsPlaceRobot)
	{
		m_cursorPickState = cpsPlaceRobot;
		m_plot3D->SetCursor( *wxCROSS_CURSOR );
	}
	else
	{	// Cancel:
		m_cursorPickState = cpsNone;
		m_plot3D->SetCursor( *wxSTANDARD_CURSOR );
		m_gl_placing_robot->setVisibility(false);
	}
	btnPlaceRobot->SetValue( m_cursorPickState == cpsPlaceRobot );
	btnPlaceRobot->Refresh();
}

void holonomic_navigator_demoFrame::OnbtnPlaceTargetClick(wxCommandEvent& event)
{
	if (m_cursorPickState!=cpsPickTarget)
	{
		m_cursorPickState = cpsPickTarget;
		m_plot3D->SetCursor( *wxCROSS_CURSOR );
	}
	else
	{	// Cancel:
		m_cursorPickState = cpsNone;
		m_plot3D->SetCursor( *wxSTANDARD_CURSOR );
		m_gl_placing_nav_target->setVisibility(false);
	}
	btnPlaceTarget->SetValue( m_cursorPickState == cpsPickTarget );
	btnPlaceTarget->Refresh();
}

void holonomic_navigator_demoFrame::OnbtnStartClick(wxCommandEvent& event)
{
	reinitSimulator();

	btnStart->Enable(false); btnStart->Refresh();
	btnStop->Enable(true);   btnStop->Refresh();
	edHoloParams->Enable(false);
	rbHoloMethod->Enable(false);
}

void holonomic_navigator_demoFrame::OnbtnStopClick(wxCommandEvent& event)
{
	btnStart->Enable(true); btnStart->Refresh();
	btnStop->Enable(false);   btnStop->Refresh();
	edHoloParams->Enable(true);
	rbHoloMethod->Enable(true);
}

// Run simulator (when "running"):
void holonomic_navigator_demoFrame::OntimRunSimulTrigger(wxTimerEvent& event)
{
	try
	{
		if (btnStop->IsEnabled())
		{
			simulateOneStep(event.GetInterval()*1e-3);
		}
		updateViewsDynamicObjects();
	}
	catch (std::exception &e)
	{
		wxMessageBox( wxString(e.what(),wxConvUTF8), wxT("Exception"), wxOK, this);
		// Stop:
		wxCommandEvent ev;
		OnbtnStopClick(ev);
	}
}

// Create navigator object & load params from GUI:
void holonomic_navigator_demoFrame::reinitSimulator()
{
	WX_START_TRY

	// Delete old & build new navigator:
	mrpt::utils::delete_safe(m_holonomicMethod);
	switch (rbHoloMethod->GetSelection())
	{
	case 0: m_holonomicMethod = new mrpt::nav::CHolonomicVFF(); break;
	case 1: m_holonomicMethod = new mrpt::nav::CHolonomicND(); break;
	default:
		throw std::runtime_error("Invalid holonomic method selected!");
	};

	// Load params:
	{
		CConfigFileMemory cfg;
		cfg.setContent( std::string(edHoloParams->GetValue().mb_str() ) );
		m_holonomicMethod->initialize(cfg);

		m_simul_options.loadFromConfigFile(cfg,"SIMULATOR");
	}

	// Update GUI stuff:
	gl_robot_sensor_range->setDiskRadius(m_simul_options.MAX_SENSOR_RADIUS*1.01,m_simul_options.MAX_SENSOR_RADIUS*0.99);


	WX_END_TRY
}

void holonomic_navigator_demoFrame::simulateOneStep(double time_step)
{
	// Simulate 360deg range scan:
	CObservation2DRangeScan      simulatedScan;

	simulatedScan.aperture = M_2PIf;
	simulatedScan.rightToLeft = true;
	simulatedScan.maxRange = m_simul_options.MAX_SENSOR_RADIUS;
	simulatedScan.sensorPose = CPose2D(0,0,0);

	m_gridMap.laserScanSimulator( simulatedScan, m_robotPose,0.5, m_simul_options.SENSOR_NUM_RANGES, m_simul_options.SENSOR_RANGE_NOISE_STD );

	gl_scan3D->setScan( simulatedScan );  // Draw real scan in 3D view

	// Normalize:
	for (size_t j=0;j<simulatedScan.scan.size();j++) simulatedScan.setScanRange(j, simulatedScan.getScanRange(j) / simulatedScan.maxRange);

	gl_scan2D->setScan( simulatedScan ); // Draw scaled scan in right-hand view

	// Navigate:
	mrpt::math::TPoint2D relTargetPose = mrpt::math::TPoint2D( mrpt::poses::CPoint2D(m_targetPoint) - mrpt::poses::CPose2D(m_robotPose) );
	relTargetPose*= 1.0/simulatedScan.maxRange;     // Normalized relative target:

	double desiredDirection,desiredSpeed;
	mrpt::nav::CHolonomicLogFileRecordPtr  out_log;

	//tictac.Tic();
	this->m_holonomicMethod->navigate(
		relTargetPose,
		simulatedScan.scan,
		m_simul_options.ROBOT_MAX_SPEED,
		desiredDirection,
		desiredSpeed,
		out_log,
		m_simul_options.MAX_SENSOR_RADIUS
		);
	// Tac

	// Move robot:
	m_robotPose.x += cos(desiredDirection) * desiredSpeed * time_step;
	m_robotPose.y += sin(desiredDirection) * desiredSpeed * time_step;

	// Update path graph:
	const TPoint3D  cur_pt(m_robotPose.x,m_robotPose.y,0.01);

	if (gl_robot_path->empty())
	     gl_robot_path->appendLine(cur_pt,cur_pt);
	else gl_robot_path->appendLineStrip(cur_pt);

	gl_rel_target->setLocation(relTargetPose.x, relTargetPose.y,0);

	// Clear stuff which will be updated if used below:
	edInfoLocalView->Clear();
	gl_nd_gaps->clear();

	// Update 2D view graphs:
	if (out_log && IS_CLASS(out_log, CLogFileRecord_ND))
	{
		CLogFileRecord_NDPtr log = CLogFileRecord_NDPtr(out_log);
		const size_t nGaps = log->gaps_ini.size();

		const string sSitu = mrpt::utils::TEnumType<CHolonomicND::TSituations>::value2name(log->situation);

		string sLog =
		      mrpt::format("ND situation : %s\n",sSitu.c_str());
		sLog+=mrpt::format("Gap count    : %u\n", static_cast<unsigned int>(nGaps) );

		edInfoLocalView->SetValue(_U(sLog.c_str()));

		gl_nd_gaps->appendLine(0,0,0, 0,0,0);

		for (size_t i=0;i<nGaps;i++)
		{
			const size_t N_STEPS = 20;
			for (size_t j=0;j<N_STEPS;j++)
			{
				const double sec = log->gaps_ini[i] + j*(log->gaps_end[i]-log->gaps_ini[i])/static_cast<double>(N_STEPS-1);
				const double ang = M_PI *( -1 + 2*sec/((float)simulatedScan.scan.size()) );
				const double d = simulatedScan.scan[sec]-0.05;
				gl_nd_gaps->appendLineStrip(d*cos(ang),d*sin(ang),0);
			}
			gl_nd_gaps->appendLineStrip(0,0,0);
		}
	}

	// Movement direction:
	const double d = desiredSpeed/m_simul_options.ROBOT_MAX_SPEED;
	gl_line_direction->setLineCoords(
		0,0,0,
		cos(desiredDirection) * d, sin(desiredDirection) * d, 0 );


}

void holonomic_navigator_demoFrame::updateViewsDynamicObjects()
{
	gl_robot->setLocation( m_robotPose.x, m_robotPose.y, 0);

	// animate target:
	{
		const double TARGET_BOUNCE_MIN = 0.7;
		const double TARGET_BOUNCE_MAX = 1;

		const double TARGET_BOUNCE_PERIOD = 1.0;
		const double t = fmod( m_runtime.Tac(), TARGET_BOUNCE_PERIOD ) / TARGET_BOUNCE_PERIOD;

		// Parabolic path
		const double s = 4*t*(TARGET_BOUNCE_MAX - TARGET_BOUNCE_MIN)*(1-t) + TARGET_BOUNCE_MIN;

		gl_target->setLocation( m_targetPoint.x, m_targetPoint.y, 0);
		gl_target->setScale(s);
	}

	// Labels:
	StatusBar1->SetStatusText( _U( mrpt::format("Robot: (%.03f,%.03f)", m_robotPose.x, m_robotPose.y ).c_str() ), 0 );
	StatusBar1->SetStatusText( _U( mrpt::format("Target: (%.03f,%.03f)", m_targetPoint.x, m_targetPoint.y ).c_str()  ), 1 );

	// Show/hide:
	gl_robot_sensor_range->setVisibility( mnuViewMaxRange->IsChecked() );
	gl_robot_path->setVisibility( mnuViewRobotPath->IsChecked() );

	// Refresh:
	m_plot3D->Refresh();
	m_plotScan->Refresh();
}

void holonomic_navigator_demoFrame::Onplot3DMouseMove(wxMouseEvent& event)
{
	int X, Y;
	event.GetPosition(&X,&Y);

	// Intersection of 3D ray with ground plane ====================
	TLine3D ray;
	m_plot3D->m_openGLScene->getViewport("main")->get3DRayForPixelCoord( X,Y,ray);
	// Create a 3D plane, e.g. Z=0
	const TPlane ground_plane(TPoint3D(0,0,0),TPoint3D(1,0,0),TPoint3D(0,1,0));
	// Intersection of the line with the plane:
	TObject3D inters;
	intersect(ray,ground_plane, inters);
	// Interpret the intersection as a point, if there is an intersection:
	TPoint3D inters_pt;
	if (inters.getPoint(inters_pt))
	{
		m_curCursorPos.x = inters_pt.x;
		m_curCursorPos.y = inters_pt.y;

		if (m_cursorPickState==cpsPickTarget)
		{
			m_gl_placing_nav_target->setVisibility(true);
			m_gl_placing_nav_target->setLocation(m_curCursorPos.x,m_curCursorPos.y,0.05);
		}
		else
		if (m_cursorPickState==cpsPlaceRobot)
		{
			m_gl_placing_robot->setVisibility(true);
			m_gl_placing_robot->setLocation(m_curCursorPos.x,m_curCursorPos.y,0.05);
		}

		StatusBar1->SetStatusText(wxString::Format(wxT("X=%.03f Y=%.04f Z=0"),m_curCursorPos.x,m_curCursorPos.y), 2);
	}

	// Do normal process in that class:
	m_plot3D->OnMouseMove(event);
}

void holonomic_navigator_demoFrame::Onplot3DMouseClick(wxMouseEvent& event)
{
	switch (m_cursorPickState)
	{
	case cpsPickTarget:
		{
			m_targetPoint = m_curCursorPos;

			btnPlaceTarget->SetValue(false);
			btnPlaceTarget->Refresh();
			m_gl_placing_nav_target->setVisibility(false);
			break;
		}
	case cpsPlaceRobot:
		{
			m_robotPose.x = m_curCursorPos.x;
			m_robotPose.y = m_curCursorPos.y;

			btnPlaceRobot->SetValue(false);
			btnPlaceRobot->Refresh();
			m_gl_placing_robot->setVisibility(false);
			break;
		}
	default:
		break;
	}

	m_plot3D->SetCursor( *wxSTANDARD_CURSOR ); // End of cross cursor
	m_cursorPickState = cpsNone; // end of mode

	// Do normal process in that class:
	m_plot3D->OnMouseDown(event);
}

// ==== holonomic_navigator_demoFrame::TOptions ======
holonomic_navigator_demoFrame::TOptions::TOptions() :
	ROBOT_MAX_SPEED   ( 4.0 ),
	MAX_SENSOR_RADIUS ( 5.0 ),
	SENSOR_NUM_RANGES ( 181),
	SENSOR_RANGE_NOISE_STD (0.02)
{
}
void holonomic_navigator_demoFrame::TOptions::loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section)
{
	MRPT_START

	// Load from config text:
	MRPT_LOAD_CONFIG_VAR(ROBOT_MAX_SPEED,double,  source,section );
	MRPT_LOAD_CONFIG_VAR(MAX_SENSOR_RADIUS,double,  source,section );
	MRPT_LOAD_CONFIG_VAR(SENSOR_NUM_RANGES, uint64_t,  source,section );
	MRPT_LOAD_CONFIG_VAR(SENSOR_RANGE_NOISE_STD,double,  source,section );

	MRPT_END
}

void holonomic_navigator_demoFrame::TOptions::saveToConfigFile(mrpt::utils::CConfigFileBase &cfg,const std::string &section) const
{
	MRPT_START
	const int WN = 40, WV = 20;

	cfg.write(section,"ROBOT_MAX_SPEED",ROBOT_MAX_SPEED,   WN,WV, "Maximum speed for the robot (m/s)");
	cfg.write(section,"MAX_SENSOR_RADIUS",MAX_SENSOR_RADIUS,   WN,WV, "Maximum range of the 360deg sensor (meters)");
	cfg.write(section,"SENSOR_NUM_RANGES",SENSOR_NUM_RANGES,   WN,WV, "Number of ranges in the 360deg sensor FOV");
	cfg.write(section,"SENSOR_RANGE_NOISE_STD",SENSOR_RANGE_NOISE_STD,   WN,WV, "Sensor noise (one sigma, in meters)");

	MRPT_END
}


void holonomic_navigator_demoFrame::OnMenuItemChangeVisibleStuff(wxCommandEvent& event)
{
	updateViewsDynamicObjects();
}

void holonomic_navigator_demoFrame::OnMenuItemClearRobotPath(wxCommandEvent& event)
{
	gl_robot_path->clear();
	updateViewsDynamicObjects();
}

void holonomic_navigator_demoFrame::OnbtnLoadMapClick(wxCommandEvent& event)
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
		f >> m_gridMap;
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
				if (!m_gridMap.loadFromBitmap(img,cell_size,cx,cy))
					wxMessageBox(_("Error"),_("Can't load the image file into the gridmap..."));
			}
			else
				wxMessageBox(_("Error"),_("Error parsing the numbers you entered..."));
		}
	}

	updateMap3DView();
	m_plot3D->Refresh();

	WX_END_TRY
}
