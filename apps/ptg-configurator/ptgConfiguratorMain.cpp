/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "ptgConfiguratorMain.h"
#include <wx/msgdlg.h>
#include "CAboutBox.h"

//(*InternalHeaders(ptgConfiguratorframe)
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/settings.h>
#include <wx/font.h>
#include <wx/intl.h>
#include <wx/image.h>
#include <wx/string.h>
//*)

#include <mrpt/gui/WxUtils.h>
#include "imgs/main_icon.xpm"
#include "../wx-common/mrpt_logo.xpm"

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

#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/utils/CConfigFileMemory.h>
#include <mrpt/utils/CConfigFilePrefixer.h>
#include <mrpt/math/geometry.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/CTicTac.h>

mrpt::nav::CParameterizedTrajectoryGenerator  * ptg = NULL;


//(*IdInit(ptgConfiguratorframe)
const long ptgConfiguratorframe::ID_STATICTEXT1 = wxNewId();
const long ptgConfiguratorframe::ID_CHOICE1 = wxNewId();
const long ptgConfiguratorframe::ID_STATICTEXT5 = wxNewId();
const long ptgConfiguratorframe::ID_STATICTEXT2 = wxNewId();
const long ptgConfiguratorframe::ID_SPINCTRL1 = wxNewId();
const long ptgConfiguratorframe::ID_BUTTON1 = wxNewId();
const long ptgConfiguratorframe::ID_CHECKBOX1 = wxNewId();
const long ptgConfiguratorframe::ID_STATICTEXT4 = wxNewId();
const long ptgConfiguratorframe::ID_TEXTCTRL5 = wxNewId();
const long ptgConfiguratorframe::ID_CHECKBOX3 = wxNewId();
const long ptgConfiguratorframe::ID_SLIDER1 = wxNewId();
const long ptgConfiguratorframe::ID_SPINCTRL2 = wxNewId();
const long ptgConfiguratorframe::ID_CHECKBOX2 = wxNewId();
const long ptgConfiguratorframe::ID_TEXTCTRL3 = wxNewId();
const long ptgConfiguratorframe::ID_STATICTEXT3 = wxNewId();
const long ptgConfiguratorframe::ID_TEXTCTRL4 = wxNewId();
const long ptgConfiguratorframe::ID_BUTTON3 = wxNewId();
const long ptgConfiguratorframe::ID_BUTTON2 = wxNewId();
const long ptgConfiguratorframe::ID_STATICTEXT6 = wxNewId();
const long ptgConfiguratorframe::ID_TEXTCTRL6 = wxNewId();
const long ptgConfiguratorframe::ID_STATICTEXT7 = wxNewId();
const long ptgConfiguratorframe::ID_TEXTCTRL7 = wxNewId();
const long ptgConfiguratorframe::ID_BUTTON4 = wxNewId();
const long ptgConfiguratorframe::ID_TEXTCTRL1 = wxNewId();
const long ptgConfiguratorframe::ID_PANEL1 = wxNewId();
const long ptgConfiguratorframe::ID_XY_GLCANVAS = wxNewId();
const long ptgConfiguratorframe::ID_TEXTCTRL2 = wxNewId();
const long ptgConfiguratorframe::idMenuQuit = wxNewId();
const long ptgConfiguratorframe::idMenuAbout = wxNewId();
const long ptgConfiguratorframe::ID_STATUSBAR1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(ptgConfiguratorframe,wxFrame)
    //(*EventTable(ptgConfiguratorframe)
    //*)
END_EVENT_TABLE()

ptgConfiguratorframe::ptgConfiguratorframe(wxWindow* parent,wxWindowID id) :
	m_cursorPickState (cpsNone )
{
    // Load my custom icons:
#if wxCHECK_VERSION(2, 8, 0)
    wxArtProvider::Push(new MyArtProvider);
#else
    wxArtProvider::PushProvider(new MyArtProvider);
#endif


    //(*Initialize(ptgConfiguratorframe)
    wxFlexGridSizer* FlexGridSizer4;
    wxMenuItem* MenuItem2;
    wxFlexGridSizer* FlexGridSizer3;
    wxMenuItem* MenuItem1;
    wxFlexGridSizer* FlexGridSizer5;
    wxFlexGridSizer* FlexGridSizer2;
    wxMenu* Menu1;
    wxFlexGridSizer* FlexGridSizer7;
    wxFlexGridSizer* FlexGridSizer8;
    wxMenuBar* MenuBar1;
    wxFlexGridSizer* FlexGridSizer6;
    wxFlexGridSizer* FlexGridSizer1;
    wxMenu* Menu2;

    Create(parent, id, _("PTG configurator - Part of the MRPT project"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("id"));
    SetClientSize(wxSize(893,576));
    {
    	wxIcon FrameIcon;
    	FrameIcon.CopyFromBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("MAIN_ICON")),wxART_OTHER));
    	SetIcon(FrameIcon);
    }
    FlexGridSizer1 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer1->AddGrowableCol(0);
    FlexGridSizer1->AddGrowableRow(1);
    Panel1 = new wxPanel(this, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    FlexGridSizer2 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer2->AddGrowableCol(0);
    FlexGridSizer2->AddGrowableRow(0);
    FlexGridSizer3 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer3->AddGrowableCol(0);
    FlexGridSizer3->AddGrowableRow(3);
    FlexGridSizer7 = new wxFlexGridSizer(1, 0, 0, 0);
    FlexGridSizer7->AddGrowableCol(1);
    StaticText1 = new wxStaticText(Panel1, ID_STATICTEXT1, _("Select a PTG class:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    FlexGridSizer7->Add(StaticText1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbPTGClass = new wxChoice(Panel1, ID_CHOICE1, wxDefaultPosition, wxDefaultSize, 0, 0, wxCB_SORT, wxDefaultValidator, _T("ID_CHOICE1"));
    FlexGridSizer7->Add(cbPTGClass, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText5 = new wxStaticText(Panel1, ID_STATICTEXT5, _("After selecting a PTG, clhange params. as desired, then click `Initialize PTG`"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT5"));
    FlexGridSizer7->Add(StaticText5, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer3->Add(FlexGridSizer7, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer4 = new wxFlexGridSizer(0, 6, 0, 0);
    StaticText2 = new wxStaticText(Panel1, ID_STATICTEXT2, _("PTG index for cfg file:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    FlexGridSizer4->Add(StaticText2, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edPTGIndex = new wxSpinCtrl(Panel1, ID_SPINCTRL1, _T("0"), wxDefaultPosition, wxDefaultSize, 0, 0, 100, 0, _T("ID_SPINCTRL1"));
    edPTGIndex->SetValue(_T("0"));
    FlexGridSizer4->Add(edPTGIndex, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnReloadParams = new wxButton(Panel1, ID_BUTTON1, _("Initialize PTG"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
    wxFont btnReloadParamsFont(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxBOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    btnReloadParams->SetFont(btnReloadParamsFont);
    FlexGridSizer4->Add(btnReloadParams, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbDrawShapePath = new wxCheckBox(Panel1, ID_CHECKBOX1, _("Draw robot shape"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
    cbDrawShapePath->SetValue(true);
    FlexGridSizer4->Add(cbDrawShapePath, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText4 = new wxStaticText(Panel1, ID_STATICTEXT4, _("Dist. btw. robot shapes:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT4"));
    FlexGridSizer4->Add(StaticText4, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edMinDistBtwShapes = new wxTextCtrl(Panel1, ID_TEXTCTRL5, _("0.5"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL5"));
    FlexGridSizer4->Add(edMinDistBtwShapes, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbHighlightOnePath = new wxCheckBox(Panel1, ID_CHECKBOX3, _("Highlight one trajectory:"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX3"));
    cbHighlightOnePath->SetValue(false);
    FlexGridSizer4->Add(cbHighlightOnePath, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    slidPathHighlight = new wxSlider(Panel1, ID_SLIDER1, 0, 0, 100, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_SLIDER1"));
    slidPathHighlight->Disable();
    FlexGridSizer4->Add(slidPathHighlight, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edIndexHighlightPath = new wxSpinCtrl(Panel1, ID_SPINCTRL2, _T("0"), wxDefaultPosition, wxDefaultSize, 0, 0, 100, 0, _T("ID_SPINCTRL2"));
    edIndexHighlightPath->SetValue(_T("0"));
    edIndexHighlightPath->Disable();
    FlexGridSizer4->Add(edIndexHighlightPath, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer3->Add(FlexGridSizer4, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer8 = new wxFlexGridSizer(0, 6, 0, 0);
    cbBuildTPObs = new wxCheckBox(Panel1, ID_CHECKBOX2, _("Build TP-Obstacles for obstacle point: x="), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX2"));
    cbBuildTPObs->SetValue(true);
    FlexGridSizer8->Add(cbBuildTPObs, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edObsX = new wxTextCtrl(Panel1, ID_TEXTCTRL3, _("9.0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL3"));
    FlexGridSizer8->Add(edObsX, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText3 = new wxStaticText(Panel1, ID_STATICTEXT3, _("y="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
    FlexGridSizer8->Add(StaticText3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edObsY = new wxTextCtrl(Panel1, ID_TEXTCTRL4, _("3.0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL4"));
    FlexGridSizer8->Add(edObsY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnPlaceObs = new wxButton(Panel1, ID_BUTTON3, _("Click to place obstacle..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON3"));
    FlexGridSizer8->Add(btnPlaceObs, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnRebuildTPObs = new wxButton(Panel1, ID_BUTTON2, _("Rebuild TP-Obs."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
    FlexGridSizer8->Add(btnRebuildTPObs, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText6 = new wxStaticText(Panel1, ID_STATICTEXT6, _("Target point:  x="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT6"));
    FlexGridSizer8->Add(StaticText6, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edTargetX = new wxTextCtrl(Panel1, ID_TEXTCTRL6, _("5.0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL6"));
    FlexGridSizer8->Add(edTargetX, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText7 = new wxStaticText(Panel1, ID_STATICTEXT7, _("y="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT7"));
    FlexGridSizer8->Add(StaticText7, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edTargetY = new wxTextCtrl(Panel1, ID_TEXTCTRL7, _("1.0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL7"));
    FlexGridSizer8->Add(edTargetY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnPlaceTarget = new wxButton(Panel1, ID_BUTTON4, _("Click to place target..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON4"));
    FlexGridSizer8->Add(btnPlaceTarget, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer3->Add(FlexGridSizer8, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    edCfg = new wxTextCtrl(Panel1, ID_TEXTCTRL1, wxEmptyString, wxDefaultPosition, wxSize(-1,150), wxTE_PROCESS_ENTER|wxTE_PROCESS_TAB|wxTE_MULTILINE|wxHSCROLL|wxTE_DONTWRAP|wxALWAYS_SHOW_SB, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    wxFont edCfgFont = wxSystemSettings::GetFont(wxSYS_OEM_FIXED_FONT);
    if ( !edCfgFont.Ok() ) edCfgFont = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
    edCfgFont.SetPointSize(8);
    edCfgFont.SetFamily(wxTELETYPE);
    edCfg->SetFont(edCfgFont);
    FlexGridSizer3->Add(edCfg, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 2);
    FlexGridSizer5 = new wxFlexGridSizer(0, 3, 0, 0);
    FlexGridSizer3->Add(FlexGridSizer5, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer2->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer6 = new wxFlexGridSizer(0, 3, 0, 0);
    FlexGridSizer2->Add(FlexGridSizer6, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    Panel1->SetSizer(FlexGridSizer2);
    FlexGridSizer2->Fit(Panel1);
    FlexGridSizer2->SetSizeHints(Panel1);
    FlexGridSizer1->Add(Panel1, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    m_plot = new CMyGLCanvas(this,ID_XY_GLCANVAS,wxDefaultPosition,wxSize(600,450),wxTAB_TRAVERSAL,_T("ID_XY_GLCANVAS"));
    FlexGridSizer1->Add(m_plot, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    edLog = new wxTextCtrl(this, ID_TEXTCTRL2, wxEmptyString, wxDefaultPosition, wxSize(-1,100), wxTE_PROCESS_ENTER|wxTE_PROCESS_TAB|wxTE_MULTILINE|wxTE_READONLY|wxHSCROLL|wxTE_DONTWRAP|wxALWAYS_SHOW_SB, wxDefaultValidator, _T("ID_TEXTCTRL2"));
    wxFont edLogFont = wxSystemSettings::GetFont(wxSYS_OEM_FIXED_FONT);
    if ( !edLogFont.Ok() ) edLogFont = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
    edLogFont.SetPointSize(8);
    edLogFont.SetFamily(wxTELETYPE);
    edLog->SetFont(edLogFont);
    FlexGridSizer1->Add(edLog, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    SetSizer(FlexGridSizer1);
    MenuBar1 = new wxMenuBar();
    Menu1 = new wxMenu();
    MenuItem1 = new wxMenuItem(Menu1, idMenuQuit, _("Quit\tAlt-F4"), _("Quit the application"), wxITEM_NORMAL);
    Menu1->Append(MenuItem1);
    MenuBar1->Append(Menu1, _("&File"));
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
    FlexGridSizer1->SetSizeHints(this);
    Center();

    Connect(ID_CHOICE1,wxEVT_COMMAND_CHOICE_SELECTED,(wxObjectEventFunction)&ptgConfiguratorframe::OncbPTGClassSelect);
    Connect(ID_SPINCTRL1,wxEVT_COMMAND_SPINCTRL_UPDATED,(wxObjectEventFunction)&ptgConfiguratorframe::OnedPTGIndexChange);
    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&ptgConfiguratorframe::OnbtnReloadParamsClick);
    Connect(ID_CHECKBOX1,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&ptgConfiguratorframe::OncbDrawShapePathClick);
    Connect(ID_CHECKBOX3,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&ptgConfiguratorframe::OncbHighlightOnePathClick);
    Connect(ID_SLIDER1,wxEVT_SCROLL_TOP|wxEVT_SCROLL_BOTTOM|wxEVT_SCROLL_LINEUP|wxEVT_SCROLL_LINEDOWN|wxEVT_SCROLL_PAGEUP|wxEVT_SCROLL_PAGEDOWN|wxEVT_SCROLL_THUMBTRACK|wxEVT_SCROLL_THUMBRELEASE|wxEVT_SCROLL_CHANGED,(wxObjectEventFunction)&ptgConfiguratorframe::OnslidPathHighlightCmdScroll);
    Connect(ID_SPINCTRL2,wxEVT_COMMAND_SPINCTRL_UPDATED,(wxObjectEventFunction)&ptgConfiguratorframe::OnedIndexHighlightPathChange);
    Connect(ID_CHECKBOX2,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&ptgConfiguratorframe::OncbBuildTPObsClick);
    Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&ptgConfiguratorframe::OnbtnPlaceObsClick);
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&ptgConfiguratorframe::OnbtnRebuildTPObsClick);
    Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&ptgConfiguratorframe::OnbtnPlaceTargetClick);
    Connect(idMenuQuit,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&ptgConfiguratorframe::OnQuit);
    Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&ptgConfiguratorframe::OnAbout);
    //*)

	m_plot->Connect(wxEVT_MOTION,(wxObjectEventFunction)&ptgConfiguratorframe::Onplot3DMouseMove,0,this);
	m_plot->Connect(wxEVT_LEFT_DOWN,(wxObjectEventFunction)&ptgConfiguratorframe::Onplot3DMouseClick,0,this);

	// Redirect all output to control:
	m_myRedirector = new CMyRedirector( edLog, true, 100, true);

	WX_START_TRY


	// Populate 3D views:
	// Split in 2 views:
	gl_view_WS      = m_plot->m_openGLScene->createViewport("main");
	gl_view_WS->setBorderSize(1);

	gl_view_TPSpace  = m_plot->m_openGLScene->createViewport("TPSpace");
	gl_view_TPSpace->setBorderSize(1);

	m_plot->addTextMessage(0.01,5,"Workspace", mrpt::utils::TColorf(1,1,1,0.75), "sans", 15,mrpt::opengl::NICE, 1);
	m_plot->addTextMessage(0.51,5,"TP-Space", mrpt::utils::TColorf(1,1,1,0.75), "sans", 15,mrpt::opengl::NICE, 2);

	gl_view_WS->setViewportPosition(0,0,0.5,1.0);
	gl_view_TPSpace->setViewportPosition(0.5,0.0,0.5,1.0);

	gl_robot_ptg_prediction = mrpt::opengl::CSetOfLines::Create();
	gl_robot_ptg_prediction->setName("ptg_prediction");
	gl_robot_ptg_prediction->setLineWidth(1.0);
	gl_robot_ptg_prediction->setColor_u8( mrpt::utils::TColor(0x00,0x00,0xff,0x90) );
	gl_view_WS->insert(gl_robot_ptg_prediction);

	gl_WS_obs = mrpt::opengl::CPointCloud::Create();
	gl_WS_obs->setPointSize(7.0);
	gl_WS_obs->setColor_u8(0,0,0);
	gl_view_WS->insert(gl_WS_obs);

	gl_WS_target = mrpt::opengl::CPointCloud::Create();
	gl_WS_target->setPointSize(7.0);
	gl_WS_target->setColor_u8(0xff,0,0);
	gl_WS_target->insertPoint(0,0,0);
	gl_WS_target->setName("WS-target");
	gl_WS_target->enableShowName(true);
	gl_view_WS->insert(gl_WS_target);

	gl_TP_target = mrpt::opengl::CPointCloud::Create();
	gl_TP_target->setPointSize(7.0);
	gl_TP_target->setColor_u8(0xff,0,0);
	gl_TP_target->insertPoint(0,0,0);
	gl_TP_target->setName("TP-target");
	gl_TP_target->enableShowName(true);
	gl_view_TPSpace->insert(gl_TP_target);


	{
		gl_axis_WS = mrpt::opengl::CAxis::Create(-10.0,-10.0,0,10.0,10.0,0.0, 1.0 ,2.0);
		gl_axis_WS->setTextScale(0.20);
		gl_axis_WS->enableTickMarks(true,true,true);
		gl_axis_WS->setColor_u8(mrpt::utils::TColor(30,30,30,50));
		gl_axis_WS->setTextLabelOrientation(0,0,0,0);
		gl_axis_WS->setTextLabelOrientation(1,0,0,0);

		gl_view_WS->insert( gl_axis_WS );
	}
	{
		gl_axis_TPS= mrpt::opengl::CAxis::Create(-1.0,-1.0,0,1.0,1.0,0.0, 0.25 ,2.0);
		gl_axis_TPS->setTextScale(0.04);
		gl_axis_TPS->enableTickMarks(true,true,false);
		gl_axis_TPS->setColor_u8(mrpt::utils::TColor(30,30,30,50));
		gl_axis_TPS->setTextLabelOrientation(0,0,0,0);
		gl_axis_TPS->setTextLabelOrientation(1,0,0,0);
		gl_view_TPSpace->insert( gl_axis_TPS );
	}

	gl_tp_obstacles = mrpt::opengl::CSetOfLines::Create();
	gl_tp_obstacles->setName("tp_obstacles");
	gl_tp_obstacles->setLineWidth(2.0);
	gl_tp_obstacles->setColor_u8( mrpt::utils::TColor(0x00,0x00,0x00,0xff) );
	gl_view_TPSpace->insert(gl_tp_obstacles);

	// Set camera:
	m_plot->cameraPointingX=0;
	m_plot->cameraPointingY=0;
	m_plot->cameraPointingZ=0;
	m_plot->cameraZoomDistance = 10;
	m_plot->cameraElevationDeg = 90;
	m_plot->cameraAzimuthDeg = -90;
	m_plot->cameraIsProjective = false;

	gl_view_TPSpace_cam = mrpt::opengl::CCamera::Create();
	gl_view_TPSpace->insert ( gl_view_TPSpace_cam );
	gl_view_TPSpace_cam->setAzimuthDegrees( -90 );
	gl_view_TPSpace_cam->setElevationDegrees(90);
	gl_view_TPSpace_cam->setProjectiveModel( false );
	gl_view_TPSpace_cam->setZoomDistance(2.5);


	// Populate list of existing PTGs:
	{
		mrpt::nav::registerAllNavigationClasses();
		const std::vector<const mrpt::utils::TRuntimeClassId*> &lstClasses = mrpt::utils::getAllRegisteredClasses();
		for (size_t i=0;i<lstClasses.size();i++)
		{
			if (!lstClasses[i]->derivedFrom( "CParameterizedTrajectoryGenerator") ||
				!mrpt::system::os::_strcmpi(lstClasses[i]->className,"CParameterizedTrajectoryGenerator") )
				continue;
			cbPTGClass->AppendString( _U( lstClasses[i]->className ));
		}
		if (cbPTGClass->GetCount()>0)
			cbPTGClass->SetSelection(0);
	}
	{
		wxCommandEvent e;
		OncbPTGClassSelect(e);
	}

	this->Maximize();

	WX_END_TRY
}

ptgConfiguratorframe::~ptgConfiguratorframe()
{
    //(*Destroy(ptgConfiguratorframe)
    //*)
	if (ptg) {
		delete ptg;
		ptg=NULL;
	}
}


void ptgConfiguratorframe::OnAbout(wxCommandEvent& event)
{
}

void ptgConfiguratorframe::OnQuit(wxCommandEvent& event)
{
	Close();
}

void ptgConfiguratorframe::OnbtnReloadParamsClick(wxCommandEvent& event)
{
	WX_START_TRY;
	if (!ptg) return;

	ptg->deinitialize();

	const std::string sKeyPrefix = mrpt::format("PTG%d_", (int)edPTGIndex->GetValue() );
	const std::string sSection = "PTG_PARAMS";

	mrpt::utils::CConfigFileMemory  cfg;
	mrpt::utils::CConfigFilePrefixer cfp;
	cfp.bind(cfg);
	cfp.setPrefixes("", sKeyPrefix );

	cfg.setContent( std::string( edCfg->GetValue().mb_str() ) );

	ptg->loadFromConfigFile(cfp,sSection);

	ptg->initialize();

	rebuild3Dview();

	WX_END_TRY;
}


void ptgConfiguratorframe::OncbPTGClassSelect(wxCommandEvent& event)
{
	WX_START_TRY;

	const int sel = cbPTGClass->GetSelection();
	if (sel<0) return;

	const std::string sSelPTG = std::string( cbPTGClass->GetString(sel).mb_str() );

	if (ptg) {
		delete ptg;
	}

	// Factory:
	const mrpt::utils::TRuntimeClassId *classId = mrpt::utils::findRegisteredClass( sSelPTG );
	if (!classId) {
		THROW_EXCEPTION_CUSTOM_MSG1("[CreatePTG] No PTG named `%s` is registered!",sSelPTG.c_str());
	}

	ptg = dynamic_cast<mrpt::nav::CParameterizedTrajectoryGenerator*>( classId->createObject() );
	if (!ptg) {
		THROW_EXCEPTION_CUSTOM_MSG1("[CreatePTG] Object of type `%s` seems not to be a PTG!",sSelPTG.c_str());
	}

	// Set some common defaults:
	ptg->loadDefaultParams();

	dumpPTGcfgToTextBox();

	WX_END_TRY;

	// Update graphs:
	rebuild3Dview();
}

void ptgConfiguratorframe::rebuild3Dview()
{
	WX_START_TRY;
	static mrpt::utils::CTicTac timer;
	const double refDist = ptg ? ptg->getRefDistance() : 10.0;
	ASSERT_(refDist>0);

	// Limits:
	gl_axis_WS->setAxisLimits(-refDist,-refDist,.0f, refDist,refDist,.0f);

	if (ptg && ptg->isInitialized())
	{
		// TP-Obstacles:
		std::vector<double> TP_Obstacles;
		ptg->initTPObstacles(TP_Obstacles);

		gl_WS_obs->clear();
		if (cbBuildTPObs->IsChecked())
		{
			double ox,oy;
			bool ok_x = edObsX->GetValue().ToDouble(&ox);
			bool ok_y = edObsY->GetValue().ToDouble(&oy);
			if (ok_x && ok_y)
			{
				gl_WS_obs->insertPoint(ox,oy,0);
				timer.Tic();
				ptg->updateTPObstacle(ox,oy, TP_Obstacles);
				const double t =timer.Tac();
				StatusBar1->SetStatusText(wxString::Format(wxT("TP-Obstacle build time: %.06f ms"),t*1e3), 2);
			}
		}

		// All paths:
		gl_robot_ptg_prediction->clear();
		gl_robot_ptg_prediction->clear();
		for (int k=0;k<ptg->getPathCount();k++)
		{
			const double max_dist = TP_Obstacles[k];
			ptg->renderPathAsSimpleLine(k,*gl_robot_ptg_prediction,0.10, max_dist);

			// Overlay a sequence of robot shapes:
			if (cbDrawShapePath->IsChecked())
			{
				double min_shape_dists = 1.0;
				edMinDistBtwShapes->GetValue().ToDouble(&min_shape_dists);
				bool done=false;
				for (double d=max_dist; !done ;d-=min_shape_dists)
				{
					if (d<0) { d=0; done=true; }
					uint16_t step;
					if (!ptg->getPathStepForDist(k, d, step))
						continue;
					mrpt::math::TPose2D p;
					ptg->getPathPose(k, step, p);
					ptg->add_robotShape_to_setOfLines(*gl_robot_ptg_prediction, mrpt::poses::CPose2D(p) );
				}
			}
		}

		// TP-Obstacles:
		gl_tp_obstacles->clear();
		const size_t nObs = TP_Obstacles.size();
		if (nObs>1)
		{
			for (size_t i=0;i<=nObs;i++)
			{
				const double d0 = TP_Obstacles[i % nObs] / refDist;
				const double a0 = M_PI * (-1.0 + 2.0 * ((i % nObs)+0.5)/nObs );
				const double d1 = TP_Obstacles[(i+1) % nObs] / refDist;
				const double a1 = M_PI * (-1.0 + 2.0 * (((i+1) % nObs)+0.5)/nObs );
				gl_tp_obstacles->appendLine(
					d0*cos(a0),d0*sin(a0),0.0,
					d1*cos(a1),d1*sin(a1),0.0 );
			}
		}
	} // if ptg init 

	// Target:
	{
		// WS:
		double tx,ty;
		bool ok_x = edTargetX->GetValue().ToDouble(&tx);
		bool ok_y = edTargetY->GetValue().ToDouble(&ty);
		if (ok_x && ok_y)
		{
			gl_WS_target->setLocation(tx,ty,0);
		}
		// TPS:
		if (ptg && ptg->isInitialized())
		{
			int k;
			double norm_d;
			bool is_exact = ptg->inverseMap_WS2TP(tx,ty,k,norm_d);
			const double dir = ptg->index2alpha(k);
			if (is_exact)
			{
				gl_TP_target->setLocation( cos(dir)*norm_d,sin(dir)*norm_d, .0 );
				gl_TP_target->setName("TP-target");
			}
			else {
				gl_TP_target->setName("TP-target (NOT EXACT)");
			}
			StatusBar1->SetStatusText(wxString::Format(
				wxT("TP-Target: k=%i (alpha=%.03f deg) norm_d=%.03f is_exact:%s"),
				k,dir*180/M_PI,norm_d,(is_exact ? "yes":"NO")), 
				1);
		}
	}


	m_plot->Refresh();
	WX_END_TRY;
}

void ptgConfiguratorframe::OnedPTGIndexChange(wxSpinEvent& event)
{
	dumpPTGcfgToTextBox();
}

void ptgConfiguratorframe::dumpPTGcfgToTextBox()
{
	if (!ptg) return;

	// Wrapper to transparently add prefixes to all config keys:
	const std::string sKeyPrefix = mrpt::format("PTG%d_", (int)edPTGIndex->GetValue() );
	const std::string sSection = "PTG_PARAMS";

	mrpt::utils::CConfigFileMemory  cfg;
	mrpt::utils::CConfigFilePrefixer cfp;
	cfp.bind(cfg);
	cfp.setPrefixes("", sKeyPrefix );

	const int WN = 40, WV = 20;
	cfp.write(sSection,"Type", ptg->GetRuntimeClass()->className,   WN,WV, "PTG C++ class name");

	// Dump default params:
	ptg->saveToConfigFile(cfp,sSection);

	edCfg->SetValue( _U( cfg.getContent().c_str() ));
}

void ptgConfiguratorframe::OncbDrawShapePathClick(wxCommandEvent& event)
{
	rebuild3Dview();
}

void ptgConfiguratorframe::OncbBuildTPObsClick(wxCommandEvent& event)
{
	rebuild3Dview();
}

void ptgConfiguratorframe::OnbtnRebuildTPObsClick(wxCommandEvent& event)
{
	rebuild3Dview();
}

void ptgConfiguratorframe::Onplot3DMouseMove(wxMouseEvent& event)
{
	using namespace mrpt::math;

	int X, Y;
	event.GetPosition(&X,&Y);

	// Intersection of 3D ray with ground plane ====================
	TLine3D ray;
	m_plot->m_openGLScene->getViewport("main")->get3DRayForPixelCoord( X,Y,ray);
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

		switch (m_cursorPickState)
		{
		case cpsPickObstacle:
			{
				edObsX->SetValue( _U( mrpt::format("%.03f",m_curCursorPos.x).c_str() ) );
				edObsY->SetValue( _U( mrpt::format("%.03f",m_curCursorPos.y).c_str() ) );
				rebuild3Dview();
			}
			break;
		case cpsPickTarget:
			{
				edTargetX->SetValue( _U( mrpt::format("%.03f",m_curCursorPos.x).c_str() ) );
				edTargetY->SetValue( _U( mrpt::format("%.03f",m_curCursorPos.y).c_str() ) );
				rebuild3Dview();
			}
			break;
		};
		StatusBar1->SetStatusText(wxString::Format(wxT("Cursor: X=%.03f Y=%.04f"),m_curCursorPos.x,m_curCursorPos.y), 0);
	}

	// Do normal process in that class:
	m_plot->OnMouseMove(event);
}

void ptgConfiguratorframe::Onplot3DMouseClick(wxMouseEvent& event)
{
	m_plot->SetCursor( *wxSTANDARD_CURSOR ); // End of cross cursor
	m_cursorPickState = cpsNone; // end of mode

	// Do normal process in that class:
	m_plot->OnMouseDown(event);
}

void ptgConfiguratorframe::OnbtnPlaceObsClick(wxCommandEvent& event)
{
	m_plot->SetCursor( *wxCROSS_CURSOR );
	m_cursorPickState = cpsPickObstacle;
}

void ptgConfiguratorframe::OnbtnPlaceTargetClick(wxCommandEvent& event)
{
	m_plot->SetCursor( *wxCROSS_CURSOR );
	m_cursorPickState = cpsPickTarget;
}

void ptgConfiguratorframe::OnslidPathHighlightCmdScroll(wxScrollEvent& event)
{
}

void ptgConfiguratorframe::OncbHighlightOnePathClick(wxCommandEvent& event)
{
	slidPathHighlight->Enable( cbHighlightOnePath->IsChecked() );
	edIndexHighlightPath->Enable( cbHighlightOnePath->IsChecked() );
	rebuild3Dview();
}

void ptgConfiguratorframe::OnedIndexHighlightPathChange(wxSpinEvent& event)
{
}
