/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "reactive_navigator_demoMain.h"
#include <wx/msgdlg.h>
#include <wx/textdlg.h>
#include "CAboutBox.h"

//(*InternalHeaders(reactive_navigator_demoframe)
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
#include <mrpt/utils/CFileGZOutputStream.h>
#include "imgs/main_icon.xpm"
#include "../wx-common/mrpt_logo.xpm"

#include "DEFAULT_GRIDMAP_DATA.h"

const double NAV_SIMUL_TIMESTEP_MS = 25;

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
#include <mrpt/system/os.h>

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


//(*IdInit(reactive_navigator_demoframe)
const long reactive_navigator_demoframe::ID_BUTTON4 = wxNewId();
const long reactive_navigator_demoframe::ID_BUTTON5 = wxNewId();
const long reactive_navigator_demoframe::ID_BUTTON7 = wxNewId();
const long reactive_navigator_demoframe::ID_BUTTON12 = wxNewId();
const long reactive_navigator_demoframe::ID_BUTTON6 = wxNewId();
const long reactive_navigator_demoframe::ID_BUTTON1 = wxNewId();
const long reactive_navigator_demoframe::ID_BUTTON9 = wxNewId();
const long reactive_navigator_demoframe::ID_BUTTON8 = wxNewId();
const long reactive_navigator_demoframe::ID_BUTTON11 = wxNewId();
const long reactive_navigator_demoframe::ID_BUTTON10 = wxNewId();
const long reactive_navigator_demoframe::ID_BUTTON2 = wxNewId();
const long reactive_navigator_demoframe::ID_BUTTON3 = wxNewId();
const long reactive_navigator_demoframe::ID_RADIOBOX2 = wxNewId();
const long reactive_navigator_demoframe::ID_CHECKBOX1 = wxNewId();
const long reactive_navigator_demoframe::ID_CHECKBOX2 = wxNewId();
const long reactive_navigator_demoframe::ID_CHECKBOX3 = wxNewId();
const long reactive_navigator_demoframe::ID_CHECKBOX4 = wxNewId();
const long reactive_navigator_demoframe::ID_CHECKBOX5 = wxNewId();
const long reactive_navigator_demoframe::ID_RADIOBOX1 = wxNewId();
const long reactive_navigator_demoframe::ID_PANEL6 = wxNewId();
const long reactive_navigator_demoframe::ID_TEXTCTRL1 = wxNewId();
const long reactive_navigator_demoframe::ID_PANEL2 = wxNewId();
const long reactive_navigator_demoframe::ID_TEXTCTRL4 = wxNewId();
const long reactive_navigator_demoframe::ID_PANEL4 = wxNewId();
const long reactive_navigator_demoframe::ID_TEXTCTRL3 = wxNewId();
const long reactive_navigator_demoframe::ID_PANEL3 = wxNewId();
const long reactive_navigator_demoframe::ID_NOTEBOOK1 = wxNewId();
const long reactive_navigator_demoframe::ID_PANEL1 = wxNewId();
const long reactive_navigator_demoframe::ID_STATICTEXT3 = wxNewId();
const long reactive_navigator_demoframe::ID_TEXTCTRL5 = wxNewId();
const long reactive_navigator_demoframe::ID_PANEL7 = wxNewId();
const long reactive_navigator_demoframe::ID_STATICTEXT2 = wxNewId();
const long reactive_navigator_demoframe::ID_STATICTEXT1 = wxNewId();
const long reactive_navigator_demoframe::ID_XY_GLCANVAS = wxNewId();
const long reactive_navigator_demoframe::ID_STATICTEXT4 = wxNewId();
const long reactive_navigator_demoframe::ID_CHOICE1 = wxNewId();
const long reactive_navigator_demoframe::ID_CUSTOM1 = wxNewId();
const long reactive_navigator_demoframe::ID_TEXTCTRL2 = wxNewId();
const long reactive_navigator_demoframe::ID_PANEL8 = wxNewId();
const long reactive_navigator_demoframe::ID_SPLITTERWINDOW2 = wxNewId();
const long reactive_navigator_demoframe::ID_PANEL5 = wxNewId();
const long reactive_navigator_demoframe::ID_SPLITTERWINDOW1 = wxNewId();
const long reactive_navigator_demoframe::ID_MENUITEM4 = wxNewId();
const long reactive_navigator_demoframe::idMenuQuit = wxNewId();
const long reactive_navigator_demoframe::ID_MENUITEM1 = wxNewId();
const long reactive_navigator_demoframe::ID_MENUITEM2 = wxNewId();
const long reactive_navigator_demoframe::ID_MENUITEM3 = wxNewId();
const long reactive_navigator_demoframe::idMenuAbout = wxNewId();
const long reactive_navigator_demoframe::ID_STATUSBAR1 = wxNewId();
const long reactive_navigator_demoframe::ID_TIMER1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(reactive_navigator_demoframe,wxFrame)
    //(*EventTable(reactive_navigator_demoframe)
    //*)
END_EVENT_TABLE()



reactive_navigator_demoframe::reactive_navigator_demoframe(wxWindow* parent,wxWindowID id) :
	m_gridMap(),
	m_targetPoint(.0,.0),
	m_is_running(false),
	m_curCursorPos(0,0),
	m_cursorPickState(cpsNone)
{
    // Load my custom icons:
#if wxCHECK_VERSION(2, 8, 0)
    wxArtProvider::Push(new MyArtProvider);
#else
    wxArtProvider::PushProvider(new MyArtProvider);
#endif


    //(*Initialize(reactive_navigator_demoframe)
    wxFlexGridSizer* FlexGridSizer4;
    wxMenuItem* MenuItem2;
    wxFlexGridSizer* FlexGridSizer10;
    wxFlexGridSizer* FlexGridSizer3;
    wxMenuItem* MenuItem1;
    wxFlexGridSizer* FlexGridSizer5;
    wxFlexGridSizer* FlexGridSizer9;
    wxFlexGridSizer* FlexGridSizer2;
    wxMenu* Menu1;
    wxFlexGridSizer* FlexGridSizer7;
    wxFlexGridSizer* FlexGridSizer8;
    wxMenuBar* MenuBar1;
    wxFlexGridSizer* FlexGridSizer6;
    wxFlexGridSizer* FlexGridSizer1;
    wxFlexGridSizer* FlexGridSizer11;
    wxBoxSizer* BoxSizer3;
    wxMenu* Menu2;
    
    Create(parent, wxID_ANY, _("Reactive Navigation Tester - Part of MRPT"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("wxID_ANY"));
    SetClientSize(wxSize(893,576));
    {
    	wxIcon FrameIcon;
    	FrameIcon.CopyFromBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("MAIN_ICON")),wxART_OTHER));
    	SetIcon(FrameIcon);
    }
    FlexGridSizer1 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer1->AddGrowableCol(1);
    FlexGridSizer1->AddGrowableRow(0);
    FlexGridSizer4 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer4->AddGrowableCol(0);
    btnStart = new wxCustomButton(this,ID_BUTTON4,_("RUN"),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_GO_FORWARD")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxSize(-1,60),wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON4"));
    btnStart->SetBitmapDisabled(btnStart->CreateBitmapDisabled(btnStart->GetBitmapLabel()));
    btnStart->SetBitmapMargin(wxSize(2,4));
    FlexGridSizer4->Add(btnStart, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    btnStop = new wxCustomButton(this,ID_BUTTON5,_("STOP"),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_CROSS_MARK")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxSize(-1,60),wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON5"));
    btnStop->SetBitmapDisabled(btnStop->CreateBitmapDisabled(btnStop->GetBitmapLabel()));
    btnStop->SetBitmapMargin(wxSize(2,4));
    FlexGridSizer4->Add(btnStop, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    FlexGridSizer4->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
    btnPlaceTarget = new wxCustomButton(this,ID_BUTTON7,_("Set target..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FIND")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxSize(-1,60),wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON7"));
    btnPlaceTarget->SetBitmapDisabled(btnPlaceTarget->CreateBitmapDisabled(btnPlaceTarget->GetBitmapLabel()));
    btnPlaceTarget->SetBitmapMargin(wxSize(2,4));
    FlexGridSizer4->Add(btnPlaceTarget, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    btnSetWaypointSeq = new wxCustomButton(this,ID_BUTTON12,_("Waypoint list..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FIND")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxSize(-1,60),wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON12"));
    btnSetWaypointSeq->SetBitmapDisabled(btnSetWaypointSeq->CreateBitmapDisabled(btnSetWaypointSeq->GetBitmapLabel()));
    btnSetWaypointSeq->SetBitmapMargin(wxSize(2,4));
    FlexGridSizer4->Add(btnSetWaypointSeq, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    btnPlaceRobot = new wxCustomButton(this,ID_BUTTON6,_("Replace robot..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FIND")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxSize(-1,60),wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON6"));
    btnPlaceRobot->SetBitmapDisabled(btnPlaceRobot->CreateBitmapDisabled(btnPlaceRobot->GetBitmapLabel()));
    btnPlaceRobot->SetBitmapMargin(wxSize(2,4));
    FlexGridSizer4->Add(btnPlaceRobot, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    FlexGridSizer4->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
    btnLoadMap = new wxCustomButton(this,ID_BUTTON1,_("Load map..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FILE_OPEN")),wxART_BUTTON),wxDefaultPosition,wxSize(-1,60),wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON1"));
    btnLoadMap->SetBitmapDisabled(btnLoadMap->CreateBitmapDisabled(btnLoadMap->GetBitmapLabel()));
    btnLoadMap->SetBitmapMargin(wxSize(2,4));
    FlexGridSizer4->Add(btnLoadMap, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    btnEmptyMap = new wxCustomButton(this,ID_BUTTON9,_("New map..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_NEW")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxSize(-1,50),wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON9"));
    btnEmptyMap->SetBitmapDisabled(btnEmptyMap->CreateBitmapDisabled(btnEmptyMap->GetBitmapLabel()));
    btnEmptyMap->SetBitmapMargin(wxSize(2,4));
    FlexGridSizer4->Add(btnEmptyMap, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    btnDrawMapObs = new wxCustomButton(this,ID_BUTTON8,_("Draw occupied..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_GO_FORWARD")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxSize(-1,50),wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON8"));
    btnDrawMapObs->SetBitmapDisabled(btnDrawMapObs->CreateBitmapDisabled(btnDrawMapObs->GetBitmapLabel()));
    btnDrawMapObs->SetBitmapMargin(wxSize(2,4));
    FlexGridSizer4->Add(btnDrawMapObs, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    btnDrawEmpty = new wxCustomButton(this,ID_BUTTON11,_("Draw empty..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_GO_FORWARD")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxSize(-1,50),wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON11"));
    btnDrawEmpty->SetBitmapDisabled(btnDrawEmpty->CreateBitmapDisabled(btnDrawEmpty->GetBitmapLabel()));
    btnDrawEmpty->SetBitmapMargin(wxSize(2,4));
    FlexGridSizer4->Add(btnDrawEmpty, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    btnSaveMap = new wxCustomButton(this,ID_BUTTON10,_("Save map..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FILE_SAVE")),wxART_BUTTON),wxDefaultPosition,wxSize(-1,64),wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON10"));
    btnSaveMap->SetBitmapDisabled(btnSaveMap->CreateBitmapDisabled(btnSaveMap->GetBitmapLabel()));
    btnSaveMap->SetBitmapMargin(wxSize(2,4));
    FlexGridSizer4->Add(btnSaveMap, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    FlexGridSizer4->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
    btnHelp = new wxCustomButton(this,ID_BUTTON2,_("About..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_QUESTION")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxSize(-1,60),wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON2"));
    btnHelp->SetBitmapDisabled(btnHelp->CreateBitmapDisabled(btnHelp->GetBitmapLabel()));
    btnHelp->SetBitmapMargin(wxSize(5,4));
    FlexGridSizer4->Add(btnHelp, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    btnQuit = new wxCustomButton(this,ID_BUTTON3,_("Exit"),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_QUIT")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxSize(-1,60),wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON3"));
    btnQuit->SetBitmapDisabled(btnQuit->CreateBitmapDisabled(btnQuit->GetBitmapLabel()));
    FlexGridSizer4->Add(btnQuit, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    FlexGridSizer1->Add(FlexGridSizer4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    SplitterWindow1 = new wxSplitterWindow(this, ID_SPLITTERWINDOW1, wxDefaultPosition, wxDefaultSize, wxSP_3D, _T("ID_SPLITTERWINDOW1"));
    SplitterWindow1->SetMinSize(wxSize(10,10));
    SplitterWindow1->SetMinimumPaneSize(10);
    Panel1 = new wxPanel(SplitterWindow1, ID_PANEL1, wxDefaultPosition, wxSize(400,143), wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    FlexGridSizer2 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer2->AddGrowableCol(1);
    FlexGridSizer2->AddGrowableRow(0);
    pnNavSelButtons = new wxPanel(Panel1, ID_PANEL6, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL6"));
    FlexGridSizer9 = new wxFlexGridSizer(0, 2, 0, 0);
    FlexGridSizer9->AddGrowableCol(0);
    wxString __wxRadioBoxChoices_1[2] = 
    {
    	_("Autonavigation (reactive)"),
    	_("Preprogrammed sequences")
    };
    rbNavMode = new wxRadioBox(pnNavSelButtons, ID_RADIOBOX2, _("Navigator"), wxDefaultPosition, wxDefaultSize, 2, __wxRadioBoxChoices_1, 1, wxRA_SPECIFY_COLS, wxDefaultValidator, _T("ID_RADIOBOX2"));
    FlexGridSizer9->Add(rbNavMode, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer3 = new wxFlexGridSizer(0, 2, 0, 0);
    cbEnableLog = new wxCheckBox(pnNavSelButtons, ID_CHECKBOX1, _("Log trajectory"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
    cbEnableLog->SetValue(false);
    FlexGridSizer3->Add(cbEnableLog, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer3->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbNavLog = new wxCheckBox(pnNavSelButtons, ID_CHECKBOX2, _("Record .navlog files"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX2"));
    cbNavLog->SetValue(false);
    FlexGridSizer3->Add(cbNavLog, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer3->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbShowPredictedPTG = new wxCheckBox(pnNavSelButtons, ID_CHECKBOX3, _("Show selected PTG prediction"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX3"));
    cbShowPredictedPTG->SetValue(true);
    FlexGridSizer3->Add(cbShowPredictedPTG, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    cbDrawShapePath = new wxCheckBox(pnNavSelButtons, ID_CHECKBOX4, _("and draw robot shape"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX4"));
    cbDrawShapePath->SetValue(true);
    FlexGridSizer3->Add(cbDrawShapePath, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbWaypointsAllowSkip = new wxCheckBox(pnNavSelButtons, ID_CHECKBOX5, _("Allow skip waypoints"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX5"));
    cbWaypointsAllowSkip->SetValue(true);
    FlexGridSizer3->Add(cbWaypointsAllowSkip, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer9->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    wxString __wxRadioBoxChoices_2[2] = 
    {
    	_("Differential (Ackermann) drive"),
    	_("Holonomic")
    };
    rbKinType = new wxRadioBox(pnNavSelButtons, ID_RADIOBOX1, _("Robot kinematics type"), wxDefaultPosition, wxDefaultSize, 2, __wxRadioBoxChoices_2, 1, wxRA_SPECIFY_COLS, wxDefaultValidator, _T("ID_RADIOBOX1"));
    FlexGridSizer9->Add(rbKinType, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    pnNavSelButtons->SetSizer(FlexGridSizer9);
    FlexGridSizer9->Fit(pnNavSelButtons);
    FlexGridSizer9->SetSizeHints(pnNavSelButtons);
    FlexGridSizer2->Add(pnNavSelButtons, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Notebook1 = new wxNotebook(Panel1, ID_NOTEBOOK1, wxDefaultPosition, wxDefaultSize, 0, _T("ID_NOTEBOOK1"));
    Notebook1->SetMinSize(wxSize(100,50));
    pnParamsGeneral = new wxPanel(Notebook1, ID_PANEL2, wxDefaultPosition, wxSize(474,170), wxTAB_TRAVERSAL, _T("ID_PANEL2"));
    FlexGridSizer7 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer7->AddGrowableCol(0);
    FlexGridSizer7->AddGrowableRow(0);
    edParamsGeneral = new wxTextCtrl(pnParamsGeneral, ID_TEXTCTRL1, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER|wxTE_PROCESS_TAB|wxTE_MULTILINE|wxHSCROLL|wxVSCROLL|wxALWAYS_SHOW_SB, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    edParamsGeneral->SetMinSize(wxSize(-1,100));
    wxFont edParamsGeneralFont(8,wxTELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    edParamsGeneral->SetFont(edParamsGeneralFont);
    FlexGridSizer7->Add(edParamsGeneral, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    pnParamsGeneral->SetSizer(FlexGridSizer7);
    FlexGridSizer7->SetSizeHints(pnParamsGeneral);
    pnParamsReactive = new wxPanel(Notebook1, ID_PANEL4, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL4"));
    FlexGridSizer8 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer8->AddGrowableCol(0);
    FlexGridSizer8->AddGrowableRow(0);
    edParamsReactive = new wxTextCtrl(pnParamsReactive, ID_TEXTCTRL4, _("# ------------------------------------------------------------------------\n# Example configuration file for MRPT Reactive Navigation engine.\n# See C++ documentation: http://reference.mrpt.org/svn/classmrpt_1_1nav_1_1_c_reactive_navigation_system.html\n# See ROS node documentation: http://wiki.ros.org/mrpt_reactivenav2d\n# ------------------------------------------------------------------------\n\n[GLOBAL_CONFIG]\n# 0 or `hmVIRTUAL_FORCE_FIELDS`: Virtual Force Field\n# 1 or `hmSEARCH_FOR_BEST_GAP`: Nearness Diagram (ND)\n# `hmFULL_EVAL`: Evaluation of all possible directions\nHOLONOMIC_METHOD = hmFULL_EVAL\nALARM_SEEMS_NOT_APPROACHING_TARGET_TIMEOUT=100 # (seconds)\n\nrobotMax_V_mps = 2.0\t\t# Speed limits\nrobotMax_W_degps =120\n\n\n# Parameters for the \"FullEval\" Holonomic method\n# ----------------------------------------------------\n[FULL_EVAL_CONFIG]\nfactorWeights=1.0 1.0 1.0 0.01 3.0\n// 0: Clearness in direction\n// 1: Closest approach to target along straight line (Euclidean)\n// 2: Distance of end collision-free point to target (Euclidean)\n// 3: Hysteresis\n// 4: Clearness to nearest obstacle along path\n\nTARGET_SLOW_APPROACHING_DISTANCE = 0.70 // Start to reduce speed when closer than this to target.\nTOO_CLOSE_OBSTACLE = 0.05 // Directions with collision-free distances below this threshold are not elegible.\nHYSTERESIS_SECTOR_COUNT = 5 // Range of \"sectors\" (directions) for hysteresis over succesive timesteps\n\nPHASE1_FACTORS = 0 1 2 // Indices of the factors above to be considered in phase 1\nPHASE2_FACTORS = 1 4 // Indices of the factors above to be considered in phase 2\nPHASE1_THRESHOLD = 0.6 // Indices of the factors above to be considered in phase 1\n\n#\tParameters for the \"VFF\" Holonomic method\n# ----------------------------------------------------\n[VFF_CONFIG]\nTARGET_SLOW_APPROACHING_DISTANCE = 0.1 // For stopping gradually\nTARGET_ATTRACTIVE_FORCE = 2.000000e+001 // Dimension-less (may have to be tuned depending on the density of obstacle sampling)\n\n#\tParameters for the \"Nearness diagram\" Holonomic method\n# ----------------------------------------------------\n[ND_CONFIG]\nWIDE_GAP_SIZE_PERCENT = 2.500000e-001 \nMAX_SECTOR_DIST_FOR_D2_PERCENT = 2.500000e-001 \nRISK_EVALUATION_SECTORS_PERCENT = 1.000000e-001 \nRISK_EVALUATION_DISTANCE = 4.000000e-001 // In normalized ps-meters [0,1]\nTOO_CLOSE_OBSTACLE = 1.500000e-001 // For stopping gradually\nTARGET_SLOW_APPROACHING_DISTANCE = 0.1 // In normalized ps-meters\nfactorWeights = 1.00 0.50 2.00 0.40 // [0]=Free space, [1]=Dist. in sectors, [2]=Closer to target (Euclidean), [3]=Hysteresis\n\n# ----------------------------------------------------\n#\tParameters for navigation: DIFFERENTIAL DRIVEN Robot\n# ----------------------------------------------------\n[DIFF_ReactiveParams]\nweights=0.5 0.05 0.5 2.0 0.5 0.3\n# 1: Free space\n# 2: Dist. in sectors\t\t\t\n# 3: Heading toward target\n# 4: Closer to target (euclidean)\n# 5: Hysteresis\n# 6: Security Distance\n\nDIST_TO_TARGET_FOR_SENDING_EVENT=0\t# Minimum. distance to target for sending the end event. Set to 0 to send it just on navigation end\n\nMinObstaclesHeight=0.0 \t\t# Minimum coordinate in the \"z\" axis for an obstacle to be taken into account.\nMaxObstaclesHeight=1.40 \t# Maximum coordinate in the \"z\" axis for an obstacle to be taken into account.\n\nMAX_REFERENCE_DISTANCE = 10.0 # Maximum distance to build PTGs (in meters), i.e. the visibility \"range\" of tentative paths\n\n# The constant time of a first-order low-pass filter of outgoing speed commands, \n# i.e. can be used to impose a maximum acceleration.\nSPEEDFILTER_TAU = 0.5 // seconds\n\n# PTGs: See classes derived from mrpt::nav::CParameterizedTrajectoryGenerator ( http://reference.mrpt.org/svn/classmrpt_1_1nav_1_1_c_parameterized_trajectory_generator.html)# refer to papers for details.\n#------------------------------------------------------------------------------\nPTG_COUNT = 3\n\nPTG0_Type = CPTG_DiffDrive_C\nPTG0_resolution = 0.05 # Look-up-table cell size or resolution (in meters)\nPTG0_refDistance= 10.0 # Maximum distance to build PTGs (in meters), i.e. the visibility \"range\" of tentative paths\nPTG0_num_paths= 121\nPTG0_v_max_mps = 1.0\nPTG0_w_max_dps = 60\nPTG0_K = 1.0\nPTG0_score_priority = 1.0\n\nPTG1_Type = CPTG_DiffDrive_alpha\nPTG1_resolution = 0.05 # Look-up-table cell size or resolution (in meters)\nPTG1_refDistance= 10.0 # Maximum distance to build PTGs (in meters), i.e. the visibility \"range\" of tentative paths\nPTG1_num_paths = 121\nPTG1_v_max_mps = 1.0\nPTG1_w_max_dps = 60\nPTG1_cte_a0v_deg = 57\nPTG1_cte_a0w_deg = 57\nPTG1_score_priority = 1.0\n\nPTG2_Type = CPTG_DiffDrive_C\nPTG2_resolution = 0.05 # Look-up-table cell size or resolution (in meters)\nPTG2_refDistance= 10.0 # Maximum distance to build PTGs (in meters), i.e. the visibility \"range\" of tentative paths\nPTG2_num_paths = 121\nPTG2_v_max_mps = 1.0\nPTG2_w_max_dps = 60\nPTG2_K = -1.0\nPTG2_score_priority = 0.5\n\n\n# Default 2D robot shape for collision checks: (ignored in ROS, superseded by node parameters)\n# Each PTG will use only one of either (a) polygonal 2D shape or, (b) radius of a circular shape\nRobotModel_shape2D_xs=-0.2 0.5 0.5 -0.2\nRobotModel_shape2D_ys=0.3 0.3 -0.3 -0.3\nRobotModel_circular_shape_radius = 0.5\n\n# ----------------------------------------------------\n#\tParameters for navigation: DIFFERENTIAL DRIVEN Robot\n# ----------------------------------------------------\n[HOLO_ReactiveParams]\nweights=0.5 0.05 0.5 2.0 0.2 0.3\n# 1: Free space\n# 2: Dist. in sectors\t\t\t\n# 3: Heading toward target\n# 4: Closer to target (euclidean)\n# 5: Hysteresis\n# 6: Security Distance\n\nDIST_TO_TARGET_FOR_SENDING_EVENT=0.6\t# Minimum. distance to target for sending the end event. Set to 0 to send it just on navigation end\n\nMinObstaclesHeight=0.0 \t\t# Minimum coordinate in the \"z\" axis for an obstacle to be taken into account.\nMaxObstaclesHeight=1.40 \t# Maximum coordinate in the \"z\" axis for an obstacle to be taken into account.\n\nMAX_REFERENCE_DISTANCE = 10.0 # Maximum distance to build PTGs (in meters), i.e. the visibility \"range\" of tentative paths\n\n# PTGs: See classes derived from mrpt::nav::CParameterizedTrajectoryGenerator ( http://reference.mrpt.org/svn/classmrpt_1_1nav_1_1_c_parameterized_trajectory_generator.html)# refer to papers for details.\n#------------------------------------------------------------------------------\nPTG_COUNT = 1\n\nPTG0_Type = CPTG_Holo_Blend\nPTG0_refDistance= 6.0 # Maximum distance to build PTGs (in meters), i.e. the visibility \"range\" of tentative paths\nPTG0_num_paths = 100\nPTG0_v_max_mps = 1.5\nPTG0_w_max_dps = 90\nPTG0_T_ramp_max = 0.8\nPTG0_score_priority = 1.0\n\n# Each PTG will use only one of either (a) polygonal 2D shape or, (b) radius of a circular shape\nRobotModel_circular_shape_radius = 0.36\n\n"), wxDefaultPosition, wxSize(226,111), wxTE_PROCESS_ENTER|wxTE_PROCESS_TAB|wxTE_MULTILINE|wxHSCROLL|wxVSCROLL|wxALWAYS_SHOW_SB, wxDefaultValidator, _T("ID_TEXTCTRL4"));
    edParamsReactive->SetMinSize(wxSize(-1,100));
    wxFont edParamsReactiveFont(8,wxTELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    edParamsReactive->SetFont(edParamsReactiveFont);
    FlexGridSizer8->Add(edParamsReactive, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    pnParamsReactive->SetSizer(FlexGridSizer8);
    FlexGridSizer8->Fit(pnParamsReactive);
    FlexGridSizer8->SetSizeHints(pnParamsReactive);
    pnParamsPreprog = new wxPanel(Notebook1, ID_PANEL3, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL3"));
    FlexGridSizer6 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer6->AddGrowableCol(0);
    FlexGridSizer6->AddGrowableRow(0);
    edManualSeqs = new wxTextCtrl(pnParamsPreprog, ID_TEXTCTRL3, _("# Params for mode \"Preprogrammed sequences\" \n# Used for debug / testing. Separate sections for different robot kinematic models. # Enter below a list of velocity cmds and they will be executed sequentially. \n\n# Kinematics: Diff driven \n# Format: cmd{%d} = {time_of_the_cmd_sec} {lin_vel_mps} {rot_speed_radps}\n[DIFF_CMDS]\ncmd1= 1.0 0.5 0.0\ncmd2= 4.0 1.0 0.5\ncmd3= 6.0 0.0 0.0\n\n\n# Kinematics: Holonomic \n# Format: cmd{%d} = {time_of_the_cmd_sec} {velocity_mps} {global_orientation_rad} {ramp_time_secs} {rot_speed_radps}\n[HOLO_CMDS]\ncmd1= 1.0 0.5 0.1.0 1.0 0.9\ncmd2= 4.0 1.0 0.4 1.0 0.9\ncmd3= 6.0 0.0 0.0 1.0 0.0\n"), wxDefaultPosition, wxSize(230,167), wxTE_PROCESS_ENTER|wxTE_PROCESS_TAB|wxTE_MULTILINE|wxHSCROLL|wxVSCROLL|wxALWAYS_SHOW_SB, wxDefaultValidator, _T("ID_TEXTCTRL3"));
    edManualSeqs->SetMinSize(wxSize(-1,100));
    wxFont edManualSeqsFont(8,wxTELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    edManualSeqs->SetFont(edManualSeqsFont);
    FlexGridSizer6->Add(edManualSeqs, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    pnParamsPreprog->SetSizer(FlexGridSizer6);
    FlexGridSizer6->Fit(pnParamsPreprog);
    FlexGridSizer6->SetSizeHints(pnParamsPreprog);
    Notebook1->AddPage(pnParamsGeneral, _("Params: Simulation"), true);
    Notebook1->AddPage(pnParamsReactive, _("Params: Reactive navigator"), false);
    Notebook1->AddPage(pnParamsPreprog, _("Params: Preprogrammed seq."), false);
    FlexGridSizer2->Add(Notebook1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    Panel1->SetSizer(FlexGridSizer2);
    FlexGridSizer2->SetSizeHints(Panel1);
    Panel5 = new wxPanel(SplitterWindow1, ID_PANEL5, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL5"));
    BoxSizer3 = new wxBoxSizer(wxHORIZONTAL);
    SplitterWindow2 = new wxSplitterWindow(Panel5, ID_SPLITTERWINDOW2, wxDefaultPosition, wxDefaultSize, wxSP_3D, _T("ID_SPLITTERWINDOW2"));
    SplitterWindow2->SetMinSize(wxSize(10,10));
    SplitterWindow2->SetMinimumPaneSize(10);
    Panel2 = new wxPanel(SplitterWindow2, ID_PANEL7, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL7"));
    FlexGridSizer10 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer10->AddGrowableCol(1);
    FlexGridSizer10->AddGrowableRow(0);
    StaticText3 = new wxStaticText(Panel2, ID_STATICTEXT3, _("Log:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
    FlexGridSizer10->Add(StaticText3, 1, wxALL|wxALIGN_TOP|wxALIGN_CENTER_HORIZONTAL, 5);
    edLog = new wxTextCtrl(Panel2, ID_TEXTCTRL5, wxEmptyString, wxDefaultPosition, wxSize(612,85), wxTE_MULTILINE|wxTE_READONLY|wxTE_DONTWRAP|wxALWAYS_SHOW_SB, wxDefaultValidator, _T("ID_TEXTCTRL5"));
    edLog->SetMinSize(wxSize(-1,70));
    edLog->SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BTNFACE));
    wxFont edLogFont(8,wxTELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    edLog->SetFont(edLogFont);
    FlexGridSizer10->Add(edLog, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel2->SetSizer(FlexGridSizer10);
    FlexGridSizer10->Fit(Panel2);
    FlexGridSizer10->SetSizeHints(Panel2);
    Panel3 = new wxPanel(SplitterWindow2, ID_PANEL8, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL8"));
    FlexGridSizer5 = new wxFlexGridSizer(2, 2, 0, 0);
    FlexGridSizer5->AddGrowableCol(0);
    FlexGridSizer5->AddGrowableCol(1);
    FlexGridSizer5->AddGrowableRow(1);
    StaticText2 = new wxStaticText(Panel3, ID_STATICTEXT2, _("[3D view of the simulated world]"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    FlexGridSizer5->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText1 = new wxStaticText(Panel3, ID_STATICTEXT1, _("[Local view]"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    FlexGridSizer5->Add(StaticText1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    m_plot3D = new CMyGLCanvas(Panel3,ID_XY_GLCANVAS,wxDefaultPosition,wxSize(450,350),wxTAB_TRAVERSAL,_T("ID_XY_GLCANVAS"));
    FlexGridSizer5->Add(m_plot3D, 1, wxALL|wxEXPAND|wxFIXED_MINSIZE|wxALIGN_LEFT|wxALIGN_TOP, 1);
    FlexGridSizer11 = new wxFlexGridSizer(4, 1, 0, 0);
    FlexGridSizer11->AddGrowableCol(0);
    FlexGridSizer11->AddGrowableRow(2);
    StaticText4 = new wxStaticText(Panel3, ID_STATICTEXT4, _("Show for PTG:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT4"));
    FlexGridSizer11->Add(StaticText4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    cbSelPTG = new wxChoice(Panel3, ID_CHOICE1, wxDefaultPosition, wxDefaultSize, 0, 0, 0, wxDefaultValidator, _T("ID_CHOICE1"));
    FlexGridSizer11->Add(cbSelPTG, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    m_plotLocalView = new CMyGLCanvas(Panel3,ID_CUSTOM1,wxDefaultPosition,wxSize(150,150),wxTAB_TRAVERSAL,_T("ID_CUSTOM1"));
    FlexGridSizer11->Add(m_plotLocalView, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    edInfoLocalView = new wxTextCtrl(Panel3, ID_TEXTCTRL2, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE|wxTE_READONLY|wxTE_DONTWRAP|wxALWAYS_SHOW_SB, wxDefaultValidator, _T("ID_TEXTCTRL2"));
    edInfoLocalView->SetMinSize(wxSize(-1,50));
    edInfoLocalView->SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BTNFACE));
    wxFont edInfoLocalViewFont(8,wxTELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    edInfoLocalView->SetFont(edInfoLocalViewFont);
    FlexGridSizer11->Add(edInfoLocalView, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    FlexGridSizer5->Add(FlexGridSizer11, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel3->SetSizer(FlexGridSizer5);
    FlexGridSizer5->Fit(Panel3);
    FlexGridSizer5->SetSizeHints(Panel3);
    SplitterWindow2->SplitHorizontally(Panel2, Panel3);
    BoxSizer3->Add(SplitterWindow2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel5->SetSizer(BoxSizer3);
    BoxSizer3->Fit(Panel5);
    BoxSizer3->SetSizeHints(Panel5);
    SplitterWindow1->SplitHorizontally(Panel1, Panel5);
    FlexGridSizer1->Add(SplitterWindow1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
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
    FlexGridSizer1->SetSizeHints(this);
    Center();
    
    Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&reactive_navigator_demoframe::OnbtnStartClick);
    Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&reactive_navigator_demoframe::OnbtnStopClick);
    Connect(ID_BUTTON7,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&reactive_navigator_demoframe::OnbtnPlaceTargetClick);
    Connect(ID_BUTTON12,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&reactive_navigator_demoframe::OnbtnSetWaypointSeqClick);
    Connect(ID_BUTTON6,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&reactive_navigator_demoframe::OnbtnPlaceRobotClick);
    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&reactive_navigator_demoframe::OnbtnLoadMapClick);
    Connect(ID_BUTTON9,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&reactive_navigator_demoframe::OnbtnEmptyMapClick);
    Connect(ID_BUTTON8,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&reactive_navigator_demoframe::OnbtnDrawMapObsClick);
    Connect(ID_BUTTON11,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&reactive_navigator_demoframe::OnbtnDrawEmptyClick);
    Connect(ID_BUTTON10,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&reactive_navigator_demoframe::OnbtnSaveMapClick);
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&reactive_navigator_demoframe::OnAbout);
    Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&reactive_navigator_demoframe::OnbtnQuitClick);
    Connect(ID_RADIOBOX1,wxEVT_COMMAND_RADIOBOX_SELECTED,(wxObjectEventFunction)&reactive_navigator_demoframe::OnrbKinTypeSelect);
    Connect(ID_TEXTCTRL3,wxEVT_COMMAND_TEXT_UPDATED,(wxObjectEventFunction)&reactive_navigator_demoframe::OnedManualKinRampsText);
    Connect(ID_NOTEBOOK1,wxEVT_COMMAND_NOTEBOOK_PAGE_CHANGED,(wxObjectEventFunction)&reactive_navigator_demoframe::OnNotebook1PageChanged1);
    Connect(ID_MENUITEM4,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&reactive_navigator_demoframe::OnbtnLoadMapClick);
    Connect(idMenuQuit,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&reactive_navigator_demoframe::OnQuit);
    Connect(ID_MENUITEM1,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&reactive_navigator_demoframe::OnMenuItemChangeVisibleStuff);
    Connect(ID_MENUITEM2,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&reactive_navigator_demoframe::OnMenuItemChangeVisibleStuff);
    Connect(ID_MENUITEM3,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&reactive_navigator_demoframe::OnMenuItemClearRobotPath);
    Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&reactive_navigator_demoframe::OnAbout);
    Connect(ID_TIMER1,wxEVT_TIMER,(wxObjectEventFunction)&reactive_navigator_demoframe::OntimRunSimulTrigger);
    //*)

	// Load updated cfg file:
	try {
		const std::string sFil = mrpt::system::find_mrpt_shared_dir() + std::string("config_files/navigation-ptgs/reactivenav-app-config.ini");
		edParamsReactive->LoadFile(_U(sFil.c_str()));
	}
	catch (...) {}

	btnStart->SetToolTip(wxT("Initializes and starts the simulator."));
	btnPlaceTarget->SetToolTip(wxT("Left-click on the map to place a navigation command."));
	btnStop->SetToolTip(wxT("Stops the simulation."));
	btnSetWaypointSeq->SetToolTip(wxT("Left-click on the map to place waypoints, right click to end and start navigation."));

	SplitterWindow1->SetSashPosition(200);
	SplitterWindow2->SetSashPosition(90);

	m_plot3D->Connect(wxEVT_MOTION,(wxObjectEventFunction)&reactive_navigator_demoframe::Onplot3DMouseMove,0,this);
	m_plot3D->Connect(wxEVT_LEFT_DOWN,(wxObjectEventFunction)&reactive_navigator_demoframe::Onplot3DMouseClick,0,this);
	m_plot3D->Connect(wxEVT_RIGHT_DOWN,(wxObjectEventFunction)&reactive_navigator_demoframe::Onplot3DMouseClick,0,this);

	mnuViewMaxRange->Check(true);
	mnuViewRobotPath->Check(true);

	updateButtonsEnableState(false);

	// Redirect all output to control:
	m_myRedirector = new CMyRedirector( edLog, true, 100, true);

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

	// Robot viz is built in OnrbKinTypeSelect()
	gl_robot_local = mrpt::opengl::CSetOfObjects::Create();
	gl_robot = mrpt::opengl::CSetOfObjects::Create();
	{
		mrpt::opengl::CSetOfObjectsPtr gl_robot_render = mrpt::opengl::CSetOfObjects::Create();
		gl_robot_render->setName("robot_render");
		gl_robot->insert(gl_robot_render);
	}
	m_plot3D->m_openGLScene->insert(gl_robot);

	gl_scan3D = mrpt::opengl::CPlanarLaserScan::Create();
	gl_scan3D->enableLine(false);
	gl_scan3D->enableSurface(true);
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
	gl_target->setVisibility(false);
	{
		mrpt::opengl::CArrowPtr obj;
		obj = mrpt::opengl::CArrow::Create( 1,0,0,  0.2f,0,0, 0.4f,0.05f, 0.15f ); obj->setColor_u8( TColor(0,0,255) ); gl_target->insert(obj);
		obj = mrpt::opengl::CArrow::Create(-1,0,0, -0.2f,0,0, 0.4f,0.05f, 0.15f ); obj->setColor_u8( TColor(0,0,255) ); gl_target->insert(obj);
		obj = mrpt::opengl::CArrow::Create( 0,1,0,  0,0.2f,0, 0.4f,0.05f, 0.15f ); obj->setColor_u8( TColor(0,0,255) ); gl_target->insert(obj);
		obj = mrpt::opengl::CArrow::Create(0,-1,0,  0,-0.2f,0, 0.4f,0.05f, 0.15f ); obj->setColor_u8( TColor(0,0,255) ); gl_target->insert(obj);
		m_plot3D->m_openGLScene->insert(gl_target);
	}

	{
		gl_waypoints_clicking = opengl::CSetOfObjects::Create();
		m_plot3D->m_openGLScene->insert(gl_waypoints_clicking);

		gl_waypoints_status= opengl::CSetOfObjects::Create();
		m_plot3D->m_openGLScene->insert(gl_waypoints_status);
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
		mrpt::opengl::CSetOfObjectsPtr obj = mrpt::opengl::stock_objects::CornerXYZSimple(1.0,2.0);
		obj->setColor_u8( TColor(255,0,0, 120) );
		m_gl_placing_robot->insert(obj);

		m_gl_placing_robot->setVisibility(false); // Start invisible.
		m_plot3D->m_openGLScene->insert(m_gl_placing_robot);
	}
	{	// Sign of "drawing obstacles":
		m_gl_drawing_obs = opengl::CSetOfObjects::Create();

		mrpt::opengl::CCylinderPtr obj = mrpt::opengl::CCylinder::Create(0.05f, 0.10f,1.0f);
		obj->setColor_u8( mrpt::utils::TColor(0xff,0x00,0x00,0x70) );
		m_gl_drawing_obs->insert(obj);

		m_gl_drawing_obs->setVisibility(false); // Start invisible.
		m_plot3D->m_openGLScene->insert(m_gl_drawing_obs);
	}

	m_plot3D->m_openGLScene->insert( mrpt::opengl::stock_objects::CornerXYZ(1) );

	gl_robot_ptg_prediction = mrpt::opengl::CSetOfLines::Create();
	gl_robot_ptg_prediction->setName("ptg_prediction");
	gl_robot_ptg_prediction->setLineWidth(2.0);
	gl_robot_ptg_prediction->setColor_u8( mrpt::utils::TColor(0x00,0x00,0xff) );
	gl_robot->insert(gl_robot_ptg_prediction);

	// Set camera:
	m_plot3D->cameraPointingX=0;
	m_plot3D->cameraPointingY=0;
	m_plot3D->cameraPointingZ=0;
	m_plot3D->cameraZoomDistance = 40;
	m_plot3D->cameraElevationDeg = 70;
	m_plot3D->cameraAzimuthDeg = -100;
	m_plot3D->cameraIsProjective = true;

	// Init simulator & its adaptor to the navigator
	{
		wxCommandEvent ev;
		OnrbKinTypeSelect(ev);
	}

	// 2D view ==============
	{
		mrpt::opengl::CGridPlaneXYPtr obj = mrpt::opengl::CGridPlaneXY::Create(-1,1.001f, -1,1.001f, 0, 1);
		obj->setColor_u8(TColor(30,30,30,50));
		m_plotLocalView->m_openGLScene->insert( obj );
	}

	gl_line_direction = mrpt::opengl::CSimpleLine::Create();
	gl_line_direction->setLineWidth(4);
	gl_line_direction->setColor_u8(TColor(0,0,0));
	m_plotLocalView->m_openGLScene->insert(gl_line_direction);

	gl_rel_target = mrpt::opengl::CPointCloud::Create();
	gl_rel_target->setPointSize(7);
	gl_rel_target->setColor_u8(TColor(0,0,255));
	gl_rel_target->insertPoint(0,0,0);
	m_plotLocalView->m_openGLScene->insert(gl_rel_target);

	gl_rel_robot = mrpt::opengl::CPointCloud::Create();
	gl_rel_robot->setPointSize(5);
	gl_rel_robot->setColor_u8(TColor(0, 0, 0));
	gl_rel_robot->insertPoint(0, 0, 0);
	m_plotLocalView->m_openGLScene->insert(gl_rel_robot);

	m_plotLocalView->m_openGLScene->insert( mrpt::opengl::stock_objects::CornerXYSimple(0.1f,2) );
	m_plotLocalView->m_openGLScene->insert( gl_robot_local );

	gl_tp_obstacles = mrpt::opengl::CSetOfLines::Create();
	gl_tp_obstacles->setLineWidth(3);
	gl_tp_obstacles->setColor_u8( TColor(0,0,0) );
	m_plotLocalView->m_openGLScene->insert(gl_tp_obstacles);

	gl_nd_gaps = mrpt::opengl::CSetOfLines::Create();
	gl_nd_gaps->setLineWidth(1);
	gl_nd_gaps->setColor_u8( TColor(255,0,0) );
	m_plotLocalView->m_openGLScene->insert(gl_nd_gaps);

	m_plotLocalView->clearColorR = 0.9f;
	m_plotLocalView->clearColorG = 0.9f;
	m_plotLocalView->clearColorB = 0.9f;

	// Set camera:
	m_plotLocalView->cameraPointingX=0;
	m_plotLocalView->cameraPointingY=0;
	m_plotLocalView->cameraPointingZ=0;
	m_plotLocalView->cameraZoomDistance = 2.2f;
	m_plotLocalView->cameraElevationDeg = 90;
	m_plotLocalView->cameraAzimuthDeg = -90;
	m_plotLocalView->cameraIsProjective = false;


	// Update positions of stuff:
	this->updateViewsDynamicObjects();


	// Retrieve default parameters for holonomic methods:
	// ------------------------------------------------------
	{
		mrpt::utils::CConfigFileMemory cfg;

		m_simul_options.saveToConfigFile(cfg,"SIMULATOR");
		edParamsGeneral->SetValue( _U( cfg.getContent().c_str() ) );
	}

	WX_END_TRY

	this->Maximize();
}

reactive_navigator_demoframe::~reactive_navigator_demoframe()
{
    //(*Destroy(reactive_navigator_demoframe)
    //*)

	// Destroy this first to avoid problems, since it may contain a ref of the simulator object:
	m_navMethod.reset();

	delete m_myRedirector;
	m_myRedirector=NULL;
}

void reactive_navigator_demoframe::OnQuit(wxCommandEvent& event)
{
    Close();
}

void reactive_navigator_demoframe::OnAbout(wxCommandEvent& event)
{
	CAboutBox dlg(this);
	dlg.ShowModal();
}

void reactive_navigator_demoframe::updateMap3DView()
{
	gl_grid->clear();
	m_gridMap.getAs3DObject(gl_grid);
}


void reactive_navigator_demoframe::OnbtnPlaceRobotClick(wxCommandEvent& event)
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

void reactive_navigator_demoframe::OnbtnPlaceTargetClick(wxCommandEvent& event)
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

void reactive_navigator_demoframe::updateButtonsEnableState(bool is_running)
{
	btnStart->Enable(!is_running); btnStart->Refresh();
	btnStop->Enable(is_running);   btnStop->Refresh();
	btnPlaceTarget->Enable(is_running); btnPlaceTarget->Refresh();
	btnSetWaypointSeq->Enable(is_running); btnSetWaypointSeq->Refresh();
	Notebook1->Enable(!is_running);
	pnNavSelButtons->Enable(!is_running);
}

void reactive_navigator_demoframe::OnbtnStartClick(wxCommandEvent& event)
{
	updateButtonsEnableState(true);
	if (!reinitSimulator())
	{
		updateButtonsEnableState(false);
		return;
	}
	m_is_running = true;
}

void reactive_navigator_demoframe::OnbtnStopClick(wxCommandEvent& event)
{
	updateButtonsEnableState(false);
	m_is_running=false;
}

// Run simulator (when "running"):
void reactive_navigator_demoframe::OntimRunSimulTrigger(wxTimerEvent& event)
{
	try
	{
		static int decim_call_dump_log = 0;
		if (decim_call_dump_log++>10) {
			decim_call_dump_log = 0;
			std::cout.flush();
			std::cerr.flush();
		}

		if (m_is_running) {
			simulateOneStep( NAV_SIMUL_TIMESTEP_MS*1e-3 );
		}
		updateViewsDynamicObjects();
		timRunSimul.Start(NAV_SIMUL_TIMESTEP_MS, true); // execute the simulation step by step ("one shot" timer) so in the event of an exception, we don't output an endless stream of errors.
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
bool reactive_navigator_demoframe::reinitSimulator()
{
	WX_START_TRY

	if (m_robotSimul2NavInterface)
	{
		// Params for simulator-to-nav interface:
		CConfigFileMemory cfg;
		cfg.setContent( std::string(edParamsReactive->GetValue().mb_str() ) );
		//m_robotSimul2NavInterface->loadConfigFile(cfg, "GLOBAL_CONFIG");
	}

	// Delete old & build new navigator:
	m_navMethod.reset();
	CConfigFileMemory cfg;
	switch (rbNavMode->GetSelection())
	{
	case 0:
		{
			mrpt::nav::CReactiveNavigationSystem *react = new mrpt::nav::CReactiveNavigationSystem(*m_robotSimul2NavInterface);
			m_navMethod.reset(react);

			react->enableKeepLogRecords();
			react->enableLogFile( cbNavLog->IsChecked() );

			cfg.setContent( std::string(edParamsReactive->GetValue().mb_str() ) );
			break;
		}
	case 1:
		{
			m_navMethod.reset( new mrpt::nav::CNavigatorManualSequence(*m_robotSimul2NavInterface) );
			cfg.setContent( std::string(edManualSeqs->GetValue().mb_str() ) );
			break;
		}
	default:
		throw std::runtime_error("Invalid nav method selected!");
	};
	ASSERT_(m_navMethod.get())

	// Load params:
	std::string sKinPrefix;
	switch ( rbKinType->GetSelection() )
	{
	case 0: sKinPrefix="DIFF_"; break;
	case 1: sKinPrefix="HOLO_"; break;
	default:
		throw std::runtime_error("Invalid kinematic model selected!");
	};

	m_navMethod->loadConfigFile(cfg,sKinPrefix);
	m_navMethod->initialize();

	// params for simulator itself:
	{
		CConfigFileMemory cfgGeneral;
		cfgGeneral.setContent( std::string(edParamsGeneral->GetValue().mb_str() ) );
		m_simul_options.loadFromConfigFile(cfgGeneral,"SIMULATOR");
	}


	// Update GUI stuff:
	gl_robot_sensor_range->setDiskRadius(m_simul_options.MAX_SENSOR_RADIUS*1.01,m_simul_options.MAX_SENSOR_RADIUS*0.99);
	gl_target->setVisibility(false);
	gl_robot_ptg_prediction->clear();

	cbSelPTG->Clear();
	{
		mrpt::nav::CAbstractPTGBasedReactive *ptg_nav = dynamic_cast<mrpt::nav::CAbstractPTGBasedReactive *>(m_navMethod.get() );
		if (ptg_nav)
		{
			for (size_t i=0;i<ptg_nav->getPTG_count();i++)
				cbSelPTG->Append(_U( ptg_nav->getPTG(i)->getDescription().c_str() ));

			if (ptg_nav->getPTG_count()>0)
				cbSelPTG->SetSelection(0);
		}
	}

	return true;

	WX_END_TRY
	return false;
}

void reactive_navigator_demoframe::simulateOneStep(double time_step)
{
	const double simul_time = m_robotSimul->getTime();

	static double LAST_TIM_LIDAR    = -1e6;
	static double LAST_TIM_REACTIVE = -1e6;

	if (simul_time-LAST_TIM_LIDAR > 1.0/m_simul_options.SENSOR_RATE )
	{
		LAST_TIM_LIDAR = simul_time;

		// Simulate 360deg range scan:
		CObservation2DRangeScan      simulatedScan;

		simulatedScan.aperture = m_simul_options.SENSOR_FOV;
		simulatedScan.rightToLeft = true;
		simulatedScan.maxRange = m_simul_options.MAX_SENSOR_RADIUS;
		simulatedScan.sensorPose = CPose2D(0,0,0.10);

		m_gridMap.laserScanSimulator( simulatedScan, m_robotSimul->getCurrentGTPose(),0.5, m_simul_options.SENSOR_NUM_RANGES, m_simul_options.SENSOR_RANGE_NOISE_STD );

		// Build the obstacles points map for the reactive:
		{
			m_latest_obstacles.insertionOptions.minDistBetweenLaserPoints = 0.005f;
			m_latest_obstacles.insertionOptions.also_interpolate = false;

			m_latest_obstacles.clear(); // erase old points
			m_latest_obstacles.insertObservation( &simulatedScan );
			// m_latest_obstacles is ref-copied into the robot2nav interface.
		}

		gl_scan3D->setScan( simulatedScan );  // Draw real scan in 3D view
	}

	// Navigate:
	if (simul_time-LAST_TIM_REACTIVE > 1.0/m_simul_options.NAVIGATION_RATE )
	{
		LAST_TIM_REACTIVE = simul_time;

		m_navMethod->navigationStep();
	}

	// Run robot simulator:
	m_robotSimul->simulateOneTimeStep(time_step);

	// Update path graph:
	const TPoint3D  cur_pt(m_robotSimul->getCurrentGTPose().x,m_robotSimul->getCurrentGTPose().y,0.01);

	static int decim_path = 0;
	if (gl_robot_path->empty() || ++decim_path>10) {
		gl_robot_path->appendLine(cur_pt,cur_pt);
	}
	else {
		gl_robot_path->appendLineStrip(cur_pt);
		decim_path=0;
	}

	if (cbEnableLog->IsChecked())
	{
		if (!m_log_trajectory_file.is_open())
		{
			if (m_log_trajectory_file.open("traj_log.txt") )
			{
				m_log_trajectory_file.printf("%% File format: TIME  X   Y  PHI  VX  VY OMEGA\n");
			}
		}
		if (m_log_trajectory_file.is_open())  // just in case there was any error opening
		{
			const mrpt::math::TPose2D pose = m_robotSimul->getCurrentGTPose();
			const mrpt::math::TTwist2D vel = m_robotSimul->getCurrentGTVel();
			m_log_trajectory_file.printf("%8.03f  %7.03f %7.03f %7.03f   %7.03f %7.03f %7.03f\n",
				m_robotSimul->getTime(),
				pose.x, pose.y, mrpt::utils::RAD2DEG(pose.phi),
				vel.vx, vel.vy, mrpt::utils::RAD2DEG(vel.omega)
				);
		}
	}
	else
	{
		if (m_log_trajectory_file.is_open())
			m_log_trajectory_file.close();
	}

	mrpt::nav::CAbstractPTGBasedReactive *ptg_nav = dynamic_cast<mrpt::nav::CAbstractPTGBasedReactive * >(m_navMethod.get());


	// Clear stuff which will be updated if used below:
	edInfoLocalView->Clear();
	gl_nd_gaps->clear();

	// Update 2D view graphs:
	mrpt::nav::CLogFileRecord lfr;
	if (ptg_nav) ptg_nav->getLastLogRecord(lfr);

	const int sel_PTG = cbSelPTG->GetSelection();
	if (sel_PTG>=0 && sel_PTG<(int)lfr.infoPerPTG.size())
	{
		const size_t nObs = lfr.infoPerPTG[sel_PTG].TP_Obstacles.size();
		if (lfr.infoPerPTG.size()>0 && IS_CLASS(lfr.infoPerPTG[sel_PTG].HLFR, CLogFileRecord_ND))
		{
			CLogFileRecord_NDPtr log = CLogFileRecord_NDPtr(lfr.infoPerPTG[sel_PTG].HLFR);
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
					const double ang = M_PI *(-1.0 + 2.0 * sec/nObs );

					const double d = lfr.infoPerPTG[sel_PTG].TP_Obstacles[sec]-0.05;
					gl_nd_gaps->appendLineStrip(d*cos(ang),d*sin(ang),0);
				}
				gl_nd_gaps->appendLineStrip(0,0,0);
			}
		}

		// TP-Obstacles:
		gl_tp_obstacles->clear();
		if (nObs>1)
		{
			for (size_t i=0;i<=nObs;i++)
			{
				const double d0 = lfr.infoPerPTG[sel_PTG].TP_Obstacles[i % nObs];
				const double a0 = M_PI * (-1.0 + 2.0 * ((i % nObs)+0.5)/nObs );
				const double d1 = lfr.infoPerPTG[sel_PTG].TP_Obstacles[(i+1) % nObs];
				const double a1 = M_PI * (-1.0 + 2.0 * (((i+1) % nObs)+0.5)/nObs );
				gl_tp_obstacles->appendLine(
					d0*cos(a0),d0*sin(a0),0.0,
					d1*cos(a1),d1*sin(a1),0.0 );
			}
		}

		// Movement direction:
		{
			const double desiredDirection = lfr.infoPerPTG[sel_PTG].desiredDirection;
			const double d = lfr.infoPerPTG[sel_PTG].desiredSpeed; ///ROBOT_MAX_SPEED;
			gl_line_direction->setLineCoords(
				0,0,0,
				cos(desiredDirection) * d, sin(desiredDirection) * d, 0 );
		}

		// TP Target:
		gl_rel_target->setLocation( lfr.infoPerPTG[sel_PTG].TP_Target );
		// TP Robot:
		gl_rel_robot->setLocation(lfr.infoPerPTG[sel_PTG].TP_Robot);

	} // end valid PTG selected

	// Draw predicted path along selected PTG:
	if (cbShowPredictedPTG->IsChecked() && lfr.infoPerPTG.size()>0 && ptg_nav)
	{
		// Selected PTG path:
		if (lfr.nSelectedPTG<=(int)ptg_nav->getPTG_count())  // the == case is for "NOP motion cmd"
		{
			const bool is_NOP_op = (lfr.nSelectedPTG == (int)ptg_nav->getPTG_count());
			const size_t idx_ptg = is_NOP_op ? lfr.ptg_index_NOP : lfr.nSelectedPTG;

			mrpt::nav::CParameterizedTrajectoryGenerator* ptg = ptg_nav->getPTG(idx_ptg);
			if (ptg)
			{
				// Draw path:
				const int selected_k = is_NOP_op ? lfr.ptg_last_k_NOP : ptg->alpha2index( lfr.infoPerPTG[lfr.nSelectedPTG].desiredDirection );
				float max_dist = ptg->getRefDistance();
				gl_robot_ptg_prediction->clear();
				ptg->updateCurrentRobotVel(lfr.ptg_last_curRobotVelLocal);
				ptg->renderPathAsSimpleLine(selected_k,*gl_robot_ptg_prediction,0.10, max_dist);
				gl_robot_ptg_prediction->setColor_u8( mrpt::utils::TColor(0xff,0x00,0x00) );

				// Place it:
				if (is_NOP_op) {
					gl_robot_ptg_prediction->setPose(lfr.rel_pose_PTG_origin_wrt_sense_NOP);
				}
				else {
					gl_robot_ptg_prediction->setPose(CPose3D());
				}

				// Overlay a sequence of robot shapes:
				if (cbDrawShapePath->IsChecked())
				{
					double min_shape_dists = 1.0;
					for (double d=min_shape_dists;d<max_dist;d+=min_shape_dists)
					{
						uint16_t step;
						if (!ptg->getPathStepForDist(selected_k, d, step))
							continue;
						mrpt::math::TPose2D p;
						ptg->getPathPose(selected_k, step, p);
						ptg->add_robotShape_to_setOfLines( *gl_robot_ptg_prediction, mrpt::poses::CPose2D(p) );
					}
				}
			}
		}
	}

	CWaypointsNavigator *wp_nav = dynamic_cast<CWaypointsNavigator *>(m_navMethod.get());
	if (wp_nav)
	{
		static wxFrame *wxFrWpInfo = nullptr; 
		static wxTextCtrl * edWpLog = nullptr;
		if (!wxFrWpInfo)
		{
			wxFrWpInfo= new wxFrame(this, -1, wxT("Waypoints info"), wxDefaultPosition, wxSize(400,150), wxRESIZE_BORDER | wxMINIMIZE_BOX | wxMAXIMIZE_BOX | wxCAPTION | wxCLIP_CHILDREN | wxSTAY_ON_TOP );
			
			edWpLog = new wxTextCtrl(wxFrWpInfo,wxNewId(), wxEmptyString, wxDefaultPosition, wxSize(400,150), wxTE_MULTILINE|wxTE_READONLY|wxTE_DONTWRAP|wxALWAYS_SHOW_SB, wxDefaultValidator, _T("ID_TEXTCTRL_WP"));
			edWpLog->SetMinSize(wxSize(190,60));
			wxFont edLogFont(8,wxFONTFAMILY_TELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
			edWpLog->SetFont(edLogFont);
		}

		TWaypointStatusSequence wp_status;
		wp_nav->getWaypointNavStatus(wp_status);
		const std::string sWpLog = wp_status.getAsText();

		if (!wp_status.waypoints.empty())
		{
			if (!wxFrWpInfo->IsShown()) wxFrWpInfo->Show();
			edWpLog->SetValue( _U(sWpLog.c_str()) );
		}

		// Plot waypoints being clicked by the user graphically:
		gl_waypoints_clicking->clear();
		for (const auto &p : m_waypoints_clicked.waypoints)
		{
			mrpt::opengl::CDiskPtr gl_pt = mrpt::opengl::CDisk::Create(0.3f,0.2f, 20);
			gl_pt->setLocation(p.target.x,p.target.y,0.01);
			gl_pt->setColor_u8(mrpt::utils::TColor(0x00,0x00,0xff));
			gl_waypoints_clicking->insert(gl_pt);
		}

		// Plot firmly set waypoints and their status:
		gl_waypoints_status->clear();
		{
			const mrpt::utils::TColor colNormal (0x00,0x00,0xff);
			const mrpt::utils::TColor colCurrent(0xff,0x00,0x20);
			unsigned int idx=0;
			for (const auto &p : wp_status.waypoints)
			{
				const bool is_cur_goal = (int(idx)==wp_status.waypoint_index_current_goal);

				mrpt::opengl::CDiskPtr gl_pt = mrpt::opengl::CDisk::Create(is_cur_goal ? 0.4f : 0.3f,0.2f, 20);
				gl_pt->setLocation(p.target.x,p.target.y,0.01);
				gl_pt->setName(mrpt::format("WayPt #%2u Reach:%s",idx, p.reached ? "YES":"NO"));
				gl_pt->enableShowName(true);
				gl_pt->setColor_u8( is_cur_goal ? colCurrent : colNormal );
				gl_waypoints_clicking->insert(gl_pt);

				++idx;
			}
		}
	}
}

void reactive_navigator_demoframe::updateViewsDynamicObjects()
{
	gl_robot->setPose( m_robotSimul->getCurrentGTPose() );

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
	StatusBar1->SetStatusText( _U( mrpt::format("Robot pose: %s vel: %s", m_robotSimul->getCurrentGTPose().asString().c_str(),m_robotSimul->getCurrentGTVel().asString().c_str() ).c_str() ), 0 );
	StatusBar1->SetStatusText( _U( mrpt::format("Simul time: %.03f Target: (%.03f,%.03f)", m_robotSimul->getTime(), m_targetPoint.x, m_targetPoint.y ).c_str()  ), 1 );

	// Show/hide:
	gl_robot_sensor_range->setVisibility( mnuViewMaxRange->IsChecked() );
	gl_robot_path->setVisibility( mnuViewRobotPath->IsChecked() );

	// Refresh:
	m_plot3D->Refresh();
	m_plotLocalView->Refresh();
}

void reactive_navigator_demoframe::Onplot3DMouseMove(wxMouseEvent& event)
{
	int X, Y;
	event.GetPosition(&X,&Y);
	bool skip_normal_process = false;

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

		switch (m_cursorPickState)
		{
		case cpsPickTarget:
		case cpsPickWaypoints:
			{
				m_gl_placing_nav_target->setVisibility(true);
				m_gl_placing_nav_target->setLocation(m_curCursorPos.x,m_curCursorPos.y,0.01);
			}
			break;
		case cpsPlaceRobot:
			{
				m_gl_placing_robot->setVisibility(true);
				m_gl_placing_robot->setLocation(m_curCursorPos.x,m_curCursorPos.y,0.01);
			}
			break;
		case cpsDrawObstacles:
		case cpsDrawClear:
			{
				m_gl_drawing_obs->setLocation(m_curCursorPos.x,m_curCursorPos.y,0.01);
				if (event.ButtonIsDown( wxMOUSE_BTN_LEFT ))
				{
					const int cx = m_gridMap.x2idx(m_curCursorPos.x);
					const int cy = m_gridMap.y2idx(m_curCursorPos.y);
					if (cx>=0 && cy>=0 && cx<(int)m_gridMap.getSizeX() && cy<(int)m_gridMap.getSizeY()) {
						m_gridMap.setCell(cx,cy,  m_cursorPickState==cpsDrawObstacles ? 0.01f : 0.99f );
						updateMap3DView();
						m_plot3D->Refresh();
					}
					skip_normal_process=true;
				}
			}
			break;
		default:
			break;
		};

		StatusBar1->SetStatusText(wxString::Format(wxT("X=%.03f Y=%.04f Z=0"),m_curCursorPos.x,m_curCursorPos.y), 2);
	}

	if (!skip_normal_process)
	{
		// Do normal process in that class:
		m_plot3D->OnMouseMove(event);
	}
}

void reactive_navigator_demoframe::Onplot3DMouseClick(wxMouseEvent& event)
{
	WX_START_TRY

	bool skip_normal_process = false;

	switch (m_cursorPickState)
	{
	case cpsPickWaypoints:
		{
			if (event.ButtonIsDown( wxMOUSE_BTN_LEFT ))
			{
				const bool allow_skip_wps = cbWaypointsAllowSkip->IsChecked();
				m_waypoints_clicked.waypoints.push_back( TWaypoint( m_curCursorPos.x,m_curCursorPos.y, 0.2 /* allowed dist */, allow_skip_wps) );
			}
			if (event.ButtonIsDown( wxMOUSE_BTN_RIGHT ))
			{
				btnPlaceTarget->SetValue(false); btnPlaceTarget->Refresh();
				m_gl_placing_nav_target->setVisibility(false);

				CWaypointsNavigator *wp_nav = dynamic_cast<CWaypointsNavigator *>(m_navMethod.get());
				if (wp_nav)
					wp_nav->navigateWaypoints(m_waypoints_clicked);

				m_waypoints_clicked.clear();

				m_plot3D->SetCursor( *wxSTANDARD_CURSOR ); // End of cross cursor
				m_cursorPickState = cpsNone; // end of mode
			}
		}
		break;

	case cpsPickTarget:
		if (event.ButtonIsDown( wxMOUSE_BTN_LEFT ))
		{
			m_targetPoint = m_curCursorPos;

			btnPlaceTarget->SetValue(false);
			btnPlaceTarget->Refresh();
			m_gl_placing_nav_target->setVisibility(false);

			// Issue a new navigation cmd:
			CAbstractPTGBasedReactive::TNavigationParamsPTG   navParams;
			navParams.target.x = m_targetPoint.x ;
			navParams.target.y = m_targetPoint.y ;
			navParams.targetAllowedDistance = 0.40f;
			navParams.targetIsRelative = false;

			// Optional: restrict the PTGs to use
			//navParams.restrict_PTG_indices.push_back(1);

			m_navMethod->navigate( &navParams );
			gl_target->setVisibility(true);

			m_plot3D->SetCursor( *wxSTANDARD_CURSOR ); // End of cross cursor
			m_cursorPickState = cpsNone; // end of mode
			break;
		}
	case cpsPlaceRobot:
		if (event.ButtonIsDown( wxMOUSE_BTN_LEFT ))
		{
			m_robotSimul->setCurrentGTPose( mrpt::math::TPose2D(m_curCursorPos.x,m_curCursorPos.y, .0 ) );

			btnPlaceRobot->SetValue(false);
			btnPlaceRobot->Refresh();
			m_gl_placing_robot->setVisibility(false);
			m_plot3D->SetCursor( *wxSTANDARD_CURSOR ); // End of cross cursor
			m_cursorPickState = cpsNone; // end of mode
			break;
		}
	case cpsDrawObstacles:
	case cpsDrawClear:
		if (event.ButtonIsDown( wxMOUSE_BTN_LEFT ))
		{
			const int cx = m_gridMap.x2idx(m_curCursorPos.x);
			const int cy = m_gridMap.y2idx(m_curCursorPos.y);
			if (cx>=0 && cy>=0 && cx<(int)m_gridMap.getSizeX() && cy<(int)m_gridMap.getSizeY()) {
				m_gridMap.setCell(cx,cy, m_cursorPickState==cpsDrawObstacles ? 0.01f : 0.99f );
				updateMap3DView();
				m_plot3D->Refresh();
			}
			skip_normal_process = true;
			break;
		}
	default:
		break;
	}


	if (!skip_normal_process) {
		// Do normal process in that class:
		m_plot3D->OnMouseDown(event);
	}
	WX_END_TRY
}

// ==== reactive_navigator_demoframe::TOptions ======
reactive_navigator_demoframe::TOptions::TOptions() :
	MAX_SENSOR_RADIUS ( 10.0 ),
	SENSOR_FOV        (M_PI*2.0),
	SENSOR_NUM_RANGES ( 181),
	SENSOR_RANGE_NOISE_STD (0.02),
	SENSOR_RATE(10.0),
	NAVIGATION_RATE(4.0)
{
}
void reactive_navigator_demoframe::TOptions::loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section)
{
	MRPT_START

	// Load from config text:
	MRPT_LOAD_CONFIG_VAR(MAX_SENSOR_RADIUS,double,  source,section );
	MRPT_LOAD_CONFIG_VAR_DEGREES(SENSOR_FOV,source,section );
	MRPT_LOAD_CONFIG_VAR(SENSOR_NUM_RANGES, uint64_t,  source,section );
	MRPT_LOAD_CONFIG_VAR(SENSOR_RANGE_NOISE_STD,double,  source,section );
	MRPT_LOAD_CONFIG_VAR(SENSOR_RATE,double,  source,section );
	MRPT_LOAD_CONFIG_VAR(NAVIGATION_RATE,double,  source,section );

	MRPT_END
}

void reactive_navigator_demoframe::TOptions::saveToConfigFile(mrpt::utils::CConfigFileBase &cfg,const std::string &section) const
{
	MRPT_START
	const int WN = 40, WV = 20;

	cfg.write(section,"MAX_SENSOR_RADIUS",MAX_SENSOR_RADIUS,   WN,WV, "Maximum range of the LiDAR sensor (meters)");
	cfg.write(section,"SENSOR_FOV",180.0/M_PI*SENSOR_FOV,   WN,WV, "Horizontal Field of view of the LiDAR (deg)");
	cfg.write(section,"SENSOR_NUM_RANGES",SENSOR_NUM_RANGES,   WN,WV, "Number of ranges in the 360deg sensor FOV");
	cfg.write(section,"SENSOR_RANGE_NOISE_STD",SENSOR_RANGE_NOISE_STD,   WN,WV, "Sensor noise (one sigma, in meters)");
	cfg.write(section,"SENSOR_RATE",SENSOR_RATE,   WN,WV, "Sensor rate (Hz)");
	cfg.write(section,"NAVIGATION_RATE",NAVIGATION_RATE,   WN,WV, "Navigation algorithm rate (Hz)");

	MRPT_END
}


void reactive_navigator_demoframe::OnMenuItemChangeVisibleStuff(wxCommandEvent& event)
{
	updateViewsDynamicObjects();
}

void reactive_navigator_demoframe::OnMenuItemClearRobotPath(wxCommandEvent& event)
{
	gl_robot_path->clear();
	updateViewsDynamicObjects();
}

void reactive_navigator_demoframe::OnbtnLoadMapClick(wxCommandEvent& event)
{
	WX_START_TRY

	wxFileDialog dlg(
		this,
		_("Select grid map to load"),
		_("."),
		_("grid.png"),
		wxT("Image files (*.png,*.jpg,*.gif) or binary gridmap files |*.png;*.jpg;*.gif;*.gridmap;*.gridmap.gz|All files (*.*)|*.*"),
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

void reactive_navigator_demoframe::OnNotebook1PageChanged(wxNotebookEvent& event)
{
}

void reactive_navigator_demoframe::OnNotebook1PageChanged1(wxNotebookEvent& event)
{
}

void reactive_navigator_demoframe::OnedManualKinRampsText(wxCommandEvent& event)
{
}

void reactive_navigator_demoframe::OnbtnQuitClick(wxCommandEvent& event)
{
	Close();
}

void create_viz_robot_holo( mrpt::opengl::CSetOfObjects &objs )
{
	objs.clear();
	{
		mrpt::opengl::CSetOfObjectsPtr gl_xyz = mrpt::opengl::stock_objects::CornerXYZSimple(1.0f, 2.0f);
		gl_xyz->setLocation(0.0,0.0, 1.25);
		objs.insert( gl_xyz );
	}

	mrpt::opengl::CCylinderPtr obj = mrpt::opengl::CCylinder::Create(0.36f /*base radius*/,0.20f /*top radius */,1.2f /*height*/);
	obj->setColor_u8( TColor::red );
	objs.insert( obj );
}
void create_viz_robot_diff( mrpt::opengl::CSetOfObjects &objs )
{
	objs.clear();
	objs.insert( mrpt::opengl::stock_objects::RobotPioneer() );
}

void reactive_navigator_demoframe::OnrbKinTypeSelect(wxCommandEvent& event)
{
	// We must first detach the simulator from the navigator object before deleting it:
	m_navMethod.reset();

	// Delete old & build new simulator:
	m_robotSimul2NavInterface.reset();
	m_robotSimul.reset();

	if (gl_robot_path) gl_robot_path->clear();
	if (gl_target) gl_target->setVisibility(false);

	switch ( rbKinType->GetSelection() )
	{
	case 0:
		{
			mrpt::kinematics::CVehicleSimul_DiffDriven *sim = new mrpt::kinematics::CVehicleSimul_DiffDriven();
			m_robotSimul.reset(sim);
			m_robotSimul2NavInterface.reset( new MyRobot2NavInterface_Diff( *sim, m_latest_obstacles ) );
			// Opengl viz:
			create_viz_robot_diff( *mrpt::opengl::CSetOfObjectsPtr( gl_robot->getByName("robot_render") ) );
			create_viz_robot_diff(*gl_robot_local);
			gl_robot_local->setScale( 1.0 / m_simul_options.MAX_SENSOR_RADIUS );
		}
		break;
	case 1:
		{
			mrpt::kinematics::CVehicleSimul_Holo *sim = new mrpt::kinematics::CVehicleSimul_Holo();
			m_robotSimul.reset(sim);
			m_robotSimul2NavInterface.reset( new MyRobot2NavInterface_Holo( *sim, m_latest_obstacles ) );
			// Opengl viz:
			create_viz_robot_holo(*mrpt::opengl::CSetOfObjectsPtr( gl_robot->getByName("robot_render") ));
			create_viz_robot_holo(*gl_robot_local);
			gl_robot_local->setScale( 1.0 / m_simul_options.MAX_SENSOR_RADIUS );
		}
		break;
	default:
		throw std::runtime_error("Invalid kinematic model selected!");
	};

}


void reactive_navigator_demoframe::OnbtnEmptyMapClick(wxCommandEvent& event)
{
	WX_START_TRY

	double lx= 30.,ly=30.;
	double res = 0.25;
	wxString s;
	s = wxGetTextFromUser( _("Width (x) [meters]:"), _("New map"), _("40.0"), this );
	if (s.IsEmpty()) return;
	if (!s.ToDouble(&lx)) { wxMessageBox(_("Invalid number")); return; }

	s = wxGetTextFromUser( _("Height (y) [meters]:"), _("New map"), _("30.0"), this );
	if (s.IsEmpty()) return;
	if (!s.ToDouble(&ly)) { wxMessageBox(_("Invalid number")); return; }

	s = wxGetTextFromUser( _("Grid resolution [meters]:"), _("New map"), _("0.25"), this );
	if (s.IsEmpty()) return;
	if (!s.ToDouble(&res)) { wxMessageBox(_("Invalid number")); return; }

	m_gridMap.setSize(-.5*lx,.5*lx, -.5*ly, .5*ly, res, 0.99f );

	updateMap3DView();
	m_plot3D->Refresh();

	WX_END_TRY

}

void reactive_navigator_demoframe::OnbtnSaveMapClick(wxCommandEvent& event)
{
	WX_START_TRY

	wxFileDialog dlg(
		this,
		_("Save gridmap to file"),
		_("."),
		_("map.gridmap.gz"),
		wxT("Binary gridmap files (*.gridmap,*.gridmap.gz)|*.gridmap;*.gridmap.gz|All files (*.*)|*.*"),
		wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

	if (dlg.ShowModal() != wxID_OK)
		return;

	CFileGZOutputStream f( std::string(dlg.GetPath().mb_str()) );
	f << m_gridMap;

	WX_END_TRY
}

void reactive_navigator_demoframe::OnbtnDrawMapObsClick(wxCommandEvent& event)
{
	if (m_cursorPickState!=cpsDrawObstacles)
	{
		m_cursorPickState = cpsDrawObstacles;
		m_plot3D->SetCursor( *wxCROSS_CURSOR );
	}
	else
	{	// Cancel:
		m_cursorPickState = cpsNone;
		m_plot3D->SetCursor( *wxSTANDARD_CURSOR );
	}
	btnDrawMapObs->SetValue( m_cursorPickState == cpsDrawObstacles ); btnDrawMapObs->Refresh();
	btnDrawEmpty->SetValue( m_cursorPickState == cpsDrawClear ); btnDrawEmpty->Refresh();
	m_gl_drawing_obs->setVisibility(m_cursorPickState==cpsDrawObstacles || m_cursorPickState==cpsDrawClear);
	m_plot3D->Refresh();
}

void reactive_navigator_demoframe::OnbtnDrawEmptyClick(wxCommandEvent& event)
{
	if (m_cursorPickState!=cpsDrawClear) {
		m_cursorPickState = cpsDrawClear;
		m_plot3D->SetCursor( *wxCROSS_CURSOR );
	}
	else {	// Cancel:
		m_cursorPickState = cpsNone;
		m_plot3D->SetCursor( *wxSTANDARD_CURSOR );
	}
	btnDrawMapObs->SetValue( m_cursorPickState == cpsDrawObstacles ); btnDrawMapObs->Refresh();
	btnDrawEmpty->SetValue( m_cursorPickState == cpsDrawClear ); btnDrawEmpty->Refresh();
	m_gl_drawing_obs->setVisibility(m_cursorPickState==cpsDrawObstacles || m_cursorPickState==cpsDrawClear);
	m_plot3D->Refresh();
}

void reactive_navigator_demoframe::OnbtnSetWaypointSeqClick(wxCommandEvent& event)
{
	CWaypointsNavigator *wp_nav = dynamic_cast<CWaypointsNavigator *>(m_navMethod.get());
	if (!wp_nav)
	{
		wxMessageBox(wxT("Navigator class does not support waypoints sequences!"));
		return;
	}

	if (m_cursorPickState!=cpsPickWaypoints) {
		m_cursorPickState = cpsPickWaypoints;
		m_plot3D->SetCursor( *wxCROSS_CURSOR );
		m_waypoints_clicked.clear();
	}
	else {	// Cancel:
		m_cursorPickState = cpsNone;
		m_plot3D->SetCursor( *wxSTANDARD_CURSOR );
	}
	btnSetWaypointSeq->SetValue( m_cursorPickState == cpsPickWaypoints ); btnSetWaypointSeq->Refresh();
	m_gl_placing_nav_target->setVisibility(m_cursorPickState==cpsPickWaypoints);
	m_plot3D->Refresh();
}
