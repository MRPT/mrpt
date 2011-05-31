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

#include "prrtnavdemoMain.h"
#include "CIniEditor.h"
#include "CAboutBox.h"
#include <wx/msgdlg.h>
#include <wx/filename.h>

#include <mrpt/base.h>


// In milliseconds:
#define SIMULATION_TIME_STEPS   20

//#define ENABLE_reactivenav_LOG_FILES true
#define ENABLE_reactivenav_LOG_FILES false

//(*InternalHeaders(ReactiveNavigationDemoFrame)
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/intl.h>
#include <wx/font.h>
#include <wx/bitmap.h>
#include <wx/icon.h>
#include <wx/image.h>
#include <wx/artprov.h>
//*)

#include <mrpt/gui/WxUtils.h>


#define WX_START_TRY \
    try \
    {


#define WX_END_TRY \
    } \
	catch(std::exception &e) \
    { \
        wxMessageBox( wxString(e.what(),wxConvUTF8), wxT("Exception"), wxOK, this); \
    } \
    catch(...) \
    { \
        wxMessageBox( _("Untyped exception!"), _("Exception"), wxOK, this); \
    }


CIniEditor    *iniEditorNavigator=NULL;
std::string   EDIT_navigatorIni;

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

// General global variables:
#include <mrpt/maps.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace std;



float  ROBOT_MODEL_LOWPASS_CNST = 0.2f;
const float  ROBOT_MODEL_DELAY = 0.0f;

// The obstacles map:
COccupancyGridMap2D		gridMap;
CRobotSimulator			robotSim(ROBOT_MODEL_LOWPASS_CNST,ROBOT_MODEL_DELAY);
TPoint2D				curCursorPos;



//(*IdInit(ReactiveNavigationDemoFrame)
const long ReactiveNavigationDemoFrame::ID_BUTTON1 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_BUTTON2 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_BUTTON6 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_CHECKBOX1 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_STATICTEXT1 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_TEXTCTRL2 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_BUTTON7 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_PANEL2 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_STATICTEXT5 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_STATICTEXT7 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_TEXTCTRL3 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_STATICTEXT8 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_TEXTCTRL4 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_BUTTON4 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_STATICTEXT6 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_STATICTEXT2 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_TEXTCTRL5 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_STATICTEXT3 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_TEXTCTRL6 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_STATICTEXT4 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_TEXTCTRL7 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_BUTTON3 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_STATICTEXT9 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_STATICTEXT10 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_TEXTCTRL8 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_BUTTON5 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_PANEL3 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_PANEL1 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_CUSTOM1 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_TEXTCTRL1 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_STATUSBAR1 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_TIMER1 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_MENUITEM1 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_MENUITEM2 = wxNewId();
//*)

const long ReactiveNavigationDemoFrame::ID_MENUITEM_SET_reactivenav_TARGET = wxNewId();


BEGIN_EVENT_TABLE(ReactiveNavigationDemoFrame,wxFrame)
    //(*EventTable(ReactiveNavigationDemoFrame)
    //*)
END_EVENT_TABLE()


void emul_printf(const char* s)
{
    cout << s;
}

ReactiveNavigationDemoFrame::ReactiveNavigationDemoFrame(wxWindow* parent,wxWindowID id)
{
    // Load my custom icons:
#if wxCHECK_VERSION(2, 8, 0)
    wxArtProvider::Push(new MyArtProvider);
#else
    wxArtProvider::PushProvider(new MyArtProvider);
#endif


    //(*Initialize(ReactiveNavigationDemoFrame)
    wxFlexGridSizer* FlexGridSizer8;
    wxFlexGridSizer* FlexGridSizer1;
    wxFlexGridSizer* FlexGridSizer2;
    wxFlexGridSizer* FlexGridSizer11;
    wxFlexGridSizer* FlexGridSizer7;
    wxFlexGridSizer* FlexGridSizer4;
    wxStaticText* StaticText1;
    wxFlexGridSizer* FlexGridSizer9;
    wxFlexGridSizer* FlexGridSizer14;
    wxFlexGridSizer* FlexGridSizer6;
    wxFlexGridSizer* FlexGridSizer3;
    wxFlexGridSizer* FlexGridSizer10;
    wxFlexGridSizer* FlexGridSizer13;
    wxFlexGridSizer* FlexGridSizer12;
    wxFlexGridSizer* FlexGridSizer5;

    Create(parent, wxID_ANY, _("PTG-based RRT navigator demo - Part of MRPT"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("wxID_ANY"));
    {
    wxIcon FrameIcon;
    FrameIcon.CopyFromBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("MAIN_ICON")),wxART_FRAME_ICON));
    SetIcon(FrameIcon);
    }
    FlexGridSizer1 = new wxFlexGridSizer(3, 1, 0, 0);
    FlexGridSizer1->AddGrowableCol(0);
    FlexGridSizer1->AddGrowableRow(1);
    Panel1 = new wxPanel(this, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    FlexGridSizer2 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer2->AddGrowableCol(1);
    FlexGridSizer3 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer3->AddGrowableCol(2);
    FlexGridSizer8 = new wxFlexGridSizer(0, 2, 0, 0);
    btnStart = new wxButton(Panel1, ID_BUTTON1, _("Simulate"), wxDefaultPosition, wxSize(100,50), 0, wxDefaultValidator, _T("ID_BUTTON1"));
    btnStart->SetDefault();
    wxFont btnStartFont(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxBOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    btnStart->SetFont(btnStartFont);
    FlexGridSizer8->Add(btnStart, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnPause = new wxButton(Panel1, ID_BUTTON2, _("Stop"), wxDefaultPosition, wxSize(100,50), 0, wxDefaultValidator, _T("ID_BUTTON2"));
    btnPause->Disable();
    FlexGridSizer8->Add(btnPause, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnResetSim = new wxButton(Panel1, ID_BUTTON6, _("Reset"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON6"));
    FlexGridSizer8->Add(btnResetSim, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer3->Add(FlexGridSizer8, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    panParams = new wxPanel(Panel1, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
    boxParams = new wxStaticBoxSizer(wxHORIZONTAL, panParams, _("Parameters"));
    FlexGridSizer5 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer9 = new wxFlexGridSizer(3, 2, 0, 0);
    cbExtMap = new wxCheckBox(panParams, ID_CHECKBOX1, _("Default map"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
    cbExtMap->SetValue(true);
    FlexGridSizer9->Add(cbExtMap, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer9->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText1 = new wxStaticText(panParams, ID_STATICTEXT1, _("External map:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    FlexGridSizer9->Add(StaticText1, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    edMapFile = new wxTextCtrl(panParams, ID_TEXTCTRL2, _("./obstacles_map.gridmap"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL2"));
    edMapFile->Disable();
    FlexGridSizer9->Add(edMapFile, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer5->Add(FlexGridSizer9, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    btnEditNavParams = new wxButton(panParams, ID_BUTTON7, _("Change navigator params..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON7"));
    FlexGridSizer5->Add(btnEditNavParams, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    boxParams->Add(FlexGridSizer5, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    panParams->SetSizer(boxParams);
    boxParams->Fit(panParams);
    boxParams->SetSizeHints(panParams);
    FlexGridSizer3->Add(panParams, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer2->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer4 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer4->AddGrowableCol(0);
    FlexGridSizer4->AddGrowableRow(0);
    panCmds = new wxPanel(Panel1, ID_PANEL3, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL3"));
    panCmds->Disable();
    boxCommands = new wxStaticBoxSizer(wxHORIZONTAL, panCmds, _("Commands"));
    FlexGridSizer6 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer10 = new wxFlexGridSizer(0, 5, 0, 0);
    StaticText5 = new wxStaticText(panCmds, ID_STATICTEXT5, _("Navigate to a point (try also right-click on the map):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT5"));
    wxFont StaticText5Font(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxBOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    StaticText5->SetFont(StaticText5Font);
    FlexGridSizer10->Add(StaticText5, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer6->Add(FlexGridSizer10, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer12 = new wxFlexGridSizer(0, 5, 0, 0);
    FlexGridSizer12->AddGrowableCol(4);
    StaticText7 = new wxStaticText(panCmds, ID_STATICTEXT7, _("x="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT7"));
    FlexGridSizer12->Add(StaticText7, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edX = new wxTextCtrl(panCmds, ID_TEXTCTRL3, _("5"), wxDefaultPosition, wxSize(50,-1), 0, wxDefaultValidator, _T("ID_TEXTCTRL3"));
    FlexGridSizer12->Add(edX, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText8 = new wxStaticText(panCmds, ID_STATICTEXT8, _("y="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT8"));
    FlexGridSizer12->Add(StaticText8, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edY = new wxTextCtrl(panCmds, ID_TEXTCTRL4, _("5"), wxDefaultPosition, wxSize(50,-1), 0, wxDefaultValidator, _T("ID_TEXTCTRL4"));
    FlexGridSizer12->Add(edY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnNavigate = new wxButton(panCmds, ID_BUTTON4, _("Send"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON4"));
    FlexGridSizer12->Add(btnNavigate, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer6->Add(FlexGridSizer12, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer11 = new wxFlexGridSizer(0, 3, 0, 0);
    StaticText6 = new wxStaticText(panCmds, ID_STATICTEXT6, _("Navigate to a pose:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT6"));
    wxFont StaticText6Font(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxBOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    StaticText6->SetFont(StaticText6Font);
    FlexGridSizer11->Add(StaticText6, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer6->Add(FlexGridSizer11, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer7 = new wxFlexGridSizer(0, 7, 0, 0);
    StaticText2 = new wxStaticText(panCmds, ID_STATICTEXT2, _("x="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    FlexGridSizer7->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edX2 = new wxTextCtrl(panCmds, ID_TEXTCTRL5, _("5"), wxDefaultPosition, wxSize(50,-1), 0, wxDefaultValidator, _T("ID_TEXTCTRL5"));
    FlexGridSizer7->Add(edX2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText3 = new wxStaticText(panCmds, ID_STATICTEXT3, _("y="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
    FlexGridSizer7->Add(StaticText3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edY2 = new wxTextCtrl(panCmds, ID_TEXTCTRL6, _("5"), wxDefaultPosition, wxSize(50,-1), 0, wxDefaultValidator, _T("ID_TEXTCTRL6"));
    FlexGridSizer7->Add(edY2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText4 = new wxStaticText(panCmds, ID_STATICTEXT4, _("phi="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT4"));
    FlexGridSizer7->Add(StaticText4, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edPhi = new wxTextCtrl(panCmds, ID_TEXTCTRL7, _("0"), wxDefaultPosition, wxSize(50,-1), 0, wxDefaultValidator, _T("ID_TEXTCTRL7"));
    FlexGridSizer7->Add(edPhi, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnNavigatePose = new wxButton(panCmds, ID_BUTTON3, _("Send"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON3"));
    FlexGridSizer7->Add(btnNavigatePose, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer6->Add(FlexGridSizer7, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer13 = new wxFlexGridSizer(0, 3, 0, 0);
    StaticText9 = new wxStaticText(panCmds, ID_STATICTEXT9, _("Follow path (debug):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT9"));
    wxFont StaticText9Font(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxBOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    StaticText9->SetFont(StaticText9Font);
    FlexGridSizer13->Add(StaticText9, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer6->Add(FlexGridSizer13, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer14 = new wxFlexGridSizer(0, 3, 0, 0);
    FlexGridSizer14->AddGrowableCol(1);
    StaticText10 = new wxStaticText(panCmds, ID_STATICTEXT10, _("[x;y;phi]="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT10"));
    FlexGridSizer14->Add(StaticText10, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edPath = new wxTextCtrl(panCmds, ID_TEXTCTRL8, _("[0 1 2 3 4;0 0.2 0.5 0.9 1.2;-1000 -1000 -1000 -1000 -1000]"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL8"));
    FlexGridSizer14->Add(edPath, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnPath = new wxButton(panCmds, ID_BUTTON5, _("Send"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON5"));
    FlexGridSizer14->Add(btnPath, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer6->Add(FlexGridSizer14, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    boxCommands->Add(FlexGridSizer6, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    panCmds->SetSizer(boxCommands);
    boxCommands->Fit(panCmds);
    boxCommands->SetSizeHints(panCmds);
    FlexGridSizer4->Add(panCmds, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer2->Add(FlexGridSizer4, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    Panel1->SetSizer(FlexGridSizer2);
    FlexGridSizer2->Fit(Panel1);
    FlexGridSizer2->SetSizeHints(Panel1);
    FlexGridSizer1->Add(Panel1, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    plot = new mpWindow(this,ID_CUSTOM1,wxPoint(192,240),wxSize(496,346),0);
    FlexGridSizer1->Add(plot, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    edLog = new wxTextCtrl(this, ID_TEXTCTRL1, wxEmptyString, wxDefaultPosition, wxSize(496,166), wxTE_MULTILINE|wxTE_READONLY|wxVSCROLL, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    wxFont edLogFont = wxSystemSettings::GetFont(wxSYS_SYSTEM_FIXED_FONT);
    if ( !edLogFont.Ok() ) edLogFont = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
    edLog->SetFont(edLogFont);
    FlexGridSizer1->Add(edLog, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    SetSizer(FlexGridSizer1);
    StatusBar1 = new wxStatusBar(this, ID_STATUSBAR1, 0, _T("ID_STATUSBAR1"));
    int __wxStatusBarWidths_1[2] = { -10, -30 };
    int __wxStatusBarStyles_1[2] = { wxSB_NORMAL, wxSB_NORMAL };
    StatusBar1->SetFieldsCount(2,__wxStatusBarWidths_1);
    StatusBar1->SetStatusStyles(2,__wxStatusBarStyles_1);
    SetStatusBar(StatusBar1);
    timSimulate.SetOwner(this, ID_TIMER1);
    timSimulate.Start(20, true);
    MenuBar1 = new wxMenuBar();
    Menu1 = new wxMenu();
    MenuItem1 = new wxMenuItem(Menu1, ID_MENUITEM1, _("Exit"), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(MenuItem1);
    MenuBar1->Append(Menu1, _("&Simulation"));
    Menu2 = new wxMenu();
    MenuItem2 = new wxMenuItem(Menu2, ID_MENUITEM2, _("About..."), wxEmptyString, wxITEM_NORMAL);
    Menu2->Append(MenuItem2);
    MenuBar1->Append(Menu2, _("&Help"));
    SetMenuBar(MenuBar1);
    FlexGridSizer1->Fit(this);
    FlexGridSizer1->SetSizeHints(this);
    Center();

    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&ReactiveNavigationDemoFrame::OnbtnStartClick);
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&ReactiveNavigationDemoFrame::OnbtnPauseClick);
    Connect(ID_BUTTON6,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&ReactiveNavigationDemoFrame::OnbtnResetSimClick);
    Connect(ID_CHECKBOX1,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&ReactiveNavigationDemoFrame::OnrbExtMapSelect);
    Connect(ID_BUTTON7,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&ReactiveNavigationDemoFrame::OnbtnEditNavParamsClick);
    Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&ReactiveNavigationDemoFrame::OnbtnNavigateClick);
    Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&ReactiveNavigationDemoFrame::OnbtnNavigatePoseClick);
    Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&ReactiveNavigationDemoFrame::OnbtnPathClick);
    plot->Connect(wxEVT_MOTION,(wxObjectEventFunction)&ReactiveNavigationDemoFrame::OnplotMouseMove,0,this);
    Connect(ID_TIMER1,wxEVT_TIMER,(wxObjectEventFunction)&ReactiveNavigationDemoFrame::OntimSimulateTrigger);
    Connect(ID_MENUITEM1,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&ReactiveNavigationDemoFrame::OnQuit);
    Connect(ID_MENUITEM2,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&ReactiveNavigationDemoFrame::OnAbout);
    //*)

	Connect( ID_MENUITEM_SET_reactivenav_TARGET, wxEVT_COMMAND_MENU_SELECTED, (wxObjectEventFunction) &ReactiveNavigationDemoFrame::OnreactivenavTargetMenu );


    lyGridmap = new mpBitmapLayer();
    plot->AddLayer(lyGridmap);

    plot->AddLayer( new mpScaleX());
    plot->AddLayer( new mpScaleY());

    lyVehicle = new mpPolygon();
    lyTarget  = new mpPolygon();
	lyLaserPoints = new mpPolygon();
	lyPlannedPath = new mpPolygon();

	plot->AddLayer( lyVehicle );
	plot->AddLayer( lyTarget );
	plot->AddLayer( lyLaserPoints );
	plot->AddLayer( lyPlannedPath );

	{
		vector<float> xs(5),ys(5);
		float CS = 0.4f;  // Cross size
		xs[0] = -CS;   ys[0] = -CS;
		xs[1] =  CS;   ys[1] =  CS;
		xs[2] =   0;   ys[2] =   0;
		xs[3] = -CS;   ys[3] =  CS;
		xs[4] =  CS;   ys[4] = -CS;

		lyTarget->setPoints(xs,ys,false);
	}

    lyVehicle->SetPen( wxPen(wxColour(255,0,0),2) );
    lyTarget->SetPen( wxPen(wxColour(0,0,255),2) );
	lyLaserPoints->SetPen( wxPen(wxColour(255,0,0),5) );
	lyLaserPoints->SetContinuity(false);
	lyPlannedPath->SetPen( wxPen(wxColour(0,0,255),5) );
	lyPlannedPath->SetContinuity(false);

    plot->LockAspect(true);
    plot->EnableDoubleBuffer(true);

    Maximize();

	SetTitle(wxT("PTG-based RRT navigator demo - Part of the MRPT project") );

	wxMenu *popupMnu = plot->GetPopupMenu();

	popupMnu->InsertSeparator(0);

	wxMenuItem *mnuTarget = new wxMenuItem(popupMnu, ID_MENUITEM_SET_reactivenav_TARGET, _("Navigate to this point"), wxEmptyString, wxITEM_NORMAL);
	popupMnu->Insert(0,mnuTarget);


	// Redirect all output to control:
	myRedirector = new CMyRedirector( edLog, false, 10,false, true );  // threadSafe=true

	// Create dialogs:
	iniEditorNavigator = new CIniEditor(this);
	EDIT_navigatorIni = std::string( iniEditorNavigator->edText->GetValue().mb_str());

	// Try to load the map:
	reloadMap();
	reloadRobotShape();


	// Debug:
	edPath->SetValue(_("[4 1 -1000 2.0 90 2.0; 7 2 -1000 2.0 90 2.0;10 1 -1000 2.0 90 2.0;10 -3 -1000 2.0 90 2.0;7 -3 -1000 2.0 90 0.2;4 -1 -1000 2.0 90 2.0;0 -1 -1000 2.0 90 2.0;0 0 -1000 2.0 90 0.0]"));

	// Set simulator params:
    plot->Fit();

	panCmds->Enable(true);
	panParams->Enable(true);
	panCmds->Enable(false);
}

ReactiveNavigationDemoFrame::~ReactiveNavigationDemoFrame()
{
    //(*Destroy(ReactiveNavigationDemoFrame)
    //*)
	if (myRedirector) {
		delete myRedirector;
		myRedirector = NULL;
	}
	delete iniEditorNavigator;
}

void ReactiveNavigationDemoFrame::OnQuit(wxCommandEvent& event)
{
    Close();
}

void ReactiveNavigationDemoFrame::OnAbout(wxCommandEvent& event)
{
	CAboutBox dlg(this);
	dlg.ShowModal();
}

void ReactiveNavigationDemoFrame::OnbtnStartClick(wxCommandEvent& event)
{
	if (!reloadMap()) return;

	initializeNavigator();

	// Enable buttons:
	btnPause->Enable(true);
	btnStart->Enable(false);
	btnResetSim->Enable(false);

	panParams->Enable(false);
	panCmds->Enable(true);


	// Set timer to start simulation:
    timSimulate.Start(SIMULATION_TIME_STEPS, false); // No One-shot
}

void ReactiveNavigationDemoFrame::OnbtnPauseClick(wxCommandEvent& event)
{
	timSimulate.Stop();

	// Enable buttons:
	btnPause->Enable(false);
	btnStart->Enable(true);
	btnResetSim->Enable(true);

	panParams->Enable(true);
	panCmds->Enable(false);

}

void ReactiveNavigationDemoFrame::OnbtnExitClick(wxCommandEvent& event)
{
    Close();
}

void ReactiveNavigationDemoFrame::initializeNavigator()
{
	try
	{
		CConfigFileMemory	iniNavigator( EDIT_navigatorIni );


		// Load config in the navigator:
		navigator.params.loadFromConfigFile( iniNavigator,"prrt-navigator");
		navigator.initialize();


		// load robot shape:
		reloadRobotShape();

		// Odometry params:
		robotSim.setOdometryErrors(true);

		robotSim.setDelayModelParams(ROBOT_MODEL_LOWPASS_CNST,ROBOT_MODEL_DELAY);

		robotSim.movementCommand(0.1,0.1);
	}
	catch(std::exception &e)
	{
		cout << e.what(); // Redirected to the log control
		cerr << e.what();
	}
}

void ReactiveNavigationDemoFrame::OnbtnNavigateClick(wxCommandEvent& event)
{
	// Send navigation command to navigator:
	wxString  strX = edX->GetValue();
	wxString  strY = edY->GetValue();
	double   x, y;
	if (!strX.ToDouble( &x )) {  wxMessageBox( _("'x' is not a valid number"), wxT("Error"), wxOK, this); return; }
	if (!strY.ToDouble( &y )) {  wxMessageBox( _("'y' is not a valid number"), wxT("Error"), wxOK, this); return; }

	lyTarget->SetCoordinateBase( x,y );
	plot->Refresh();

	// Send command:
	navigator.navigate( TPoint2D(x,y) );
}


bool ReactiveNavigationDemoFrame::reloadMap()
{
    try
	{
		// Load internal or external map?
		if (cbExtMap->GetValue())
		{
			// Internal:
			CMemoryStream  s( DEFAULT_GRIDMAP_DATA, sizeof(DEFAULT_GRIDMAP_DATA) );
			s >> gridMap;
		}
		else
		{
			// External
			string filName = string(edMapFile->GetValue().mb_str());
			if ( !mrpt::system::fileExists( filName ) )
			{
				wxMessageBox( _U( format("Grid map file '%s'  cannot be found.\nSimulation cannot start until a valid map is provided",filName.c_str()).c_str() ) , wxT("Error"), wxOK, this);
				return false;
			}

			CFileGZInputStream  f( filName  );
			f >> gridMap;
		}

		// Set the map image:
		CImage imgGrid;
		gridMap.getAsImage(imgGrid);
		wxImage *newBmp = mrpt::gui::MRPTImage2wxImage( imgGrid );
		double lx = gridMap.getXMax()-gridMap.getXMin();
		double ly = gridMap.getYMax()-gridMap.getYMin();
		lyGridmap->SetBitmap(
			*newBmp,
			gridMap.getXMin(),
			gridMap.getYMin(),
			lx,
			ly );
		delete newBmp;


		// Refresh display:
		plot->Refresh();

		return true;
    }
	catch(std::exception &e)
    {
        wxMessageBox( _U( e.what() ), _("Exception"), wxOK, this);
		return false;
    }
    catch(...)
    {
        wxMessageBox( _("Untyped exception!"), _("Exception"), wxOK, this);
		return false;
    }
}

void ReactiveNavigationDemoFrame::OnplotMouseMove(wxMouseEvent& event)
{
	int X, Y;
	event.GetPosition(&X,&Y);
    curCursorPos.x = plot->p2x(X);
    curCursorPos.y = plot->p2y(Y);

	StatusBar1->SetStatusText(_U(format("X=%.03f Y=%.04f",curCursorPos.x,curCursorPos.y).c_str()), 0);

    event.Skip();
}


void ReactiveNavigationDemoFrame::OnreactivenavTargetMenu(wxCommandEvent& event)
{
	edX->SetValue( _U( format("%.03f", curCursorPos.x ).c_str() ) );
	edY->SetValue( _U( format("%.03f", curCursorPos.y ).c_str() ) );
	edX2->SetValue( edX->GetValue() );
	edY2->SetValue( edY->GetValue() );

	OnbtnNavigateClick( event );
}

void ReactiveNavigationDemoFrame::OntimSimulateTrigger(wxTimerEvent& event)
{
	WX_START_TRY

	static bool IamIN = false;
	if (IamIN) return;
	IamIN=true;


	if ( btnStart->IsEnabled() )
	{
		wxCommandEvent dummy;
		OnbtnPauseClick( dummy );
		IamIN=false;
		return; // We are paused!
	}

	// Navigation end?
#if 0
	if (reacNavObj->getCurrentState() != CAbstractReactiveNavigationSystem::NAVIGATING )
	{
		cout << "NAVIGATION FINISHED - Select a new target and press 'Start' again." << endl;
		wxCommandEvent dummy;
		OnbtnPauseClick( dummy );
		IamIN=false;
		return;
	}

	// Empty log?
	if (edLog->GetNumberOfLines()>300)
	{
		edLog->Clear();
	}
#endif


	// Simulate one time interval for the robot & display it:
	// ----------------------------------------------------------
	robotSim.simulateInterval( 0.001 * SIMULATION_TIME_STEPS );

	lyVehicle->SetCoordinateBase(
		robotSim.getX(),
		robotSim.getY(),
		robotSim.getPHI() );

	// Send new "localization" data to the navigator:
	static int cntLocaliz = 0;
	if (++cntLocaliz==20)
	{
		cntLocaliz = 0;

		TPose2D  robotLocaliz( robotSim.getX(), robotSim.getY(), robotSim.getPHI() );
		CMatrixDouble33 robotCov;
		robotCov(0,0) = robotCov(1,1) = square(0.03);
		robotCov(2,2) = square(DEG2RAD(1));

		navigator.processNewLocalization(robotLocaliz,robotCov, now() );
	}

	// Send odometry data to the navigator:
	{
		TPose2D odo;
		robotSim.getOdometry(odo);
		float v = robotSim.getV();
		float w = robotSim.getW();
		navigator.processNewOdometryInfo( odo,now(),true,v,w);
	}



	// Simulate & display laser scan:
	// ------------------------------------------------
	static int cntLaserDecimate = 0;
	if (++cntLaserDecimate>=4)
	{
		cntLaserDecimate = 0;

		CPose2D  robotPose;
		robotSim.getRealPose(robotPose);

		CObservation2DRangeScan    laserScan;
		laserScan.aperture = DEG2RAD(360.0f);
		laserScan.rightToLeft = true;
		laserScan.maxRange  = 7.0f;
		laserScan.stdError  = 0.003f;

		gridMap.laserScanSimulator(
			laserScan,
			robotPose,
			0.5f, // grid cell threshold
			361,  // Number of rays
			laserScan.stdError // Noise std
			);

		// Build the points map:
		vector<float> xs,ys,zs;
		laserScan.buildAuxPointsMap<CPointsMap>()->getAllPoints(xs,ys,zs);

		lyLaserPoints->setPoints(xs,ys);

		// Send data to the navigator -------------------
		navigator.processNewObstaclesData(laserScan.buildAuxPointsMap<CPointsMap>(), now() );
	}

	lyLaserPoints->SetCoordinateBase(
		robotSim.getX(),
		robotSim.getY(),
		robotSim.getPHI() );


	// Draw the projected planned path:
	// ------------------------------------------------
	static int cntPlannedPath = 0;
	if (++cntPlannedPath>10)
	{
		cntPlannedPath=0;

		REAC::CPRRTNavigator::TPlannedPath path;
		navigator.getCurrentPlannedPath( path );

		vector<float> xs,ys;
		for (REAC::CPRRTNavigator::TPlannedPath::const_iterator i=path.begin();i!=path.end();i++)
		{
			// Draw a small arrow in the robot direction:
			if ( REAC::CPRRTNavigator::INVALID_HEADING == i->p.phi )
			{	// Any heading:
				xs.push_back(i->p.x);
				ys.push_back(i->p.y);
			}
			else
			{
				const double cc = cos(i->p.phi);
				const double ss = sin(i->p.phi);
				for (double r=0;r<0.10;r+=0.01)
				{
					xs.push_back(i->p.x+r*cc);
					ys.push_back(i->p.y+r*ss);
				}
			}
		}
		lyPlannedPath->setPoints(xs,ys);
	}


	// Update all graphs now:
	plot->Refresh();

	StatusBar1->SetStatusText(
		_U(format("Pose=(%.03f,%.03f,%.02fdeg) (v,w)=(%.03f,%.02f)",
		robotSim.getX(),
		robotSim.getY(),
		RAD2DEG(robotSim.getPHI()),
		robotSim.getV(),
		RAD2DEG(robotSim.getW()) ).c_str() ), 1 );


	// Show log texts:
	if(myRedirector)
	{
		static int nLogCounts = 0;
		if (++nLogCounts>10)
		{
			nLogCounts=0;
			myRedirector->dumpNow();
		}
	}

	wxTheApp->Yield();  // Let the app. process messages

	IamIN=false;

	WX_END_TRY
}

void ReactiveNavigationDemoFrame::reloadRobotShape()
{
	try
	{
		// Get ini-data:
		CConfigFileMemory	iniNavigator( EDIT_navigatorIni ); //string(iniEditorNavigator->edText->GetValue().mb_str()) );

		// Robot shape is a bit special to load:
		const std::string sShape = iniNavigator.read_string("prrt-navigator","robot_shape","",true);
		CMatrixDouble mShape;
		if (!mShape.fromMatlabStringFormat(sShape))
			THROW_EXCEPTION_CUSTOM_MSG1("Error parsing robot_shape matrix: '%s'",sShape.c_str());
		ASSERT_(size(mShape,1)==2)
		ASSERT_(size(mShape,2)>=3)

		vector<double> xs,ys;
		mShape.extractRow(0,xs);
		mShape.extractRow(1,ys);

		lyVehicle->setPoints(xs,ys,true);
	}
	catch( std::exception & e)
	{
		cout << e.what() << endl;
		cerr << e.what() << endl;
	}
}

void ReactiveNavigationDemoFrame::OnbtnResetClick(wxCommandEvent& event)
{
}

void ReactiveNavigationDemoFrame::OnbtnEditNavParamsClick(wxCommandEvent& event)
{
	iniEditorNavigator->Center();
	iniEditorNavigator->edText->SetValue(_U( EDIT_navigatorIni.c_str() ));

    if (iniEditorNavigator->ShowModal())
	{
		EDIT_navigatorIni = std::string( iniEditorNavigator->edText->GetValue().mb_str());
	}
}

void ReactiveNavigationDemoFrame::OnrbExtMapSelect(wxCommandEvent& event)
{
    edMapFile->Enable( ! cbExtMap->GetValue() );
}

// Accept commands from the navigator:
bool CMyNavigator::onMotionCommand(float v, float w )
{
	robotSim.movementCommand(v,w);
	return true;
}


void ReactiveNavigationDemoFrame::OnbtnNavigatePoseClick(wxCommandEvent& event)
{
	// Send navigation command to navigator:
	wxString  strX   = edX2->GetValue();
	wxString  strY   = edY2->GetValue();
	wxString  strPhi = edPhi->GetValue();
	double   x, y, phi;
	if (!strX.ToDouble( &x )) {  wxMessageBox( _("'x' is not a valid number"), wxT("Error"), wxOK, this); return; }
	if (!strY.ToDouble( &y )) {  wxMessageBox( _("'y' is not a valid number"), wxT("Error"), wxOK, this); return; }
	if (!strPhi.ToDouble( &phi )) {  wxMessageBox( _("'phi' is not a valid number"), wxT("Error"), wxOK, this); return; }

	lyTarget->SetCoordinateBase( x,y );
	plot->Refresh();

	// Send command:
	navigator.navigate( TPose2D(x,y,wrapToPi(DEG2RAD(phi)) ));
}

void ReactiveNavigationDemoFrame::OnbtnPathClick(wxCommandEvent& event)
{
	const std::string path =  std::string(edPath->GetValue().mb_str());
	CMatrixDouble M;
	if (!M.fromMatlabStringFormat(path) || size(M,1)==0 || size(M,2)!=6 )
	{
		::wxMessageBox(_("Malformed path: It must be a Nx6 matrix in MATLAB format"),_("Error"));
		return;
	}

	// Recover path:
	REAC::CPRRTNavigator::TPlannedPath p;
	for (size_t i=0;i<size(M,1);i++)
	{
		REAC::CPRRTNavigator::TPathData point;
		point.p.x = M(i,0);
		point.p.y = M(i,1);
		point.p.phi = M(i,2)==-1000 ? -10 : DEG2RAD( M(i,2) );

		point.max_v = M(i,3);
		point.max_w = DEG2RAD( M(i,4) );

		point.trg_v = M(i,5);

		p.push_back(point);
	}
	navigator.setPathToFollow( p );
}

void ReactiveNavigationDemoFrame::OnbtnResetSimClick(wxCommandEvent& event)
{
	robotSim.resetStatus();

	lyVehicle->SetCoordinateBase(
		robotSim.getX(),
		robotSim.getY(),
		robotSim.getPHI() );

	plot->Refresh();
}

