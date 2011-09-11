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

#include "ReactiveNavigationDemoMain.h"
#include "CIniEditor.h"
#include <wx/msgdlg.h>
#include <wx/filename.h>

// In milliseconds:
#define SIMULATION_TIME_STEPS   100

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



// The default configuration strings:
std::string EDIT_internalCfgReactive;
std::string EDIT_internalCfgRobot;

CIniEditor    *iniEditorRobot=NULL, *iniEditoreactivenav=NULL;

#include "imgs/main_icon.xpm"
#include "../wx-common/mrpt_logo.xpm"

#include "DEFAULT_GRIDMAP_DATA.h"

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
#include <mrpt/base.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;

//wxImage * auxMRPTImage2wxImage( const CImage &img );


#include <mrpt/reactivenav/CReactiveNavigationSystem.h>
using namespace mrpt::reactivenav;

CReactiveNavigationSystem		*reacNavObj=NULL;

// The obstacles map:
COccupancyGridMap2D		gridMap;
CRobotSimulator			robotSim(1e-9f,0);
TPoint2D				curCursorPos;

class CMyReactInterface : public CReactiveInterfaceImplementation
{
public:
	bool getCurrentPoseAndSpeeds( mrpt::poses::CPose2D &curPose, float &curV, float &curW)
	{
		robotSim.getRealPose( curPose );
		curV = robotSim.getV();
		curW = robotSim.getW();
		return true;
	}

	bool changeSpeeds( float v, float w )
	{
		robotSim.movementCommand(v,w);
		return true;
	}

	bool senseObstacles( mrpt::slam::CSimplePointsMap 		&obstacles )
	{
		CPose2D  robotPose;

		robotSim.getRealPose(robotPose);

		CObservation2DRangeScan    laserScan;
		laserScan.aperture = M_2PIf;
		laserScan.rightToLeft = true;
		laserScan.maxRange  = 7.0f;
		laserScan.stdError  = 0.003f;


		gridMap.laserScanSimulator(
			laserScan,
			robotPose,
			0.5f, // grid cell threshold
			361,  // Number of rays
			0     // Noise std
			);

		// Build the points map:
		obstacles.insertionOptions.minDistBetweenLaserPoints = 0.005f;
		obstacles.insertionOptions.also_interpolate = false;

		obstacles.clear();
		obstacles.insertObservation( &laserScan );

		// Draw points:
		vector<float> xs,ys,zs;
		obstacles.getAllPoints(xs,ys,zs);

		the_frame->lyLaserPoints->setPoints(xs,ys);
		the_frame->lyLaserPoints->SetCoordinateBase(
			robotSim.getX(),
			robotSim.getY(),
			robotSim.getPHI() );

		return true;
	}

	void notifyHeadingDirection(const double heading_dir_angle)
	{

	}
	
	ReactiveNavigationDemoFrame *the_frame;
};

CMyReactInterface  myReactiveInterface;

//(*IdInit(ReactiveNavigationDemoFrame)
const long ReactiveNavigationDemoFrame::ID_BUTTON1 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_BUTTON2 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_CHECKBOX3 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_BUTTON3 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_CHECKBOX1 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_STATICTEXT1 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_TEXTCTRL2 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_BUTTON6 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_BUTTON7 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_CHECKBOX2 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_STATICTEXT5 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_TEXTCTRL5 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_STATICTEXT6 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_TEXTCTRL6 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_STATICTEXT2 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_TEXTCTRL3 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_STATICTEXT4 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_STATICTEXT3 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_TEXTCTRL4 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_BUTTON4 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_PANEL1 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_CUSTOM1 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_TEXTCTRL1 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_STATUSBAR1 = wxNewId();
const long ReactiveNavigationDemoFrame::ID_TIMER1 = wxNewId();
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
    wxStaticBoxSizer* StaticBoxSizer2;
    wxFlexGridSizer* FlexGridSizer8;
    wxFlexGridSizer* FlexGridSizer1;
    wxFlexGridSizer* FlexGridSizer2;
    wxFlexGridSizer* FlexGridSizer7;
    wxFlexGridSizer* FlexGridSizer4;
    wxStaticText* StaticText1;
    wxStaticBoxSizer* StaticBoxSizer3;
    wxFlexGridSizer* FlexGridSizer6;
    wxFlexGridSizer* FlexGridSizer3;
    wxFlexGridSizer* FlexGridSizer5;
    wxStaticBoxSizer* StaticBoxSizer1;
    
    Create(parent, id, _("Reactive Navigation Demo - Part of the MRPT project - J.L. Blanco (C) 2005-2008"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("id"));
    {
    wxIcon FrameIcon;
    FrameIcon.CopyFromBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("MAIN_ICON")),wxART_FRAME_ICON));
    SetIcon(FrameIcon);
    }
    FlexGridSizer1 = new wxFlexGridSizer(3, 1, 0, 0);
    FlexGridSizer1->AddGrowableCol(0);
    FlexGridSizer1->AddGrowableRow(1);
    Panel1 = new wxPanel(this, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    FlexGridSizer2 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer2->AddGrowableCol(0);
    FlexGridSizer3 = new wxFlexGridSizer(0, 5, 0, 0);
    FlexGridSizer3->AddGrowableCol(3);
    btnStart = new wxButton(Panel1, ID_BUTTON1, _("Simulate"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
    FlexGridSizer3->Add(btnStart, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnPause = new wxButton(Panel1, ID_BUTTON2, _("Pause"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
    btnPause->Disable();
    FlexGridSizer3->Add(btnPause, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbLog = new wxCheckBox(Panel1, ID_CHECKBOX3, _("Generate navigation log file"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX3"));
    cbLog->SetValue(false);
    FlexGridSizer3->Add(cbLog, 1, wxALL|wxALIGN_LEFT|wxALIGN_BOTTOM, 5);
    FlexGridSizer3->Add(-1,-1,1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnExit = new wxButton(Panel1, ID_BUTTON3, _("EXIT"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON3"));
    FlexGridSizer3->Add(btnExit, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer2->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer4 = new wxFlexGridSizer(0, 3, 0, 0);
    FlexGridSizer4->AddGrowableCol(0);
    FlexGridSizer4->AddGrowableCol(1);
    StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, Panel1, _("Obstacle grid map "));
    FlexGridSizer5 = new wxFlexGridSizer(3, 1, 0, 0);
    cbExtMap = new wxCheckBox(Panel1, ID_CHECKBOX1, _("Use internal default map"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
    cbExtMap->SetValue(true);
    FlexGridSizer5->Add(cbExtMap, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    StaticText1 = new wxStaticText(Panel1, ID_STATICTEXT1, _("External map file:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    FlexGridSizer5->Add(StaticText1, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    edMapFile = new wxTextCtrl(Panel1, ID_TEXTCTRL2, _("./obstacles_map.gridmap"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL2"));
    edMapFile->Disable();
    FlexGridSizer5->Add(edMapFile, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer1->Add(FlexGridSizer5, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer4->Add(StaticBoxSizer1, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer3 = new wxStaticBoxSizer(wxVERTICAL, Panel1, _("Navigation parameters"));
    FlexGridSizer7 = new wxFlexGridSizer(0, 2, 0, 0);
    btnEditRobotParams = new wxButton(Panel1, ID_BUTTON6, _("Edit robot parameters..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON6"));
    FlexGridSizer7->Add(btnEditRobotParams, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnEditNavParams = new wxButton(Panel1, ID_BUTTON7, _("Edit navig. parameters..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON7"));
    FlexGridSizer7->Add(btnEditNavParams, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbInternalParams = new wxCheckBox(Panel1, ID_CHECKBOX2, _("Use external config files:"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX2"));
    cbInternalParams->SetValue(false);
    FlexGridSizer7->Add(cbInternalParams, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer7->Add(-1,-1,1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer3->Add(FlexGridSizer7, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer8 = new wxFlexGridSizer(2, 2, 0, 0);
    FlexGridSizer8->AddGrowableCol(1);
    StaticText5 = new wxStaticText(Panel1, ID_STATICTEXT5, _("Robot parameters:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT5"));
    FlexGridSizer8->Add(StaticText5, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edRobotCfgFile = new wxTextCtrl(Panel1, ID_TEXTCTRL5, _("./CONFIG_RobotDescription.ini"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL5"));
    edRobotCfgFile->Disable();
    FlexGridSizer8->Add(edRobotCfgFile, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText6 = new wxStaticText(Panel1, ID_STATICTEXT6, _("Navigation parameters:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT6"));
    FlexGridSizer8->Add(StaticText6, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edNavCfgFile = new wxTextCtrl(Panel1, ID_TEXTCTRL6, _("./CONFIG_ReactiveNavigator.ini"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL6"));
    edNavCfgFile->Disable();
    FlexGridSizer8->Add(edNavCfgFile, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer3->Add(FlexGridSizer8, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer4->Add(StaticBoxSizer3, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer2 = new wxStaticBoxSizer(wxHORIZONTAL, Panel1, _("Navigation target:"));
    FlexGridSizer6 = new wxFlexGridSizer(2, 3, 0, 0);
    StaticText2 = new wxStaticText(Panel1, ID_STATICTEXT2, _("x="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    FlexGridSizer6->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edX = new wxTextCtrl(Panel1, ID_TEXTCTRL3, _("5"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL3"));
    FlexGridSizer6->Add(edX, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText4 = new wxStaticText(Panel1, ID_STATICTEXT4, _("(Right click on map for an\n easier way of entering commands)"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT4"));
    FlexGridSizer6->Add(StaticText4, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText3 = new wxStaticText(Panel1, ID_STATICTEXT3, _("y="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
    FlexGridSizer6->Add(StaticText3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edY = new wxTextCtrl(Panel1, ID_TEXTCTRL4, _("5"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL4"));
    FlexGridSizer6->Add(edY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnNavigate = new wxButton(Panel1, ID_BUTTON4, _("SET TARGET"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON4"));
    btnNavigate->SetDefault();
    FlexGridSizer6->Add(btnNavigate, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer2->Add(FlexGridSizer6, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer4->Add(StaticBoxSizer2, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer2->Add(FlexGridSizer4, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    Panel1->SetSizer(FlexGridSizer2);
    FlexGridSizer2->Fit(Panel1);
    FlexGridSizer2->SetSizeHints(Panel1);
    FlexGridSizer1->Add(Panel1, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    plot = new mpWindow(this,ID_CUSTOM1,wxPoint(192,240),wxSize(496,346),0);
    FlexGridSizer1->Add(plot, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    edLog = new wxTextCtrl(this, ID_TEXTCTRL1, wxEmptyString, wxDefaultPosition, wxSize(496,166), wxTE_MULTILINE|wxTE_READONLY|wxVSCROLL, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    wxFont edLogFont = wxSystemSettings::GetFont(wxSYS_OEM_FIXED_FONT);
    if ( !edLogFont.Ok() ) edLogFont = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
    edLogFont.SetPointSize((int)(edLogFont.GetPointSize() * 1.000000));
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
    FlexGridSizer1->Fit(this);
    FlexGridSizer1->SetSizeHints(this);
    Center();
    
    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&ReactiveNavigationDemoFrame::OnbtnStartClick);
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&ReactiveNavigationDemoFrame::OnbtnPauseClick);
    Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&ReactiveNavigationDemoFrame::OnbtnExitClick);
    Connect(ID_CHECKBOX1,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&ReactiveNavigationDemoFrame::OnrbExtMapSelect);
    Connect(ID_BUTTON6,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&ReactiveNavigationDemoFrame::OnbtnEditRobotParamsClick);
    Connect(ID_BUTTON7,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&ReactiveNavigationDemoFrame::OnbtnEditNavParamsClick);
    Connect(ID_CHECKBOX2,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&ReactiveNavigationDemoFrame::OncbInternalParamsClick);
    Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&ReactiveNavigationDemoFrame::OnbtnNavigateClick);
    plot->Connect(wxEVT_MOTION,(wxObjectEventFunction)&ReactiveNavigationDemoFrame::OnplotMouseMove,0,this);
    Connect(ID_TIMER1,wxEVT_TIMER,(wxObjectEventFunction)&ReactiveNavigationDemoFrame::OntimSimulateTrigger);
    //*)

	Connect( ID_MENUITEM_SET_reactivenav_TARGET, wxEVT_COMMAND_MENU_SELECTED, (wxObjectEventFunction) &ReactiveNavigationDemoFrame::OnreactivenavTargetMenu );


    lyGridmap = new mpBitmapLayer();
    plot->AddLayer(lyGridmap);

    plot->AddLayer( new mpScaleX());
    plot->AddLayer( new mpScaleY());

    lyVehicle = new mpPolygon();
    lyTarget  = new mpPolygon();
    lyLaserPoints = new mpPolygon();

	plot->AddLayer( lyLaserPoints );
	plot->AddLayer( lyVehicle );
	plot->AddLayer( lyTarget );

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

    plot->LockAspect(true);
    plot->EnableDoubleBuffer(true);

    Maximize();



	wxMenu *popupMnu = plot->GetPopupMenu();

	popupMnu->InsertSeparator(0);

	wxMenuItem *mnuTarget = new wxMenuItem(popupMnu, ID_MENUITEM_SET_reactivenav_TARGET, _("Navigate to this point"), wxEmptyString, wxITEM_NORMAL);
	popupMnu->Insert(0,mnuTarget);


	// Redirect all output to control:
	myRedirector = new CMyRedirector( edLog, true, 10 );

	// Create dialogs:
	iniEditorRobot = new CIniEditor(this);
	iniEditoreactivenav = new CIniEditor(this);

	iniEditorRobot->edText->SetValue( wxT("; ---------------------------------------------------------------\n; FILE: CONFIG_RobotDescription.ini\n;\n;  All robot dependant modules should look here to determine\n;   the actual running platform. The posibilities are:\n;\t-> \"Sancho\"\n;\t-> \"Sena\"\n;  Some physical description parameters are stored for each \n;    robot.\n;\n;  JLBC @ OCT-2005\n; ---------------------------------------------------------------\n[ROBOT_NAME]\nName=SYMCAR\n\n; -------------------------------\n;\t       SYMCAR\n; -------------------------------\n[SYMCAR]\nPLS_Pose_x=0 ; Laser range scaner 3D position in the robot\nPLS_Pose_y=0\nPLS_Pose_z=0.31\nPLS_Pose_yaw=0\t\t; Angles in degrees\nPLS_Pose_pitch=0\nPLS_Pose_roll=0\n\n") );

	// VC2003 complains about the too big string:
	wxString	auxStr = wxT("; ---------------------------------------------------------------\n; FILE: CONFIG_ReactiveNavigator.ini\n;\n;  In this file there are parameters to the reactive navigation\n;   module.\n;\n;  JLBC @ OCT-2005\n; ---------------------------------------------------------------\n\n\n[GLOBAL_CONFIG]\n; 0: VFF,  1: ND\nHOLONOMIC_METHOD=1\nALARM_SEEMS_NOT_APPROACHING_TARGET_TIMEOUT=100\n\n; ----------------------------------------------------\n;\tParameters for the \"Nearness diagram\" Holonomic method\n; ----------------------------------------------------\n[ND_CONFIG]\nfactorWeights=1.0 0.5 2.0 0.4\n; 1: Free space\n; 2: Dist. in sectors\n; 3: Closer to target (euclidean)\n; 4: Hysteresis\nWIDE_GAP_SIZE_PERCENT=0.50\nMAX_SECTOR_DIST_FOR_D2_PERCENT=0.25\n");
	auxStr << wxT("RISK_EVALUATION_SECTORS_PERCENT=0.25\nRISK_EVALUATION_DISTANCE=0.15\t\t; In normalized ps-meters [0,1]\nTARGET_SLOW_APPROACHING_DISTANCE=1.00\t; For stop gradually\nTOO_CLOSE_OBSTACLE=0.02\t\t\t; In normalized ps-meters\n\n\n; ----------------------------------------------------\n;\tParameters for the navigation on ROBOT: \"SYM_CAR\"\n; ----------------------------------------------------\n[SYMCAR]\nweights=0.5 0.05 0.5 2.0 0.5 0.1\n; 1: Free space\n; 2: Dist. in sectors\t\t\t\n; 3: Heading toward target\n; 4: Closer to target (euclidean)\n; 5: Hysteresis\n; 6: Security Distance\n\nDIST_TO_TARGET_FOR_SENDING_EVENT=1.25\t; Minimum. distance to target for sending the end event. Set to 0 to send it just on navigation end\n\nMinObstaclesHeight=0.0 \t\t; Minimum coordinate in the \"z\" axis for an obstacle to be taken into account.\nMaxObstaclesHeight=1.40 \t\t; Maximum coordinate in the \"z\" axis for an obstacle to be taken into account.\n\nrobotMax_V_mps=0.70\t\t\t; Speed limits\nrobotMax_W_degps=50\nROBOTMODEL_DELAY=0\t\t\t; The delay until motor reaction\nROBOTMODEL_TAU=0\t\t\t; The \"TAU\" time constant of a first order lowpass filter\n\nMAX_REFERENCE_DISTANCE=3.50\nWIDE_GAP_SIZE_PERCENT=0.40\nRISK_EVALUATION_DISTANCE=0.5\nRISK_EVALUATION_SECTORS_PERCENT=0.20\nMAX_SECTOR_DIST_FOR_D2_PERCENT=0.25\nRESOLUCION_REJILLA_X=0.03\nRESOLUCION_REJILLA_Y=0.03\n\nPTG_COUNT=4\n\n; \t\tC-PTGs:\n; ------------------------------------\nPTG0_Type=1\nPTG0_nAlfas=300\nPTG0_v_max_mps=0.7\nPTG0_w_max_gps=50\nPTG0_K=1.0\n\nPTG1_Type=1\nPTG1_nAlfas=300\nPTG1_v_max_mps=0.7\nPTG1_w_max_gps=20\nPTG1_K=1.0\n\nPTG3_Type=1\nPTG3_nAlfas=300\nPTG3_v_max_mps=0.7\nPTG3_w_max_gps=40\nPTG3_K=-1.0\n\n\n; \t     a-A type PTGs:\n; ------------------------------------\nPTG2_Type=2\nPTG2_nAlfas=300\nPTG2_v_max_mps=0.7\nPTG2_w_max_gps=50\nPTG2_cte_a0v_deg=10\nPTG2_cte_a0w_deg=40\n\n\nRobotModel_shape2D_xs=-0.2 0.5 0.5 -0.2\nRobotModel_shape2D_ys=0.3 0.3 -0.3 -0.3\n");
	iniEditoreactivenav->edText->SetValue( auxStr );

	EDIT_internalCfgRobot = string( iniEditorRobot->edText->GetValue().mb_str() );
	EDIT_internalCfgReactive = string( iniEditoreactivenav->edText->GetValue().mb_str() );


	// Try to load the map:
	reloadMap();
	reloadRobotShape();

	// Set simulator params:
    plot->Fit();
}

ReactiveNavigationDemoFrame::~ReactiveNavigationDemoFrame()
{
    //(*Destroy(ReactiveNavigationDemoFrame)
    //*)

    delete reacNavObj; reacNavObj = NULL;
	delete myRedirector; myRedirector = NULL;

	delete iniEditorRobot;
	delete iniEditoreactivenav;
}

void ReactiveNavigationDemoFrame::OnQuit(wxCommandEvent& event)
{
    Close();
}

void ReactiveNavigationDemoFrame::OnAbout(wxCommandEvent& event)
{
}

void ReactiveNavigationDemoFrame::OnbtnStartClick(wxCommandEvent& event)
{
	if (!reloadMap()) return;

	tryConstructReactiveNavigator();

	if (!reacNavObj)
	{
		wxMessageBox( _("Could not create the reactive navigation object! Check the log output for error messages."), wxT("Error"), wxOK, this);
		return;
	}


	// Enable buttons:
	btnPause->Enable(true);
	btnStart->Enable(false);


	// Set timer to start simulation:
    timSimulate.Start(SIMULATION_TIME_STEPS, false); // No One-shot
}

void ReactiveNavigationDemoFrame::OnbtnPauseClick(wxCommandEvent& event)
{
	// Enable buttons:
	btnPause->Enable(false);
	btnStart->Enable(true);

}

void ReactiveNavigationDemoFrame::OnbtnExitClick(wxCommandEvent& event)
{
    Close();
}

// both objects must be DELETED by the caller!
void createConfigSources(
	ReactiveNavigationDemoFrame *frame,
	CConfigFileBase    *& iniReactive,
	CConfigFileBase    *& configRobotIni )
{
	// Set config files/internal data:
	if ( frame->cbInternalParams->GetValue() )
	{
		// Use external files:
		string filReac( frame->edNavCfgFile->GetValue().mb_str()  );
		string filRobot( frame->edRobotCfgFile->GetValue().mb_str() );

		if ( !mrpt::system::fileExists( filReac ))
		{
			wxMessageBox( _U( format("Cannot open file : '%s'",filReac.c_str()).c_str()), wxT("Error"), wxOK, frame);
			return;
		}
		if ( !mrpt::system::fileExists( filRobot ))
		{
			wxMessageBox( _U( format("Cannot open file : '%s'",filRobot.c_str()).c_str()), wxT("Error"), wxOK, frame);
			return;
		}

		iniReactive = new CConfigFile( filReac );
		configRobotIni = new CConfigFile( filRobot );
	}
	else
	{
		// Use internal strings:
		iniReactive = new CConfigFileMemory( EDIT_internalCfgReactive );
		configRobotIni = new CConfigFileMemory( EDIT_internalCfgRobot );
	}
}

void ReactiveNavigationDemoFrame::tryConstructReactiveNavigator()
{
	try
	{
		myReactiveInterface.the_frame = this;

		CConfigFileBase    *iniReactive=NULL, *configRobotIni=NULL;

		// Get ini-data:
		createConfigSources(this, iniReactive, configRobotIni );
		if (!iniReactive || !configRobotIni) return;

		if (!reacNavObj)
		{
			const bool ENABLE_reactivenav_LOG_FILES = cbLog->GetValue();

			// Create reactive nav. object:
			reacNavObj = new CReactiveNavigationSystem(
				myReactiveInterface,
				true,
				ENABLE_reactivenav_LOG_FILES );
		}

		// Reload config:
		reacNavObj->loadConfigFile(
			*iniReactive,
			*configRobotIni );

		delete iniReactive;
		delete configRobotIni;

		// load robot shape:
		reloadRobotShape();

	}
	catch(std::exception &e)
	{
		cout << e.what(); // Redirected to the log control
		cerr << e.what();
	}
}

void ReactiveNavigationDemoFrame::OnbtnNavigateClick(wxCommandEvent& event)
{

	if (!reacNavObj)
	{
		// Initialized ok?
		tryConstructReactiveNavigator();

		if (!reacNavObj)
		{
			wxMessageBox( _("Could not create the reactive navigation object! Check the log output for error messages."), wxT("Error"), wxOK, this);
			return;
		}
	}


	// Send navigation command to navigator:
	wxString  strX = edX->GetValue();
	wxString  strY = edY->GetValue();
	double   x, y;
	if (!strX.ToDouble( &x )) {  wxMessageBox( _("'x' is not a valid number"), wxT("Error"), wxOK, this); return; }
	if (!strY.ToDouble( &y )) {  wxMessageBox( _("'y' is not a valid number"), wxT("Error"), wxOK, this); return; }

	lyTarget->SetCoordinateBase( x,y );
	plot->Refresh();

	if (reacNavObj)
	{
		CAbstractReactiveNavigationSystem::TNavigationParams   navParams;
		navParams.target.x = x ;
		navParams.target.y = y ;
		navParams.targetAllowedDistance = 0.40f;
		navParams.targetIsRelative = false;

		reacNavObj->navigate( &navParams );
	}
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

	OnbtnNavigateClick( event );
}

void ReactiveNavigationDemoFrame::OntimSimulateTrigger(wxTimerEvent& event)
{
	WX_START_TRY

	static bool IamIN = false;

	if (IamIN)
		return;


	IamIN=true;


	if ( btnStart->IsEnabled() )
	{
		wxCommandEvent dummy;
		OnbtnPauseClick( dummy );
		IamIN=false;

		return; // We are paused!
	}

	ASSERT_(reacNavObj!=NULL);

	// Enable/disable log files on-the-fly:
	reacNavObj->enableLogFile( cbLog->GetValue() );

	// Navigation end?
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

	// Go on, simulate one time step:
	// Robot sim:
	robotSim.simulateInterval( 0.001 * SIMULATION_TIME_STEPS );

	reacNavObj->navigationStep();

	// Update the target & robot pose & redraw:

	lyVehicle->SetCoordinateBase(
		robotSim.getX(),
		robotSim.getY(),
		robotSim.getPHI() );


	plot->Refresh();

	StatusBar1->SetStatusText(
		_U(format("Pose=(%.03f,%.03f,%.02fdeg) (v,w)=(%.03f,%.02f)",
		robotSim.getX(),
		robotSim.getY(),
		RAD2DEG(robotSim.getPHI()),
		robotSim.getV(),
		RAD2DEG(robotSim.getW()) ).c_str() ), 1 );


	// Set timer to continue simulation:
    //timSimulate.Start(SIMULATION_TIME_STEPS, true); // One-shot


	IamIN=false;

	WX_END_TRY
}

void ReactiveNavigationDemoFrame::reloadRobotShape()
{
	try
	{
		// Get ini-data:
		CConfigFileBase    *iniReactive=NULL, *configRobotIni=NULL;
		createConfigSources(this, iniReactive, configRobotIni );
		if (!iniReactive || !configRobotIni) return;

		string robotName = configRobotIni->read_string("ROBOT_NAME","Name","SENA");
		vector<float> xs,ys;

		iniReactive->read_vector(robotName,"RobotModel_shape2D_xs",vector<float>(0), xs, true );
		iniReactive->read_vector(robotName,"RobotModel_shape2D_ys",vector<float>(0), ys, true );

		delete iniReactive; iniReactive=NULL;
		delete configRobotIni; configRobotIni=NULL;

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

void ReactiveNavigationDemoFrame::OnbtnEditRobotParamsClick(wxCommandEvent& event)
{
	iniEditorRobot->Center();
    if (iniEditorRobot->ShowModal())
		EDIT_internalCfgRobot = string( iniEditorRobot->edText->GetValue().mb_str() );
}

void ReactiveNavigationDemoFrame::OnbtnEditNavParamsClick(wxCommandEvent& event)
{
	iniEditoreactivenav->Center();
    if (iniEditoreactivenav->ShowModal())
		EDIT_internalCfgReactive = string( iniEditoreactivenav->edText->GetValue().mb_str() );
}

void ReactiveNavigationDemoFrame::OnrbExtMapSelect(wxCommandEvent& event)
{
    edMapFile->Enable( ! cbExtMap->GetValue() );
}

void ReactiveNavigationDemoFrame::OncbInternalParamsClick(wxCommandEvent& event)
{
    btnEditRobotParams->Enable( ! cbInternalParams->GetValue() );
    btnEditNavParams->Enable( ! cbInternalParams->GetValue() );

    edRobotCfgFile->Enable( cbInternalParams->GetValue() );
    edNavCfgFile->Enable( cbInternalParams->GetValue() );
}



// Debug windows:
#if 0
	if (m_debugWindows)
	{
		if (!m_debugWin_WS.present())
			m_debugWin_WS = mrpt::gui::CDisplayWindowPlotsPtr( new mrpt::gui::CDisplayWindowPlots("[ReactiveNav] Workspace") );

		// Plot obstacles:
		{
			vector<float> xs,ys;
			CSimplePointsMap	pointsTrans = WS_Obstacles;
			pointsTrans.changeCoordinatesReference(curPose);

			pointsTrans.getAllPoints(xs,ys);
			m_debugWin_WS->plot(xs,ys,"b.3","obstacles");
		}

		// Plot robot shape:
		{
			vector<double> xs,ys;
			robotShape.getAllVertices(xs,ys);
			if (!xs.empty()) { xs.push_back(xs[0]); ys.push_back(ys[0]); }
			for (size_t i=0;i<xs.size();i++)
			{
				const CPoint2D  p = curPose + CPoint2D(xs[i],ys[i]);
				xs[i] = p.x();
				ys[i] = p.y();
			}
			m_debugWin_WS->plot(xs,ys,"r-2","shape");
		}

		// Plot current dir:
		{

			vector<double> xs(2),ys(2);
			xs[0] = curPose.x();
			ys[0] = curPose.y();

			xs[1] = curPose.x() + cos(cur_approx_heading_dir+curPose.phi()) * 1.5;
			ys[1] = curPose.y() + sin(cur_approx_heading_dir+curPose.phi()) * 1.5;

			m_debugWin_WS->plot(xs,ys,"b-","cur_dir");
		}

		// Plot target point:
		{
			vector<double> xs,ys;
			xs.push_back( m_navigationParams.target.x );
			ys.push_back( m_navigationParams.target.y );
			m_debugWin_WS->plot(xs,ys,"k.7","target");
		}

		m_debugWin_WS->axis_fit(true);
	}
#endif
