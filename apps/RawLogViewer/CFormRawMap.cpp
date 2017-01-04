/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "CFormRawMap.h"

#include "xRawLogViewerMain.h"

#include <wx/msgdlg.h>
#include <wx/filedlg.h>
#include <wx/progdlg.h>
#include <wx/app.h>

//(*InternalHeaders(CFormRawMap)
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/font.h>
#include <wx/intl.h>
#include <wx/image.h>
#include <wx/string.h>
//*)

#include <wx/numdlg.h>

// General global variables:
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/system/os.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/math/geometry.h>
#include <mrpt/topography.h>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::gui;
using namespace std;

// The map built from laser & odometry.
CMultiMetricMap			theMap;
CPose3DInterpolator		robot_path;
mrpt::topography::TPathFromRTKInfo	rtk_path_info;


extern xRawLogViewerFrame				*theMainWindow;


//(*IdInit(CFormRawMap)
const long CFormRawMap::ID_STATICTEXT7 = wxNewId();
const long CFormRawMap::ID_STATICTEXT6 = wxNewId();
const long CFormRawMap::ID_STATICTEXT5 = wxNewId();
const long CFormRawMap::ID_STATICTEXT1 = wxNewId();
const long CFormRawMap::ID_SLIDER1 = wxNewId();
const long CFormRawMap::ID_SPINCTRL1 = wxNewId();
const long CFormRawMap::ID_STATICTEXT3 = wxNewId();
const long CFormRawMap::ID_SLIDER2 = wxNewId();
const long CFormRawMap::ID_SPINCTRL2 = wxNewId();
const long CFormRawMap::ID_STATICTEXT10 = wxNewId();
const long CFormRawMap::ID_SLIDER3 = wxNewId();
const long CFormRawMap::ID_SPINCTRL3 = wxNewId();
const long CFormRawMap::ID_BUTTON2 = wxNewId();
const long CFormRawMap::ID_BUTTON6 = wxNewId();
const long CFormRawMap::ID_BUTTON5 = wxNewId();
const long CFormRawMap::ID_BUTTON1 = wxNewId();
const long CFormRawMap::ID_BUTTON3 = wxNewId();
const long CFormRawMap::ID_BUTTON7 = wxNewId();
const long CFormRawMap::ID_BUTTON8 = wxNewId();
const long CFormRawMap::ID_BUTTON9 = wxNewId();
const long CFormRawMap::ID_STATICTEXT8 = wxNewId();
const long CFormRawMap::ID_STATICTEXT2 = wxNewId();
const long CFormRawMap::ID_BITMAPBUTTON1 = wxNewId();
const long CFormRawMap::ID_BUTTON4 = wxNewId();
const long CFormRawMap::ID_TEXTCTRL1 = wxNewId();
const long CFormRawMap::ID_PANEL1 = wxNewId();
const long CFormRawMap::ID_CUSTOM2 = wxNewId();
const long CFormRawMap::ID_PANEL3 = wxNewId();
//*)

BEGIN_EVENT_TABLE(CFormRawMap,wxDialog)
    //(*EventTable(CFormRawMap)
    //*)
END_EVENT_TABLE()

CFormRawMap::CFormRawMap(wxWindow* parent,wxWindowID )
{
    //(*Initialize(CFormRawMap)
    wxFlexGridSizer* FlexGridSizer4;
    wxFlexGridSizer* FlexGridSizer10;
    wxFlexGridSizer* FlexGridSizer5;
    wxFlexGridSizer* FlexGridSizer9;
    wxStaticBoxSizer* boxResults;
    wxFlexGridSizer* FlexGridSizer7;
    wxFlexGridSizer* FlexGridSizer8;
    wxFlexGridSizer* FlexGridSizer6;
    wxStaticBoxSizer* StaticBoxSizer1;
    wxFlexGridSizer* FlexGridSizer11;
    
    Create(parent, wxID_ANY, _("Creation of \"raw map & paths\" from scans & odometry"), wxDefaultPosition, wxDefaultSize, wxCAPTION|wxDEFAULT_DIALOG_STYLE|wxSYSTEM_MENU|wxRESIZE_BORDER|wxMAXIMIZE_BOX, _T("wxID_ANY"));
    FlexGridSizer1 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer1->AddGrowableCol(0);
    FlexGridSizer1->AddGrowableRow(1);
    Panel2 = new wxPanel(this, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    FlexGridSizer2 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer2->AddGrowableCol(1);
    FlexGridSizer2->AddGrowableRow(0);
    FlexGridSizer4 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer3 = new wxFlexGridSizer(4, 3, 0, 0);
    FlexGridSizer3->AddGrowableCol(1);
    StaticText5 = new wxStaticText(Panel2, ID_STATICTEXT7, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT7"));
    FlexGridSizer3->Add(StaticText5, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText4 = new wxStaticText(Panel2, ID_STATICTEXT6, _("Select which indexes to process:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT6"));
    FlexGridSizer3->Add(StaticText4, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText2 = new wxStaticText(Panel2, ID_STATICTEXT5, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT5"));
    FlexGridSizer3->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText1 = new wxStaticText(Panel2, ID_STATICTEXT1, _("First entry:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    FlexGridSizer3->Add(StaticText1, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    slFrom = new wxSlider(Panel2, ID_SLIDER1, 0, 0, 100, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_SLIDER1"));
    FlexGridSizer3->Add(slFrom, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    edFirst = new wxSpinCtrl(Panel2, ID_SPINCTRL1, _T("0"), wxDefaultPosition, wxDefaultSize, 0, 0, 100, 0, _T("ID_SPINCTRL1"));
    edFirst->SetValue(_T("0"));
    FlexGridSizer3->Add(edFirst, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText3 = new wxStaticText(Panel2, ID_STATICTEXT3, _("Last entry:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
    FlexGridSizer3->Add(StaticText3, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    slTo = new wxSlider(Panel2, ID_SLIDER2, 0, 0, 100, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_SLIDER2"));
    FlexGridSizer3->Add(slTo, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    edLast = new wxSpinCtrl(Panel2, ID_SPINCTRL2, _T("0"), wxDefaultPosition, wxDefaultSize, 0, 0, 100, 0, _T("ID_SPINCTRL2"));
    edLast->SetValue(_T("0"));
    FlexGridSizer3->Add(edLast, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText6 = new wxStaticText(Panel2, ID_STATICTEXT10, _("Decimation:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT10"));
    FlexGridSizer3->Add(StaticText6, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    slDecimate = new wxSlider(Panel2, ID_SLIDER3, 0, 1, 200, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_SLIDER3"));
    FlexGridSizer3->Add(slDecimate, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    edDec = new wxSpinCtrl(Panel2, ID_SPINCTRL3, _T("1"), wxDefaultPosition, wxDefaultSize, 0, 1, 200, 1, _T("ID_SPINCTRL3"));
    edDec->SetValue(_T("1"));
    FlexGridSizer3->Add(edDec, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer4->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer5 = new wxFlexGridSizer(3, 1, 0, 0);
    FlexGridSizer5->AddGrowableCol(0);
    FlexGridSizer6 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer6->AddGrowableCol(0);
    FlexGridSizer6->AddGrowableRow(0);
    StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, Panel2, _("Generate..."));
    FlexGridSizer8 = new wxFlexGridSizer(0, 3, 0, 0);
    FlexGridSizer8->AddGrowableCol(0);
    btnGenerate = new wxButton(Panel2, ID_BUTTON2, _("Map from odometry"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
    btnGenerate->SetDefault();
    FlexGridSizer8->Add(btnGenerate, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnGenerateRTK = new wxButton(Panel2, ID_BUTTON6, _("Map from RTK GPS"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON6"));
    btnGenerateRTK->SetDefault();
    FlexGridSizer8->Add(btnGenerateRTK, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnGeneratePaths = new wxButton(Panel2, ID_BUTTON5, _("Random paths..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON5"));
    btnGeneratePaths->SetDefault();
    FlexGridSizer8->Add(btnGeneratePaths, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    StaticBoxSizer1->Add(FlexGridSizer8, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer6->Add(StaticBoxSizer1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 3);
    FlexGridSizer5->Add(FlexGridSizer6, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer7 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer7->AddGrowableCol(0);
    FlexGridSizer7->AddGrowableRow(0);
    boxResults = new wxStaticBoxSizer(wxHORIZONTAL, Panel2, _("Result"));
    FlexGridSizer9 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer11 = new wxFlexGridSizer(3, 3, 0, 0);
    FlexGridSizer11->AddGrowableCol(0);
    btnSaveTxt = new wxButton(Panel2, ID_BUTTON1, _("Save map as text..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
    btnSaveTxt->Disable();
    FlexGridSizer11->Add(btnSaveTxt, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnSave3D = new wxButton(Panel2, ID_BUTTON3, _("Save map as 3D scene..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON3"));
    btnSave3D->Disable();
    FlexGridSizer11->Add(btnSave3D, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnSavePath = new wxButton(Panel2, ID_BUTTON7, _("Save vehicle path..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON7"));
    btnSavePath->Disable();
    FlexGridSizer11->Add(btnSavePath, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnSaveObsPath = new wxButton(Panel2, ID_BUTTON8, _("Save observations poses..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON8"));
    btnSaveObsPath->Disable();
    FlexGridSizer11->Add(btnSaveObsPath, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnView3D = new wxButton(Panel2, ID_BUTTON9, _("Preview map in 3D"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON9"));
    btnView3D->Disable();
    FlexGridSizer11->Add(btnView3D, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer11->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    lbCount = new wxStaticText(Panel2, ID_STATICTEXT8, _("Point count=0 \n (No decimation)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT8"));
    FlexGridSizer11->Add(lbCount, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    lbLength = new wxStaticText(Panel2, ID_STATICTEXT2, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    FlexGridSizer11->Add(lbLength, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer9->Add(FlexGridSizer11, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    boxResults->Add(FlexGridSizer9, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer7->Add(boxResults, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer5->Add(FlexGridSizer7, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 3);
    FlexGridSizer10 = new wxFlexGridSizer(1, 3, 0, 0);
    FlexGridSizer10->AddGrowableCol(0);
    FlexGridSizer10->AddGrowableRow(0);
    FlexGridSizer10->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnHelp = new wxBitmapButton(Panel2, ID_BITMAPBUTTON1, wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_INFORMATION")),wxART_BUTTON), wxDefaultPosition, wxDefaultSize, wxBU_AUTODRAW, wxDefaultValidator, _T("ID_BITMAPBUTTON1"));
    FlexGridSizer10->Add(btnHelp, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 4);
    btnClose = new wxButton(Panel2, ID_BUTTON4, _("Close"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON4"));
    FlexGridSizer10->Add(btnClose, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer5->Add(FlexGridSizer10, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer4->Add(FlexGridSizer5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer2->Add(FlexGridSizer4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    edOpts = new wxTextCtrl(Panel2, ID_TEXTCTRL1, _("; ====================================================\n; MULTIMETRIC MAP CONFIGURATION\n; ====================================================\n[map]\n; Creation of maps:\noccupancyGrid_count = 0\ngasGrid_count = 0\nlandmarksMap_count = 0\nbeaconMap_count = 0\npointsMap_count = 1\nheightMap_count = 0\ncolourPointsMap_count=0\n\n; ====================================================\n; MULTIMETRIC MAP: PointsMap #00\n; ====================================================\n; Insertion Options for PointsMap 00:\n[map_pointsMap_00_insertOpts]\nminDistBetweenLaserPoints = 0.05\nisPlanarMap = 0\nalso_interpolate = 0\n\n; ====================================================\n; MULTIMETRIC MAP: HeightMap #00\n; ====================================================\n; Creation Options for HeightMap 00:\n[map_heightGrid_00_creationOpts]\nmapType = 0 \t\t; See CHeightGridMap2D::CHeightGridMap2D\nmin_x = -10\nmax_x = 10\nmin_y = -10\nmax_y = 10\nresolution = 0.10\n\n; ====================================================\n; MULTIMETRIC MAP: HeightMap #00\n; ====================================================\n; Insertion Options for HeightMap 00:\n[map_heightGrid_00_insertOpts]\nfilterByHeight = 0 ; 0/1: Do not/do filter.\nz_min =-0.10\nz_max = 0.10\n\n; ====================================================\n; MULTIMETRIC MAP: ColourPointsMap #00\n; ====================================================\n; Insertion Options for ColourPointsMap 00:\n[map_colourPointsMap_00_insertOpts]\nminDistBetweenLaserPoints = 0.05\nisPlanarMap = 0\nalso_interpolate = 0\n\n; Additional options for use in RTK GPS-based map building\n[RTK_MAP]\ndisableGPSInterp = 0 // If set to 1, disable GPS interpolation for sequences of JAVAD readings\nsmooth_filter = 3 // Smooth pitch & roll angles by averaging each value with a window of N previous and next elements (=0: Disable)\n"), wxDefaultPosition, wxSize(247,94), wxTE_PROCESS_ENTER|wxTE_PROCESS_TAB|wxTE_MULTILINE|wxHSCROLL, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    wxFont edOptsFont(8,wxSWISS,wxFONTSTYLE_NORMAL,wxNORMAL,false,_T("Courier New"),wxFONTENCODING_DEFAULT);
    edOpts->SetFont(edOptsFont);
    FlexGridSizer2->Add(edOpts, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    Panel2->SetSizer(FlexGridSizer2);
    FlexGridSizer2->Fit(Panel2);
    FlexGridSizer2->SetSizeHints(Panel2);
    FlexGridSizer1->Add(Panel2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel3 = new wxPanel(this, ID_PANEL3, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL3"));
    BoxSizer1 = new wxBoxSizer(wxHORIZONTAL);
    plotMap = new mpWindow(Panel3,ID_CUSTOM2,wxDefaultPosition,wxSize(742,380),0);
    BoxSizer1->Add(plotMap, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel3->SetSizer(BoxSizer1);
    BoxSizer1->Fit(Panel3);
    BoxSizer1->SetSizeHints(Panel3);
    FlexGridSizer1->Add(Panel3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    SetSizer(FlexGridSizer1);
    FlexGridSizer1->Fit(this);
    FlexGridSizer1->SetSizeHints(this);
    Center();
    
    Connect(ID_SLIDER1,wxEVT_SCROLL_THUMBTRACK,(wxObjectEventFunction)&CFormRawMap::OnslFromCmdScrollThumbTrack);
    Connect(ID_SLIDER1,wxEVT_SCROLL_CHANGED,(wxObjectEventFunction)&CFormRawMap::OnslFromCmdScrollThumbTrack);
    Connect(ID_SLIDER2,wxEVT_SCROLL_THUMBTRACK,(wxObjectEventFunction)&CFormRawMap::OnslToCmdScrollThumbTrack);
    Connect(ID_SLIDER2,wxEVT_SCROLL_CHANGED,(wxObjectEventFunction)&CFormRawMap::OnslToCmdScrollThumbTrack);
    Connect(ID_SLIDER3,wxEVT_SCROLL_THUMBTRACK,(wxObjectEventFunction)&CFormRawMap::OnslDecimateCmdScrollThumbTrack);
    Connect(ID_SLIDER3,wxEVT_SCROLL_CHANGED,(wxObjectEventFunction)&CFormRawMap::OnslDecimateCmdScrollThumbTrack);
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormRawMap::OnbtnGenerateClick);
    Connect(ID_BUTTON6,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormRawMap::OnGenerateFromRTK);
    Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormRawMap::OnbtnGeneratePathsClick);
    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormRawMap::OnbtnSaveTxtClick);
    Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormRawMap::OnbtnSave3DClick);
    Connect(ID_BUTTON7,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormRawMap::OnbtnSavePathClick);
    Connect(ID_BUTTON8,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormRawMap::OnbtnSaveObsPathClick);
    Connect(ID_BUTTON9,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormRawMap::OnbtnView3DClick);
    Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormRawMap::OnbtnCloseClick);
    //*)

    Maximize();
    FlexGridSizer1->RecalcSizes();
}

CFormRawMap::~CFormRawMap()
{
    //(*Destroy(CFormRawMap)
    //*)
}

void loadMapInto3DScene(COpenGLScene &scene)
{
	{
		TPoint3D minC,maxC;
		if (robot_path.size())
			robot_path.getBoundingBox(minC,maxC);
		else
		{
			minC=CPoint3D(-100,-100,0);
			maxC=CPoint3D(100,100,0);
		}

		mrpt::opengl::CGridPlaneXYPtr	gridobj = mrpt::opengl::CGridPlaneXY::Create( minC.x-20, maxC.x+20, minC.y-20, maxC.y+20, minC.z-2, 5 );
		gridobj->setColor(0.3,0.3,0.3, 1);
		scene.insert( gridobj );
	}

	// Camera points at path beginning:
	if (!robot_path.empty())
	{
		CPose3D &p = robot_path.begin()->second;
		scene.getViewport("main")->getCamera().setPointingAt( p );
		scene.enableFollowCamera(true);
	}

	// The robot path:
	{
		mrpt::opengl::CSetOfLinesPtr  obj = mrpt::opengl::CSetOfLines::Create();

		obj->setColor(0,1,0, 0.5);
		obj->setLineWidth(4);

		mrpt::opengl::CSetOfLinesPtr  obj2 = mrpt::opengl::CSetOfLines::Create();
		obj2->setColor(1,0,0, 0.5);
		obj2->setLineWidth(2);

		double x0=0,y0=0,z0=0;
		TTimeStamp	last_t=INVALID_TIMESTAMP, this_t;

		for (CPose3DInterpolator::iterator it=robot_path.begin();it!=robot_path.end();++it)
		{
			CPose3D &p = it->second;
			this_t = it->first;

			if ( distanceBetweenPoints(x0,y0,z0, p.x(),p.y(),p.z()) < 5.5 )
			{
				obj->appendLine( x0,y0,z0, p.x(),p.y(),p.z() );
			}
			else if (last_t!=INVALID_TIMESTAMP)
			{
				// We have a gap without GT:
				// Try to interpolate using the best GPS path:
				// map<TTimeStamp,CPoint3D> best_gps_path;		// time -> 3D local coords
				map<TTimeStamp,TPoint3D>::const_iterator i1 = rtk_path_info.best_gps_path.lower_bound(last_t);
				map<TTimeStamp,TPoint3D>::const_iterator i2 = rtk_path_info.best_gps_path.upper_bound(this_t);

				//cout << mrpt::system::timeLocalToString(last_t) << " -> " << mrpt::system::timeLocalToString(this_t) << " D: " << std::distance(i1,i2) << endl;

				if (i1!=rtk_path_info.best_gps_path.end())
				{
					for (map<TTimeStamp,TPoint3D>::const_iterator t=i1;t!=i2 && t!=rtk_path_info.best_gps_path.end();++t)
					{
						obj2->appendLine( x0,y0,z0, t->second.x,t->second.y,t->second.z );
						x0 = t->second.x;
						y0 = t->second.y;
						z0 = t->second.z;
					}
				}
			}
			// For the next loop:
			x0 = p.x();
			y0 = p.y();
			z0 = p.z();
			last_t = this_t;
		}

		// Perhaps we have one final segment of the path without GT:
		if (!rtk_path_info.best_gps_path.empty())
		{
			this_t = rtk_path_info.best_gps_path.rbegin()->first;
			// We have a gap without GT:
			// Try to interpolate using the best GPS path:
			// map<TTimeStamp,CPoint3D> best_gps_path;		// time -> 3D local coords
			map<TTimeStamp,TPoint3D>::const_iterator i1 = rtk_path_info.best_gps_path.lower_bound(last_t);
			map<TTimeStamp,TPoint3D>::const_iterator i2 = rtk_path_info.best_gps_path.upper_bound(this_t);

			//cout << mrpt::system::timeLocalToString(last_t) << " -> " << mrpt::system::timeLocalToString(this_t) << " D: " << std::distance(i1,i2) << endl;

			if (i1!=rtk_path_info.best_gps_path.end())
			{
				for (map<TTimeStamp,TPoint3D>::const_iterator t=i1;t!=i2 && t!=rtk_path_info.best_gps_path.end();++t)
				{
					obj2->appendLine( x0,y0,z0, t->second.x,t->second.y,t->second.z );
					x0 = t->second.x;
					y0 = t->second.y;
					z0 = t->second.z;
				}
			}
		}

		scene.insert(obj);
		scene.insert(obj2);
	}

	// The built maps:
	// ---------------------------
	CPointsMap::COLOR_3DSCENE_R =
	CPointsMap::COLOR_3DSCENE_G =
	CPointsMap::COLOR_3DSCENE_B = 0.9;

	opengl::CSetOfObjectsPtr objs = opengl::CSetOfObjects::Create();
	theMap.getAs3DObject( objs );
	scene.insert( objs );
}


// From slider moved:
void CFormRawMap::OnslFromCmdScrollThumbTrack(wxScrollEvent& )
{
    int toVal = slTo->GetValue();
    int curVal = slFrom->GetValue();

    if (curVal>toVal) slFrom->SetValue(toVal);
    edFirst->SetValue( wxString::Format(_("%d"),slFrom->GetValue()) );
}

// To slider moved:
void CFormRawMap::OnslToCmdScrollThumbTrack(wxScrollEvent& )
{
    int fromVal = slFrom->GetValue();
    int curVal = slTo->GetValue();

    if (curVal<fromVal) slTo->SetValue(fromVal);
    edLast->SetValue( wxString::Format(_("%d"),slTo->GetValue()) );
}

// Generate the map:
void CFormRawMap::OnbtnGenerateClick(wxCommandEvent& )
{
    WX_START_TRY

    // Go: generate the map:
    size_t      i;
    CPose2D     curPose(0,0,0);

    size_t      first = edFirst->GetValue();
    size_t      last  = edLast->GetValue();
    size_t      decimate = edDec->GetValue();
    ASSERT_(first<=last);
    ASSERT_(last<=rawlog.size()-1);
    ASSERT_(decimate>0);

    // Create a memory "ini file" with the text in the window:
    CConfigFileMemory       configSrc( CStringList( std::string(edOpts->GetValue().mb_str()) ) );

	TSetOfMetricMapInitializers		lstMaps;
	lstMaps.loadFromConfigFile( configSrc, "map" );
	theMap.setListOfMaps( &lstMaps );

	CPointsMapPtr	thePntsMap;

	if( !theMap.m_pointsMaps.empty() )
		thePntsMap = theMap.m_pointsMaps[0];
	else if (theMap.m_colourPointsMap.present())
		thePntsMap = theMap.m_colourPointsMap;

    wxBusyCursor    waitCursor;

    wxProgressDialog    progDia(
        wxT("Creating raw map"),
        wxT("Working..."),
        (int)(last-first+1), // range
        this, // parent
        wxPD_CAN_ABORT |
        wxPD_APP_MODAL |
        wxPD_SMOOTH |
        wxPD_AUTO_HIDE |
        wxPD_ELAPSED_TIME |
        wxPD_ESTIMATED_TIME |
        wxPD_REMAINING_TIME);

    wxTheApp->Yield();  // Let the app. process messages
    size_t count = 0;

    vector<float>    pathX,pathY;
    bool            abort = false;

    robot_path.clear();

    // An (aprox) estimate of the final size of the map (great improve in speed!)
	if (thePntsMap) thePntsMap->reserve( (last-first+1)*800 );

	TTimeStamp	last_tim = INVALID_TIMESTAMP;

    for (i=first;!abort && i<=last;i++)
    {
    	bool addNewPathEntry = false;

    	switch( rawlog.getType(i) )
        {
		case CRawlog::etActionCollection:
			{
				CActionCollectionPtr    acts = rawlog.getAsAction(i);
				CPose2D                 poseIncrement;
				bool                    poseIncrementLoaded = false;

				for (size_t j=0;j<acts->size();j++)
				{
					CActionPtr act = acts->get(j);
					if (act->GetRuntimeClass() == CLASS_ID( CActionRobotMovement2D ))
					{
						CActionRobotMovement2DPtr mov = CActionRobotMovement2DPtr( act );

						// Load an odometry estimation, but only if it is the only movement
						//  estimation source: any other may be a better one:
						if ( !poseIncrementLoaded || mov->estimationMethod!= CActionRobotMovement2D::emOdometry )
						{
							poseIncrementLoaded=true;
							mov->poseChange->getMean( poseIncrement );
						}
					}
				}

				if ( !poseIncrementLoaded && i<last )
					THROW_EXCEPTION_CUSTOM_MSG1("ERROR: Odometry not found at step %d!",(int)i);

				curPose = curPose + poseIncrement;
				addNewPathEntry=true;
			}
			break;
		case CRawlog::etSensoryFrame:
			{
				if (( (i>>1) % decimate)==0)
				{
					CPose3D		dumPose(curPose);
					rawlog.getAsObservations(i)->insertObservationsInto( &theMap, &dumPose );
				}
				addNewPathEntry=true;
			}
			break;
		case CRawlog::etObservation:
			{
				// Always, process odometry:
				const CObservation* obs = rawlog.getAsObservation(i).pointer();
				if (IS_CLASS(obs,CObservationOdometry))
				{
					const CObservationOdometry* obsOdo = static_cast<const CObservationOdometry*>(obs);
					curPose = obsOdo->odometry;
				}

				if (( (i>>1) % decimate)==0)
				{
					CPose3D		dumPose(curPose);
					theMap.insertObservation( rawlog.getAsObservation(i).pointer(), &dumPose );
					last_tim = rawlog.getAsObservation(i)->timestamp;
				}
				addNewPathEntry=true;
			}
			break;
		default:
			break;
		}; // end switch

		if (addNewPathEntry)
		{
			pathX.push_back( curPose.x() );
			pathY.push_back( curPose.y() );
			if ( last_tim != INVALID_TIMESTAMP )
				robot_path.insert( last_tim, curPose );
		}


        if ((count++ % 50)==0)
        {
            if (!progDia.Update((int)(i-first)))
                abort = true;
            wxTheApp->Yield();
        }
    } // end for i

    progDia.Update( (int)(last-first+1) );


    // Load into the graphs:
    // ----------------------------------
    plotMap->DelAllLayers(true,false);

    mpFXYVector *lyPoints = new mpFXYVector();
    mpFXYVector *lyPath   = new mpFXYVector();
    lyPath->SetPen( wxPen(wxColour(255,0,0),2) );
    lyPath->SetContinuity( true );
    lyPoints->SetPen( wxPen(wxColour(0,0,255),0) );

    plotMap->AddLayer( lyPoints );
    plotMap->AddLayer( lyPath );
    plotMap->EnableDoubleBuffer(true);

    lyPath->SetData( pathX,pathY );

	if (thePntsMap)
	{
		size_t nPts = thePntsMap->size();
		size_t decimation = 1;

		if (nPts>100000)
		{
			decimation = nPts / 100000;
		}

		vector<float>    Xs,Ys;
		thePntsMap->getAllPoints(Xs,Ys, decimation );

		lyPoints->SetData(Xs,Ys);
		plotMap->LockAspect(false);
		plotMap->Fit();      // Update the window to show the new data fitted.
		plotMap->LockAspect(true);
		plotMap->AddLayer( new mpScaleX() );
		plotMap->AddLayer( new mpScaleY() );

		if (decimation>1)
				lbCount->SetLabel( wxString::Format(_("Point count=%u\n(Decimation=%u)"),unsigned(Xs.size()), unsigned(decimation) ) );
		else	lbCount->SetLabel( wxString::Format(_("Point count=%u\n(No decimation)"),unsigned(Xs.size()) ) );
	}

	// Enable "results" buttons:
	btnSaveTxt->Enable();
	btnSave3D->Enable();
	btnSavePath->Enable();
	btnSaveTxt->Enable();
	btnSaveObsPath->Enable();
	btnView3D->Enable();

    WX_END_TRY
}

void CFormRawMap::OnbtnSaveTxtClick(wxCommandEvent& )
{
    wxString caption = wxT("Save as text file...");
    wxString wildcard = wxT("Text files (*.txt)|*.txt|All files (*.*)|*.*");

    wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );

    wxString defaultFilename = _( "map.txt" );
    wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT);

    if (dialog.ShowModal() == wxID_OK)
    {
        wxString fileName = dialog.GetPath();
        try
        {
			theMap.saveMetricMapRepresentationToFile( std::string(fileName.mb_str()) );
        }
        catch (std::exception &e)
        {
            wxMessageBox( _U(e.what()), _("Exception"), wxOK, this);
        }
    }
}

// Save as a 3D scene:
void CFormRawMap::OnbtnSave3DClick(wxCommandEvent& )
{
    wxString caption = wxT("Save as 3D scene file...");
    wxString wildcard = wxT("MRPT 3D scene files (*.3Dscene)|*.3Dscene|All files (*.*)|*.*");

    wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );

    wxString defaultFilename = _( "map.3Dscene" );
    wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT);

    if (dialog.ShowModal() == wxID_OK)
    {
        try
        {
			wxBusyCursor    waitCursor;

            CFileGZOutputStream fil( std::string(dialog.GetPath().mb_str()) );
            COpenGLScene		scene;

            loadMapInto3DScene(scene);

            fil << scene;
        }
        catch (std::exception &e)
        {
            wxMessageBox( _U(e.what()), _("Exception"), wxOK, this);
        }
    }
}

// Close:
void CFormRawMap::OnbtnCloseClick(wxCommandEvent& )
{
    Close();
}

void CFormRawMap::OnslDecimateCmdScrollThumbTrack(wxScrollEvent& )
{
    edDec->SetValue( wxString::Format(_("%d"),slDecimate->GetValue() ) );
}

// Generate random paths from the motion model:
void CFormRawMap::OnbtnGeneratePathsClick(wxCommandEvent& )
{
    WX_START_TRY

    // Go: generate the map:
    size_t      i;


    long nPaths = ::wxGetNumberFromUser(
                      _("Each incremental pose will be drawn from its probabilistic motion model)"),
                      _("How many random samples of the path to generate?\n"),
                      _("Generation of random paths"),
                      100 /*Default*/, 1/*Min*/, 10000/*Max*/);

    if (nPaths<0) return;

    size_t      first = edFirst->GetValue();
    size_t      last  = edLast->GetValue();
    size_t      decimate = edDec->GetValue();
    ASSERT_(first<=last);
    ASSERT_(last<=rawlog.size()-1);
    ASSERT_(decimate>0);
    ASSERT_(nPaths>0 && nPaths<100000);

    // Load into the graphs:
    // ----------------------------------
    plotMap->DelAllLayers(true,false);

    wxBusyCursor    waitCursor;

    size_t     nComputationSteps = (last-first+1)*nPaths;

    wxProgressDialog    progDia(
        wxT("Generating paths"),
        wxT("Working..."),
        (int)nComputationSteps, // range
        this, // parent
        wxPD_CAN_ABORT |
        wxPD_APP_MODAL |
        wxPD_SMOOTH |
        wxPD_AUTO_HIDE |
        wxPD_ELAPSED_TIME |
        wxPD_ESTIMATED_TIME |
        wxPD_REMAINING_TIME);

    wxTheApp->Yield();  // Let the app. process messages
    size_t count = 0;

    bool            abort = false;

    CPosePDFParticles	pdfParts( nPaths );

    for (int pathIter = 0;!abort && pathIter<nPaths;pathIter++)
    {
        vector<float>    pathX,pathY,pathPhi;
        CPose2D         curPose(0,0,0);

        // Initial pose:
        pathX.push_back(0);
        pathY.push_back(0);
        pathPhi.push_back(0);

        for (i=first;!abort && i<=last;i++)
        {
            switch ( rawlog.getType(i) )
            {
			case CRawlog::etActionCollection:
            	{
					CActionCollectionPtr    acts = rawlog.getAsAction(i);
					CPose2D                 poseIncrement;
					bool                    poseIncrementLoaded = false;

					for (size_t j=0;j<acts->size();j++)
					{
						CActionPtr act = acts->get(j);
						if (act->GetRuntimeClass() == CLASS_ID( CActionRobotMovement2D ))
						{
							CActionRobotMovement2DPtr mov = CActionRobotMovement2DPtr( act );

							// Load an odometry estimation, but only if it is the only movement
							//  estimation source: any other may be a better one:
							if ( !poseIncrementLoaded || mov->estimationMethod!= CActionRobotMovement2D::emOdometry )
							{
								poseIncrementLoaded=true;
								// Draw random pose:
								mov->drawSingleSample( poseIncrement );
							}
						}
					}

					if ( !poseIncrementLoaded )
						THROW_EXCEPTION_CUSTOM_MSG1("ERROR: Odometry not found at step %d!",(int)i);

					curPose = curPose + poseIncrement;
					pathX.push_back( curPose.x() );
					pathY.push_back( curPose.y() );
					pathPhi.push_back( curPose.phi() );
				}
				break;

			default:
				{
					// It's an observation... nothing to do.
				}
            };

            if ((count++ % 50)==0)
            {
                if (!progDia.Update((int)count)) abort = true;
                wxTheApp->Yield();
            }
        } // end for i

        if (!abort)
        {
            // Save for the covariance:
            // ----------------------------
            *(pdfParts.m_particles[pathIter].d) = CPose2D(
                                                      pathX[pathX.size()-1],
                                                      pathY[pathX.size()-1],
                                                      pathPhi[pathX.size()-1] );

            // Add new layer with this path:
            // ----------------------------------
            mpFXYVector *lyPath   = new mpFXYVector();
            lyPath->SetPen( wxPen(wxColour(255,0,0),1) );
            lyPath->SetContinuity( true );

            // Load into the graphs:
            lyPath->SetData( pathX,pathY );

            plotMap->AddLayer( lyPath, false );
        }

    } // end for path samples

    progDia.Update( (int)nComputationSteps );

    // Add a layer with a covariance with the last pose:
    CMatrixDouble33 COV;
    CPose2D	meanPose;
	pdfParts.getCovarianceAndMean(COV,meanPose);

    mpCovarianceEllipse	*lyCov = new mpCovarianceEllipse( COV(0,0),COV(1,1),COV(0,1) );
    lyCov->SetContinuity( true );
    lyCov->SetCoordinateBase( meanPose.x(), meanPose.y(), 0 );
    plotMap->AddLayer( lyCov ,false);

    //
    plotMap->EnableDoubleBuffer(true);

    plotMap->Fit();      // Update the window to show the new data fitted.
    plotMap->LockAspect(true);
    plotMap->AddLayer( new mpScaleX() );
    plotMap->AddLayer( new mpScaleY() );

    lbCount->SetLabel( wxString::Format(_("%d paths"),nPaths) );

    WX_END_TRY
}

// ----------------------------------------------------------------------------------------
// Generate a "Ground-truth" path from RTK GPS data, then build maps from those goos poses
// ----------------------------------------------------------------------------------------
void CFormRawMap::OnGenerateFromRTK(wxCommandEvent& )
{
    WX_START_TRY

    // Go: generate the map:
    size_t      i;

    size_t      first = edFirst->GetValue();
    size_t      last  = edLast->GetValue();
    size_t      decimate = slDecimate->GetValue();
    ASSERT_(first<=last);
    ASSERT_(last<=rawlog.size()-1);
    ASSERT_(decimate>0);

    // Create a memory "ini file" with the text in the window:
    CConfigFileMemory       configSrc( CStringList( std::string(edOpts->GetValue().mb_str()) ) );

	TSetOfMetricMapInitializers		lstMaps;
	lstMaps.loadFromConfigFile( configSrc, "map" );
	theMap.setListOfMaps( &lstMaps );


	// -------------------------------------------
	// Run path reconstruction:
	// -------------------------------------------
	mrpt::topography::path_from_rtk_gps(
		robot_path,
		rawlog,
		first,
		last,
		true, // Is GUI
		configSrc.read_bool("RTK_MAP","disableGPSInterp",false),
		configSrc.read_int("RTK_MAP", "smooth_filter", 1 ),
		&rtk_path_info );

	// ---------------------------------------------------
	//   Now, build the map using the interpolator
    // --------------------------------------------------
    wxBusyCursor    waitCursor;

	CPointsMapPtr	thePntsMap;

	if( !theMap.m_pointsMaps.empty() )
		thePntsMap = theMap.m_pointsMaps[0];
	else if (theMap.m_colourPointsMap.present())
		thePntsMap = theMap.m_colourPointsMap;

	// An (aprox) estimate of the final size of the map (great improve in speed!)
	if (thePntsMap) thePntsMap->reserve( last-first+1 );

    size_t count = 0;
    bool	abort = false;

    wxProgressDialog    progDia2(
        wxString::Format(wxT("Building map - %u GPS points"),(unsigned) robot_path.size() ),
        wxT("Populating the map with observations..."),
        (int)(last-first+1), // range
        this, // parent
        wxPD_CAN_ABORT |
        wxPD_APP_MODAL |
        wxPD_SMOOTH |
        wxPD_AUTO_HIDE |
        wxPD_ELAPSED_TIME |
        wxPD_ESTIMATED_TIME |
        wxPD_REMAINING_TIME);

	progDia2.SetSize( 400,progDia2.GetSize().GetHeight() );
    wxTheApp->Yield();  // Let the app. process messages

    for (i=first;!abort && i<=last;i++)
    {
        switch ( rawlog.getType(i) )
		{
		default:
			break;

        case CRawlog::etObservation:
            {
                CObservationPtr o = rawlog.getAsObservation(i);

				// Interpolate:
				CPose3D		p;
				bool		valid_interp;
				robot_path.interpolate( o->timestamp, p, valid_interp);

				if (valid_interp)
				{
					// Decimation counters for each sensor label:
					static std::map<string, size_t>	decim_count;
					size_t &dec_cnt = decim_count[o->sensorLabel];

					if ( (++dec_cnt % decimate)==0)
						theMap.insertObservation( o.pointer(), &p );
				}
            }
            break;
        } // end switch type

		// Show progress:
        if ((count++ % 100)==0)
        {
            if (!progDia2.Update((int)(i-first)))
                abort = true;
            wxTheApp->Yield();
        }
    } // end for i

    progDia2.Update( (int)(last-first+1) );


    // Load into the graphs:
    // ----------------------------------
	vector<float>			pathX,pathY;
	pathX.reserve(robot_path.size());
	pathY.reserve(robot_path.size());

	double overall_dist = 0;
	double last_x=0,last_y=0,last_z=0;

	for (CPose3DInterpolator::iterator i=robot_path.begin();i!=robot_path.end();++i)
	{
		if (i!=robot_path.begin())
			overall_dist += i->second.distance3DTo(last_x,last_y,last_z);
		last_x=i->second.x();
		last_y=i->second.y();
		last_z=i->second.z();

		pathX.push_back(i->second.x());
		pathY.push_back(i->second.y());
	}

    plotMap->DelAllLayers(true,false);

    mpFXYVector *lyPoints = new mpFXYVector();
    mpFXYVector *lyPath   = new mpFXYVector();
    lyPath->SetPen( wxPen(wxColour(255,0,0),2) );
    lyPath->SetContinuity( true );
    lyPoints->SetPen( wxPen(wxColour(0,0,255),0) );

    plotMap->AddLayer( lyPoints );
    plotMap->AddLayer( lyPath );
    plotMap->EnableDoubleBuffer(true);

    lyPath->SetData( pathX,pathY );

	if (thePntsMap)
	{
		size_t nPts = thePntsMap->size();
		size_t decimation = 1;

		if (nPts>100000)
		{
			decimation = nPts / 100000;
		}

		vector<float>    Xs,Ys;
		thePntsMap->getAllPoints(Xs,Ys, decimation );

		lyPoints->SetData(Xs,Ys);
		plotMap->LockAspect(false);
		plotMap->Fit();      // Update the window to show the new data fitted.
		plotMap->LockAspect(true);
		plotMap->AddLayer( new mpScaleX() );
		plotMap->AddLayer( new mpScaleY() );

		if (decimation>1)
				lbCount->SetLabel( wxString::Format(_("Point count=%u\n(Decimation=%u)"),unsigned(Xs.size()), unsigned(decimation) ) );
		else	lbCount->SetLabel( wxString::Format(_("Point count=%u\n(No decimation)"),unsigned(Xs.size()) ) );
	}

	lbLength->SetLabel( wxString::Format( _("Overall path length: %.03f meters"), overall_dist ) );

	// Enable "results" buttons:
	btnSaveTxt->Enable();
	btnSave3D->Enable();
	btnSavePath->Enable();
	btnSaveTxt->Enable();
	btnSaveObsPath->Enable();
	btnView3D->Enable();

    WX_END_TRY
}

void CFormRawMap::OnbtnSavePathClick(wxCommandEvent& )
{
	WX_START_TRY

	if (!robot_path.size())
	{
		wxMessageBox(_("The robot path is empty."));
		return;
	}

	string outputPath;

	// ---------------------------------------------
	// Save the vehicle path
	// ---------------------------------------------
    wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
    {
		wxFileDialog dialog(this, wxT("Save path as txt file..."), defaultDir, wxT( "GT_path_vehicle.txt" ),wxT("Text files (*.txt)|*.txt|All files (*.*)|*.*"), wxFD_SAVE | wxFD_OVERWRITE_PROMPT);

		if (dialog.ShowModal() != wxID_OK) return;

		wxBusyCursor    waitCursor;

		cout << robot_path.size() << endl;

		CFileOutputStream	f( string(dialog.GetPath().mb_str()) );
		for ( CPose3DInterpolator::const_iterator i=robot_path.begin();i!=robot_path.end();++i)
		{
			const double	t  = timestampTotime_t(i->first);
			const CPose3D	&p = i->second;
			f.printf("%.06f %.06f %.06f %.06f %.06f %.06f %.06f ",
				t,
				p.x(),p.y(),p.z(),
				p.yaw(),p.pitch(),p.roll());

			// The uncertainty, if available:
			mrpt::aligned_containers<mrpt::system::TTimeStamp, mrpt::math::CMatrixDouble66 >::map_t::const_iterator Wit = rtk_path_info.vehicle_uncertainty.find(i->first);
			if (Wit!=rtk_path_info.vehicle_uncertainty.end())
			{
				const CMatrixDouble66 &C = Wit->second;

				f.printf(" %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e\n",
					C(0,0),C(0,1),C(0,2),C(0,3),C(0,4),C(0,5),
					C(1,1),C(1,2),C(1,3),C(1,4),C(1,5),
					C(2,2),C(2,3),C(2,4),C(2,5),
					C(3,3),C(3,4),C(3,5),
					C(4,4),C(4,5),
					C(5,5) );
			}
			else
				f.printf(" %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e\n",
					1e-4,0.0,0.0,0.0,0.0,0.0,
					1e-4,0.0,0.0,0.0,0.0,
					1e-4,0.0,0.0,0.0,
					1e-4,0.0,0.0,
					1e-4,0.0,
					1e-4 );
		}

		outputPath = mrpt::system::extractFileDirectory( string(dialog.GetPath().mb_str()) );
    }

	// ---------------------------------------------
	// Save the vehicle path (binary)
	// ---------------------------------------------
	{
		wxFileDialog dialog(this, wxT("Save path as binary file..."), defaultDir, wxT( "GT_path_vehicle.bin.gz" ),wxT("Binary gz-compressed files (*.bin.gz)|*.bin.gz|All files (*.*)|*.*"), wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
		if (dialog.ShowModal() != wxID_OK) return;

		wxBusyCursor    waitCursor;
		CFileGZOutputStream( string(dialog.GetPath().mb_str()) ) << robot_path;
	}

	// ---------------------------------------------
	// Save the vehicle path, interpolated 100Hz
	// ---------------------------------------------
    {
		wxFileDialog dialog(this, wxT("Save interpolated path as txt file..."), defaultDir, wxT( "GT_path_vehicle_interp.txt" ),wxT("Text files (*.txt)|*.txt|All files (*.*)|*.*"), wxFD_SAVE | wxFD_OVERWRITE_PROMPT);

		if (dialog.ShowModal() != wxID_OK) return;

 	   wxBusyCursor    waitCursor;
		const double interval = 0.01;
		if (!robot_path.saveInterpolatedToTextFile( string(dialog.GetPath().mb_str()), interval ) )
			::wxMessageBox(_("Error creating file."));
    }

	// ---------------------------------------------
	// Save quality measure
	// ---------------------------------------------
	if (!rtk_path_info.mahalabis_quality_measure.empty())
	{
		wxFileDialog dialog(this, wxT("Save path confidence measure..."), defaultDir, wxT( "GT_path_vehicle_confidence.txt" ),wxT("Text files (*.txt)|*.txt|All files (*.*)|*.*"), wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
		if (dialog.ShowModal() != wxID_OK) return;

		wxBusyCursor    waitCursor;
		CFileOutputStream	f( string(dialog.GetPath().mb_str())  );


		for (map<TTimeStamp,double>::const_iterator i=rtk_path_info.mahalabis_quality_measure.begin();i!=rtk_path_info.mahalabis_quality_measure.end();++i)
			f.printf("%.06f %.06e\n", timestampTotime_t(i->first),i->second);

	}

	// ---------------------------------------------
	// Save other sensors GT predicted path
	// ---------------------------------------------
	{
		vector_string  the_labels = theMainWindow->AskForObservationByLabelMultiple("Choose additional sensors to export GT path:");
		if( the_labels.size() == 0 )
			return;

		// Standard matrix used for all the sensors on the vehicle:
		CMatrixDouble	COV_sensor_local;
		COV_sensor_local.zeros(6,6);

		{
			CConfigFileMemory	cfg;
			rawlog.getCommentTextAsConfigFile( cfg );
			COV_sensor_local(0,0) = square( cfg.read_double("ERROR_SENSOR_POSES","std_x",1e-3) );
			COV_sensor_local(1,1) = square( cfg.read_double("ERROR_SENSOR_POSES","std_y",1e-3) );
			COV_sensor_local(2,2) = square( cfg.read_double("ERROR_SENSOR_POSES","std_z",1e-3) );
			COV_sensor_local(3,3) = square( DEG2RAD( cfg.read_double("ERROR_SENSOR_POSES","std_yaw",1e-3) ) );
			COV_sensor_local(4,4) = square( DEG2RAD( cfg.read_double("ERROR_SENSOR_POSES","std_pitch",1e-3) ) );
			COV_sensor_local(5,5) = square( DEG2RAD( cfg.read_double("ERROR_SENSOR_POSES","std_roll",1e-3) ) );
		}



		ASSERT_( size(COV_sensor_local,1)==6 && COV_sensor_local.isSquare() );

		vector_string::iterator				itStr;
		std::vector<FILE*>					outFiles( the_labels.size() );
		std::vector<FILE*>::iterator		itOutFiles;

		// Generate output files
		for( itStr = the_labels.begin(), itOutFiles = outFiles.begin(); itStr != the_labels.end(); itStr++, itOutFiles++ )
		{
			*itOutFiles = mrpt::system::os::fopen( format( "%s/GT_path_%s.txt", outputPath.c_str(), itStr->c_str() ), "wt" );
			ASSERT_(*itOutFiles);
		}

		wxBusyCursor    waitCursor;

		CRawlog::iterator itRawlog;
		for( itRawlog = rawlog.begin(); itRawlog != rawlog.end(); itRawlog++ )
		{
			if( itRawlog.getType() == mrpt::obs::CRawlog::etObservation )
			{
				CObservationPtr obs( *itRawlog );
				for( itStr = the_labels.begin(), itOutFiles = outFiles.begin(); itStr != the_labels.end(); itStr++, itOutFiles++ )
				{
					if( obs->sensorLabel == *itStr )
					{
						bool valid;
						CPose3D intRobotPose;
						robot_path.interpolate( obs->timestamp, intRobotPose, valid );
						if( !valid )
							continue;


						CPose3DPDFGaussian	veh_pose;
						veh_pose.mean.setFromValues(0,0,0,0,0,0);
						if (size(rtk_path_info.W_star,1)==6 && size(rtk_path_info.W_star,2)==6)
						{
							// Uncertainty estimation:
							veh_pose.cov = rtk_path_info.W_star;
						}
						else
						{
							veh_pose.cov.setIdentity();
							veh_pose.cov*=1e-6;
						}

						// add the transformation of the vehicle:
						veh_pose.changeCoordinatesReference( intRobotPose );

						// add now that of the sensor:
						CPose3DPDFGaussian	sensor_on_the_vehicle;
						obs->getSensorPose( sensor_on_the_vehicle.mean );
						sensor_on_the_vehicle.cov = COV_sensor_local;

						CPose3DPDFGaussian	global_sensor_pose = veh_pose + sensor_on_the_vehicle;  // Global sensor pose with uncert.

						// save the composed path:
						{
							CPose3D &p = global_sensor_pose.mean;
							mrpt::system::os::fprintf( *itOutFiles, "%.06f %.06f %.06f %.06f %.06f %.06f %.06f ",
								timestampTotime_t( obs->timestamp ) ,p.x(),p.y(),p.z(), p.yaw(),p.pitch(),p.roll() );

							CMatrixDouble66 &C = global_sensor_pose.cov;
							mrpt::system::os::fprintf( *itOutFiles, "%e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e\n",
								C(0,0),C(0,1),C(0,2),C(0,3),C(0,4),C(0,5),
								C(1,1),C(1,2),C(1,3),C(1,4),C(1,5),
								C(2,2),C(2,3),C(2,4),C(2,5),
								C(3,3),C(3,4),C(3,5),
								C(4,4),C(4,5),
								C(5,5) );
						}
					}  // end if
				} // end for
			} // end if
		} // end for

		for( itOutFiles = outFiles.begin(); itOutFiles != outFiles.end(); itOutFiles++ )
			mrpt::system::os::fclose( *itOutFiles );
	}

	WX_END_TRY
}

void CFormRawMap::OnbtnSaveObsPathClick(wxCommandEvent& )
{
}


void CFormRawMap::OnbtnView3DClick(wxCommandEvent& )
{
	COpenGLScene		scene;

	loadMapInto3DScene(scene);

	win3Dmap = CDisplayWindow3DPtr( new CDisplayWindow3D("Raw-map 3D preview") );
	COpenGLScenePtr the_scene = win3Dmap->get3DSceneAndLock();
	*the_scene = scene;
	win3Dmap->unlockAccess3DScene();
	win3Dmap->repaint();
}

void CFormRawMap::OnbtnHelpClick(wxCommandEvent& )
{
}
