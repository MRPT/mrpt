/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "CScanMatching.h"

//(*InternalHeaders(CScanMatching)
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/font.h>
#include <wx/intl.h>
#include <wx/image.h>
#include <wx/string.h>
//*)

#include <wx/app.h>
#include <wx/log.h>
#include <wx/msgdlg.h>
#include <wx/progdlg.h>
#include <wx/busyinfo.h>

#include "xRawLogViewerMain.h"
#include <mrpt/gui/CMyRedirector.h>


// General global variables:
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/slam/CICP.h>

#include <mrpt/gui/WxUtils.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace std;


//(*IdInit(CScanMatching)
const long CScanMatching::ID_STATICTEXT2 = wxNewId();
const long CScanMatching::ID_BITMAPBUTTON1 = wxNewId();
const long CScanMatching::ID_STATICTEXT3 = wxNewId();
const long CScanMatching::ID_TEXTCTRL2 = wxNewId();
const long CScanMatching::ID_STATICTEXT4 = wxNewId();
const long CScanMatching::ID_TEXTCTRL3 = wxNewId();
const long CScanMatching::ID_TEXTCTRL6 = wxNewId();
const long CScanMatching::ID_STATICTEXT5 = wxNewId();
const long CScanMatching::ID_RADIOBUTTON1 = wxNewId();
const long CScanMatching::ID_RADIOBUTTON2 = wxNewId();
const long CScanMatching::ID_TEXTCTRL4 = wxNewId();
const long CScanMatching::ID_TEXTCTRL9 = wxNewId();
const long CScanMatching::ID_NOTEBOOK1 = wxNewId();
const long CScanMatching::ID_TEXTCTRL7 = wxNewId();
const long CScanMatching::ID_PANEL1 = wxNewId();
const long CScanMatching::ID_STATICTEXT6 = wxNewId();
const long CScanMatching::ID_CUSTOM1 = wxNewId();
const long CScanMatching::ID_BUTTON1 = wxNewId();
const long CScanMatching::ID_CHECKBOX1 = wxNewId();
const long CScanMatching::ID_BUTTON2 = wxNewId();
const long CScanMatching::ID_STATICTEXT1 = wxNewId();
const long CScanMatching::ID_GAUGE1 = wxNewId();
const long CScanMatching::ID_PANEL5 = wxNewId();
const long CScanMatching::ID_PANEL3 = wxNewId();
const long CScanMatching::ID_TEXTCTRL1 = wxNewId();
const long CScanMatching::ID_PANEL4 = wxNewId();
const long CScanMatching::ID_SPLITTERWINDOW2 = wxNewId();
const long CScanMatching::ID_PANEL2 = wxNewId();
const long CScanMatching::ID_SPLITTERWINDOW1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(CScanMatching,wxDialog)
	//(*EventTable(CScanMatching)
	//*)
END_EVENT_TABLE()

CScanMatching::CScanMatching(wxWindow* parent,wxWindowID)
{
	//(*Initialize(CScanMatching)
	wxFlexGridSizer* FlexGridSizer4;
	wxStaticBoxSizer* StaticBoxSizer4;
	wxFlexGridSizer* FlexGridSizer10;
	wxFlexGridSizer* FlexGridSizer3;
	wxFlexGridSizer* FlexGridSizer5;
	wxFlexGridSizer* FlexGridSizer9;
	wxFlexGridSizer* FlexGridSizer2;
	wxFlexGridSizer* FlexGridSizer7;
	wxStaticBoxSizer* StaticBoxSizer3;
	wxFlexGridSizer* FlexGridSizer8;
	wxFlexGridSizer* FlexGridSizer13;
	wxFlexGridSizer* FlexGridSizer12;
	wxFlexGridSizer* FlexGridSizer6;
	wxStaticBoxSizer* StaticBoxSizer1;
	wxFlexGridSizer* FlexGridSizer1;
	wxFlexGridSizer* FlexGridSizer11;
	wxStaticBoxSizer* StaticBoxSizer5;
	
	Create(parent, wxID_ANY, _("Scan Matching Experimenting Module"), wxDefaultPosition, wxDefaultSize, wxCAPTION|wxDEFAULT_DIALOG_STYLE|wxSYSTEM_MENU|wxRESIZE_BORDER|wxCLOSE_BOX|wxMAXIMIZE_BOX|wxMINIMIZE_BOX, _T("wxID_ANY"));
	SetMinSize(wxSize(200,200));
	FlexGridSizer1 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer1->AddGrowableCol(0);
	FlexGridSizer1->AddGrowableRow(0);
	SplitterWindow1 = new wxSplitterWindow(this, ID_SPLITTERWINDOW1, wxDefaultPosition, wxDefaultSize, wxSP_3D|wxSP_LIVE_UPDATE, _T("ID_SPLITTERWINDOW1"));
	SplitterWindow1->SetMinSize(wxSize(50,50));
	SplitterWindow1->SetMinimumPaneSize(50);
	Panel1 = new wxPanel(SplitterWindow1, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
	Panel1->SetMinSize(wxSize(100,-1));
	FlexGridSizer2 = new wxFlexGridSizer(4, 1, 0, 0);
	FlexGridSizer2->AddGrowableCol(0);
	FlexGridSizer2->AddGrowableRow(1);
	FlexGridSizer2->AddGrowableRow(2);
	FlexGridSizer2->AddGrowableRow(3);
	StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, Panel1, _("Observation indexes"));
	FlexGridSizer5 = new wxFlexGridSizer(2, 1, 0, 0);
	FlexGridSizer5->AddGrowableCol(0);
	FlexGridSizer13 = new wxFlexGridSizer(0, 2, 0, 0);
	FlexGridSizer13->AddGrowableCol(0);
	StaticText2 = new wxStaticText(Panel1, ID_STATICTEXT2, _("Enter the rawlog indexes of the observations\n to use for generating the maps to align:"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT2"));
	FlexGridSizer13->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	btnHelp = new wxBitmapButton(Panel1, ID_BITMAPBUTTON1, wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_INFORMATION")),wxART_BUTTON), wxDefaultPosition, wxDefaultSize, wxBU_AUTODRAW, wxDefaultValidator, _T("ID_BITMAPBUTTON1"));
	FlexGridSizer13->Add(btnHelp, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer5->Add(FlexGridSizer13, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer6 = new wxFlexGridSizer(0, 2, 0, 0);
	FlexGridSizer6->AddGrowableCol(1);
	StaticText3 = new wxStaticText(Panel1, ID_STATICTEXT3, _("First (reference):"), wxDefaultPosition, wxDefaultSize, wxALIGN_RIGHT, _T("ID_STATICTEXT3"));
	FlexGridSizer6->Add(StaticText3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	edFirst = new wxTextCtrl(Panel1, ID_TEXTCTRL2, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL2"));
	FlexGridSizer6->Add(edFirst, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticText4 = new wxStaticText(Panel1, ID_STATICTEXT4, _("Second (to align with first):"), wxDefaultPosition, wxDefaultSize, wxALIGN_RIGHT, _T("ID_STATICTEXT4"));
	FlexGridSizer6->Add(StaticText4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	edSecond = new wxTextCtrl(Panel1, ID_TEXTCTRL3, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL3"));
	FlexGridSizer6->Add(edSecond, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer5->Add(FlexGridSizer6, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	StaticBoxSizer1->Add(FlexGridSizer5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer2->Add(StaticBoxSizer1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticBoxSizer5 = new wxStaticBoxSizer(wxHORIZONTAL, Panel1, _("Config of the ICP algorithm:"));
	edOptICP = new wxTextCtrl(Panel1, ID_TEXTCTRL6, _("; Options for CICP::TConfigParams. See the MRPT reference for further information\n; -------------------------------------------------------------------------------------\n[InitialPosition]\nx=0 ; The initial position for the iterative ICP algorithm\ny=0 ; x/y in meters, phi in degrees\nphi_DEG=0\n\n[ICP]\n; The maximum number of iterations to execute if convergence is not achieved before\nmaxIterations=40 \n\n; Initial maximum distance for matching a pair of points\nthresholdDist=0.75 \n\n; An angular factor (in degrees) to increase the matching distance for distant points.\nthresholdAng_DEG=0.15 \n\n; After convergence, the thresholds are multiplied by this constant and ICP keep running (provides finer matching)\nALFA=0.30 \n\n; This is the smallest the distance threshold can become after stopping ICP and accepting the result.\nsmallestThresholdDist=0.10\n\n; 1: Use the closest points only, 0: Use all the correspondences within the threshold (more robust sometimes, but slower)\nonlyClosestCorrespondences=1 \n\n; This is the variance in X & Y of the input points, used to scale\n; the covariance of the output estimation (given as a 2D+heading Gaussian).\n; Refer to ScanMatching::leastSquareErrorRigidTransformation\ncovariance_varPoints=0.004\n\n\n; Whether to perform a RANSAC step after convergence, to improve\n; the result. \ndoRANSAC=0\n\n; Parameters for the RANSAC step. See reference for ScanMatching::robustRigidTransformation\nransac_minSetSize=3\nransac_maxSetSize=20\nransac_mahalanobisDistanceThreshold=3.0\nransac_nSimulations=100\nnormalizationStd=0.02\n"), wxDefaultPosition, wxSize(300,87), wxTE_MULTILINE|wxHSCROLL|wxVSCROLL, wxDefaultValidator, _T("ID_TEXTCTRL6"));
	wxFont edOptICPFont(8,wxFONTFAMILY_TELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,_T("Monospace"),wxFONTENCODING_DEFAULT);
	edOptICP->SetFont(edOptICPFont);
	StaticBoxSizer5->Add(edOptICP, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer2->Add(StaticBoxSizer5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
	StaticBoxSizer3 = new wxStaticBoxSizer(wxHORIZONTAL, Panel1, _("Config of the first (reference) map:"));
	FlexGridSizer3 = new wxFlexGridSizer(2, 1, 0, 0);
	FlexGridSizer3->AddGrowableCol(0);
	FlexGridSizer3->AddGrowableRow(1);
	FlexGridSizer8 = new wxFlexGridSizer(1, 3, 0, 0);
	FlexGridSizer8->AddGrowableCol(0);
	StaticText5 = new wxStaticText(Panel1, ID_STATICTEXT5, _("Map type:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT5"));
	FlexGridSizer8->Add(StaticText5, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	rbPoint = new wxRadioButton(Panel1, ID_RADIOBUTTON1, _("Point map"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_RADIOBUTTON1"));
	rbPoint->SetValue(true);
	FlexGridSizer8->Add(rbPoint, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	rbGrid = new wxRadioButton(Panel1, ID_RADIOBUTTON2, _("Occupancy grid map"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_RADIOBUTTON2"));
	FlexGridSizer8->Add(rbGrid, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer3->Add(FlexGridSizer8, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	Notebook1 = new wxNotebook(Panel1, ID_NOTEBOOK1, wxDefaultPosition, wxDefaultSize, 0, _T("ID_NOTEBOOK1"));
	edOptRefPnt = new wxTextCtrl(Notebook1, ID_TEXTCTRL4, _("; Configuration for CPointsMap::TInsertionOptions - See MRPT reference for help\n; ---------------------------------------------------------------------------------\n[InsertionOptions]\nminDistBetweenLaserPoints=0.05 ; The minimum distance between points (in 3D): If two points are too close, one of them is not inserted into the map.\nalso_interpolate=0 ; If set to true, far points (<1m) are interpolated with samples at minDistSqrBetweenLaserPoints intervals (Default is false).\ndisableDeletion=1 ; If set to false (default) points in the same plane as the inserted scan and inside the free space, are erased.\nfuseWithExisting=0 ; If set to true (default), inserted points are fused with previously existent ones.\nisPlanarMap=1 ; If set to true, only HORIZONTAL (i.e. XY plane) measurements will be inserted in the map. Default value is false, thus 3D maps are generated\nmaxDistForInterpolatePoints=1 ; The maximum distance between two points to interpolate between them (ONLY when also_interpolate=true).\n"), wxDefaultPosition, wxSize(300,87), wxTE_MULTILINE|wxHSCROLL|wxVSCROLL, wxDefaultValidator, _T("ID_TEXTCTRL4"));
	wxFont edOptRefPntFont(8,wxFONTFAMILY_TELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,_T("Monospace"),wxFONTENCODING_DEFAULT);
	edOptRefPnt->SetFont(edOptRefPntFont);
	edOptRefGrid = new wxTextCtrl(Notebook1, ID_TEXTCTRL9, _("; Configuration for COccupancyGridMap2D::TInsertionOptions - See MRPT reference for help\n; ---------------------------------------------------------------------------------\n[Construction]\nresolution=0.05 ; The grid cell size, in meters.\n\n[InsertionOptions]\nmapAltitude=0 ; The height (z axis) over the ground at which this grid map is at\nuseMapAltitude=0 ; If this is set to 1, only observations that match the grid height are inserted in the map.\nmaxDistanceInsertion=40 ; The maximum distance from the scan origin for updating cells, in meters\nmaxOccupancyUpdateCertainty=0.8 ; The \"certainty\" used to update the cells using the Bayes update formulas. Values are in the range 0.5 (No update) to 1.0 (last value is taken forgetting all old readings of that cell).\nconsiderInvalidRangesAsFreeSpace=1 ; Whether to consider \"lost-ranges\" as free space.\n\n"), wxDefaultPosition, wxSize(300,87), wxTE_MULTILINE|wxHSCROLL|wxVSCROLL, wxDefaultValidator, _T("ID_TEXTCTRL9"));
	wxFont edOptRefGridFont(8,wxFONTFAMILY_TELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,_T("Monospace"),wxFONTENCODING_DEFAULT);
	edOptRefGrid->SetFont(edOptRefGridFont);
	Notebook1->AddPage(edOptRefPnt, _("Point map config"), false);
	Notebook1->AddPage(edOptRefGrid, _("Grid map config"), false);
	FlexGridSizer3->Add(Notebook1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
	StaticBoxSizer3->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
	FlexGridSizer2->Add(StaticBoxSizer3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
	StaticBoxSizer4 = new wxStaticBoxSizer(wxHORIZONTAL, Panel1, _("Config of the second (to align) point map:"));
	edOptAlignMap = new wxTextCtrl(Panel1, ID_TEXTCTRL7, _("; Configuration for CPointsMap::TInsertionOptions - See MRPT reference for help\n; ---------------------------------------------------------------------------------\n[InsertionOptions]\nminDistBetweenLaserPoints=0.05 ; The minimum distance between points (in 3D): If two points are too close, one of them is not inserted into the map.\nalso_interpolate=0 ; If set to true, far points (<1m) are interpolated with samples at minDistSqrBetweenLaserPoints intervals (Default is false).\ndisableDeletion=1 ; If set to false (default) points in the same plane as the inserted scan and inside the free space, are erased.\nfuseWithExisting=0 ; If set to true (default), inserted points are fused with previously existent ones.\nisPlanarMap=1 ; If set to true, only HORIZONTAL (i.e. XY plane) measurements will be inserted in the map. Default value is false, thus 3D maps are generated\nmaxDistForInterpolatePoints=1 ; The maximum distance between two points to interpolate between them (ONLY when also_interpolate=true).\n"), wxDefaultPosition, wxSize(300,87), wxTE_MULTILINE|wxHSCROLL|wxVSCROLL, wxDefaultValidator, _T("ID_TEXTCTRL7"));
	wxFont edOptAlignMapFont(8,wxFONTFAMILY_TELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,_T("Monospace"),wxFONTENCODING_DEFAULT);
	edOptAlignMap->SetFont(edOptAlignMapFont);
	StaticBoxSizer4->Add(edOptAlignMap, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer2->Add(StaticBoxSizer4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
	Panel1->SetSizer(FlexGridSizer2);
	FlexGridSizer2->Fit(Panel1);
	FlexGridSizer2->SetSizeHints(Panel1);
	Panel2 = new wxPanel(SplitterWindow1, ID_PANEL2, wxDefaultPosition, wxSize(200,200), wxTAB_TRAVERSAL, _T("ID_PANEL2"));
	FlexGridSizer4 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer4->AddGrowableCol(0);
	FlexGridSizer4->AddGrowableRow(0);
	SplitterWindow2 = new wxSplitterWindow(Panel2, ID_SPLITTERWINDOW2, wxDefaultPosition, wxDefaultSize, wxSP_3D, _T("ID_SPLITTERWINDOW2"));
	SplitterWindow2->SetMinSize(wxSize(10,10));
	SplitterWindow2->SetMinimumPaneSize(10);
	Panel3 = new wxPanel(SplitterWindow2, ID_PANEL3, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL3"));
	FlexGridSizer9 = new wxFlexGridSizer(3, 1, 0, 0);
	FlexGridSizer9->AddGrowableCol(0);
	FlexGridSizer9->AddGrowableRow(1);
	StaticText6 = new wxStaticText(Panel3, ID_STATICTEXT6, _("Scan Matching Status:\n(Reference map: Blue or gridmap; Map to be aligned: Red)"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT6"));
	FlexGridSizer9->Add(StaticText6, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 3);
	plotMaps = new mpWindow(Panel3,ID_CUSTOM1,wxDefaultPosition,wxSize(341,359),0);
	FlexGridSizer9->Add(plotMaps, 1, wxALL|wxEXPAND|wxFIXED_MINSIZE|wxALIGN_LEFT|wxALIGN_TOP, 0);
	Panel5 = new wxPanel(Panel3, ID_PANEL5, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL5"));
	FlexGridSizer7 = new wxFlexGridSizer(2, 1, 0, 0);
	FlexGridSizer7->AddGrowableCol(0);
	FlexGridSizer7->AddGrowableRow(0);
	FlexGridSizer11 = new wxFlexGridSizer(1, 3, 0, 0);
	FlexGridSizer11->AddGrowableCol(1);
	btnRunICP = new wxButton(Panel5, ID_BUTTON1, _("Run ICP"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
	btnRunICP->SetDefault();
	FlexGridSizer11->Add(btnRunICP, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	cbAnimate = new wxCheckBox(Panel5, ID_CHECKBOX1, _("Animate step by step"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
	cbAnimate->SetValue(false);
	FlexGridSizer11->Add(cbAnimate, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	btnClose = new wxButton(Panel5, ID_BUTTON2, _("CLOSE"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
	FlexGridSizer11->Add(btnClose, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer7->Add(FlexGridSizer11, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer12 = new wxFlexGridSizer(1, 2, 0, 0);
	FlexGridSizer12->AddGrowableCol(1);
	txtStep = new wxStaticText(Panel5, ID_STATICTEXT1, _("Step: 0 / 0"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
	FlexGridSizer12->Add(txtStep, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	pbSteps = new wxGauge(Panel5, ID_GAUGE1, 100, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_GAUGE1"));
	FlexGridSizer12->Add(pbSteps, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer7->Add(FlexGridSizer12, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	Panel5->SetSizer(FlexGridSizer7);
	FlexGridSizer7->Fit(Panel5);
	FlexGridSizer7->SetSizeHints(Panel5);
	FlexGridSizer9->Add(Panel5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	Panel3->SetSizer(FlexGridSizer9);
	FlexGridSizer9->Fit(Panel3);
	FlexGridSizer9->SetSizeHints(Panel3);
	Panel4 = new wxPanel(SplitterWindow2, ID_PANEL4, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL4"));
	FlexGridSizer10 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer10->AddGrowableCol(0);
	FlexGridSizer10->AddGrowableRow(0);
	txtLog = new wxTextCtrl(Panel4, ID_TEXTCTRL1, _("(Log messages)"), wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE|wxTE_READONLY|wxHSCROLL|wxVSCROLL, wxDefaultValidator, _T("ID_TEXTCTRL1"));
	wxFont txtLogFont(8,wxFONTFAMILY_TELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,_T("Monospace"),wxFONTENCODING_DEFAULT);
	txtLog->SetFont(txtLogFont);
	FlexGridSizer10->Add(txtLog, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	Panel4->SetSizer(FlexGridSizer10);
	FlexGridSizer10->Fit(Panel4);
	FlexGridSizer10->SetSizeHints(Panel4);
	SplitterWindow2->SplitHorizontally(Panel3, Panel4);
	FlexGridSizer4->Add(SplitterWindow2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	Panel2->SetSizer(FlexGridSizer4);
	FlexGridSizer4->SetSizeHints(Panel2);
	SplitterWindow1->SplitVertically(Panel1, Panel2);
	FlexGridSizer1->Add(SplitterWindow1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	SetSizer(FlexGridSizer1);
	FlexGridSizer1->Fit(this);
	FlexGridSizer1->SetSizeHints(this);
	Center();
	
	Connect(ID_BITMAPBUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CScanMatching::OnbtnHelpClick);
	Connect(ID_RADIOBUTTON1,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&CScanMatching::OChangeSelectedMapType);
	Connect(ID_RADIOBUTTON2,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&CScanMatching::OChangeSelectedMapType);
	Connect(ID_NOTEBOOK1,wxEVT_COMMAND_NOTEBOOK_PAGE_CHANGING,(wxObjectEventFunction)&CScanMatching::OnNotebook1PageChanging);
	Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CScanMatching::OnbtnICPClick);
	Connect(ID_CHECKBOX1,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&CScanMatching::OncbAnimateClick);
	Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CScanMatching::OnbtnCloseClick);
	//*)


	plotMaps->AddLayer( new mpScaleX() );
	plotMaps->AddLayer( new mpScaleY() );
	plotMaps->LockAspect( true);
	plotMaps->Fit(-10,10,-10,10);

	plotMaps->EnableDoubleBuffer(true);


	// Initially these controls are hiden:
	txtStep->Show( false );
	pbSteps->Show( false );



	Fit();


    wxLog *old_log = wxLog::SetActiveTarget( new wxLogTextCtrl( txtLog ) );
    delete old_log;

}

CScanMatching::~CScanMatching()
{
	//(*Destroy(CScanMatching)
	//*)
}


void CScanMatching::OnInit(wxInitDialogEvent& )
{
}


// Disable buttons while ICP is in animation:
class CMyButtonsDisabler
{
	CScanMatching *m_wnd;

	public:
	CMyButtonsDisabler( CScanMatching *obj ) : m_wnd(obj)
	{
		m_wnd->btnClose->Enable(false);
		m_wnd->btnRunICP->Enable(false);
	}

	~CMyButtonsDisabler()
	{
		m_wnd->btnClose->Enable(true);
		m_wnd->btnRunICP->Enable(true);
	}
};

// Perform the ICP:
void CScanMatching::OnbtnICPClick(wxCommandEvent&)
{
	WX_START_TRY

	// The reference and the new maps:
	CSimplePointsMap		refMapPt;
	COccupancyGridMap2D 	refMapGrid;
	CSimplePointsMap        newMapPt;
	CICP					icp;
	CICP::TReturnInfo		icpInfo;

	// Get indexes for creating the maps:
	long refIndx;
	edFirst->GetValue().ToLong(&refIndx);
	long newIndx;
	edSecond->GetValue().ToLong(&newIndx);

	// Get the SFs, and check classes:
	if ( rawlog.getType( refIndx )!= CRawlog::etObservation && rawlog.getType( refIndx )!= CRawlog::etSensoryFrame )
	{
		wxMessageBox( _("The first index is not a sensory frame or an observation!\n Select an index that corresponds to observations"),_("Error"),wxOK, this);
		return;
	}
	if ( rawlog.getType( newIndx)!= CRawlog::etObservation && rawlog.getType( newIndx)!= CRawlog::etSensoryFrame )
	{
		wxMessageBox( _("The second index is not a sensory frame or an observation!\n Select an index that corresponds to observations"),_("Error"),wxOK, this);
		return;
	}

	CSerializablePtr obj_ref = rawlog.getAsGeneric( refIndx );
	CSerializablePtr obj_new = rawlog.getAsGeneric( newIndx );

	CPosePDFPtr	poseEst;
	float		runTime;

	// Load ICP options:
	// ------------------------------------------
	CConfigFileMemory	icpCfg( CStringList( string( edOptICP->GetValue().mb_str() ) ) );
	icp.options.loadFromConfigFile( icpCfg,"ICP" );

	// EXTRA options:
	CPose2D  initialEst(
		icpCfg.read_float("InitialPosition","x",0,true),
		icpCfg.read_float("InitialPosition","y",0,true),
		DEG2RAD( icpCfg.read_float("InitialPosition","phi_DEG",0,true) ) );

	txtLog->Clear();

	// Redirect cout to the log text:
	//wxStreamToTextRedirector redirect( txtLog );

	CMyRedirector myRedirector( txtLog );

	icp.options.dumpToConsole();

	// Create reference map & load its options:
	// ------------------------------------------
	bool  useGridMap = rbGrid->GetValue();
	if (!useGridMap)
	{
		CConfigFileMemory	refCfg( CStringList( string( edOptRefPnt->GetValue().mb_str() ) ) );
		refMapPt.insertionOptions.loadFromConfigFile(refCfg,"InsertionOptions");
		cout << "REFERENCE MAP FOR THE ICP:" << endl;
		refMapPt.insertionOptions.dumpToConsole();
	}
	else
	{
		CConfigFileMemory	refCfg( CStringList( string( edOptRefGrid->GetValue().mb_str() ) ) );
		float		gridRes = refCfg.read_float("Construction","resolution",0.05f);
		refMapGrid.setSize(-10,10,-10,10,gridRes);
		refMapGrid.insertionOptions.loadFromConfigFile(refCfg,"InsertionOptions");

		cout << "REFERENCE MAP FOR THE ICP:" << endl;
		refMapGrid.insertionOptions.dumpToConsole();
	}

	CMetricMap *refMap = useGridMap ? (CMetricMap*)&refMapGrid : (CMetricMap*)&refMapPt;

	// Load new map options:
	// ----------------------------
	{
		CConfigFileMemory	refCfg( CStringList( string( edOptAlignMap->GetValue().mb_str() ) ) );
		newMapPt.insertionOptions.loadFromConfigFile(refCfg,"InsertionOptions");

		cout << "NEW MAP (TO ALIGN) FOR THE ICP:" << endl;
		newMapPt.insertionOptions.dumpToConsole();
	}

	// Insert the observations:
	// --------------------------------------
	if (IS_CLASS(obj_ref,CSensoryFrame))
	{
		CSensoryFramePtr SF_ref = CSensoryFramePtr(obj_ref);
		SF_ref->insertObservationsInto(refMap);
	}
	else if (IS_DERIVED(obj_ref,CObservation))
	{
		CObservationPtr	obs_ref = CObservationPtr(obj_ref);
		refMap->insertObservation(obs_ref.pointer());
	}
	else THROW_EXCEPTION("Unexpected runtime class found.");

	CPose3D	newMapRobotPose( 0,0,0 );
	if (obj_new->GetRuntimeClass()==CLASS_ID(CSensoryFrame))
	{
		CSensoryFramePtr SF_new = CSensoryFramePtr(obj_new);
		SF_new->insertObservationsInto(&newMapPt, &newMapRobotPose);
	}
	else if (obj_new->GetRuntimeClass()->derivedFrom(CLASS_ID(CObservation)))
	{
		CObservationPtr	obs_new = CObservationPtr(obj_new);
		newMapPt.insertObservation(obs_new.pointer());
	}
	else THROW_EXCEPTION("Unexpected runtime class found.");


	// Create initial graphs:
	// --------------------------------------
	// Delete all existing draw layers:
	plotMaps->DelAllLayers(true,false);

	mpBitmapLayer	*lyRefGrid = NULL;
	mpFXYVector	    *lyRefPt   = NULL;
	if (useGridMap)
	{
		lyRefGrid = new mpBitmapLayer();
		CImage imgGrid;
		refMapGrid.getAsImage(imgGrid);
		wxImage *newBmp = mrpt::gui::MRPTImage2wxImage( imgGrid );
		double lx = refMapGrid.getXMax()-refMapGrid.getXMin();
		double ly = refMapGrid.getYMax()-refMapGrid.getYMin();
		lyRefGrid->SetBitmap(
			*newBmp,
			refMapGrid.getXMin(),
			refMapGrid.getYMin(),
			lx,
			ly );

		delete newBmp;
		plotMaps->AddLayer( lyRefGrid );
	}
	else
	{
		lyRefPt = new mpFXYVector(_("ref_map"));
		lyRefPt->SetPen( wxPen(wxColour(0,0,255),3) );
		lyRefPt->SetContinuity( false );

		vector<float> xs,ys;
		refMapPt.getAllPoints(xs,ys);
		lyRefPt->SetData( xs, ys );

		plotMaps->AddLayer( lyRefPt );
	}

	mpPolygon	    *lyNewPt;
	// The new map:
	{
		lyNewPt   = new mpPolygon(_("new_map"));
		lyNewPt->SetPen( wxPen(wxColour(255,0,0),3) );
		lyNewPt->SetContinuity( false );
		lyNewPt->ShowName(false);

		vector<float> xs,ys;
		newMapPt.getAllPoints(xs,ys);
		lyNewPt->setPoints( xs, ys, false );

		plotMaps->AddLayer( lyNewPt );
	}

	// The origin axis for the robot poses:
	mpPolygon	    *lyNewRobotPose;
	{
		lyNewRobotPose=new mpPolygon(_("Robot pose"));
		lyNewRobotPose->SetPen( wxPen(wxColor(0,0,0),2 ) );
		lyNewRobotPose->SetContinuity(true);
		lyNewRobotPose->ShowName(true);
		vector<float> xs,ys;
		xs.push_back(0); ys.push_back(0);
		xs.push_back(1); ys.push_back(0);
		xs.push_back(0); ys.push_back(0);
		xs.push_back(0); ys.push_back(1);
		lyNewRobotPose->setPoints(xs,ys, false);

		plotMaps->AddLayer( lyNewRobotPose );
	}


	// The uncertainty ellipse of the ICP estimation:
	mpCovarianceEllipse 	*lyEstimateEllipse;
	{
		lyEstimateEllipse=new mpCovarianceEllipse();
		lyEstimateEllipse->SetPen( wxPen(wxColor(0,0,0),2 ) );
		lyEstimateEllipse->SetContinuity(true);
		lyEstimateEllipse->ShowName(false);
		lyEstimateEllipse->SetQuantiles(3);

		plotMaps->AddLayer( lyEstimateEllipse );
	}



	// Add X/Y axis:
	plotMaps->AddLayer( new mpScaleX() );
	plotMaps->AddLayer( new mpScaleY() );



	// Align:
	// --------------------------------------
	bool isAnimation = cbAnimate->GetValue();
	int  maxSteps = icp.options.maxIterations;
	int  curStep = isAnimation ? 0:maxSteps;
	CPose2D		estMean;
	CMatrixDouble33 	estCov;


	{
		wxBusyCursor    waitCursor;
		CMyButtonsDisabler   buttonsDisabler(this);

		cout << "===========" <<endl;
		cout << "  ICP RUN:" << endl;
		cout << "===========" <<endl;

		pbSteps->SetRange(maxSteps);


		while (curStep <= maxSteps)
		{
			if (isAnimation)
			{
				txtStep->SetLabel( _U( format("Step: %u / %u",curStep,maxSteps).c_str() ) );
				pbSteps->SetValue(curStep);
				wxTheApp->Yield();  // Let the app. process messages
			}

			icp.options.maxIterations = curStep;

			poseEst = icp.Align(
						  refMap,
						  (CMetricMap*)&newMapPt,
						  initialEst,
						  &runTime,
						  &icpInfo );

			// Show the final graphs:
			// --------------------------------------
			poseEst->getCovarianceAndMean(estCov,estMean);

			lyNewPt->SetCoordinateBase(estMean.x(),estMean.y(),estMean.phi());
			lyNewRobotPose->SetCoordinateBase(estMean.x(),estMean.y(),estMean.phi());

			lyEstimateEllipse->SetCoordinateBase(estMean.x(),estMean.y());
			lyEstimateEllipse->SetCovarianceMatrix( estCov(0,0), estCov(0,1), estCov(1,1) );

			plotMaps->UpdateAll();

			// Show text log:
			if (isAnimation)
			{
				cout << format("EXECUTING %i steps:\n---------------------------\n",curStep);
			}
			cout << format("Time:%fms\n",runTime*1e3f);
			cout << format("Iterations executed: %i\n",icpInfo.nIterations);
			cout << format("Goodness: %.02f%%\n",100*icpInfo.goodness);
			cout << format("Quality: %.04f\n",icpInfo.quality);

			// Already converged?
			if ( isAnimation && icpInfo.nIterations < (curStep-3) )
			{
				curStep = maxSteps;
				pbSteps->SetValue(maxSteps);
			}

			cout << format("Estimated pose:\n Mean=(%f,%f,%fdeg)\n",estMean.x(),estMean.y(),RAD2DEG(estMean.phi()));
			cout << " Covariance:\n";
			cout << estCov << endl;

			cout << " std(x)   = " << sqrt(estCov(0,0)) << " m. " << endl;
			cout << " std(y)   = " << sqrt(estCov(1,1)) << " m. " << endl;
			cout << " std(phi) = " << RAD2DEG(sqrt(estCov(2,2))) << " deg. " << endl;

			cout << format("Output PDF class is: %s\n",poseEst->GetRuntimeClass()->className );
			if (poseEst->GetRuntimeClass()==CLASS_ID(CPosePDFSOG))
			{
				CPosePDFSOGPtr SOG = CPosePDFSOGPtr(poseEst);
				size_t  i,n=SOG->size();
				cout << format("# of gaussians in SOG: %i\n",(int)n);
				for (i=0;i<n;i++)
				{
					cout << format("SOG[%02i]:w=%e mean=",(int)i, SOG->get(i).log_w)
					     << SOG->get(i).mean << endl;
				}

			}

			//delete poseEst; poseEst=NULL;

			if (isAnimation)
			{
				wxTheApp->Yield();  // Let the app. process messages
				::wxMilliSleep(100);
			}

			curStep++;
		} // end while

		// End of wait cursor
	}

	WX_END_TRY
}

void CScanMatching::OncbAnimateClick(wxCommandEvent&)
{
	bool show = cbAnimate->GetValue();
	txtStep->Show( show );
	pbSteps->Show( show );

	pbSteps->Fit();
	txtStep->Fit();
	Panel5->Fit();
}

void CScanMatching::OChangeSelectedMapType(wxCommandEvent&)
{
	Notebook1->ChangeSelection( rbPoint->GetValue() ? 0:1 );
}

void CScanMatching::OnNotebook1PageChanging(wxNotebookEvent& event)
{
	event.Veto();
}

void CScanMatching::OnbtnCloseClick(wxCommandEvent&)
{
	Close();
}

void CScanMatching::OnbtnHelpClick(wxCommandEvent&)
{
	wxString s;
	s << _("Scan matching tries to register two maps. Each of them is created by inserting observations,\n");
	s << _(" typically laser scans, into point or grid maps. You must enter here the indexes of the observations\n");
	s << _(" to be inserted in each map. You can select just one index, or several ones separated by spaces.\n");
	s << _(" All the observations will be inserted at (0,0,0) in each map.");

	wxMessageBox(s);
}
