/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include "hmt_slam_guiMain.h"
#include "MyArtProvider.h"
#include "CAboutBox.h"


#include <wx/msgdlg.h>

//(*InternalHeaders(hmt_slam_guiFrame)
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/intl.h>
#include <wx/font.h>
#include <wx/tglbtn.h>
#include <wx/bitmap.h>
#include <wx/icon.h>
#include <wx/image.h>
#include <wx/artprov.h>
//*)

#include <mrpt/utils.h>
#include <mrpt/system/filesystem.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::hmtslam;
using namespace mrpt::slam;
using namespace mrpt::system;
using namespace mrpt::poses;
using namespace mrpt::utils;


//(*IdInit(hmt_slam_guiFrame)
const long hmt_slam_guiFrame::ID_BUTTON1 = wxNewId();
const long hmt_slam_guiFrame::ID_STATICLINE3 = wxNewId();
const long hmt_slam_guiFrame::ID_BUTTON2 = wxNewId();
const long hmt_slam_guiFrame::ID_BUTTON3 = wxNewId();
const long hmt_slam_guiFrame::ID_STATICLINE1 = wxNewId();
const long hmt_slam_guiFrame::ID_BUTTON4 = wxNewId();
const long hmt_slam_guiFrame::ID_BUTTON6 = wxNewId();
const long hmt_slam_guiFrame::ID_STATICLINE2 = wxNewId();
const long hmt_slam_guiFrame::ID_BUTTON12 = wxNewId();
const long hmt_slam_guiFrame::ID_BUTTON10 = wxNewId();
const long hmt_slam_guiFrame::ID_BUTTON5 = wxNewId();
const long hmt_slam_guiFrame::ID_PANEL1 = wxNewId();
const long hmt_slam_guiFrame::ID_STATICTEXT1 = wxNewId();
const long hmt_slam_guiFrame::ID_TEXTCTRL1 = wxNewId();
const long hmt_slam_guiFrame::ID_BUTTON11 = wxNewId();
const long hmt_slam_guiFrame::ID_STATICTEXT6 = wxNewId();
const long hmt_slam_guiFrame::ID_TEXTCTRL2 = wxNewId();
const long hmt_slam_guiFrame::ID_PANEL3 = wxNewId();
const long hmt_slam_guiFrame::ID_STATICTEXT2 = wxNewId();
const long hmt_slam_guiFrame::ID_CHOICE1 = wxNewId();
const long hmt_slam_guiFrame::ID_TREECTRL1 = wxNewId();
const long hmt_slam_guiFrame::ID_PANEL15 = wxNewId();
const long hmt_slam_guiFrame::ID_PANEL17 = wxNewId();
const long hmt_slam_guiFrame::ID_PANEL16 = wxNewId();
const long hmt_slam_guiFrame::ID_NOTEBOOK2 = wxNewId();
const long hmt_slam_guiFrame::ID_STATICTEXT5 = wxNewId();
const long hmt_slam_guiFrame::ID_BUTTON7 = wxNewId();
const long hmt_slam_guiFrame::ID_BUTTON8 = wxNewId();
const long hmt_slam_guiFrame::ID_BUTTON9 = wxNewId();
const long hmt_slam_guiFrame::ID_PANEL14 = wxNewId();
const long hmt_slam_guiFrame::ID_PANEL8 = wxNewId();
const long hmt_slam_guiFrame::ID_PANEL5 = wxNewId();
const long hmt_slam_guiFrame::ID_STATICTEXT3 = wxNewId();
const long hmt_slam_guiFrame::ID_XY_GLCANVAS = wxNewId();
const long hmt_slam_guiFrame::ID_PANEL11 = wxNewId();
const long hmt_slam_guiFrame::ID_PANEL10 = wxNewId();
const long hmt_slam_guiFrame::ID_STATICTEXT4 = wxNewId();
const long hmt_slam_guiFrame::ID_CUSTOM1 = wxNewId();
const long hmt_slam_guiFrame::ID_PANEL13 = wxNewId();
const long hmt_slam_guiFrame::ID_PANEL12 = wxNewId();
const long hmt_slam_guiFrame::ID_SPLITTERWINDOW2 = wxNewId();
const long hmt_slam_guiFrame::ID_PANEL7 = wxNewId();
const long hmt_slam_guiFrame::ID_SPLITTERWINDOW1 = wxNewId();
const long hmt_slam_guiFrame::ID_PANEL4 = wxNewId();
const long hmt_slam_guiFrame::ID_NOTEBOOK1 = wxNewId();
const long hmt_slam_guiFrame::ID_PANEL2 = wxNewId();
const long hmt_slam_guiFrame::ID_MENUITEM1 = wxNewId();
const long hmt_slam_guiFrame::ID_MENUITEM2 = wxNewId();
const long hmt_slam_guiFrame::ID_MENUITEM3 = wxNewId();
const long hmt_slam_guiFrame::idMenuQuit = wxNewId();
const long hmt_slam_guiFrame::ID_MENUITEM6 = wxNewId();
const long hmt_slam_guiFrame::ID_MENUITEM4 = wxNewId();
const long hmt_slam_guiFrame::ID_MENUITEM5 = wxNewId();
const long hmt_slam_guiFrame::idMenuAbout = wxNewId();
const long hmt_slam_guiFrame::ID_STATUSBAR1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(hmt_slam_guiFrame,wxFrame)
    //(*EventTable(hmt_slam_guiFrame)
    //*)
END_EVENT_TABLE()

hmt_slam_guiFrame::hmt_slam_guiFrame(wxWindow* parent,wxWindowID id)  :
	m_hmtslam(NULL)
{
	// Load my custom icons:
#if wxCHECK_VERSION(2, 8, 0)
    wxArtProvider::Push(new CMyArtProvider);
#else
    wxArtProvider::PushProvider(new CMyArtProvider);
#endif


    //(*Initialize(hmt_slam_guiFrame)
    wxMenuItem* MenuItem2;
    wxMenuItem* MenuItem1;
    wxFlexGridSizer* FlexGridSizer8;
    wxFlexGridSizer* FlexGridSizer1;
    wxFlexGridSizer* FlexGridSizer2;
    wxFlexGridSizer* FlexGridSizer15;
    wxBoxSizer* BoxSizer3;
    wxMenu* Menu1;
    wxFlexGridSizer* FlexGridSizer17;
    wxFlexGridSizer* FlexGridSizer11;
    wxFlexGridSizer* FlexGridSizer7;
    wxFlexGridSizer* FlexGridSizer4;
    wxFlexGridSizer* FlexGridSizer9;
    wxFlexGridSizer* FlexGridSizer14;
    wxFlexGridSizer* FlexGridSizer6;
    wxFlexGridSizer* FlexGridSizer3;
    wxMenuItem* MenuItem5;
    wxFlexGridSizer* FlexGridSizer16;
    wxFlexGridSizer* FlexGridSizer10;
    wxBoxSizer* BoxSizer1;
    wxFlexGridSizer* FlexGridSizer13;
    wxMenuBar* MenuBar1;
    wxFlexGridSizer* FlexGridSizer18;
    wxMenuItem* MenuItem6;
    wxFlexGridSizer* FlexGridSizer12;
    wxMenu* Menu2;
    wxFlexGridSizer* FlexGridSizer5;
    wxMenuItem* MenuItem8;

    Create(parent, id, _("HTM-SLAM - Part of the MRPT project"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("id"));
    SetClientSize(wxSize(700,400));
    SetMinSize(wxSize(-1,300));
    {
    wxIcon FrameIcon;
    FrameIcon.CopyFromBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("MAIN_ICON")),wxART_FRAME_ICON));
    SetIcon(FrameIcon);
    }
    FlexGridSizer1 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer1->AddGrowableCol(0);
    FlexGridSizer1->AddGrowableRow(1);
    Panel1 = new wxPanel(this, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    FlexGridSizer2 = new wxFlexGridSizer(0, 12, 0, 0);
    FlexGridSizer2->AddGrowableCol(8);
    btnReset = new wxCustomButton(Panel1,ID_BUTTON1,_("Reset"),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_RESET")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM|wxCUSTBUT_FLAT,wxDefaultValidator,_T("ID_BUTTON1"));
    btnReset->SetBitmapDisabled(btnReset->CreateBitmapDisabled(btnReset->GetBitmapLabel()));
    btnReset->SetLabelMargin(wxSize(10,2));
    btnReset->SetBitmapMargin(wxSize(-1,3));
    FlexGridSizer2->Add(btnReset, 1, wxALL|wxALIGN_LEFT|wxALIGN_BOTTOM, 5);
    StaticLine3 = new wxStaticLine(Panel1, ID_STATICLINE3, wxDefaultPosition, wxSize(1,-1), wxLI_VERTICAL, _T("ID_STATICLINE3"));
    FlexGridSizer2->Add(StaticLine3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 5);
    btnLoad = new wxCustomButton(Panel1,ID_BUTTON2,_("Load state..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_LOAD")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM|wxCUSTBUT_FLAT,wxDefaultValidator,_T("ID_BUTTON2"));
    btnLoad->SetBitmapDisabled(btnLoad->CreateBitmapDisabled(btnLoad->GetBitmapLabel()));
    btnLoad->SetLabelMargin(wxSize(10,2));
    btnLoad->SetBitmapMargin(wxSize(-1,3));
    FlexGridSizer2->Add(btnLoad, 1, wxALL|wxALIGN_LEFT|wxALIGN_BOTTOM, 5);
    btnSave = new wxCustomButton(Panel1,ID_BUTTON3,_("Save state.."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_SAVE")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM|wxCUSTBUT_FLAT,wxDefaultValidator,_T("ID_BUTTON3"));
    btnSave->SetBitmapDisabled(btnSave->CreateBitmapDisabled(btnSave->GetBitmapLabel()));
    btnSave->SetLabelMargin(wxSize(10,2));
    btnSave->SetBitmapMargin(wxSize(-1,3));
    FlexGridSizer2->Add(btnSave, 1, wxALL|wxALIGN_LEFT|wxALIGN_BOTTOM, 5);
    StaticLine1 = new wxStaticLine(Panel1, ID_STATICLINE1, wxDefaultPosition, wxSize(1,-1), wxLI_VERTICAL, _T("ID_STATICLINE1"));
    FlexGridSizer2->Add(StaticLine1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 5);
    btnStart = new wxCustomButton(Panel1,ID_BUTTON4,_("START"),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_PLAY")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM|wxCUSTBUT_FLAT,wxDefaultValidator,_T("ID_BUTTON4"));
    btnStart->SetBitmapDisabled(btnStart->CreateBitmapDisabled(btnStart->GetBitmapLabel()));
    btnStart->SetLabelMargin(wxSize(10,2));
    btnStart->SetBitmapMargin(wxSize(-1,3));
    FlexGridSizer2->Add(btnStart, 1, wxALL|wxALIGN_LEFT|wxALIGN_BOTTOM, 5);
    btnPause = new wxCustomButton(Panel1,ID_BUTTON6,_("Pause"),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_STOP")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM|wxCUSTBUT_FLAT,wxDefaultValidator,_T("ID_BUTTON6"));
    btnPause->SetBitmapDisabled(btnPause->CreateBitmapDisabled(btnPause->GetBitmapLabel()));
    btnPause->SetLabelMargin(wxSize(10,2));
    btnPause->SetBitmapMargin(wxSize(-1,3));
    FlexGridSizer2->Add(btnPause, 1, wxALL|wxALIGN_LEFT|wxALIGN_BOTTOM, 5);
    StaticLine2 = new wxStaticLine(Panel1, ID_STATICLINE2, wxDefaultPosition, wxSize(1,-1), wxLI_VERTICAL, _T("ID_STATICLINE2"));
    FlexGridSizer2->Add(StaticLine2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 5);
    btnShowLogWin = new wxCustomButton(Panel1,ID_BUTTON12,_("Show log"),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_LOG")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM|wxCUSTBUT_FLAT,wxDefaultValidator,_T("ID_BUTTON12"));
    btnShowLogWin->SetBitmapDisabled(btnShowLogWin->CreateBitmapDisabled(btnShowLogWin->GetBitmapLabel()));
    btnShowLogWin->SetLabelMargin(wxSize(10,2));
    btnShowLogWin->SetBitmapMargin(wxSize(-1,3));
    FlexGridSizer2->Add(btnShowLogWin, 1, wxALL|wxALIGN_LEFT|wxALIGN_BOTTOM, 5);
    FlexGridSizer2->Add(-1,-1,1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 5);
    btnAbout = new wxCustomButton(Panel1,ID_BUTTON10,_("About..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_ABOUT")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM|wxCUSTBUT_FLAT,wxDefaultValidator,_T("ID_BUTTON10"));
    btnAbout->SetBitmapDisabled(btnAbout->CreateBitmapDisabled(btnAbout->GetBitmapLabel()));
    btnAbout->SetLabelMargin(wxSize(10,2));
    btnAbout->SetBitmapMargin(wxSize(-1,3));
    FlexGridSizer2->Add(btnAbout, 1, wxALL|wxALIGN_LEFT|wxALIGN_BOTTOM, 5);
    btnQuit = new wxCustomButton(Panel1,ID_BUTTON5,_("Quit"),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_QUIT")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM|wxCUSTBUT_FLAT,wxDefaultValidator,_T("ID_BUTTON5"));
    btnQuit->SetBitmapDisabled(btnQuit->CreateBitmapDisabled(btnQuit->GetBitmapLabel()));
    btnQuit->SetLabelMargin(wxSize(10,2));
    btnQuit->SetBitmapMargin(wxSize(-1,3));
    FlexGridSizer2->Add(btnQuit, 1, wxALL|wxALIGN_LEFT|wxALIGN_BOTTOM, 5);
    Panel1->SetSizer(FlexGridSizer2);
    FlexGridSizer2->Fit(Panel1);
    FlexGridSizer2->SetSizeHints(Panel1);
    FlexGridSizer1->Add(Panel1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 0);
    Panel2 = new wxPanel(this, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
    FlexGridSizer3 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer3->AddGrowableCol(0);
    FlexGridSizer3->AddGrowableRow(0);
    Notebook1 = new wxNotebook(Panel2, ID_NOTEBOOK1, wxDefaultPosition, wxDefaultSize, 0, _T("ID_NOTEBOOK1"));
    panConfig = new wxPanel(Notebook1, ID_PANEL3, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL3"));
    FlexGridSizer16 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer16->AddGrowableCol(0);
    FlexGridSizer16->AddGrowableRow(1);
    FlexGridSizer17 = new wxFlexGridSizer(1, 3, 0, 0);
    FlexGridSizer17->AddGrowableCol(1);
    StaticText1 = new wxStaticText(panConfig, ID_STATICTEXT1, _("Input rawlog file:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    FlexGridSizer17->Add(StaticText1, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    edInputRawlog = new wxTextCtrl(panConfig, ID_TEXTCTRL1, _("dataset.rawlog"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    FlexGridSizer17->Add(edInputRawlog, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 5);
    btnPickRawlog = new wxButton(panConfig, ID_BUTTON11, _("Pick..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON11"));
    FlexGridSizer17->Add(btnPickRawlog, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer16->Add(FlexGridSizer17, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 0);
    FlexGridSizer18 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer18->AddGrowableCol(0);
    FlexGridSizer18->AddGrowableRow(1);
    StaticText6 = new wxStaticText(panConfig, ID_STATICTEXT6, _("More parameters:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT6"));
    FlexGridSizer18->Add(StaticText6, 1, wxALL|wxALIGN_BOTTOM|wxALIGN_CENTER_HORIZONTAL, 5);
    edRestParams = new wxTextCtrl(panConfig, ID_TEXTCTRL2, _("//====================================================\n//               HMT-SLAM\n// Here come global parameters for the app.\n//====================================================\n[HMT-SLAM]\n\n// The directory where the log files will be saved (left in blank if no log is required)\nLOG_OUTPUT_DIR\t= LOG_HTMSLAM_MALAGA\n\nrawlog_offset\t= 0\t\t// Whether to skip some rawlog entries \nLOG_FREQUENCY\t= 20\t// The frequency of log files generation:\nLOG_SHOW3D\t\t= 1\nrandom_seed\t\t= 1234\t// 0:Randomize, !=0:use that seed.\n\n// --------------------------------\n// Local SLAM method selection:\n//   1: RBPF_2DLASER\n// --------------------------------\nSLAM_METHOD=1\n\n//SLAM_MIN_DIST_BETWEEN_OBS=1.0\t\t// Map updates threshold (meters)\n//SLAM_MIN_HEADING_BETWEEN_OBS_DEG=50\t// Map updates threshold (degrees)\n\nSLAM_MIN_DIST_BETWEEN_OBS=1.25\t\t// Map updates threshold (meters)\nSLAM_MIN_HEADING_BETWEEN_OBS_DEG=30\t// Map updates threshold (degrees)\n\nMIN_ODOMETRY_STD_XY\t\t= 0.05\t\t// Minimum sigma in odometry increments (meters)\nMIN_ODOMETRY_STD_PHI\t= 2\t\t\t// Minimum sigma in odometry increments (deg)\n\n// Loop closure detectors:\n// gridmaps\n// images\nTLC_DETECTORS=gridmaps\n\n// ====================================================\n//          TLC_GRIDMATCHING\n//\n//  Top. Loop-closure detector based on grid-matching\n// ====================================================\n[TLC_GRIDMATCHING]\nfeatsPerSquareMeter\t\t= 0.012\n\nthreshold_max\t\t\t= 0.20 \t\t// For considering candidate matches\nthreshold_delta\t\t\t= 0.09\n\nransac_prob_good_inliers = 0.9999999999  // Prob. of a good inliers (for the number of iterations).\n\nmaxKLd_for_merge        = 0.9\t\t// Merge of close SOG modes\n\nmin_ICP_goodness\t= 0.25\nmax_ICP_mahadist\t= 20 //10 // The maximum Mahalanobis distance between the initial and final poses in the ICP not to discard the hypothesis (default=10)\n\nransac_minSetSizeRatio\t= 0.15 // 0.20\n\nransac_mahalanobisDistanceThreshold\t= 6\t\t// amRobust method only\nransac_chi2_quantile\t= 0.5 \t\t\t\t// amModifiedRANSAC method only\n\nsave_feat_coors\t\t\t= 0\t\t// Dump correspondences to grid_feats\ndebug_save_map_pairs\t= 1\t\t// Save the pair of maps with the best correspondences\ndebug_show_corrs\t\t= 0\t\t// Debug output of graphs\n\n\n// ----------------------------------------------------------\n// All the params of the feature detectors/descriptors\n// ----------------------------------------------------------\nfeatsType\t\t\t= 1\t\t// 0: KLT, 1: Harris, 3: SIFT, 4: SURF\n\n// The feature descriptor to use: 0=detector already has descriptor, \n//  1= SIFT, 2=SURF, 4=Spin images, 8=Polar images, 16=log-polar images \nfeature_descriptor\t\t= 8\n\npatchSize\t\t\t= 0   \t// Not needed\n\nKLTOptions.min_distance\t\t= 6\t\t\t// Pixels\nKLTOptions.threshold\t\t= 0.01 // 0.10  // 0.20\n\nharrisOptions.min_distance\t= 6\t\t\t// Pixels\nharrisOptions.threshold \t= 0.10  // 0.20\n\nSIFTOptions.implementation\t= 3\t\t\t// Hess\n\nSURFOptions.rotation_invariant\t= 1\t\t// 0=64 dims, 1=128dims\n\nSpinImagesOptions.hist_size_distance\t= 10 \nSpinImagesOptions.hist_size_intensity\t= 10 \nSpinImagesOptions.radius\t\t\t= 20\n\nPolarImagesOptions.bins_angle\t\t\t= 8\nPolarImagesOptions.bins_distance\t\t= 6\nPolarImagesOptions.radius\t\t\t= 40\n\nLogPolarImagesOptions.radius\t\t\t= 20\nLogPolarImagesOptions.num_angles\t\t= 8\n\n\n\n// ====================================================\n//\n//            \tPARTICLE_FILTER\n//\n//  Parameters of the PARTICLE FILTER within each LMH,\n//   invoked & implemented in CLSLAM_RBPF_2DLASER\n// ====================================================\n[PARTICLE_FILTER]\n//----------------------------------------------------------------------------------\n// The Particle Filter algorithm:\n//\t0: pfStandardProposal\n//\t1: pfAuxiliaryPFStandard\n//\t2: pfOptimalProposal      *** (ICP,...)\n//\t3: pfAuxiliaryPFOptimal\t  *** (Optimal SAMPLING)\n//\n// See: http://www.mrpt.org/Particle_Filters\n//----------------------------------------------------------------------------------\nPF_algorithm=3\n\nadaptiveSampleSize\t= 0\t\t// 0: Fixed # of particles, 1: KLD adaptive\n\n//----------------------------------------------------------------------------------\n// The Particle Filter Resampling method:\n//\t0: prMultinomial\n//\t1: prResidual\n//\t2: prStratified\n//\t3: prSystematic\n//\n// See: /docs/html/topic_resampling.html or     http://www.mrpt.org/    topic_resampling.html\n//----------------------------------------------------------------------------------\nresamplingMethod=0\npfAuxFilterOptimal_MaximumSearchSamples = 250\t\t// For PF algorithm=3\n\nsampleSize\t= 5\t\t// Number of particles (for fixed number algorithms)\nBETA\t\t= 0.50\t// Resampling ESS threshold\t\npowFactor\t= 0.01\t\t\t// A \"power factor\" for updating weights\t\t\t\n\n\n// ====================================================\n//\t\tGRAPH_CUT\n//\n//  Params for Area Abstraction (AA)\n// ====================================================\n[GRAPH_CUT]\npartitionThreshold                    = 0.6     // In the range [0,1]. Lower gives larger clusters.\nminDistForCorrespondence              = 0.50\nuseMapMatching                        = 1\nminimumNumberElementsEachCluster      = 5\n\n// ====================================================\n//\n//            MULTIMETRIC MAP CONFIGURATION\n//\n//  The params for creating the metric maps for \n//   each LMH.\n// ====================================================\n[MetricMaps]\n// Creation of maps:\noccupancyGrid_count\t\t\t= 1\ngasGrid_count\t\t\t\t= 0\nlandmarksMap_count\t\t\t= 0\nbeaconMap_count\t\t\t\t= 0\npointsMap_count\t\t\t\t= 1\n\n// Selection of map for likelihood: (fuseAll=-1,occGrid=0, points=1,landmarks=2,gasGrid=3)\nlikelihoodMapSelection\t\t= 0\n\n// Enables (1) / Disables (0) insertion into specific maps:\nenableInsertion_pointsMap\t= 1\nenableInsertion_landmarksMap= 1\nenableInsertion_beaconMap\t= 1\nenableInsertion_gridMaps\t= 1\nenableInsertion_gasGridMaps\t= 1\n\n// ====================================================\n//   MULTIMETRIC MAP: OccGrid #00\n// ====================================================\n// Creation Options for OccupancyGridMap 00:\n[MetricMaps_occupancyGrid_00_creationOpts]\nresolution=0.07\ndisableSaveAs3DObject=0\n\n\n// Insertion Options for OccupancyGridMap 00:\n[MetricMaps_occupancyGrid_00_insertOpts]\nmapAltitude\t\t\t\t\t\t\t= 0\nuseMapAltitude\t\t\t\t\t\t= 0\nmaxDistanceInsertion\t\t\t\t= 35\nmaxOccupancyUpdateCertainty\t\t\t= 0.60\nconsiderInvalidRangesAsFreeSpace\t= 1\nminLaserScanNoiseStd\t\t\t\t= 0.001\nhorizontalTolerance\t\t\t\t\t= 0.9 // In degrees\n\nCFD_features_gaussian_size\t\t\t= 3\nCFD_features_median_size\t\t\t= 3\n\n\n// Likelihood Options for OccupancyGridMap 00:\n[MetricMaps_occupancyGrid_00_likelihoodOpts]\nlikelihoodMethod\t\t\t\t= 4  // 0=MI, 1=Beam Model, 2=RSLC, 3=Cells Difs, 4=LF_Thrun, 5=LF_II\nLF_decimation\t\t\t\t\t= 4\nLF_stdHit\t\t\t\t\t\t= 0.10\nLF_maxCorrsDistance\t\t\t\t= 0.50\nLF_zHit\t\t\t\t\t\t\t= 0.999\nLF_zRandom\t\t\t\t\t\t= 0.001\nLF_maxRange\t\t\t\t\t\t= 60\nLF_alternateAverageMethod\t\t= 0\nenableLikelihoodCache\t\t\t= 1\n\n// ====================================================\n//   MULTIMETRIC MAP: PointMap #00\n// ====================================================\n// Creation Options for Pointsmap 00:\n// Creation Options for OccupancyGridMap 00:\n[MetricMaps_PointsMap_00_creationOpts]\ndisableSaveAs3DObject=0\n\n[MetricMaps_PointsMap_00_insertOpts]\nminDistBetweenLaserPoints=0.05  // The minimum distance between points (in 3D): If two points are too close, one of them is not inserted into the map.\nisPlanarMap=0                   // If set to true, only HORIZONTAL (i.e. XY plane) measurements will be inserted in the map. Default value is false, thus 3D maps are generated\n"), wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER|wxTE_PROCESS_TAB|wxTE_MULTILINE|wxHSCROLL|wxALWAYS_SHOW_SB, wxDefaultValidator, _T("ID_TEXTCTRL2"));
    wxFont edRestParamsFont(8,wxFONTFAMILY_TELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    edRestParams->SetFont(edRestParamsFont);
    FlexGridSizer18->Add(edRestParams, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 1);
    FlexGridSizer16->Add(FlexGridSizer18, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 0);
    panConfig->SetSizer(FlexGridSizer16);
    FlexGridSizer16->Fit(panConfig);
    FlexGridSizer16->SetSizeHints(panConfig);
    panMapView = new wxPanel(Notebook1, ID_PANEL4, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL4"));
    FlexGridSizer4 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer4->AddGrowableCol(0);
    FlexGridSizer4->AddGrowableRow(0);
    SplitterWindow1 = new wxSplitterWindow(panMapView, ID_SPLITTERWINDOW1, wxPoint(176,320), wxDefaultSize, wxSP_3D|wxSP_LIVE_UPDATE, _T("ID_SPLITTERWINDOW1"));
    SplitterWindow1->SetMinSize(wxSize(10,10));
    SplitterWindow1->SetMinimumPaneSize(10);
    Panel3 = new wxPanel(SplitterWindow1, ID_PANEL5, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL5"));
    FlexGridSizer5 = new wxFlexGridSizer(3, 1, 0, 0);
    FlexGridSizer5->AddGrowableCol(0);
    FlexGridSizer5->AddGrowableRow(1);
    FlexGridSizer7 = new wxFlexGridSizer(0, 2, 0, 0);
    StaticText2 = new wxStaticText(Panel3, ID_STATICTEXT2, _("Select hypothesis:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    FlexGridSizer7->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbHypos = new wxChoice(Panel3, ID_CHOICE1, wxDefaultPosition, wxDefaultSize, 0, 0, 0, wxDefaultValidator, _T("ID_CHOICE1"));
    FlexGridSizer7->Add(cbHypos, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer5->Add(FlexGridSizer7, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 0);
    Notebook2 = new wxNotebook(Panel3, ID_NOTEBOOK2, wxDefaultPosition, wxDefaultSize, wxNB_MULTILINE, _T("ID_NOTEBOOK2"));
    panTreeView = new wxPanel(Notebook2, ID_PANEL15, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL15"));
    FlexGridSizer15 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer15->AddGrowableCol(0);
    FlexGridSizer15->AddGrowableRow(0);
    treeView = new wxTreeCtrl(panTreeView, ID_TREECTRL1, wxDefaultPosition, wxDefaultSize, wxTR_LINES_AT_ROOT|wxTR_MULTIPLE|wxTR_DEFAULT_STYLE|wxVSCROLL|wxHSCROLL, wxDefaultValidator, _T("ID_TREECTRL1"));
    FlexGridSizer15->Add(treeView, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 0);
    panTreeView->SetSizer(FlexGridSizer15);
    FlexGridSizer15->Fit(panTreeView);
    FlexGridSizer15->SetSizeHints(panTreeView);
    Panel15 = new wxPanel(Notebook2, ID_PANEL17, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL17"));
    Panel14 = new wxPanel(Notebook2, ID_PANEL16, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL16"));
    Notebook2->AddPage(panTreeView, _("Tree view"), true);
    Notebook2->AddPage(Panel15, _("All nodes"), false);
    Notebook2->AddPage(Panel14, _("All arcs"), false);
    FlexGridSizer5->Add(Notebook2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 1);
    Panel8 = new wxPanel(Panel3, ID_PANEL8, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL8"));
    FlexGridSizer8 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer8->AddGrowableCol(0);
    FlexGridSizer8->AddGrowableRow(0);
    BoxSizer3 = new wxBoxSizer(wxHORIZONTAL);
    StaticText5 = new wxStaticText(Panel8, ID_STATICTEXT5, _("Edit the map"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT5"));
    wxFont StaticText5Font = wxSystemSettings::GetFont(wxSYS_SYSTEM_FONT);
    if ( !StaticText5Font.Ok() ) StaticText5Font = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
    StaticText5Font.SetPointSize((int)(StaticText5Font.GetPointSize() * 1.000000));
    StaticText5Font.SetWeight(wxFONTWEIGHT_BOLD);
    StaticText5->SetFont(StaticText5Font);
    BoxSizer3->Add(StaticText5, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer8->Add(BoxSizer3, 1, wxALL|wxALIGN_BOTTOM|wxALIGN_CENTER_HORIZONTAL, 0);
    Panel12 = new wxPanel(Panel8, ID_PANEL14, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL14"));
    FlexGridSizer14 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer14->AddGrowableCol(0);
    FlexGridSizer6 = new wxFlexGridSizer(0, 3, 0, 0);
    btnImportArea = new wxCustomButton(Panel12,ID_BUTTON7,_("Import area/metric map..."),wxNullBitmap,wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM|wxCUSTBUT_FLAT,wxDefaultValidator,_T("ID_BUTTON7"));
    btnImportArea->SetLabelMargin(wxSize(10,2));
    btnImportArea->SetBitmapMargin(wxSize(-1,3));
    FlexGridSizer6->Add(btnImportArea, 1, wxALL|wxALIGN_LEFT|wxALIGN_BOTTOM, 5);
    FlexGridSizer14->Add(FlexGridSizer6, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 0);
    FlexGridSizer9 = new wxFlexGridSizer(0, 3, 0, 0);
    btnAddNode = new wxCustomButton(Panel12,ID_BUTTON8,_("Add node..."),wxNullBitmap,wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM|wxCUSTBUT_FLAT,wxDefaultValidator,_T("ID_BUTTON8"));
    btnAddNode->SetLabelMargin(wxSize(10,2));
    btnAddNode->SetBitmapMargin(wxSize(-1,3));
    FlexGridSizer9->Add(btnAddNode, 1, wxALL|wxALIGN_LEFT|wxALIGN_BOTTOM, 5);
    btnAddArc = new wxCustomButton(Panel12,ID_BUTTON9,_("Add arc..."),wxNullBitmap,wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM|wxCUSTBUT_FLAT,wxDefaultValidator,_T("ID_BUTTON9"));
    btnAddArc->SetLabelMargin(wxSize(10,2));
    btnAddArc->SetBitmapMargin(wxSize(-1,3));
    FlexGridSizer9->Add(btnAddArc, 1, wxALL|wxALIGN_LEFT|wxALIGN_BOTTOM, 5);
    FlexGridSizer14->Add(FlexGridSizer9, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 0);
    Panel12->SetSizer(FlexGridSizer14);
    FlexGridSizer14->Fit(Panel12);
    FlexGridSizer14->SetSizeHints(Panel12);
    FlexGridSizer8->Add(Panel12, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 0);
    Panel8->SetSizer(FlexGridSizer8);
    FlexGridSizer8->Fit(Panel8);
    FlexGridSizer8->SetSizeHints(Panel8);
    FlexGridSizer5->Add(Panel8, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 0);
    Panel3->SetSizer(FlexGridSizer5);
    FlexGridSizer5->Fit(Panel3);
    FlexGridSizer5->SetSizeHints(Panel3);
    Panel4 = new wxPanel(SplitterWindow1, ID_PANEL7, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL7"));
    BoxSizer1 = new wxBoxSizer(wxHORIZONTAL);
    SplitterWindow2 = new wxSplitterWindow(Panel4, ID_SPLITTERWINDOW2, wxDefaultPosition, wxDefaultSize, wxSP_3D|wxSP_LIVE_UPDATE, _T("ID_SPLITTERWINDOW2"));
    SplitterWindow2->SetMinSize(wxSize(60,60));
    SplitterWindow2->SetMinimumPaneSize(60);
    Panel6 = new wxPanel(SplitterWindow2, ID_PANEL10, wxDefaultPosition, wxSize(100,100), wxTAB_TRAVERSAL, _T("ID_PANEL10"));
    Panel6->SetMinSize(wxSize(100,100));
    FlexGridSizer10 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer10->AddGrowableCol(0);
    FlexGridSizer10->AddGrowableRow(1);
    StaticText3 = new wxStaticText(Panel6, ID_STATICTEXT3, _("Global HMT map"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT3"));
    wxFont StaticText3Font = wxSystemSettings::GetFont(wxSYS_SYSTEM_FONT);
    if ( !StaticText3Font.Ok() ) StaticText3Font = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
    StaticText3Font.SetPointSize((int)(StaticText3Font.GetPointSize() * 1.000000));
    StaticText3Font.SetWeight(wxFONTWEIGHT_BOLD);
    StaticText3->SetFont(StaticText3Font);
    FlexGridSizer10->Add(StaticText3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel7 = new wxPanel(Panel6, ID_PANEL11, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL11"));
    FlexGridSizer11 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer11->AddGrowableCol(0);
    FlexGridSizer11->AddGrowableRow(0);
    m_glGlobalHMTMap = new CMyGLCanvas(Panel7,ID_XY_GLCANVAS,wxDefaultPosition,wxDefaultSize,wxTAB_TRAVERSAL,_T("ID_XY_GLCANVAS"));
    FlexGridSizer11->Add(m_glGlobalHMTMap, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 0);
    Panel7->SetSizer(FlexGridSizer11);
    FlexGridSizer11->Fit(Panel7);
    FlexGridSizer11->SetSizeHints(Panel7);
    FlexGridSizer10->Add(Panel7, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel6->SetSizer(FlexGridSizer10);
    FlexGridSizer10->SetSizeHints(Panel6);
    Panel10 = new wxPanel(SplitterWindow2, ID_PANEL12, wxDefaultPosition, wxSize(100,100), wxTAB_TRAVERSAL, _T("ID_PANEL12"));
    Panel10->SetMinSize(wxSize(100,100));
    FlexGridSizer12 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer12->AddGrowableCol(0);
    FlexGridSizer12->AddGrowableRow(1);
    StaticText4 = new wxStaticText(Panel10, ID_STATICTEXT4, _("Selected area local map"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT4"));
    wxFont StaticText4Font = wxSystemSettings::GetFont(wxSYS_SYSTEM_FONT);
    if ( !StaticText4Font.Ok() ) StaticText4Font = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
    StaticText4Font.SetPointSize((int)(StaticText4Font.GetPointSize() * 1.000000));
    StaticText4Font.SetWeight(wxFONTWEIGHT_BOLD);
    StaticText4->SetFont(StaticText4Font);
    FlexGridSizer12->Add(StaticText4, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel11 = new wxPanel(Panel10, ID_PANEL13, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL13"));
    FlexGridSizer13 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer13->AddGrowableCol(0);
    FlexGridSizer13->AddGrowableRow(0);
    m_glLocalArea = new CMyGLCanvas(Panel11,ID_CUSTOM1,wxDefaultPosition,wxDefaultSize,wxTAB_TRAVERSAL,_T("ID_CUSTOM1"));
    FlexGridSizer13->Add(m_glLocalArea, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 0);
    Panel11->SetSizer(FlexGridSizer13);
    FlexGridSizer13->Fit(Panel11);
    FlexGridSizer13->SetSizeHints(Panel11);
    FlexGridSizer12->Add(Panel11, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel10->SetSizer(FlexGridSizer12);
    FlexGridSizer12->SetSizeHints(Panel10);
    SplitterWindow2->SplitHorizontally(Panel6, Panel10);
    BoxSizer1->Add(SplitterWindow2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel4->SetSizer(BoxSizer1);
    BoxSizer1->Fit(Panel4);
    BoxSizer1->SetSizeHints(Panel4);
    SplitterWindow1->SplitVertically(Panel3, Panel4);
    FlexGridSizer4->Add(SplitterWindow1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 0);
    panMapView->SetSizer(FlexGridSizer4);
    FlexGridSizer4->Fit(panMapView);
    FlexGridSizer4->SetSizeHints(panMapView);
    Notebook1->AddPage(panConfig, _("SLAM parameters"), false);
    Notebook1->AddPage(panMapView, _("HMT-MAP view"), true);
    FlexGridSizer3->Add(Notebook1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 1);
    Panel2->SetSizer(FlexGridSizer3);
    FlexGridSizer3->Fit(Panel2);
    FlexGridSizer3->SetSizeHints(Panel2);
    FlexGridSizer1->Add(Panel2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 0);
    SetSizer(FlexGridSizer1);
    MenuBar1 = new wxMenuBar();
    Menu1 = new wxMenu();
    MenuItem3 = new wxMenuItem(Menu1, ID_MENUITEM1, _("Reset HMT map"), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(MenuItem3);
    Menu1->AppendSeparator();
    MenuItem4 = new wxMenuItem(Menu1, ID_MENUITEM2, _("Load state..."), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(MenuItem4);
    MenuItem5 = new wxMenuItem(Menu1, ID_MENUITEM3, _("Save state..."), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(MenuItem5);
    Menu1->AppendSeparator();
    MenuItem1 = new wxMenuItem(Menu1, idMenuQuit, _("Quit\tAlt-F4"), _("Quit the application"), wxITEM_NORMAL);
    Menu1->Append(MenuItem1);
    MenuBar1->Append(Menu1, _("&File"));
    Menu3 = new wxMenu();
    MenuItem8 = new wxMenuItem(Menu3, ID_MENUITEM6, _("Set parameters"), wxEmptyString, wxITEM_NORMAL);
    Menu3->Append(MenuItem8);
    Menu3->AppendSeparator();
    MenuItem6 = new wxMenuItem(Menu3, ID_MENUITEM4, _("Start SLAM"), wxEmptyString, wxITEM_NORMAL);
    Menu3->Append(MenuItem6);
    MenuItem7 = new wxMenuItem(Menu3, ID_MENUITEM5, _("Pause SLAM"), wxEmptyString, wxITEM_NORMAL);
    Menu3->Append(MenuItem7);
    MenuBar1->Append(Menu3, _("Map Building"));
    Menu2 = new wxMenu();
    MenuItem2 = new wxMenuItem(Menu2, idMenuAbout, _("About\tF1"), _("Show info about this application"), wxITEM_NORMAL);
    Menu2->Append(MenuItem2);
    MenuBar1->Append(Menu2, _("Help"));
    SetMenuBar(MenuBar1);
    StatusBar1 = new wxStatusBar(this, ID_STATUSBAR1, 0, _T("ID_STATUSBAR1"));
    int __wxStatusBarWidths_1[1] = { -1 };
    int __wxStatusBarStyles_1[1] = { wxSB_NORMAL };
    StatusBar1->SetFieldsCount(1,__wxStatusBarWidths_1);
    StatusBar1->SetStatusStyles(1,__wxStatusBarStyles_1);
    SetStatusBar(StatusBar1);
    FlexGridSizer1->SetSizeHints(this);

    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&hmt_slam_guiFrame::OnbtnResetClick);
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&hmt_slam_guiFrame::OnbtnLoadClick);
    Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&hmt_slam_guiFrame::OnbtnSaveClick);
    Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&hmt_slam_guiFrame::OnbtnStartClick);
    Connect(ID_BUTTON6,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&hmt_slam_guiFrame::OnbtnPauseClick);
    Connect(ID_BUTTON12,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&hmt_slam_guiFrame::OnbtnShowLogWinClick);
    Connect(ID_BUTTON10,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&hmt_slam_guiFrame::OnAbout);
    Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&hmt_slam_guiFrame::OnQuit);
    Connect(ID_BUTTON11,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&hmt_slam_guiFrame::OnbtnPickRawlogClick);
    Connect(ID_NOTEBOOK2,wxEVT_COMMAND_NOTEBOOK_PAGE_CHANGED,(wxObjectEventFunction)&hmt_slam_guiFrame::OnNotebook2PageChanged);
    Connect(ID_MENUITEM1,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&hmt_slam_guiFrame::OnbtnResetClick);
    Connect(ID_MENUITEM2,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&hmt_slam_guiFrame::OnbtnLoadClick);
    Connect(ID_MENUITEM3,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&hmt_slam_guiFrame::OnbtnSaveClick);
    Connect(idMenuQuit,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&hmt_slam_guiFrame::OnQuit);
    Connect(ID_MENUITEM6,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&hmt_slam_guiFrame::OnMenuSetSLAMParameter);
    Connect(ID_MENUITEM4,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&hmt_slam_guiFrame::OnbtnStartClick);
    Connect(ID_MENUITEM5,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&hmt_slam_guiFrame::OnbtnPauseClick);
    Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&hmt_slam_guiFrame::OnAbout);
    //*)



	// Initialize data ========================================================

	// Create log window:
	m_logWin = new CDlgLog(this);
	// { wxCommandEvent dum; OnbtnShowLogWinClick(dum); }

	cout << "Initializing HMT-SLAM visual application...\n";

	m_hmtslam = new CHMTSLAM();


	cout << "Initializing HMT-SLAM visual application DONE.\n";

	// Reset HMT map:
	{ wxCommandEvent dum; OnbtnResetClick(dum); }


	// Launch Thread:
	m_hThreadHMTSLAM = mrpt::system::createThreadFromObjectMethod(this, &hmt_slam_guiFrame::thread_HMTSLAM );

	// Set default size of the window:
    this->SetSize(600,500);
    this->Maximize();
}

hmt_slam_guiFrame::~hmt_slam_guiFrame()
{
	WX_START_TRY

	// Stop thread:
	m_thread_in_queue.push(new TThreadMsg(OP_QUIT_THREAD));
	mrpt::system::joinThread(m_hThreadHMTSLAM);

	delete_safe(m_hmtslam);

    //(*Destroy(hmt_slam_guiFrame)
    //*)

	WX_END_TRY
}

void hmt_slam_guiFrame::OnQuit(wxCommandEvent& event)
{
    Close();
}

void hmt_slam_guiFrame::OnAbout(wxCommandEvent& event)
{
	CAboutBox dlg(this);
	dlg.ShowModal();
}

void hmt_slam_guiFrame::OnNotebook2PageChanged(wxNotebookEvent& event)
{
}

void hmt_slam_guiFrame::loadHMTConfigFromSettings()
{
	std::string s;

	// From the text block:
	s+= std::string(edRestParams->GetValue().mb_str());

	CConfigFileMemory  cfg(s);

	// From GUI controls:
	cfg.write("HMT-SLAM","rawlog_file", std::string(this->edInputRawlog->GetValue().mb_str()) );

	// Load
	m_hmtslam->loadOptions(cfg);
}

// RESET =======
void hmt_slam_guiFrame::OnbtnResetClick(wxCommandEvent& event)
{
	WX_START_TRY

	this->loadHMTConfigFromSettings();
	m_hmtslam->initializeEmptyMap();
	updateAllMapViews();

	WX_END_TRY
}

void hmt_slam_guiFrame::OnbtnLoadClick(wxCommandEvent& event)
{
	string 	fil;
	if ( AskForOpenHMTMap( fil ) )
		loadHTMSLAMFromFile( fil );
}

bool hmt_slam_guiFrame::loadHTMSLAMFromFile( const std::string &filePath )
{
    WX_START_TRY

    if (!fileExists(filePath))
	{
		wxMessageBox(_U( string(string("File doesn't exist:\n")+filePath).c_str() ),_("Error loading file"),wxOK,this);
		return false;
	}

	wxBusyCursor busy;

	// Save the path
	WX_START_TRY
        string the_path( extractFileDirectory(filePath) );
        //iniFile->write(iniFileSect,"LastDir", the_path );
	WX_END_TRY

    // Load
    CFileGZInputStream(filePath) >> *m_hmtslam;

    m_curFileOpen = filePath;

    // Refresh views:
    // ---------------------------

    // The tree:
    rebuildTreeView();

    // The global map:
    updateGlobalMapView();

    return true;

    WX_END_TRY

    return false;
}


void hmt_slam_guiFrame::rebuildTreeView()
{
	WX_START_TRY

	wxBusyCursor        waitCursor;
	treeView->DeleteAllItems();

	treeView->SetQuickBestSize(true);

	// Root element & Areas:
	wxTreeItemId root = treeView->AddRoot(_("Areas"),0,-1,NULL);

	CHierarchicalMHMap::const_iterator it;
	size_t i;

	for (i=0,it=m_hmtslam->m_map.begin();it!=m_hmtslam->m_map.end();it++,i++)
	{
		string str = format( "Area %i", (int)it->second->getID() );

		//wxTreeItemId treeNode =
		treeView->AppendItem(
			root,
			_U(str.c_str()),
			0,-1,
			new CItemData(it->second,i));
	}

	treeView->ExpandAll();

	// List of hypotheses:
	cbHypos->Clear();

	for ( aligned_containers<THypothesisID, CLocalMetricHypothesis>::map_t::const_iterator l= m_hmtslam->m_LMHs.begin();l!=m_hmtslam->m_LMHs.end();++l)
		cbHypos->Append( _U( format("%i",(int)l->first).c_str()) );

	cbHypos->SetSelection(0);

    WX_END_TRY
}

//------------------------------------------------------------------------
//    Asks the user for a file, return false if user cancels
//------------------------------------------------------------------------
bool hmt_slam_guiFrame::AskForOpenHMTMap( std::string &fil )
{
	wxString caption = wxT("Choose a file to open");
	wxString wildcard = wxT("HMT-SLAM files (*.hmtslam;*.hmtslam.gz)|*.hmtslam;*.hmtslam.gz|All files (*.*)|*.*");
	wxString defaultDir; //( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
	wxString defaultFilename = wxT("");

	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_OPEN | wxFD_FILE_MUST_EXIST );

	if (dialog.ShowModal() == wxID_OK)
	{
		fil = string( dialog.GetPath().mb_str() );
		return true;
	}
	else return false;
}


void hmt_slam_guiFrame::OnbtnSaveClick(wxCommandEvent& event)
{
}

void hmt_slam_guiFrame::OnbtnStartClick(wxCommandEvent& event)
{
	m_thread_in_queue.push(new TThreadMsg(OP_START_SLAM));
}

void hmt_slam_guiFrame::OnbtnPauseClick(wxCommandEvent& event)
{
	m_thread_in_queue.push(new TThreadMsg(OP_PAUSE_SLAM));
}

void hmt_slam_guiFrame::OnMenuSetSLAMParameter(wxCommandEvent& event)
{
}

void hmt_slam_guiFrame::OnbtnPickRawlogClick(wxCommandEvent& event)
{
}

void hmt_slam_guiFrame::OnbtnShowLogWinClick(wxCommandEvent& event)
{
	if (m_logWin->IsVisible())
	{
		m_logWin->Hide();
		btnShowLogWin->SetValue( false );
		btnShowLogWin->Refresh();
	}
	else
	{
		m_logWin->Show();
		btnShowLogWin->SetValue( true );
		btnShowLogWin->Refresh();
	}

}
