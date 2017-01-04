/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include "navlog_viewer_GUI_designMain.h"
#include "CAboutBox.h"
#include <wx/msgdlg.h>

//(*InternalHeaders(navlog_viewer_GUI_designDialog)
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/tglbtn.h>
#include <wx/intl.h>
#include <wx/image.h>
#include <wx/string.h>
//*)

#include <wx/msgdlg.h>
#include <wx/filedlg.h>
#include <wx/busyinfo.h>
#include <wx/log.h>
#include <wx/textdlg.h>
#include <wx/dirdlg.h>
#include <wx/tipdlg.h>
#include <wx/statbox.h>

#include <mrpt/system.h>
#include <mrpt/utils/CFileInputStream.h>
#include <mrpt/utils/CConfigFileMemory.h>
#include <mrpt/utils/CConfigFilePrefixer.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/math/utils.h>
#include <mrpt/math/geometry.h> // intersect()
#include <mrpt/utils/printf_vector.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CMesh.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/stock_objects.h>

extern std::string global_fileToOpen;

const double fy = 9, Ay = 12;   // Font size & line spaces for GUI-overlayed text lines
#define ADD_WIN_TEXTMSG(__MSG) \
	win1->addTextMessage(5.0, 5 + (lineY++) * Ay, __MSG, mrpt::utils::TColorf(1, 1, 1), "mono", fy, mrpt::opengl::NICE, unique_id++);


using namespace std;
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::maps;
using namespace mrpt::poses;
using namespace mrpt::gui;
using namespace mrpt::nav;


//(*IdInit(navlog_viewer_GUI_designDialog)
const long navlog_viewer_GUI_designDialog::ID_BUTTON1 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT1 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_TEXTCTRL1 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_BUTTON2 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_BUTTON3 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_SLIDER1 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_BUTTON4 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_BUTTON5 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT9 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_TEXTCTRL3 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_PANEL2 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT2 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT3 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT4 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT5 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT6 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT7 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_PANEL3 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_RADIOBOX1 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_CHECKBOX1 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_CHECKBOX2 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_CHECKBOX3 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_CHECKBOX4 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_CHECKBOX5 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT8 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_TEXTCTRL2 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_BUTTON6 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_PANEL1 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_TIMER1 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_TIMER2 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_MENUITEM2 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_MENUITEM1 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_MENUITEM3 = wxNewId();
//*)

const long navlog_viewer_GUI_designDialog::ID_TIMER3 = wxNewId();

BEGIN_EVENT_TABLE(navlog_viewer_GUI_designDialog,wxFrame)
    //(*EventTable(navlog_viewer_GUI_designDialog)
    //*)
END_EVENT_TABLE()

#include "../wx-common/mrpt_logo.xpm"
#include "imgs/main_icon.xpm"

// A custom Art provider for customizing the icons:
class MyArtProvider : public wxArtProvider
{
protected:
    virtual wxBitmap CreateBitmap(const wxArtID& id,
                                  const wxArtClient& client,
                                  const wxSize& size)
	{
		if (id == wxART_MAKE_ART_ID(MAIN_ICON))   return wxBitmap(main_icon_xpm);
		if (id == wxART_MAKE_ART_ID(IMG_MRPT_LOGO))  return wxBitmap(mrpt_logo_xpm);

		// Any wxWidgets icons not implemented here will be provided by the default art provider.
		return wxNullBitmap;
	}
};


navlog_viewer_GUI_designDialog::navlog_viewer_GUI_designDialog(wxWindow* parent,wxWindowID id)
{
    // Load my custom icons:
#if wxCHECK_VERSION(2, 8, 0)
    wxArtProvider::Push(new MyArtProvider);
#else
    wxArtProvider::PushProvider(new MyArtProvider);
#endif


    //(*Initialize(navlog_viewer_GUI_designDialog)
    wxStaticBoxSizer* StaticBoxSizer2;
    wxFlexGridSizer* FlexGridSizer4;
    wxFlexGridSizer* FlexGridSizer3;
    wxMenuItem* mnuSaveScoreMatrix;
    wxFlexGridSizer* FlexGridSizer5;
    wxFlexGridSizer* FlexGridSizer2;
    wxFlexGridSizer* FlexGridSizer7;
    wxFlexGridSizer* FlexGridSizer8;
    wxFlexGridSizer* FlexGridSizer6;
    wxStaticBoxSizer* StaticBoxSizer1;
    wxFlexGridSizer* FlexGridSizer1;

    Create(parent, wxID_ANY, _("Navigation log viewer - Part of the MRPT project"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE, _T("wxID_ANY"));
    Move(wxPoint(20,20));
    FlexGridSizer1 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer1->AddGrowableCol(0);
    FlexGridSizer1->AddGrowableRow(0);
    Panel_AUX = new wxPanel(this, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    FlexGridSizer2 = new wxFlexGridSizer(3, 1, 0, 0);
    FlexGridSizer2->AddGrowableCol(0);
    FlexGridSizer3 = new wxFlexGridSizer(1, 5, 0, 0);
    FlexGridSizer3->AddGrowableCol(2);
    btnLoad = new wxCustomButton(Panel_AUX,ID_BUTTON1,_("Load log..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FOLDER")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxSize(70,55),wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON1"));
    btnLoad->SetBitmapDisabled(btnLoad->CreateBitmapDisabled(btnLoad->GetBitmapLabel()));
    btnLoad->SetBitmapMargin(wxSize(2,4));
    FlexGridSizer3->Add(btnLoad, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    StaticText1 = new wxStaticText(Panel_AUX, ID_STATICTEXT1, _("Loaded file:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    FlexGridSizer3->Add(StaticText1, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    edLogFile = new wxTextCtrl(Panel_AUX, ID_TEXTCTRL1, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE|wxTE_READONLY, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    FlexGridSizer3->Add(edLogFile, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnHelp = new wxCustomButton(Panel_AUX,ID_BUTTON2,_("About..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_HELP")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxSize(70,55),wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON2"));
    btnHelp->SetBitmapDisabled(btnHelp->CreateBitmapDisabled(btnHelp->GetBitmapLabel()));
    btnHelp->SetBitmapMargin(wxSize(20,4));
    FlexGridSizer3->Add(btnHelp, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnQuit = new wxCustomButton(Panel_AUX,ID_BUTTON3,_("Exit"),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_QUIT")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxSize(70,55),wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON3"));
    btnQuit->SetBitmapDisabled(btnQuit->CreateBitmapDisabled(btnQuit->GetBitmapLabel()));
    btnQuit->SetBitmapMargin(wxSize(20,4));
    FlexGridSizer3->Add(btnQuit, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer2->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer4 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer4->AddGrowableCol(0);
    StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, Panel_AUX, _("Browse the log:"));
    Panel1 = new wxPanel(Panel_AUX, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
    FlexGridSizer5 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer5->AddGrowableCol(0);
    FlexGridSizer6 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer6->AddGrowableCol(0);
    slidLog = new wxSlider(Panel1, ID_SLIDER1, 0, 0, 100, wxDefaultPosition, wxDefaultSize, wxSL_LABELS, wxDefaultValidator, _T("ID_SLIDER1"));
    FlexGridSizer6->Add(slidLog, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer5->Add(FlexGridSizer6, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer7 = new wxFlexGridSizer(0, 2, 0, 0);
    btnPlay = new wxButton(Panel1, ID_BUTTON4, _("Play"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON4"));
    FlexGridSizer7->Add(btnPlay, 1, wxALL|wxALIGN_TOP|wxALIGN_CENTER_HORIZONTAL, 5);
    btnStop = new wxButton(Panel1, ID_BUTTON5, _("Stop"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON5"));
    btnStop->Disable();
    FlexGridSizer7->Add(btnStop, 1, wxALL|wxALIGN_TOP|wxALIGN_CENTER_HORIZONTAL, 5);
    StaticText6 = new wxStaticText(Panel1, ID_STATICTEXT9, _("Animation delay (ms):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT9"));
    FlexGridSizer7->Add(StaticText6, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edAnimDelayMS = new wxTextCtrl(Panel1, ID_TEXTCTRL3, _("50"), wxDefaultPosition, wxSize(55,21), 0, wxDefaultValidator, _T("ID_TEXTCTRL3"));
    FlexGridSizer7->Add(edAnimDelayMS, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer5->Add(FlexGridSizer7, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel1->SetSizer(FlexGridSizer5);
    FlexGridSizer5->Fit(Panel1);
    FlexGridSizer5->SetSizeHints(Panel1);
    StaticBoxSizer1->Add(Panel1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer4->Add(StaticBoxSizer1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    flexGridRightHand = new wxFlexGridSizer(2, 1, 0, 0);
    flexGridRightHand->AddGrowableCol(0);
    flexGridRightHand->AddGrowableRow(0);
    flexGridRightHand->AddGrowableRow(1);
    StaticBoxSizer2 = new wxStaticBoxSizer(wxHORIZONTAL, Panel_AUX, _("Information:"));
    Panel3 = new wxPanel(Panel_AUX, ID_PANEL3, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL3"));
    FlexGridSizer9 = new wxFlexGridSizer(0, 2, 0, 0);
    StaticText2 = new wxStaticText(Panel3, ID_STATICTEXT2, _("Log entries:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    FlexGridSizer9->Add(StaticText2, 1, wxALL|wxALIGN_RIGHT|wxALIGN_TOP, 5);
    txtLogEntries = new wxStaticText(Panel3, ID_STATICTEXT3, _("0"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
    FlexGridSizer9->Add(txtLogEntries, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
    StaticText3 = new wxStaticText(Panel3, ID_STATICTEXT4, _("Duration:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT4"));
    FlexGridSizer9->Add(StaticText3, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    txtLogDuration = new wxStaticText(Panel3, ID_STATICTEXT5, _("0"), wxDefaultPosition, wxSize(80,-1), 0, _T("ID_STATICTEXT5"));
    FlexGridSizer9->Add(txtLogDuration, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
    StaticText4 = new wxStaticText(Panel3, ID_STATICTEXT6, _("Selected PTG:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT6"));
    FlexGridSizer9->Add(StaticText4, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    txtSelectedPTG = new wxStaticText(Panel3, ID_STATICTEXT7, _("-"), wxDefaultPosition, wxSize(80,-1), 0, _T("ID_STATICTEXT7"));
    FlexGridSizer9->Add(txtSelectedPTG, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
    Panel3->SetSizer(FlexGridSizer9);
    FlexGridSizer9->Fit(Panel3);
    FlexGridSizer9->SetSizeHints(Panel3);
    StaticBoxSizer2->Add(Panel3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    flexGridRightHand->Add(StaticBoxSizer2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    wxString __wxRadioBoxChoices_1[3] =
    {
    	_("TP-Obstacles only"),
    	_("+ phase1 score"),
    	_("+ phase2 score")
    };
    rbPerPTGPlots = new wxRadioBox(Panel_AUX, ID_RADIOBOX1, _(" Per PTG plots: "), wxDefaultPosition, wxDefaultSize, 3, __wxRadioBoxChoices_1, 1, wxRA_HORIZONTAL, wxDefaultValidator, _T("ID_RADIOBOX1"));
    flexGridRightHand->Add(rbPerPTGPlots, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    FlexGridSizer4->Add(flexGridRightHand, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer2->Add(FlexGridSizer4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer8 = new wxFlexGridSizer(0, 3, 0, 0);
    FlexGridSizer8->AddGrowableCol(0);
    FlexGridSizer8->AddGrowableCol(1);
    cbDrawShapePath = new wxCheckBox(Panel_AUX, ID_CHECKBOX1, _("Draw shape along path"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
    cbDrawShapePath->SetValue(true);
    FlexGridSizer8->Add(cbDrawShapePath, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbGlobalFrame = new wxCheckBox(Panel_AUX, ID_CHECKBOX2, _("Represent in global frame"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX2"));
    cbGlobalFrame->SetValue(true);
    FlexGridSizer8->Add(cbGlobalFrame, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbShowRelPoses = new wxCheckBox(Panel_AUX, ID_CHECKBOX3, _("Show delays model-based poses"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX3"));
    cbShowRelPoses->SetValue(true);
    FlexGridSizer8->Add(cbShowRelPoses, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbShowAllDebugEntries = new wxCheckBox(Panel_AUX, ID_CHECKBOX4, _("Show all debug fields"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX4"));
    cbShowAllDebugEntries->SetValue(true);
    FlexGridSizer8->Add(cbShowAllDebugEntries, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbShowXY = new wxCheckBox(Panel_AUX, ID_CHECKBOX5, _("Show cursor (X,Y) pos"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX5"));
    cbShowXY->SetValue(false);
    FlexGridSizer8->Add(cbShowXY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer8->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText5 = new wxStaticText(Panel_AUX, ID_STATICTEXT8, _("Shape draw min. dist:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT8"));
    FlexGridSizer8->Add(StaticText5, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edShapeMinDist = new wxTextCtrl(Panel_AUX, ID_TEXTCTRL2, _("1.0"), wxDefaultPosition, wxSize(55,21), 0, wxDefaultValidator, _T("ID_TEXTCTRL2"));
    FlexGridSizer8->Add(edShapeMinDist, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    btnMoreOps = new wxButton(Panel_AUX, ID_BUTTON6, _("More..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON6"));
    FlexGridSizer8->Add(btnMoreOps, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer2->Add(FlexGridSizer8, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel_AUX->SetSizer(FlexGridSizer2);
    FlexGridSizer2->Fit(Panel_AUX);
    FlexGridSizer2->SetSizeHints(Panel_AUX);
    FlexGridSizer1->Add(Panel_AUX, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    SetSizer(FlexGridSizer1);
    timPlay.SetOwner(this, ID_TIMER1);
    timAutoload.SetOwner(this, ID_TIMER2);
    timAutoload.Start(20, false);
    mnuSeePTGParams = new wxMenuItem((&mnuMoreOps), ID_MENUITEM2, _("See PTG params..."), wxEmptyString, wxITEM_NORMAL);
    mnuMoreOps.Append(mnuSeePTGParams);
    mnuMatlabPlots = new wxMenuItem((&mnuMoreOps), ID_MENUITEM1, _("Export map plot to MATLAB..."), wxEmptyString, wxITEM_NORMAL);
    mnuMoreOps.Append(mnuMatlabPlots);
    mnuSaveScoreMatrix = new wxMenuItem((&mnuMoreOps), ID_MENUITEM3, _("Save score matrices..."), wxEmptyString, wxITEM_NORMAL);
    mnuMoreOps.Append(mnuSaveScoreMatrix);
    FlexGridSizer1->Fit(this);
    FlexGridSizer1->SetSizeHints(this);

    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnbtnLoadClick);
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnbtnHelpClick);
    Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnbtnQuitClick);
    Connect(ID_SLIDER1,wxEVT_SCROLL_THUMBTRACK,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnslidLogCmdScroll);
    Connect(ID_SLIDER1,wxEVT_SCROLL_CHANGED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnslidLogCmdScroll);
    Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnbtnPlayClick);
    Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnbtnStopClick);
    Connect(ID_RADIOBOX1,wxEVT_COMMAND_RADIOBOX_SELECTED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnrbPerPTGPlotsSelect);
    Connect(ID_CHECKBOX1,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OncbGlobalFrameClick);
    Connect(ID_CHECKBOX2,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OncbGlobalFrameClick);
    Connect(ID_CHECKBOX3,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OncbGlobalFrameClick);
    Connect(ID_CHECKBOX4,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OncbGlobalFrameClick);
    Connect(ID_CHECKBOX5,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OncbShowXYClick);
    Connect(ID_BUTTON6,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnbtnMoreOpsClick);
    Connect(ID_TIMER1,wxEVT_TIMER,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OntimPlayTrigger);
    Connect(ID_TIMER2,wxEVT_TIMER,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OntimAutoloadTrigger);
    Connect(ID_MENUITEM2,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnmnuSeePTGParamsSelected);
    Connect(ID_MENUITEM1,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnmnuMatlabPlotsSelected);
    Connect(ID_MENUITEM3,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnmnuSaveScoreMatrixSelected);
    //*)

	timMouseXY.SetOwner(this, ID_TIMER3);
	Connect(ID_TIMER3, wxEVT_TIMER, (wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OntimMouseXY);

	cbShowAllDebugEntries->SetValue(false);

	{
    	wxIcon FrameIcon;
    	FrameIcon.CopyFromBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("MAIN_ICON")),wxART_FRAME_ICON));
    	SetIcon(FrameIcon);
    }

	m_log_first_tim = INVALID_TIMESTAMP;
	m_log_last_tim  = INVALID_TIMESTAMP;

	UpdateInfoFromLoadedLog(); // Disable some controls, etc..
}

navlog_viewer_GUI_designDialog::~navlog_viewer_GUI_designDialog()
{
    //(*Destroy(navlog_viewer_GUI_designDialog)
    //*)
	// Clean all windows:
	m_mywins.clear();
	m_mywins3D.clear();
	mrpt::system::sleep(100);
}

// ---------------------------------------------------------
// Load log file
// ---------------------------------------------------------
void navlog_viewer_GUI_designDialog::OnbtnLoadClick(wxCommandEvent& event)
{
	WX_START_TRY

	wxFileDialog dialog(
		this,
		_("Load log...") /*caption*/,
		_(".") /* defaultDir */,
		_("") /* defaultFilename */,
		_("Navigation logs (*.reactivenavlog;*.navlog)|*.reactivenavlog;*.navlog") /* wildcard */,
		wxFD_OPEN | wxFD_FILE_MUST_EXIST );
	if (dialog.ShowModal() != wxID_OK) return;

	const string filName(dialog.GetPath().mb_str());

	loadLogfile(filName);
	WX_END_TRY
}

void navlog_viewer_GUI_designDialog::loadLogfile(const std::string &filName)
{
	WX_START_TRY

	this->edLogFile->SetLabel(_U(filName.c_str()));

	CFileInputStream f(filName);

	m_logdata.clear();
	m_logdata_ptg_paths.clear();

	wxBusyCursor busy;

	set<string>  validClasses;
	validClasses.insert("CLogFileRecord");

	m_log_first_tim = INVALID_TIMESTAMP;
	m_log_last_tim  = INVALID_TIMESTAMP;

	for (;;)
	{
		try
		{
			CSerializablePtr obj = f.ReadObject();
			if (validClasses.find(string(obj->GetRuntimeClass()->className))==validClasses.end())
			{
				wxMessageBox(_U(format("Unexpected class found: %s",obj->GetRuntimeClass()->className).c_str()),_("Error loading log:"));
				break;
			}
			m_logdata.push_back(obj);

			// generate time stats:
			if (IS_CLASS(obj,CLogFileRecord))
			{
				const CLogFileRecordPtr logptr = CLogFileRecordPtr(obj);
				const auto it = logptr->timestamps.find("tim_start_iteration");
				if ( it != logptr->timestamps.end())
					m_log_last_tim = it->second;

				if (!logptr->infoPerPTG.empty())
				{
					size_t nPTGs = logptr->infoPerPTG.size();
					m_logdata_ptg_paths.resize(nPTGs);
					for (size_t i=0;i<nPTGs;i++)
						if (logptr->infoPerPTG[i].ptg)
							m_logdata_ptg_paths[i] = logptr->infoPerPTG[i].ptg;
				}
			}

			if (m_log_first_tim == INVALID_TIMESTAMP && m_log_last_tim!=INVALID_TIMESTAMP)
				m_log_first_tim = m_log_last_tim;
		}
		catch (CExceptionEOF &)
		{
			break;
		}
		catch (std::exception &e)
		{
			// EOF in the middle of an object... It may be usual if the logger is shut down not cleanly.
			wxMessageBox( wxString(e.what(),wxConvUTF8), wxT("Loading ended with an exception"), wxOK, this);
			break;
		}
	}

	// Update stats, etc...
	UpdateInfoFromLoadedLog();

	WX_END_TRY
}

void navlog_viewer_GUI_designDialog::OnbtnHelpClick(wxCommandEvent& event)
{
	CAboutBox dlg(NULL); // NULL so it gets centered on the screen
	dlg.ShowModal();
}

void navlog_viewer_GUI_designDialog::OnbtnQuitClick(wxCommandEvent& event)
{
	Close();
}

void navlog_viewer_GUI_designDialog::OnbtnPlayClick(wxCommandEvent& event)
{
	btnPlay->Enable(false);
	btnStop->Enable(true);

	long ms = 20;
	edAnimDelayMS->GetValue().ToLong(&ms);
	timPlay.Start(ms, true);
}

void navlog_viewer_GUI_designDialog::OnbtnStopClick(wxCommandEvent& event)
{
	btnPlay->Enable(true);
	btnStop->Enable(false);
}

void navlog_viewer_GUI_designDialog::OntimPlayTrigger(wxTimerEvent& event)
{
	if ( !btnStop->IsEnabled() ) return;

	int p = this->slidLog->GetValue();
	if ((p+1)<=this->slidLog->GetMax())
	{
		this->slidLog->SetValue(p+1);
		wxScrollEvent d;
		OnslidLogCmdScroll(d);
		// Next shot:
		long ms = 20;
		edAnimDelayMS->GetValue().ToLong(&ms);
		timPlay.Start(ms, true);
	}
	else
	{
		wxCommandEvent d;
		OnbtnStopClick(d);
	}
}


void navlog_viewer_GUI_designDialog::UpdateInfoFromLoadedLog()
{
	const size_t N = m_logdata.size();

	this->Panel1->Enable(N!=0);

	if (N>0)
	{
		this->txtLogEntries->SetLabel( wxString::Format(_("%u"),(unsigned)N ));
		this->slidLog->SetRange(0,N-1);
		this->slidLog->SetValue(0);
		wxScrollEvent d;
		OnslidLogCmdScroll(d);

		MRPT_TODO("Refactor this to handle different cmdvel types during one log");
#if 0
		CDisplayWindowPlotsPtr &win = m_mywins["VW"];
		if (!win)  {
			win= CDisplayWindowPlots::Create("Commanded v (red)/w (blue)",400,200);
			win->setPos(900,20);
			win->axis(-5,5,-5,5, true);
		}

		std::vector<std::vector<double> > cmd_vels;
		std::string sCmdVelTitle = "Velocity cmd: ";
		const char cols[4] = { 'r','b','k','g' };
		const char* cols_txt[4] = { "red","blue","black","green" };

		for (size_t i=0;i<N;i++)
		{
			CLogFileRecordPtr logptr = CLogFileRecordPtr(m_logdata[i]);
			const CLogFileRecord &log = *logptr;
			if (!log.cmd_vel) continue; // NOP cmd
			const size_t vel_len = log.cmd_vel->getVelCmdLength();
			if (i==0) {
				cmd_vels.resize( vel_len );
				for (size_t k = 0; k < vel_len; k++) {
					cmd_vels[k].resize(N);
					sCmdVelTitle += log.cmd_vel->getVelCmdDescription(k);
					sCmdVelTitle += mrpt::format(" (%s), ",cols_txt[k%4]);
				}
			}
			for (size_t k = 0; k < vel_len; k++)
				cmd_vels[k][i] = log.cmd_vel->getVelCmdElement(k);
		}
		win->clf();
		for (size_t i=0;i<cmd_vels.size();i++)
		{
			win->plot(cmd_vels[i],mrpt::format("%c-",cols[i%4]),mrpt::format("vc%u",(unsigned int)i));
			win->plot(cmd_vels[i],mrpt::format("%c3.",cols[i%4]),mrpt::format("vp%u",(unsigned int)i));
		}
		win->axis_fit();
		win->setWindowTitle(sCmdVelTitle);
#endif
	}

	std::string sDuration("???");
	if (m_log_first_tim != INVALID_TIMESTAMP && m_log_last_tim!=INVALID_TIMESTAMP)
	{
		sDuration = mrpt::system::intervalFormat( mrpt::system::timeDifference(m_log_first_tim,m_log_last_tim) );
	}
	this->txtLogDuration->SetLabel( _U(sDuration.c_str()));;

	flexGridRightHand->RecalcSizes();
	this->Fit();
}

// ---------------------------------------------
// 				DRAW ONE LOG RECORD
// ---------------------------------------------
void navlog_viewer_GUI_designDialog::OnslidLogCmdScroll(wxScrollEvent& event)
{
	WX_START_TRY

	const int i = this->slidLog->GetValue();

	// In the future, we could handle more log classes. For now, only "CLogFileRecordPtr":
	CLogFileRecordPtr logptr = CLogFileRecordPtr(m_logdata[i]);
	const CLogFileRecord &log = *logptr;

	txtSelectedPTG->SetLabel( wxString::Format(_("%u from [0-%u]"), static_cast<unsigned int>(log.nSelectedPTG), static_cast<unsigned int>(log.nPTGs-1) ) );

	// Draw WS-obstacles
	// --------------------------------
	{
		CDisplayWindow3DPtr &win1 = m_mywins3D["WS_obs"];
		if (!win1)  {
			win1= CDisplayWindow3D::Create("Sensed obstacles",500,400);
			win1->setPos(600,20);
			win1->setCameraAzimuthDeg(-90);
			win1->setCameraElevationDeg(90);
			{
				mrpt::opengl::COpenGLScenePtr scene;
				mrpt::gui::CDisplayWindow3DLocker  locker(*win1,scene);


				// XY ground plane:
				mrpt::opengl::CGridPlaneXYPtr gl_grid = mrpt::opengl::CGridPlaneXY::Create(-20,20, -20,20, 0, 1, 0.75f);
				gl_grid->setColor_u8( mrpt::utils::TColor(0xa0a0a0, 0x90) );
				scene->insert( gl_grid );

				// XYZ corner at origin:
				scene->insert( mrpt::opengl::stock_objects::CornerXYZSimple(1.0, 2.0) );
			}
		}

		// Update 3D view:
		{
			mrpt::opengl::COpenGLScenePtr scene;
			mrpt::gui::CDisplayWindow3DLocker  locker(*win1,scene);

			const CVectorFloat shap_x = log.robotShape_x, shap_y = log.robotShape_y;

			// Robot frame of reference:
			mrpt::opengl::CSetOfObjectsPtr gl_robot_frame;
			{
				mrpt::opengl::CRenderizablePtr gl_rbframe_r = scene->getByName("robot_frame");  // Get or create if new
				if (!gl_rbframe_r) {
					gl_robot_frame = mrpt::opengl::CSetOfObjects::Create();
					gl_robot_frame->setName("robot_frame");
					scene->insert(gl_robot_frame);
				} else {
					gl_robot_frame = mrpt::opengl::CSetOfObjectsPtr(gl_rbframe_r);
				}
				// Global or local coordinates?
				if (this->cbGlobalFrame->IsChecked()) {
					gl_robot_frame->setPose(  mrpt::poses::CPose3D(log.robotOdometryPose) );
					// Move the window focus:
					float px,py,pz;
					win1->getCameraPointingToPoint(px,py,pz);
					const float cam_zoom = win1->getCameraZoom();
					if ( log.robotOdometryPose.distance2DTo(px,py)>.3*cam_zoom )
						win1->setCameraPointingToPoint(log.robotOdometryPose.x(),log.robotOdometryPose.y(),0.0);
				} else {
					gl_robot_frame->setPose( mrpt::poses::CPose3D() );
				}
			}

			// Extrapolated poses from delay models:
			{
				mrpt::opengl::CSetOfObjectsPtr gl_relposes;
				mrpt::opengl::CRenderizablePtr gl_relposes_r = gl_robot_frame->getByName("relposes");  // Get or create if new
				if (!gl_relposes_r) {
					gl_relposes = mrpt::opengl::CSetOfObjects::Create();
					gl_relposes->setName("relposes");
					gl_robot_frame->insert(gl_relposes);
				}
				else {
					gl_relposes = mrpt::opengl::CSetOfObjectsPtr(gl_relposes_r);
				}

				gl_relposes->clear();
				if (cbShowRelPoses->IsChecked())
				{
					{
						mrpt::opengl::CSetOfObjectsPtr gl_relpose_sense = mrpt::opengl::stock_objects::CornerXYSimple(0.3f, 1);
						gl_relpose_sense->setName("sense");
						gl_relpose_sense->enableShowName(true);
						gl_relpose_sense->setPose(log.relPoseSense);
						gl_relposes->insert(gl_relpose_sense);
					}
					{
						mrpt::opengl::CSetOfObjectsPtr gl_relpose_cmdvel = mrpt::opengl::stock_objects::CornerXYSimple(0.3f, 1);
						gl_relpose_cmdvel->setName("cmdVel");
						gl_relpose_cmdvel->enableShowName(true);
						gl_relpose_cmdvel->setPose(log.relPoseVelCmd);
						gl_relposes->insert(gl_relpose_cmdvel);
					}
				}
			}

			{
				// Obstacles:
				mrpt::opengl::CPointCloudPtr gl_obs;
				mrpt::opengl::CRenderizablePtr gl_obs_r = gl_robot_frame->getByName("obs");  // Get or create if new
				if (!gl_obs_r) {
					gl_obs = mrpt::opengl::CPointCloud::Create();
					gl_obs->setName("obs");
					gl_obs->setPointSize(3.0);
					gl_obs->setColor_u8( mrpt::utils::TColor(0x00,0x00,0xff));
					gl_robot_frame->insert(gl_obs);
				} else {
					gl_obs = mrpt::opengl::CPointCloudPtr(gl_obs_r);
				}
				gl_obs->loadFromPointsMap(&log.WS_Obstacles);
				if (cbShowRelPoses->IsChecked())
						gl_obs->setPose(log.relPoseSense);
				else	gl_obs->setPose(mrpt::poses::CPose3D());
			}

			{
				// Selected PTG path:
				mrpt::opengl::CSetOfLinesPtr   gl_path;
				mrpt::opengl::CRenderizablePtr gl_path_r = gl_robot_frame->getByName("path");  // Get or create if new
				if (!gl_path_r) {
					gl_path = mrpt::opengl::CSetOfLines::Create();
					gl_path->setName("path");
					gl_path->setLineWidth(2.0);
					gl_path->setColor_u8( mrpt::utils::TColor(0x00,0x00,0xff) );
					gl_robot_frame->insert(gl_path);
				} else {
					gl_path = mrpt::opengl::CSetOfLinesPtr(gl_path_r);
				}
				const bool is_NOP_cmd = log.ptg_index_NOP >= 0;
				const size_t sel_ptg_idx = !is_NOP_cmd ? log.nSelectedPTG : log.ptg_index_NOP;
				if (m_logdata_ptg_paths.size()>sel_ptg_idx)
				{
					mrpt::nav::CParameterizedTrajectoryGeneratorPtr ptg = m_logdata_ptg_paths[sel_ptg_idx];
					if (ptg)
					{
						if (!ptg->isInitialized())
							ptg->initialize();

						// Set instantaneous kinematic state:
						if (!is_NOP_cmd)
								ptg->updateCurrentRobotVel(log.cur_vel_local);
						else	ptg->updateCurrentRobotVel(log.ptg_last_curRobotVelLocal);

						// Draw path:
						const int selected_k =
							log.ptg_index_NOP < 0 ?
							ptg->alpha2index(log.infoPerPTG[sel_ptg_idx].desiredDirection)
							:
							log.ptg_last_k_NOP;
						float max_dist = ptg->getRefDistance();
						gl_path->clear();
						ptg->add_robotShape_to_setOfLines(*gl_path);

						ptg->renderPathAsSimpleLine(selected_k,*gl_path,0.10, max_dist);
						gl_path->setColor_u8( mrpt::utils::TColor(0xff,0x00,0x00) );

						// PTG origin:
						// enable delays model?
						mrpt::poses::CPose2D ptg_origin = (cbShowRelPoses->IsChecked()) ? log.relPoseVelCmd : CPose2D();

						// "NOP cmd" case:
						if (log.ptg_index_NOP >= 0) {
							ptg_origin = ptg_origin - log.rel_cur_pose_wrt_last_vel_cmd_NOP;
						}

						gl_path->setPose(ptg_origin);

						// Overlay a sequence of robot shapes:
						if (cbDrawShapePath->IsChecked())
						{
							double min_shape_dists = 1.0;
							edShapeMinDist->GetValue().ToDouble(&min_shape_dists);

							for (double d=min_shape_dists;d<max_dist;d+=min_shape_dists)
							{
								uint16_t step;
								if (!ptg->getPathStepForDist(selected_k, d, step))
									continue;
								mrpt::math::TPose2D p;
								ptg->getPathPose(selected_k, step, p);
								ptg->add_robotShape_to_setOfLines(*gl_path, mrpt::poses::CPose2D(p) );
							}
						}
						{
							// Robot shape:
							mrpt::opengl::CSetOfLinesPtr   gl_shape;
							mrpt::opengl::CRenderizablePtr gl_shape_r = gl_robot_frame->getByName("shape");  // Get or create if new
							if (!gl_shape_r) {
								gl_shape = mrpt::opengl::CSetOfLines::Create();
								gl_shape->setName("shape");
								gl_shape->setLineWidth(4.0);
								gl_shape->setColor_u8( mrpt::utils::TColor(0xff,0x00,0x00) );
								gl_robot_frame->insert(gl_shape);
							} else {
								gl_shape = mrpt::opengl::CSetOfLinesPtr(gl_shape_r);
							}
							gl_shape->clear();
							ptg->add_robotShape_to_setOfLines(*gl_shape);
						}
						{
							mrpt::opengl::CSetOfLinesPtr   gl_shape;
							mrpt::opengl::CRenderizablePtr gl_shape_r = gl_robot_frame->getByName("velocity");  // Get or create if new
							if (!gl_shape_r) {
								gl_shape = mrpt::opengl::CSetOfLines::Create();
								gl_shape->setName("velocity");
								gl_shape->setLineWidth(4.0);
								gl_shape->setColor_u8( mrpt::utils::TColor(0x00,0xff,0xff) );
								gl_robot_frame->insert(gl_shape);
							} else {
								gl_shape = mrpt::opengl::CSetOfLinesPtr(gl_shape_r);
							}
							gl_shape->clear();
							const mrpt::math::TTwist2D &velLocal = log.cur_vel_local;
							gl_shape->appendLine(0,0,0, velLocal.vx, velLocal.vy, 0);
						}
					}
				}
			}
			{
				// Target:
				mrpt::opengl::CPointCloudPtr   gl_trg;
				mrpt::opengl::CRenderizablePtr gl_trg_r = gl_robot_frame->getByName("target");  // Get or create if new
				if (!gl_trg_r) {
					gl_trg = mrpt::opengl::CPointCloud::Create();
					gl_trg->setName("target");
					gl_trg->enableShowName(true);
					gl_trg->setPointSize(9.0);
					gl_trg->setColor_u8( mrpt::utils::TColor(0x00,0x00,0x00) );
					gl_robot_frame->insert(gl_trg);
				} else {
					gl_trg = mrpt::opengl::CPointCloudPtr(gl_trg_r);
				}
				// Move the map & add a point at (0,0,0) so the name label appears at the target:
				gl_trg->clear();
				gl_trg->setLocation(log.WS_target_relative.x,log.WS_target_relative.y,0);
				gl_trg->insertPoint(0,0,0);
			}
		}

		// Show extra info as text msgs:
		// ---------------------------------
		int lineY = 0, unique_id = 0;
		win1->clearTextMessages();

		// Mouse position at Z=0
		// Updated in timer callback:
		if (cbShowXY->IsChecked()) {
			lineY++;
			unique_id++;
		}

		if (cbShowAllDebugEntries->IsChecked()) {
			for (const auto &e : log.timestamps)
				ADD_WIN_TEXTMSG(mrpt::format("Timestamp %-20s=%s", e.first.c_str(), mrpt::system::dateTimeLocalToString(e.second).c_str()) );
		}

		ADD_WIN_TEXTMSG(mrpt::format("cmd_vel=%s", log.cmd_vel ? log.cmd_vel->asString().c_str() : "NOP (Continue last PTG)"));

		ADD_WIN_TEXTMSG(mrpt::format("cur_vel      =[%.02f m/s, %0.2f m/s, %.02f dps]",log.cur_vel.vx, log.cur_vel.vy, mrpt::utils::RAD2DEG(log.cur_vel.omega)) );
		ADD_WIN_TEXTMSG(mrpt::format("cur_vel_local=[%.02f m/s, %0.2f m/s, %.02f dps]", log.cur_vel_local.vx, log.cur_vel_local.vy, mrpt::utils::RAD2DEG(log.cur_vel_local.omega)) );

		ADD_WIN_TEXTMSG(mrpt::format("robot_pose=%s", log.robotOdometryPose.asString().c_str()));

		if (log.cmd_vel_original)
		{
			std::stringstream ss;
			ss << "original cmd_vel: ";
			ss << log.cmd_vel_original->asString();
			ADD_WIN_TEXTMSG( ss.str() );
		}

		{
			std::stringstream ss;
			ss << "Performance: ";
			for (size_t i=0;i<log.infoPerPTG.size();i++)
				ss << "PTG#" << i << mrpt::format(" TPObs:%ss HoloNav:%ss |", mrpt::system::unitsFormat(log.infoPerPTG[i].timeForTPObsTransformation).c_str(),mrpt::system::unitsFormat(log.infoPerPTG[i].timeForHolonomicMethod).c_str());
			ADD_WIN_TEXTMSG(ss.str());
		}

		for (unsigned int nPTG=0;nPTG<log.infoPerPTG.size();nPTG++)
		{
			const CLogFileRecord::TInfoPerPTG &pI = log.infoPerPTG[nPTG];

			mrpt::utils::TColorf col;
			if (((int)nPTG)==log.nSelectedPTG)
			     col = mrpt::utils::TColorf(1,1,1);
			else col = mrpt::utils::TColorf(.8f,.8f,.8f);

			ADD_WIN_TEXTMSG(mrpt::format("PTG#%u: SelDir=%+7.01f deg SelSpeed=%.03f Eval=%5.03f factors=%s", nPTG, mrpt::utils::RAD2DEG(pI.desiredDirection), pI.desiredSpeed, pI.evaluation, sprintf_vector("%5.02f ", pI.evalFactors).c_str()));
		}

		ADD_WIN_TEXTMSG(mrpt::format("relPoseSense: %s relPoseVelCmd:%s",
			log.relPoseSense.asString().c_str(),
			log.relPoseVelCmd.asString().c_str()));

		if (cbShowAllDebugEntries->IsChecked()) {
			for (const auto &e : log.values)
				ADD_WIN_TEXTMSG(format("%-30s=%s ", e.first.c_str(), mrpt::system::unitsFormat(e.second, 3, false).c_str()));

			for (const auto &e : log.additional_debug_msgs)
				ADD_WIN_TEXTMSG(format("%-30s=%s ", e.first.c_str(), e.second.c_str()));
		}

		win1->repaint();
	}

	// Draw TP-obstacles
	// --------------------------------
	for (unsigned int nPTG=0;nPTG<log.infoPerPTG.size();nPTG++)  // log.infoPerPTG.size() may be != nPTGs in the last entry is used for "NOP cmdvel"
	{
		CDisplayWindow3DPtr &win = m_mywins3D[format("PTG%u",nPTG)];
		if (!win)  {
			const static int W = 290;
			const static int H = 270;

			win= CDisplayWindow3D::Create(format("%u|TP-Obstacles [%s]",nPTG,log.infoPerPTG[nPTG].PTG_desc.c_str()),W,H);
			win->setPos(20+(W+30)*nPTG, 380);

			{
				mrpt::opengl::COpenGLScenePtr scene;
				mrpt::gui::CDisplayWindow3DLocker locker(*win, scene);

				scene->insert(mrpt::opengl::CGridPlaneXY::Create(-1.0f, 1.0f, -1.0f, 1.0f, .0f, 1.0f));
				scene->insert(mrpt::opengl::stock_objects::CornerXYSimple(0.4f,2.0f) );

				win->setCameraAzimuthDeg(-90);
				win->setCameraElevationDeg(90);
				win->setCameraZoom(2.1f);
				win->setCameraProjective(false);

				{
					auto gl_obj = mrpt::opengl::CDisk::Create();
					gl_obj->setDiskRadius(1.01f, 1.0);
					gl_obj->setSlicesCount(30);
					gl_obj->setColor_u8(mrpt::utils::TColor(0x30, 0x30, 0x30, 0xff));
					scene->insert(gl_obj);
				}
				{
					auto gl_obj = mrpt::opengl::CSetOfLines::Create();
					gl_obj->setName("tp_obstacles");
					gl_obj->setLineWidth(1.0f);
					gl_obj->setVerticesPointSize(4.0f);
					gl_obj->setColor_u8(mrpt::utils::TColor(0x00, 0x00, 0xff, 0xff));
					scene->insert(gl_obj);
				}
				{
					auto gl_obj = mrpt::opengl::CSetOfLines::Create();
					gl_obj->setName("score_phase1");
					gl_obj->setLineWidth(1.0f);
					gl_obj->setVerticesPointSize(2.0f);
					gl_obj->setColor_u8(mrpt::utils::TColor(0xff, 0xff, 0x00, 0xff));
					scene->insert(gl_obj);
				}
				{
					auto gl_obj = mrpt::opengl::CSetOfLines::Create();
					gl_obj->setName("score_phase2");
					gl_obj->setLineWidth(1.0f);
					gl_obj->setVerticesPointSize(2.0f);
					gl_obj->setColor_u8(mrpt::utils::TColor(0xff, 0xff, 0xff, 0xff));
					scene->insert(gl_obj);
				}
				{
					auto gl_obj = mrpt::opengl::CPointCloud::Create();
					gl_obj->setName("tp_target");
					gl_obj->setPointSize(5.0f);
					gl_obj->setColor_u8(mrpt::utils::TColor(0x30, 0x30, 0x30, 0xff));
					scene->insert(gl_obj);
				}
				{
					auto gl_obj = mrpt::opengl::CPointCloud::Create();
					gl_obj->setName("tp_robot");
					gl_obj->setPointSize(4.0f);
					gl_obj->setColor_u8(mrpt::utils::TColor(0xff, 0x00, 0x00, 0xa0));
					scene->insert(gl_obj);
				}
				{
					auto gl_obj = mrpt::opengl::CSetOfLines::Create();
					gl_obj->setName("tp_selected_dir");
					gl_obj->setLineWidth(5.0f);
					gl_obj->setColor_u8(mrpt::utils::TColor(0x00, 0xff, 0x00, 0xff));
					scene->insert(gl_obj);
				}
				{
					auto gl_obj = mrpt::opengl::CMesh::Create(true /*transparency*/);
					gl_obj->setName("tp_clearance");
					gl_obj->setScale(1.0f, 1.0f, 5.0f);
					scene->insert(gl_obj);
				}
			}  // End window locker:
		}

		{
			mrpt::opengl::COpenGLScenePtr scene;
			mrpt::gui::CDisplayWindow3DLocker locker(*win, scene);

			// Draw dynamic stuff:
			const CLogFileRecord::TInfoPerPTG &pI = log.infoPerPTG[nPTG];
			vector<float> xs,ys;

			const size_t nAlphas = pI.TP_Obstacles.size();
			//ASSERT_(nAlphas>0)  // In case of "invalid" PTGs during navigation, TP_Obstacles may be left uncomputed.

			// Chosen direction:
			{
				const double aDir = pI.desiredDirection;

				auto gl_obj = mrpt::opengl::CSetOfLinesPtr(scene->getByName("tp_selected_dir"));
				gl_obj->clear();
				gl_obj->appendLine(
					0, 0, 0,
					pI.desiredSpeed*cos(aDir), pI.desiredSpeed*sin(aDir), 0);
			}

			// obstacles:
			xs.clear(); ys.clear();
			xs.reserve(nAlphas); ys.reserve(nAlphas);
			for (size_t i=0;i<nAlphas;++i)
			{
				const double a = -M_PI + (i+0.5)*2*M_PI/double(nAlphas);
				const double r = pI.TP_Obstacles[i];
				xs.push_back(r*cos(a));
				ys.push_back(r*sin(a));
			}
			{
				auto gl_obj = mrpt::opengl::CSetOfLinesPtr(scene->getByName("tp_obstacles"));
				gl_obj->clear();
				if (nAlphas>2)
				{
					gl_obj->appendLine(xs[0], ys[0], 0, xs[1], ys[1], 0);
					for (size_t i = 2; i < nAlphas; i++)
						gl_obj->appendLineStrip(xs[i], ys[i], 0);
				}
			}

			// Target:
			{
				auto gl_obj = mrpt::opengl::CPointCloudPtr(scene->getByName("tp_target"));
				gl_obj->clear();
				gl_obj->insertPoint(pI.TP_Target.x, pI.TP_Target.y,0);
			}

			// Current robot pt (normally in pure reactive, at (0,0)):
			{
				auto gl_obj = mrpt::opengl::CPointCloudPtr(scene->getByName("tp_robot"));
				gl_obj->clear();
				gl_obj->insertPoint(pI.TP_Robot.x, pI.TP_Robot.y, 0);
			}

			// Clearance-diagram:
			{
				auto gl_obj = mrpt::opengl::CMeshPtr(scene->getByName("tp_clearance"));
				if (pI.clearance.raw_clearances.empty())
					gl_obj->setVisibility(false);
				else
				{
					gl_obj->setVisibility(true);
					pI.clearance.renderAs3DObject(*gl_obj, -1.0, 1.0, -1.0, 1.0, 0.15);
				}
			}
			// Clearance-diagram:
			{
				auto gl_obj1 = mrpt::opengl::CSetOfLinesPtr(scene->getByName("score_phase1"));
				auto gl_obj2 = mrpt::opengl::CSetOfLinesPtr(scene->getByName("score_phase2"));
				const bool visible1 = rbPerPTGPlots->GetSelection() >= 1;
				const bool visible2 = rbPerPTGPlots->GetSelection() >= 2;
				gl_obj1->clear();
				gl_obj2->clear();
				gl_obj1->setVisibility(visible1);
				gl_obj2->setVisibility(visible2);

				if ((visible1 || visible2) && pI.HLFR && IS_CLASS(pI.HLFR, CLogFileRecord_FullEval) && nAlphas>2)
				{
					CLogFileRecord_FullEvalPtr log_FE = CLogFileRecord_FullEvalPtr(pI.HLFR);

					const mrpt::math::CMatrixD &scores = log_FE->dirs_scores;
					if (scores.rows() == nAlphas && scores.cols() >= 6)
					{
						vector<float> xs1, ys1, xs2,ys2;
						xs1.reserve(nAlphas); ys1.reserve(nAlphas);
						xs2.reserve(nAlphas); ys2.reserve(nAlphas);
						for (size_t i = 0; i<nAlphas; ++i)
						{
							const double a = -M_PI + (i + 0.5) * 2 * M_PI / double(nAlphas);
							const double r1 = scores(i, 5), r2 = scores(i, 6);
							xs1.push_back(r1*cos(a));
							ys1.push_back(r1*sin(a));
							xs2.push_back(r2*cos(a));
							ys2.push_back(r2*sin(a));
						}
						gl_obj1->appendLine(xs1[0], ys1[0], 0, xs1[1], ys1[1], 0);
						for (size_t i = 2; i < nAlphas; i++)
							gl_obj1->appendLineStrip(xs1[i], ys1[i], 0);

						gl_obj2->appendLine(xs2[0], ys2[0], 0, xs2[1], ys2[1], 0);
						for (size_t i = 2; i < nAlphas; i++)
							gl_obj2->appendLineStrip(xs2[i], ys2[i], 0);
					}
				}
			}

			// In the case of ND algorithm: draw gaps
#if 0
			if (pI.HLFR && IS_CLASS(pI.HLFR, CLogFileRecord_ND))
			{
				CLogFileRecord_NDPtr log_ND = CLogFileRecord_NDPtr(pI.HLFR);
				const size_t nGaps = log_ND->gaps_ini.size();
				ASSERT_( log_ND->gaps_end.size()==nGaps );
				xs.clear(); ys.clear();
				for (size_t nG=0;nG<nGaps;nG++)
				{
					const int32_t ang_ini = log_ND->gaps_ini[nG];
					const int32_t ang_end = log_ND->gaps_end[nG];

					xs.push_back(0);ys.push_back(0);
					for (int i=ang_ini;i<ang_end;i++)
					{
						const double a = -M_PI + (i+0.5)*2*M_PI/double(nAlphas);
						const double r = pI.TP_Obstacles[i] - 0.04;
						xs.push_back(r*cos(a));
						ys.push_back(r*sin(a));
					}
					xs.push_back(0);ys.push_back(0);
				}
				//win->plot(xs,ys,"k-2", "TPOBS-Gaps");
			}
#endif
		}  // End window locker:

		win->repaint();

	} // end for each PTG

	// Draw time cursor in v/w plots:
	{
		CDisplayWindowPlotsPtr &win = m_mywins["VW"];
		if (win)
		{
			std::vector<double> xs(2),ys(2);
			xs[0] = i; xs[1] = i;
			ys[0] = -2.0; ys[1] = 2.0;
			win->plot(xs,ys,"k-3","cursor_time");
		}
	}


	WX_END_TRY
}



void navlog_viewer_GUI_designDialog::OntimAutoloadTrigger(wxTimerEvent& event)
{
	if (!global_fileToOpen.empty() && mrpt::system::fileExists(global_fileToOpen))
	{
		const std::string sFil = global_fileToOpen;
		global_fileToOpen.clear();
		loadLogfile(sFil);
	}
}

void navlog_viewer_GUI_designDialog::OnbtnMoreOpsClick(wxCommandEvent& event)
{
    this->PopupMenu(&mnuMoreOps);
}

// ------------------------------------------------------------------------
//     Generate a MATLAB script that draws the overall navigation log
// ------------------------------------------------------------------------
void navlog_viewer_GUI_designDialog::OnmnuMatlabPlotsSelected(wxCommandEvent& event)
{
	WX_START_TRY

	wxFileDialog dialog(
		this,
		_("Save MATLAB/Octave script that draws the log...") /*caption*/,
		_(".") /* defaultDir */,
		_("drawLog.m") /* defaultFilename */,
		_("MATLAB/Octave script (*.m)|*.m") /* wildcard */,
		wxFD_SAVE | wxFD_OVERWRITE_PROMPT );
	if (dialog.ShowModal() != wxID_OK) return;

	const string filName(dialog.GetPath().mb_str());

    ofstream f(filName.c_str());
    if (!f.is_open())
        throw runtime_error("Error writing to file!");

    f << "% Script for drawing navigation log\n"
      << "% Generated automatically by navlog-viewer - MRPT " << mrpt::system::MRPT_getVersion() << "\n"
      << "%  From log: " << string(edLogFile->GetLabel().mbc_str()) << "\n"
      << "% -------------------------------------------------------------------------\n\n";

	f << "%%\n"
		<< "function [] = main()\n"
		<< "figure;"
		<< "title('Path for " << mrpt::system::extractFileName(filName) <<"');"
    << "% Robot shape: (x,y) in each line\n"
    << "rs = [-0.3 -0.3;0.6 -0.3;0.6 0.3;-0.3 0.3];\n"
    << "dec_shps = 15;"
    << "dec=0;";

    const int DECIMATE_POINTS = 10;
    int decim_point_cnt =0;

    std::vector<float> X,Y;    // Obstacles
    std::vector<float> TX,TY;  // Target over time

    const size_t N = m_logdata.size();
	for (size_t i=0;i<N;i++)
	{
        const CLogFileRecordPtr logsptr = CLogFileRecordPtr( m_logdata[i] );
        const CLogFileRecord * logptr = logsptr.pointer();

		const CPose2D robotPose = logptr->robotOdometryPose;
		CPose2D observationBasePose = robotPose;

		if (cbShowRelPoses->IsChecked())
			observationBasePose = observationBasePose + logptr->relPoseSense;

        f << format("dec=dec+1; if (dec>=dec_shps); drawRobotShape(rs,[%f %f %f]); dec=0; end\n",
			robotPose.x(), robotPose.y(), robotPose.phi() );

        if (++decim_point_cnt>=DECIMATE_POINTS)
        {
            CSimplePointsMap pts;
            pts.changeCoordinatesReference( logptr->WS_Obstacles, observationBasePose);

            const std::vector<float> &pX = pts.getPointsBufferRef_x();
            const std::vector<float> &pY = pts.getPointsBufferRef_y();

            X.insert(X.begin(),pX.begin(),pX.end());
            Y.insert(Y.begin(),pY.begin(),pY.end());
        }

        // Target:
        const mrpt::math::TPoint2D trg_glob = mrpt::math::TPoint2D(robotPose +logptr->WS_target_relative );
        if (TX.empty() || std::abs((*TX.rbegin())-trg_glob.x)>1e-3 || std::abs((*TY.rbegin())-trg_glob.y)>1e-3 )
        {
            TX.push_back(trg_glob.x);
            TY.push_back(trg_glob.y);
        }
	}

	f << "\n % Points: \n"
	  << " Ps = [";
    for (size_t k=0;k<X.size();k++)
        f << X[k] << " " << Y[k] << "\n";

	f << "];\n"
	  << "plot(Ps(:,1),Ps(:,2),'k.','MarkerSize',3);\n";

	f << "\n % Target point: \n"
	  << " Ts = [";
    for (size_t k=0;k<TX.size();k++)
        f << TX[k] << " " << TY[k] << "\n";

	f << "];\n"
	  << "plot(Ts(:,1),Ts(:,2),'rx','MarkerSize',10);\n";

    f << "axis equal;\n"
    << "\n";

    f
    << "%% drawRobotShape()\n"
    << "function [] = drawRobotShape(sh,pose)\n"
    << "nPts=size(sh,1);\n"
    << "Pts=[];\n"
    << "for i=1:(nPts+1),\n"
    << "    j=mod(i-1,nPts)+1;\n"
    << "    cc=cos(pose(3)); ss=sin(pose(3)); x=pose(1); y=pose(2);\n"
    << "    Pts=[Pts;x+cc*sh(j,1)-ss*sh(j,2) y+ss*sh(j,1)+cc*sh(j,2) ];\n"
    << "end\n"
    << "plot(Pts(:,1),Pts(:,2)); hold on;\n";

	WX_END_TRY
}

void navlog_viewer_GUI_designDialog::OnmnuSeePTGParamsSelected(wxCommandEvent& event)
{
	WX_START_TRY

	const std::string sSection = "PTG_PARAMS";
	mrpt::utils::CConfigFileMemory cfg;

	cfg.write(sSection,"PTG_COUNT",m_logdata_ptg_paths.size() );

	CConfigFilePrefixer cfg_pre;
	cfg_pre.bind(cfg);

	for (size_t i=0;i<m_logdata_ptg_paths.size();i++)
	{
		mrpt::nav::CParameterizedTrajectoryGeneratorPtr ptg = m_logdata_ptg_paths[i];
		if (!ptg) continue;

		const std::string sKeyPrefix = mrpt::format("PTG%d_", (int)i );
		cfg_pre.setPrefixes("",sKeyPrefix);

		ptg->saveToConfigFile(cfg_pre, sSection);
	}

	const std::string sCfgText = cfg.getContent();

	wxMessageBox( _U( sCfgText.c_str() ), _("PTG parameters as stored in the log:"), wxOK, this);

	WX_END_TRY
}

void navlog_viewer_GUI_designDialog::OncbGlobalFrameClick(wxCommandEvent& event)
{
	wxScrollEvent d;
	OnslidLogCmdScroll(d);
}

void navlog_viewer_GUI_designDialog::OnmnuSaveScoreMatrixSelected(wxCommandEvent& event)
{
	WX_START_TRY

	wxFileDialog dialog(
		this,
		_("Save scores matrices...") /*caption*/,
		_(".") /* defaultDir */,
		_("scores.txt") /* defaultFilename */,
		_("MATLAB/Octave plain text file (*.txt)|*.txt") /* wildcard */,
		wxFD_SAVE | wxFD_OVERWRITE_PROMPT );
	if (dialog.ShowModal() != wxID_OK) return;

	const string filName(dialog.GetPath().mb_str());
	const size_t N = m_logdata.size();
	for (size_t i=0;i<N;i++)
	{
		const CLogFileRecordPtr logsptr = CLogFileRecordPtr( m_logdata[i] );
		const CLogFileRecord * logptr = logsptr.pointer();

		for (size_t iPTG = 0; iPTG<logptr->infoPerPTG.size(); iPTG++)
		{
			const CHolonomicLogFileRecordPtr & hlog = logptr->infoPerPTG[iPTG].HLFR;
			if (!hlog.present()) continue;

			const mrpt::math::CMatrixD * dirs_scores = hlog->getDirectionScores();
			if (!dirs_scores || dirs_scores->getRowCount()<2) continue;

			const std::string sFil = mrpt::system::fileNameChangeExtension(filName, mrpt::format("step%06u_ptg%02u.txt",(unsigned int)i, (unsigned int)iPTG ) );

			dirs_scores->saveToTextFile(sFil,mrpt::math::MATRIX_FORMAT_FIXED);
		}
	}

	WX_END_TRY
}

void navlog_viewer_GUI_designDialog::OntimMouseXY(wxTimerEvent& event)
{
	// Mouse position at Z=0
	CDisplayWindow3DPtr &win1 = m_mywins3D["WS_obs"];
	if (!win1) return;

	int lineY = 0, unique_id = 0;

	{
		mrpt::math::TLine3D mouse_ray;
		win1->getLastMousePositionRay(mouse_ray);

		// Create a 3D plane, e.g. Z=0
		const mrpt::math::TPlane ground_plane(TPoint3D(0, 0, 0), TPoint3D(1, 0, 0), TPoint3D(0, 1, 0));
		// Intersection of the line with the plane:
		mrpt::math::TObject3D inters;
		mrpt::math::intersect(mouse_ray, ground_plane, inters);
		// Interpret the intersection as a point, if there is an intersection:
		mrpt::math::TPoint3D inters_pt;
		if (inters.getPoint(inters_pt))
		{
			ADD_WIN_TEXTMSG(mrpt::format("Mouse pos: X=%.04f  Y=%.04f", inters_pt.x, inters_pt.y));
			win1->repaint();
		}
	}

}

void navlog_viewer_GUI_designDialog::OncbShowXYClick(wxCommandEvent& event)
{
	if (cbShowXY->IsChecked())
	     timMouseXY.Start(100, false);
	else timMouseXY.Stop();
}

void navlog_viewer_GUI_designDialog::OnrbPerPTGPlotsSelect(wxCommandEvent& event)
{
	wxScrollEvent d;
	OnslidLogCmdScroll(d);
}
