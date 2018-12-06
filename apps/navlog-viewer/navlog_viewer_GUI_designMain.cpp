/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "navlog_viewer_GUI_designMain.h"
#include <mrpt/gui/about_box.h>
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

#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/config/CConfigFilePrefixer.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/math/utils.h>
#include <mrpt/math/geometry.h>  // intersect()
#include <mrpt/containers/printf_vector.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CMesh.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/stock_objects.h>
#include <algorithm>  // replace()

extern std::string global_fileToOpen;

const double fy = 9,
			 Ay = 12;  // Font size & line spaces for GUI-overlayed text lines
#define ADD_WIN_TEXTMSG_COL(__MSG, __COL)                                      \
	win1->addTextMessage(                                                      \
		5.0, 5 + (lineY++) * Ay, __MSG, __COL, "mono", fy, mrpt::opengl::NICE, \
		unique_id++);

#define ADD_WIN_TEXTMSG(__MSG) \
	ADD_WIN_TEXTMSG_COL(__MSG, mrpt::img::TColorf(1, 1, 1))

using namespace std;
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::img;
using namespace mrpt::maps;
using namespace mrpt::serialization;
using namespace mrpt::config;
using namespace mrpt::poses;
using namespace mrpt::gui;
using namespace mrpt::nav;

//(*IdInit(navlog_viewer_GUI_designDialog)
const long navlog_viewer_GUI_designDialog::ID_BUTTON1 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_BUTTON2 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_BUTTON3 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT1 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_TEXTCTRL1 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_RADIOBOX1 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_CHECKLISTBOX1 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_SLIDER1 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_BUTTON6 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_BUTTON4 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_BUTTON5 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT9 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_TEXTCTRL3 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT8 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_TEXTCTRL2 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_PANEL2 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT2 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT3 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT4 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT5 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT6 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT7 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_PANEL3 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_PANEL1 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_TIMER1 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_TIMER2 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_MENUITEM2 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_MENUITEM1 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_MENUITEM3 = wxNewId();
//*)

const long ID_MENUITEM_SAVE_MATLAB_PATH = wxNewId();

const long navlog_viewer_GUI_designDialog::ID_TIMER3 = wxNewId();

BEGIN_EVENT_TABLE(navlog_viewer_GUI_designDialog, wxFrame)
//(*EventTable(navlog_viewer_GUI_designDialog)
//*)
END_EVENT_TABLE()

#include "../wx-common/mrpt_logo.xpm"
#include "imgs/main_icon.xpm"

// A custom Art provider for customizing the icons:
class MyArtProvider : public wxArtProvider
{
   protected:
	wxBitmap CreateBitmap(
		const wxArtID& id, const wxArtClient& client,
		const wxSize& size) override
	{
		if (id == wxART_MAKE_ART_ID(MAIN_ICON)) return wxBitmap(main_icon_xpm);
		if (id == wxART_MAKE_ART_ID(IMG_MRPT_LOGO))
			return wxBitmap(mrpt_logo_xpm);

		// Any wxWidgets icons not implemented here will be provided by the
		// default art provider.
		return wxNullBitmap;
	}
};

navlog_viewer_GUI_designDialog::navlog_viewer_GUI_designDialog(
	wxWindow* parent, wxWindowID id)
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
	wxFlexGridSizer* FlexGridSizer10;
	wxFlexGridSizer* FlexGridSizer3;
	wxMenuItem* mnuSaveScoreMatrix;
	wxFlexGridSizer* FlexGridSizer5;
	wxFlexGridSizer* FlexGridSizer2;
	wxFlexGridSizer* FlexGridSizer7;
	wxFlexGridSizer* FlexGridSizer6;
	wxFlexGridSizer* FlexGridSizer1;
	wxFlexGridSizer* FlexGridSizer11;

	Create(
		parent, wxID_ANY, _("Navigation log viewer - Part of the MRPT project"),
		wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE,
		_T("wxID_ANY"));
	Move(wxPoint(20, 20));
	FlexGridSizer1 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer1->AddGrowableCol(0);
	FlexGridSizer1->AddGrowableRow(0);
	Panel_AUX = new wxPanel(
		this, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL,
		_T("ID_PANEL1"));
	FlexGridSizer2 = new wxFlexGridSizer(3, 1, 0, 0);
	FlexGridSizer2->AddGrowableCol(0);
	FlexGridSizer11 = new wxFlexGridSizer(0, 1, 0, 0);
	FlexGridSizer11->AddGrowableCol(0);
	FlexGridSizer3 = new wxFlexGridSizer(1, 3, 0, 0);
	FlexGridSizer3->AddGrowableCol(2);
	FlexGridSizer10 = new wxFlexGridSizer(2, 0, 0, 0);
	FlexGridSizer10->AddGrowableCol(0);
	btnLoad = new wxCustomButton(
		Panel_AUX, ID_BUTTON1, _("Load log..."),
		wxArtProvider::GetBitmap(
			wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FOLDER")),
			wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),
		wxDefaultPosition, wxSize(70, 55), wxCUSTBUT_BUTTON | wxCUSTBUT_BOTTOM,
		wxDefaultValidator, _T("ID_BUTTON1"));
	btnLoad->SetBitmapDisabled(
		btnLoad->CreateBitmapDisabled(btnLoad->GetBitmapLabel()));
	btnLoad->SetBitmapMargin(wxSize(2, 4));
	FlexGridSizer10->Add(
		btnLoad, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
	btnHelp = new wxCustomButton(
		Panel_AUX, ID_BUTTON2, _("About..."),
		wxArtProvider::GetBitmap(
			wxART_MAKE_ART_ID_FROM_STR(_T("wxART_HELP")),
			wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),
		wxDefaultPosition, wxSize(70, 55), wxCUSTBUT_BUTTON | wxCUSTBUT_BOTTOM,
		wxDefaultValidator, _T("ID_BUTTON2"));
	btnHelp->SetBitmapDisabled(
		btnHelp->CreateBitmapDisabled(btnHelp->GetBitmapLabel()));
	btnHelp->SetBitmapMargin(wxSize(20, 4));
	FlexGridSizer10->Add(
		btnHelp, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
	btnQuit = new wxCustomButton(
		Panel_AUX, ID_BUTTON3, _("Exit"),
		wxArtProvider::GetBitmap(
			wxART_MAKE_ART_ID_FROM_STR(_T("wxART_QUIT")),
			wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),
		wxDefaultPosition, wxSize(70, 55), wxCUSTBUT_BUTTON | wxCUSTBUT_BOTTOM,
		wxDefaultValidator, _T("ID_BUTTON3"));
	btnQuit->SetBitmapDisabled(
		btnQuit->CreateBitmapDisabled(btnQuit->GetBitmapLabel()));
	btnQuit->SetBitmapMargin(wxSize(20, 4));
	FlexGridSizer10->Add(
		btnQuit, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
	StaticText1 = new wxStaticText(
		Panel_AUX, ID_STATICTEXT1, _("Loaded file:"), wxDefaultPosition,
		wxDefaultSize, 0, _T("ID_STATICTEXT1"));
	FlexGridSizer10->Add(
		StaticText1, 1, wxALL | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
	edLogFile = new wxTextCtrl(
		Panel_AUX, ID_TEXTCTRL1, wxEmptyString, wxDefaultPosition,
		wxDefaultSize, wxTE_MULTILINE | wxTE_READONLY, wxDefaultValidator,
		_T("ID_TEXTCTRL1"));
	FlexGridSizer10->Add(
		edLogFile, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
	FlexGridSizer3->Add(
		FlexGridSizer10, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	wxString __wxRadioBoxChoices_1[3] = {
		_("TP-Obstacles only"), _("+ final scores"), _("+ preliminary scores")};
	rbPerPTGPlots = new wxRadioBox(
		Panel_AUX, ID_RADIOBOX1, _("Per PTG plots:"), wxDefaultPosition,
		wxDefaultSize, 3, __wxRadioBoxChoices_1, 1, wxRA_HORIZONTAL,
		wxDefaultValidator, _T("ID_RADIOBOX1"));
	FlexGridSizer3->Add(
		rbPerPTGPlots, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
	cbList = new wxCheckListBox(
		Panel_AUX, ID_CHECKLISTBOX1, wxDefaultPosition, wxSize(250, 71), 0,
		nullptr, 0, wxDefaultValidator, _T("ID_CHECKLISTBOX1"));
	FlexGridSizer3->Add(
		cbList, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 2);
	FlexGridSizer11->Add(
		FlexGridSizer3, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	FlexGridSizer2->Add(
		FlexGridSizer11, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	FlexGridSizer4 = new wxFlexGridSizer(1, 2, 0, 0);
	FlexGridSizer4->AddGrowableCol(0);
	Panel1 = new wxPanel(
		Panel_AUX, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL,
		_T("ID_PANEL2"));
	FlexGridSizer7 = new wxFlexGridSizer(0, 1, 0, 0);
	FlexGridSizer7->AddGrowableCol(0);
	FlexGridSizer6 = new wxFlexGridSizer(0, 2, 0, 0);
	FlexGridSizer6->AddGrowableCol(0);
	slidLog = new wxSlider(
		Panel1, ID_SLIDER1, 0, 0, 100, wxDefaultPosition, wxDefaultSize,
		wxSL_LABELS, wxDefaultValidator, _T("ID_SLIDER1"));
	FlexGridSizer6->Add(
		slidLog, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
	btnMoreOps = new wxButton(
		Panel1, ID_BUTTON6, _("More ops..."), wxDefaultPosition, wxDefaultSize,
		0, wxDefaultValidator, _T("ID_BUTTON6"));
	FlexGridSizer6->Add(
		btnMoreOps, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer7->Add(
		FlexGridSizer6, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	FlexGridSizer5 = new wxFlexGridSizer(0, 6, 0, 0);
	btnPlay = new wxButton(
		Panel1, ID_BUTTON4, _("Play"), wxDefaultPosition, wxDefaultSize, 0,
		wxDefaultValidator, _T("ID_BUTTON4"));
	FlexGridSizer5->Add(
		btnPlay, 1, wxALL | wxALIGN_TOP | wxALIGN_CENTER_HORIZONTAL, 5);
	btnStop = new wxButton(
		Panel1, ID_BUTTON5, _("Stop"), wxDefaultPosition, wxDefaultSize, 0,
		wxDefaultValidator, _T("ID_BUTTON5"));
	btnStop->Disable();
	FlexGridSizer5->Add(
		btnStop, 1, wxALL | wxALIGN_TOP | wxALIGN_CENTER_HORIZONTAL, 5);
	StaticText6 = new wxStaticText(
		Panel1, ID_STATICTEXT9, _("Animation delay (ms):"), wxDefaultPosition,
		wxDefaultSize, 0, _T("ID_STATICTEXT9"));
	FlexGridSizer5->Add(
		StaticText6, 1, wxALL | wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL, 5);
	edAnimDelayMS = new wxTextCtrl(
		Panel1, ID_TEXTCTRL3, _("50"), wxDefaultPosition, wxSize(55, 21), 0,
		wxDefaultValidator, _T("ID_TEXTCTRL3"));
	FlexGridSizer5->Add(
		edAnimDelayMS, 1, wxALL | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
	StaticText5 = new wxStaticText(
		Panel1, ID_STATICTEXT8, _("Shape draw min. dist:"), wxDefaultPosition,
		wxDefaultSize, 0, _T("ID_STATICTEXT8"));
	FlexGridSizer5->Add(
		StaticText5, 1, wxALL | wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL, 5);
	edShapeMinDist = new wxTextCtrl(
		Panel1, ID_TEXTCTRL2, _("1.0"), wxDefaultPosition, wxSize(55, 21), 0,
		wxDefaultValidator, _T("ID_TEXTCTRL2"));
	FlexGridSizer5->Add(
		edShapeMinDist, 1, wxALL | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer7->Add(
		FlexGridSizer5, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	Panel1->SetSizer(FlexGridSizer7);
	FlexGridSizer7->Fit(Panel1);
	FlexGridSizer7->SetSizeHints(Panel1);
	FlexGridSizer4->Add(
		Panel1, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	StaticBoxSizer2 =
		new wxStaticBoxSizer(wxHORIZONTAL, Panel_AUX, _("Information:"));
	Panel3 = new wxPanel(
		Panel_AUX, ID_PANEL3, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL,
		_T("ID_PANEL3"));
	FlexGridSizer9 = new wxFlexGridSizer(0, 2, 0, 0);
	StaticText2 = new wxStaticText(
		Panel3, ID_STATICTEXT2, _("Log entries:"), wxDefaultPosition,
		wxDefaultSize, 0, _T("ID_STATICTEXT2"));
	FlexGridSizer9->Add(StaticText2, 1, wxALL | wxALIGN_RIGHT | wxALIGN_TOP, 5);
	txtLogEntries = new wxStaticText(
		Panel3, ID_STATICTEXT3, _("0"), wxDefaultPosition, wxDefaultSize, 0,
		_T("ID_STATICTEXT3"));
	FlexGridSizer9->Add(
		txtLogEntries, 1, wxALL | wxALIGN_LEFT | wxALIGN_TOP, 5);
	StaticText3 = new wxStaticText(
		Panel3, ID_STATICTEXT4, _("Duration:"), wxDefaultPosition,
		wxDefaultSize, 0, _T("ID_STATICTEXT4"));
	FlexGridSizer9->Add(
		StaticText3, 1, wxALL | wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL, 5);
	txtLogDuration = new wxStaticText(
		Panel3, ID_STATICTEXT5, _("0"), wxDefaultPosition, wxSize(80, -1), 0,
		_T("ID_STATICTEXT5"));
	FlexGridSizer9->Add(
		txtLogDuration, 1, wxALL | wxALIGN_LEFT | wxALIGN_TOP, 5);
	StaticText4 = new wxStaticText(
		Panel3, ID_STATICTEXT6, _("Selected PTG:"), wxDefaultPosition,
		wxDefaultSize, 0, _T("ID_STATICTEXT6"));
	FlexGridSizer9->Add(
		StaticText4, 1, wxALL | wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL, 5);
	txtSelectedPTG = new wxStaticText(
		Panel3, ID_STATICTEXT7, _("-"), wxDefaultPosition, wxSize(80, -1), 0,
		_T("ID_STATICTEXT7"));
	FlexGridSizer9->Add(
		txtSelectedPTG, 1, wxALL | wxALIGN_LEFT | wxALIGN_TOP, 5);
	Panel3->SetSizer(FlexGridSizer9);
	FlexGridSizer9->Fit(Panel3);
	FlexGridSizer9->SetSizeHints(Panel3);
	StaticBoxSizer2->Add(
		Panel3, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	FlexGridSizer4->Add(
		StaticBoxSizer2, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer2->Add(
		FlexGridSizer4, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	Panel_AUX->SetSizer(FlexGridSizer2);
	FlexGridSizer2->Fit(Panel_AUX);
	FlexGridSizer2->SetSizeHints(Panel_AUX);
	FlexGridSizer1->Add(
		Panel_AUX, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	SetSizer(FlexGridSizer1);
	timPlay.SetOwner(this, ID_TIMER1);
	timAutoload.SetOwner(this, ID_TIMER2);
	timAutoload.Start(20, false);
	mnuSeePTGParams = new wxMenuItem(
		(&mnuMoreOps), ID_MENUITEM2, _("See PTG params..."), wxEmptyString,
		wxITEM_NORMAL);
	mnuMoreOps.Append(mnuSeePTGParams);
	mnuMatlabPlots = new wxMenuItem(
		(&mnuMoreOps), ID_MENUITEM1, _("Export map plot to MATLAB..."),
		wxEmptyString, wxITEM_NORMAL);
	mnuMoreOps.Append(mnuMatlabPlots);
	mnuSaveScoreMatrix = new wxMenuItem(
		(&mnuMoreOps), ID_MENUITEM3, _("Save score matrices..."), wxEmptyString,
		wxITEM_NORMAL);
	mnuMoreOps.Append(mnuSaveScoreMatrix);
	FlexGridSizer1->Fit(this);
	FlexGridSizer1->SetSizeHints(this);

	Connect(
		ID_BUTTON1, wxEVT_COMMAND_BUTTON_CLICKED,
		(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnbtnLoadClick);
	Connect(
		ID_BUTTON2, wxEVT_COMMAND_BUTTON_CLICKED,
		(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnbtnHelpClick);
	Connect(
		ID_BUTTON3, wxEVT_COMMAND_BUTTON_CLICKED,
		(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnbtnQuitClick);
	Connect(
		ID_RADIOBOX1, wxEVT_COMMAND_RADIOBOX_SELECTED,
		(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::
			OnrbPerPTGPlotsSelect);
	Connect(
		ID_CHECKLISTBOX1, wxEVT_COMMAND_CHECKLISTBOX_TOGGLED,
		(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::
			OncbGlobalFrameClick);
	Connect(
		ID_SLIDER1, wxEVT_SCROLL_THUMBTRACK,
		(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::
			OnslidLogCmdScroll);
	Connect(
		ID_SLIDER1, wxEVT_SCROLL_CHANGED,
		(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::
			OnslidLogCmdScroll);
	Connect(
		ID_BUTTON6, wxEVT_COMMAND_BUTTON_CLICKED,
		(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::
			OnbtnMoreOpsClick);
	Connect(
		ID_BUTTON4, wxEVT_COMMAND_BUTTON_CLICKED,
		(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnbtnPlayClick);
	Connect(
		ID_BUTTON5, wxEVT_COMMAND_BUTTON_CLICKED,
		(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnbtnStopClick);
	Connect(
		ID_TIMER1, wxEVT_TIMER,
		(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::
			OntimPlayTrigger);
	Connect(
		ID_TIMER2, wxEVT_TIMER,
		(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::
			OntimAutoloadTrigger);
	Connect(
		ID_MENUITEM2, wxEVT_COMMAND_MENU_SELECTED,
		(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::
			OnmnuSeePTGParamsSelected);
	Connect(
		ID_MENUITEM1, wxEVT_COMMAND_MENU_SELECTED,
		(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::
			OnmnuMatlabPlotsSelected);
	Connect(
		ID_MENUITEM3, wxEVT_COMMAND_MENU_SELECTED,
		(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::
			OnmnuSaveScoreMatrixSelected);
	//*)

	{
		wxMenuItem* mnuMatlabExportPaths;
		mnuMatlabExportPaths = new wxMenuItem(
			(&mnuMoreOps), ID_MENUITEM_SAVE_MATLAB_PATH,
			_("Export paths info to MATLAB..."), wxEmptyString, wxITEM_NORMAL);
		mnuMoreOps.Append(mnuMatlabExportPaths);
		Connect(
			ID_MENUITEM_SAVE_MATLAB_PATH, wxEVT_COMMAND_MENU_SELECTED,
			(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::
				OnmnuMatlabExportPaths);
	}

	m_cbIdx_DrawShape = cbList->Append(_("Draw shape along path"));
	m_cbIdx_ShowAllDebugFields = cbList->Append(_("Show all debug fields"));
	m_cbIdx_GlobalFrame = cbList->Append(_("Represent in global frame"));
	m_cbIdx_UseOdometryCoords = cbList->Append(_("Use raw odometry"));
	m_cbIdx_ShowDelays = cbList->Append(_("Show delays model-based poses"));
	m_cbIdx_ClearanceOverPath =
		cbList->Append(_("Clearance over path (uncheck=pointwise)"));
	m_cbIdx_ShowCursor = cbList->Append(_("Show cursor (X,Y) pos"));

	cbList->Check(m_cbIdx_DrawShape);
	cbList->Check(m_cbIdx_GlobalFrame);
	cbList->Check(m_cbIdx_ShowDelays);

	timMouseXY.SetOwner(this, ID_TIMER3);
	Connect(
		ID_TIMER3, wxEVT_TIMER,
		(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OntimMouseXY);

	rbPerPTGPlots->SetSelection(2);

	{
		wxIcon FrameIcon;
		FrameIcon.CopyFromBitmap(wxArtProvider::GetBitmap(
			wxART_MAKE_ART_ID_FROM_STR(_T("MAIN_ICON")), wxART_FRAME_ICON));
		SetIcon(FrameIcon);
	}

	m_log_first_tim = INVALID_TIMESTAMP;
	m_log_last_tim = INVALID_TIMESTAMP;

	UpdateInfoFromLoadedLog();  // Disable some controls, etc..
}

navlog_viewer_GUI_designDialog::~navlog_viewer_GUI_designDialog()
{
	//(*Destroy(navlog_viewer_GUI_designDialog)
	//*)
	// Clean all windows:
	m_mywins.clear();
	m_mywins3D.clear();
	std::this_thread::sleep_for(100ms);
}

// ---------------------------------------------------------
// Load log file
// ---------------------------------------------------------
void navlog_viewer_GUI_designDialog::OnbtnLoadClick(wxCommandEvent& event)
{
	WX_START_TRY

	wxFileDialog dialog(
		this, _("Load log...") /*caption*/, _(".") /* defaultDir */,
		_("") /* defaultFilename */,
		_("Navigation logs "
		  "(*.reactivenavlog;*.navlog)|*."
		  "reactivenavlog;*.navlog") /* wildcard */,
		wxFD_OPEN | wxFD_FILE_MUST_EXIST);
	if (dialog.ShowModal() != wxID_OK) return;

	const string filName(dialog.GetPath().mb_str());

	loadLogfile(filName);
	WX_END_TRY
}

void navlog_viewer_GUI_designDialog::loadLogfile(const std::string& filName)
{
	WX_START_TRY

	this->edLogFile->SetLabel(filName.c_str());

	mrpt::io::CFileGZInputStream f(filName);

	m_logdata.clear();
	m_logdata_ptg_paths.clear();

	wxBusyCursor busy;

	set<string> validClasses;
	validClasses.insert("CLogFileRecord");

	m_log_first_tim = INVALID_TIMESTAMP;
	m_log_last_tim = INVALID_TIMESTAMP;

	auto arch = archiveFrom(f);
	for (;;)
	{
		try
		{
			CSerializable::Ptr obj = arch.ReadObject();
			if (validClasses.find(string(obj->GetRuntimeClass()->className)) ==
				validClasses.end())
			{
				wxMessageBox(
					(format(
						 "Unexpected class found: %s",
						 obj->GetRuntimeClass()->className)
						 .c_str()),
					_("Error loading log:"));
				break;
			}
			m_logdata.push_back(obj);

			// generate time stats:
			if (IS_CLASS(obj, CLogFileRecord))
			{
				const CLogFileRecord::Ptr logptr =
					std::dynamic_pointer_cast<CLogFileRecord>(obj);
				const auto it = logptr->timestamps.find("tim_start_iteration");
				if (it != logptr->timestamps.end()) m_log_last_tim = it->second;

				if (!logptr->infoPerPTG.empty())
				{
					size_t nPTGs = logptr->infoPerPTG.size();
					if (nPTGs > m_logdata_ptg_paths.size())
					{
						m_logdata_ptg_paths.resize(nPTGs);
						for (size_t i = 0; i < nPTGs; i++)
							if (logptr->infoPerPTG[i].ptg)
								m_logdata_ptg_paths[i] =
									logptr->infoPerPTG[i].ptg;
					}
				}
			}

			if (m_log_first_tim == INVALID_TIMESTAMP &&
				m_log_last_tim != INVALID_TIMESTAMP)
				m_log_first_tim = m_log_last_tim;
		}
		catch (CExceptionEOF&)
		{
			break;
		}
		catch (const std::exception& e)
		{
			// EOF in the middle of an object... It may be usual if the logger
			// is shut down not cleanly.
			wxMessageBox(
				mrpt::exception_to_str(e),
				wxT("Loading ended with an exception"), wxOK, this);
			break;
		}
	}

	// Update stats, etc...
	UpdateInfoFromLoadedLog();

	WX_END_TRY
}

void navlog_viewer_GUI_designDialog::OnbtnHelpClick(wxCommandEvent& event)
{
	mrpt::gui::show_mrpt_about_box_wxWidgets(
		this, "navlog-viewer",
		"Invoke with --help to see command line arguments.");
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
	if (!btnStop->IsEnabled()) return;

	int p = this->slidLog->GetValue();
	if ((p + 1) <= this->slidLog->GetMax())
	{
		this->slidLog->SetValue(p + 1);
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

	this->Panel1->Enable(N != 0);

	if (N > 0)
	{
		this->txtLogEntries->SetLabel(wxString::Format(_("%u"), (unsigned)N));
		this->slidLog->SetRange(0, N - 1);
		this->slidLog->SetValue(0);
		wxScrollEvent d;
		OnslidLogCmdScroll(d);

		// In the past there was a window here to draw cmd_vels: it's now
		// replaced by a more versatile system to log
		// data to MATLAB/Octave files.
	}

	std::string sDuration("???");
	if (m_log_first_tim != INVALID_TIMESTAMP &&
		m_log_last_tim != INVALID_TIMESTAMP)
	{
		sDuration = mrpt::system::intervalFormat(
			mrpt::system::timeDifference(m_log_first_tim, m_log_last_tim));
	}
	this->txtLogDuration->SetLabel(sDuration.c_str());
	;

	// flexGridRightHand->RecalcSizes();
	this->Fit();
}

// ---------------------------------------------
// 				DRAW ONE LOG RECORD
// ---------------------------------------------
void navlog_viewer_GUI_designDialog::OnslidLogCmdScroll(wxScrollEvent& event)
{
	WX_START_TRY

	const int log_idx = this->slidLog->GetValue();
	if (log_idx >= int(m_logdata.size())) return;
	// In the future, we could handle more log classes. For now, only
	// "CLogFileRecord::Ptr":
	auto logptr = std::dynamic_pointer_cast<CLogFileRecord>(m_logdata[log_idx]);
	const CLogFileRecord& log = *logptr;

	txtSelectedPTG->SetLabel(wxString::Format(
		_("%i from [0-%u]"), static_cast<int>(log.nSelectedPTG),
		static_cast<unsigned int>(log.nPTGs - 1)));

	const bool is_NOP_cmd = log.ptg_index_NOP >= 0;
	const int sel_ptg_idx = !is_NOP_cmd ? log.nSelectedPTG : log.ptg_index_NOP;

	// Draw WS-obstacles
	// --------------------------------
	{
		CDisplayWindow3D::Ptr& win1 = m_mywins3D["WS_obs"];
		if (!win1)
		{
			win1 = mrpt::make_aligned_shared<CDisplayWindow3D>(
				"Sensed obstacles", 500, 400);
			win1->setPos(800, 20);
			win1->setCameraAzimuthDeg(-90);
			win1->setCameraElevationDeg(90);
			{
				mrpt::opengl::COpenGLScene::Ptr scene;
				mrpt::gui::CDisplayWindow3DLocker locker(*win1, scene);

				// XY ground plane:
				mrpt::opengl::CGridPlaneXY::Ptr gl_grid =
					mrpt::make_aligned_shared<mrpt::opengl::CGridPlaneXY>(
						-20, 20, -20, 20, 0, 1, 0.75f);
				gl_grid->setColor_u8(mrpt::img::TColor(0xa0a0a0, 0x90));
				scene->insert(gl_grid);

				// XYZ corner at origin:
				scene->insert(
					mrpt::opengl::stock_objects::CornerXYZSimple(1.0, 2.0));
			}
		}

		// Update 3D view:
		{
			mrpt::opengl::COpenGLScene::Ptr scene;
			mrpt::gui::CDisplayWindow3DLocker locker(*win1, scene);

			const CVectorFloat shap_x = log.robotShape_x,
							   shap_y = log.robotShape_y;

			// Robot frame of reference:
			mrpt::opengl::CSetOfObjects::Ptr gl_robot_frame;
			{
				mrpt::opengl::CRenderizable::Ptr gl_rbframe_r =
					scene->getByName("robot_frame");  // Get or create if new
				if (!gl_rbframe_r)
				{
					gl_robot_frame = mrpt::make_aligned_shared<
						mrpt::opengl::CSetOfObjects>();
					gl_robot_frame->setName("robot_frame");
					scene->insert(gl_robot_frame);
				}
				else
				{
					gl_robot_frame =
						std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(
							gl_rbframe_r);
				}
				// Global or local coordinates?
				if (cbList->IsChecked(m_cbIdx_UseOdometryCoords))
				{
					// Use odometry pose increment wrt initial instant,
					// taking the localization-based "good" pose of the initial
					// instant as a reference, such that the obtained
					// coordinates closely match those expected in the "map"
					// frame:
					auto log0ptr =
						mrpt::ptr_cast<CLogFileRecord>::from(m_logdata[0]);
					ASSERT_(log0ptr.get());
					const auto curPose =
						log0ptr->robotPoseLocalization +
						(log.robotPoseOdometry - log0ptr->robotPoseOdometry);

					gl_robot_frame->setPose(curPose);
				}
				else if (cbList->IsChecked(m_cbIdx_GlobalFrame))
				{
					gl_robot_frame->setPose(mrpt::poses::CPose3D(
						mrpt::poses::CPose2D(log.robotPoseLocalization)));
					// Move the window focus:
					float px, py, pz;
					win1->getCameraPointingToPoint(px, py, pz);
					const float cam_zoom = win1->getCameraZoom();
					if ((mrpt::math::TPoint2D(log.robotPoseLocalization) -
						 mrpt::math::TPoint2D(px, py))
							.norm() > .3 * cam_zoom)
						win1->setCameraPointingToPoint(
							log.robotPoseLocalization.x,
							log.robotPoseLocalization.y, 0.0);
				}
				else
				{
					gl_robot_frame->setPose(mrpt::poses::CPose3D());
				}
			}

			// Extrapolated poses from delay models:
			{
				mrpt::opengl::CSetOfObjects::Ptr gl_relposes;
				mrpt::opengl::CRenderizable::Ptr gl_relposes_r =
					gl_robot_frame->getByName(
						"relposes");  // Get or create if new
				if (!gl_relposes_r)
				{
					gl_relposes = mrpt::make_aligned_shared<
						mrpt::opengl::CSetOfObjects>();
					gl_relposes->setName("relposes");
					gl_robot_frame->insert(gl_relposes);
				}
				else
				{
					gl_relposes =
						std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(
							gl_relposes_r);
				}

				gl_relposes->clear();
				if (cbList->IsChecked(m_cbIdx_ShowDelays))
				{
					{
						mrpt::opengl::CSetOfObjects::Ptr gl_relpose_sense =
							mrpt::opengl::stock_objects::CornerXYSimple(
								0.3f, 1);
						gl_relpose_sense->setName("sense");
						gl_relpose_sense->enableShowName(true);
						gl_relpose_sense->setPose(log.relPoseSense);
						gl_relposes->insert(gl_relpose_sense);
					}
					{
						mrpt::opengl::CSetOfObjects::Ptr gl_relpose_cmdvel =
							mrpt::opengl::stock_objects::CornerXYSimple(
								0.3f, 1);
						gl_relpose_cmdvel->setName("cmdVel");
						gl_relpose_cmdvel->enableShowName(true);
						gl_relpose_cmdvel->setPose(log.relPoseVelCmd);
						gl_relposes->insert(gl_relpose_cmdvel);
					}
				}
			}

			{
				// Obstacles: original
				mrpt::opengl::CPointCloud::Ptr gl_obs;
				mrpt::opengl::CRenderizable::Ptr gl_obs_r =
					gl_robot_frame->getByName(
						"obs-raw");  // Get or create if new
				if (!gl_obs_r)
				{
					gl_obs =
						mrpt::make_aligned_shared<mrpt::opengl::CPointCloud>();
					gl_obs->setName("obs-raw");
					gl_obs->setPointSize(3);
					gl_obs->setColor_u8(mrpt::img::TColor(0xff, 0xff, 0x00));
					gl_robot_frame->insert(gl_obs);
				}
				else
				{
					gl_obs = dynamic_pointer_cast<mrpt::opengl::CPointCloud>(
						gl_obs_r);
				}
				gl_obs->loadFromPointsMap(&log.WS_Obstacles_original);
				if (cbList->IsChecked(m_cbIdx_ShowDelays))
					gl_obs->setPose(log.relPoseSense);
				else
					gl_obs->setPose(mrpt::poses::CPose3D());
			}
			{
				// Obstacles: after optional filtering
				mrpt::opengl::CPointCloud::Ptr gl_obs;
				mrpt::opengl::CRenderizable::Ptr gl_obs_r =
					gl_robot_frame->getByName("obs");  // Get or create if new
				if (!gl_obs_r)
				{
					gl_obs =
						mrpt::make_aligned_shared<mrpt::opengl::CPointCloud>();
					gl_obs->setName("obs");
					gl_obs->setPointSize(3.0);
					gl_obs->setColor_u8(mrpt::img::TColor(0x00, 0x00, 0xff));
					gl_robot_frame->insert(gl_obs);
				}
				else
				{
					gl_obs =
						std::dynamic_pointer_cast<mrpt::opengl::CPointCloud>(
							gl_obs_r);
				}
				gl_obs->loadFromPointsMap(&log.WS_Obstacles);
				if (cbList->IsChecked(m_cbIdx_ShowDelays))
					gl_obs->setPose(log.relPoseSense);
				else
					gl_obs->setPose(mrpt::poses::CPose3D());
			}

			{
				// Selected PTG path:
				mrpt::opengl::CSetOfLines::Ptr gl_path;
				mrpt::opengl::CRenderizable::Ptr gl_path_r =
					gl_robot_frame->getByName("path");  // Get or create if new
				if (!gl_path_r)
				{
					gl_path =
						mrpt::make_aligned_shared<mrpt::opengl::CSetOfLines>();
					gl_path->setName("path");
					gl_path->setLineWidth(2.0);
					gl_path->setColor_u8(mrpt::img::TColor(0x00, 0x00, 0xff));
					gl_robot_frame->insert(gl_path);
				}
				else
					gl_path = mrpt::ptr_cast<mrpt::opengl::CSetOfLines>::from(
						gl_path_r);
				gl_path->clear();
				if (sel_ptg_idx < int(m_logdata_ptg_paths.size()) &&
					sel_ptg_idx >= 0)
				{
					mrpt::nav::CParameterizedTrajectoryGenerator::Ptr ptg =
						m_logdata_ptg_paths[sel_ptg_idx];
					if (ptg)
					{
						if (!ptg->isInitialized()) ptg->initialize();

						// Set instantaneous dyn state:
						ptg->updateNavDynamicState(
							is_NOP_cmd ? log.ptg_last_navDynState
									   : log.navDynState);

						// Draw path:
						const int selected_k =
							log.ptg_index_NOP < 0
								? ptg->alpha2index(log.infoPerPTG[sel_ptg_idx]
													   .desiredDirection)
								: log.ptg_last_k_NOP;
						float max_dist = ptg->getRefDistance();
						ptg->add_robotShape_to_setOfLines(*gl_path);

						ptg->renderPathAsSimpleLine(
							selected_k, *gl_path, 0.10, max_dist);
						gl_path->setColor_u8(
							mrpt::img::TColor(0xff, 0x00, 0x00));

						// PTG origin:
						// enable delays model?
						mrpt::math::TPose2D ptg_origin =
							(cbList->IsChecked(m_cbIdx_ShowDelays))
								? log.relPoseVelCmd
								: mrpt::math::TPose2D(0, 0, 0);

						// "NOP cmd" case:
						if (log.ptg_index_NOP >= 0)
						{
							ptg_origin = ptg_origin -
										 log.rel_cur_pose_wrt_last_vel_cmd_NOP;
						}

						gl_path->setPose(ptg_origin);

						// Overlay a sequence of robot shapes:
						if (cbList->IsChecked(m_cbIdx_DrawShape))
						{
							double min_shape_dists = 1.0;
							edShapeMinDist->GetValue().ToDouble(
								&min_shape_dists);

							for (double d = min_shape_dists; d < max_dist;
								 d += min_shape_dists)
							{
								uint32_t step;
								if (!ptg->getPathStepForDist(
										selected_k, d, step))
									continue;
								mrpt::math::TPose2D p;
								ptg->getPathPose(selected_k, step, p);
								ptg->add_robotShape_to_setOfLines(
									*gl_path, mrpt::poses::CPose2D(p));
							}
						}
						{
							// Robot shape:
							mrpt::opengl::CSetOfLines::Ptr gl_shape;
							mrpt::opengl::CRenderizable::Ptr gl_shape_r =
								gl_robot_frame->getByName(
									"shape");  // Get or create if new
							if (!gl_shape_r)
							{
								gl_shape = mrpt::make_aligned_shared<
									mrpt::opengl::CSetOfLines>();
								gl_shape->setName("shape");
								gl_shape->setLineWidth(4.0);
								gl_shape->setColor_u8(
									mrpt::img::TColor(0xff, 0x00, 0x00));
								gl_robot_frame->insert(gl_shape);
							}
							else
							{
								gl_shape = std::dynamic_pointer_cast<
									mrpt::opengl::CSetOfLines>(gl_shape_r);
							}
							gl_shape->clear();
							ptg->add_robotShape_to_setOfLines(*gl_shape);
						}
						{
							mrpt::opengl::CSetOfLines::Ptr gl_shape;
							mrpt::opengl::CRenderizable::Ptr gl_shape_r =
								gl_robot_frame->getByName(
									"velocity");  // Get or create if new
							if (!gl_shape_r)
							{
								gl_shape = mrpt::make_aligned_shared<
									mrpt::opengl::CSetOfLines>();
								gl_shape->setName("velocity");
								gl_shape->setLineWidth(4.0);
								gl_shape->setColor_u8(
									mrpt::img::TColor(0x00, 0xff, 0xff));
								gl_robot_frame->insert(gl_shape);
							}
							else
							{
								gl_shape = std::dynamic_pointer_cast<
									mrpt::opengl::CSetOfLines>(gl_shape_r);
							}
							gl_shape->clear();
							const mrpt::math::TTwist2D& velLocal =
								log.cur_vel_local;
							gl_shape->appendLine(
								0, 0, 0, velLocal.vx, velLocal.vy, 0);
						}
					}
				}
			}
			{
				// Target:
				mrpt::opengl::CPointCloud::Ptr gl_trg;
				mrpt::opengl::CRenderizable::Ptr gl_trg_r =
					gl_robot_frame->getByName(
						"target");  // Get or create if new
				if (!gl_trg_r)
				{
					gl_trg =
						mrpt::make_aligned_shared<mrpt::opengl::CPointCloud>();
					gl_trg->setName("target");
					gl_trg->enableShowName(true);
					gl_trg->setPointSize(9.0);
					gl_trg->setColor_u8(mrpt::img::TColor(0x00, 0x00, 0x00));
					gl_robot_frame->insert(gl_trg);
				}
				else
				{
					gl_trg =
						std::dynamic_pointer_cast<mrpt::opengl::CPointCloud>(
							gl_trg_r);
				}
				// Move the map & add a point at (0,0,0) so the name label
				// appears at the target:
				gl_trg->clear();
				if (!log.WS_targets_relative.empty())
				{
					const auto t0 = log.WS_targets_relative[0];
					const float tz = .05f;
					gl_trg->setLocation(t0.x, t0.y, tz);
					for (const auto& t : log.WS_targets_relative)
					{
						gl_trg->insertPoint(t.x - t0.x, t.y - t0.y, tz);
					}
				}
			}
		}

		// Show extra info as text msgs:
		// ---------------------------------
		int lineY = 0, unique_id = 0;
		win1->clearTextMessages();

		// Mouse position at Z=0
		// Updated in timer callback:
		if (cbList->IsChecked(m_cbIdx_ShowCursor))
		{
			lineY++;
			unique_id++;
		}

		if (cbList->IsChecked(m_cbIdx_ShowAllDebugFields))
		{
			for (const auto& e : log.timestamps)
				ADD_WIN_TEXTMSG(mrpt::format(
					"Timestamp %-20s=%s", e.first.c_str(),
					mrpt::system::dateTimeLocalToString(e.second).c_str()));
		}

		{
			const unsigned int nObsOrg = log.WS_Obstacles_original.size(),
							   nObs = log.WS_Obstacles.size();
			const unsigned int nObsFiltered = nObsOrg - nObs;

			if (nObsFiltered)
			{
				ADD_WIN_TEXTMSG(mrpt::format(
					"Obstacle points=%u (%u filtered out)", nObs,
					nObsFiltered));
			}
			else
			{
				ADD_WIN_TEXTMSG(mrpt::format("Obstacle points=%u", nObs));
			}
		}

		ADD_WIN_TEXTMSG(mrpt::format(
			"cmd_vel=%s", log.cmd_vel ? log.cmd_vel->asString().c_str()
									  : "NOP (Continue last PTG)"));

		ADD_WIN_TEXTMSG(mrpt::format(
			"cur_vel      =[%.02f m/s, %0.2f m/s, %.02f dps]", log.cur_vel.vx,
			log.cur_vel.vy, mrpt::RAD2DEG(log.cur_vel.omega)));
		ADD_WIN_TEXTMSG(mrpt::format(
			"cur_vel_local=[%.02f m/s, %0.2f m/s, %.02f dps]",
			log.cur_vel_local.vx, log.cur_vel_local.vy,
			mrpt::RAD2DEG(log.cur_vel_local.omega)));

		ADD_WIN_TEXTMSG(mrpt::format(
			"robot_pose=%s", log.robotPoseLocalization.asString().c_str()));
		{
			for (unsigned int i = 0; i < log.WS_targets_relative.size(); i++)
			{
				ADD_WIN_TEXTMSG(mrpt::format(
					"rel_target[%u]=%s", i,
					log.WS_targets_relative[i].asString().c_str()));
			}
		}

		if (log.cmd_vel_original)
		{
			std::stringstream ss;
			ss << "original cmd_vel: ";
			ss << log.cmd_vel_original->asString();
			ADD_WIN_TEXTMSG(ss.str());
		}

		{
			std::stringstream ss;
			ss << "Performance: ";
			for (size_t i = 0; i < log.infoPerPTG.size(); i++)
				ss << "PTG#" << i
				   << mrpt::format(
						  " TPObs:%ss HoloNav:%ss |",
						  mrpt::system::unitsFormat(
							  log.infoPerPTG[i].timeForTPObsTransformation)
							  .c_str(),
						  mrpt::system::unitsFormat(
							  log.infoPerPTG[i].timeForHolonomicMethod)
							  .c_str());
			ADD_WIN_TEXTMSG(ss.str());
		}

		for (unsigned int nPTG = 0; nPTG < log.infoPerPTG.size(); nPTG++)
		{
			const CLogFileRecord::TInfoPerPTG& pI = log.infoPerPTG[nPTG];

			mrpt::img::TColorf col;
			if (((int)nPTG) == log.nSelectedPTG)
				col = mrpt::img::TColorf(1, 1, 1);
			else
				col = mrpt::img::TColorf(.8f, .8f, .8f);

			auto sFactors = pI.evalFactors.getAsString();
			std::replace(sFactors.begin(), sFactors.end(), '\r', ' ');
			std::replace(sFactors.begin(), sFactors.end(), '\n', ' ');

			ADD_WIN_TEXTMSG_COL(
				mrpt::format(
					"PTG#%u: SelDir=%+7.01f deg SelSpeed=%.03f Eval=%5.03f. %s",
					nPTG, mrpt::RAD2DEG(pI.desiredDirection), pI.desiredSpeed,
					pI.evaluation, sFactors.c_str()),
				col);
		}

		ADD_WIN_TEXTMSG(mrpt::format(
			"relPoseSense: %s relPoseVelCmd:%s",
			log.relPoseSense.asString().c_str(),
			log.relPoseVelCmd.asString().c_str()));

		if (cbList->IsChecked(m_cbIdx_ShowAllDebugFields))
		{
			for (const auto& e : log.values)
				ADD_WIN_TEXTMSG(format(
					"%-30s=%s ", e.first.c_str(),
					mrpt::system::unitsFormat(e.second, 3, false).c_str()));

			for (const auto& e : log.additional_debug_msgs)
				ADD_WIN_TEXTMSG(
					format("%-30s=%s ", e.first.c_str(), e.second.c_str()));
		}

		win1->repaint();
	}

	// Draw TP-obstacles
	// --------------------------------
	for (unsigned int nPTG = 0; nPTG < log.infoPerPTG.size();
		 nPTG++)  // log.infoPerPTG.size() may be != nPTGs in the last entry is
	// used for "NOP cmdvel"
	{
		const bool is_selected_ptg = (int(nPTG) == log.nSelectedPTG);
		CDisplayWindow3D::Ptr& win = m_mywins3D[format("PTG%u", nPTG)];
		if (!win)
		{
			const static int W = 290;
			const static int H = 270;

			win = mrpt::make_aligned_shared<CDisplayWindow3D>(
				format("%u|TP-Obstacles", nPTG), W, H);
			win->setPos(
				20 + (W + 10) * (nPTG % 3), 280 + (H + 10) * (nPTG / 3));
			win->addTextMessage(
				4, 4,
				format("[%u]:%s", nPTG, log.infoPerPTG[nPTG].PTG_desc.c_str()),
				TColorf(1.0f, 1.0f, 1.0f), "sans", 8, mrpt::opengl::NICE,
				0 /*id*/, 1.5, 0.1, true /*shadow*/);

			{
				mrpt::opengl::COpenGLScene::Ptr scene;
				mrpt::gui::CDisplayWindow3DLocker locker(*win, scene);

				scene->insert(
					mrpt::make_aligned_shared<mrpt::opengl::CGridPlaneXY>(
						-1.0f, 1.0f, -1.0f, 1.0f, .0f, 1.0f));
				scene->insert(
					mrpt::opengl::stock_objects::CornerXYSimple(0.4f, 2.0f));

				win->setCameraAzimuthDeg(-90);
				win->setCameraElevationDeg(90);
				win->setCameraZoom(2.1f);
				win->setCameraProjective(false);

				{
					auto gl_obj =
						mrpt::make_aligned_shared<mrpt::opengl::CDisk>();
					gl_obj->setDiskRadius(1.01f, 1.0);
					gl_obj->setSlicesCount(30);
					gl_obj->setColor_u8(
						mrpt::img::TColor(0x30, 0x30, 0x30, 0xff));
					scene->insert(gl_obj);
				}
				{
					auto gl_obj =
						mrpt::make_aligned_shared<mrpt::opengl::CSetOfLines>();
					gl_obj->setName("tp_obstacles");
					gl_obj->setLineWidth(1.0f);
					gl_obj->setVerticesPointSize(4.0f);
					gl_obj->setColor_u8(
						mrpt::img::TColor(0x00, 0x00, 0xff, 0xff));
					scene->insert(gl_obj);
				}
				{
					auto gl_obj =
						mrpt::make_aligned_shared<mrpt::opengl::CSetOfLines>();
					gl_obj->setName("score_phase1");
					gl_obj->setLineWidth(1.0f);
					gl_obj->setVerticesPointSize(2.0f);
					gl_obj->setColor_u8(
						mrpt::img::TColor(0xff, 0xff, 0x00, 0xff));
					scene->insert(gl_obj);
				}
				{
					auto gl_obj =
						mrpt::make_aligned_shared<mrpt::opengl::CSetOfLines>();
					gl_obj->setName("score_phase2");
					gl_obj->setLineWidth(1.0f);
					gl_obj->setVerticesPointSize(2.0f);
					gl_obj->setColor_u8(
						mrpt::img::TColor(0xff, 0xff, 0xff, 0xff));
					scene->insert(gl_obj);
				}
				{
					auto gl_obj =
						mrpt::make_aligned_shared<mrpt::opengl::CSetOfLines>();
					gl_obj->setName("tp_selected_dir");
					gl_obj->setLineWidth(3.0f);
					gl_obj->setColor_u8(
						mrpt::img::TColor(0x00, 0xff, 0x00, 0xd0));
					scene->insert(gl_obj);
				}
				{
					auto gl_obj =
						mrpt::make_aligned_shared<mrpt::opengl::CPointCloud>();
					gl_obj->setName("tp_target");
					gl_obj->setPointSize(5.0f);
					gl_obj->setColor_u8(
						mrpt::img::TColor(0x30, 0x30, 0x30, 0xff));
					gl_obj->setLocation(0, 0, 0.02f);
					scene->insert(gl_obj);
				}
				{
					auto gl_obj =
						mrpt::make_aligned_shared<mrpt::opengl::CPointCloud>();
					gl_obj->setName("tp_robot");
					gl_obj->setPointSize(4.0f);
					gl_obj->setColor_u8(
						mrpt::img::TColor(0xff, 0x00, 0x00, 0xa0));
					gl_obj->setLocation(0, 0, 0.02f);
					scene->insert(gl_obj);
				}
				{
					auto gl_obj =
						mrpt::make_aligned_shared<mrpt::opengl::CMesh>(
							true /*transparency*/);
					gl_obj->setName("tp_clearance");
					gl_obj->setScale(1.0f, 1.0f, 5.0f);
					scene->insert(gl_obj);
				}
			}  // End window locker:
		}

		{
			mrpt::opengl::COpenGLScene::Ptr scene;
			mrpt::gui::CDisplayWindow3DLocker locker(*win, scene);

			// Draw dynamic stuff:
			const CLogFileRecord::TInfoPerPTG& pI = log.infoPerPTG[nPTG];
			vector<float> xs, ys;

			const size_t nAlphas = pI.TP_Obstacles.size();

			win->clearTextMessages();

			win->addTextMessage(
				4, 4,
				format("[%u]:%s", nPTG, log.infoPerPTG[nPTG].PTG_desc.c_str()),
				is_selected_ptg ? TColorf(1.0f, 1.0f, 0.f)
								: TColorf(1.0f, 1.0f, 1.0f),
				"mono", 8, mrpt::opengl::NICE, 0 /*id*/, 1.5, 0.1,
				true /*shadow*/);

			// Chosen direction:
			{
				const double aDir = pI.desiredDirection;

				auto gl_obj =
					std::dynamic_pointer_cast<mrpt::opengl::CSetOfLines>(
						scene->getByName("tp_selected_dir"));
				gl_obj->clear();
				gl_obj->appendLine(
					0, 0, 0, pI.desiredSpeed * cos(aDir),
					pI.desiredSpeed * sin(aDir), 0);
			}

			// obstacles:
			xs.clear();
			ys.clear();
			xs.reserve(nAlphas);
			ys.reserve(nAlphas);
			for (size_t i = 0; i < nAlphas; ++i)
			{
				const double a = -M_PI + (i + 0.5) * 2 * M_PI / double(nAlphas);
				const double r = pI.TP_Obstacles[i];
				xs.push_back(r * cos(a));
				ys.push_back(r * sin(a));
			}
			{
				auto gl_obj =
					std::dynamic_pointer_cast<mrpt::opengl::CSetOfLines>(
						scene->getByName("tp_obstacles"));
				gl_obj->clear();
				if (nAlphas > 2)
				{
					gl_obj->appendLine(xs[0], ys[0], 0, xs[1], ys[1], 0);
					for (size_t i = 2; i < nAlphas; i++)
						gl_obj->appendLineStrip(xs[i], ys[i], 0);
				}
			}

			// Target:
			{
				auto gl_obj =
					std::dynamic_pointer_cast<mrpt::opengl::CPointCloud>(
						scene->getByName("tp_target"));
				gl_obj->clear();
				for (const auto& p : pI.TP_Targets)
				{
					gl_obj->insertPoint(p.x, p.y, .0);
				}

				if (!pI.TP_Targets.empty())
				{
					double ang =
						::atan2(pI.TP_Targets[0].y, pI.TP_Targets[0].x);
					int tp_target_k = 0;
					if (sel_ptg_idx < int(m_logdata_ptg_paths.size()) &&
						sel_ptg_idx >= 0)
					{
						mrpt::nav::CParameterizedTrajectoryGenerator::Ptr ptg =
							m_logdata_ptg_paths[sel_ptg_idx];
						if (ptg)
						{
							tp_target_k = ptg->alpha2index(ang);
						}
					}

					win->addTextMessage(
						4, -12,
						format(
							"TP_Target[0]=(%.02f,%.02f) k=%i ang=%.02f deg",
							pI.TP_Targets[0].x, pI.TP_Targets[0].y, tp_target_k,
							mrpt::RAD2DEG(ang)),
						TColorf(1.0f, 1.0f, 1.0f), "mono", 8,
						mrpt::opengl::NICE, 1 /*id*/, 1.5, 0.1,
						false /*shadow*/);
				}
			}
			if (cbList->IsChecked(m_cbIdx_ShowAllDebugFields))
			{
				unsigned int unique_id = 2;
				int lineY = 1;
				for (const auto& e : pI.evalFactors)
				{
					win->addTextMessage(
						4, 5 + (lineY++) * 11,
						mrpt::format("%20s=%6.03f", e.first.c_str(), e.second),
						mrpt::img::TColorf(1, 1, 1), "mono", 9,
						mrpt::opengl::NICE, unique_id++, 1.5, 0.1, true);
				}
			}

			// Current robot pt (normally in pure reactive, at (0,0)):
			{
				auto gl_obj =
					std::dynamic_pointer_cast<mrpt::opengl::CPointCloud>(
						scene->getByName("tp_robot"));
				gl_obj->clear();
				gl_obj->insertPoint(pI.TP_Robot.x, pI.TP_Robot.y, 0);
			}

			// Clearance-diagram:
			{
				auto gl_obj = std::dynamic_pointer_cast<mrpt::opengl::CMesh>(
					scene->getByName("tp_clearance"));
				if (pI.clearance.empty())
					gl_obj->setVisibility(false);
				else
				{
					gl_obj->setVisibility(true);
					pI.clearance.renderAs3DObject(
						*gl_obj, -1.0, 1.0, -1.0, 1.0, 0.15,
						cbList->IsChecked(
							m_cbIdx_ClearanceOverPath) /*interp over path*/);
				}
			}
			// Clearance-diagram:
			{
				auto gl_obj1 =
					std::dynamic_pointer_cast<mrpt::opengl::CSetOfLines>(
						scene->getByName("score_phase1"));
				auto gl_obj2 =
					std::dynamic_pointer_cast<mrpt::opengl::CSetOfLines>(
						scene->getByName("score_phase2"));
				const bool visible1 = rbPerPTGPlots->GetSelection() >= 1;
				const bool visible2 = rbPerPTGPlots->GetSelection() >= 2;
				gl_obj1->clear();
				gl_obj2->clear();
				gl_obj1->setVisibility(visible1);
				gl_obj2->setVisibility(visible2);

				if ((visible1 || visible2) && pI.HLFR && nAlphas > 2)
				{
					const auto& dir_evals = pI.HLFR->dirs_eval;
					const bool has_scores =
						!dir_evals.empty() && dir_evals[0].size() == nAlphas;
					const size_t num_scores = dir_evals.size();

					for (size_t iScore = 0; has_scores && iScore < num_scores;
						 iScore++)
					{
						vector<mrpt::math::TPoint2D> pts;
						pts.reserve(nAlphas);
						for (size_t i = 0; i < nAlphas; ++i)
						{
							const double a =
								-M_PI + (i + 0.5) * 2 * M_PI / double(nAlphas);
							const double r = dir_evals[iScore][i];
							pts.emplace_back(r * cos(a), r * sin(a));
						}

						mrpt::opengl::CSetOfLines::Ptr& gl_obj =
							iScore == (num_scores - 1) ? gl_obj1 : gl_obj2;

						gl_obj->appendLine(
							pts[0].x, pts[0].y, 0, pts[1].x, pts[1].y, 0);
						for (size_t i = 2; i < nAlphas; i++)
							gl_obj->appendLineStrip(pts[i].x, pts[i].y, 0);
					}
				}
			}

// In the case of ND algorithm: draw gaps
#if 0
			if (pI.HLFR && IS_CLASS(pI.HLFR, CLogFileRecord_ND))
			{
				CLogFileRecord_ND::Ptr log_ND = CLogFileRecord_ND::Ptr(pI.HLFR);
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

	}  // end for each PTG

	// Draw time cursor in v/w plots:
	{
		CDisplayWindowPlots::Ptr& win = m_mywins["VW"];
		if (win)
		{
			std::vector<double> xs(2), ys(2);
			xs[0] = log_idx;
			xs[1] = log_idx;
			ys[0] = -2.0;
			ys[1] = 2.0;
			win->plot(xs, ys, "k-3", "cursor_time");
		}
	}

	WX_END_TRY
}

void navlog_viewer_GUI_designDialog::OntimAutoloadTrigger(wxTimerEvent& event)
{
	if (!global_fileToOpen.empty() &&
		mrpt::system::fileExists(global_fileToOpen))
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
void navlog_viewer_GUI_designDialog::OnmnuMatlabPlotsSelected(
	wxCommandEvent& event)
{
	WX_START_TRY

	wxFileDialog dialog(
		this, _("Save MATLAB/Octave script that draws the log...") /*caption*/,
		_(".") /* defaultDir */, _("drawLog.m") /* defaultFilename */,
		_("MATLAB/Octave script (*.m)|*.m") /* wildcard */,
		wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
	if (dialog.ShowModal() != wxID_OK) return;

	const string filName(dialog.GetPath().mb_str());

	ofstream f(filName.c_str());
	if (!f.is_open()) throw runtime_error("Error writing to file!");

	f << "% Script for drawing navigation log\n"
	  << "% Generated automatically by navlog-viewer - MRPT "
	  << mrpt::system::MRPT_getVersion() << "\n"
	  << "%  From log: " << string(edLogFile->GetLabel().mbc_str()) << "\n"
	  << "% "
		 "---------------------------------------------------------------------"
		 "----\n\n";

	f << "%%\n"
	  << "function [] = main()\n"
	  << "figure;"
	  << "title('Path for " << mrpt::system::extractFileName(filName) << "');"
	  << "% Robot shape: (x,y) in each line\n"
	  << "rs = [-0.3 -0.3;0.6 -0.3;0.6 0.3;-0.3 0.3];\n"
	  << "dec_shps = 15;"
	  << "dec=0;";

	const int DECIMATE_POINTS = 10;
	int decim_point_cnt = 0;

	std::vector<float> X, Y;  // Obstacles
	std::vector<float> TX, TY;  // Target over time

	const size_t N = m_logdata.size();
	for (size_t i = 0; i < N; i++)
	{
		const CLogFileRecord::Ptr logsptr =
			std::dynamic_pointer_cast<CLogFileRecord>(m_logdata[i]);
		const CLogFileRecord* logptr = logsptr.get();

		const auto robotPose = logptr->robotPoseLocalization;
		CPose2D observationBasePose = CPose2D(robotPose);

		if (cbList->IsChecked(m_cbIdx_ShowDelays))
			observationBasePose =
				observationBasePose + CPose2D(logptr->relPoseSense);

		f << format(
			"dec=dec+1; if (dec>=dec_shps); drawRobotShape(rs,[%f %f %f]); "
			"dec=0; end\n",
			robotPose.x, robotPose.y, robotPose.phi);

		if (++decim_point_cnt >= DECIMATE_POINTS)
		{
			CSimplePointsMap pts;
			pts.changeCoordinatesReference(
				logptr->WS_Obstacles, CPose3D(observationBasePose));

			const auto& pX = pts.getPointsBufferRef_x();
			const auto& pY = pts.getPointsBufferRef_y();

			X.insert(X.begin(), pX.begin(), pX.end());
			Y.insert(Y.begin(), pY.begin(), pY.end());
		}

		// Target:
		const mrpt::math::TPoint2D trg_glob =
			mrpt::math::TPoint2D(robotPose + logptr->WS_targets_relative[0]);
		if (TX.empty() || std::abs((*TX.rbegin()) - trg_glob.x) > 1e-3 ||
			std::abs((*TY.rbegin()) - trg_glob.y) > 1e-3)
		{
			TX.push_back(trg_glob.x);
			TY.push_back(trg_glob.y);
		}
	}

	f << "\n % Points: \n"
	  << " Ps = [";
	for (size_t k = 0; k < X.size(); k++)
	{
		f << X[k] << " " << Y[k] << "\n";
	}

	f << "];\n"
	  << "plot(Ps(:,1),Ps(:,2),'k.','MarkerSize',3);\n";

	f << "\n % Target point: \n"
	  << " Ts = [";
	for (size_t k = 0; k < TX.size(); k++)
	{
		f << TX[k] << " " << TY[k] << "\n";
	}

	f << "];\n"
	  << "plot(Ts(:,1),Ts(:,2),'rx','MarkerSize',10);\n";

	f << "axis equal;\n"
	  << "\n";

	f << "%% drawRobotShape()\n"
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

void navlog_viewer_GUI_designDialog::OnmnuSeePTGParamsSelected(
	wxCommandEvent& event)
{
	WX_START_TRY

	const std::string sSection = "PTG_PARAMS";
	mrpt::config::CConfigFileMemory cfg;

	cfg.write(sSection, "PTG_COUNT", m_logdata_ptg_paths.size());

	CConfigFilePrefixer cfg_pre;
	cfg_pre.bind(cfg);

	for (size_t i = 0; i < m_logdata_ptg_paths.size(); i++)
	{
		mrpt::nav::CParameterizedTrajectoryGenerator::Ptr ptg =
			m_logdata_ptg_paths[i];
		if (!ptg) continue;

		const std::string sKeyPrefix = mrpt::format("PTG%d_", (int)i);
		cfg_pre.setPrefixes("", sKeyPrefix);

		ptg->saveToConfigFile(cfg_pre, sSection);
	}

	const std::string sCfgText = cfg.getContent();

	wxMessageBox(
		sCfgText.c_str(), _("PTG parameters as stored in the log:"), wxOK,
		this);

	WX_END_TRY
}

void navlog_viewer_GUI_designDialog::OncbGlobalFrameClick(wxCommandEvent& event)
{
	if (cbList->IsChecked(m_cbIdx_ShowCursor))
		timMouseXY.Start(100, false);
	else
		timMouseXY.Stop();

	wxScrollEvent d;
	OnslidLogCmdScroll(d);
}

void navlog_viewer_GUI_designDialog::OnmnuSaveScoreMatrixSelected(
	wxCommandEvent& event)
{
	WX_START_TRY

	wxFileDialog dialog(
		this, _("Save scores matrices...") /*caption*/, _(".") /* defaultDir */,
		_("scores.txt") /* defaultFilename */,
		_("MATLAB/Octave plain text file (*.txt)|*.txt") /* wildcard */,
		wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
	if (dialog.ShowModal() != wxID_OK) return;

	const string filName(dialog.GetPath().mb_str());
	const size_t N = m_logdata.size();
	for (size_t i = 0; i < N; i++)
	{
		const CLogFileRecord::Ptr logsptr =
			std::dynamic_pointer_cast<CLogFileRecord>(m_logdata[i]);
		const CLogFileRecord* logptr = logsptr.get();

		for (size_t iPTG = 0; iPTG < logptr->infoPerPTG.size(); iPTG++)
		{
			const CHolonomicLogFileRecord::Ptr& hlog =
				logptr->infoPerPTG[iPTG].HLFR;
			if (!hlog) continue;

			const mrpt::math::CMatrixD* dirs_scores =
				hlog->getDirectionScores();
			if (!dirs_scores || dirs_scores->rows() < 2) continue;

			const std::string sFil = mrpt::system::fileNameChangeExtension(
				filName, mrpt::format(
							 "step%06u_ptg%02u.txt", (unsigned int)i,
							 (unsigned int)iPTG));

			dirs_scores->saveToTextFile(sFil, mrpt::math::MATRIX_FORMAT_FIXED);
		}
	}

	WX_END_TRY
}

void navlog_viewer_GUI_designDialog::OntimMouseXY(wxTimerEvent& event)
{
	// Mouse position at Z=0
	CDisplayWindow3D::Ptr& win1 = m_mywins3D["WS_obs"];
	if (!win1) return;

	int lineY = 0, unique_id = 0;

	{
		mrpt::math::TLine3D mouse_ray;
		win1->getLastMousePositionRay(mouse_ray);

		// Create a 3D plane, e.g. Z=0
		const mrpt::math::TPlane ground_plane(
			TPoint3D(0, 0, 0), TPoint3D(1, 0, 0), TPoint3D(0, 1, 0));
		// Intersection of the line with the plane:
		mrpt::math::TObject3D inters;
		mrpt::math::intersect(mouse_ray, ground_plane, inters);
		// Interpret the intersection as a point, if there is an intersection:
		mrpt::math::TPoint3D inters_pt;
		if (inters.getPoint(inters_pt))
		{
			ADD_WIN_TEXTMSG(mrpt::format(
				"Mouse pos: X=%.04f  Y=%.04f", inters_pt.x, inters_pt.y));
			win1->repaint();
		}
	}
}

void navlog_viewer_GUI_designDialog::OnmnuMatlabExportPaths(
	wxCommandEvent& event)
{
	WX_START_TRY;

	const size_t N = m_logdata.size();
	ASSERTMSG_(N > 0, "Log is empty! Load a valid log first...");

	wxFileDialog dialog(
		this, _("Save MATLAB/Octave script with path info...") /*caption*/,
		_(".") /* defaultDir */,
		(mrpt::system::extractFileName(
			 std::string(this->edLogFile->GetValue().mb_str())) +
		 std::string("_log.m"))
			.c_str() /* defaultFilename */,
		_("MATLAB/Octave script (*.m)|*.m") /* wildcard */,
		wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
	if (dialog.ShowModal() != wxID_OK) return;

	const string filName(dialog.GetPath().mb_str());

	ofstream f(filName.c_str());
	if (!f.is_open()) throw runtime_error("Error writing to file!");

	f << "% Script for drawing navigation paths\n"
	  << "% Generated automatically by navlog-viewer - MRPT "
	  << mrpt::system::MRPT_getVersion() << "\n"
	  << "%  From log: " << string(edLogFile->GetLabel().mbc_str()) << "\n"
	  << "% "
		 "---------------------------------------------------------------------"
		 "----\n\n";

	std::map<double, int> selected_PTG_over_time;  // time: tim_start_iteration
	std::map<double, double> iteration_duration;  // time: tim_start_iteration
	struct TRobotPoseVel
	{
		mrpt::math::TPose2D pose, poseOdom;
		mrpt::math::TTwist2D velGlobal, velLocal;
	};
	std::map<double, TRobotPoseVel> global_local_vel;  // time: curPoseAndVel
	std::map<double, double> vals_timoff_obstacles, vals_timoff_curPoseVelAge,
		vals_timoff_sendVelCmd;  // time: tim_start_iteration

	const int MAX_CMDVEL_COMPONENTS = 15;
	using cmdvel_vector_t = Eigen::Matrix<double, 1, MAX_CMDVEL_COMPONENTS>;
	mrpt::aligned_std_map<double, cmdvel_vector_t>
		cmdvels;  // time: tim_send_cmd_vel

	double tim_start_iteration = .0;

	for (size_t i = 0; i < N; i++)
	{
		const CLogFileRecord::Ptr logsptr =
			std::dynamic_pointer_cast<CLogFileRecord>(m_logdata[i]);
		const CLogFileRecord* logptr = logsptr.get();

		{
			const auto it = logptr->timestamps.find("tim_start_iteration");
			if (it != logptr->timestamps.end())
			{
				tim_start_iteration =
					mrpt::system::timestampToDouble(it->second);
			}
			else
			{
				tim_start_iteration++;  // just in case we don't have valid
				// iteration timestamp (??)
			}
		}

		// selected PTG:
		selected_PTG_over_time[tim_start_iteration] = logptr->nSelectedPTG;

		// iter dur:
		{
			auto itExecT = logptr->values.find("executionTime");
			if (itExecT != logptr->values.end())
				iteration_duration[tim_start_iteration] = itExecT->second;
		}

		// timoff_obstacles;
		{
			const auto it = logptr->values.find("timoff_obstacles");
			if (it != logptr->values.end())
				vals_timoff_obstacles[tim_start_iteration] = it->second;
		}
		// timoff_curPoseVelAge;
		{
			const auto it = logptr->values.find("timoff_curPoseVelAge");
			if (it != logptr->values.end())
				vals_timoff_curPoseVelAge[tim_start_iteration] = it->second;
		}
		// timoff_sendVelCmd
		{
			const auto it = logptr->values.find("timoff_sendVelCmd");
			if (it != logptr->values.end())
				vals_timoff_sendVelCmd[tim_start_iteration] = it->second;
		}

		// curPoseAndVel:
		{
			double tim_pose = tim_start_iteration;  // default

			const auto it = logptr->timestamps.find("curPoseAndVel");
			if (it != logptr->timestamps.end())
			{
				tim_pose = mrpt::system::timestampToDouble(it->second);
			}

			auto& p = global_local_vel[tim_pose];
			p.pose = logptr->robotPoseLocalization;
			p.poseOdom = logptr->robotPoseOdometry;
			p.velGlobal = logptr->cur_vel;
			p.velLocal = logptr->cur_vel_local;
		}

		// send cmd vels:
		if (logptr->cmd_vel)
		{
			double tim_send_cmd_vel = tim_start_iteration;  // default
			const auto it = logptr->timestamps.find("tim_send_cmd_vel");
			if (it != logptr->timestamps.end())
			{
				tim_send_cmd_vel = mrpt::system::timestampToDouble(it->second);
			}

			auto& p = cmdvels[tim_send_cmd_vel];
			p.setZero();
			for (size_t k = 0; k < logptr->cmd_vel->getVelCmdLength(); k++)
				p(k) = logptr->cmd_vel->getVelCmdElement(k);
		}

	}  // end for each timestep

	double t_ref = 0;
	if (!selected_PTG_over_time.empty())
	{
		t_ref = selected_PTG_over_time.begin()->first;
	}

	f << "clear; close all;\n";

	f << "% robot pose over time. Columns: [time curPoseAndVel, x,y,phi_rad,  "
		 "odo_x,odo_y,odo_phi_rad]\n"
		 "robot_pose = [";
	for (const auto& e : global_local_vel)
	{
		f << (e.first - t_ref) << "," << e.second.pose.x << ","
		  << e.second.pose.y << "," << e.second.pose.phi << ", "
		  << e.second.poseOdom.x << "," << e.second.poseOdom.y << ","
		  << e.second.poseOdom.phi << " ; ";
	}
	f << "];\n"
		 "robot_pose(:,4) = unwrap(robot_pose(:,4));\n"
		 "figure(); subplot(2,1,1); \n"
		 "plot(robot_pose(:, 1), robot_pose(:, 2 : 4), '.', robot_pose(:, 1), "
		 "robot_pose(:, 2 : 4), '-'); xlabel('Time'); legend('x', 'y', 'phi "
		 "(rad)'); title('robot pose'); \n"
		 "xl=xlim; subplot(2,1,2);\n"
		 "robot_pose_diff = diff(robot_pose(:, 2 : 4));\n"
		 "idxs_rob_incr_z = find(robot_pose_diff(:, 1) == 0 & "
		 "robot_pose_diff(:, 2) == 0 & robot_pose_diff(:, 3) == 0);\n"
		 "plot(robot_pose(2:end, 1), diff(robot_pose(:, 2 : 4)), '.');\n"
		 "hold on;\n"
		 "plot(robot_pose(idxs_rob_incr_z, 1), "
		 "robot_pose_diff(idxs_rob_incr_z, :), 'k', 'MarkerSize', 11, "
		 "'Marker', 'square');\n"
		 "xlabel('Time'); legend('x', 'y', 'phi (rad)'); title('Robot pose "
		 "*increments*');\n"
		 "xlim(xl);\n\n";

	f << "% Selected PTG over time. Columns: [tim_start_iteration, 0-based "
		 "index selected PTG]\n"
		 "selected_PTG = [";
	for (const auto& e : selected_PTG_over_time)
	{
		f << (e.first - t_ref) << "," << e.second << " ; ";
	}
	f << "];\n"
		 "figure(); plot(selected_PTG(:,1),selected_PTG(:,2), 'x'); "
		 "xlabel('Time'); ylabel('Selected PTG');\n\n";

	if (!iteration_duration.empty())
	{
		f << "% Iteration duration. Columns: [tim_start_iteration, "
			 "iter_duration_seconds]\n"
			 "iteration_duration = [";
		for (const auto& e : iteration_duration)
		{
			f << (e.first - t_ref) << "," << e.second << " ; ";
		}
		f << "];\n"
			 "figure(); plot(iteration_duration(:,1),iteration_duration(:,2), "
			 "'x');\n"
			 "hold on;\n"
			 "plot(selected_PTG(1:(end-1),1),diff(selected_PTG(:,1)),'.')"
			 ";"
			 "xlabel('Time'); legend('Iteration duration', 'Diff consecutive "
			 "call time'); title('rnav_iter_call_time_duration');\n\n";
	}

	if (!vals_timoff_obstacles.empty())
	{
		f << "% vals_timoff_obstacles. Columns: [tim_start_iteration, value]\n"
			 "timoff_obstacles = [";
		for (const auto& e : vals_timoff_obstacles)
		{
			f << (e.first - t_ref) << " , " << e.second << " ; ";
		}
		f << "];\n"
			 "figure(); plot(timoff_obstacles(:,1),timoff_obstacles(:,2), "
			 "'.',timoff_obstacles(:,1),timoff_obstacles(:,2), '-'); "
			 "xlabel('Time'); title('timoff_obstacles');\n\n";
	}
	if (!vals_timoff_curPoseVelAge.empty())
	{
		f << "% vals_timoff_curPoseVelAge. Columns: [tim_start_iteration, "
			 "value]\n"
			 "timoff_curPoseVelAge = [";
		for (const auto& e : vals_timoff_curPoseVelAge)
		{
			f << (e.first - t_ref) << " , " << e.second << " ; ";
		}
		f << "];\n"
			 "figure(); "
			 "plot(timoff_curPoseVelAge(:,1),timoff_curPoseVelAge(:,2), "
			 "'.',timoff_curPoseVelAge(:,1),timoff_curPoseVelAge(:,2), '-'); "
			 "xlabel('Time'); title('timoff_curPoseVelAge');\n\n";
	}
	if (!vals_timoff_sendVelCmd.empty())
	{
		f << "% vals_timoff_sendVelCmd. Columns: [tim_start_iteration, value]\n"
			 "timoff_sendVelCmd = [";
		for (const auto& e : vals_timoff_sendVelCmd)
		{
			f << (e.first - t_ref) << " , " << e.second << " ; ";
		}
		f << "];\n"
			 "figure(); plot(timoff_sendVelCmd (:,1),timoff_sendVelCmd (:,2), "
			 "'.',timoff_sendVelCmd (:,1),timoff_sendVelCmd (:,2), '-'); "
			 "xlabel('Time'); title('timoff_sendVelCmd ');\n\n";
	}

	f << "% robot vel over time. Columns: [time curPoseAndVel, "
		 "vx,vy,omega_rad_sec]\n"
		 "robot_vel_global = [";
	for (const auto& e : global_local_vel)
	{
		f << (e.first - t_ref) << "," << e.second.velGlobal.vx << ","
		  << e.second.velGlobal.vy << "," << e.second.velGlobal.omega << " ; ";
	}
	f << "];\n"
		 "figure(); plot(robot_vel_global(:,1),robot_vel_global(:,2:4), '.', "
		 "robot_vel_global(:,1),robot_vel_global(:,2:4), '-'); xlabel('Time'); "
		 "title('Velocities (global)'); legend('vx','vy','omega');\n\n";

	f << "robot_vel_local = [";
	for (const auto& e : global_local_vel)
	{
		f << (e.first - t_ref) << "," << e.second.velLocal.vx << ","
		  << e.second.velLocal.vy << "," << e.second.velLocal.omega << " ; ";
	}
	f << "];\n"
		 "figure(); plot(robot_vel_local(:,1),robot_vel_local(:,2:4), '.', "
		 "robot_vel_local(:,1),robot_vel_local(:,2:4), '-'); xlabel('Time'); "
		 "title('Velocities (local)'); legend('vx','vy','omega');\n\n";

	// cmdvels:
	f << "% Movement commands sent to robot. Columns: [time curPoseAndVel, "
		 "vx,vy,omega_rad_sec]\n"
		 "cmdvels = [";
	for (const auto& e : cmdvels)
	{
		f << (e.first - t_ref) << " ";
		f << e.second << " ; ";
	}
	f << "];\n"
		 "figure(); plot(cmdvels(:,1),cmdvels(:,2:"
	  << (MAX_CMDVEL_COMPONENTS + 1)
	  << "), '.', cmdvels(:,1),cmdvels(:,2:" << (MAX_CMDVEL_COMPONENTS + 1)
	  << "), '-'); xlabel('Time'); title('Issued motion commands (meaning "
		 "CVehicleVelCmd-dependend)');\n\n";

	WX_END_TRY;
}

void navlog_viewer_GUI_designDialog::OnrbPerPTGPlotsSelect(
	wxCommandEvent& event)
{
	wxScrollEvent d;
	OnslidLogCmdScroll(d);
}
