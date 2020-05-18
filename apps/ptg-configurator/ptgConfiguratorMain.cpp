/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "ptgConfiguratorMain.h"
#include <mrpt/gui/about_box.h>
#include <wx/msgdlg.h>

//(*InternalHeaders(ptgConfiguratorframe)
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/font.h>
#include <wx/image.h>
#include <wx/intl.h>
#include <wx/settings.h>
#include <wx/string.h>
//*)
#include <mrpt/system/os.h>

#include <mrpt/gui/WxUtils.h>
#include <mrpt/gui/wx28-fixes.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/system/string_utils.h>
#include "../wx-common/mrpt_logo.xpm"
#include "imgs/main_icon.xpm"

// A custom Art provider for customizing the icons:
class MyArtProvider : public wxArtProvider
{
   protected:
	wxBitmap CreateBitmap(
		const wxArtID& id, const wxArtClient& client,
		const wxSize& size) override;
};

// CreateBitmap function
wxBitmap MyArtProvider::CreateBitmap(
	const wxArtID& id, const wxArtClient& client, const wxSize& size)
{
	if (id == wxART_MAKE_ART_ID(MAIN_ICON)) return wxBitmap(main_icon_xpm);
	if (id == wxART_MAKE_ART_ID(IMG_MRPT_LOGO)) return wxBitmap(mrpt_logo_xpm);
	// Any wxWidgets icons not implemented here
	// will be provided by the default art provider.
	return wxNullBitmap;
}

#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/config/CConfigFilePrefixer.h>
#include <mrpt/math/geometry.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/os.h>

mrpt::nav::CParameterizedTrajectoryGenerator::Ptr ptg;

//(*IdInit(ptgConfiguratorframe)
const long ptgConfiguratorframe::ID_STATICTEXT1 = wxNewId();
const long ptgConfiguratorframe::ID_CHOICE1 = wxNewId();
const long ptgConfiguratorframe::ID_STATICTEXT5 = wxNewId();
const long ptgConfiguratorframe::ID_BUTTON1 = wxNewId();
const long ptgConfiguratorframe::ID_STATICTEXT2 = wxNewId();
const long ptgConfiguratorframe::ID_SPINCTRL1 = wxNewId();
const long ptgConfiguratorframe::ID_CHECKBOX1 = wxNewId();
const long ptgConfiguratorframe::ID_STATICTEXT4 = wxNewId();
const long ptgConfiguratorframe::ID_TEXTCTRL5 = wxNewId();
const long ptgConfiguratorframe::ID_BUTTON5 = wxNewId();
const long ptgConfiguratorframe::ID_CHECKBOX3 = wxNewId();
const long ptgConfiguratorframe::ID_SLIDER1 = wxNewId();
const long ptgConfiguratorframe::ID_SPINCTRL2 = wxNewId();
const long ptgConfiguratorframe::ID_CHECKBOX4 = wxNewId();
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
const long ptgConfiguratorframe::ID_STATICTEXT17 = wxNewId();
const long ptgConfiguratorframe::ID_TEXTCTRL8 = wxNewId();
const long ptgConfiguratorframe::ID_BUTTON4 = wxNewId();
const long ptgConfiguratorframe::ID_TEXTCTRL1 = wxNewId();
const long ptgConfiguratorframe::ID_TEXTCTRL2 = wxNewId();
const long ptgConfiguratorframe::ID_PANEL1 = wxNewId();
const long ptgConfiguratorframe::ID_XY_GLCANVAS = wxNewId();
const long ptgConfiguratorframe::ID_CHECKBOX5 = wxNewId();
const long ptgConfiguratorframe::ID_CHECKBOX6 = wxNewId();
const long ptgConfiguratorframe::ID_CHECKBOX7 = wxNewId();
const long ptgConfiguratorframe::ID_CUSTOM2 = wxNewId();
const long ptgConfiguratorframe::ID_PANEL2 = wxNewId();
const long ptgConfiguratorframe::ID_CUSTOM1 = wxNewId();
const long ptgConfiguratorframe::ID_PANEL3 = wxNewId();
const long ptgConfiguratorframe::ID_STATICTEXT8 = wxNewId();
const long ptgConfiguratorframe::ID_CUSTOM3 = wxNewId();
const long ptgConfiguratorframe::ID_STATICTEXT9 = wxNewId();
const long ptgConfiguratorframe::ID_CUSTOM4 = wxNewId();
const long ptgConfiguratorframe::ID_PANEL4 = wxNewId();
const long ptgConfiguratorframe::ID_STATICTEXT10 = wxNewId();
const long ptgConfiguratorframe::ID_CUSTOM5 = wxNewId();
const long ptgConfiguratorframe::ID_STATICTEXT11 = wxNewId();
const long ptgConfiguratorframe::ID_CUSTOM6 = wxNewId();
const long ptgConfiguratorframe::ID_STATICTEXT12 = wxNewId();
const long ptgConfiguratorframe::ID_CUSTOM7 = wxNewId();
const long ptgConfiguratorframe::ID_STATICTEXT16 = wxNewId();
const long ptgConfiguratorframe::ID_CUSTOM11 = wxNewId();
const long ptgConfiguratorframe::ID_PANEL5 = wxNewId();
const long ptgConfiguratorframe::ID_STATICTEXT13 = wxNewId();
const long ptgConfiguratorframe::ID_CUSTOM8 = wxNewId();
const long ptgConfiguratorframe::ID_STATICTEXT14 = wxNewId();
const long ptgConfiguratorframe::ID_CUSTOM9 = wxNewId();
const long ptgConfiguratorframe::ID_STATICTEXT15 = wxNewId();
const long ptgConfiguratorframe::ID_CUSTOM10 = wxNewId();
const long ptgConfiguratorframe::ID_PANEL6 = wxNewId();
const long ptgConfiguratorframe::ID_NOTEBOOK1 = wxNewId();
const long ptgConfiguratorframe::idMenuQuit = wxNewId();
const long ptgConfiguratorframe::idMenuAbout = wxNewId();
const long ptgConfiguratorframe::ID_STATUSBAR1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(ptgConfiguratorframe, wxFrame)
//(*EventTable(ptgConfiguratorframe)
//*)
END_EVENT_TABLE()

ptgConfiguratorframe::ptgConfiguratorframe(wxWindow* parent, wxWindowID id)
	: m_cursorPickState(cpsNone)
{
	// Load my custom icons:
	wxArtProvider::Push(new MyArtProvider);

	//(*Initialize(ptgConfiguratorframe)
	wxFlexGridSizer* FlexGridSizer4;
	wxMenuItem* MenuItem2;
	wxFlexGridSizer* FlexGridSizer10;
	wxFlexGridSizer* FlexGridSizer3;
	wxMenuItem* MenuItem1;
	wxFlexGridSizer* FlexGridSizer9;
	wxFlexGridSizer* FlexGridSizer2;
	wxMenu* Menu1;
	wxFlexGridSizer* FlexGridSizer7;
	wxFlexGridSizer* FlexGridSizer15;
	wxFlexGridSizer* FlexGridSizer8;
	wxFlexGridSizer* FlexGridSizer14;
	wxFlexGridSizer* FlexGridSizer13;
	wxFlexGridSizer* FlexGridSizer12;
	wxMenuBar* MenuBar1;
	wxFlexGridSizer* FlexGridSizer6;
	wxStaticBoxSizer* StaticBoxSizer1;
	wxFlexGridSizer* FlexGridSizer1;
	wxFlexGridSizer* FlexGridSizer11;
	wxMenu* Menu2;

	Create(
		parent, wxID_ANY, _("PTG configurator - Part of the MRPT project"),
		wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE,
		_T("wxID_ANY"));
	// SetClientSize(wxSize(893, 576));
	{
		wxIcon FrameIcon;
		FrameIcon.CopyFromBitmap(wxArtProvider::GetBitmap(
			wxART_MAKE_ART_ID_FROM_STR(_T("MAIN_ICON")), wxART_OTHER));
		SetIcon(FrameIcon);
	}
	FlexGridSizer1 = new wxFlexGridSizer(0, 1, 0, 0);
	FlexGridSizer1->AddGrowableCol(0);
	FlexGridSizer1->AddGrowableRow(1);
	Panel1 = new wxPanel(
		this, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL,
		_T("ID_PANEL1"));
	FlexGridSizer2 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer2->AddGrowableCol(0);
	FlexGridSizer2->AddGrowableRow(0);
	FlexGridSizer3 = new wxFlexGridSizer(0, 1, 0, 0);
	FlexGridSizer3->AddGrowableCol(0);
	FlexGridSizer3->AddGrowableRow(3);
	FlexGridSizer7 = new wxFlexGridSizer(1, 0, 0, 0);
	StaticText1 = new wxStaticText(
		Panel1, ID_STATICTEXT1, _("Select a PTG class:"), wxDefaultPosition,
		wxDefaultSize, 0, _T("ID_STATICTEXT1"));
	FlexGridSizer7->Add(
		StaticText1, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	cbPTGClass = new wxChoice(
		Panel1, ID_CHOICE1, wxDefaultPosition, wxDefaultSize, 0, nullptr,
		wxCB_SORT, wxDefaultValidator, _T("ID_CHOICE1"));
	FlexGridSizer7->Add(
		cbPTGClass, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
	StaticText5 = new wxStaticText(
		Panel1, ID_STATICTEXT5,
		_("then change params as desired and click `Initialize PTG`"),
		wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT5"));
	FlexGridSizer7->Add(
		StaticText5, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	btnReloadParams = new wxButton(
		Panel1, ID_BUTTON1, _("Initialize PTG"), wxDefaultPosition,
		wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
	wxFont btnReloadParamsFont(
		wxDEFAULT, wxDEFAULT, wxFONTSTYLE_NORMAL, wxBOLD, false, wxEmptyString,
		wxFONTENCODING_DEFAULT);
	btnReloadParams->SetFont(btnReloadParamsFont);
	FlexGridSizer7->Add(
		btnReloadParams, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer3->Add(
		FlexGridSizer7, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	FlexGridSizer4 = new wxFlexGridSizer(0, 6, 0, 0);
	StaticText2 = new wxStaticText(
		Panel1, ID_STATICTEXT2, _("PTG index for cfg file:"), wxDefaultPosition,
		wxDefaultSize, 0, _T("ID_STATICTEXT2"));
	FlexGridSizer4->Add(
		StaticText2, 1, wxALL | wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL, 5);
	edPTGIndex = new wxSpinCtrl(
		Panel1, ID_SPINCTRL1, _T("0"), wxDefaultPosition, wxDefaultSize, 0, 0,
		100, 0, _T("ID_SPINCTRL1"));
	edPTGIndex->SetValue(_T("0"));
	FlexGridSizer4->Add(
		edPTGIndex, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
	cbDrawShapePath = new wxCheckBox(
		Panel1, ID_CHECKBOX1, _("Draw robot shape"), wxDefaultPosition,
		wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
	cbDrawShapePath->SetValue(true);
	FlexGridSizer4->Add(
		cbDrawShapePath, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	StaticText4 = new wxStaticText(
		Panel1, ID_STATICTEXT4, _("Dist. btw. robot shapes:"),
		wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT4"));
	FlexGridSizer4->Add(
		StaticText4, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	edMinDistBtwShapes = new wxTextCtrl(
		Panel1, ID_TEXTCTRL5, _("0.5"), wxDefaultPosition, wxSize(50, -1), 0,
		wxDefaultValidator, _T("ID_TEXTCTRL5"));
	FlexGridSizer4->Add(
		edMinDistBtwShapes, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	btnLoadPlugin = new wxButton(
		Panel1, ID_BUTTON5, _("Load Plugin"), wxDefaultPosition, wxDefaultSize,
		0, wxDefaultValidator, _T("ID_BUTTON5"));
	FlexGridSizer4->Add(
		btnLoadPlugin, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	cbHighlightOnePath = new wxCheckBox(
		Panel1, ID_CHECKBOX3, _("Highlight one trajectory:"), wxDefaultPosition,
		wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX3"));
	cbHighlightOnePath->SetValue(true);
	FlexGridSizer4->Add(
		cbHighlightOnePath, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	slidPathHighlight = new wxSlider(
		Panel1, ID_SLIDER1, 0, 0, 100, wxDefaultPosition, wxDefaultSize, 0,
		wxDefaultValidator, _T("ID_SLIDER1"));
	FlexGridSizer4->Add(
		slidPathHighlight, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	edIndexHighlightPath = new wxSpinCtrl(
		Panel1, ID_SPINCTRL2, _T("0"), wxDefaultPosition, wxDefaultSize, 0, 0,
		100, 0, _T("ID_SPINCTRL2"));
	edIndexHighlightPath->SetValue(_T("0"));
	FlexGridSizer4->Add(
		edIndexHighlightPath, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	cbShowOnlySelectedTraj = new wxCheckBox(
		Panel1, ID_CHECKBOX4, _("Show only selected traj."), wxDefaultPosition,
		wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX4"));
	cbShowOnlySelectedTraj->SetValue(false);
	FlexGridSizer4->Add(
		cbShowOnlySelectedTraj, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer4->Add(
		-1, -1, 1, wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL,
		5);
	FlexGridSizer3->Add(
		FlexGridSizer4, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	FlexGridSizer8 = new wxFlexGridSizer(1, 0, 0, 0);
	cbBuildTPObs = new wxCheckBox(
		Panel1, ID_CHECKBOX2, _("Obstacle point: x="), wxDefaultPosition,
		wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX2"));
	cbBuildTPObs->SetValue(true);
	FlexGridSizer8->Add(
		cbBuildTPObs, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	edObsX = new wxTextCtrl(
		Panel1, ID_TEXTCTRL3, _("9.0"), wxDefaultPosition, wxSize(50, -1), 0,
		wxDefaultValidator, _T("ID_TEXTCTRL3"));
	FlexGridSizer8->Add(
		edObsX, 1, wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL,
		5);
	StaticText3 = new wxStaticText(
		Panel1, ID_STATICTEXT3, _("y="), wxDefaultPosition, wxDefaultSize, 0,
		_T("ID_STATICTEXT3"));
	FlexGridSizer8->Add(
		StaticText3, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	edObsY = new wxTextCtrl(
		Panel1, ID_TEXTCTRL4, _("3.0"), wxDefaultPosition, wxSize(50, -1), 0,
		wxDefaultValidator, _T("ID_TEXTCTRL4"));
	FlexGridSizer8->Add(
		edObsY, 1, wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL,
		5);
	btnPlaceObs = new wxButton(
		Panel1, ID_BUTTON3, _("Click to place obstacle..."), wxDefaultPosition,
		wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON3"));
	FlexGridSizer8->Add(
		btnPlaceObs, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	btnRebuildTPObs = new wxButton(
		Panel1, ID_BUTTON2, _("Rebuild TP-Obs now"), wxDefaultPosition,
		wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
	FlexGridSizer8->Add(
		btnRebuildTPObs, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	StaticText6 = new wxStaticText(
		Panel1, ID_STATICTEXT6, _("Target: x="), wxDefaultPosition,
		wxDefaultSize, 0, _T("ID_STATICTEXT6"));
	FlexGridSizer8->Add(
		StaticText6, 1, wxALL | wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL, 5);
	edTargetX = new wxTextCtrl(
		Panel1, ID_TEXTCTRL6, _("5.0"), wxDefaultPosition, wxSize(31, -1), 0,
		wxDefaultValidator, _T("ID_TEXTCTRL6"));
	FlexGridSizer8->Add(
		edTargetX, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	StaticText7 = new wxStaticText(
		Panel1, ID_STATICTEXT7, _("y="), wxDefaultPosition, wxDefaultSize, 0,
		_T("ID_STATICTEXT7"));
	FlexGridSizer8->Add(
		StaticText7, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	edTargetY = new wxTextCtrl(
		Panel1, ID_TEXTCTRL7, _("1.0"), wxDefaultPosition, wxSize(35, -1), 0,
		wxDefaultValidator, _T("ID_TEXTCTRL7"));
	FlexGridSizer8->Add(
		edTargetY, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	StaticText17 = new wxStaticText(
		Panel1, ID_STATICTEXT17, _("RelSpeed="), wxDefaultPosition,
		wxDefaultSize, 0, _T("ID_STATICTEXT17"));
	FlexGridSizer8->Add(
		StaticText17, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	edRelSpeedAtTarget = new wxTextCtrl(
		Panel1, ID_TEXTCTRL8, _("0.0"), wxDefaultPosition, wxSize(34, -1), 0,
		wxDefaultValidator, _T("ID_TEXTCTRL8"));
	FlexGridSizer8->Add(
		edRelSpeedAtTarget, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	btnPlaceTarget = new wxButton(
		Panel1, ID_BUTTON4, _("Click to place target..."), wxDefaultPosition,
		wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON4"));
	FlexGridSizer8->Add(
		btnPlaceTarget, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer3->Add(
		FlexGridSizer8, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	FlexGridSizer6 = new wxFlexGridSizer(1, 2, 0, 0);
	FlexGridSizer6->AddGrowableCol(0);
	FlexGridSizer6->AddGrowableCol(1);
	FlexGridSizer6->AddGrowableRow(0);
	edCfg = new wxTextCtrl(
		Panel1, ID_TEXTCTRL1, wxEmptyString, wxDefaultPosition, wxSize(-1, 150),
		wxTE_PROCESS_ENTER | wxTE_PROCESS_TAB | wxTE_MULTILINE | wxHSCROLL |
			wxTE_DONTWRAP | wxALWAYS_SHOW_SB,
		wxDefaultValidator, _T("ID_TEXTCTRL1"));
	wxFont edCfgFont = wxSystemSettings::GetFont(wxSYS_OEM_FIXED_FONT);
	if (!edCfgFont.Ok())
		edCfgFont = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
	edCfgFont.SetPointSize(8);
	edCfgFont.SetFamily(wxFONTFAMILY_TELETYPE);
	edCfg->SetFont(edCfgFont);
	FlexGridSizer6->Add(
		edCfg, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 2);
	edLog = new wxTextCtrl(
		Panel1, ID_TEXTCTRL2, wxEmptyString, wxDefaultPosition, wxSize(-1, 100),
		wxTE_PROCESS_ENTER | wxTE_PROCESS_TAB | wxTE_MULTILINE | wxTE_READONLY |
			wxHSCROLL | wxTE_DONTWRAP | wxALWAYS_SHOW_SB,
		wxDefaultValidator, _T("ID_TEXTCTRL2"));
	wxFont edLogFont = wxSystemSettings::GetFont(wxSYS_OEM_FIXED_FONT);
	if (!edLogFont.Ok())
		edLogFont = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
	edLogFont.SetPointSize(8);
	edLogFont.SetFamily(wxFONTFAMILY_TELETYPE);
	edLog->SetFont(edLogFont);
	FlexGridSizer6->Add(
		edLog, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	FlexGridSizer3->Add(
		FlexGridSizer6, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	FlexGridSizer2->Add(
		FlexGridSizer3, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	Panel1->SetSizer(FlexGridSizer2);
	FlexGridSizer2->Fit(Panel1);
	FlexGridSizer2->SetSizeHints(Panel1);
	FlexGridSizer1->Add(
		Panel1, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	FlexGridSizer9 = new wxFlexGridSizer(1, 2, 0, 0);
	FlexGridSizer9->AddGrowableCol(0);
	FlexGridSizer9->AddGrowableCol(1);
	FlexGridSizer9->AddGrowableRow(0);
	m_plot = new CMyGLCanvas(
		this, ID_XY_GLCANVAS, wxDefaultPosition, wxSize(150, 300),
		wxTAB_TRAVERSAL, _T("ID_XY_GLCANVAS"));
	FlexGridSizer9->Add(
		m_plot, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	Notebook1 = new wxNotebook(
		this, ID_NOTEBOOK1, wxDefaultPosition, wxDefaultSize, 0,
		_T("ID_NOTEBOOK1"));
	Panel2 = new wxPanel(
		Notebook1, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL,
		_T("ID_PANEL2"));
	FlexGridSizer10 = new wxFlexGridSizer(2, 1, 0, 0);
	FlexGridSizer10->AddGrowableCol(0);
	FlexGridSizer10->AddGrowableRow(1);
	StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, Panel2, _("Show:"));
	FlexGridSizer15 = new wxFlexGridSizer(0, 4, 0, 0);
	cbShowTPObs = new wxCheckBox(
		Panel2, ID_CHECKBOX5, _("TP-Obstacles"), wxDefaultPosition,
		wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX5"));
	cbShowTPObs->SetValue(true);
	FlexGridSizer15->Add(
		cbShowTPObs, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	cbShowClearance = new wxCheckBox(
		Panel2, ID_CHECKBOX6, _("Clearance diagram"), wxDefaultPosition,
		wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX6"));
	cbShowClearance->SetValue(true);
	FlexGridSizer15->Add(
		cbShowClearance, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	cbClearanceInterp = new wxCheckBox(
		Panel2, ID_CHECKBOX7, _("Interpolate clearance diagram"),
		wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator,
		_T("ID_CHECKBOX7"));
	cbClearanceInterp->SetValue(false);
	FlexGridSizer15->Add(
		cbClearanceInterp, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	StaticBoxSizer1->Add(
		FlexGridSizer15, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	FlexGridSizer10->Add(
		StaticBoxSizer1, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 2);
	m_plotTPSpace = new CMyGLCanvas(
		Panel2, ID_CUSTOM2, wxDefaultPosition, wxSize(150, 300),
		wxTAB_TRAVERSAL, _T("ID_CUSTOM2"));
	FlexGridSizer10->Add(
		m_plotTPSpace, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	Panel2->SetSizer(FlexGridSizer10);
	FlexGridSizer10->Fit(Panel2);
	FlexGridSizer10->SetSizeHints(Panel2);
	Panel3 = new wxPanel(
		Notebook1, ID_PANEL3, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL,
		_T("ID_PANEL3"));
	FlexGridSizer11 = new wxFlexGridSizer(0, 1, 0, 0);
	FlexGridSizer11->AddGrowableCol(0);
	FlexGridSizer11->AddGrowableRow(0);
	m_plotVelCmds =
		new mpWindow(Panel3, ID_CUSTOM1, wxDefaultPosition, wxDefaultSize, 0);
	FlexGridSizer11->Add(
		m_plotVelCmds, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	Panel3->SetSizer(FlexGridSizer11);
	FlexGridSizer11->Fit(Panel3);
	FlexGridSizer11->SetSizeHints(Panel3);
	Panel4 = new wxPanel(
		Notebook1, ID_PANEL4, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL,
		_T("ID_PANEL4"));
	FlexGridSizer12 = new wxFlexGridSizer(4, 1, 0, 0);
	FlexGridSizer12->AddGrowableCol(0);
	FlexGridSizer12->AddGrowableRow(1);
	FlexGridSizer12->AddGrowableRow(3);
	StaticText8 = new wxStaticText(
		Panel4, ID_STATICTEXT8,
		_("Motion direction to robot heading angle: "
		  "for selected PTG over time [deg]"),
		wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT8"));
	FlexGridSizer12->Add(StaticText8, 1, wxALL | wxALIGN_LEFT | wxALIGN_TOP, 5);
	m_plotHeadAngIndiv =
		new mpWindow(Panel4, ID_CUSTOM3, wxDefaultPosition, wxDefaultSize, 0);
	FlexGridSizer12->Add(
		m_plotHeadAngIndiv, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP,
		0);
	StaticText9 = new wxStaticText(
		Panel4, ID_STATICTEXT9,
		_("Motion direction to robot heading angle: "
		  "maximum value for each trajectory [deg]"),
		wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT9"));
	FlexGridSizer12->Add(StaticText9, 1, wxALL | wxALIGN_LEFT | wxALIGN_TOP, 5);
	m_plotHeadAngAll =
		new mpWindow(Panel4, ID_CUSTOM4, wxDefaultPosition, wxDefaultSize, 0);
	FlexGridSizer12->Add(
		m_plotHeadAngAll, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	Panel4->SetSizer(FlexGridSizer12);
	FlexGridSizer12->Fit(Panel4);
	FlexGridSizer12->SetSizeHints(Panel4);
	Panel5 = new wxPanel(
		Notebook1, ID_PANEL5, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL,
		_T("ID_PANEL5"));
	FlexGridSizer13 = new wxFlexGridSizer(8, 1, 0, 0);
	FlexGridSizer13->AddGrowableCol(0);
	FlexGridSizer13->AddGrowableRow(1);
	FlexGridSizer13->AddGrowableRow(3);
	FlexGridSizer13->AddGrowableRow(5);
	FlexGridSizer13->AddGrowableRow(7);
	StaticText10 = new wxStaticText(
		Panel5, ID_STATICTEXT10, _("Selected path trajectory: X [m]"),
		wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT10"));
	FlexGridSizer13->Add(
		StaticText10, 1, wxALL | wxALIGN_LEFT | wxALIGN_TOP, 5);
	m_plotPathX =
		new mpWindow(Panel5, ID_CUSTOM5, wxDefaultPosition, wxDefaultSize, 0);
	FlexGridSizer13->Add(
		m_plotPathX, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	StaticText11 = new wxStaticText(
		Panel5, ID_STATICTEXT11, _("Selected path trajectory: Y [m]"),
		wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT11"));
	FlexGridSizer13->Add(
		StaticText11, 1, wxALL | wxALIGN_LEFT | wxALIGN_TOP, 5);
	m_plotPathY =
		new mpWindow(Panel5, ID_CUSTOM6, wxDefaultPosition, wxDefaultSize, 0);
	FlexGridSizer13->Add(
		m_plotPathY, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	StaticText12 = new wxStaticText(
		Panel5, ID_STATICTEXT12, _("Selected path trajectory: Phi [deg]"),
		wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT12"));
	FlexGridSizer13->Add(
		StaticText12, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
	m_plotPathPhi =
		new mpWindow(Panel5, ID_CUSTOM7, wxDefaultPosition, wxDefaultSize, 0);
	FlexGridSizer13->Add(
		m_plotPathPhi, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	StaticText16 = new wxStaticText(
		Panel5, ID_STATICTEXT16,
		_("Selected path trajectory: traversed distance [m]"),
		wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT16"));
	FlexGridSizer13->Add(
		StaticText16, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
	m_plotPathDist =
		new mpWindow(Panel5, ID_CUSTOM11, wxDefaultPosition, wxDefaultSize, 0);
	FlexGridSizer13->Add(
		m_plotPathDist, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	Panel5->SetSizer(FlexGridSizer13);
	FlexGridSizer13->Fit(Panel5);
	FlexGridSizer13->SetSizeHints(Panel5);
	Panel6 = new wxPanel(
		Notebook1, ID_PANEL6, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL,
		_T("ID_PANEL6"));
	FlexGridSizer14 = new wxFlexGridSizer(6, 1, 0, 0);
	FlexGridSizer14->AddGrowableCol(0);
	FlexGridSizer14->AddGrowableRow(1);
	FlexGridSizer14->AddGrowableRow(3);
	FlexGridSizer14->AddGrowableRow(5);
	StaticText13 = new wxStaticText(
		Panel6, ID_STATICTEXT13, _("Selected path trajectory: dX/dt [m/s]"),
		wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT13"));
	FlexGridSizer14->Add(
		StaticText13, 1, wxALL | wxALIGN_LEFT | wxALIGN_TOP, 5);
	m_plotPathXp =
		new mpWindow(Panel6, ID_CUSTOM8, wxDefaultPosition, wxDefaultSize, 0);
	FlexGridSizer14->Add(
		m_plotPathXp, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	StaticText14 = new wxStaticText(
		Panel6, ID_STATICTEXT14, _("Selected path trajectory: dY/dt [m/s]"),
		wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT14"));
	FlexGridSizer14->Add(
		StaticText14, 1, wxALL | wxALIGN_LEFT | wxALIGN_TOP, 5);
	m_plotPathYp =
		new mpWindow(Panel6, ID_CUSTOM9, wxDefaultPosition, wxDefaultSize, 0);
	FlexGridSizer14->Add(
		m_plotPathYp, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	StaticText15 = new wxStaticText(
		Panel6, ID_STATICTEXT15, _("Selected path trajectory: w [deg/s]"),
		wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT15"));
	FlexGridSizer14->Add(
		StaticText15, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
	m_plotPathW =
		new mpWindow(Panel6, ID_CUSTOM10, wxDefaultPosition, wxDefaultSize, 0);
	FlexGridSizer14->Add(
		m_plotPathW, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	Panel6->SetSizer(FlexGridSizer14);
	FlexGridSizer14->Fit(Panel6);
	FlexGridSizer14->SetSizeHints(Panel6);
	Notebook1->AddPage(Panel2, _("TP-Space"), true);
	Notebook1->AddPage(Panel3, _("VelCmds@t=0"), false);
	Notebook1->AddPage(Panel4, _("Head angle"), false);
	Notebook1->AddPage(Panel5, _("Robot path"), false);
	Notebook1->AddPage(Panel6, _("Robot velocity components"), false);
	FlexGridSizer9->Add(
		Notebook1, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	FlexGridSizer1->Add(
		FlexGridSizer9, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	SetSizer(FlexGridSizer1);
	MenuBar1 = new wxMenuBar();
	Menu1 = new wxMenu();
	MenuItem1 = new wxMenuItem(
		Menu1, idMenuQuit, _("Quit\tAlt-F4"), _("Quit the application"),
		wxITEM_NORMAL);
	Menu1->Append(MenuItem1);
	MenuBar1->Append(Menu1, _("&File"));
	Menu2 = new wxMenu();
	MenuItem2 = new wxMenuItem(
		Menu2, idMenuAbout, _("About\tF1"),
		_("Show info about this application"), wxITEM_NORMAL);
	Menu2->Append(MenuItem2);
	MenuBar1->Append(Menu2, _("Help"));
	SetMenuBar(MenuBar1);
	StatusBar1 = new wxStatusBar(this, ID_STATUSBAR1, 0, _T("ID_STATUSBAR1"));
	int __wxStatusBarWidths_1[4] = {-2, -5, -3, -3};
	int __wxStatusBarStyles_1[4] = {wxSB_NORMAL, wxSB_NORMAL, wxSB_NORMAL,
									wxSB_NORMAL};
	StatusBar1->SetFieldsCount(4, __wxStatusBarWidths_1);
	StatusBar1->SetStatusStyles(4, __wxStatusBarStyles_1);
	SetStatusBar(StatusBar1);
	FlexGridSizer1->SetSizeHints(this);
	Center();

	using pcf = ptgConfiguratorframe;

	Bind(wxEVT_CHOICE, &pcf::OncbPTGClassSelect, this, ID_CHOICE1);
	Bind(wxEVT_BUTTON, &pcf::OnbtnReloadParamsClick, this, ID_BUTTON1);
	Bind(
		wxEVT_COMMAND_SPINCTRL_UPDATED, &pcf::OnedPTGIndexChange, this,
		ID_SPINCTRL1);
	Bind(wxEVT_CHECKBOX, &pcf::OncbDrawShapePathClick, this, ID_CHECKBOX1);
	Bind(wxEVT_BUTTON, &pcf::OnButton1Click, this, ID_BUTTON5);
	Bind(wxEVT_CHECKBOX, &pcf::OncbHighlightOnePathClick, this, ID_CHECKBOX3);
	Bind(wxEVT_SLIDER, &pcf::OnslidPathHighlightCmdScroll, this, ID_SLIDER1);
	Bind(
		wxEVT_COMMAND_SPINCTRL_UPDATED, &pcf::OnedIndexHighlightPathChange,
		this, ID_SPINCTRL2);
	Bind(wxEVT_CHECKBOX, &pcf::OncbHighlightOnePathClick, this, ID_CHECKBOX4);
	Bind(wxEVT_CHECKBOX, &pcf::OncbBuildTPObsClick, this, ID_CHECKBOX2);
	Bind(wxEVT_BUTTON, &pcf::OnbtnPlaceObsClick, this, ID_BUTTON3);
	Bind(wxEVT_BUTTON, &pcf::OnbtnRebuildTPObsClick, this, ID_BUTTON2);
	Bind(wxEVT_BUTTON, &pcf::OnbtnPlaceTargetClick, this, ID_BUTTON4);
	Bind(wxEVT_CHECKBOX, &pcf::OnrbShowTPSelectSelect, this, ID_CHECKBOX5);
	Bind(wxEVT_CHECKBOX, &pcf::OnrbShowTPSelectSelect, this, ID_CHECKBOX6);
	Bind(wxEVT_CHECKBOX, &pcf::OnrbShowTPSelectSelect, this, ID_CHECKBOX7);
	Bind(wxEVT_MENU, &pcf::OnQuit, this, idMenuQuit);
	Bind(wxEVT_MENU, &pcf::OnAbout, this, idMenuAbout);
	//*)

	m_plot->Bind(wxEVT_MOTION, &pcf::Onplot3DMouseMove, this);
	m_plot->Bind(wxEVT_LEFT_DOWN, &pcf::Onplot3DMouseClick, this);

	// Redirect all output to control:
	m_myRedirector =
		std::make_unique<CMyRedirector>(edLog, false, 100, true, false, true);

	WX_START_TRY

	// 2D plots:
	prepareRobotPathPlot(m_plotHeadAngAll, &m_graph_head_all, "head_all");
	prepareRobotPathPlot(m_plotHeadAngIndiv, &m_graph_head_indiv, "head_indiv");

	//
	prepareRobotPathPlot(m_plotPathX, &m_graph_path_x, "x");
	prepareRobotPathPlot(m_plotPathY, &m_graph_path_y, "y");
	prepareRobotPathPlot(m_plotPathPhi, &m_graph_path_phi, "phi");
	prepareRobotPathPlot(m_plotPathDist, &m_graph_path_dist, "dist");
	//
	prepareRobotPathPlot(m_plotPathXp, &m_graph_path_vx, "vx");
	prepareRobotPathPlot(m_plotPathYp, &m_graph_path_vy, "vy");
	prepareRobotPathPlot(m_plotPathW, &m_graph_path_omega, "omega");

	// Populate 3D views:
	// ---------------------------
	gl_view_WS = m_plot->getOpenGLSceneRef()->getViewport();
	gl_view_TPSpace = m_plotTPSpace->getOpenGLSceneRef()->getViewport();

	gl_TPSpace_TP_obstacles = mrpt::opengl::CSetOfObjects::Create();
	gl_TPSpace_clearance =
		mrpt::opengl::CMesh::Create(true, -5.0f, 5.0f, -5.0f, 5.0f);
	gl_TPSpace_clearance_interp =
		mrpt::opengl::CMesh::Create(true, -5.0f, 5.0f, -5.0f, 5.0f);
	gl_TPSpace_clearance_interp->setVisibility(false);

	gl_view_TPSpace->insert(gl_TPSpace_TP_obstacles);
	gl_view_TPSpace->insert(gl_TPSpace_clearance);
	gl_view_TPSpace->insert(gl_TPSpace_clearance_interp);

	m_plot->getOpenGLSceneRef()->getViewport()->addTextMessage(
		0.01, 5, "Workspace", 1);
	m_plotTPSpace->getOpenGLSceneRef()->getViewport()->addTextMessage(
		0.01, 5, "TP-Space", 2);

	gl_robot_ptg_prediction = mrpt::opengl::CSetOfLines::Create();
	gl_robot_ptg_prediction->setName("ptg_prediction");
	gl_robot_ptg_prediction->setLineWidth(1.0);
	gl_robot_ptg_prediction->setColor_u8(
		mrpt::img::TColor(0x00, 0x00, 0xff, 0x90));
	gl_view_WS->insert(gl_robot_ptg_prediction);

	gl_robot_ptg_prediction_highlight = mrpt::opengl::CSetOfLines::Create();
	gl_robot_ptg_prediction_highlight->setName("ptg_prediction_highlight");
	gl_robot_ptg_prediction_highlight->setLineWidth(3.0);
	gl_robot_ptg_prediction_highlight->setColor_u8(
		mrpt::img::TColor(0xff, 0x00, 0x00, 0xff));
	gl_view_WS->insert(gl_robot_ptg_prediction_highlight);

	gl_WS_obs = mrpt::opengl::CPointCloud::Create();
	gl_WS_obs->setPointSize(7.0);
	gl_WS_obs->setColor_u8(0, 0, 0);
	gl_view_WS->insert(gl_WS_obs);

	gl_WS_target = mrpt::opengl::CPointCloud::Create();
	gl_WS_target->setPointSize(7.0);
	gl_WS_target->setColor_u8(0xff, 0, 0);
	gl_WS_target->insertPoint(0, 0, 0);
	gl_WS_target->setName("WS-target");
	gl_WS_target->enableShowName(true);
	gl_view_WS->insert(gl_WS_target);

	gl_WS_target_reprojected = mrpt::opengl::CPointCloud::Create();
	gl_WS_target_reprojected->setPointSize(5.0);
	gl_WS_target_reprojected->setColor_u8(0xff, 0xff, 0x00, 0xe0);
	gl_WS_target_reprojected->insertPoint(0, 0, 0);
	gl_WS_target_reprojected->setName("WS-target-reproj");
	gl_WS_target_reprojected->enableShowName(true);
	gl_view_WS->insert(gl_WS_target_reprojected);

	gl_TP_target = mrpt::opengl::CPointCloud::Create();
	gl_TP_target->setPointSize(7.0);
	gl_TP_target->setColor_u8(0xff, 0, 0);
	gl_TP_target->insertPoint(0, 0, 0);
	gl_TP_target->setName("TP-target");
	gl_TP_target->enableShowName(true);
	gl_view_TPSpace->insert(gl_TP_target);

	{
		gl_axis_WS = mrpt::opengl::CAxis::Create(
			-10.0, -10.0, 0, 10.0, 10.0, 0.0, 1.0, 2.0);
		gl_axis_WS->setTextScale(0.20f);
		gl_axis_WS->enableTickMarks(true, true, true);
		gl_axis_WS->setColor_u8(mrpt::img::TColor(30, 30, 30, 50));
		gl_axis_WS->setTextLabelOrientation(0, 0, 0, 0);
		gl_axis_WS->setTextLabelOrientation(1, 0, 0, 0);

		gl_view_WS->insert(gl_axis_WS);
	}
	{
		gl_axis_TPS = mrpt::opengl::CAxis::Create(
			-1.0, -1.0, 0, 1.0, 1.0, 0.0, 0.25, 2.0);
		gl_axis_TPS->setTextScale(0.04f);
		gl_axis_TPS->enableTickMarks(true, true, false);
		gl_axis_TPS->setColor_u8(mrpt::img::TColor(30, 30, 30, 50));
		gl_axis_TPS->setTextLabelOrientation(0, 0, 0, 0);
		gl_axis_TPS->setTextLabelOrientation(1, 0, 0, 0);
		gl_view_TPSpace->insert(gl_axis_TPS);
	}

	gl_tp_obstacles = mrpt::opengl::CSetOfLines::Create();
	gl_tp_obstacles->setName("tp_obstacles");
	gl_tp_obstacles->setLineWidth(2.0f);
	gl_tp_obstacles->setColor_u8(mrpt::img::TColor(0x00, 0x00, 0x00, 0xff));

	gl_TPSpace_TP_obstacles->insert(gl_tp_obstacles);

	// Set camera:
	m_plot->setCameraPointing(0.0f, 0.0f, 0.0f);
	m_plot->setZoomDistance(10.0f);
	m_plot->setElevationDegrees(90.0f);
	m_plot->setAzimuthDegrees(-90.0f);
	m_plot->setCameraProjective(false);

#if 0
	// Fixed camera:
	gl_view_TPSpace_cam = mrpt::opengl::CCamera::Create();
	gl_view_TPSpace->insert ( gl_view_TPSpace_cam );
	gl_view_TPSpace_cam->setAzimuthDegrees( -90 );
	gl_view_TPSpace_cam->setElevationDegrees(90);
	gl_view_TPSpace_cam->setProjectiveModel( false );
	gl_view_TPSpace_cam->setZoomDistance(2.1f);
#else
	// User can rotate view:
	m_plotTPSpace->setCameraPointing(0.0f, 0.0f, 0.0f);
	m_plotTPSpace->setZoomDistance(2.1f);
	m_plotTPSpace->setElevationDegrees(90.0f);
	m_plotTPSpace->setAzimuthDegrees(-90.0f);
	m_plotTPSpace->setCameraProjective(false);
#endif

	// Populate list of existing PTGs:
	{
		// mrpt::nav::registerAllNavigationClasses();
		const std::vector<const mrpt::rtti::TRuntimeClassId*>& lstClasses =
			mrpt::rtti::getAllRegisteredClasses();
		for (auto lstClasse : lstClasses)
		{
			if (!lstClasse->derivedFrom("CParameterizedTrajectoryGenerator") ||
				!mrpt::system::os::_strcmpi(
					lstClasse->className, "CParameterizedTrajectoryGenerator"))
				continue;
			cbPTGClass->AppendString(lstClasse->className);
		}
		if (cbPTGClass->GetCount() > 0) cbPTGClass->SetSelection(0);
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
}

void ptgConfiguratorframe::prepareRobotPathPlot(
	mpWindow* plot, mpFXYVector** graph, const std::string& name)
{
	plot->AddLayer(new mpScaleX(wxT("t [s]"), mpALIGN_CENTER, false /*grid*/));
	plot->AddLayer(new mpScaleY(wxT("y"), mpALIGN_CENTER, false /*grid*/));

	*graph = new mpFXYVector(name.c_str());
	(*graph)->SetPen(wxPen(wxColour(0, 0, 255), 5));
	(*graph)->SetContinuity(false);
	plot->AddLayer(*graph);
}

void ptgConfiguratorframe::OnAbout(wxCommandEvent&)
{
	mrpt::gui::show_mrpt_about_box_wxWidgets(this, "ptg-configurator");
}
void ptgConfiguratorframe::OnQuit(wxCommandEvent&) { Close(); }
void ptgConfiguratorframe::OnbtnReloadParamsClick(wxCommandEvent& event)
{
	WX_START_TRY;
	if (!ptg) return;

	ptg->deinitialize();

	const std::string sKeyPrefix =
		mrpt::format("PTG%d_", (int)edPTGIndex->GetValue());
	const std::string sSection = "PTG_PARAMS";

	mrpt::config::CConfigFileMemory cfg;
	mrpt::config::CConfigFilePrefixer cfp;
	cfp.bind(cfg);
	cfp.setPrefixes("", sKeyPrefix);

	cfg.setContent(std::string(edCfg->GetValue().mb_str()));

	ptg->loadFromConfigFile(cfp, sSection);

	ptg->initialize();

	// PTG: Realize updated dynamical state (not from a real robot in this app,
	// but already loaded from a config file in this app for debugging):
	ptg->updateNavDynamicState(ptg->getCurrentNavDynamicState(), true);

	// one-time GUI init for each PTG settings:
	edIndexHighlightPath->SetRange(0, ptg->getPathCount() - 1);
	slidPathHighlight->SetRange(0, ptg->getPathCount() - 1);

	// first time full GUI refresh:
	rebuild3Dview();

	WX_END_TRY;
}

void ptgConfiguratorframe::OncbPTGClassSelect(wxCommandEvent& event)
{
	WX_START_TRY;

	const int sel = cbPTGClass->GetSelection();
	if (sel < 0) return;

	const std::string sSelPTG =
		std::string(cbPTGClass->GetString(sel).mb_str());

	ptg.reset();

	// Factory:
	const mrpt::rtti::TRuntimeClassId* classId =
		mrpt::rtti::findRegisteredClass(sSelPTG);
	if (!classId)
	{
		THROW_EXCEPTION_FMT(
			"[CreatePTG] No PTG named `%s` is registered!", sSelPTG.c_str());
	}

	ptg = mrpt::ptr_cast<mrpt::nav::CParameterizedTrajectoryGenerator>::from(
		classId->createObject());
	if (!ptg)
	{
		THROW_EXCEPTION_FMT(
			"[CreatePTG] Object of type `%s` seems not to be a PTG!",
			sSelPTG.c_str());
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
	static mrpt::system::CTicTac timer;
	const double refDist = ptg ? ptg->getRefDistance() : 10.0;
	ASSERT_(refDist > 0);

	// Limits:
	gl_axis_WS->setAxisLimits(-refDist, -refDist, .0f, refDist, refDist, .0f);

	double tx = 10.0, ty = .0;  // Target in WS
	{
		bool ok_x = edTargetX->GetValue().ToDouble(&tx);
		bool ok_y = edTargetY->GetValue().ToDouble(&ty);
		if (ok_x && ok_y)
		{
			gl_WS_target->setLocation(tx, ty, 0);
		}
	}

	if (ptg && ptg->isInitialized())
	{
		// Update PTG dynamic state with new target:
		{
			mrpt::nav::CParameterizedTrajectoryGenerator::TNavDynamicState
				navdyn = ptg->getCurrentNavDynamicState();
			navdyn.relTarget.x = tx;
			navdyn.relTarget.y = ty;
			edRelSpeedAtTarget->GetValue().ToDouble(&navdyn.targetRelSpeed);
			ptg->updateNavDynamicState(navdyn);
		}

		// TP-Obstacles:
		std::vector<double> TP_Obstacles;
		ptg->initTPObstacles(TP_Obstacles);

		gl_WS_obs->clear();
		{
			double ox, oy;
			bool ok_x = edObsX->GetValue().ToDouble(&ox);
			bool ok_y = edObsY->GetValue().ToDouble(&oy);
			if (ok_x && ok_y)
			{
				if (cbBuildTPObs->IsChecked())
				{
					gl_WS_obs->insertPoint(ox, oy, 0);
					timer.Tic();
					ptg->updateTPObstacle(ox, oy, TP_Obstacles);
					const double t = timer.Tac();
					StatusBar1->SetStatusText(
						wxString::Format(
							wxT("TP-Obstacle build time: %ss"),
							mrpt::system::unitsFormat(t, 2).c_str()),
						2);
				}

				// Clearance diagram:
				mrpt::nav::ClearanceDiagram cd;
				ptg->initClearanceDiagram(cd);
				double tim_build_cd = .0;
				{
					timer.Tic();
					ptg->updateClearance(ox, oy, cd);
					ptg->updateClearancePost(cd, TP_Obstacles);
					tim_build_cd = timer.Tac();
				}

				timer.Tic();
				cd.renderAs3DObject(
					*gl_TPSpace_clearance, -1.0, 1.0, -1.0, 1.0, 0.05,
					false /*interpolate*/);
				cd.renderAs3DObject(
					*gl_TPSpace_clearance_interp, -1.0, 1.0, -1.0, 1.0, 0.05,
					true /*interpolate*/);
				const double tim_render_cd = timer.Tac();

				StatusBar1->SetStatusText(
					wxString::Format(
						wxT("Clearance-diagram time: build=%ss render=%ss"),
						mrpt::system::unitsFormat(tim_build_cd, 2).c_str(),
						mrpt::system::unitsFormat(tim_render_cd, 2).c_str()),
					3);
			}
		}

		try
		{
			const double ptg_alpha =
				ptg->index2alpha(edIndexHighlightPath->GetValue());
			StaticText12->SetLabel(wxString::Format(
				_("Selected path trajectory: Phi [deg]. PTG alpha=%.03f "
				  "[deg]"),
				mrpt::RAD2DEG(ptg_alpha)));
		}
		catch (...)
		{
		}

		const size_t nPTGPaths = ptg->getPathCount();
		// All paths:
		gl_robot_ptg_prediction->clear();
		gl_robot_ptg_prediction_highlight->clear();
		for (size_t k = 0; k < nPTGPaths; k++)
		{
			if (cbShowOnlySelectedTraj->IsChecked() &&
				k != size_t(edIndexHighlightPath->GetValue()))
				continue;

			const double max_dist = TP_Obstacles[k];

			mrpt::opengl::CSetOfLines& sol =
				cbHighlightOnePath->IsChecked() &&
						k == size_t(edIndexHighlightPath->GetValue())
					? *gl_robot_ptg_prediction_highlight
					: *gl_robot_ptg_prediction;

			ptg->renderPathAsSimpleLine(k, sol, 0.10f, max_dist);

			// Overlay a sequence of robot shapes:
			if (cbDrawShapePath->IsChecked())
			{
				double min_shape_dists = 1.0;
				edMinDistBtwShapes->GetValue().ToDouble(&min_shape_dists);
				bool done = false;
				for (double d = max_dist; !done; d -= min_shape_dists)
				{
					if (d < 0)
					{
						d = 0;
						done = true;
					}
					uint32_t step;
					if (!ptg->getPathStepForDist(k, d, step)) continue;
					mrpt::math::TPose2D p;
					ptg->getPathPose(k, step, p);
					ptg->add_robotShape_to_setOfLines(
						sol, mrpt::poses::CPose2D(p));
				}
			}
		}

		// 2D angle to robot head plots:
		std::vector<double> robotHeadAng_x, robotHeadAng_y,
			robotHeadAngAll_x(nPTGPaths), robotHeadAngAll_y(nPTGPaths);
		std::vector<double> robotPath_x, robotPath_y, robotPath_phi,
			robotPath_dist;
		std::vector<double> robotPath_vx, robotPath_vy, robotPath_w;
		const double dt = ptg->getPathStepDuration();

		for (size_t k = 0; k < nPTGPaths; k++)
		{
			robotHeadAngAll_x[k] = k;
			bool is_selected_path =
				(k == size_t(edIndexHighlightPath->GetValue()));

			size_t nSteps = ptg->getPathStepCount(k);

			if (is_selected_path)
			{
				robotHeadAng_x.resize(nSteps);
				robotHeadAng_y.resize(nSteps);
				robotPath_x.resize(nSteps);
				robotPath_y.resize(nSteps);
				robotPath_phi.resize(nSteps);
				robotPath_dist.resize(nSteps);
				robotPath_vx.resize(nSteps);
				robotPath_vy.resize(nSteps);
				robotPath_w.resize(nSteps);
			}

			double maxRobotHeadErr = .0;
			for (size_t j = 0; j < nSteps; j++)
			{
				const mrpt::math::TPose2D curPose = ptg->getPathPose(k, j);
				const mrpt::math::TTwist2D curVel = ptg->getPathTwist(k, j);

				// Head calc:
				const double head2dir =
					(curVel.vy != 0 || curVel.vx != 0)
						? mrpt::math::angDistance(
							  ::atan2(curVel.vy, curVel.vx), curPose.phi)
						: .0;

				if (is_selected_path)
				{
					robotHeadAng_x[j] = j * dt;
					robotHeadAng_y[j] = mrpt::RAD2DEG(head2dir);
					robotPath_x[j] = curPose.x;
					robotPath_y[j] = curPose.y;
					robotPath_phi[j] = mrpt::RAD2DEG(curPose.phi);
					robotPath_dist[j] = ptg->getPathDist(k, j);

					robotPath_vx[j] = curVel.vx;
					robotPath_vy[j] = curVel.vy;
					robotPath_w[j] = mrpt::RAD2DEG(curVel.omega);
				}

				mrpt::keep_max(maxRobotHeadErr, std::abs(head2dir));
			}

			robotHeadAngAll_y[k] = mrpt::RAD2DEG(maxRobotHeadErr);
		}
		m_graph_head_all->SetData(robotHeadAngAll_x, robotHeadAngAll_y);
		m_graph_head_indiv->SetData(robotHeadAng_x, robotHeadAng_y);

		m_graph_path_x->SetData(robotHeadAng_x, robotPath_x);
		m_graph_path_y->SetData(robotHeadAng_x, robotPath_y);
		m_graph_path_phi->SetData(robotHeadAng_x, robotPath_phi);
		m_graph_path_dist->SetData(robotHeadAng_x, robotPath_dist);

		m_graph_path_vx->SetData(robotHeadAng_x, robotPath_vx);
		m_graph_path_vy->SetData(robotHeadAng_x, robotPath_vy);
		m_graph_path_omega->SetData(robotHeadAng_x, robotPath_w);

		// TP-Obstacles:
		gl_tp_obstacles->clear();
		const size_t nObs = TP_Obstacles.size();
		if (nObs > 1)
		{
			for (size_t i = 0; i <= nObs; i++)
			{
				const double d0 = TP_Obstacles[i % nObs] / refDist;
				const double a0 =
					M_PI * (-1.0 + 2.0 * ((i % nObs) + 0.5) / nObs);
				const double d1 = TP_Obstacles[(i + 1) % nObs] / refDist;
				const double a1 =
					M_PI * (-1.0 + 2.0 * (((i + 1) % nObs) + 0.5) / nObs);
				gl_tp_obstacles->appendLine(
					d0 * cos(a0), d0 * sin(a0), 0.0, d1 * cos(a1), d1 * sin(a1),
					0.0);
			}
		}
	}  // if ptg init

	// Target:
	{
		if (ptg && ptg->isInitialized())
		{
			int k = 0;
			double norm_d = 0;
			bool is_exact = ptg->inverseMap_WS2TP(tx, ty, k, norm_d);
			const double dir = ptg->index2alpha(k);
			if (is_exact)
			{
				gl_TP_target->setLocation(
					cos(dir) * norm_d, sin(dir) * norm_d, .0);
				gl_TP_target->setName("TP-target");
			}
			else
			{
				gl_TP_target->setName("TP-target (NOT EXACT)");
			}
			StatusBar1->SetStatusText(
				wxString::Format(
					wxT("TP-Target: k=%i (alpha=%.03f deg) norm_d=%.03f "
						"is_exact:%s"),
					k, dir * 180 / M_PI, norm_d, (is_exact ? "yes" : "NO")),
				1);

			// Sanity check: reproject TP_target back to WS:
			uint32_t check_step;
			if (ptg->getPathStepForDist(
					k, norm_d * ptg->getRefDistance(), check_step))
			{
				mrpt::math::TPose2D p;
				ptg->getPathPose(k, check_step, p);
				gl_WS_target_reprojected->setLocation(p.x, p.y, 0);
				gl_WS_target_reprojected->setName("WS-Target-reproj");
			}
			else
			{
				gl_WS_target_reprojected->setName(
					"WS-Target-reproj (not exact!)");
			}
		}
	}

	m_plotHeadAngAll->Fit();
	m_plotHeadAngIndiv->Fit();
	m_plotPathX->Fit();
	m_plotPathY->Fit();
	m_plotPathPhi->Fit();
	m_plotPathXp->Fit();
	m_plotPathYp->Fit();
	m_plotPathW->Fit();
	m_plotPathDist->Fit();

	m_plotHeadAngAll->UpdateAll();
	m_plotHeadAngIndiv->UpdateAll();
	m_plotPathX->UpdateAll();
	m_plotPathY->UpdateAll();
	m_plotPathPhi->UpdateAll();
	m_plotPathXp->UpdateAll();
	m_plotPathYp->UpdateAll();
	m_plotPathW->UpdateAll();
	m_plotPathDist->UpdateAll();

	m_plot->Refresh();
	m_plotTPSpace->Refresh();
	WX_END_TRY;
}

void ptgConfiguratorframe::loadPlugin()
{
	wxFileDialog openFileDialog(
		this, _("Open library"), wxT(""), wxT(""),
		wxT("so files (*.so)|*.so|so files (*.so.*)|*.so.*|*.dll"),
		wxFD_OPEN | wxFD_FILE_MUST_EXIST);

	if (openFileDialog.ShowModal() == wxID_CANCEL) return;

	const std::string sLib = std::string(openFileDialog.GetPath().mb_str());
	mrpt::system::loadPluginModule(sLib);

	// Populate list of existing PTGs:
	{
		cbPTGClass->Clear();
		const std::vector<const mrpt::rtti::TRuntimeClassId*>& lstClasses =
			mrpt::rtti::getAllRegisteredClasses();
		for (auto lstClasse : lstClasses)
		{
			if (!lstClasse->derivedFrom("CParameterizedTrajectoryGenerator") ||
				!mrpt::system::os::_strcmpi(
					lstClasse->className, "CParameterizedTrajectoryGenerator"))
				continue;
			cbPTGClass->AppendString(lstClasse->className);
		}
		if (cbPTGClass->GetCount() > 0) cbPTGClass->SetSelection(0);
	}
	{
		wxCommandEvent e;
		OncbPTGClassSelect(e);
	}
}

void ptgConfiguratorframe::OnedPTGIndexChange(wxSpinEvent& event)
{
	dumpPTGcfgToTextBox();
}

void ptgConfiguratorframe::dumpPTGcfgToTextBox()
{
	if (!ptg) return;

	// Wrapper to transparently add prefixes to all config keys:
	const std::string sKeyPrefix =
		mrpt::format("PTG%d_", (int)edPTGIndex->GetValue());
	const std::string sSection = "PTG_PARAMS";

	mrpt::config::CConfigFileMemory cfg;
	mrpt::config::CConfigFilePrefixer cfp;
	cfp.bind(cfg);
	cfp.setPrefixes("", sKeyPrefix);

	const int WN = 25, WV = 30;
	cfp.write(
		sSection, "Type", ptg->GetRuntimeClass()->className, WN, WV,
		"PTG C++ class name");

	// Dump default params:
	ptg->saveToConfigFile(cfp, sSection);

	edCfg->SetValue(cfg.getContent().c_str());
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
	event.GetPosition(&X, &Y);

	// Intersection of 3D ray with ground plane ====================
	TLine3D ray;
	m_plot->getOpenGLSceneRef()->getViewport("main")->get3DRayForPixelCoord(
		X, Y, ray);
	// Create a 3D plane, e.g. Z=0
	const TPlane ground_plane(
		TPoint3D(0, 0, 0), TPoint3D(1, 0, 0), TPoint3D(0, 1, 0));
	// Intersection of the line with the plane:
	TObject3D inters;
	intersect(ray, ground_plane, inters);
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
				edObsX->SetValue(
					(mrpt::format("%.03f", m_curCursorPos.x).c_str()));
				edObsY->SetValue(
					(mrpt::format("%.03f", m_curCursorPos.y).c_str()));
				rebuild3Dview();
			}
			break;
			case cpsPickTarget:
			{
				edTargetX->SetValue(
					(mrpt::format("%.03f", m_curCursorPos.x).c_str()));
				edTargetY->SetValue(
					(mrpt::format("%.03f", m_curCursorPos.y).c_str()));
				rebuild3Dview();
			}
			break;
			default:
				break;
		};
		StatusBar1->SetStatusText(
			wxString::Format(
				wxT("Cursor: X=%.03f Y=%.04f"), m_curCursorPos.x,
				m_curCursorPos.y),
			0);
	}

	// Do normal process in that class:
	m_plot->OnMouseMove(event);
}

void ptgConfiguratorframe::Onplot3DMouseClick(wxMouseEvent& event)
{
	m_plot->SetCursor(*wxSTANDARD_CURSOR);  // End of cross cursor
	m_cursorPickState = cpsNone;  // end of mode

	// Do normal process in that class:
	m_plot->OnMouseDown(event);
}

void ptgConfiguratorframe::OnbtnPlaceObsClick(wxCommandEvent& event)
{
	m_plot->SetCursor(*wxCROSS_CURSOR);
	m_cursorPickState = cpsPickObstacle;
}

void ptgConfiguratorframe::OnbtnPlaceTargetClick(wxCommandEvent& event)
{
	m_plot->SetCursor(*wxCROSS_CURSOR);
	m_cursorPickState = cpsPickTarget;
}

void ptgConfiguratorframe::OnslidPathHighlightCmdScroll(wxCommandEvent&)
{
	edIndexHighlightPath->SetValue(slidPathHighlight->GetValue());
	wxSpinEvent dm;
	OnedIndexHighlightPathChange(dm);
}

void ptgConfiguratorframe::OncbHighlightOnePathClick(wxCommandEvent& event)
{
	// slidPathHighlight->Enable( cbHighlightOnePath->IsChecked() );
	// edIndexHighlightPath->Enable( cbHighlightOnePath->IsChecked() );
	rebuild3Dview();
}

void ptgConfiguratorframe::OnedIndexHighlightPathChange(wxSpinEvent& event)
{
	rebuild3Dview();
}

void ptgConfiguratorframe::OnButton1Click(wxCommandEvent& event)
{
	loadPlugin();
}

void ptgConfiguratorframe::OnrbShowTPSelectSelect(wxCommandEvent& event)
{
	WX_START_TRY;

	gl_TPSpace_TP_obstacles->setVisibility(cbShowTPObs->IsChecked());
	gl_TPSpace_clearance->setVisibility(
		cbShowClearance->IsChecked() && !cbClearanceInterp->IsChecked());
	gl_TPSpace_clearance_interp->setVisibility(
		cbShowClearance->IsChecked() && cbClearanceInterp->IsChecked());

	m_plotTPSpace->Refresh();
	WX_END_TRY;
}
