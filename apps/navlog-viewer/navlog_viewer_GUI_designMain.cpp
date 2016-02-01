/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
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
#include <mrpt/math/utils.h>
#include <mrpt/utils/printf_vector.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/stock_objects.h>

extern std::string global_fileToOpen;

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
const long navlog_viewer_GUI_designDialog::ID_PANEL2 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT2 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT3 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT4 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT5 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT6 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT7 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_CHECKBOX1 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_STATICTEXT8 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_TEXTCTRL2 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_PANEL3 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_BUTTON6 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_PANEL1 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_TIMER1 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_TIMER2 = wxNewId();
const long navlog_viewer_GUI_designDialog::ID_MENUITEM1 = wxNewId();
//*)

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
    wxFlexGridSizer* FlexGridSizer5;
    wxFlexGridSizer* FlexGridSizer2;
    wxFlexGridSizer* FlexGridSizer7;
    wxFlexGridSizer* FlexGridSizer6;
    wxStaticBoxSizer* StaticBoxSizer1;
    wxFlexGridSizer* FlexGridSizer1;

    Create(parent, wxID_ANY, _("Navigation log viewer - Part of the MRPT project"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE, _T("wxID_ANY"));
    Move(wxPoint(20,20));
    FlexGridSizer1 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer1->AddGrowableCol(0);
    FlexGridSizer1->AddGrowableRow(0);
    Panel_AUX = new wxPanel(this, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    FlexGridSizer2 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer2->AddGrowableCol(0);
    FlexGridSizer3 = new wxFlexGridSizer(1, 5, 0, 0);
    FlexGridSizer3->AddGrowableCol(2);
    btnLoad = new wxCustomButton(Panel_AUX,ID_BUTTON1,_("Load log..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FOLDER")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxSize(70,55),wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON1"));
    btnLoad->SetBitmapDisabled(btnLoad->CreateBitmapDisabled(btnLoad->GetBitmapLabel()));
    btnLoad->SetBitmapMargin(wxSize(2,4));
    FlexGridSizer3->Add(btnLoad, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    StaticText1 = new wxStaticText(Panel_AUX, ID_STATICTEXT1, _("Loaded file:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    FlexGridSizer3->Add(StaticText1, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    edLogFile = new wxTextCtrl(Panel_AUX, ID_TEXTCTRL1, wxEmptyString, wxDefaultPosition, wxSize(213,27), wxTE_READONLY, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    FlexGridSizer3->Add(edLogFile, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    btnHelp = new wxCustomButton(Panel_AUX,ID_BUTTON2,_("About..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_HELP")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxSize(70,55),wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON2"));
    btnHelp->SetBitmapDisabled(btnHelp->CreateBitmapDisabled(btnHelp->GetBitmapLabel()));
    btnHelp->SetBitmapMargin(wxSize(20,4));
    FlexGridSizer3->Add(btnHelp, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    btnQuit = new wxCustomButton(Panel_AUX,ID_BUTTON3,_("Exit"),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_QUIT")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxSize(70,55),wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON3"));
    btnQuit->SetBitmapDisabled(btnQuit->CreateBitmapDisabled(btnQuit->GetBitmapLabel()));
    btnQuit->SetBitmapMargin(wxSize(20,4));
    FlexGridSizer3->Add(btnQuit, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
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
    FlexGridSizer7 = new wxFlexGridSizer(0, 3, 0, 0);
    btnPlay = new wxButton(Panel1, ID_BUTTON4, _("Play"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON4"));
    FlexGridSizer7->Add(btnPlay, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnStop = new wxButton(Panel1, ID_BUTTON5, _("Stop"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON5"));
    btnStop->Disable();
    FlexGridSizer7->Add(btnStop, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
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
    FlexGridSizer9->Add(txtLogDuration, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText4 = new wxStaticText(Panel3, ID_STATICTEXT6, _("Selected PTG:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT6"));
    FlexGridSizer9->Add(StaticText4, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    txtSelectedPTG = new wxStaticText(Panel3, ID_STATICTEXT7, _("-"), wxDefaultPosition, wxSize(80,-1), 0, _T("ID_STATICTEXT7"));
    FlexGridSizer9->Add(txtSelectedPTG, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbDrawShapePath = new wxCheckBox(Panel3, ID_CHECKBOX1, _("Draw shape along path"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
    cbDrawShapePath->SetValue(true);
    FlexGridSizer9->Add(cbDrawShapePath, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer9->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText5 = new wxStaticText(Panel3, ID_STATICTEXT8, _("Shape draw min. dist:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT8"));
    FlexGridSizer9->Add(StaticText5, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edShapeMinDist = new wxTextCtrl(Panel3, ID_TEXTCTRL2, _("1.0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL2"));
    FlexGridSizer9->Add(edShapeMinDist, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel3->SetSizer(FlexGridSizer9);
    FlexGridSizer9->Fit(Panel3);
    FlexGridSizer9->SetSizeHints(Panel3);
    StaticBoxSizer2->Add(Panel3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    flexGridRightHand->Add(StaticBoxSizer2, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnMoreOps = new wxButton(Panel_AUX, ID_BUTTON6, _("More..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON6"));
    flexGridRightHand->Add(btnMoreOps, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer4->Add(flexGridRightHand, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer2->Add(FlexGridSizer4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel_AUX->SetSizer(FlexGridSizer2);
    FlexGridSizer2->Fit(Panel_AUX);
    FlexGridSizer2->SetSizeHints(Panel_AUX);
    FlexGridSizer1->Add(Panel_AUX, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    SetSizer(FlexGridSizer1);
    timPlay.SetOwner(this, ID_TIMER1);
    timAutoload.SetOwner(this, ID_TIMER2);
    timAutoload.Start(900, false);
    mnuMatlabPlots = new wxMenuItem((&mnuMoreOps), ID_MENUITEM1, _("Export map plot to MATLAB..."), wxEmptyString, wxITEM_NORMAL);
    mnuMoreOps.Append(mnuMatlabPlots);
    FlexGridSizer1->Fit(this);
    FlexGridSizer1->SetSizeHints(this);

    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnbtnLoadClick);
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnbtnHelpClick);
    Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnbtnQuitClick);
    Connect(ID_SLIDER1,wxEVT_SCROLL_THUMBTRACK,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnslidLogCmdScroll);
    Connect(ID_SLIDER1,wxEVT_SCROLL_CHANGED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnslidLogCmdScroll);
    Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnbtnPlayClick);
    Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnbtnStopClick);
    Connect(ID_BUTTON6,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnbtnMoreOpsClick);
    Connect(ID_TIMER1,wxEVT_TIMER,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OntimPlayTrigger);
    Connect(ID_TIMER2,wxEVT_TIMER,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OntimAutoloadTrigger);
    Connect(ID_MENUITEM1,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&navlog_viewer_GUI_designDialog::OnmnuMatlabPlotsSelected);
    //*)

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
	this->m_mywins.clear();
	this->m_mywins3D.clear();
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

	this->edLogFile->SetValue(_U(filName.c_str()));

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
				if (logptr->timestamp!=INVALID_TIMESTAMP)
					m_log_last_tim = logptr->timestamp;

				if (!logptr->infoPerPTG.empty())
				{
					size_t nPTGs = logptr->infoPerPTG.size();
					m_logdata_ptg_paths.resize(nPTGs);
					for (size_t i=0;i<nPTGs;i++)
						if (logptr->infoPerPTG[i].ptg_trajectory)
							m_logdata_ptg_paths[i] = logptr->infoPerPTG[i].ptg_trajectory;
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

	timPlay.Start(20, true);
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
		timPlay.Start(20, true);
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

		CDisplayWindowPlotsPtr &win = m_mywins["VW"];
		if (!win)  {
			win= CDisplayWindowPlots::Create("Commanded v (red)/w (blue)",400,200);
			win->setPos(900,20);
			win->axis(-5,5,-5,5, true);
		}

		std::vector<double> vs(N),ws(N);
		for (size_t i=0;i<N;i++)
		{
			CLogFileRecordPtr logptr = CLogFileRecordPtr(m_logdata[i]);
			const CLogFileRecord &log = *logptr;
			vs[i] = log.v;
			ws[i] = log.w;
		}
		win->clf();
		win->plot(vs,"r-","v1"); win->plot(vs,"r2.","v2");
		win->plot(ws,"b-","w1"); win->plot(ws,"b2.","w2");
		win->axis_fit();
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

// Aux function
void add_robotShape_to_setOfLines(
	const CVectorFloat &shap_x_,
	const CVectorFloat &shap_y_, 
	mrpt::opengl::CSetOfLines &gl_shape, 
	const mrpt::poses::CPose2D &origin = mrpt::poses::CPose2D () )
{
	const int N = shap_x_.size();
	if (N>=2 && N==shap_y_.size() )
	{
		// Transform coordinates:
		CVectorDouble shap_x(N), shap_y(N),shap_z(N);
		for (int i=0;i<N;i++) {
			origin.composePoint(
				shap_x_[i], shap_y_[i], 0,
				shap_x[i],  shap_y[i],  shap_z[i]);
		}

		gl_shape.appendLine( shap_x[0], shap_y[0], shap_z[0], shap_x[1],shap_y[1],shap_z[1] );
		for (int i=0;i<=shap_x.size();i++) {
			const int idx = i % shap_x.size();
			gl_shape.appendLineStrip( shap_x[idx],shap_y[idx], shap_z[idx]);
		}
	}
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
			win1= CDisplayWindow3D::Create("Sensed obstacles",300,270);
			win1->setPos(600,20);
			win1->setCameraAzimuthDeg(-90);
			win1->setCameraElevationDeg(90);
			
			{
				mrpt::opengl::COpenGLScenePtr &scene = win1->get3DSceneAndLock();

				// XY ground plane:
				mrpt::opengl::CGridPlaneXYPtr gl_grid = mrpt::opengl::CGridPlaneXY::Create(-20,20, -20,20, 0, 1, 0.75f);
				gl_grid->setColor_u8( mrpt::utils::TColor(0xa0a0a0, 0x90) );
				scene->insert( gl_grid );

				// XYZ corner at origin:
				scene->insert( mrpt::opengl::stock_objects::CornerXYZSimple(1.0, 2.0) );

				win1->unlockAccess3DScene();
			}
		}

		// Update 3D view:
		{
			mrpt::opengl::COpenGLScenePtr &scene = win1->get3DSceneAndLock();

			const CVectorFloat shap_x = log.robotShape_x, shap_y = log.robotShape_y;

			{
				// Obstacles:  Was: win1->plot(log.WS_Obstacles.getPointsBufferRef_x(),log.WS_Obstacles.getPointsBufferRef_y(),"b.4");
				mrpt::opengl::CPointCloudPtr gl_obs;
				mrpt::opengl::CRenderizablePtr gl_obs_r = scene->getByName("obs");  // Get or create if new
				if (!gl_obs_r) {
					gl_obs = mrpt::opengl::CPointCloud::Create();
					gl_obs->setName("obs");
					gl_obs->setPointSize(3.0);
					gl_obs->setColor_u8( mrpt::utils::TColor(0x00,0x00,0xff));
					scene->insert(gl_obs);
				} else {
					gl_obs = mrpt::opengl::CPointCloudPtr(gl_obs_r);
				}
				gl_obs->loadFromPointsMap(&log.WS_Obstacles);
			}

			{
				// Selected PTG path:
				mrpt::opengl::CSetOfLinesPtr   gl_path;
				mrpt::opengl::CRenderizablePtr gl_path_r = scene->getByName("path");  // Get or create if new
				if (!gl_path_r) {
					gl_path = mrpt::opengl::CSetOfLines::Create();
					gl_path->setName("path");
					gl_path->setLineWidth(2.0);
					gl_path->setColor_u8( mrpt::utils::TColor(0x00,0x00,0xff) );
					scene->insert(gl_path);
				} else {
					gl_path = mrpt::opengl::CSetOfLinesPtr(gl_path_r);
				}
				if ((int)m_logdata_ptg_paths.size()>log.nSelectedPTG)
				{
					mrpt::nav::CParameterizedTrajectoryGeneratorPtr ptg = m_logdata_ptg_paths[log.nSelectedPTG];
					if (ptg)
					{
						// Draw path:
						const int selected_k = ptg->alpha2index( log.infoPerPTG[log.nSelectedPTG].desiredDirection );
						float max_dist = ptg->refDistance;
						gl_path->clear();
						ptg->renderPathAsSimpleLine(selected_k,*gl_path,0.10, max_dist);
						gl_path->setColor_u8( mrpt::utils::TColor(0xff,0x00,0x00) );

						// Overlay a sequence of robot shapes:
						if (cbDrawShapePath->IsChecked())
						{
							double min_shape_dists = 1.0;
							edShapeMinDist->GetValue().ToDouble(&min_shape_dists);

							for (double d=min_shape_dists;d<max_dist;d+=min_shape_dists)
							{
								float x,y,phi,t;
								ptg->getCPointWhen_d_Is(d,selected_k,x,y,phi,t);
								mrpt::poses::CPose2D pose(x,y,phi);
								add_robotShape_to_setOfLines(shap_x,shap_y, *gl_path, pose);
							}
						}
					}
				}
			}
			{
				// Robot shape:
				mrpt::opengl::CSetOfLinesPtr   gl_shape;
				mrpt::opengl::CRenderizablePtr gl_shape_r = scene->getByName("shape");  // Get or create if new
				if (!gl_shape_r) {
					gl_shape = mrpt::opengl::CSetOfLines::Create();
					gl_shape->setName("shape");
					gl_shape->setLineWidth(4.0);
					gl_shape->setColor_u8( mrpt::utils::TColor(0xff,0x00,0x00) );
					scene->insert(gl_shape);
				} else {
					gl_shape = mrpt::opengl::CSetOfLinesPtr(gl_shape_r);
				}
				gl_shape->clear();
				add_robotShape_to_setOfLines(shap_x,shap_y, *gl_shape);
			}
			{
				// Target:
				mrpt::opengl::CPointCloudPtr   gl_trg;
				mrpt::opengl::CRenderizablePtr gl_trg_r = scene->getByName("target");  // Get or create if new
				if (!gl_trg_r) {
					gl_trg = mrpt::opengl::CPointCloud::Create();
					gl_trg->setName("target");
					gl_trg->enableShowName(true);
					gl_trg->setPointSize(9.0);
					gl_trg->setColor_u8( mrpt::utils::TColor(0x00,0x00,0x00) );
					scene->insert(gl_trg);
				} else {
					gl_trg = mrpt::opengl::CPointCloudPtr(gl_trg_r);
				}
				// Move the map & add a point at (0,0,0) so the name label appears at the target:
				gl_trg->clear();
				gl_trg->setLocation(log.WS_target_relative.x,log.WS_target_relative.y,0);
				gl_trg->insertPoint(0,0,0);
			}

			win1->unlockAccess3DScene();
		}

		// Show extra info as text msgs:
		// ---------------------------------
		const double fy = 10, Ay = 15;   // Font size & line spaces

		win1->addTextMessage(5.0, 5+0*Ay, mrpt::format("Timestamp: %s",mrpt::system::dateTimeLocalToString( log.timestamp ).c_str()),
				mrpt::utils::TColorf(1,1,1), "mono", fy, mrpt::opengl::NICE,  0 /*unique txt index*/ );
		win1->addTextMessage(5.0, 5+1*Ay, mrpt::format("cmd_{v,w}={%5.02f m/s,%5.02f deg/s} current_{v,w}={%5.02f m/s,%5.02f deg/s}",log.v, RAD2DEG(log.w), log.actual_v,RAD2DEG(log.actual_w) ),
				mrpt::utils::TColorf(1,1,1), "mono", fy, mrpt::opengl::NICE,  1 /*unique txt index*/ );
		for (unsigned int nPTG=0;nPTG<log.nPTGs;nPTG++)
		{
			const CLogFileRecord::TInfoPerPTG &pI = log.infoPerPTG[nPTG];

			mrpt::utils::TColorf col; 
			if (((int)nPTG)==log.nSelectedPTG)
			     col = mrpt::utils::TColorf(1,1,1);
			else col = mrpt::utils::TColorf(.8,.8,.8);

			win1->addTextMessage(5.0, 5+ Ay*(2+nPTG),
				mrpt::format("PTG#%u: Eval=%5.03f factors=%s", nPTG, pI.evaluation, sprintf_vector("%5.02f ", pI.evalFactors).c_str() ),
				col, "mono", fy, mrpt::opengl::NICE,  10+nPTG /*unique txt index*/ );

		}

		win1->repaint();
	}

	// Draw PTG-obstacles
	// --------------------------------
	for (unsigned int nPTG=0;nPTG<log.nPTGs;nPTG++)
	{
		CDisplayWindowPlotsPtr &win = m_mywins[format("PTG%u",nPTG)];
		if (!win)  {
			const static int W = 290;
			const static int H = 270;

			win= CDisplayWindowPlots::Create(format("%u|TP-Obstacles [%s]",nPTG,log.infoPerPTG[nPTG].PTG_desc.c_str()),W,H);
			win->setPos(20+(W+30)*nPTG, 380);
			win->axis(-1,1,-1,1, true);

			// Draw static stuff:
			win->plot( make_vector<1,double>(0),make_vector<1,double>(0),"r.5",  "central_dot");

			vector<float> xs,ys;
			for (size_t i=0;i<100;++i)
			{
				xs.push_back(cos(i*2*M_PI/100));
				ys.push_back(sin(i*2*M_PI/100));
			}
			win->plot(xs,ys,"k-", "out_circle");
		}
		// Draw dynamic stuff:
		const CLogFileRecord::TInfoPerPTG &pI = log.infoPerPTG[nPTG];
		vector<float> xs,ys;

		const size_t nAlphas = pI.TP_Obstacles.size();
		//ASSERT_(nAlphas>0)  // In case of "invalid" PTGs during navigation, TP_Obstacles may be left uncomputed.

		// Chosen direction:
		xs.resize(2);
		ys.resize(2);
		xs[0] = 0; ys[0] = 0;
		const double aDir = pI.desiredDirection;
		xs[1] = 0.8*cos(aDir);
		ys[1] = 0.8*sin(aDir);

		win->plot(xs,ys,"g-5","SEL_DIR");


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
		win->plot(xs,ys,"b-2", "TPOBS");

		// Target:
		win->plot(make_vector<1,double>(pI.TP_Target.x),make_vector<1,double>(pI.TP_Target.y),"k.9", "TPTARGET");


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
		loadLogfile(global_fileToOpen);
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
      << "%  From log: " << string(edLogFile->GetValue().mbc_str()) << "\n"
      << "% -------------------------------------------------------------------------\n\n";

    f << "%%\n"
    << "function [] = main()\n"
    << "% Robot shape: (x,y) in each line\n"
    << "rs = [-0.3 -0.3;0.6 -0.3;0.6 0.3;-0.3 0.3];\n"
    << "decimate_robot_shapes = 15;"
    << "decim_cnt=0;";

    const int DECIMATE_POINTS = 10;
    int decim_point_cnt =0;

    std::vector<float> X,Y;    // Obstacles
    std::vector<float> TX,TY;  // Target over time

    const size_t N = m_logdata.size();
	for (size_t i=0;i<N;i++)
	{
        const CLogFileRecordPtr logsptr = CLogFileRecordPtr( m_logdata[i] );
        const CLogFileRecord * logptr = logsptr.pointer();

        f << format("decim_cnt=decim_cnt+1; if (decim_cnt>=decimate_robot_shapes); drawRobotShape(rs,[%f %f %f]); decim_cnt=0; end\n",
            logptr->robotOdometryPose.x(),
            logptr->robotOdometryPose.y(),
            logptr->robotOdometryPose.phi()
            );

        if (++decim_point_cnt>=DECIMATE_POINTS)
        {
            CSimplePointsMap pts;
            pts.changeCoordinatesReference( logptr->WS_Obstacles, logptr->robotOdometryPose );

            const std::vector<float> &pX = pts.getPointsBufferRef_x();
            const std::vector<float> &pY = pts.getPointsBufferRef_y();

            X.insert(X.begin(),pX.begin(),pX.end());
            Y.insert(Y.begin(),pY.begin(),pY.end());
        }

        // Target:
        const mrpt::math::TPoint2D trg_glob = mrpt::math::TPoint2D( logptr->robotOdometryPose+logptr->WS_target_relative );
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
	  << "plot(Ts(:,1),Ts(:,2),'rx','MarkerSize',5);\n";

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
