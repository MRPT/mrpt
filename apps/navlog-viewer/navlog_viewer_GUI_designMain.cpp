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


#include "navlog_viewer_GUI_designMain.h"
#include "CAboutBox.h"
#include <wx/msgdlg.h>

//(*InternalHeaders(navlog_viewer_GUI_designDialog)
#include <wx/string.h>
#include <wx/intl.h>
#include <wx/tglbtn.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/artprov.h>
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

extern std::string global_fileToOpen;

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::gui;
using namespace mrpt::reactivenav;


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
    wxFlexGridSizer* FlexGridSizer1;
    wxFlexGridSizer* FlexGridSizer2;
    wxFlexGridSizer* FlexGridSizer7;
    wxFlexGridSizer* FlexGridSizer4;
    wxFlexGridSizer* FlexGridSizer6;
    wxFlexGridSizer* FlexGridSizer3;
    wxFlexGridSizer* FlexGridSizer5;
    wxStaticBoxSizer* StaticBoxSizer1;

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
    timAutoload.Start(20, true);
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
			}

			if (m_log_first_tim == INVALID_TIMESTAMP && m_log_last_tim!=INVALID_TIMESTAMP)
                m_log_first_tim = m_log_last_tim;
		}
		catch (CExceptionEOF &)
		{
			break;
		}
		catch (std::exception &)
		{
			// EOF in the middle of an object... It may be usual if the logger is shut down not cleanly.
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

	// Draw WS-obstacles
	// --------------------------------
	{
		CDisplayWindowPlotsPtr &win1 = m_mywins["WS_obs"];
		if (!win1)  {
			win1= CDisplayWindowPlots::Create("Sensed obstacles",300,270);
			win1->setPos(600,20);
			win1->axis(-5,5,-5,5, true);
		}

		win1->clf();
		win1->hold_on();

		// Obstacles:
		win1->plot(log.WS_Obstacles.getPointsBufferRef_x(),log.WS_Obstacles.getPointsBufferRef_y(),"b.4");

		// Robot shape:
		vector_float shap_x = log.robotShape_x;
		vector_float shap_y = log.robotShape_y;
		if (!shap_x.empty()) shap_x.push_back(shap_x[0]);
		if (!shap_y.empty()) shap_y.push_back(shap_y[0]);
		win1->plot(shap_x,shap_y,"r3");

		// Target:
		win1->plot(make_vector<1,double>(log.WS_target_relative.x()),make_vector<1,double>(log.WS_target_relative.y()),"k.9");
	}

	// Draw PTG-obstacles
	// --------------------------------
	for (unsigned int nPTG=0;nPTG<log.nPTGs;nPTG++)
	{
		CDisplayWindowPlotsPtr &win = m_mywins[format("PTG%u",nPTG)];
		if (!win)  {
			const static int W = 290;
			const static int H = 270;

			win= CDisplayWindowPlots::Create(format("PT-Obstacles [%s]",log.infoPerPTG[nPTG].PTG_desc.c_str()),W,H);
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
		ASSERT_(nAlphas>0)

		// Chosen direction:
		xs.resize(2);
		ys.resize(2);
		xs[0] = 0; ys[0] = 0;
		const double aDir = pI.desiredDirection*2*M_PI/double(nAlphas);
		xs[1] = 0.8*cos(aDir);
		ys[1] = 0.8*sin(aDir);

		win->plot(xs,ys,"g-5","SEL_DIR");


		// obstacles:
		xs.clear(); ys.clear();
		xs.reserve(nAlphas); ys.reserve(nAlphas);
		for (size_t i=0;i<nAlphas;++i)
		{
			const double a = -M_PI + i*2*M_PI/double(nAlphas);
			const double r = pI.TP_Obstacles[i];
			xs.push_back(r*cos(a));
			ys.push_back(r*sin(a));
		}
		win->plot(xs,ys,"b-2", "TPOBS");

		// Target:
		win->plot(make_vector<1,double>(pI.TP_Target.x()),make_vector<1,double>(pI.TP_Target.y()),"k.9", "TPTARGET");


	} // end for each PTG

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
