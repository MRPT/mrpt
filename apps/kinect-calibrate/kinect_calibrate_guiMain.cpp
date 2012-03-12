/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
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

/*
  App      : kinect-calibrate
  Web page : http://www.mrpt.org/Kinect_and_MRPT

  Usage    : Run and follow on-screen instructions
*/

#include "kinect_calibrate_guiMain.h"
#include <wx/msgdlg.h>
#include "CAboutBox.h"

//(*InternalHeaders(kinect_calibrate_guiDialog)
#include <wx/string.h>
#include <wx/intl.h>
//*)

#include <mrpt/hwdrivers/CKinect.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::hwdrivers;
using namespace std;


#include "../wx-common/mrpt_logo.xpm"
#include "imgs/kinect.xpm"
#include "imgs/kinect-covered-projector.h"

// A custom Art provider for customizing the icons:
class MyArtProvider : public wxArtProvider
{
protected:
    virtual wxBitmap CreateBitmap(const wxArtID& id,
                                  const wxArtClient& client,
                                  const wxSize& size);
};
wxBitmap MyArtProvider::CreateBitmap(const wxArtID& id,
                                     const wxArtClient& client,
                                     const wxSize& size)
{
    if (id == wxART_MAKE_ART_ID(MAIN_ICON))   return wxBitmap(kinect_xpm);
    if (id == wxART_MAKE_ART_ID(IMG_MRPT_LOGO))  return wxBitmap(mrpt_logo_xpm);

    // Any wxWidgets icons not implemented here
    // will be provided by the default art provider.
    return wxNullBitmap;
}



//(*IdInit(kinect_calibrate_guiDialog)
const long kinect_calibrate_guiDialog::ID_CUSTOM2 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT6 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT7 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT8 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT5 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON3 = wxNewId();
const long kinect_calibrate_guiDialog::ID_TEXTCTRL2 = wxNewId();
const long kinect_calibrate_guiDialog::ID_PANEL1 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CUSTOM3 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT11 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON5 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT9 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON4 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT10 = wxNewId();
const long kinect_calibrate_guiDialog::ID_TEXTCTRL4 = wxNewId();
const long kinect_calibrate_guiDialog::ID_PANEL3 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CUSTOM1 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT1 = wxNewId();
const long kinect_calibrate_guiDialog::ID_SPINCTRL1 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT2 = wxNewId();
const long kinect_calibrate_guiDialog::ID_SPINCTRL2 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT3 = wxNewId();
const long kinect_calibrate_guiDialog::ID_TEXTCTRL1 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT4 = wxNewId();
const long kinect_calibrate_guiDialog::ID_TEXTCTRL3 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CHECKBOX1 = wxNewId();
const long kinect_calibrate_guiDialog::ID_PANEL7 = wxNewId();
const long kinect_calibrate_guiDialog::ID_PANEL4 = wxNewId();
const long kinect_calibrate_guiDialog::ID_PANEL5 = wxNewId();
const long kinect_calibrate_guiDialog::ID_PANEL6 = wxNewId();
const long kinect_calibrate_guiDialog::ID_NOTEBOOK1 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON1 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON2 = wxNewId();
const long kinect_calibrate_guiDialog::ID_PANEL2 = wxNewId();
const long kinect_calibrate_guiDialog::ID_TIMER1 = wxNewId();
const long kinect_calibrate_guiDialog::ID_TIMER2 = wxNewId();
//*)

BEGIN_EVENT_TABLE(kinect_calibrate_guiDialog,wxDialog)
    //(*EventTable(kinect_calibrate_guiDialog)
    //*)
END_EVENT_TABLE()

kinect_calibrate_guiDialog::kinect_calibrate_guiDialog(wxWindow* parent,wxWindowID id)
	: m_my_redirector(NULL)
{
	// Load my custom icons:
#if wxCHECK_VERSION(2, 8, 0)
    wxArtProvider::Push(new MyArtProvider);
#else
    wxArtProvider::PushProvider(new MyArtProvider);
#endif

    //(*Initialize(kinect_calibrate_guiDialog)
    wxFlexGridSizer* FlexGridSizer1;
    wxFlexGridSizer* FlexGridSizer2;
    wxStaticBoxSizer* StaticBoxSizer5;
    wxFlexGridSizer* FlexGridSizer17;
    wxFlexGridSizer* FlexGridSizer7;
    wxFlexGridSizer* FlexGridSizer4;
    wxStaticBoxSizer* StaticBoxSizer3;
    wxFlexGridSizer* FlexGridSizer6;
    wxFlexGridSizer* FlexGridSizer3;
    wxStaticBoxSizer* StaticBoxSizer4;
    wxFlexGridSizer* FlexGridSizer18;
    wxFlexGridSizer* FlexGridSizer5;

    Create(parent, id, _("Kinect calibration Wizard - Part of MRPT"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE, _T("id"));
    FlexGridSizer1 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer1->AddGrowableCol(0);
    FlexGridSizer1->AddGrowableRow(0);
    Notebook1 = new wxNotebook(this, ID_NOTEBOOK1, wxDefaultPosition, wxDefaultSize, 0, _T("ID_NOTEBOOK1"));
    Panel1 = new wxPanel(Notebook1, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    m_imgStaticKinect = new mrpt::gui::wxMRPTImageControl(Panel1,ID_CUSTOM2,wxPoint(280,96).x,wxPoint(280,96).y,wxSize(300,116).GetWidth(), wxSize(300,116).GetHeight() );
    StaticText6 = new wxStaticText(Panel1, ID_STATICTEXT6, _("Prepare to calibrate your Kinect sensor:"), wxPoint(40,32), wxDefaultSize, 0, _T("ID_STATICTEXT6"));
    StaticText7 = new wxStaticText(Panel1, ID_STATICTEXT7, _("1) Make sure the IR projector is covered, for example, by tapying a piece of some opaque material on it."), wxPoint(56,56), wxSize(805,45), 0, _T("ID_STATICTEXT7"));
    StaticText8 = new wxStaticText(Panel1, ID_STATICTEXT8, _("2) Print a checkerboard and stick it firmly to some rigid board or paperboard.\n   If you don\'t have any pattern at hand, get one from:"), wxPoint(48,256), wxSize(661,37), 0, _T("ID_STATICTEXT8"));
    StaticText5 = new wxStaticText(Panel1, ID_STATICTEXT5, _("3) Connect the Kinect to the computer and wait a few seconds until the driver establishes connection."), wxPoint(40,360), wxSize(757,45), 0, _T("ID_STATICTEXT5"));
    btnNext1 = new wxButton(Panel1, ID_BUTTON3, _("START"), wxPoint(384,416), wxSize(117,45), 0, wxDefaultValidator, _T("ID_BUTTON3"));
    btnNext1->SetDefault();
    TextCtrl1 = new wxTextCtrl(Panel1, ID_TEXTCTRL2, _("http://www.mrpt.org/downloads/camera-calibration-checker-board_9x7.pdf"), wxPoint(64,296), wxSize(605,27), wxTE_READONLY, wxDefaultValidator, _T("ID_TEXTCTRL2"));
    Panel3 = new wxPanel(Notebook1, ID_PANEL3, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL3"));
    FlexGridSizer5 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer5->AddGrowableCol(1);
    m_realtimeview_test = new mrpt::gui::wxMRPTImageControl(Panel3,ID_CUSTOM3,wxDefaultPosition.x,wxDefaultPosition.y,wxSize(640,480).GetWidth(), wxSize(640,480).GetHeight() );
    FlexGridSizer5->Add(m_realtimeview_test, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer7 = new wxFlexGridSizer(6, 1, 0, 0);
    FlexGridSizer7->AddGrowableCol(0);
    FlexGridSizer7->AddGrowableRow(5);
    StaticText11 = new wxStaticText(Panel3, ID_STATICTEXT11, _("Press \"Connect\" to start grabbing:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT11"));
    FlexGridSizer7->Add(StaticText11, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnConnect = new wxButton(Panel3, ID_BUTTON5, _("Connect..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON5"));
    FlexGridSizer7->Add(btnConnect, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText9 = new wxStaticText(Panel3, ID_STATICTEXT9, _("You should be seeing the \nRGB channel on the left panel.\n\nIf everything works OK, \npress \"Continue\"."), wxPoint(40,32), wxDefaultSize, 0, _T("ID_STATICTEXT9"));
    FlexGridSizer7->Add(StaticText9, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnNext2 = new wxButton(Panel3, ID_BUTTON4, _("CONTINUE..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON4"));
    btnNext2->SetDefault();
    btnNext2->Disable();
    FlexGridSizer7->Add(btnNext2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText10 = new wxStaticText(Panel3, ID_STATICTEXT10, _("Log:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT10"));
    FlexGridSizer7->Add(StaticText10, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edLogTest = new wxTextCtrl(Panel3, ID_TEXTCTRL4, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_AUTO_SCROLL|wxTE_MULTILINE|wxTE_READONLY|wxHSCROLL|wxVSCROLL, wxDefaultValidator, _T("ID_TEXTCTRL4"));
    FlexGridSizer7->Add(edLogTest, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer5->Add(FlexGridSizer7, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    Panel3->SetSizer(FlexGridSizer5);
    FlexGridSizer5->Fit(Panel3);
    FlexGridSizer5->SetSizeHints(Panel3);
    Panel4 = new wxPanel(Notebook1, ID_PANEL4, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL4"));
    FlexGridSizer3 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer3->AddGrowableCol(1);
    m_realtimeview_cap = new mrpt::gui::wxMRPTImageControl(Panel4,ID_CUSTOM1,wxDefaultPosition.x,wxDefaultPosition.y,wxSize(640,480).GetWidth(), wxSize(640,480).GetHeight() );
    FlexGridSizer3->Add(m_realtimeview_cap, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel7 = new wxPanel(Panel4, ID_PANEL7, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL7"));
    FlexGridSizer4 = new wxFlexGridSizer(0, 3, 0, 0);
    StaticBoxSizer3 = new wxStaticBoxSizer(wxHORIZONTAL, Panel7, _("Checkerboard parameters "));
    FlexGridSizer6 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer6->AddGrowableCol(0);
    FlexGridSizer6->AddGrowableCol(1);
    StaticBoxSizer4 = new wxStaticBoxSizer(wxHORIZONTAL, Panel7, _("Number of inner corners: "));
    FlexGridSizer17 = new wxFlexGridSizer(1, 4, 0, 0);
    StaticText1 = new wxStaticText(Panel7, ID_STATICTEXT1, _("In X:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    FlexGridSizer17->Add(StaticText1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edSizeX = new wxSpinCtrl(Panel7, ID_SPINCTRL1, _T("5"), wxDefaultPosition, wxSize(50,-1), 0, 1, 200, 5, _T("ID_SPINCTRL1"));
    edSizeX->SetValue(_T("5"));
    FlexGridSizer17->Add(edSizeX, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText2 = new wxStaticText(Panel7, ID_STATICTEXT2, _("In Y:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    FlexGridSizer17->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edSizeY = new wxSpinCtrl(Panel7, ID_SPINCTRL2, _T("8"), wxDefaultPosition, wxSize(50,-1), 0, 1, 200, 8, _T("ID_SPINCTRL2"));
    edSizeY->SetValue(_T("8"));
    FlexGridSizer17->Add(edSizeY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer4->Add(FlexGridSizer17, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer6->Add(StaticBoxSizer4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer5 = new wxStaticBoxSizer(wxHORIZONTAL, Panel7, _(" Size of quads (in mm): "));
    FlexGridSizer18 = new wxFlexGridSizer(1, 4, 0, 0);
    StaticText3 = new wxStaticText(Panel7, ID_STATICTEXT3, _("In X:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
    FlexGridSizer18->Add(StaticText3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edLengthX = new wxTextCtrl(Panel7, ID_TEXTCTRL1, _("40.0"), wxDefaultPosition, wxSize(40,-1), 0, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    FlexGridSizer18->Add(edLengthX, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText4 = new wxStaticText(Panel7, ID_STATICTEXT4, _("In Y:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT4"));
    FlexGridSizer18->Add(StaticText4, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edLengthY = new wxTextCtrl(Panel7, ID_TEXTCTRL3, _("40.0"), wxDefaultPosition, wxSize(40,-1), 0, wxDefaultValidator, _T("ID_TEXTCTRL3"));
    FlexGridSizer18->Add(edLengthY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer5->Add(FlexGridSizer18, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer6->Add(StaticBoxSizer5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    cbNormalize = new wxCheckBox(Panel7, ID_CHECKBOX1, _("Normalize image"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
    cbNormalize->SetValue(true);
    FlexGridSizer6->Add(cbNormalize, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer3->Add(FlexGridSizer6, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 3);
    FlexGridSizer4->Add(StaticBoxSizer3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel7->SetSizer(FlexGridSizer4);
    FlexGridSizer4->Fit(Panel7);
    FlexGridSizer4->SetSizeHints(Panel7);
    FlexGridSizer3->Add(Panel7, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel4->SetSizer(FlexGridSizer3);
    FlexGridSizer3->Fit(Panel4);
    FlexGridSizer3->SetSizeHints(Panel4);
    Panel5 = new wxPanel(Notebook1, ID_PANEL5, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL5"));
    Panel6 = new wxPanel(Notebook1, ID_PANEL6, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL6"));
    Notebook1->AddPage(Panel1, _("0) Instructions"), false);
    Notebook1->AddPage(Panel3, _("1) Test connection"), false);
    Notebook1->AddPage(Panel4, _("2) Capture"), false);
    Notebook1->AddPage(Panel5, _("3) Verify"), false);
    Notebook1->AddPage(Panel6, _("4) Optimization"), false);
    FlexGridSizer1->Add(Notebook1, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel2 = new wxPanel(this, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
    FlexGridSizer2 = new wxFlexGridSizer(0, 3, 0, 0);
    FlexGridSizer2->AddGrowableCol(0);
    FlexGridSizer2->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    bntAbout = new wxButton(Panel2, ID_BUTTON1, _("About"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
    FlexGridSizer2->Add(bntAbout, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnQuit = new wxButton(Panel2, ID_BUTTON2, _("Quit"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
    FlexGridSizer2->Add(btnQuit, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel2->SetSizer(FlexGridSizer2);
    FlexGridSizer2->Fit(Panel2);
    FlexGridSizer2->SetSizeHints(Panel2);
    FlexGridSizer1->Add(Panel2, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    SetSizer(FlexGridSizer1);
    timConsoleDump.SetOwner(this, ID_TIMER1);
    timConsoleDump.Start(100, false);
    timMisc.SetOwner(this, ID_TIMER2);
    timMisc.Start(2, false);
    FlexGridSizer1->Fit(this);
    FlexGridSizer1->SetSizeHints(this);
    Center();

    Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnNext1Click);
    Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnConnectClick);
    Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnNext1Click);
    Connect(ID_NOTEBOOK1,wxEVT_COMMAND_NOTEBOOK_PAGE_CHANGING,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnNotebook1PageChanging);
    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnAbout);
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnQuitClick);
    Connect(ID_TIMER1,wxEVT_TIMER,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OntimConsoleDumpTrigger);
    Connect(ID_TIMER2,wxEVT_TIMER,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OntimMiscTrigger);
    Connect(wxID_ANY,wxEVT_CLOSE_WINDOW,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnClose);
    //*)

	// Set std::cout/cerr out:
	m_my_redirector = new CMyRedirector(
		edLogTest,
		false, // yieldApplication
		3000, // bufferSize
		true, // also_cerr
		true, // threadSafe -> we must call dumpNow()
		true // also_to_cout_cerr
		);

	// App icon:
    this->SetIcon( wxIcon(kinect_xpm) );

    // Load image from embedded JPEG image in .h:
    try
    {
		mrpt::utils::CMemoryStream img1_jpeg_stream( kinect_covered_projector_img_jpeg, sizeof(kinect_covered_projector_img_jpeg) );
		mrpt::utils::CImage  img1;
		img1.loadFromStreamAsJPEG(img1_jpeg_stream);
		m_imgStaticKinect->AssignImage(img1);
    }
	catch(...) { }

}

kinect_calibrate_guiDialog::~kinect_calibrate_guiDialog()
{
	delete m_my_redirector; m_my_redirector = NULL;

    //(*Destroy(kinect_calibrate_guiDialog)
    //*)
}

void kinect_calibrate_guiDialog::OnQuit(wxCommandEvent& event)
{
    Close();
}

void kinect_calibrate_guiDialog::OnAbout(wxCommandEvent& event)
{
	CAboutBox	dlg(this);
	dlg.ShowModal();
}

// Don't allow the user to change tab by hand:
void kinect_calibrate_guiDialog::OnNotebook1PageChanging(wxNotebookEvent& event)
{
	event.Veto();
}

void kinect_calibrate_guiDialog::OnbtnNext1Click(wxCommandEvent& event)
{
	Notebook1->ChangeSelection( Notebook1->GetSelection() + 1 );
}

void kinect_calibrate_guiDialog::OnbtnQuitClick(wxCommandEvent& event)
{
	Close();
}

void kinect_calibrate_guiDialog::OnbtnConnectClick(wxCommandEvent& event)
{
	btnNext2->Enable(false);

	if (!m_cap_thread.isClear())
	{
		// Shoulnd't reach here...just in case:
		btnConnect->Enable(false);
		return;
	}

	m_cap_thread_data.quit = false;

	// Launch thread:
	m_cap_thread = mrpt::system::createThreadFromObjectMethod(this, &kinect_calibrate_guiDialog::thread_grabbing);
	btnConnect->Enable(false);

	// ...

}


void kinect_calibrate_guiDialog::OntimConsoleDumpTrigger(wxTimerEvent& event)
{
	if (m_my_redirector) m_my_redirector->dumpNow();
}


// The thread in charge of opening the Kinect sensor & grabbing data.
void kinect_calibrate_guiDialog::thread_grabbing()
{
	TThreadParam &p = this->m_cap_thread_data;
	p.terminated = false;

	try
	{
		CKinect  kinect;

		// Set params:
		// kinect.enableGrab3DPoints(true);
		// kinect.enablePreviewRGB(true);
		//...

		// Open:
		cout << "Calling CKinect::initialize()...";
		kinect.initialize();
		cout << "OK\n";

		CTicTac tictac;
		int nImgs = 0;
		bool there_is_obs=true, hard_error=false;

		while (!hard_error && !p.quit)
		{
			// Grab new observation from the camera:
			CObservation3DRangeScanPtr  obs     = CObservation3DRangeScan::Create(); // Smart pointers to observations

			kinect.getNextObservation(*obs,there_is_obs,hard_error);

			if (!hard_error && there_is_obs)
			{
				p.new_obs.set(obs);
			}

			if (p.command!=0)
			{
				switch (p.command)
				{
					case 's':
						p.tilt_ang_deg-=1;
						if (p.tilt_ang_deg<-31) p.tilt_ang_deg=-31;
						kinect.setTiltAngleDegrees(p.tilt_ang_deg);
						break;
					case 'w':
						p.tilt_ang_deg+=1;
						if (p.tilt_ang_deg>31) p.tilt_ang_deg=31;
						kinect.setTiltAngleDegrees(p.tilt_ang_deg);
						break;
					case 'c':
						// Switch video input:
						kinect.setVideoChannel( kinect.getVideoChannel()==CKinect::VIDEO_CHANNEL_RGB ?  CKinect::VIDEO_CHANNEL_IR : CKinect::VIDEO_CHANNEL_RGB);
						break;
					case 27:
						p.quit = true;
						break;
				}

				// Clear pushed key flag:
				p.command = 0;
			}

			nImgs++;
			if (nImgs>10)
			{
				p.Hz = nImgs / tictac.Tac();
				nImgs=0;
				tictac.Tic();
			}
		}
	}
	catch(std::exception &e)
	{
		cout << "Exception in Kinect thread: " << e.what() << endl;
		p.quit = true;  // Exit for some error
	}
	p.terminated = true;
}


void kinect_calibrate_guiDialog::OntimMiscTrigger(wxTimerEvent& event)
{
	// if the thread was launched and has closed (e.g. for some error), clear the handle:
	if (!m_cap_thread.isClear() && m_cap_thread_data.terminated)
	{
		m_cap_thread.clear();
		btnConnect->Enable(true);
	}

	// If we have a new image, process it depending on the current tab:
	// -----------------------------------------------------------------
	if (!m_cap_thread.isClear())
	{	// we're grabbing:

		CObservation3DRangeScanPtr possiblyNewObs = m_cap_thread_data.new_obs.get();
		if (possiblyNewObs && possiblyNewObs->timestamp!=INVALID_TIMESTAMP &&
			(!m_last_obs  || possiblyNewObs->timestamp!=m_last_obs->timestamp ) )
		{	// It IS a new observation:
			m_last_obs     = possiblyNewObs;

			if (m_last_obs->hasIntensityImage )
			{
				try
				{
					ProcessNewGrabbedObs();
				}
				catch(std::exception &e)
				{
					cerr << e.what();
				}
			}

		} // end It IS a new observation
	} // end if m_cap_thread -> we're grabbing
}

void kinect_calibrate_guiDialog::OnClose(wxCloseEvent& event)
{
	if (!m_cap_thread.isClear() && !m_cap_thread_data.terminated)
	{
		m_cap_thread_data.quit = true;
		cout << "Waiting for the grabbing thread to end...\n";
		mrpt::system::joinThread( m_cap_thread );
		m_cap_thread.clear();
		cout << "Grabbing thread closed.\n";
	}
	event.Skip();
}


// ---------------------------------------
// PROCESS NEW IMAGE (In: m_last_obs)
// ---------------------------------------
void kinect_calibrate_guiDialog::ProcessNewGrabbedObs()
{
	switch (Notebook1->GetSelection())
	{
	default:
		break;

	// ------------------------------------------
	//   Tab 1: Testing
	// ------------------------------------------
	case 1:
		{
			m_realtimeview_test->AssignImage( m_last_obs->intensityImage );
			m_realtimeview_test->Refresh(false);
			if (!btnNext2->IsEnabled()) btnNext2->Enable();
		}
		break;

	}


}

