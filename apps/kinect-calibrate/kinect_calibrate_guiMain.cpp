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

#include <mrpt/utils/CImage.h>
#include <mrpt/utils/CMemoryStream.h>

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
//*)

BEGIN_EVENT_TABLE(kinect_calibrate_guiDialog,wxDialog)
    //(*EventTable(kinect_calibrate_guiDialog)
    //*)
END_EVENT_TABLE()

kinect_calibrate_guiDialog::kinect_calibrate_guiDialog(wxWindow* parent,wxWindowID id)
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
    wxFlexGridSizer* FlexGridSizer4;
    wxStaticBoxSizer* StaticBoxSizer3;
    wxFlexGridSizer* FlexGridSizer6;
    wxFlexGridSizer* FlexGridSizer3;
    wxStaticBoxSizer* StaticBoxSizer4;
    wxFlexGridSizer* FlexGridSizer18;

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
    Panel4 = new wxPanel(Notebook1, ID_PANEL4, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL4"));
    FlexGridSizer3 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer3->AddGrowableCol(0);
    m_realtimeview = new mrpt::gui::wxMRPTImageControl(Panel4,ID_CUSTOM1,wxDefaultPosition.x,wxDefaultPosition.y,wxSize(640,480).GetWidth(), wxSize(640,480).GetHeight() );
    FlexGridSizer3->Add(m_realtimeview, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
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
    FlexGridSizer1->Fit(this);
    FlexGridSizer1->SetSizeHints(this);
    Center();

    Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnNext1Click);
    Connect(ID_NOTEBOOK1,wxEVT_COMMAND_NOTEBOOK_PAGE_CHANGING,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnNotebook1PageChanging);
    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnAbout);
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnQuitClick);
    //*)

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
