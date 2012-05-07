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
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/intl.h>
#include <wx/font.h>
//*)

#include <mrpt/hwdrivers/CKinect.h>
#include <mrpt/vision/chessboard_find_corners.h>
#include <mrpt/vision/chessboard_stereo_camera_calib.h>

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
const long kinect_calibrate_guiDialog::ID_STATICTEXT21 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON3 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON8 = wxNewId();
const long kinect_calibrate_guiDialog::ID_TEXTCTRL2 = wxNewId();
const long kinect_calibrate_guiDialog::ID_PANEL1 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CUSTOM3 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT11 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON5 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT9 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON4 = wxNewId();
const long kinect_calibrate_guiDialog::ID_PANEL3 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CUSTOM1 = wxNewId();
const long kinect_calibrate_guiDialog::ID_TEXTCTRL5 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT1 = wxNewId();
const long kinect_calibrate_guiDialog::ID_SPINCTRL1 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT2 = wxNewId();
const long kinect_calibrate_guiDialog::ID_SPINCTRL2 = wxNewId();
const long kinect_calibrate_guiDialog::ID_RADIOBOX1 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CHECKBOX1 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT22 = wxNewId();
const long kinect_calibrate_guiDialog::ID_SPINCTRL7 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON6 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT12 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON7 = wxNewId();
const long kinect_calibrate_guiDialog::ID_PANEL7 = wxNewId();
const long kinect_calibrate_guiDialog::ID_PANEL4 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT23 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CHECKLISTBOX1 = wxNewId();
const long kinect_calibrate_guiDialog::ID_TEXTCTRL9 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON9 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON10 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON11 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON12 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT19 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CUSTOM4 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT20 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CUSTOM5 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT25 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT13 = wxNewId();
const long kinect_calibrate_guiDialog::ID_SPINCTRL3 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT14 = wxNewId();
const long kinect_calibrate_guiDialog::ID_SPINCTRL4 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT15 = wxNewId();
const long kinect_calibrate_guiDialog::ID_TEXTCTRL6 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT16 = wxNewId();
const long kinect_calibrate_guiDialog::ID_TEXTCTRL7 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CHECKBOX2 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CHECKBOX4 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CHECKBOX5 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CHECKBOX6 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CHECKBOX7 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CHECKBOX8 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT26 = wxNewId();
const long kinect_calibrate_guiDialog::ID_SPINCTRL8 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON14 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT24 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON13 = wxNewId();
const long kinect_calibrate_guiDialog::ID_TEXTCTRL8 = wxNewId();
const long kinect_calibrate_guiDialog::ID_PANEL6 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT3 = wxNewId();
const long kinect_calibrate_guiDialog::ID_SPINCTRL5 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT4 = wxNewId();
const long kinect_calibrate_guiDialog::ID_SPINCTRL6 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT17 = wxNewId();
const long kinect_calibrate_guiDialog::ID_TEXTCTRL1 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT18 = wxNewId();
const long kinect_calibrate_guiDialog::ID_TEXTCTRL3 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CHECKBOX3 = wxNewId();
const long kinect_calibrate_guiDialog::ID_PANEL5 = wxNewId();
const long kinect_calibrate_guiDialog::ID_NOTEBOOK1 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT10 = wxNewId();
const long kinect_calibrate_guiDialog::ID_TEXTCTRL4 = wxNewId();
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
	m_grabstate = gsIdle;

	// Load my custom icons:
#if wxCHECK_VERSION(2, 8, 0)
    wxArtProvider::Push(new MyArtProvider);
#else
    wxArtProvider::PushProvider(new MyArtProvider);
#endif

    //(*Initialize(kinect_calibrate_guiDialog)
    wxFlexGridSizer* FlexGridSizer30;
    wxStaticBoxSizer* StaticBoxSizer2;
    wxFlexGridSizer* FlexGridSizer21;
    wxFlexGridSizer* FlexGridSizer28;
    wxFlexGridSizer* FlexGridSizer8;
    wxStaticBoxSizer* StaticBoxSizer10;
    wxFlexGridSizer* FlexGridSizer1;
    wxFlexGridSizer* FlexGridSizer2;
    wxFlexGridSizer* FlexGridSizer25;
    wxFlexGridSizer* FlexGridSizer15;
    wxStaticBoxSizer* StaticBoxSizer7;
    wxStaticBoxSizer* StaticBoxSizer5;
    wxFlexGridSizer* FlexGridSizer17;
    wxFlexGridSizer* FlexGridSizer29;
    wxFlexGridSizer* FlexGridSizer19;
    wxFlexGridSizer* FlexGridSizer11;
    wxFlexGridSizer* FlexGridSizer7;
    wxFlexGridSizer* FlexGridSizer4;
    wxFlexGridSizer* FlexGridSizer9;
    wxFlexGridSizer* FlexGridSizer26;
    wxFlexGridSizer* FlexGridSizer14;
    wxStaticBoxSizer* StaticBoxSizer3;
    wxFlexGridSizer* FlexGridSizer33;
    wxFlexGridSizer* FlexGridSizer6;
    wxFlexGridSizer* FlexGridSizer27;
    wxFlexGridSizer* FlexGridSizer3;
    wxFlexGridSizer* FlexGridSizer22;
    wxStaticBoxSizer* StaticBoxSizer8;
    wxFlexGridSizer* FlexGridSizer31;
    wxStaticBoxSizer* StaticBoxSizer9;
    wxStaticBoxSizer* StaticBoxSizer4;
    wxStaticBoxSizer* StaticBoxSizer6;
    wxFlexGridSizer* FlexGridSizer16;
    wxFlexGridSizer* FlexGridSizer34;
    wxFlexGridSizer* FlexGridSizer23;
    wxFlexGridSizer* FlexGridSizer10;
    wxBoxSizer* BoxSizer1;
    wxFlexGridSizer* FlexGridSizer13;
    wxStaticBoxSizer* StaticBoxSizer11;
    wxFlexGridSizer* FlexGridSizer18;
    wxFlexGridSizer* FlexGridSizer36;
    wxFlexGridSizer* FlexGridSizer12;
    wxFlexGridSizer* FlexGridSizer35;
    wxFlexGridSizer* FlexGridSizer5;
    wxFlexGridSizer* FlexGridSizer24;
    wxFlexGridSizer* FlexGridSizer32;
    wxStaticBoxSizer* StaticBoxSizer1;
    wxFlexGridSizer* FlexGridSizer20;

    Create(parent, wxID_ANY, _("Kinect calibration Wizard - Part of MRPT"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE|wxRESIZE_BORDER|wxMAXIMIZE_BOX|wxMINIMIZE_BOX, _T("wxID_ANY"));
    FlexGridSizer1 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer1->AddGrowableCol(0);
    FlexGridSizer1->AddGrowableRow(0);
    Notebook1 = new wxNotebook(this, ID_NOTEBOOK1, wxDefaultPosition, wxDefaultSize, wxNB_BOTTOM, _T("ID_NOTEBOOK1"));
    Panel1 = new wxPanel(Notebook1, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    m_imgStaticKinect = new mrpt::gui::wxMRPTImageControl(Panel1,ID_CUSTOM2,wxPoint(280,96).x,wxPoint(280,96).y,wxSize(300,116).GetWidth(), wxSize(300,116).GetHeight() );
    StaticText6 = new wxStaticText(Panel1, ID_STATICTEXT6, _("Prepare to calibrate your Kinect sensor:"), wxPoint(40,32), wxDefaultSize, 0, _T("ID_STATICTEXT6"));
    StaticText7 = new wxStaticText(Panel1, ID_STATICTEXT7, _("1) Make sure the IR projector is covered, for example, by tapying a piece of some opaque material on it."), wxPoint(56,56), wxSize(805,45), 0, _T("ID_STATICTEXT7"));
    StaticText8 = new wxStaticText(Panel1, ID_STATICTEXT8, _("2) Print a checkerboard and stick it firmly to some rigid board or paperboard.\n   If you don\'t have any pattern at hand, get one from:"), wxPoint(48,256), wxSize(661,37), 0, _T("ID_STATICTEXT8"));
    StaticText5 = new wxStaticText(Panel1, ID_STATICTEXT5, _("3) Connect the Kinect to the computer and wait a few seconds until the driver establishes connection."), wxPoint(32,344), wxSize(757,45), 0, _T("ID_STATICTEXT5"));
    StaticText20 = new wxStaticText(Panel1, ID_STATICTEXT21, _("... or, if you already have stereo images saved to disk, go straight to the calibration optimizer:"), wxPoint(32,432), wxSize(757,21), 0, _T("ID_STATICTEXT21"));
    btnNext1 = new wxButton(Panel1, ID_BUTTON3, _("START"), wxPoint(376,368), wxSize(117,45), 0, wxDefaultValidator, _T("ID_BUTTON3"));
    btnNext1->SetDefault();
    btnSkipToCalib = new wxButton(Panel1, ID_BUTTON8, _("Skip to Calibration Optimizer"), wxPoint(320,448), wxSize(245,40), 0, wxDefaultValidator, _T("ID_BUTTON8"));
    btnSkipToCalib->SetDefault();
    TextCtrl1 = new wxTextCtrl(Panel1, ID_TEXTCTRL2, _("http://www.mrpt.org/downloads/camera-calibration-checker-board_9x7.pdf"), wxPoint(64,296), wxSize(605,27), wxTE_READONLY, wxDefaultValidator, _T("ID_TEXTCTRL2"));
    Panel3 = new wxPanel(Notebook1, ID_PANEL3, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL3"));
    FlexGridSizer5 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer5->AddGrowableCol(1);
    FlexGridSizer5->AddGrowableRow(0);
    m_realtimeview_test = new mrpt::gui::wxMRPTImageControl(Panel3,ID_CUSTOM3,wxDefaultPosition.x,wxDefaultPosition.y,wxSize(640,480).GetWidth(), wxSize(640,480).GetHeight() );
    FlexGridSizer5->Add(m_realtimeview_test, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer7 = new wxFlexGridSizer(6, 1, 0, 0);
    FlexGridSizer7->AddGrowableCol(0);
    FlexGridSizer7->AddGrowableRow(5);
    StaticText11 = new wxStaticText(Panel3, ID_STATICTEXT11, _("Press \"Connect\" to start grabbing:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT11"));
    FlexGridSizer7->Add(StaticText11, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnConnect = new wxButton(Panel3, ID_BUTTON5, _("Connect..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON5"));
    FlexGridSizer7->Add(btnConnect, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText9 = new wxStaticText(Panel3, ID_STATICTEXT9, _("You should be seeing the \nRGB channel on the left panel.\n\nIf everything works OK, \npress \"Continue\"."), wxPoint(40,32), wxDefaultSize, 0, _T("ID_STATICTEXT9"));
    FlexGridSizer7->Add(StaticText9, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnNext2 = new wxButton(Panel3, ID_BUTTON4, _("CONTINUE..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON4"));
    btnNext2->SetDefault();
    btnNext2->Disable();
    FlexGridSizer7->Add(btnNext2, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer5->Add(FlexGridSizer7, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    Panel3->SetSizer(FlexGridSizer5);
    FlexGridSizer5->Fit(Panel3);
    FlexGridSizer5->SetSizeHints(Panel3);
    Panel4 = new wxPanel(Notebook1, ID_PANEL4, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL4"));
    FlexGridSizer3 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer3->AddGrowableCol(1);
    FlexGridSizer3->AddGrowableRow(0);
    m_realtimeview_cap = new mrpt::gui::wxMRPTImageControl(Panel4,ID_CUSTOM1,wxDefaultPosition.x,wxDefaultPosition.y,wxSize(640,480).GetWidth(), wxSize(640,480).GetHeight() );
    FlexGridSizer3->Add(m_realtimeview_cap, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel7 = new wxPanel(Panel4, ID_PANEL7, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL7"));
    FlexGridSizer4 = new wxFlexGridSizer(3, 1, 0, 0);
    FlexGridSizer4->AddGrowableCol(0);
    FlexGridSizer4->AddGrowableRow(0);
    StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, Panel7, _(" INSTRUCTIONS "));
    BoxSizer1 = new wxBoxSizer(wxHORIZONTAL);
    TextCtrl2 = new wxTextCtrl(Panel7, ID_TEXTCTRL5, _("- Place the Kinect sensor such that the checkerboard is completely visible in its two channels (visible & IR). Click in the channel switch button to make sure of this.\n- Make sure the Kinect and the board are STATIC: don\'t hold them by hand, etc.\n- Only then, CLICK on \"CAPTURE\" to save one pair of images. DON\'T MOVE the Kinect while the pair of images are being captured. \n- Repeat to take as many image pairs as you want, trying to make sure that the checkerboard has been seen in as many different parts of the image field as possible."), wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE|wxTE_READONLY|wxTE_WORDWRAP, wxDefaultValidator, _T("ID_TEXTCTRL5"));
    TextCtrl2->SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BTNFACE));
    BoxSizer1->Add(TextCtrl2, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer1->Add(BoxSizer1, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer4->Add(StaticBoxSizer1, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer3 = new wxStaticBoxSizer(wxHORIZONTAL, Panel7, _("Checkerboard parameters "));
    FlexGridSizer6 = new wxFlexGridSizer(1, 1, 0, 0);
    StaticBoxSizer4 = new wxStaticBoxSizer(wxHORIZONTAL, Panel7, _("Number of inner corners: "));
    FlexGridSizer17 = new wxFlexGridSizer(1, 4, 0, 0);
    StaticText1 = new wxStaticText(Panel7, ID_STATICTEXT1, _("In X:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    FlexGridSizer17->Add(StaticText1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edSizeX = new wxSpinCtrl(Panel7, ID_SPINCTRL1, _T("7"), wxDefaultPosition, wxSize(50,-1), 0, 1, 200, 7, _T("ID_SPINCTRL1"));
    edSizeX->SetValue(_T("7"));
    FlexGridSizer17->Add(edSizeX, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText2 = new wxStaticText(Panel7, ID_STATICTEXT2, _("In Y:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    FlexGridSizer17->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edSizeY = new wxSpinCtrl(Panel7, ID_SPINCTRL2, _T("9"), wxDefaultPosition, wxSize(50,-1), 0, 1, 200, 9, _T("ID_SPINCTRL2"));
    edSizeY->SetValue(_T("9"));
    FlexGridSizer17->Add(edSizeY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer4->Add(FlexGridSizer17, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer6->Add(StaticBoxSizer4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer3->Add(FlexGridSizer6, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 3);
    FlexGridSizer4->Add(StaticBoxSizer3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer2 = new wxStaticBoxSizer(wxHORIZONTAL, Panel7, _(" Controls "));
    FlexGridSizer8 = new wxFlexGridSizer(6, 1, 0, 0);
    FlexGridSizer8->AddGrowableCol(0);
    wxString __wxRadioBoxChoices_1[2] =
    {
    _("RGB (Visible)"),
    _("IR")
    };
    rbChannelSwitch = new wxRadioBox(Panel7, ID_RADIOBOX1, _(" Channel switch"), wxDefaultPosition, wxDefaultSize, 2, __wxRadioBoxChoices_1, 1, 0, wxDefaultValidator, _T("ID_RADIOBOX1"));
    rbChannelSwitch->SetSelection(0);
    FlexGridSizer8->Add(rbChannelSwitch, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbNormalize = new wxCheckBox(Panel7, ID_CHECKBOX1, _("Normalize IR image"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
    cbNormalize->SetValue(true);
    FlexGridSizer8->Add(cbNormalize, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer25 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer25->AddGrowableCol(0);
    FlexGridSizer25->AddGrowableCol(1);
    StaticText21 = new wxStaticText(Panel7, ID_STATICTEXT22, _("Tilt (degrees):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT22"));
    FlexGridSizer25->Add(StaticText21, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edTilt = new wxSpinCtrl(Panel7, ID_SPINCTRL7, _T("0"), wxDefaultPosition, wxDefaultSize, 0, -30, 30, 0, _T("ID_SPINCTRL7"));
    edTilt->SetValue(_T("0"));
    FlexGridSizer25->Add(edTilt, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer8->Add(FlexGridSizer25, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    btnCapture = new wxButton(Panel7, ID_BUTTON6, _("CAPTURE NOW"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON6"));
    wxFont btnCaptureFont(10,wxSWISS,wxFONTSTYLE_NORMAL,wxBOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    btnCapture->SetFont(btnCaptureFont);
    FlexGridSizer8->Add(btnCapture, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    lbNumCaptured = new wxStaticText(Panel7, ID_STATICTEXT12, _("Captured image pairs: 0"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT12"));
    FlexGridSizer8->Add(lbNumCaptured, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnNextCalib = new wxButton(Panel7, ID_BUTTON7, _("OK, go to calibrate >>"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON7"));
    btnNextCalib->Disable();
    wxFont btnNextCalibFont(10,wxSWISS,wxFONTSTYLE_NORMAL,wxBOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    btnNextCalib->SetFont(btnNextCalibFont);
    FlexGridSizer8->Add(btnNextCalib, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer2->Add(FlexGridSizer8, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 3);
    FlexGridSizer4->Add(StaticBoxSizer2, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel7->SetSizer(FlexGridSizer4);
    FlexGridSizer4->Fit(Panel7);
    FlexGridSizer4->SetSizeHints(Panel7);
    FlexGridSizer3->Add(Panel7, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel4->SetSizer(FlexGridSizer3);
    FlexGridSizer3->Fit(Panel4);
    FlexGridSizer3->SetSizeHints(Panel4);
    Panel6 = new wxPanel(Notebook1, ID_PANEL6, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL6"));
    FlexGridSizer9 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer9->AddGrowableCol(0);
    FlexGridSizer9->AddGrowableRow(0);
    FlexGridSizer9->AddGrowableRow(1);
    FlexGridSizer10 = new wxFlexGridSizer(1, 3, 0, 0);
    FlexGridSizer10->AddGrowableCol(0);
    FlexGridSizer10->AddGrowableCol(1);
    FlexGridSizer10->AddGrowableCol(2);
    FlexGridSizer10->AddGrowableRow(0);
    FlexGridSizer22 = new wxFlexGridSizer(4, 1, 0, 0);
    FlexGridSizer22->AddGrowableCol(0);
    FlexGridSizer22->AddGrowableRow(1);
    FlexGridSizer22->AddGrowableRow(2);
    StaticText22 = new wxStaticText(Panel6, ID_STATICTEXT23, _("List of stereo images:"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT23"));
    wxFont StaticText22Font(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxBOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    StaticText22->SetFont(StaticText22Font);
    FlexGridSizer22->Add(StaticText22, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 2);
    lbImagePairs = new wxCheckListBox(Panel6, ID_CHECKLISTBOX1, wxDefaultPosition, wxDefaultSize, 0, 0, wxLB_SINGLE, wxDefaultValidator, _T("ID_CHECKLISTBOX1"));
    FlexGridSizer22->Add(lbImagePairs, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer32 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer32->AddGrowableCol(0);
    FlexGridSizer32->AddGrowableRow(0);
    TextCtrl7 = new wxTextCtrl(Panel6, ID_TEXTCTRL9, _("Only the marked image pairs will be used in the optimization."), wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE|wxTE_READONLY|wxTE_WORDWRAP, wxDefaultValidator, _T("ID_TEXTCTRL9"));
    wxFont TextCtrl7Font(8,wxDEFAULT,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    TextCtrl7->SetFont(TextCtrl7Font);
    FlexGridSizer32->Add(TextCtrl7, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer22->Add(FlexGridSizer32, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer26 = new wxFlexGridSizer(2, 2, 0, 0);
    FlexGridSizer26->AddGrowableCol(0);
    FlexGridSizer26->AddGrowableCol(1);
    btnListMarkAll = new wxButton(Panel6, ID_BUTTON9, _("Mark all"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON9"));
    FlexGridSizer26->Add(btnListMarkAll, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnListLoad = new wxButton(Panel6, ID_BUTTON10, _("Load..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON10"));
    FlexGridSizer26->Add(btnListLoad, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnListMarkNone = new wxButton(Panel6, ID_BUTTON11, _("Mark none"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON11"));
    FlexGridSizer26->Add(btnListMarkNone, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnListSave = new wxButton(Panel6, ID_BUTTON12, _("Save..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON12"));
    FlexGridSizer26->Add(btnListSave, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer22->Add(FlexGridSizer26, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer10->Add(FlexGridSizer22, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer23 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer23->AddGrowableCol(0);
    FlexGridSizer23->AddGrowableRow(1);
    StaticText18 = new wxStaticText(Panel6, ID_STATICTEXT19, _("(Left camera)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT19"));
    FlexGridSizer23->Add(StaticText18, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
    m_view_left = new mrpt::gui::wxMRPTImageControl(Panel6,ID_CUSTOM4,wxDefaultPosition.x,wxDefaultPosition.y,wxSize(320,240).GetWidth(), wxSize(320,240).GetHeight() );
    FlexGridSizer23->Add(m_view_left, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer10->Add(FlexGridSizer23, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
    FlexGridSizer24 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer24->AddGrowableCol(0);
    FlexGridSizer24->AddGrowableRow(1);
    StaticText19 = new wxStaticText(Panel6, ID_STATICTEXT20, _("(Right camera)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT20"));
    FlexGridSizer24->Add(StaticText19, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
    m_view_right = new mrpt::gui::wxMRPTImageControl(Panel6,ID_CUSTOM5,wxDefaultPosition.x,wxDefaultPosition.y,wxSize(320,240).GetWidth(), wxSize(320,240).GetHeight() );
    FlexGridSizer24->Add(m_view_right, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer10->Add(FlexGridSizer24, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
    FlexGridSizer9->Add(FlexGridSizer10, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer11 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer11->AddGrowableCol(1);
    FlexGridSizer11->AddGrowableRow(0);
    FlexGridSizer29 = new wxFlexGridSizer(3, 1, 0, 0);
    FlexGridSizer29->AddGrowableCol(0);
    FlexGridSizer29->AddGrowableRow(1);
    FlexGridSizer30 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer30->AddGrowableCol(0);
    StaticText24 = new wxStaticText(Panel6, ID_STATICTEXT25, _("Calibration options"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT25"));
    wxFont StaticText24Font(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxBOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    StaticText24->SetFont(StaticText24Font);
    FlexGridSizer30->Add(StaticText24, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer29->Add(FlexGridSizer30, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer31 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer31->AddGrowableCol(0);
    FlexGridSizer12 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer12->AddGrowableCol(0);
    FlexGridSizer12->AddGrowableCol(1);
    FlexGridSizer12->AddGrowableRow(0);
    StaticBoxSizer6 = new wxStaticBoxSizer(wxHORIZONTAL, Panel6, _("Checkerboard parameters "));
    FlexGridSizer20 = new wxFlexGridSizer(3, 1, 0, 0);
    StaticBoxSizer7 = new wxStaticBoxSizer(wxHORIZONTAL, Panel6, _("Number of inner corners: "));
    FlexGridSizer21 = new wxFlexGridSizer(1, 4, 0, 0);
    StaticText12 = new wxStaticText(Panel6, ID_STATICTEXT13, _("In X:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT13"));
    FlexGridSizer21->Add(StaticText12, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edCalibCheckX = new wxSpinCtrl(Panel6, ID_SPINCTRL3, _T("7"), wxDefaultPosition, wxSize(50,-1), 0, 1, 200, 7, _T("ID_SPINCTRL3"));
    edCalibCheckX->SetValue(_T("7"));
    FlexGridSizer21->Add(edCalibCheckX, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText13 = new wxStaticText(Panel6, ID_STATICTEXT14, _("In Y:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT14"));
    FlexGridSizer21->Add(StaticText13, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edCalibCheckY = new wxSpinCtrl(Panel6, ID_SPINCTRL4, _T("9"), wxDefaultPosition, wxSize(50,-1), 0, 1, 200, 9, _T("ID_SPINCTRL4"));
    edCalibCheckY->SetValue(_T("9"));
    FlexGridSizer21->Add(edCalibCheckY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer7->Add(FlexGridSizer21, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer20->Add(StaticBoxSizer7, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer8 = new wxStaticBoxSizer(wxHORIZONTAL, Panel6, _(" Size of quads (in mm): "));
    FlexGridSizer33 = new wxFlexGridSizer(1, 4, 0, 0);
    StaticText14 = new wxStaticText(Panel6, ID_STATICTEXT15, _("In X:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT15"));
    FlexGridSizer33->Add(StaticText14, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edCalibSizeX = new wxTextCtrl(Panel6, ID_TEXTCTRL6, _("40.0"), wxDefaultPosition, wxSize(40,-1), 0, wxDefaultValidator, _T("ID_TEXTCTRL6"));
    FlexGridSizer33->Add(edCalibSizeX, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText15 = new wxStaticText(Panel6, ID_STATICTEXT16, _("In Y:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT16"));
    FlexGridSizer33->Add(StaticText15, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edCalibSizeY = new wxTextCtrl(Panel6, ID_TEXTCTRL7, _("40.0"), wxDefaultPosition, wxSize(40,-1), 0, wxDefaultValidator, _T("ID_TEXTCTRL7"));
    FlexGridSizer33->Add(edCalibSizeY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer8->Add(FlexGridSizer33, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer20->Add(StaticBoxSizer8, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    cbCalibNormalize = new wxCheckBox(Panel6, ID_CHECKBOX2, _("Normalize images"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX2"));
    cbCalibNormalize->SetValue(true);
    FlexGridSizer20->Add(cbCalibNormalize, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer6->Add(FlexGridSizer20, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 3);
    FlexGridSizer12->Add(StaticBoxSizer6, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer34 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer34->AddGrowableCol(0);
    StaticBoxSizer11 = new wxStaticBoxSizer(wxHORIZONTAL, Panel6, _(" Parameters to optimize  "));
    FlexGridSizer35 = new wxFlexGridSizer(5, 1, 0, 0);
    FlexGridSizer35->AddGrowableCol(0);
    cbOptK1 = new wxCheckBox(Panel6, ID_CHECKBOX4, _("k1 (r^2 distortion)"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX4"));
    cbOptK1->SetValue(true);
    FlexGridSizer35->Add(cbOptK1, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 1);
    cbOptK2 = new wxCheckBox(Panel6, ID_CHECKBOX5, _("k2 (r^4 distortion)"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX5"));
    cbOptK2->SetValue(true);
    FlexGridSizer35->Add(cbOptK2, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 1);
    cbOptK3 = new wxCheckBox(Panel6, ID_CHECKBOX6, _("k3 (r^6 distortion)"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX6"));
    cbOptK3->SetValue(false);
    FlexGridSizer35->Add(cbOptK3, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 1);
    cbOptT1 = new wxCheckBox(Panel6, ID_CHECKBOX7, _("t1 (tang. dist.)"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX7"));
    cbOptT1->SetValue(false);
    FlexGridSizer35->Add(cbOptT1, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 1);
    cbOptT2 = new wxCheckBox(Panel6, ID_CHECKBOX8, _("t2 (tang. dist.)"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX8"));
    cbOptT2->SetValue(false);
    FlexGridSizer35->Add(cbOptT2, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 1);
    StaticBoxSizer11->Add(FlexGridSizer35, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer34->Add(StaticBoxSizer11, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer36 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer36->AddGrowableCol(1);
    StaticText25 = new wxStaticText(Panel6, ID_STATICTEXT26, _("Max. iterations:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT26"));
    FlexGridSizer36->Add(StaticText25, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edMaxIters = new wxSpinCtrl(Panel6, ID_SPINCTRL8, _T("1000"), wxDefaultPosition, wxDefaultSize, 0, 0, 10000, 1000, _T("ID_SPINCTRL8"));
    edMaxIters->SetValue(_T("1000"));
    FlexGridSizer36->Add(edMaxIters, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer34->Add(FlexGridSizer36, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer12->Add(FlexGridSizer34, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer31->Add(FlexGridSizer12, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    btnRunCalib = new wxButton(Panel6, ID_BUTTON14, _("Run optimizer..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON14"));
    wxFont btnRunCalibFont(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxBOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    btnRunCalib->SetFont(btnRunCalibFont);
    FlexGridSizer31->Add(btnRunCalib, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer29->Add(FlexGridSizer31, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer11->Add(FlexGridSizer29, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer27 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer27->AddGrowableCol(0);
    FlexGridSizer27->AddGrowableRow(1);
    FlexGridSizer28 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer28->AddGrowableCol(0);
    StaticText23 = new wxStaticText(Panel6, ID_STATICTEXT24, _("Calibration results:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT24"));
    wxFont StaticText23Font(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxBOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    StaticText23->SetFont(StaticText23Font);
    FlexGridSizer28->Add(StaticText23, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnSaveCalib = new wxButton(Panel6, ID_BUTTON13, _("Save calibration..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON13"));
    FlexGridSizer28->Add(btnSaveCalib, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer27->Add(FlexGridSizer28, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    edLogCalibResult = new wxTextCtrl(Panel6, ID_TEXTCTRL8, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE|wxTE_READONLY|wxTE_DONTWRAP, wxDefaultValidator, _T("ID_TEXTCTRL8"));
    wxFont edLogCalibResultFont(8,wxTELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    edLogCalibResult->SetFont(edLogCalibResultFont);
    FlexGridSizer27->Add(edLogCalibResult, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer11->Add(FlexGridSizer27, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer9->Add(FlexGridSizer11, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    Panel6->SetSizer(FlexGridSizer9);
    FlexGridSizer9->Fit(Panel6);
    FlexGridSizer9->SetSizeHints(Panel6);
    Panel5 = new wxPanel(Notebook1, ID_PANEL5, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL5"));
    FlexGridSizer15 = new wxFlexGridSizer(3, 3, 0, 0);
    StaticBoxSizer5 = new wxStaticBoxSizer(wxHORIZONTAL, Panel5, _("Checkerboard parameters "));
    FlexGridSizer16 = new wxFlexGridSizer(3, 1, 0, 0);
    StaticBoxSizer9 = new wxStaticBoxSizer(wxHORIZONTAL, Panel5, _("Number of inner corners: "));
    FlexGridSizer18 = new wxFlexGridSizer(1, 4, 0, 0);
    StaticText3 = new wxStaticText(Panel5, ID_STATICTEXT3, _("In X:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
    FlexGridSizer18->Add(StaticText3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    SpinCtrl3 = new wxSpinCtrl(Panel5, ID_SPINCTRL5, _T("5"), wxDefaultPosition, wxSize(50,-1), 0, 1, 200, 5, _T("ID_SPINCTRL5"));
    SpinCtrl3->SetValue(_T("5"));
    FlexGridSizer18->Add(SpinCtrl3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText4 = new wxStaticText(Panel5, ID_STATICTEXT4, _("In Y:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT4"));
    FlexGridSizer18->Add(StaticText4, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    SpinCtrl4 = new wxSpinCtrl(Panel5, ID_SPINCTRL6, _T("8"), wxDefaultPosition, wxSize(50,-1), 0, 1, 200, 8, _T("ID_SPINCTRL6"));
    SpinCtrl4->SetValue(_T("8"));
    FlexGridSizer18->Add(SpinCtrl4, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer9->Add(FlexGridSizer18, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer16->Add(StaticBoxSizer9, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer10 = new wxStaticBoxSizer(wxHORIZONTAL, Panel5, _(" Size of quads (in mm): "));
    FlexGridSizer19 = new wxFlexGridSizer(1, 4, 0, 0);
    StaticText16 = new wxStaticText(Panel5, ID_STATICTEXT17, _("In X:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT17"));
    FlexGridSizer19->Add(StaticText16, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    TextCtrl5 = new wxTextCtrl(Panel5, ID_TEXTCTRL1, _("40.0"), wxDefaultPosition, wxSize(40,-1), 0, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    FlexGridSizer19->Add(TextCtrl5, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText17 = new wxStaticText(Panel5, ID_STATICTEXT18, _("In Y:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT18"));
    FlexGridSizer19->Add(StaticText17, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    TextCtrl6 = new wxTextCtrl(Panel5, ID_TEXTCTRL3, _("40.0"), wxDefaultPosition, wxSize(40,-1), 0, wxDefaultValidator, _T("ID_TEXTCTRL3"));
    FlexGridSizer19->Add(TextCtrl6, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer10->Add(FlexGridSizer19, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer16->Add(StaticBoxSizer10, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    CheckBox2 = new wxCheckBox(Panel5, ID_CHECKBOX3, _("Normalize image"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX3"));
    CheckBox2->SetValue(true);
    FlexGridSizer16->Add(CheckBox2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer5->Add(FlexGridSizer16, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 3);
    FlexGridSizer15->Add(StaticBoxSizer5, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel5->SetSizer(FlexGridSizer15);
    FlexGridSizer15->Fit(Panel5);
    FlexGridSizer15->SetSizeHints(Panel5);
    Notebook1->AddPage(Panel1, _("0) Instructions"), false);
    Notebook1->AddPage(Panel3, _("1) Test connection"), false);
    Notebook1->AddPage(Panel4, _("2) Capture"), false);
    Notebook1->AddPage(Panel6, _("3) Stereo/RGBD calibration"), false);
    Notebook1->AddPage(Panel5, _("4) Live Kinect test"), false);
    FlexGridSizer1->Add(Notebook1, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel2 = new wxPanel(this, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
    FlexGridSizer2 = new wxFlexGridSizer(0, 2, 0, 0);
    FlexGridSizer2->AddGrowableCol(0);
    FlexGridSizer13 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer13->AddGrowableCol(1);
    FlexGridSizer13->AddGrowableRow(0);
    StaticText10 = new wxStaticText(Panel2, ID_STATICTEXT10, _("Log:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT10"));
    FlexGridSizer13->Add(StaticText10, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edLogTest = new wxTextCtrl(Panel2, ID_TEXTCTRL4, wxEmptyString, wxDefaultPosition, wxSize(700,63), wxTE_AUTO_SCROLL|wxTE_MULTILINE|wxTE_READONLY|wxHSCROLL|wxVSCROLL, wxDefaultValidator, _T("ID_TEXTCTRL4"));
    wxFont edLogTestFont(8,wxTELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    edLogTest->SetFont(edLogTestFont);
    FlexGridSizer13->Add(edLogTest, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer2->Add(FlexGridSizer13, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer14 = new wxFlexGridSizer(2, 1, 0, 0);
    bntAbout = new wxButton(Panel2, ID_BUTTON1, _("About"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
    FlexGridSizer14->Add(bntAbout, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnQuit = new wxButton(Panel2, ID_BUTTON2, _("Quit"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
    FlexGridSizer14->Add(btnQuit, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer2->Add(FlexGridSizer14, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
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
    Connect(ID_BUTTON8,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnSkipToCalibClick);
    Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnConnectClick);
    Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnNext1Click);
    Connect(ID_RADIOBOX1,wxEVT_COMMAND_RADIOBOX_SELECTED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnrbChannelSwitchSelect);
    Connect(ID_SPINCTRL7,wxEVT_COMMAND_SPINCTRL_UPDATED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnedTiltChange);
    Connect(ID_BUTTON6,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnCaptureClick);
    Connect(ID_BUTTON7,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnNextCalibClick);
    Connect(ID_CHECKLISTBOX1,wxEVT_COMMAND_LISTBOX_SELECTED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnlbImagePairsSelect);
    Connect(ID_BUTTON9,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnListMarkAllClick);
    Connect(ID_BUTTON11,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnListMarkNoneClick);
    Connect(ID_BUTTON14,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnRunCalibClick);
    Connect(ID_BUTTON13,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnSaveCalibClick);
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
		0, // bufferSize
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
		m_imgStaticKinect->Refresh(false);
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
	if (event.GetSelection()==0)
	{
		if (event.GetOldSelection()==1 || event.GetOldSelection()==2)
		{
			this->StopLiveGrabThreads();
			event.Skip(); // Permit change
		}
		else
		{
			if (wxYES==::wxMessageBox(_("Your unsaved data will be lost, are you sure you want to leave this page?"),_("Confirm"),wxYES_NO, this))
				event.Skip(); // Permit change
			else
				event.Veto(); // Veto change
		}
	}
	else
	{
		event.Veto(); // Veto change
	}
}

void kinect_calibrate_guiDialog::OnbtnNext1Click(wxCommandEvent& event)
{
	if (Notebook1->GetSelection()==0)
	{
		btnConnect->Enable(true);
		btnNext2->Enable(false);
	}

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

	m_grabstate = gsIdle;
	m_calib_images.clear();

	m_cap_thread_data.quit = false;
	m_findcorners_thread_data.quit = false;

	// Launch thread:
	m_cap_thread         = mrpt::system::createThreadFromObjectMethod(this, &kinect_calibrate_guiDialog::thread_grabbing);
	m_findcorners_thread = mrpt::system::createThreadFromObjectMethod(this, &kinect_calibrate_guiDialog::thread_find_corners);
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

		// We only have to grab the intensity channel:
		kinect.enableGrabRGB(true); // RGB / IR channels:
		kinect.enableGrab3DPoints(false);
		kinect.enableGrabDepth(false);
		kinect.enableGrabAccelerometers(false);

		// Open:
		kinect.setVideoChannel( p.select_IR_channel ? CKinect::VIDEO_CHANNEL_IR : CKinect::VIDEO_CHANNEL_RGB);
		bool last_select_IR_channel = p.select_IR_channel;

		cout << "Calling CKinect::initialize()...";
		kinect.initialize();
		cout << "OK\n";

		CTicTac tictac;
		int nImgs = 0;
		bool there_is_obs=true, hard_error=false;

		kinect.setTiltAngleDegrees(p.tilt_ang_deg);
		double old_tilt_ang_deg = p.tilt_ang_deg;

		while (!hard_error && !p.quit)
		{
			// Grab new observation from the camera:
			CObservation3DRangeScanPtr  obs     = CObservation3DRangeScan::Create(); // Smart pointers to observations

			kinect.getNextObservation(*obs,there_is_obs,hard_error);

			if (!hard_error && there_is_obs)
			{
				p.new_obs.set(obs);
			}

			if (old_tilt_ang_deg != p.tilt_ang_deg)
			{
				old_tilt_ang_deg = p.tilt_ang_deg;
				kinect.setTiltAngleDegrees(p.tilt_ang_deg);
			}

			// Shall we switch between IR/RGB channels?
			if (last_select_IR_channel != p.select_IR_channel)
			{
				kinect.setVideoChannel( p.select_IR_channel ? CKinect::VIDEO_CHANNEL_IR : CKinect::VIDEO_CHANNEL_RGB);
				last_select_IR_channel = p.select_IR_channel;
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
		cout << "[Kinect thread] Exception: " << e.what() << endl;
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

void kinect_calibrate_guiDialog::StopLiveGrabThreads()
{
	wxBusyInfo info(_("Waiting for grab threads to close..."),this);
	wxTheApp->Yield();

	if (!m_cap_thread.isClear() && !m_cap_thread_data.terminated)
	{
		m_cap_thread_data.quit = true;
		cout << "Waiting for the grabbing thread to end...\n";
		for (int i=0;i<1000 && !m_cap_thread_data.terminated;i++) mrpt::system::sleep(1);
		mrpt::system::terminateThread( m_cap_thread );
		m_cap_thread.clear();
		cout << "Grabbing thread closed.\n";
	}
	if (!m_findcorners_thread.isClear() && !m_findcorners_thread_data.terminated)
	{
		m_findcorners_thread_data.quit = true;
		cout << "Waiting for the corner find thread to end...\n";
		for (int i=0;i<1000 && !m_findcorners_thread_data.terminated;i++) mrpt::system::sleep(1);
		mrpt::system::terminateThread( m_findcorners_thread );
		m_findcorners_thread.clear();
		cout << "Corner finding thread closed.\n";
	}
}


void kinect_calibrate_guiDialog::OnClose(wxCloseEvent& event)
{
	StopLiveGrabThreads();
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

	// ------------------------------------------
	//   Tab 2: Capturing
	// ------------------------------------------
	case 2:
		{
			// Capture stuff --------------------
			static int cnt_skip_frames = 0;
			vector<string> center_messages;

			if (m_grabstate!=gsIdle)
				center_messages.push_back(string("* DON'T MOVE EITHER THE SENSOR OR THE CHESSBOARD *"));

			switch (m_grabstate)
			{
			case gsIdle:
				cnt_skip_frames = 0;
				break;

			case gsSwitchingRGB:
				center_messages.push_back(string(" Switching to RGB channel..."));
				m_cap_thread_data.select_IR_channel = false;
				if (m_last_obs->intensityImageChannel == CObservation3DRangeScan::CH_VISIBLE) {
					if (cnt_skip_frames++>40) {
						cnt_skip_frames = 0;
						m_grabstate = gsCapturingRGB;
					}
				}
				break;

			case gsCapturingRGB:
				{
				// Grab RGB (left) image:
				m_calib_images.resize(m_calib_images.size()+1);

				mrpt::vision::TImageStereoCalibData &scd = *m_calib_images.rbegin();
				scd.left.img_original = m_last_obs->intensityImage;

				// Switch to IR:
				m_grabstate = gsSwitchingIR;
				}
				break;

			case gsSwitchingIR:
				center_messages.push_back(string(" Switching to IR channel..."));
				m_cap_thread_data.select_IR_channel = true;
				if (m_last_obs->intensityImageChannel == CObservation3DRangeScan::CH_IR) {
					if (cnt_skip_frames++>40) {
						cnt_skip_frames = 0;
						m_grabstate = gsCapturingIR;
					}
				}
				break;

			case gsCapturingIR:
				{
				// Grab IR (right) image:
				mrpt::vision::TImageStereoCalibData &scd = *m_calib_images.rbegin();
				scd.right.img_original = m_last_obs->intensityImage;

				// Done capturing: Back to idle:
				m_grabstate = gsIdle;
				lbNumCaptured->SetLabel( wxString::Format(_("Captured image pairs: %u"),static_cast<unsigned int>(m_calib_images.size()) ) );
				btnCapture->Enable();
				btnNextCalib->Enable();

				// Switch back to user-selected channel:
				m_cap_thread_data.select_IR_channel = (rbChannelSwitch->GetSelection() == 1);
				}
				break;
			};

			// Display stuff --------------------
			CImage img_to_show;
			img_to_show.copyFastFrom(m_last_obs->intensityImage); // (Destroys org img)

			if (!img_to_show.isColor() && cbNormalize->IsChecked())
				img_to_show.normalize();

			// Do live detect & draw chessboard in parallel:
			if (m_findcorners_thread_data.ready_for_new_images)
			{
				m_findcorners_thread_data.image = img_to_show;
				m_findcorners_thread_data.image_timestamp = m_last_obs->timestamp;
			}

			static std::vector<TPixelCoordf>  last_valid_corners;
			static mrpt::system::TTimeStamp   last_valid_corners_tim = mrpt::system::now();

			if (m_findcorners_thread_data.detected_corners_done)
			{
				// We have new corners to draw:
				if (! m_findcorners_thread_data.detected_corners.empty() )
				{
					last_valid_corners_tim = mrpt::system::now();
					last_valid_corners = m_findcorners_thread_data.detected_corners;
				}
				m_findcorners_thread_data.detected_corners_done = false; // Signal that we've read the data.
			}

			// Makes an RGB color even if it was grayscale so we can draw color stuff:
			img_to_show.colorImageInPlace();

			// Draw detected corners "persistently" during some instants:
			static bool at_least_detected_once = false;
			if (mrpt::system::timeDifference(last_valid_corners_tim, mrpt::system::now()) < 0.5 )
			{
				const unsigned int cx = edSizeX->GetValue(), cy = edSizeY->GetValue();
				img_to_show.drawChessboardCorners( last_valid_corners, cx,cy );

				if (!last_valid_corners.empty()) at_least_detected_once = true;
			}
			else
			{
				center_messages.push_back("*WARNING*: No chessboard detected!");
				if (!at_least_detected_once) center_messages.push_back("Make sure the Rows x Cols size is correct.");
			}

			// Messages:
			img_to_show.selectTextFont("10x20");
			for (size_t i=0;i<center_messages.size();i++)
			{
				const string &s = center_messages[i];
				const int x = img_to_show.getWidth()/2 - 10*s.size()/2;
				const int y = 230 + 25*i;
				img_to_show.textOut(x,y,s,TColor::black);
				img_to_show.textOut(x-1,y-1,s,TColor::white);
			}

			// Show:
			m_realtimeview_cap->AssignImage( img_to_show );
			m_realtimeview_cap->Refresh(false);
		}
		break;
	}

}

// User clicks to switch channels:
void kinect_calibrate_guiDialog::OnrbChannelSwitchSelect(wxCommandEvent& event)
{
	// The actual change of channel is done in the grabber thread.
	m_cap_thread_data.select_IR_channel = (rbChannelSwitch->GetSelection() == 1);
}

// The thread in charge of finding corners in parallel to the GUI
void kinect_calibrate_guiDialog::thread_find_corners()
{
	TThreadDetectCornerParam  &p = this->m_findcorners_thread_data;
	p.terminated = false;
	p.ready_for_new_images = true;

	mrpt::system::TTimeStamp last_img_proc = INVALID_TIMESTAMP;

	while (!p.quit)
	{
		try
		{
			// New image to process??
			p.ready_for_new_images=true;

			if (!p.detected_corners_done && last_img_proc!=p.image_timestamp && p.image_timestamp!=INVALID_TIMESTAMP)
			{
				// Yes:
				p.ready_for_new_images=false;

				const unsigned int cx = edSizeX->GetValue(), cy = edSizeY->GetValue();

				const CImage img_gray( p.image, FAST_REF_OR_CONVERT_TO_GRAY );
				bool detect_ok = mrpt::vision::findChessboardCorners(
					img_gray,
					p.detected_corners,
					cx,cy,
					cbNormalize->IsChecked() // normalize_image
					);

				if (!detect_ok) p.detected_corners.clear();

				p.detected_corners_done = true; // will be set to false when the buffer "p.detected_corners" has been read.
				p.ready_for_new_images=true;
			}
			else
			{
				mrpt::system::sleep(2);
			}
		}
		catch(std::exception &e)
		{
			cout << "[corner finding thread] Exception: " << e.what() << endl;
		}
	}
	p.terminated = true;
}

// Button: Capture one image pair.
void kinect_calibrate_guiDialog::OnbtnCaptureClick(wxCommandEvent& event)
{
	btnCapture->Disable();
	btnNextCalib->Disable();
	m_grabstate = gsSwitchingRGB;
}


void kinect_calibrate_guiDialog::OnbtnNextCalibClick(wxCommandEvent& event)
{
	// Stop grabbing:
	StopLiveGrabThreads();
	Notebook1->ChangeSelection( Notebook1->GetSelection() + 1 );

	UpdateListOfImages();
}

void kinect_calibrate_guiDialog::OnbtnSkipToCalibClick(wxCommandEvent& event)
{
	Notebook1->ChangeSelection(3);
}

void kinect_calibrate_guiDialog::OnedTiltChange(wxSpinEvent& event)
{
	m_cap_thread_data.tilt_ang_deg = edTilt->GetValue();
}

void kinect_calibrate_guiDialog::OnbtnListMarkAllClick(wxCommandEvent& event)
{
	const unsigned int N = lbImagePairs->GetCount();
	for (unsigned int i=0;i<N;i++)
		lbImagePairs->Check(i, true);
}

void kinect_calibrate_guiDialog::OnbtnListMarkNoneClick(wxCommandEvent& event)
{
	const unsigned int N = lbImagePairs->GetCount();
	for (unsigned int i=0;i<N;i++)
		lbImagePairs->Check(i, false);
}

void kinect_calibrate_guiDialog::UpdateListOfImages()
{
	wxArrayString  lstStrs;
	for (size_t i=0;i<m_calib_images.size();i++)
	{
		lstStrs.Add( wxString::Format(_("ImgPair #%u"),static_cast<unsigned int>(i) ) );
	}
	lbImagePairs->Set(lstStrs);

	wxCommandEvent ev;
	this->OnbtnListMarkAllClick(ev);

	lbImagePairs->SetSelection(0);
	ProcessNewSelectedImageListBox();
}

void kinect_calibrate_guiDialog::OnlbImagePairsSelect(wxCommandEvent& event)
{
	this->ProcessNewSelectedImageListBox();
}
void kinect_calibrate_guiDialog::ProcessNewSelectedImageListBox()
{
	try
	{
		const int sel = lbImagePairs->GetSelection();

		if (sel==wxNOT_FOUND || sel>=(int)m_calib_images.size() )
		{
			mrpt::utils::CImage img(320,240, CH_RGB);
			img.filledRectangle(0,0,319,239, TColor(200,200,200) );
			img.textOut(100,110, "(No image selected)", TColor::white);

			this->m_view_left->AssignImage(img);
			this->m_view_right->AssignImage(img);
		}
		else
		{
			CImage il = m_calib_images[sel].left.img_original.scaleHalf();
			CImage ir = m_calib_images[sel].right.img_original.scaleHalf();

			if (cbCalibNormalize->IsChecked())
			{
				if (!il.isColor()) il.normalize();
				if (!ir.isColor()) ir.normalize();
			}

			this->m_view_left->AssignImage ( il );
			this->m_view_right->AssignImage( ir );
		}

		this->m_view_left->Refresh();
		this->m_view_right->Refresh();
	}
	catch(std::exception &e)
	{
		wxMessageBox( wxString::Format(_("[Draw selected image] Exception:\n %s"), e.what() ) );
	}
}

// button: Run optimizer
void kinect_calibrate_guiDialog::OnbtnRunCalibClick(wxCommandEvent& event)
{
	try
	{
		// ============ Set parameters ============
		m_calib_params.check_size_x = edCalibCheckX->GetValue();
		m_calib_params.check_size_y = edCalibCheckY->GetValue();

		double sx,sy;
		edCalibSizeX->GetValue().ToDouble( &sx );
		edCalibSizeY->GetValue().ToDouble( &sy );

		m_calib_params.check_squares_length_X_meters = 1e-3*sx;
		m_calib_params.check_squares_length_Y_meters = 1e-3*sy;
		m_calib_params.maxIters = edMaxIters->GetValue();
		m_calib_params.verbose = true;

		m_calib_params.optimize_k1 = cbOptK1->IsChecked();
		m_calib_params.optimize_k2 = cbOptK2->IsChecked();
		m_calib_params.optimize_k3 = cbOptK3->IsChecked();

		m_calib_params.optimize_t1 = cbOptT1->IsChecked();
		m_calib_params.optimize_t2 = cbOptT2->IsChecked();

		m_calib_params.normalize_image = cbCalibNormalize->IsChecked();

		edLogCalibResult->Clear();

		// Run calibration:
		bool res;
		{
			wxBusyInfo info(_("Running optimizer..."),this);
			wxTheApp->Yield();

			res = mrpt::vision::checkerBoardStereoCalibration( m_calib_images, m_calib_params, m_calib_result );
		}

		if (!res)
		{
			edLogCalibResult->AppendText(_("\nCalibration returned an error status.\n"));
		}
		else
		{ // std::cout redir:
			CMyRedirector redir(edLogCalibResult, true, 400 );

			time_t systime;
			time(&systime);
			struct tm * timeinfo=localtime(&systime);

			cout << mrpt::format(
				"# Stereo camera calibration report\n"
				"# Generated by kinect-calibrate - MRPT at %s"
				"# (This file is loadable from rawlog-edit and other MRPT tools)\n"
				"# ---------------------------------------------------------------------\n\n",
				asctime(timeinfo)
				);

			const TCamera &lc = m_calib_result.cam_params.leftCamera;
			const TCamera &rc = m_calib_result.cam_params.rightCamera;

			// Inverse variance of the estimates, in this order:
			//  [fx fy cx cy k1 k2 k3 t1 t2].
			const Eigen::Array<double,9,1> & lc_inf = m_calib_result.left_params_inv_variance;
			const Eigen::Array<double,9,1> & rc_inf = m_calib_result.right_params_inv_variance;
			const double std_detector = 0.2; // pixels

			cout << mrpt::format(
				"# Left camera (RGB in Kinect) calibration parameters (and 95%% confidence intervals):\n"
				"[CAMERA_PARAMS_LEFT]\n"
				"resolution = [%u %u]\n"
				"cx         = %f  // +/- %.03f\n"
				"cy         = %f  // +/- %.03f\n"
				"fx         = %f  // +/- %.03f\n"
				"fy         = %f  // +/- %.03f\n"
				"dist       = [%e %e %e %e %e]    // The order is: [K1 K2 T1 T2 K3]\n\n",
					lc.ncols, lc.nrows,
					lc.cx(), 2.0*std_detector/std::sqrt(lc_inf[2]),
					lc.cy(), 2.0*std_detector/std::sqrt(lc_inf[3]),
					lc.fx(), 2.0*std_detector/std::sqrt(lc_inf[0]),
					lc.fy(), 2.0*std_detector/std::sqrt(lc_inf[1]),
					lc.k1(), lc.k2(), lc.p1(), lc.p2(), lc.k3() );

			cout << mrpt::format(
				"# Right camera (Depth in Kinect) calibration parameters  (and 95%% confidence intervals):\n"
				"[CAMERA_PARAMS_RIGHT]\n"
				"resolution = [%u %u]\n"
				"cx         = %f  // +/- %.03f\n"
				"cy         = %f  // +/- %.03f\n"
				"fx         = %f  // +/- %.03f\n"
				"fy         = %f  // +/- %.03f\n"
				"dist       = [%e %e %e %e %e]    // The order is: [K1 K2 T1 T2 K3]\n\n",
					rc.ncols, rc.nrows,
					rc.cx(), 2.0*std_detector/std::sqrt(rc_inf[2]),
					rc.cy(), 2.0*std_detector/std::sqrt(rc_inf[3]),
					rc.fx(), 2.0*std_detector/std::sqrt(rc_inf[0]),
					rc.fy(), 2.0*std_detector/std::sqrt(rc_inf[1]),
					rc.k1(), rc.k2(), rc.p1(), rc.p2(), rc.k3() );

			// pose:
			const mrpt::poses::CPose3D     RT_YPR(- m_calib_result.right2left_camera_pose );
			const mrpt::poses::CPose3DQuat RT_quat(RT_YPR);

			cout << mrpt::format(
				"# Relative pose of the right camera wrt to the left camera:\n"
				"[CAMERA_PARAMS_LEFT2RIGHT_POSE]\n"
				"translation_only     = [%e %e %e]\n"
				"rotation_matrix_only = %s\n"
				"pose_yaw_pitch_roll  = %s\n"
				"pose_quaternion      = %s\n\n"
				,
					RT_YPR.x(),RT_YPR.y(),RT_YPR.z(),
					RT_YPR.getRotationMatrix().inMatlabFormat(13).c_str(),
					RT_YPR.asString().c_str(),
					RT_quat.asString().c_str()
				);

			cout << mrpt::format(
				"# Info about calibration parameters:\n"
				"[CALIB_METAINFO]\n"
				"number_good_cheesboards = %u\n"
				"average_reprojection_error = %f // RMSE pixels\n"
				"cheesboard_nx = %i\n"
				"cheesboard_ny = %i\n"
				"cheesboard_square_size_x = %f // (m)\n"
				"cheesboard_square_size_y = %f // (m)\n\n"
				,
				static_cast<unsigned int>(m_calib_result.final_number_good_image_pairs),
				m_calib_result.final_rmse,
				m_calib_params.check_size_x,m_calib_params.check_size_y,
				m_calib_params.check_squares_length_X_meters,
				m_calib_params.check_squares_length_Y_meters
				);

		} // end of std::cout redir

		wxMessageBox( wxString::Format(_("Optimization finished with RMSE=%.03fpx"),m_calib_result.final_rmse), _("Done"), 0, this );

	}
	catch(std::exception &e)
	{
		wxMessageBox( wxString::Format(_("[Optimizer] Exception:\n %s"), e.what() ) );
	}
}

void kinect_calibrate_guiDialog::OnbtnSaveCalibClick(wxCommandEvent& event)
{
}
