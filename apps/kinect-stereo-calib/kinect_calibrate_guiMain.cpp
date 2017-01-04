/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*
  App      : kinect-stereo-calib
  Web pages: http://www.mrpt.org/Application:kinect-stereo-calib
             http://www.mrpt.org/Kinect_and_MRPT

  Usage    : Run and follow on-screen instructions
*/

#include "kinect_calibrate_guiMain.h"
#include <wx/msgdlg.h>
#include <wx/progdlg.h>
#include "CAboutBox.h"

//(*InternalHeaders(kinect_calibrate_guiDialog)
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/intl.h>
#include <wx/font.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/artprov.h>
//*)

#include <mrpt/hwdrivers/CKinect.h>
#include <mrpt/vision/chessboard_find_corners.h>
#include <mrpt/vision/chessboard_stereo_camera_calib.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/gui/WxUtils.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
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

const wxString imageWildcards = _("Image files (*.png;*.jpg;...)|*.png;*.PNG;*.JPG;*.jpg;*.BMP;*.bmp;*.TIF;*.tif|All files (*.*)|*.*");


//(*IdInit(kinect_calibrate_guiDialog)
const long kinect_calibrate_guiDialog::ID_STATICTEXT27 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON15 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON16 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON17 = wxNewId();
const long kinect_calibrate_guiDialog::ID_PANEL8 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT28 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT29 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CUSTOM2 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT30 = wxNewId();
const long kinect_calibrate_guiDialog::ID_TEXTCTRL10 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT31 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON3 = wxNewId();
const long kinect_calibrate_guiDialog::ID_PANEL9 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CUSTOM3 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT11 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON5 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT9 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON4 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT5 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON8 = wxNewId();
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
const long kinect_calibrate_guiDialog::ID_LISTBOX1 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON9 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON10 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON11 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON12 = wxNewId();
const long kinect_calibrate_guiDialog::ID_RADIOBOX2 = wxNewId();
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
const long kinect_calibrate_guiDialog::ID_PANEL11 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CHECKBOX4 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CHECKBOX5 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CHECKBOX6 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CHECKBOX7 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CHECKBOX8 = wxNewId();
const long kinect_calibrate_guiDialog::ID_PANEL12 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT26 = wxNewId();
const long kinect_calibrate_guiDialog::ID_SPINCTRL8 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CHECKBOX3 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CHECKBOX2 = wxNewId();
const long kinect_calibrate_guiDialog::ID_PANEL13 = wxNewId();
const long kinect_calibrate_guiDialog::ID_NOTEBOOK3 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON14 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT24 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON13 = wxNewId();
const long kinect_calibrate_guiDialog::ID_TEXTCTRL8 = wxNewId();
const long kinect_calibrate_guiDialog::ID_PANEL1 = wxNewId();
const long kinect_calibrate_guiDialog::ID_CUSTOM6 = wxNewId();
const long kinect_calibrate_guiDialog::ID_PANEL10 = wxNewId();
const long kinect_calibrate_guiDialog::ID_NOTEBOOK2 = wxNewId();
const long kinect_calibrate_guiDialog::ID_PANEL6 = wxNewId();
const long kinect_calibrate_guiDialog::ID_XY_GLCANVAS = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT3 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON18 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON20 = wxNewId();
const long kinect_calibrate_guiDialog::ID_STATICTEXT4 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON19 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BUTTON21 = wxNewId();
const long kinect_calibrate_guiDialog::ID_BITMAPBUTTON1 = wxNewId();
const long kinect_calibrate_guiDialog::ID_GRID1 = wxNewId();
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
	: m_config(_("kinect-stereo-calib")), m_my_redirector(NULL)
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
    wxFlexGridSizer* FlexGridSizer1;
    wxFlexGridSizer* FlexGridSizer2;
    wxFlexGridSizer* FlexGridSizer25;
    wxFlexGridSizer* FlexGridSizer15;
    wxStaticBoxSizer* StaticBoxSizer7;
    wxFlexGridSizer* FlexGridSizer17;
    wxFlexGridSizer* FlexGridSizer29;
    wxFlexGridSizer* FlexGridSizer11;
    wxFlexGridSizer* FlexGridSizer19;
    wxFlexGridSizer* FlexGridSizer41;
    wxFlexGridSizer* FlexGridSizer40;
    wxFlexGridSizer* FlexGridSizer7;
    wxFlexGridSizer* FlexGridSizer4;
    wxFlexGridSizer* FlexGridSizer9;
    wxFlexGridSizer* FlexGridSizer26;
    wxFlexGridSizer* FlexGridSizer14;
    wxStaticBoxSizer* StaticBoxSizer3;
    wxFlexGridSizer* FlexGridSizer33;
    wxFlexGridSizer* FlexGridSizer6;
    wxFlexGridSizer* FlexGridSizer38;
    wxFlexGridSizer* FlexGridSizer27;
    wxFlexGridSizer* FlexGridSizer42;
    wxFlexGridSizer* FlexGridSizer37;
    wxFlexGridSizer* FlexGridSizer3;
    wxFlexGridSizer* FlexGridSizer22;
    wxStaticBoxSizer* StaticBoxSizer8;
    wxFlexGridSizer* FlexGridSizer31;
    wxFlexGridSizer* FlexGridSizer43;
    wxFlexGridSizer* FlexGridSizer39;
    wxStaticBoxSizer* StaticBoxSizer4;
    wxFlexGridSizer* FlexGridSizer16;
    wxFlexGridSizer* FlexGridSizer34;
    wxFlexGridSizer* FlexGridSizer23;
    wxFlexGridSizer* FlexGridSizer10;
    wxBoxSizer* BoxSizer1;
    wxFlexGridSizer* FlexGridSizer13;
    wxFlexGridSizer* FlexGridSizer18;
    wxFlexGridSizer* FlexGridSizer36;
    wxFlexGridSizer* FlexGridSizer35;
    wxFlexGridSizer* FlexGridSizer5;
    wxFlexGridSizer* FlexGridSizer24;
    wxFlexGridSizer* FlexGridSizer32;
    wxStaticBoxSizer* StaticBoxSizer1;
    wxFlexGridSizer* FlexGridSizer20;

    Create(parent, wxID_ANY, _("Kinect & stereo camera calibration wizard - Part of MRPT"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE|wxRESIZE_BORDER|wxMAXIMIZE_BOX|wxMINIMIZE_BOX, _T("wxID_ANY"));
    Move(wxPoint(-1,-1));
    FlexGridSizer1 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer1->AddGrowableCol(0);
    FlexGridSizer1->AddGrowableRow(0);
    Notebook1 = new wxNotebook(this, ID_NOTEBOOK1, wxDefaultPosition, wxDefaultSize, wxNB_BOTTOM, _T("ID_NOTEBOOK1"));
    Panel8 = new wxPanel(Notebook1, ID_PANEL8, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL8"));
    FlexGridSizer37 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer37->AddGrowableCol(0);
    FlexGridSizer37->AddGrowableRow(0);
    FlexGridSizer38 = new wxFlexGridSizer(3, 1, 0, 0);
    FlexGridSizer38->AddGrowableCol(0);
    StaticText26 = new wxStaticText(Panel8, ID_STATICTEXT27, _("What do you want to do\?"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT27"));
    wxFont StaticText26Font(16,wxDEFAULT,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    StaticText26->SetFont(StaticText26Font);
    FlexGridSizer38->Add(StaticText26, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer38->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer39 = new wxFlexGridSizer(3, 1, 0, 0);
    FlexGridSizer39->AddGrowableCol(0);
    btnOpCalibKinect = new wxButton(Panel8, ID_BUTTON15, _("Calibrate a Kinect sensor (live)"), wxDefaultPosition, wxSize(-1,50), 0, wxDefaultValidator, _T("ID_BUTTON15"));
    FlexGridSizer39->Add(btnOpCalibKinect, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnOpCalibStereoGeneric = new wxButton(Panel8, ID_BUTTON16, _("Calibrate a stereo/Kinect camera (from images in disk)"), wxDefaultPosition, wxSize(-1,50), 0, wxDefaultValidator, _T("ID_BUTTON16"));
    FlexGridSizer39->Add(btnOpCalibStereoGeneric, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnOpTestKinect = new wxButton(Panel8, ID_BUTTON17, _("Visually inspect the correctness of a Kinect calib. (live)"), wxDefaultPosition, wxSize(-1,50), 0, wxDefaultValidator, _T("ID_BUTTON17"));
    FlexGridSizer39->Add(btnOpTestKinect, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer38->Add(FlexGridSizer39, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer37->Add(FlexGridSizer38, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    Panel8->SetSizer(FlexGridSizer37);
    FlexGridSizer37->Fit(Panel8);
    FlexGridSizer37->SetSizeHints(Panel8);
    Panel9 = new wxPanel(Notebook1, ID_PANEL9, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL9"));
    FlexGridSizer40 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer40->AddGrowableCol(0);
    FlexGridSizer40->AddGrowableRow(0);
    FlexGridSizer41 = new wxFlexGridSizer(7, 1, 0, 0);
    StaticText27 = new wxStaticText(Panel9, ID_STATICTEXT28, _("Prepare to calibrate your Kinect sensor"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT28"));
    wxFont StaticText27Font(15,wxDEFAULT,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    StaticText27->SetFont(StaticText27Font);
    FlexGridSizer41->Add(StaticText27, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText28 = new wxStaticText(Panel9, ID_STATICTEXT29, _("1) Make sure the IR projector is covered, for example, by tapying a piece of some opaque material on it."), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT29"));
    FlexGridSizer41->Add(StaticText28, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    m_imgStaticKinect = new mrpt::gui::wxMRPTImageControl(Panel9,ID_CUSTOM2,wxDefaultPosition.x,wxDefaultPosition.y,wxSize(300,116).GetWidth(), wxSize(300,116).GetHeight() );
    FlexGridSizer41->Add(m_imgStaticKinect, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText29 = new wxStaticText(Panel9, ID_STATICTEXT30, _("2) Print a checkerboard and stick it firmly to some rigid board or paperboard.\n   If you don\'t have any pattern at hand, get one from:"), wxPoint(48,256), wxSize(661,37), 0, _T("ID_STATICTEXT30"));
    FlexGridSizer41->Add(StaticText29, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    TextCtrl3 = new wxTextCtrl(Panel9, ID_TEXTCTRL10, _("http://www.mrpt.org/downloads/camera-calibration-checker-board_9x7.pdf"), wxDefaultPosition, wxDefaultSize, wxTE_READONLY, wxDefaultValidator, _T("ID_TEXTCTRL10"));
    FlexGridSizer41->Add(TextCtrl3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    StaticText30 = new wxStaticText(Panel9, ID_STATICTEXT31, _("3) Connect the Kinect to the computer and wait a few seconds until the driver establishes connection."), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT31"));
    FlexGridSizer41->Add(StaticText30, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    btnNext1 = new wxButton(Panel9, ID_BUTTON3, _("START"), wxPoint(376,368), wxSize(117,45), 0, wxDefaultValidator, _T("ID_BUTTON3"));
    btnNext1->SetDefault();
    FlexGridSizer41->Add(btnNext1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer40->Add(FlexGridSizer41, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel9->SetSizer(FlexGridSizer40);
    FlexGridSizer40->Fit(Panel9);
    FlexGridSizer40->SetSizeHints(Panel9);
    Panel3 = new wxPanel(Notebook1, ID_PANEL3, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL3"));
    FlexGridSizer5 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer5->AddGrowableCol(1);
    FlexGridSizer5->AddGrowableRow(0);
    m_realtimeview_test = new mrpt::gui::wxMRPTImageControl(Panel3,ID_CUSTOM3,wxDefaultPosition.x,wxDefaultPosition.y,wxSize(640,480).GetWidth(), wxSize(640,480).GetHeight() );
    FlexGridSizer5->Add(m_realtimeview_test, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer7 = new wxFlexGridSizer(7, 1, 0, 0);
    FlexGridSizer7->AddGrowableCol(0);
    FlexGridSizer7->AddGrowableRow(6);
    StaticText11 = new wxStaticText(Panel3, ID_STATICTEXT11, _("Press \"Connect\" to start grabbing:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT11"));
    FlexGridSizer7->Add(StaticText11, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnConnect = new wxButton(Panel3, ID_BUTTON5, _("Connect..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON5"));
    FlexGridSizer7->Add(btnConnect, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    StaticText9 = new wxStaticText(Panel3, ID_STATICTEXT9, _("You should be seeing the \nRGB channel on the left panel.\n\nIf everything works OK, \npress \"Continue\"."), wxPoint(40,32), wxDefaultSize, 0, _T("ID_STATICTEXT9"));
    FlexGridSizer7->Add(StaticText9, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnNext2 = new wxButton(Panel3, ID_BUTTON4, _("CONTINUE..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON4"));
    btnNext2->SetDefault();
    btnNext2->Disable();
    FlexGridSizer7->Add(btnNext2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    StaticText5 = new wxStaticText(Panel3, ID_STATICTEXT5, _("(If no error was found but still \nhave no images, try disconnecting \nand connecting again:)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT5"));
    FlexGridSizer7->Add(StaticText5, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnDisconnect = new wxButton(Panel3, ID_BUTTON8, _("Disconnect"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON8"));
    btnDisconnect->Disable();
    FlexGridSizer7->Add(btnDisconnect, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer5->Add(FlexGridSizer7, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
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
    BoxSizer1->Add(TextCtrl2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    StaticBoxSizer1->Add(BoxSizer1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer4->Add(StaticBoxSizer1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
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
    StaticBoxSizer4->Add(FlexGridSizer17, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer6->Add(StaticBoxSizer4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    StaticBoxSizer3->Add(FlexGridSizer6, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 3);
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
    FlexGridSizer8->Add(rbChannelSwitch, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
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
    FlexGridSizer25->Add(edTilt, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer8->Add(FlexGridSizer25, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    btnCapture = new wxButton(Panel7, ID_BUTTON6, _("CAPTURE NOW"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON6"));
    wxFont btnCaptureFont(10,wxSWISS,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    btnCapture->SetFont(btnCaptureFont);
    FlexGridSizer8->Add(btnCapture, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    lbNumCaptured = new wxStaticText(Panel7, ID_STATICTEXT12, _("Captured image pairs: 0"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT12"));
    FlexGridSizer8->Add(lbNumCaptured, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnNextCalib = new wxButton(Panel7, ID_BUTTON7, _("OK, go to calibrate >>"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON7"));
    btnNextCalib->Disable();
    wxFont btnNextCalibFont(10,wxSWISS,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    btnNextCalib->SetFont(btnNextCalibFont);
    FlexGridSizer8->Add(btnNextCalib, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    StaticBoxSizer2->Add(FlexGridSizer8, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 3);
    FlexGridSizer4->Add(StaticBoxSizer2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    Panel7->SetSizer(FlexGridSizer4);
    FlexGridSizer4->Fit(Panel7);
    FlexGridSizer4->SetSizeHints(Panel7);
    FlexGridSizer3->Add(Panel7, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    Panel4->SetSizer(FlexGridSizer3);
    FlexGridSizer3->Fit(Panel4);
    FlexGridSizer3->SetSizeHints(Panel4);
    Panel6 = new wxPanel(Notebook1, ID_PANEL6, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL6"));
    FlexGridSizer9 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer9->AddGrowableCol(0);
    FlexGridSizer9->AddGrowableRow(0);
    FlexGridSizer9->AddGrowableRow(1);
    FlexGridSizer10 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer10->AddGrowableCol(0);
    FlexGridSizer10->AddGrowableCol(1);
    FlexGridSizer10->AddGrowableRow(0);
    FlexGridSizer22 = new wxFlexGridSizer(4, 1, 0, 0);
    FlexGridSizer22->AddGrowableCol(0);
    FlexGridSizer22->AddGrowableRow(1);
    StaticText22 = new wxStaticText(Panel6, ID_STATICTEXT23, _("List of stereo images:"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT23"));
    wxFont StaticText22Font(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    StaticText22->SetFont(StaticText22Font);
    FlexGridSizer22->Add(StaticText22, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 2);
    lbImagePairs = new wxListBox(Panel6, ID_LISTBOX1, wxDefaultPosition, wxDefaultSize, 0, 0, wxLB_SINGLE, wxDefaultValidator, _T("ID_LISTBOX1"));
    FlexGridSizer22->Add(lbImagePairs, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer26 = new wxFlexGridSizer(3, 2, 0, 0);
    FlexGridSizer26->AddGrowableCol(0);
    FlexGridSizer26->AddGrowableCol(1);
    btnListRemoveSelected = new wxButton(Panel6, ID_BUTTON9, _("Remove selected"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON9"));
    wxFont btnListRemoveSelectedFont(8,wxDEFAULT,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    btnListRemoveSelected->SetFont(btnListRemoveSelectedFont);
    FlexGridSizer26->Add(btnListRemoveSelected, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnListLoad = new wxButton(Panel6, ID_BUTTON10, _("Load image pair..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON10"));
    wxFont btnListLoadFont(8,wxDEFAULT,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    btnListLoad->SetFont(btnListLoadFont);
    FlexGridSizer26->Add(btnListLoad, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnLoadImageList = new wxButton(Panel6, ID_BUTTON11, _("Load list of images..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON11"));
    wxFont btnLoadImageListFont(8,wxDEFAULT,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    btnLoadImageList->SetFont(btnLoadImageListFont);
    FlexGridSizer26->Add(btnLoadImageList, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnListSave = new wxButton(Panel6, ID_BUTTON12, _("Save all images..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON12"));
    wxFont btnListSaveFont(8,wxDEFAULT,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    btnListSave->SetFont(btnListSaveFont);
    FlexGridSizer26->Add(btnListSave, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer22->Add(FlexGridSizer26, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer10->Add(FlexGridSizer22, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer32 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer32->AddGrowableCol(0);
    FlexGridSizer32->AddGrowableRow(1);
    wxString __wxRadioBoxChoices_2[4] =
    {
    _("Original"),
    _("Detected chessboard"),
    _("Reprojected corners"),
    _("Undistorted")
    };
    rbShowImages = new wxRadioBox(Panel6, ID_RADIOBOX2, _(" Show: "), wxDefaultPosition, wxDefaultSize, 4, __wxRadioBoxChoices_2, 1, 0, wxDefaultValidator, _T("ID_RADIOBOX2"));
    rbShowImages->SetSelection(0);
    wxFont rbShowImagesFont(8,wxDEFAULT,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    rbShowImages->SetFont(rbShowImagesFont);
    FlexGridSizer32->Add(rbShowImages, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer42 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer42->AddGrowableCol(0);
    FlexGridSizer42->AddGrowableCol(1);
    FlexGridSizer42->AddGrowableRow(0);
    FlexGridSizer23 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer23->AddGrowableCol(0);
    FlexGridSizer23->AddGrowableRow(1);
    StaticText18 = new wxStaticText(Panel6, ID_STATICTEXT19, _("(Left camera)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT19"));
    FlexGridSizer23->Add(StaticText18, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
    m_view_left = new mrpt::gui::wxMRPTImageControl(Panel6,ID_CUSTOM4,wxDefaultPosition.x,wxDefaultPosition.y,wxSize(320,240).GetWidth(), wxSize(320,240).GetHeight() );
    FlexGridSizer23->Add(m_view_left, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer42->Add(FlexGridSizer23, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    FlexGridSizer24 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer24->AddGrowableCol(0);
    FlexGridSizer24->AddGrowableRow(1);
    StaticText19 = new wxStaticText(Panel6, ID_STATICTEXT20, _("(Right camera)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT20"));
    FlexGridSizer24->Add(StaticText19, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
    m_view_right = new mrpt::gui::wxMRPTImageControl(Panel6,ID_CUSTOM5,wxDefaultPosition.x,wxDefaultPosition.y,wxSize(320,240).GetWidth(), wxSize(320,240).GetHeight() );
    FlexGridSizer24->Add(m_view_right, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer42->Add(FlexGridSizer24, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    FlexGridSizer32->Add(FlexGridSizer42, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer10->Add(FlexGridSizer32, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer9->Add(FlexGridSizer10, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer11 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer11->AddGrowableCol(1);
    FlexGridSizer11->AddGrowableRow(0);
    FlexGridSizer29 = new wxFlexGridSizer(3, 1, 0, 0);
    FlexGridSizer29->AddGrowableCol(0);
    FlexGridSizer29->AddGrowableRow(1);
    FlexGridSizer30 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer30->AddGrowableCol(0);
    StaticText24 = new wxStaticText(Panel6, ID_STATICTEXT25, _("Calibration options"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT25"));
    wxFont StaticText24Font(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    StaticText24->SetFont(StaticText24Font);
    FlexGridSizer30->Add(StaticText24, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer29->Add(FlexGridSizer30, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Notebook2 = new wxNotebook(Panel6, ID_NOTEBOOK3, wxDefaultPosition, wxDefaultSize, 0, _T("ID_NOTEBOOK3"));
    Panel11 = new wxPanel(Notebook2, ID_PANEL11, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL11"));
    FlexGridSizer20 = new wxFlexGridSizer(2, 1, 0, 0);
    StaticBoxSizer7 = new wxStaticBoxSizer(wxHORIZONTAL, Panel11, _("Number of inner corners: "));
    FlexGridSizer21 = new wxFlexGridSizer(1, 4, 0, 0);
    StaticText12 = new wxStaticText(Panel11, ID_STATICTEXT13, _("In X:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT13"));
    FlexGridSizer21->Add(StaticText12, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edCalibCheckX = new wxSpinCtrl(Panel11, ID_SPINCTRL3, _T("7"), wxDefaultPosition, wxSize(50,-1), 0, 1, 200, 7, _T("ID_SPINCTRL3"));
    edCalibCheckX->SetValue(_T("7"));
    FlexGridSizer21->Add(edCalibCheckX, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText13 = new wxStaticText(Panel11, ID_STATICTEXT14, _("In Y:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT14"));
    FlexGridSizer21->Add(StaticText13, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edCalibCheckY = new wxSpinCtrl(Panel11, ID_SPINCTRL4, _T("9"), wxDefaultPosition, wxSize(50,-1), 0, 1, 200, 9, _T("ID_SPINCTRL4"));
    edCalibCheckY->SetValue(_T("9"));
    FlexGridSizer21->Add(edCalibCheckY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer7->Add(FlexGridSizer21, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer20->Add(StaticBoxSizer7, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    StaticBoxSizer8 = new wxStaticBoxSizer(wxHORIZONTAL, Panel11, _(" Size of quads (in mm): "));
    FlexGridSizer33 = new wxFlexGridSizer(1, 4, 0, 0);
    StaticText14 = new wxStaticText(Panel11, ID_STATICTEXT15, _("In X:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT15"));
    FlexGridSizer33->Add(StaticText14, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edCalibSizeX = new wxTextCtrl(Panel11, ID_TEXTCTRL6, _("40.0"), wxDefaultPosition, wxSize(40,-1), 0, wxDefaultValidator, _T("ID_TEXTCTRL6"));
    FlexGridSizer33->Add(edCalibSizeX, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText15 = new wxStaticText(Panel11, ID_STATICTEXT16, _("In Y:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT16"));
    FlexGridSizer33->Add(StaticText15, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edCalibSizeY = new wxTextCtrl(Panel11, ID_TEXTCTRL7, _("40.0"), wxDefaultPosition, wxSize(40,-1), 0, wxDefaultValidator, _T("ID_TEXTCTRL7"));
    FlexGridSizer33->Add(edCalibSizeY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer8->Add(FlexGridSizer33, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer20->Add(StaticBoxSizer8, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel11->SetSizer(FlexGridSizer20);
    FlexGridSizer20->Fit(Panel11);
    FlexGridSizer20->SetSizeHints(Panel11);
    Panel12 = new wxPanel(Notebook2, ID_PANEL12, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL12"));
    FlexGridSizer35 = new wxFlexGridSizer(3, 2, 0, 0);
    FlexGridSizer35->AddGrowableCol(0);
    cbOptK1 = new wxCheckBox(Panel12, ID_CHECKBOX4, _("k1 (r^2 dist.)"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX4"));
    cbOptK1->SetValue(true);
    FlexGridSizer35->Add(cbOptK1, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 1);
    cbOptK2 = new wxCheckBox(Panel12, ID_CHECKBOX5, _("k2 (r^4 dist.)"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX5"));
    cbOptK2->SetValue(true);
    FlexGridSizer35->Add(cbOptK2, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 1);
    cbOptK3 = new wxCheckBox(Panel12, ID_CHECKBOX6, _("k3 (r^6 dist.)"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX6"));
    cbOptK3->SetValue(false);
    FlexGridSizer35->Add(cbOptK3, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 1);
    FlexGridSizer35->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbOptT1 = new wxCheckBox(Panel12, ID_CHECKBOX7, _("t1 (tang. dist.)"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX7"));
    cbOptT1->SetValue(false);
    FlexGridSizer35->Add(cbOptT1, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 1);
    cbOptT2 = new wxCheckBox(Panel12, ID_CHECKBOX8, _("t2 (tang. dist.)"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX8"));
    cbOptT2->SetValue(false);
    FlexGridSizer35->Add(cbOptT2, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 1);
    Panel12->SetSizer(FlexGridSizer35);
    FlexGridSizer35->Fit(Panel12);
    FlexGridSizer35->SetSizeHints(Panel12);
    Panel13 = new wxPanel(Notebook2, ID_PANEL13, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL13"));
    FlexGridSizer34 = new wxFlexGridSizer(3, 1, 0, 0);
    FlexGridSizer34->AddGrowableCol(0);
    FlexGridSizer36 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer36->AddGrowableCol(1);
    StaticText25 = new wxStaticText(Panel13, ID_STATICTEXT26, _("Max. iterations:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT26"));
    FlexGridSizer36->Add(StaticText25, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edMaxIters = new wxSpinCtrl(Panel13, ID_SPINCTRL8, _T("1000"), wxDefaultPosition, wxDefaultSize, 0, 0, 10000, 1000, _T("ID_SPINCTRL8"));
    edMaxIters->SetValue(_T("1000"));
    FlexGridSizer36->Add(edMaxIters, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer34->Add(FlexGridSizer36, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    cbCalibUseRobust = new wxCheckBox(Panel13, ID_CHECKBOX3, _("Pseudo-Huber robust kernel"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX3"));
    cbCalibUseRobust->SetValue(false);
    FlexGridSizer34->Add(cbCalibUseRobust, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbCalibNormalize = new wxCheckBox(Panel13, ID_CHECKBOX2, _("Normalize images"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX2"));
    cbCalibNormalize->SetValue(true);
    FlexGridSizer34->Add(cbCalibNormalize, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel13->SetSizer(FlexGridSizer34);
    FlexGridSizer34->Fit(Panel13);
    FlexGridSizer34->SetSizeHints(Panel13);
    Notebook2->AddPage(Panel11, _("Board"), true);
    Notebook2->AddPage(Panel12, _("Params. to find"), false);
    Notebook2->AddPage(Panel13, _("Settings"), false);
    FlexGridSizer29->Add(Notebook2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer31 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer31->AddGrowableCol(0);
    btnRunCalib = new wxButton(Panel6, ID_BUTTON14, _("Run optimizer..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON14"));
    wxFont btnRunCalibFont(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    btnRunCalib->SetFont(btnRunCalibFont);
    FlexGridSizer31->Add(btnRunCalib, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer29->Add(FlexGridSizer31, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer11->Add(FlexGridSizer29, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer27 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer27->AddGrowableCol(0);
    FlexGridSizer27->AddGrowableRow(1);
    FlexGridSizer28 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer28->AddGrowableCol(0);
    StaticText23 = new wxStaticText(Panel6, ID_STATICTEXT24, _("Calibration results:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT24"));
    wxFont StaticText23Font(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    StaticText23->SetFont(StaticText23Font);
    FlexGridSizer28->Add(StaticText23, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnSaveCalib = new wxButton(Panel6, ID_BUTTON13, _("Save calibration..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON13"));
    FlexGridSizer28->Add(btnSaveCalib, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer27->Add(FlexGridSizer28, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    NotebookCalibResults = new wxNotebook(Panel6, ID_NOTEBOOK2, wxDefaultPosition, wxDefaultSize, 0, _T("ID_NOTEBOOK2"));
    Panel1 = new wxPanel(NotebookCalibResults, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    FlexGridSizer19 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer19->AddGrowableCol(0);
    FlexGridSizer19->AddGrowableRow(0);
    edLogCalibResult = new wxTextCtrl(Panel1, ID_TEXTCTRL8, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE|wxTE_READONLY|wxTE_DONTWRAP, wxDefaultValidator, _T("ID_TEXTCTRL8"));
    wxFont edLogCalibResultFont(8,wxFONTFAMILY_TELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    edLogCalibResult->SetFont(edLogCalibResultFont);
    FlexGridSizer19->Add(edLogCalibResult, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel1->SetSizer(FlexGridSizer19);
    FlexGridSizer19->Fit(Panel1);
    FlexGridSizer19->SetSizeHints(Panel1);
    Panel10 = new wxPanel(NotebookCalibResults, ID_PANEL10, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL10"));
    FlexGridSizer43 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer43->AddGrowableCol(0);
    FlexGridSizer43->AddGrowableRow(0);
    m_plot3D_cameras = new CMyGLCanvas(Panel10,ID_CUSTOM6,wxDefaultPosition,wxDefaultSize,wxTAB_TRAVERSAL,_T("ID_CUSTOM6"));
    FlexGridSizer43->Add(m_plot3D_cameras, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel10->SetSizer(FlexGridSizer43);
    FlexGridSizer43->Fit(Panel10);
    FlexGridSizer43->SetSizeHints(Panel10);
    NotebookCalibResults->AddPage(Panel1, _("Report"), false);
    NotebookCalibResults->AddPage(Panel10, _("6D camera poses"), true);
    FlexGridSizer27->Add(NotebookCalibResults, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer11->Add(FlexGridSizer27, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer9->Add(FlexGridSizer11, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel6->SetSizer(FlexGridSizer9);
    FlexGridSizer9->Fit(Panel6);
    FlexGridSizer9->SetSizeHints(Panel6);
    Panel5 = new wxPanel(Notebook1, ID_PANEL5, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL5"));
    FlexGridSizer15 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer15->AddGrowableCol(0);
    FlexGridSizer15->AddGrowableRow(0);
    m_plot3D = new CMyGLCanvas(Panel5,ID_XY_GLCANVAS,wxDefaultPosition,wxDefaultSize,wxTAB_TRAVERSAL,_T("ID_XY_GLCANVAS"));
    FlexGridSizer15->Add(m_plot3D, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    FlexGridSizer16 = new wxFlexGridSizer(6, 1, 0, 0);
    FlexGridSizer16->AddGrowableCol(0);
    FlexGridSizer16->AddGrowableRow(5);
    StaticText3 = new wxStaticText(Panel5, ID_STATICTEXT3, _("Press \"Connect\" to start grabbing:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
    FlexGridSizer16->Add(StaticText3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnConnectLive3D = new wxButton(Panel5, ID_BUTTON18, _("Connect..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON18"));
    FlexGridSizer16->Add(btnConnectLive3D, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnDisconnectLive = new wxButton(Panel5, ID_BUTTON20, _("Disconnect"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON20"));
    btnDisconnectLive->Disable();
    FlexGridSizer16->Add(btnDisconnectLive, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    StaticText4 = new wxStaticText(Panel5, ID_STATICTEXT4, _("You should be seeing the\nlive colored 3D point cloud. \n\nBy loading a correct calibration\nfile, the colors should match \naccurately to the real objects\neven close to sharp edges.\n\nIf no error was found but still\nhave no images, try disconnecting \nand connecting again."), wxPoint(40,32), wxDefaultSize, 0, _T("ID_STATICTEXT4"));
    FlexGridSizer16->Add(StaticText4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer18 = new wxFlexGridSizer(1, 3, 0, 0);
    btnLoadCalib = new wxButton(Panel5, ID_BUTTON19, _("Load calib..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON19"));
    wxFont btnLoadCalibFont(8,wxDEFAULT,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    btnLoadCalib->SetFont(btnLoadCalibFont);
    FlexGridSizer18->Add(btnLoadCalib, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnSaveCalibLive = new wxButton(Panel5, ID_BUTTON21, _("Save calib..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON21"));
    wxFont btnSaveCalibLiveFont(8,wxDEFAULT,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    btnSaveCalibLive->SetFont(btnSaveCalibLiveFont);
    FlexGridSizer18->Add(btnSaveCalibLive, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnHelpLiveCalib = new wxBitmapButton(Panel5, ID_BITMAPBUTTON1, wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_HELP")),wxART_BUTTON), wxDefaultPosition, wxDefaultSize, wxBU_AUTODRAW, wxDefaultValidator, _T("ID_BITMAPBUTTON1"));
    btnHelpLiveCalib->SetDefault();
    btnHelpLiveCalib->SetToolTip(_("Help..."));
    FlexGridSizer18->Add(btnHelpLiveCalib, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer16->Add(FlexGridSizer18, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    m_grid_live_calib = new wxGrid(Panel5, ID_GRID1, wxDefaultPosition, wxDefaultSize, wxVSCROLL|wxHSCROLL, _T("ID_GRID1"));
    m_grid_live_calib->CreateGrid(3,1);
    wxFont m_grid_live_calibFont(8,wxDEFAULT,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    m_grid_live_calib->SetFont(m_grid_live_calibFont);
    m_grid_live_calib->EnableEditing(true);
    m_grid_live_calib->EnableGridLines(true);
    m_grid_live_calib->SetRowLabelSize(90);
    m_grid_live_calib->SetDefaultColSize(100, true);
    wxFont GridLabelFont_1(8,wxDEFAULT,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    m_grid_live_calib->SetLabelFont(GridLabelFont_1);
    m_grid_live_calib->SetDefaultCellFont( m_grid_live_calib->GetFont() );
    m_grid_live_calib->SetDefaultCellTextColour( m_grid_live_calib->GetForegroundColour() );
    FlexGridSizer16->Add(m_grid_live_calib, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer15->Add(FlexGridSizer16, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel5->SetSizer(FlexGridSizer15);
    FlexGridSizer15->Fit(Panel5);
    FlexGridSizer15->SetSizeHints(Panel5);
    Notebook1->AddPage(Panel8, _("Welcome"), true);
    Notebook1->AddPage(Panel9, _("0) Instructions"), false);
    Notebook1->AddPage(Panel3, _("1) Test connection"), false);
    Notebook1->AddPage(Panel4, _("2) Capture"), false);
    Notebook1->AddPage(Panel6, _("3) Stereo/RGBD calibration"), false);
    Notebook1->AddPage(Panel5, _("4) Live Kinect test"), false);
    FlexGridSizer1->Add(Notebook1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    Panel2 = new wxPanel(this, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
    FlexGridSizer2 = new wxFlexGridSizer(0, 2, 0, 0);
    FlexGridSizer2->AddGrowableCol(0);
    FlexGridSizer13 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer13->AddGrowableCol(1);
    FlexGridSizer13->AddGrowableRow(0);
    StaticText10 = new wxStaticText(Panel2, ID_STATICTEXT10, _("Log:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT10"));
    FlexGridSizer13->Add(StaticText10, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edLogTest = new wxTextCtrl(Panel2, ID_TEXTCTRL4, wxEmptyString, wxDefaultPosition, wxSize(-1,50), wxTE_MULTILINE|wxTE_READONLY|wxHSCROLL|wxVSCROLL, wxDefaultValidator, _T("ID_TEXTCTRL4"));
    wxFont edLogTestFont(8,wxFONTFAMILY_TELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    edLogTest->SetFont(edLogTestFont);
    FlexGridSizer13->Add(edLogTest, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer2->Add(FlexGridSizer13, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer14 = new wxFlexGridSizer(1, 2, 0, 0);
    bntAbout = new wxButton(Panel2, ID_BUTTON1, _("About"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
    FlexGridSizer14->Add(bntAbout, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnQuit = new wxButton(Panel2, ID_BUTTON2, _("Quit"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
    FlexGridSizer14->Add(btnQuit, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer2->Add(FlexGridSizer14, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    Panel2->SetSizer(FlexGridSizer2);
    FlexGridSizer2->Fit(Panel2);
    FlexGridSizer2->SetSizeHints(Panel2);
    FlexGridSizer1->Add(Panel2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    SetSizer(FlexGridSizer1);
    timConsoleDump.SetOwner(this, ID_TIMER1);
    timConsoleDump.Start(100, false);
    timMisc.SetOwner(this, ID_TIMER2);
    timMisc.Start(2, false);
    FlexGridSizer1->Fit(this);
    FlexGridSizer1->SetSizeHints(this);
    Center();

    Connect(ID_BUTTON15,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnOpCalibKinectClick);
    Connect(ID_BUTTON16,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnOpCalibStereoGenericClick);
    Connect(ID_BUTTON17,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnOpTestKinectClick);
    Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnNext1Click);
    Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnConnectClick);
    Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnNext1Click);
    Connect(ID_BUTTON8,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnDisconnectClick);
    Connect(ID_RADIOBOX1,wxEVT_COMMAND_RADIOBOX_SELECTED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnrbChannelSwitchSelect);
    Connect(ID_SPINCTRL7,wxEVT_COMMAND_SPINCTRL_UPDATED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnedTiltChange);
    Connect(ID_BUTTON6,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnCaptureClick);
    Connect(ID_BUTTON7,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnNextCalibClick);
    Connect(ID_LISTBOX1,wxEVT_COMMAND_LISTBOX_SELECTED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnlbImagePairsSelect);
    Connect(ID_BUTTON9,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnListRemoveSelectedClick);
    Connect(ID_BUTTON10,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnListLoadClick);
    Connect(ID_BUTTON11,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnLoadImageListClick);
    Connect(ID_BUTTON12,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnListSaveClick);
    Connect(ID_RADIOBOX2,wxEVT_COMMAND_RADIOBOX_SELECTED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnrbShowImagesSelect);
    Connect(ID_SPINCTRL3,wxEVT_COMMAND_SPINCTRL_UPDATED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnNeedsToUpdate6DCamPlot);
    Connect(ID_SPINCTRL4,wxEVT_COMMAND_SPINCTRL_UPDATED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnNeedsToUpdate6DCamPlot);
    Connect(ID_TEXTCTRL6,wxEVT_COMMAND_TEXT_UPDATED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnedCalibSizeXText);
    Connect(ID_TEXTCTRL7,wxEVT_COMMAND_TEXT_UPDATED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnedCalibSizeXText);
    Connect(ID_CHECKBOX3,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OncbCalibNormalizeClick);
    Connect(ID_CHECKBOX2,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OncbCalibNormalizeClick);
    Connect(ID_BUTTON14,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnRunCalibClick);
    Connect(ID_BUTTON13,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnSaveCalibClick);
    Connect(ID_BUTTON18,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnConnectLive3DClick);
    Connect(ID_BUTTON20,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnDisconnectLiveClick);
    Connect(ID_BUTTON19,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnLoadCalibClick);
    Connect(ID_BUTTON21,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnSaveCalibLiveClick);
    Connect(ID_BITMAPBUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnHelpLiveCalibClick);
    Panel5->Connect(wxEVT_SET_FOCUS,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnPanel5SetFocus,0,this);
    Connect(ID_NOTEBOOK1,wxEVT_COMMAND_NOTEBOOK_PAGE_CHANGING,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnNotebook1PageChanging);
    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnAbout);
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnbtnQuitClick);
    Connect(ID_TIMER1,wxEVT_TIMER,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OntimConsoleDumpTrigger);
    Connect(ID_TIMER2,wxEVT_TIMER,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OntimMiscTrigger);
    Connect(wxID_ANY,wxEVT_CLOSE_WINDOW,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnClose);
    Connect(wxEVT_SIZE,(wxObjectEventFunction)&kinect_calibrate_guiDialog::OnResize);
    //*)

    Connect(ID_GRID1,
#if wxCHECK_VERSION(3, 1, 0)
	wxEVT_GRID_CELL_CHANGED,
#else
	wxEVT_GRID_CELL_CHANGE,
#endif
(wxObjectEventFunction)&kinect_calibrate_guiDialog::Onm_grid_live_calibCellChange);

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



	// Prepare 3D scene: (of live view)
	// ------------------------------------------
	m_plot3D->m_openGLScene = mrpt::opengl::COpenGLScene::Create();

	// Ground plane:
	{
		mrpt::opengl::CGridPlaneXYPtr obj = mrpt::opengl::CGridPlaneXY::Create(-10,10, -10,10, 0, 1);
		obj->setColor_u8( TColor(200,200,200) );
		m_plot3D->m_openGLScene->insert(obj);
	}
	//// XYZ corner:
	//m_plot3D->m_openGLScene->insert( mrpt::opengl::stock_objects::CornerXYZSimple(0.5,2) );

	// 3D points:
	m_gl_3d_points = mrpt::opengl::CPointCloudColoured::Create();
	m_gl_3d_points->setPointSize(2);
	m_plot3D->m_openGLScene->insert( m_gl_3d_points );

	m_gl_corner_left  = mrpt::opengl::stock_objects::CornerXYZSimple(0.03f,2);
	m_gl_corner_right = mrpt::opengl::stock_objects::CornerXYZSimple(0.03f,2);
	m_plot3D->m_openGLScene->insert( m_gl_corner_left );
	m_plot3D->m_openGLScene->insert( m_gl_corner_right );

	// Prepare 3D scene: (of calibrate view)
	// ------------------------------------------
	CalibUpdate3DViewCameras();


	// Init. grid cells:
	this->LiveCalibGridInitialize();
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
	if (event.GetSelection()==5 && event.GetOldSelection()==4)
	{
		event.Skip(); // Permit change
	}
	else
	{
		event.Veto(); // Veto change
	}
}

void kinect_calibrate_guiDialog::OnbtnNext1Click(wxCommandEvent& event)
{
	if (Notebook1->GetSelection()==1)
	{
		btnConnect->Enable(true);
		btnDisconnect->Enable(false);
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
	m_cap_thread_data.flag_grab_depth = false; // we don't need depth data for image registration.
	m_findcorners_thread_data.quit = false;

	// Launch thread:
	m_cap_thread         = mrpt::system::createThreadFromObjectMethod(this, &kinect_calibrate_guiDialog::thread_grabbing);
	m_findcorners_thread = mrpt::system::createThreadFromObjectMethod(this, &kinect_calibrate_guiDialog::thread_find_corners);
	btnConnect->Enable(false);
	btnDisconnect->Enable(true);

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
		kinect.enableGrabDepth( p.flag_grab_depth );
		kinect.enableGrab3DPoints(false);
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
		btnDisconnect->Enable(false);
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
	case 2:
		{
			CImage im = m_last_obs->intensityImage;

			const std::string s = mrpt::system::dateTimeLocalToString( m_last_obs->timestamp );
			im.textOut( 6,6, s, TColor::black );
			im.textOut( 5,5, s, TColor::white );

			m_realtimeview_test->AssignImage( im );
			m_realtimeview_test->Refresh(false);
			if (!btnNext2->IsEnabled()) btnNext2->Enable();
		}
		break;

	// ------------------------------------------
	//   Tab 2: Capturing
	// ------------------------------------------
	case 3:
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
				// Grab RGB (right) image:
				m_calib_images.resize(m_calib_images.size()+1);

				mrpt::vision::TImageStereoCalibData &scd = *m_calib_images.rbegin();
				scd.right.img_original = m_last_obs->intensityImage;

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
				// Grab IR (left) image:
				mrpt::vision::TImageStereoCalibData &scd = *m_calib_images.rbegin();
				scd.left.img_original = m_last_obs->intensityImage;

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
				//img_to_show.normalize();
				img_to_show.equalizeHistInPlace();

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

	// ------------------------------------------
	//   Tab 4: Live test:
	// ------------------------------------------
	case 5:
		{
			m_plot3D->last_timestamp = m_last_obs->timestamp;

			m_last_obs->cameraParams          = m_calib_result.cam_params.leftCamera;
			m_last_obs->cameraParamsIntensity = m_calib_result.cam_params.rightCamera;

			const mrpt::poses::CPose3D l2r =  mrpt::poses::CPose3D(0,0,0, DEG2RAD(-90),0,DEG2RAD(-90)) + (-m_calib_result.right2left_camera_pose);

			m_last_obs->relativePoseIntensityWRTDepth = l2r; // L->R (Depth -> Intensity/RGB)

			T3DPointsProjectionParams pp;
			pp.takeIntoAccountSensorPoseOnRobot = false;

			m_last_obs->project3DPointsFromDepthImageInto(*m_gl_3d_points, pp);

			m_gl_corner_left->setPose( mrpt::poses::CPose3D() );
			m_gl_corner_right->setPose( l2r );

			m_plot3D->Refresh(false);
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

void kinect_calibrate_guiDialog::OnedTiltChange(wxSpinEvent& event)
{
	m_cap_thread_data.tilt_ang_deg = edTilt->GetValue();
}

void kinect_calibrate_guiDialog::UpdateListOfImages()
{
	wxArrayString  lstStrs;
	for (size_t i=0;i<m_calib_images.size();i++)
	{
		lstStrs.Add( wxString::Format(_("ImgPair #%u"),static_cast<unsigned int>(i) ) );
	}
	lbImagePairs->Set(lstStrs);

	lbImagePairs->SetSelection( m_calib_images.size()-1 );
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
			const int image_mode = rbShowImages->GetSelection();

			CImage il, ir;

			// Common part to all (but one) modes:
			if (image_mode!=1)
			{
				il = m_calib_images[sel].left.img_original;
				ir = m_calib_images[sel].right.img_original;

				if (cbCalibNormalize->IsChecked()) {
					if (!il.isColor()) il.equalizeHistInPlace();
					if (!ir.isColor()) ir.equalizeHistInPlace();
				}
			}


			switch (image_mode)
			{
			// ======= Original images =======
			case 0:
				{ // Nothing else to do.
				}
				break;
			// ======= Detected chessboard =======
			case 1:
				{
					il = m_calib_images[sel].left.img_checkboard;
					ir = m_calib_images[sel].right.img_checkboard;
				}
				break;

			// ======= Reprojected corners =======
			case 2:
				{
					il.colorImageInPlace();
					ir.colorImageInPlace();

					il.drawChessboardCorners(m_calib_images[sel].left.projectedPoints_distorted, m_calib_params.check_size_x, m_calib_params.check_size_y );
					ir.drawChessboardCorners(m_calib_images[sel].right.projectedPoints_distorted, m_calib_params.check_size_x, m_calib_params.check_size_y );
				}
				break;
			// ======= Undistorted image =======
			case 3:
				{
					il.rectifyImageInPlace( m_calib_result.cam_params.leftCamera );
					ir.rectifyImageInPlace( m_calib_result.cam_params.rightCamera );
				}
				break;
			// ======= Rectified images =======
			case 4:
				{

//					projectedPoints_distorted
//					m_calib_images[sel].left.
				}
				break;

			default:
					// Shouldn't arrive here!
					break;
			}

			// Resize images, keeping aspect ratio:
			const wxSize szView = m_view_left->GetSize();  // In theory, right&left will be always equal.
			const double szRatio = static_cast<double>(szView.x)/szView.y;

			const mrpt::utils::TImageSize szL = il.getSize();
			const mrpt::utils::TImageSize szR = ir.getSize();

			const double lRatio = static_cast<double>(szL.x)/szL.y; // Don't assume both images have equal size
			const double rRatio = static_cast<double>(szR.x)/szR.y;

			mrpt::utils::TImageSize trg_sz_l, trg_sz_r;

			if (szRatio<lRatio)
			{	// Fill y
				trg_sz_l.x = szView.x;
				trg_sz_l.y = szL.y * szView.x/static_cast<double>(szL.x);
			} else
			{	// Fill x
				trg_sz_l.x = szL.x * szView.y/static_cast<double>(szL.y);
				trg_sz_l.y = szView.y;
			}

			if (szRatio<rRatio)
			{	// Fill y
				trg_sz_r.x = szView.x;
				trg_sz_r.y = szR.y * szView.x/static_cast<double>(szR.x);
			} else
			{	// Fill x
				trg_sz_r.x = szR.x * szView.y/static_cast<double>(szR.y);
				trg_sz_r.y = szView.y;
			}

			il.scaleImage(trg_sz_l.x,trg_sz_l.y, IMG_INTERP_AREA );
			ir.scaleImage(trg_sz_r.x,trg_sz_r.y, IMG_INTERP_AREA );

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

struct TCalibCallbackData
{
	kinect_calibrate_guiDialog *win;
	wxProgressDialog *pd;
};

void myCalibCallback(const mrpt::vision::TImageStereoCallbackData &d, void* user_data)
{
	if ((d.current_iter & 0x0F)!=0) return;

	TCalibCallbackData *dat = reinterpret_cast<TCalibCallbackData*>(user_data);

	string s;
	switch (d.calibRound)
	{
		case -1:
			s = mrpt::format("Detecting corners: %u images done out of %u",d.nImgsProcessed,d.nImgsToProcess);
			break;
		case 0:
			s = "Round #1: Calibration without distortion";
			break;
		case 1:
			s = "Round #2: Full calibration";
			break;
	};

	if (d.calibRound==0 || d.calibRound==1)
	{
		s+= mrpt::format(" (RMSE=%.05f px)", d.current_rmse);
	}

	dat->pd->Update(d.current_iter, _U(s.c_str()));
	dat->pd->SetSize(500,100);

	wxTheApp->Yield();
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
		m_calib_params.use_robust_kernel = cbCalibUseRobust->IsChecked();

//			wxBusyInfo info(_("Running optimizer..."),this);
//			wxTheApp->Yield();

		wxProgressDialog  pd(_("Calibration in progress..."), _("(starting)"), m_calib_params.maxIters, this, wxPD_APP_MODAL | wxPD_SMOOTH | wxPD_ELAPSED_TIME );
		pd.SetSize(500,100);
		wxTheApp->Yield();

		// Prep. callback function:
		TCalibCallbackData callback_data;
		callback_data.win = this;
		callback_data.pd = &pd;

		m_calib_params.callback = &myCalibCallback;
		m_calib_params.callback_user_param = reinterpret_cast<void*>(&callback_data);

		edLogCalibResult->Clear();

		// Run calibration:
		bool res;
		{
			res = mrpt::vision::checkerBoardStereoCalibration( m_calib_images, m_calib_params, m_calib_result );
		}

		pd.Close();

		// Repaint 6D camera poses:
		CalibUpdate3DViewCameras();
		m_plot3D_cameras->Refresh();

		// And reprojected corners:
		rbShowImages->SetSelection(2);
		ProcessNewSelectedImageListBox();

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
				"# Generated by kinect-stereo-calib - MRPT at %s"
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
				"# Left camera (IR/Depth in Kinect) calibration parameters (and 95%% confidence intervals):\n"
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
				"# Right camera (RGB in Kinect) calibration parameters  (and 95%% confidence intervals):\n"
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
				"# This assumes that both camera frames are such that +Z points\n"
				"# forwards, and +X and +Y to the right and downwards.\n"
				"[CAMERA_PARAMS_LEFT2RIGHT_POSE]\n"
				"translation_only     = [%e %e %e]\n"
				"rotation_matrix_only = %s\n"
				"pose_yaw_pitch_roll  = %s  // (YPR in degrees)\n"
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
	WX_START_TRY

	wxString startPath;
	m_config.Read(_("last_path"),&startPath);

    wxFileDialog  dialog(this, _("Save calibration file"), startPath, _("calib.ini") ,_("Configuration files (*.ini,*.cfg)|*.ini;*.cfg|All files (*.*)|*.*"), wxFD_SAVE | wxFD_OVERWRITE_PROMPT );
    if (dialog.ShowModal() == wxID_OK)
    {
        if (!edLogCalibResult->SaveFile( dialog.GetPath() ))
			throw std::runtime_error("Error saving file!");
    }

    WX_END_TRY
}

void kinect_calibrate_guiDialog::OnbtnOpCalibKinectClick(wxCommandEvent& event)
{
	Notebook1->ChangeSelection(1);
}

void kinect_calibrate_guiDialog::OnbtnOpCalibStereoGenericClick(wxCommandEvent& event)
{
	Notebook1->ChangeSelection(4);
}

void kinect_calibrate_guiDialog::OnbtnOpTestKinectClick(wxCommandEvent& event)
{
	LiveCalibUpdateToGrid();
	Notebook1->ChangeSelection(5);
}

void kinect_calibrate_guiDialog::OnbtnDisconnectClick(wxCommandEvent& event)
{
	this->StopLiveGrabThreads();

	btnConnect->Enable();
	btnNext2->Disable();
}

void kinect_calibrate_guiDialog::OnbtnListLoadClick(wxCommandEvent& event)
{
	WX_START_TRY

	wxString startPath;
	m_config.Read(_("last_path"),&startPath);

    wxFileDialog  dialog1(this, _("Choose LEFT image"), startPath, _("") ,imageWildcards, wxFD_OPEN | wxFD_FILE_MUST_EXIST );
    if (dialog1.ShowModal() == wxID_OK)
    {
        const string file_img_l = string( dialog1.GetPath().mb_str() );

        const string fil_dir = mrpt::system::extractFileDirectory(file_img_l);

        startPath = _U(fil_dir.c_str());
        m_config.Write(_("last_path"),startPath);

		wxFileDialog  dialog2(this, _("Choose RIGHT image"), startPath, _("") ,imageWildcards, wxFD_OPEN | wxFD_FILE_MUST_EXIST );
		if (dialog2.ShowModal() == wxID_OK)
		{
			const string file_img_r = string( dialog2.GetPath().mb_str() );

			// Append image pair:
			m_calib_images.resize( m_calib_images.size() + 1 );

			mrpt::vision::TImageStereoCalibData & scd = *m_calib_images.rbegin();

			if (!scd.left.img_original.loadFromFile(file_img_l)) THROW_EXCEPTION_CUSTOM_MSG1("Error loading image: %s",file_img_l.c_str() )
			if (!scd.right.img_original.loadFromFile(file_img_r)) THROW_EXCEPTION_CUSTOM_MSG1("Error loading image: %s",file_img_r.c_str() )

			UpdateListOfImages();
		}
    }

    WX_END_TRY
}

void kinect_calibrate_guiDialog::OnbtnListSaveClick(wxCommandEvent& event)
{
    WX_START_TRY

    if (m_calib_images.empty())
		throw std::runtime_error("There are no images to save!");

	wxString startPath;
	m_config.Read(_("last_path"),&startPath);

	startPath = startPath + _("/calib_images");

    wxDirDialog dlg(this, _("Choose destination DIRECTORY (will be created if it does not exist)"), startPath, wxDD_DEFAULT_STYLE);

    if (dlg.ShowModal() == wxID_OK)
    {
		const string trg_dir = string( dlg.GetPath().mb_str() );
		mrpt::system::createDirectory(trg_dir);

    	const size_t N = m_calib_images.size();

    	const string sListFile = mrpt::format("%s/list_calibration_images.txt",trg_dir.c_str());
		if (mrpt::system::fileExists(sListFile)) throw std::runtime_error(mrpt::format("Error: target listing file already exists (won't overwrite as precaution): %s",sListFile.c_str()));

		{
			wxBusyCursor  busy;
			wxTheApp->Yield();

			ofstream fo(sListFile.c_str());

			for (unsigned int i=0;i<N;i++)
			{
				const mrpt::vision::TImageStereoCalibData & scd = m_calib_images[i];

				const string sLr = mrpt::format("left_%04u.png",i);
				const string sRr = mrpt::format("right_%04u.png",i);

				const string sL = mrpt::format("%s/%s",trg_dir.c_str(),sLr.c_str());
				const string sR = mrpt::format("%s/%s",trg_dir.c_str(),sRr.c_str());

				if (mrpt::system::fileExists(sL)) throw std::runtime_error(mrpt::format("Error: target image filename already exists (won't overwrite as precaution): %s",sL.c_str()));
				if (mrpt::system::fileExists(sR)) throw std::runtime_error(mrpt::format("Error: target image filename already exists (won't overwrite as precaution): %s",sR.c_str()));

				if (!scd.left.img_original.saveToFile(sL) ) throw std::runtime_error(mrpt::format("Error: can't write image file: %s",sL.c_str()));
				if (!scd.right.img_original.saveToFile(sR)) throw std::runtime_error(mrpt::format("Error: can't write image file: %s",sR.c_str()));

				fo << sLr << endl << sRr << endl;
			}
		}

		wxMessageBox(_U(mrpt::format("Images saved! An index file has been also generated:\n%s",sListFile.c_str()).c_str()), _("Done!") );
	}

    WX_END_TRY
}

void kinect_calibrate_guiDialog::OnbtnListRemoveSelectedClick(wxCommandEvent& event)
{
    WX_START_TRY
	const int sel = lbImagePairs->GetSelection();

	if (sel==wxNOT_FOUND || sel>=(int)m_calib_images.size() ) return;

	m_calib_images.erase( m_calib_images.begin() + sel );
	UpdateListOfImages();
    WX_END_TRY
}

void kinect_calibrate_guiDialog::OnbtnLoadImageListClick(wxCommandEvent& event)
{
    WX_START_TRY

	wxMessageBox(_("Image list files contain one file name per line, in order:\nLEFT1\nRIGHT1\nLEFT2\nRIGHT2\n..."),  _("Information") );

	wxString startPath;
	m_config.Read(_("last_path"),&startPath);

    wxFileDialog  dialog(this, _("Select text file with list of image files"), startPath, _("") ,_("Text files (*.txt)|*.txt|All files (*.*)|*.*"), wxFD_OPEN | wxFD_FILE_MUST_EXIST );
    if (dialog.ShowModal() == wxID_OK)
    {
        const string file_list = string( dialog.GetPath().mb_str() );

        const string fil_dir = mrpt::system::extractFileDirectory(file_list);
        startPath = _U(fil_dir.c_str());
        m_config.Write(_("last_path"),startPath);

        mrpt::utils::CStringList lst;
        lst.loadFromFile(file_list);

        if (lst.size()==0) throw std::runtime_error("Error: List file seems to be empty.");
        if ((lst.size() & 1)!=0) throw std::runtime_error("Error: Number of lines in file must be even.");

        const unsigned int N = lst.size()/2;

		m_calib_images.clear();
		m_calib_images.resize(N);

		wxBusyCursor  busy;
		wxTheApp->Yield();


        for (unsigned int i=0;i<N;i++)
        {
        	const string &sL = lst(2*i+0);
        	const string &sR = lst(2*i+1);

			mrpt::vision::TImageStereoCalibData & scd = m_calib_images[i];

			if (!scd.left.img_original.loadFromFile(sL) &&
				!scd.left.img_original.loadFromFile(fil_dir+string("/")+sL) ) throw std::runtime_error(mrpt::format("Error: loading left image '%s' of %u'th pair",sL.c_str(),i).c_str() );

			if (!scd.right.img_original.loadFromFile(sR) &&
				!scd.right.img_original.loadFromFile(fil_dir+string("/")+sR) ) throw std::runtime_error(mrpt::format("Error: loading right image '%s' of %u'th pair",sR.c_str(),i).c_str() );
        }
		UpdateListOfImages();
    }

    WX_END_TRY
}

void kinect_calibrate_guiDialog::OnrbShowImagesSelect(wxCommandEvent& event)
{
	ProcessNewSelectedImageListBox();
}

void kinect_calibrate_guiDialog::OncbCalibNormalizeClick(wxCommandEvent& event)
{
	ProcessNewSelectedImageListBox();
}

void kinect_calibrate_guiDialog::OnResize(wxSizeEvent& event)
{
	if (Notebook1->GetSelection()==4)
		ProcessNewSelectedImageListBox();

	event.Skip();
}

void kinect_calibrate_guiDialog::OnbtnConnectLive3DClick(wxCommandEvent& event)
{
	btnConnectLive3D->Enable(false);

	if (!m_cap_thread.isClear()) {
		// Shoulnd't reach here...just in case:
		return;
	}

	m_cap_thread_data.quit = false;
	m_cap_thread_data.flag_grab_depth = true;
	m_cap_thread_data.select_IR_channel = false;

	// Launch thread:
	m_cap_thread         = mrpt::system::createThreadFromObjectMethod(this, &kinect_calibrate_guiDialog::thread_grabbing);

	btnDisconnectLive->Enable(true);
}

void kinect_calibrate_guiDialog::OnbtnDisconnectLiveClick(wxCommandEvent& event)
{
	this->StopLiveGrabThreads();

	btnConnectLive3D->Enable(true);
	btnDisconnectLive->Enable(false);
}


void kinect_calibrate_guiDialog::OnbtnApplyCalibLiveClick(wxCommandEvent& event)
{
}


// Save current live-panel calib to disk:
void kinect_calibrate_guiDialog::OnbtnSaveCalibLiveClick(wxCommandEvent& event)
{
	WX_START_TRY

	wxString startPath;
	m_config.Read(_("last_path"),&startPath);

    wxFileDialog  dialog(this, _("Save calibration file"), startPath, _("calib.ini") ,_("Configuration files (*.ini,*.cfg)|*.ini;*.cfg|All files (*.*)|*.*"), wxFD_SAVE | wxFD_OVERWRITE_PROMPT );
    if (dialog.ShowModal() == wxID_OK)
    {
        const string outFil = string( dialog.GetPath().mb_str() );

    	std::ofstream f(outFil.c_str());

    	if (!f.is_open()) throw std::runtime_error("Error opening file for writing!");

		time_t systime;
		time(&systime);
		struct tm * timeinfo=localtime(&systime);

		f << mrpt::format(
			"# Stereo camera (manual) calibration report\n"
			"# Generated by kinect-stereo-calib - MRPT at %s"
			"# (This file is loadable from rawlog-edit and other MRPT tools)\n"
			"# ---------------------------------------------------------------------\n\n",
			asctime(timeinfo)
			);

		const TCamera &lc = m_calib_result.cam_params.leftCamera;
		const TCamera &rc = m_calib_result.cam_params.rightCamera;

		f << mrpt::format(
			"# Left camera (IR/Depth in Kinect) calibration parameters:\n"
			"[CAMERA_PARAMS_LEFT]\n"
			"resolution = [%u %u]\n"
			"cx         = %f\n"
			"cy         = %f\n"
			"fx         = %f\n"
			"fy         = %f\n"
			"dist       = [%e %e %e %e %e]    // The order is: [K1 K2 T1 T2 K3]\n\n",
				lc.ncols, lc.nrows,
				lc.cx(),
				lc.cy(),
				lc.fx(),
				lc.fy(),
				lc.k1(), lc.k2(), lc.p1(), lc.p2(), lc.k3() );

		f << mrpt::format(
			"# Right camera (RGB in Kinect) calibration parameters:\n"
			"[CAMERA_PARAMS_RIGHT]\n"
			"resolution = [%u %u]\n"
			"cx         = %f\n"
			"cy         = %f\n"
			"fx         = %f\n"
			"fy         = %f\n"
			"dist       = [%e %e %e %e %e]    // The order is: [K1 K2 T1 T2 K3]\n\n",
				rc.ncols, rc.nrows,
				rc.cx(),
				rc.cy(),
				rc.fx(),
				rc.fy(),
				rc.k1(), rc.k2(), rc.p1(), rc.p2(), rc.k3() );

		// pose:
		const mrpt::poses::CPose3D     RT_YPR(- m_calib_result.right2left_camera_pose );
		const mrpt::poses::CPose3DQuat RT_quat(RT_YPR);

		f << mrpt::format(
			"# Relative pose of the right camera wrt to the left camera:\n"
			"# This assumes that both camera frames are such that +Z points\n"
			"# forwards, and +X and +Y to the right and downwards.\n"
			"[CAMERA_PARAMS_LEFT2RIGHT_POSE]\n"
			"translation_only     = [%e %e %e]\n"
			"rotation_matrix_only = %s\n"
			"pose_yaw_pitch_roll  = %s  // (YPR in degrees)\n"
			"pose_quaternion      = %s\n\n"
			,
				RT_YPR.x(),RT_YPR.y(),RT_YPR.z(),
				RT_YPR.getRotationMatrix().inMatlabFormat(13).c_str(),
				RT_YPR.asString().c_str(),
				RT_quat.asString().c_str()
			);
    }

    WX_END_TRY
}

void kinect_calibrate_guiDialog::OnbtnLoadCalibClick(wxCommandEvent& event)
{
	WX_START_TRY

	wxString startPath;
	m_config.Read(_("last_path"),&startPath);

    wxFileDialog  dialog(this, _("Load calibration file"), startPath, _("calib.ini") ,_("Configuration files (*.ini,*.cfg)|*.ini;*.cfg|All files (*.*)|*.*"), wxFD_OPEN |  wxFD_FILE_MUST_EXIST );
    if (dialog.ShowModal() == wxID_OK)
    {
        const string inFil = string( dialog.GetPath().mb_str() );

        mrpt::utils::CConfigFile cfg(inFil);


        m_calib_result.cam_params.leftCamera.loadFromConfigFile("CAMERA_PARAMS_LEFT",cfg);
        m_calib_result.cam_params.rightCamera.loadFromConfigFile("CAMERA_PARAMS_RIGHT",cfg);

        const string sYPR = cfg.read_string("CAMERA_PARAMS_LEFT2RIGHT_POSE","pose_yaw_pitch_roll","", true);

		// Load L->R pose
        mrpt::poses::CPose3D p;
        p.fromString(sYPR);

        // Convert to R->L pose:
        m_calib_result.right2left_camera_pose = -p;

        // Update grid:
		LiveCalibUpdateToGrid();
    }

    WX_END_TRY
}

// Aux. function
void kinect_calibrate_guiDialog::fillGridLine(int r,  const char *label_prefix, const char *label, const std::string &val )
{
	m_grid_live_calib->SetRowLabelValue(r,_U( (std::string(label_prefix)+std::string(".")+std::string(label)).c_str() ) );
	m_grid_live_calib->SetCellValue(r,0,_U(val.c_str()));
}

void kinect_calibrate_guiDialog::LiveCalibGridInitialize()
{
	// Fill all the fields of the calibration:
	m_grid_live_calib->BeginBatch();

	// Clear:
	m_grid_live_calib->DeleteCols(0,m_grid_live_calib->GetNumberCols(),false);
	m_grid_live_calib->DeleteRows(0,m_grid_live_calib->GetNumberRows(),false);

	// Build cells & labels:
	m_grid_live_calib->AppendCols(1,false);
	m_grid_live_calib->AppendRows(24,false);
	m_grid_live_calib->SetColLabelSize(0); // Hide

	m_grid_live_calib->EndBatch();
}

// m_calib_result -> m_grid_live_calib
void kinect_calibrate_guiDialog::LiveCalibUpdateToGrid()
{
	// Fill all the fields of the calibration:
	m_grid_live_calib->BeginBatch();

	// Fill-in data:
	int r=0;
	for (int lr=0;lr<2;lr++)
	{
		const char * sN = (lr==0 ? "Left" : "Right");
		const mrpt::utils::TCamera &c = (lr==0 ? m_calib_result.cam_params.leftCamera : m_calib_result.cam_params.rightCamera);

		fillGridLine(r++, sN,"cx", format("%.4f",c.cx()) );
		fillGridLine(r++, sN,"cy", format("%.4f",c.cy()) );
		fillGridLine(r++, sN,"fx", format("%.4f",c.fx()) );
		fillGridLine(r++, sN,"fy", format("%.4f",c.fy()) );
		fillGridLine(r++, sN,"k1", format("%.4e",c.k1()) );
		fillGridLine(r++, sN,"k2", format("%.4e",c.k2()) );
		fillGridLine(r++, sN,"k3", format("%.4e",c.k3()) );
		fillGridLine(r++, sN,"t1", format("%.4e",c.p1()) );
		fillGridLine(r++, sN,"t2", format("%.4e",c.p2()) );
	}

	// Show the L->R pose (more intuitive to edit by hand):
	const mrpt::poses::CPose3D l2r = - m_calib_result.right2left_camera_pose;
	fillGridLine(r++, "L2R_pose","x", format("%.4f", l2r.x() ) );
	fillGridLine(r++, "L2R_pose","y", format("%.4f", l2r.y() ) );
	fillGridLine(r++, "L2R_pose","z", format("%.4f", l2r.z() ) );
	fillGridLine(r++, "L2R_pose","yaw", format("%.4f", l2r.yaw() ) );
	fillGridLine(r++, "L2R_pose","pitch", format("%.4f", l2r.pitch() ) );
	fillGridLine(r++, "L2R_pose","roll", format("%.4f", l2r.roll() ) );

	m_grid_live_calib->EndBatch();
}


double kinect_calibrate_guiDialog::parseGridLine(int r)
{
	double R;
	const wxString s = m_grid_live_calib->GetCellValue(r,0);

	if (!s.ToDouble(&R))
			throw std::runtime_error( mrpt::format("Error parsing double string: '%s'", (const char*)s.mb_str(wxConvUTF8) ).c_str() );
	else 	return R;
}

// m_grid_live_calib -> m_calib_result
void kinect_calibrate_guiDialog::LiveCalibUpdateFromGrid()
{
	WX_START_TRY

	// Parse data:
	int r=0;
	for (int lr=0;lr<2;lr++)
	{
		mrpt::utils::TCamera &c = (lr==0 ? m_calib_result.cam_params.leftCamera : m_calib_result.cam_params.rightCamera);

		c.cx( parseGridLine(r++) );
		c.cy( parseGridLine(r++) );
		c.fx( parseGridLine(r++) );
		c.fy( parseGridLine(r++) );

		c.k1( parseGridLine(r++) );
		c.k2( parseGridLine(r++) );
		c.k3( parseGridLine(r++) );
		c.p1( parseGridLine(r++) );
		c.p2( parseGridLine(r++) );
	}

	// Show the L->R pose in Kinect coordinates (more intuitive to edit by hand):
	const mrpt::poses::CPose3D l2r(
		parseGridLine(r), parseGridLine(r+1), parseGridLine(r+2),  // x,y,z
		parseGridLine(r+3), parseGridLine(r+4), parseGridLine(r+5)  // yaw,pitch,roll
		);

	const mrpt::poses::CPose3D r2l = (-l2r);
	m_calib_result.right2left_camera_pose = r2l;

	WX_END_TRY
}

void kinect_calibrate_guiDialog::OnPanel5SetFocus(wxFocusEvent& event)
{
}

void kinect_calibrate_guiDialog::Onm_grid_live_calibCellChange(wxGridEvent& event)
{
	this->LiveCalibUpdateFromGrid();
}


void kinect_calibrate_guiDialog::OnbtnHelpLiveCalibClick(wxCommandEvent& event)
{
}

// Recreate 3D view of reconstructed 6D camera poses during calibration:
void kinect_calibrate_guiDialog::CalibUpdate3DViewCameras()
{
	WX_START_TRY

	mrpt::opengl::COpenGLScenePtr scene = mrpt::opengl::COpenGLScene::Create();

	// Ground plane:
	{
		mrpt::opengl::CGridPlaneXYPtr obj = mrpt::opengl::CGridPlaneXY::Create(-10,10, -10,10, 0, 1);
		obj->setColor_u8( TColor(200,200,200) );
		scene->insert(obj);
	}
	// XYZ corner:
	scene->insert( mrpt::opengl::stock_objects::CornerXYZSimple(0.5,2) );

	const unsigned int  check_size_x = edCalibCheckX->GetValue();
	const unsigned int  check_size_y = edCalibCheckY->GetValue();

	double check_squares_length_X_meters,check_squares_length_Y_meters;
	edCalibSizeX->GetValue().ToDouble( &check_squares_length_X_meters );
	edCalibSizeY->GetValue().ToDouble( &check_squares_length_Y_meters );

	check_squares_length_X_meters*=1e-3;
	check_squares_length_Y_meters*=1e-3;

	if (!check_squares_length_X_meters || !check_squares_length_Y_meters) return;

	opengl::CSetOfObjectsPtr gl_objs = opengl::CSetOfObjects::Create();

	opengl::CGridPlaneXYPtr	grid = opengl::CGridPlaneXY::Create(0,check_size_x*check_squares_length_X_meters, 0, check_size_y*check_squares_length_Y_meters, 0, check_squares_length_X_meters );
	gl_objs->insert( grid );

	const size_t N = m_calib_result.left_cam_poses.size();
	ASSERT_EQUAL_(m_calib_result.image_pair_was_used.size(), N);

	for (size_t i=0;i<N;i++)
	{
		if (m_calib_result.image_pair_was_used[i])
		{
			{
				mrpt::opengl::CSetOfObjectsPtr	cor = mrpt::opengl::stock_objects::CornerXYZSimple(0.05f,2);
				cor->setName( mrpt::format("#%u",static_cast<unsigned int>(i) ));
				cor->enableShowName(true);

				mrpt::poses::CPose3D  p = -m_calib_result.left_cam_poses[i]; // Inversed poses are estimated.
				cor->setPose(p);

				gl_objs->insert( cor );
			}
			{
				mrpt::opengl::CSetOfObjectsPtr	cor = mrpt::opengl::stock_objects::CornerXYZSimple(0.05f,2);
				mrpt::poses::CPose3D  p = -(m_calib_result.right2left_camera_pose+m_calib_result.left_cam_poses[i]); // Inversed poses are estimated.
				cor->setPose(p);
				gl_objs->insert( cor );
			}
		}
	}

	gl_objs->setPose( mrpt::poses::CPose3D(0,0,0, DEG2RAD(0),DEG2RAD(180),DEG2RAD(0) ) );
	scene->insert(gl_objs);

	m_plot3D_cameras->m_openGLScene = scene;

	WX_END_TRY
}

void kinect_calibrate_guiDialog::OnNeedsToUpdate6DCamPlot(wxSpinEvent& event)
{
	CalibUpdate3DViewCameras();
	m_plot3D_cameras->Refresh();
}

void kinect_calibrate_guiDialog::OnedCalibSizeXText(wxCommandEvent& event)
{
	CalibUpdate3DViewCameras();
	m_plot3D_cameras->Refresh();
}
