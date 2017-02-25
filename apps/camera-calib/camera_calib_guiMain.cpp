/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
    APPLICATION: Camera calibration GUI
    AUTHOR: Jose Luis Blanco, based on code from the OpenCV library.
  ---------------------------------------------------------------*/


#include "camera_calib_guiMain.h"
#include "CDlgCalibWizardOnline.h"
#include "CDlgPoseEst.h"

//(*InternalHeaders(camera_calib_guiDialog)
#include <wx/settings.h>
#include <wx/font.h>
#include <wx/intl.h>
#include <wx/string.h>
//*)

#include <wx/filedlg.h>
#include <wx/msgdlg.h>
#include <wx/progdlg.h>


#include <mrpt/gui/WxUtils.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>

#include <fstream>
#include <iostream>

#include <Eigen/Dense>
#include <mrpt/vision/pnp_algos.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::vision;
using namespace mrpt::gui;
using namespace std;

#include <mrpt/gui/CMyRedirector.h>

#include "CAboutBox.h"


#define USE_EXTERNAL_STORAGE_IMGS 1

// VARIABLES  ================================

TCalibrationImageList  lst_images;	// Here are all the images: file_name -> data
mrpt::utils::TCamera     camera_params;

// END VARIABLES  ============================


#include "imgs/icono_main.xpm"
#include "../wx-common/mrpt_logo.xpm"

// A custom Art provider for customizing the icons:
class MyArtProvider : public wxArtProvider
{
protected:
    virtual wxBitmap CreateBitmap(const wxArtID& id,
                                  const wxArtClient& client,
                                  const wxSize& size);
};

// CreateBitmap function
wxBitmap MyArtProvider::CreateBitmap(const wxArtID& id,
                                     const wxArtClient& client,
                                     const wxSize& size)
{
    if (id == wxART_MAKE_ART_ID(MAIN_ICON))   return wxBitmap(icono_main_xpm);
    if (id == wxART_MAKE_ART_ID(IMG_MRPT_LOGO))  return wxBitmap(mrpt_logo_xpm);

    // Any wxWidgets icons not implemented here
    // will be provided by the default art provider.
    return wxNullBitmap;
}




//(*IdInit(camera_calib_guiDialog)
const long camera_calib_guiDialog::ID_BUTTON8 = wxNewId();
const long camera_calib_guiDialog::ID_BUTTON1 = wxNewId();
const long camera_calib_guiDialog::ID_BUTTON2 = wxNewId();
const long camera_calib_guiDialog::ID_BUTTON9 = wxNewId();
const long camera_calib_guiDialog::ID_LISTBOX1 = wxNewId();
const long camera_calib_guiDialog::ID_STATICTEXT5 = wxNewId();
const long camera_calib_guiDialog::ID_CHOICE1 = wxNewId();
const long camera_calib_guiDialog::ID_STATICTEXT1 = wxNewId();
const long camera_calib_guiDialog::ID_SPINCTRL1 = wxNewId();
const long camera_calib_guiDialog::ID_STATICTEXT2 = wxNewId();
const long camera_calib_guiDialog::ID_SPINCTRL2 = wxNewId();
const long camera_calib_guiDialog::ID_RADIOBOX1 = wxNewId();
const long camera_calib_guiDialog::ID_STATICTEXT3 = wxNewId();
const long camera_calib_guiDialog::ID_TEXTCTRL1 = wxNewId();
const long camera_calib_guiDialog::ID_STATICTEXT6 = wxNewId();
const long camera_calib_guiDialog::ID_TEXTCTRL3 = wxNewId();
const long camera_calib_guiDialog::ID_CHECKBOX1 = wxNewId();
const long camera_calib_guiDialog::ID_TEXTCTRL2 = wxNewId();
const long camera_calib_guiDialog::ID_BUTTON3 = wxNewId();
const long camera_calib_guiDialog::ID_BUTTON6 = wxNewId();
const long camera_calib_guiDialog::ID_BUTTON7 = wxNewId();
const long camera_calib_guiDialog::ID_BUTTON5 = wxNewId();
const long camera_calib_guiDialog::ID_BUTTON4 = wxNewId();
const long camera_calib_guiDialog::ID_CUSTOM2 = wxNewId();
const long camera_calib_guiDialog::ID_SCROLLEDWINDOW2 = wxNewId();
const long camera_calib_guiDialog::ID_PANEL2 = wxNewId();
const long camera_calib_guiDialog::ID_CUSTOM1 = wxNewId();
const long camera_calib_guiDialog::ID_SCROLLEDWINDOW3 = wxNewId();
const long camera_calib_guiDialog::ID_PANEL3 = wxNewId();
const long camera_calib_guiDialog::ID_XY_GLCANVAS = wxNewId();
const long camera_calib_guiDialog::ID_PANEL1 = wxNewId();
const long camera_calib_guiDialog::ID_NOTEBOOK1 = wxNewId();
const long camera_calib_guiDialog::ID_BUTTON10 = wxNewId();
//*)

BEGIN_EVENT_TABLE(camera_calib_guiDialog,wxDialog)
    //(*EventTable(camera_calib_guiDialog)
    //*)
END_EVENT_TABLE()

camera_calib_guiDialog::camera_calib_guiDialog(wxWindow* parent,wxWindowID id)
{
	// Load my custom icons:
#if wxCHECK_VERSION(2, 8, 0)
    wxArtProvider::Push(new MyArtProvider);
#else
    wxArtProvider::PushProvider(new MyArtProvider);
#endif


	//(*Initialize(camera_calib_guiDialog)
	wxStaticBoxSizer* StaticBoxSizer2;
	wxFlexGridSizer* FlexGridSizer4;
	wxFlexGridSizer* FlexGridSizer16;
	wxStaticBoxSizer* StaticBoxSizer4;
	wxFlexGridSizer* FlexGridSizer10;
	wxFlexGridSizer* FlexGridSizer3;
	wxFlexGridSizer* FlexGridSizer5;
	wxFlexGridSizer* FlexGridSizer9;
	wxFlexGridSizer* FlexGridSizer2;
	wxFlexGridSizer* FlexGridSizer7;
	wxStaticBoxSizer* StaticBoxSizer3;
	wxFlexGridSizer* FlexGridSizer15;
	wxFlexGridSizer* FlexGridSizer18;
	wxFlexGridSizer* FlexGridSizer8;
	wxFlexGridSizer* FlexGridSizer13;
	wxFlexGridSizer* FlexGridSizer12;
	wxFlexGridSizer* FlexGridSizer6;
	wxStaticBoxSizer* StaticBoxSizer1;
	wxFlexGridSizer* FlexGridSizer1;
	wxFlexGridSizer* FlexGridSizer17;
	wxStaticBoxSizer* StaticBoxSizer5;

	Create(parent, id, _("Camera calibration GUI - Part of the MRPT project"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE|wxRESIZE_BORDER|wxMAXIMIZE_BOX, _T("id"));
	FlexGridSizer1 = new wxFlexGridSizer(1, 2, 0, 0);
	FlexGridSizer1->AddGrowableCol(1);
	FlexGridSizer1->AddGrowableRow(0);
	FlexGridSizer2 = new wxFlexGridSizer(3, 1, 0, 0);
	FlexGridSizer2->AddGrowableCol(0);
	FlexGridSizer2->AddGrowableRow(0);
	FlexGridSizer2->AddGrowableRow(2);
	StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("List of images"));
	FlexGridSizer4 = new wxFlexGridSizer(2, 1, 0, 0);
	FlexGridSizer4->AddGrowableCol(0);
	FlexGridSizer4->AddGrowableRow(1);
	FlexGridSizer5 = new wxFlexGridSizer(0, 4, 0, 0);
	btnCaptureNow = new wxButton(this, ID_BUTTON8, _("Grab now..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON8"));
	wxFont btnCaptureNowFont(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxBOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
	btnCaptureNow->SetFont(btnCaptureNowFont);
  	FlexGridSizer5->Add(btnCaptureNow, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	btnPoseEstimateNow = new wxButton(this, ID_BUTTON10, _("Pose Est. now..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON10"));
	wxFont btnPoseEstimateNowFont(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
	btnPoseEstimateNow->SetFont(btnPoseEstimateNowFont);
	FlexGridSizer5->Add(btnPoseEstimateNow, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	Button11 = new wxButton(this, ID_BUTTON1, _("Add image(s)..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
	FlexGridSizer5->Add(Button11, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	Button22 = new wxButton(this, ID_BUTTON2, _("Clear all"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
	FlexGridSizer5->Add(Button22, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	btnSaveImages = new wxButton(this, ID_BUTTON9, _("Save all..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON9"));
	btnSaveImages->Disable();
	FlexGridSizer5->Add(btnSaveImages, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer4->Add(FlexGridSizer5, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
	FlexGridSizer15 = new wxFlexGridSizer(1, 2, 0, 0);
	FlexGridSizer15->AddGrowableCol(0);
	FlexGridSizer15->AddGrowableRow(0);
	lbFiles = new wxListBox(this, ID_LISTBOX1, wxDefaultPosition, wxSize(294,84), 0, 0, wxLB_ALWAYS_SB|wxVSCROLL|wxHSCROLL|wxALWAYS_SHOW_SB, wxDefaultValidator, _T("ID_LISTBOX1"));
	FlexGridSizer15->Add(lbFiles, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer16 = new wxFlexGridSizer(0, 1, 0, 0);
	StaticText5 = new wxStaticText(this, ID_STATICTEXT5, _("Zoom:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT5"));
	FlexGridSizer16->Add(StaticText5, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	cbZoom = new wxChoice(this, ID_CHOICE1, wxDefaultPosition, wxDefaultSize, 0, 0, 0, wxDefaultValidator, _T("ID_CHOICE1"));
	cbZoom->Append(_("25%"));
	cbZoom->Append(_("50%"));
	cbZoom->Append(_("75%"));
	cbZoom->SetSelection( cbZoom->Append(_("100%")) );
	cbZoom->Append(_("150%"));
	cbZoom->Append(_("200%"));
	cbZoom->Append(_("300%"));
	cbZoom->Append(_("400%"));
	cbZoom->Append(_("500%"));
	FlexGridSizer16->Add(cbZoom, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer15->Add(FlexGridSizer16, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer4->Add(FlexGridSizer15, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	StaticBoxSizer1->Add(FlexGridSizer4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer2->Add(StaticBoxSizer1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
	StaticBoxSizer3 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Checkerboard detection parameters"));
	FlexGridSizer6 = new wxFlexGridSizer(2, 2, 0, 0);
	FlexGridSizer6->AddGrowableCol(0);
	FlexGridSizer6->AddGrowableCol(1);
	StaticBoxSizer4 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Number of inner corners: "));
	FlexGridSizer17 = new wxFlexGridSizer(1, 4, 0, 0);
	StaticText1 = new wxStaticText(this, ID_STATICTEXT1, _("In X:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
	FlexGridSizer17->Add(StaticText1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	edSizeX = new wxSpinCtrl(this, ID_SPINCTRL1, _T("9"), wxDefaultPosition, wxSize(50,-1), 0, 1, 200, 5, _T("ID_SPINCTRL1"));
	edSizeX->SetValue(_T("9"));
	FlexGridSizer17->Add(edSizeX, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText2 = new wxStaticText(this, ID_STATICTEXT2, _("In Y:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
	FlexGridSizer17->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	edSizeY = new wxSpinCtrl(this, ID_SPINCTRL2, _T("6"), wxDefaultPosition, wxSize(50,-1), 0, 1, 200, 8, _T("ID_SPINCTRL2"));
	edSizeY->SetValue(_T("6"));
	FlexGridSizer17->Add(edSizeY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticBoxSizer4->Add(FlexGridSizer17, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer6->Add(StaticBoxSizer4, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 2);
	wxString __wxRadioBoxChoices_1[2] =
	{
		_("OpenCV\'s default"),
		_("Scaramuzza et al.\'s")
	};
	rbMethod = new wxRadioBox(this, ID_RADIOBOX1, _(" Detector method: "), wxDefaultPosition, wxDefaultSize, 2, __wxRadioBoxChoices_1, 1, 0, wxDefaultValidator, _T("ID_RADIOBOX1"));
	FlexGridSizer6->Add(rbMethod, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
	StaticBoxSizer5 = new wxStaticBoxSizer(wxHORIZONTAL, this, _(" Size of quads (in mm): "));
	FlexGridSizer18 = new wxFlexGridSizer(1, 4, 0, 0);
	StaticText3 = new wxStaticText(this, ID_STATICTEXT3, _("In X:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
	FlexGridSizer18->Add(StaticText3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	edLengthX = new wxTextCtrl(this, ID_TEXTCTRL1, _("25.0"), wxDefaultPosition, wxSize(40,-1), 0, wxDefaultValidator, _T("ID_TEXTCTRL1"));
	FlexGridSizer18->Add(edLengthX, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText6 = new wxStaticText(this, ID_STATICTEXT6, _("In Y:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT6"));
	FlexGridSizer18->Add(StaticText6, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	edLengthY = new wxTextCtrl(this, ID_TEXTCTRL3, _("25.0"), wxDefaultPosition, wxSize(40,-1), 0, wxDefaultValidator, _T("ID_TEXTCTRL3"));
	FlexGridSizer18->Add(edLengthY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticBoxSizer5->Add(FlexGridSizer18, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer6->Add(StaticBoxSizer5, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 2);
	cbNormalize = new wxCheckBox(this, ID_CHECKBOX1, _("Normalize image"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
	cbNormalize->SetValue(true);
	FlexGridSizer6->Add(cbNormalize, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticBoxSizer3->Add(FlexGridSizer6, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer2->Add(StaticBoxSizer3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
	StaticBoxSizer2 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Calibration"));
	FlexGridSizer7 = new wxFlexGridSizer(2, 1, 0, 0);
	FlexGridSizer7->AddGrowableCol(0);
	FlexGridSizer7->AddGrowableRow(0);
	FlexGridSizer9 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer9->AddGrowableCol(0);
	FlexGridSizer9->AddGrowableRow(0);
	txtLog = new wxTextCtrl(this, ID_TEXTCTRL2, _("(Calibration results)"), wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE|wxTE_READONLY|wxHSCROLL|wxVSCROLL, wxDefaultValidator, _T("ID_TEXTCTRL2"));
	wxFont txtLogFont = wxSystemSettings::GetFont(wxSYS_OEM_FIXED_FONT);
	if ( !txtLogFont.Ok() ) txtLogFont = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
	txtLogFont.SetPointSize(9);
	txtLog->SetFont(txtLogFont);
	FlexGridSizer9->Add(txtLog, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer7->Add(FlexGridSizer9, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer8 = new wxFlexGridSizer(2, 3, 0, 0);
	FlexGridSizer8->AddGrowableCol(0);
	FlexGridSizer8->AddGrowableCol(1);
	btnRunCalib = new wxButton(this, ID_BUTTON3, _("Calibrate"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON3"));
	btnRunCalib->SetDefault();
	wxFont btnRunCalibFont(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxBOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
	btnRunCalib->SetFont(btnRunCalibFont);
	FlexGridSizer8->Add(btnRunCalib, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	btnSave = new wxButton(this, ID_BUTTON6, _("Save matrices..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON6"));
	FlexGridSizer8->Add(btnSave, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer8->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	btnManualRect = new wxButton(this, ID_BUTTON7, _("Manual params..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON7"));
	FlexGridSizer8->Add(btnManualRect, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	btnAbout = new wxButton(this, ID_BUTTON5, _("About"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON5"));
	FlexGridSizer8->Add(btnAbout, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	btnClose = new wxButton(this, ID_BUTTON4, _("Quit"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON4"));
	FlexGridSizer8->Add(btnClose, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer7->Add(FlexGridSizer8, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	StaticBoxSizer2->Add(FlexGridSizer7, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer2->Add(StaticBoxSizer2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
	FlexGridSizer1->Add(FlexGridSizer2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer3 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer3->AddGrowableCol(0);
	FlexGridSizer3->AddGrowableRow(0);
	Notebook1 = new wxNotebook(this, ID_NOTEBOOK1, wxDefaultPosition, wxSize(719,570), 0, _T("ID_NOTEBOOK1"));
	Panel2 = new wxPanel(Notebook1, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
	FlexGridSizer13 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer13->AddGrowableCol(0);
	FlexGridSizer13->AddGrowableRow(0);
	ScrolledWindow2 = new wxScrolledWindow(Panel2, ID_SCROLLEDWINDOW2, wxDefaultPosition, wxDefaultSize, wxHSCROLL|wxVSCROLL, _T("ID_SCROLLEDWINDOW2"));
	FlexGridSizer11 = new wxFlexGridSizer(0, 1, 0, 0);
	FlexGridSizer11->AddGrowableCol(0);
	FlexGridSizer11->AddGrowableRow(0);
	bmpOriginal = new mrpt::gui::wxMRPTImageControl(ScrolledWindow2,ID_CUSTOM2,wxDefaultPosition.x,wxDefaultPosition.y,wxSize(640,480).GetWidth(), wxSize(640,480).GetHeight() );
	FlexGridSizer11->Add(bmpOriginal, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 0);
	ScrolledWindow2->SetSizer(FlexGridSizer11);
	FlexGridSizer11->Fit(ScrolledWindow2);
	FlexGridSizer11->SetSizeHints(ScrolledWindow2);
	FlexGridSizer13->Add(ScrolledWindow2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	Panel2->SetSizer(FlexGridSizer13);
	FlexGridSizer13->Fit(Panel2);
	FlexGridSizer13->SetSizeHints(Panel2);
	Panel3 = new wxPanel(Notebook1, ID_PANEL3, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL3"));
	FlexGridSizer10 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer10->AddGrowableCol(0);
	FlexGridSizer10->AddGrowableRow(0);
	ScrolledWindow3 = new wxScrolledWindow(Panel3, ID_SCROLLEDWINDOW3, wxDefaultPosition, wxDefaultSize, wxHSCROLL|wxVSCROLL, _T("ID_SCROLLEDWINDOW3"));
	FlexGridSizer14 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer14->AddGrowableCol(0);
	FlexGridSizer14->AddGrowableRow(0);
	bmpRectified = new mrpt::gui::wxMRPTImageControl(ScrolledWindow3,ID_CUSTOM1,wxDefaultPosition.x,wxDefaultPosition.y,wxSize(640,480).GetWidth(), wxSize(640,480).GetHeight() );
	FlexGridSizer14->Add(bmpRectified, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 0);
	ScrolledWindow3->SetSizer(FlexGridSizer14);
	FlexGridSizer14->Fit(ScrolledWindow3);
	FlexGridSizer14->SetSizeHints(ScrolledWindow3);
	FlexGridSizer10->Add(ScrolledWindow3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	Panel3->SetSizer(FlexGridSizer10);
	FlexGridSizer10->Fit(Panel3);
	FlexGridSizer10->SetSizeHints(Panel3);
	Panel1 = new wxPanel(Notebook1, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
	FlexGridSizer12 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer12->AddGrowableCol(0);
	FlexGridSizer12->AddGrowableRow(0);
	m_3Dview = new CMyGLCanvas(Panel1,ID_XY_GLCANVAS,wxDefaultPosition,wxSize(-1,300),wxTAB_TRAVERSAL,_T("ID_XY_GLCANVAS"));
	FlexGridSizer12->Add(m_3Dview, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	Panel1->SetSizer(FlexGridSizer12);
	FlexGridSizer12->Fit(Panel1);
	FlexGridSizer12->SetSizeHints(Panel1);
	Notebook1->AddPage(Panel2, _("Original Image"), false);
	Notebook1->AddPage(Panel3, _("Rectified image and reprojected points"), true);
	Notebook1->AddPage(Panel1, _("3D view"), false);
	FlexGridSizer3->Add(Notebook1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
	FlexGridSizer1->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	SetSizer(FlexGridSizer1);
	FlexGridSizer1->Fit(this);
	FlexGridSizer1->SetSizeHints(this);

	Connect(ID_BUTTON8,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&camera_calib_guiDialog::OnbtnCaptureNowClick);
	Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&camera_calib_guiDialog::OnAddImage);
	Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&camera_calib_guiDialog::OnListClear);
	Connect(ID_BUTTON9,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&camera_calib_guiDialog::OnbtnSaveImagesClick);
	Connect(ID_LISTBOX1,wxEVT_COMMAND_LISTBOX_SELECTED,(wxObjectEventFunction)&camera_calib_guiDialog::OnlbFilesSelect);
	Connect(ID_CHOICE1,wxEVT_COMMAND_CHOICE_SELECTED,(wxObjectEventFunction)&camera_calib_guiDialog::OncbZoomSelect);
	Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&camera_calib_guiDialog::OnbtnRunCalibClick);
	Connect(ID_BUTTON6,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&camera_calib_guiDialog::OnbtnSaveClick);
	Connect(ID_BUTTON7,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&camera_calib_guiDialog::OnbtnManualRectClick);
	Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&camera_calib_guiDialog::OnbtnAboutClick);
	Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&camera_calib_guiDialog::OnbtnCloseClick);
	Connect(ID_BUTTON10,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&camera_calib_guiDialog::OnbtnPoseEstimateNowClick);
	//*)


	camera_params.intrinsicParams(0,0) = 0; // Indicate calib didn't run yet.

	wxIcon icon;
	icon.CopyFromBitmap( wxBitmap(wxImage( icono_main_xpm )) );
	this->SetIcon( icon );

	this->show3Dview(); // Empty 3D scene

	Center();
	this->SetTitle( _U( format("Camera calibration %s - Part of the MRPT project",CAMERA_CALIB_GUI_VERSION).c_str() ) );
	Maximize();
}

camera_calib_guiDialog::~camera_calib_guiDialog()
{
    //(*Destroy(camera_calib_guiDialog)
    //*)
	this->clearListImages();
}


// Ask the user for new files to add to the list:
void camera_calib_guiDialog::OnAddImage(wxCommandEvent& event)
{
	try
	{

	wxFileDialog 	dlg(
		this,
		_("Select image(s) to open"),
		_("."),
		_(""),
		_("Image files (*.bmp;*.png;*.jpg)|*.bmp;*.png;*.jpg|All files (*.*)|*.*"),
		wxFD_OPEN | wxFD_FILE_MUST_EXIST | wxFD_MULTIPLE | wxFD_PREVIEW );

	if (wxID_OK!=dlg.ShowModal()) return;

	wxBusyCursor waitcur;

	wxArrayString files;
	dlg.GetPaths(files);

	wxProgressDialog    progDia(
		wxT("Adding image files"),
		wxT("Processing..."),
		files.Count() , // range
		this, // parent
		wxPD_CAN_ABORT |
		wxPD_APP_MODAL |
		wxPD_SMOOTH |
		wxPD_AUTO_HIDE |
		wxPD_ELAPSED_TIME |
		wxPD_ESTIMATED_TIME |
		wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages


	int counter_loops=0;

	for (unsigned int i=0;i<files.Count();i++)
	{
		if (counter_loops++ % 5 == 0)
		{
			if (!progDia.Update( i ))
				break;
			wxTheApp->Yield();  // Let the app. process messages
		}

		const string fil = string(files[i].mb_str());

		TImageCalibData	dat;

#if USE_EXTERNAL_STORAGE_IMGS
		// Optimization: Use external storage:
		// ---------------------------------------
		//string tmp_check     = mrpt::system::getTempFileName()+".jpg";
		//string tmp_rectified = mrpt::system::getTempFileName()+".jpg";
		//dat.img_original.saveToFile(tmp_check);
		//dat.img_original.saveToFile(tmp_rectified);

		// mark all imgs. as external storage:
		dat.img_original.setExternalStorage(fil);
		//dat.img_checkboard.setExternalStorage(tmp_check);
		//dat.img_rectified.setExternalStorage(tmp_rectified);

		dat.img_original.unload();
		//dat.img_checkboard.unload();
		//dat.img_rectified.unload();
#else
		// All in memory:
		if (!dat.img_original.loadFromFile(fil))
		{
			wxMessageBox(_U(format("Error loading file: %s",fil.c_str()).c_str()), _("Error"));
			this->updateListOfImages();
			return;
		}

#endif

		lst_images[fil] = dat;
	}

	this->updateListOfImages();

	}
	catch(std::exception &e)
	{
		wxMessageBox(_U(e.what()),_("Error"),wxICON_INFORMATION,this);
	}
}


void camera_calib_guiDialog::clearListImages()
{
	lst_images.clear();
}

// Clear list of images.
void camera_calib_guiDialog::OnListClear(wxCommandEvent& event)
{
	this->clearListImages();
	this->updateListOfImages();
}

void camera_calib_guiDialog::OnbtnRunCalibClick(wxCommandEvent& event)
{
	try
	{

	txtLog->Clear();

	CMyRedirector	redire(txtLog, true, 10);

	const unsigned int  check_size_x = edSizeX->GetValue();
	const unsigned int  check_size_y = edSizeY->GetValue();
	const double        check_squares_length_X_meters = 0.001 * atof( string(edLengthX->GetValue().mb_str()).c_str() );
	const double        check_squares_length_Y_meters = 0.001 * atof( string(edLengthY->GetValue().mb_str()).c_str() );

	const bool			normalize_image = cbNormalize->GetValue();

	const bool			useScaramuzzaAlternativeDetector =  rbMethod->GetSelection() == 1;

	wxBusyCursor waitcur;

	bool res = mrpt::vision::checkerBoardCameraCalibration(
		lst_images,
		check_size_x,
		check_size_y,
		check_squares_length_X_meters,
		check_squares_length_Y_meters,
		camera_params,
		normalize_image,
		NULL /* MSE */,
		false /* skip draw */,
		useScaramuzzaAlternativeDetector);


	refreshDisplayedImage();

	if (!res)
		wxMessageBox(_("Calibration finished with error: Please check the text log to see what's wrong"), _("Error"));

	if (res)
		show3Dview();

	}
	catch(std::exception &e)
	{
		wxMessageBox(_U(e.what()),_("Error"),wxICON_INFORMATION,this);
	}
}

void camera_calib_guiDialog::OnbtnCloseClick(wxCommandEvent& event)
{
	Close();
}

void camera_calib_guiDialog::OnbtnAboutClick(wxCommandEvent& event)
{
	CAboutBox	dlg(this);
	dlg.ShowModal();
}

// save matrices:
void camera_calib_guiDialog::OnbtnSaveClick(wxCommandEvent& event)
{
	if (camera_params.intrinsicParams(0,0)==0)
	{
		wxMessageBox(_("Run the calibration first"),_("Error"));
		return;
	}

	{
		wxFileDialog 	dlg(
			this,
			_("Save intrinsic parameters matrix"),
			_("."),
			_("intrinsic_matrix.txt"),
			_("Text files (*.txt)|*.txt|All files (*.*)|*.*"),
			wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

		if (wxID_OK!=dlg.ShowModal()) return;

		camera_params.intrinsicParams.saveToTextFile(	string(dlg.GetPath().mb_str()) );
	}

	{
		wxFileDialog 	dlg(
			this,
			_("Save distortion parameters"),
			_("."),
			_("distortion_matrix.txt"),
			_("Text files (*.txt)|*.txt|All files (*.*)|*.*"),
			wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

		if (wxID_OK!=dlg.ShowModal()) return;

		CMatrixDouble  M(1,5);
		for (unsigned i=0;i<5;i++)
			M(0,i) = camera_params.dist[i];

		M.saveToTextFile( string(dlg.GetPath().mb_str()) );
	}
}

// Update the listbox from lst_img_files
void camera_calib_guiDialog::updateListOfImages()
{
	lbFiles->Clear();
	for (TCalibrationImageList::iterator s=lst_images.begin();s!=lst_images.end();++s)
		lbFiles->Append(_U(s->first.c_str()));

	btnSaveImages->Enable( !lst_images.empty() );

	refreshDisplayedImage();
}


// Shows the image selected in the listbox:
void camera_calib_guiDialog::refreshDisplayedImage()
{
	try
	{

	if (!lbFiles->GetCount())
	{
		// No images:
		return;
	}


	// Assure there's one selected:
	if (lbFiles->GetSelection()==wxNOT_FOUND)
		lbFiles->SetSelection(0);

	const string selFile = string(lbFiles->GetStringSelection().mb_str());

	TCalibrationImageList::iterator it = lst_images.find(selFile);
	if (it==lst_images.end()) return;

	// Zoom:
	const std::string strZoom = string(cbZoom->GetStringSelection().mb_str());
	const double zoomVal = 0.01*atof(strZoom.c_str());

	ASSERT_(zoomVal>0 && zoomVal<30)


	TImageSize  imgSizes(0,0);

	// Generate the images on-the-fly:
	CImage  imgOrgColor;
	it->second.img_original.colorImage(imgOrgColor);

	imgSizes = imgOrgColor.getSize();

	// Rectify:
	CImage  imgRect;
	if (camera_params.intrinsicParams(0,0)==0)
	{
		// Not calibrated yet:
		imgRect = imgOrgColor;
		imgRect.scaleImage(imgSizes.x*zoomVal,imgSizes.y*zoomVal, IMG_INTERP_NN);
	}
	else
	{
		imgOrgColor.rectifyImage(imgRect,camera_params);
		imgRect.scaleImage(imgSizes.x*zoomVal,imgSizes.y*zoomVal, IMG_INTERP_NN);

		// Draw reprojected:
		for (unsigned int k=0;k<it->second.projectedPoints_undistorted.size();k++)
			imgRect.drawCircle( zoomVal*it->second.projectedPoints_undistorted[k].x, zoomVal*it->second.projectedPoints_undistorted[k].y, 4, TColor(0,255,64) );

		imgRect.drawCircle( 10,10, 4, TColor(0,255,64) );
		imgRect.textOut(18,4,"Reprojected corners",TColor::white);
	}

	// Zoom images:
	imgOrgColor.scaleImage(imgSizes.x*zoomVal,imgSizes.y*zoomVal, IMG_INTERP_NN);

	imgSizes = imgOrgColor.getSize();

	CImage  imgCheck = imgOrgColor;

	// Draw the board:
	for (unsigned int k=0;k<it->second.detected_corners.size();k++)
	{
		imgCheck.cross(it->second.detected_corners[k].x *zoomVal, it->second.detected_corners[k].y *zoomVal, TColor::blue, '+', 3 );
		imgCheck.drawCircle( it->second.projectedPoints_distorted[k].x*zoomVal, it->second.projectedPoints_distorted[k].y*zoomVal, 4, TColor(0,255,64) );
	}
	imgCheck.drawCircle( 10,10, 4, TColor(0,255,64) );
	imgCheck.textOut(18,4,"Reprojected corners",TColor::white);

	imgCheck.cross( 10,30, TColor::blue, '+', 3 );
	imgCheck.textOut(18,24,"Detected corners",TColor::white);


	this->bmpOriginal->AssignImage( imgCheck );
	this->bmpRectified->AssignImage( imgRect );

	it->second.img_original.unload();


	// Plot:

	this->bmpOriginal->SetMinSize(wxSize(imgSizes.x,imgSizes.y));
	this->bmpOriginal->SetMaxSize(wxSize(imgSizes.x,imgSizes.y));
	this->bmpOriginal->SetSize(imgSizes.x,imgSizes.y);

	this->bmpRectified->SetMinSize(wxSize(imgSizes.x,imgSizes.y));
	this->bmpRectified->SetMaxSize(wxSize(imgSizes.x,imgSizes.y));
	this->bmpRectified->SetSize(imgSizes.x,imgSizes.y);

	this->FlexGridSizer11->RecalcSizes();
	this->FlexGridSizer14->RecalcSizes();

	//this->ScrolledWindow2->SetVirtualSize(wxSize(imgSizes.x,imgSizes.y));
	//this->ScrolledWindow3->SetVirtualSize(wxSize(imgSizes.x,imgSizes.y));
	this->ScrolledWindow2->SetScrollbars(1,1,imgSizes.x,imgSizes.y);
	this->ScrolledWindow3->SetScrollbars(1,1,imgSizes.x,imgSizes.y);

	this->bmpOriginal->Refresh(false);
	this->bmpRectified->Refresh(false);

	}
	catch(std::exception &e)
	{
		wxMessageBox(_U(e.what()),_("Error"),wxICON_INFORMATION,this);
	}
}

void camera_calib_guiDialog::OnlbFilesSelect(wxCommandEvent& event)
{
	refreshDisplayedImage();
}

void camera_calib_guiDialog::show3Dview()
{
	mrpt::opengl::COpenGLScenePtr	scene = mrpt::opengl::COpenGLScene::Create();

	const unsigned int  check_size_x = edSizeX->GetValue();
	const unsigned int  check_size_y = edSizeY->GetValue();
	const double        check_squares_length_X_meters = 0.001 * atof( string(edLengthX->GetValue().mb_str()).c_str() );
	const double        check_squares_length_Y_meters = 0.001 * atof( string(edLengthY->GetValue().mb_str()).c_str() );

	if (!check_squares_length_X_meters || !check_squares_length_Y_meters) return;

	opengl::CGridPlaneXYPtr	grid = opengl::CGridPlaneXY::Create(0,check_size_x*check_squares_length_X_meters, 0, check_size_y*check_squares_length_Y_meters, 0, check_squares_length_X_meters );
	scene->insert( grid );

	for (TCalibrationImageList::iterator it=lst_images.begin();it!=lst_images.end();++it)
	{
		if (!it->second.detected_corners.empty())
		{
			mrpt::opengl::CSetOfObjectsPtr	cor = mrpt::opengl::stock_objects::CornerXYZ();
			cor->setName( mrpt::system::extractFileName(it->first) );
			cor->enableShowName(true);
			cor->setScale(0.1f);
			cor->setPose( it->second.reconstructed_camera_pose );

			scene->insert( cor );
		}
	}

	//scene->insert( mrpt::opengl::stock_objects::CornerXYZ() );

	this->m_3Dview->m_openGLScene = scene;
	this->m_3Dview->Refresh();

}

// Enter calib. params manually:
void camera_calib_guiDialog::OnbtnManualRectClick(wxCommandEvent& event)
{
	wxMessageBox(_("Please, enter calibration parameters manually next to overpass automatically obtained parameters."),_("Manual parameters"));

	wxString s;

	if (camera_params.intrinsicParams(0,0)==0)
	{
		wxMessageBox(_("Run the calibration first"),_("Error"));
		return;
	}


	s = wxGetTextFromUser(_("Focus length in X pixel size (fx):"),_("Manual parameters"),wxString::Format(wxT("%.07f"),camera_params.intrinsicParams(0,0)), this );
	if (s.IsEmpty()) return;
	if (!s.ToDouble(&camera_params.intrinsicParams(0,0))) { wxMessageBox(_("Invalid number")); return; }

	s = wxGetTextFromUser(_("Focus length in Y pixel size (fy):"),_("Manual parameters"),wxString::Format(wxT("%.07f"),camera_params.intrinsicParams(1,1)), this );
	if (s.IsEmpty()) return;
	if (!s.ToDouble(&camera_params.intrinsicParams(1,1))) { wxMessageBox(_("Invalid number")); return; }

	s = wxGetTextFromUser(_("Image center X (cx):"),_("Manual parameters"),wxString::Format(wxT("%.07f"),camera_params.intrinsicParams(0,2)), this );
	if (s.IsEmpty()) return;
	if (!s.ToDouble(&camera_params.intrinsicParams(0,2))) { wxMessageBox(_("Invalid number")); return; }

	s = wxGetTextFromUser(_("Image center Y (cy):"),_("Manual parameters"),wxString::Format(wxT("%.07f"),camera_params.intrinsicParams(1,2)), this );
	if (s.IsEmpty()) return;
	if (!s.ToDouble(&camera_params.intrinsicParams(1,2))) { wxMessageBox(_("Invalid number")); return; }



	s = wxGetTextFromUser(_("Distortion param p1:"),_("Manual parameters"),wxString::Format(wxT("%.07f"),camera_params.dist[0]), this );
	if (s.IsEmpty()) return;
	if (!s.ToDouble(&camera_params.dist[0])) { wxMessageBox(_("Invalid number")); return; }

	s = wxGetTextFromUser(_("Distortion param p2:"),_("Manual parameters"),wxString::Format(wxT("%.07f"),camera_params.dist[1]), this );
	if (s.IsEmpty()) return;
	if (!s.ToDouble(&camera_params.dist[1])) { wxMessageBox(_("Invalid number")); return; }

	s = wxGetTextFromUser(_("Distortion param k1:"),_("Manual parameters"),wxString::Format(wxT("%.07f"),camera_params.dist[2]), this );
	if (s.IsEmpty()) return;
	if (!s.ToDouble(&camera_params.dist[2])) { wxMessageBox(_("Invalid number")); return; }

	s = wxGetTextFromUser(_("Distortion param k2:"),_("Manual parameters"),wxString::Format(wxT("%.07f"),camera_params.dist[3]), this );
	if (s.IsEmpty()) return;
	if (!s.ToDouble(&camera_params.dist[3])) { wxMessageBox(_("Invalid number")); return; }


	refreshDisplayedImage();
}

void camera_calib_guiDialog::OnbtnCaptureNowClick(wxCommandEvent& event)
{
  	CDlgCalibWizardOnline dlg(this);

	// Set pattern params:
	dlg.edLengthX->SetValue( this->edLengthX->GetValue() );
	dlg.edLengthY->SetValue( this->edLengthY->GetValue() );
	dlg.edSizeX->SetValue( this->edSizeX->GetValue() );
	dlg.edSizeY->SetValue( this->edSizeY->GetValue() );
	dlg.cbNormalize->SetValue( this->cbNormalize->GetValue() );


	// Run:
	dlg.ShowModal();

	// Get params:
	this->edLengthX->SetValue( dlg.edLengthX->GetValue() );
	this->edLengthY->SetValue( dlg.edLengthY->GetValue() );
	this->edSizeX->SetValue( dlg.edSizeX->GetValue() );
	this->edSizeY->SetValue( dlg.edSizeY->GetValue() );
	this->cbNormalize->SetValue( dlg.cbNormalize->GetValue() );

	// Get images:
	lst_images = dlg.m_calibFrames;
	this->updateListOfImages();


}

void camera_calib_guiDialog::OnbtnPoseEstimateNowClick(wxCommandEvent& event)
{
	// Compute pose using PnP Algorithms toolkit

  	CDlgPoseEst dlg(this);

	// Set pattern params:
	dlg.edLengthX->SetValue( this->edLengthX->GetValue() );
	dlg.edLengthY->SetValue( this->edLengthY->GetValue() );
	dlg.edSizeX->SetValue( this->edSizeX->GetValue() );
	dlg.edSizeY->SetValue( this->edSizeY->GetValue() );
	dlg.cbNormalize->SetValue( this->cbNormalize->GetValue() );


	// Run:
	dlg.ShowModal();

	// Get params:
	this->edLengthX->SetValue( dlg.edLengthX->GetValue() );
	this->edLengthY->SetValue( dlg.edLengthY->GetValue() );
	this->edSizeX->SetValue( dlg.edSizeX->GetValue() );
	this->edSizeY->SetValue( dlg.edSizeY->GetValue() );
	this->cbNormalize->SetValue( dlg.cbNormalize->GetValue() );

	// Get images:
	lst_images = dlg.m_calibFrames;
	this->updateListOfImages();
}

void camera_calib_guiDialog::OnbtnSaveImagesClick(wxCommandEvent& event)
{
	try
	{
		if (lst_images.empty()) return;

		wxDirDialog  dlg(this,_("Select the directory where to save the images"),_("."));

		if (dlg.ShowModal()==wxID_OK)
		{
			string dir = string(dlg.GetPath().mb_str());

			for (TCalibrationImageList::iterator s=lst_images.begin();s!=lst_images.end();++s)
				s->second.img_original.saveToFile( dir+string("/")+s->first+string(".png") );
		}
	}
	catch(std::exception &e)
	{
		wxMessageBox(_U(e.what()),_("Error"),wxICON_INFORMATION,this);
	}
}

void camera_calib_guiDialog::OncbZoomSelect(wxCommandEvent& event)
{
	refreshDisplayedImage();
}
