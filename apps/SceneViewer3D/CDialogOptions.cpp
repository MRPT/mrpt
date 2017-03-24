/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

//#include "_DSceneViewerMain.h"

#include <wx/app.h>

#include <wx/string.h>
#include <wx/intl.h>
#include <wx/bitmap.h>
#include <wx/icon.h>
#include <wx/image.h>
#include <wx/artprov.h>
#include <wx/msgdlg.h>
#include <wx/filedlg.h>
#include <wx/progdlg.h>
#include <wx/imaglist.h>
#include <wx/busyinfo.h>
#include <wx/log.h>
#include <wx/textdlg.h>
#include <wx/dirdlg.h>
#include <wx/colordlg.h>
#include <wx/dcmemory.h>

#include "CDialogOptions.h"
#include "_DSceneViewerMain.h"

//(*InternalHeaders(CDialogOptions)
#include <wx/intl.h>
#include <wx/string.h>
//*)

//(*IdInit(CDialogOptions)
const long CDialogOptions::ID_STATICTEXT1 = wxNewId();
const long CDialogOptions::ID_SPINCTRL1 = wxNewId();
const long CDialogOptions::ID_CHECKBOX1 = wxNewId();
const long CDialogOptions::ID_CHECKBOX2 = wxNewId();
const long CDialogOptions::ID_CHECKBOX3 = wxNewId();
const long CDialogOptions::ID_PANEL1 = wxNewId();
const long CDialogOptions::ID_PANEL2 = wxNewId();
const long CDialogOptions::ID_STATICTEXT6 = wxNewId();
const long CDialogOptions::ID_SPINCTRL2 = wxNewId();
const long CDialogOptions::ID_STATICTEXT3 = wxNewId();
const long CDialogOptions::ID_SPINCTRL3 = wxNewId();
const long CDialogOptions::ID_STATICTEXT7 = wxNewId();
const long CDialogOptions::ID_SPINCTRL4 = wxNewId();
const long CDialogOptions::ID_STATICTEXT2 = wxNewId();
const long CDialogOptions::ID_SPINCTRL6 = wxNewId();
const long CDialogOptions::ID_STATICTEXT5 = wxNewId();
const long CDialogOptions::ID_SPINCTRL8 = wxNewId();
const long CDialogOptions::ID_STATICTEXT4 = wxNewId();
const long CDialogOptions::ID_SPINCTRL5 = wxNewId();
const long CDialogOptions::ID_STATICTEXT10 = wxNewId();
const long CDialogOptions::ID_SPINCTRL10 = wxNewId();
const long CDialogOptions::ID_STATICTEXT11 = wxNewId();
const long CDialogOptions::ID_SPINCTRL9 = wxNewId();
const long CDialogOptions::ID_STATICTEXT8 = wxNewId();
const long CDialogOptions::ID_STATICTEXT9 = wxNewId();
const long CDialogOptions::ID_SPINCTRL7 = wxNewId();
const long CDialogOptions::ID_PANEL3 = wxNewId();
const long CDialogOptions::ID_NOTEBOOK1 = wxNewId();
const long CDialogOptions::ID_BUTTON1 = wxNewId();
const long CDialogOptions::ID_BUTTON2 = wxNewId();
//*)

BEGIN_EVENT_TABLE(CDialogOptions,wxDialog)
	//(*EventTable(CDialogOptions)
	//*)
END_EVENT_TABLE()

CDialogOptions::CDialogOptions(wxWindow* parent,wxWindowID id,const wxPoint& pos,const wxSize& size)
{
	//(*Initialize(CDialogOptions)
	wxStaticBoxSizer* StaticBoxSizer2;
	wxFlexGridSizer* FlexGridSizer4;
	wxFlexGridSizer* FlexGridSizer3;
	wxFlexGridSizer* FlexGridSizer5;
	wxFlexGridSizer* FlexGridSizer2;
	wxFlexGridSizer* FlexGridSizer7;
	wxFlexGridSizer* FlexGridSizer8;
	wxFlexGridSizer* FlexGridSizer6;
	wxStaticBoxSizer* StaticBoxSizer1;
	wxFlexGridSizer* FlexGridSizer1;

	Create(parent, wxID_ANY, _("Options"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE, _T("wxID_ANY"));
	FlexGridSizer1 = new wxFlexGridSizer(2, 1, 0, 0);
	FlexGridSizer1->AddGrowableCol(0);
	FlexGridSizer1->AddGrowableRow(0);
	Notebook1 = new wxNotebook(this, ID_NOTEBOOK1, wxDefaultPosition, wxSize(415,251), 0, _T("ID_NOTEBOOK1"));
	Panel1 = new wxPanel(Notebook1, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
	FlexGridSizer3 = new wxFlexGridSizer(4, 1, 0, 0);
	FlexGridSizer3->AddGrowableCol(0);
	FlexGridSizer4 = new wxFlexGridSizer(1, 3, 0, 0);
	StaticText1 = new wxStaticText(Panel1, ID_STATICTEXT1, _("Delay between frames in \"autoplay\" (ms):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
	FlexGridSizer4->Add(StaticText1, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edDelay = new wxSpinCtrl(Panel1, ID_SPINCTRL1, _T("0"), wxDefaultPosition, wxDefaultSize, 0, 0, 10000, 0, _T("ID_SPINCTRL1"));
	edDelay->SetValue(_T("0"));
	FlexGridSizer4->Add(edDelay, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer4->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer3->Add(FlexGridSizer4, 1, wxALL|wxEXPAND, 5);
	cbViewFileName = new wxCheckBox(Panel1, ID_CHECKBOX1, _("View file name overlapped to viewport"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
	cbViewFileName->SetValue(false);
	FlexGridSizer3->Add(cbViewFileName, 1, wxALL|wxEXPAND, 5);
	cbFreeCamera = new wxCheckBox(Panel1, ID_CHECKBOX2, _("Always allow change camera zoom && elevation && azimuth"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX2"));
	cbFreeCamera->SetValue(false);
	FlexGridSizer3->Add(cbFreeCamera, 1, wxALL|wxEXPAND, 5);
	cbFreeCameraNoAzimuth = new wxCheckBox(Panel1, ID_CHECKBOX3, _("Always allow change camera zoom && elevation"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX3"));
	cbFreeCameraNoAzimuth->SetValue(false);
	FlexGridSizer3->Add(cbFreeCameraNoAzimuth, 1, wxALL|wxEXPAND, 5);
	Panel1->SetSizer(FlexGridSizer3);
	FlexGridSizer3->Fit(Panel1);
	FlexGridSizer3->SetSizeHints(Panel1);
	Panel2 = new wxPanel(Notebook1, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
	Panel3 = new wxPanel(Notebook1, ID_PANEL3, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL3"));
	FlexGridSizer5 = new wxFlexGridSizer(0, 1, 0, 0);
	FlexGridSizer5->AddGrowableCol(0);
	StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, Panel3, _("Camera configuration"));
	FlexGridSizer6 = new wxFlexGridSizer(3, 2, 0, 0);
	FlexGridSizer6->AddGrowableCol(1);
	StaticText6 = new wxStaticText(Panel3, ID_STATICTEXT6, _("X"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT6"));
	FlexGridSizer6->Add(StaticText6, 1, wxALL|wxEXPAND, 5);
	SpinCtrl1 = new wxSpinCtrl(Panel3, ID_SPINCTRL2, _T("0"), wxDefaultPosition, wxSize(94,21), 0, -100, 100, 0, _T("ID_SPINCTRL2"));
	SpinCtrl1->SetValue(_T("0"));
	FlexGridSizer6->Add(SpinCtrl1, 1, wxALL|wxEXPAND, 5);
	StaticText3 = new wxStaticText(Panel3, ID_STATICTEXT3, _("Y"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
	FlexGridSizer6->Add(StaticText3, 1, wxALL|wxEXPAND, 5);
	SpinCtrl2 = new wxSpinCtrl(Panel3, ID_SPINCTRL3, _T("0"), wxDefaultPosition, wxSize(93,21), 0, -100, 100, 0, _T("ID_SPINCTRL3"));
	SpinCtrl2->SetValue(_T("0"));
	FlexGridSizer6->Add(SpinCtrl2, 1, wxALL|wxEXPAND, 5);
	StaticText7 = new wxStaticText(Panel3, ID_STATICTEXT7, _("Z"), wxDefaultPosition, wxSize(16,21), 0, _T("ID_STATICTEXT7"));
	FlexGridSizer6->Add(StaticText7, 1, wxALL|wxEXPAND, 5);
	SpinCtrl3 = new wxSpinCtrl(Panel3, ID_SPINCTRL4, _T("0"), wxDefaultPosition, wxSize(64,21), 0, -100, 100, 0, _T("ID_SPINCTRL4"));
	SpinCtrl3->SetValue(_T("0"));
	FlexGridSizer6->Add(SpinCtrl3, 1, wxALL|wxEXPAND, 5);
	StaticBoxSizer1->Add(FlexGridSizer6, 1, wxALL|wxEXPAND, 5);
	FlexGridSizer7 = new wxFlexGridSizer(3, 2, 0, 0);
	FlexGridSizer7->AddGrowableCol(1);
	StaticText2 = new wxStaticText(Panel3, ID_STATICTEXT2, _("Zoom distance"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
	FlexGridSizer7->Add(StaticText2, 1, wxALL|wxEXPAND, 5);
	SpinCtrl5 = new wxSpinCtrl(Panel3, ID_SPINCTRL6, _T("10"), wxDefaultPosition, wxSize(100,21), 0, -100, 100, 10, _T("ID_SPINCTRL6"));
	SpinCtrl5->SetValue(_T("10"));
	FlexGridSizer7->Add(SpinCtrl5, 1, wxALL|wxEXPAND, 5);
	StaticText5 = new wxStaticText(Panel3, ID_STATICTEXT5, _("Elevation degrees"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT5"));
	FlexGridSizer7->Add(StaticText5, 1, wxALL|wxEXPAND, 5);
	SpinCtrl7 = new wxSpinCtrl(Panel3, ID_SPINCTRL8, _T("35"), wxDefaultPosition, wxSize(47,21), 0, 0, 359, 35, _T("ID_SPINCTRL8"));
	SpinCtrl7->SetValue(_T("35"));
	FlexGridSizer7->Add(SpinCtrl7, 1, wxALL|wxEXPAND, 5);
	StaticText4 = new wxStaticText(Panel3, ID_STATICTEXT4, _("Azimuth degrees"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT4"));
	FlexGridSizer7->Add(StaticText4, 1, wxALL|wxEXPAND, 5);
	SpinCtrl4 = new wxSpinCtrl(Panel3, ID_SPINCTRL5, _T("45"), wxDefaultPosition, wxSize(47,21), 0, 0, 359, 45, _T("ID_SPINCTRL5"));
	SpinCtrl4->SetValue(_T("45"));
	FlexGridSizer7->Add(SpinCtrl4, 1, wxALL|wxEXPAND, 5);
	StaticBoxSizer1->Add(FlexGridSizer7, 1, wxALL|wxEXPAND, 5);
	FlexGridSizer5->Add(StaticBoxSizer1, 1, wxALL|wxEXPAND, 5);
	StaticBoxSizer2 = new wxStaticBoxSizer(wxHORIZONTAL, Panel3, _("Azimuth offset (degrees)"));
	FlexGridSizer8 = new wxFlexGridSizer(1, 7, 0, 0);
	FlexGridSizer8->AddGrowableCol(0);
	FlexGridSizer8->AddGrowableCol(1);
	FlexGridSizer8->AddGrowableCol(2);
	FlexGridSizer8->AddGrowableCol(3);
	FlexGridSizer8->AddGrowableCol(4);
	FlexGridSizer8->AddGrowableCol(5);
	FlexGridSizer8->AddGrowableCol(6);
	StaticText10 = new wxStaticText(Panel3, ID_STATICTEXT10, _("-"), wxDefaultPosition, wxSize(15,21), wxALIGN_RIGHT, _T("ID_STATICTEXT10"));
	FlexGridSizer8->Add(StaticText10, 1, wxALL|wxEXPAND, 5);
	SpinCtrl9 = new wxSpinCtrl(Panel3, ID_SPINCTRL10, _T("45"), wxDefaultPosition, wxSize(70,21), 0, 0, 180, 45, _T("ID_SPINCTRL10"));
	SpinCtrl9->SetValue(_T("45"));
	FlexGridSizer8->Add(SpinCtrl9, 1, wxALL|wxEXPAND, 5);
	StaticText11 = new wxStaticText(Panel3, ID_STATICTEXT11, _("step"), wxDefaultPosition, wxSize(30,21), wxALIGN_RIGHT, _T("ID_STATICTEXT11"));
	FlexGridSizer8->Add(StaticText11, 1, wxALL|wxEXPAND, 5);
	SpinCtrl8 = new wxSpinCtrl(Panel3, ID_SPINCTRL9, _T("5"), wxDefaultPosition, wxSize(70,21), 0, 0, 10, 5, _T("ID_SPINCTRL9"));
	SpinCtrl8->SetValue(_T("5"));
	FlexGridSizer8->Add(SpinCtrl8, 1, wxALL|wxEXPAND, 5);
	StaticText8 = new wxStaticText(Panel3, ID_STATICTEXT8, _("/10"), wxDefaultPosition, wxSize(22,21), wxALIGN_LEFT, _T("ID_STATICTEXT8"));
	FlexGridSizer8->Add(StaticText8, 1, wxALL|wxEXPAND, 5);
	StaticText9 = new wxStaticText(Panel3, ID_STATICTEXT9, _("+"), wxDefaultPosition, wxSize(15,21), wxALIGN_RIGHT, _T("ID_STATICTEXT9"));
	FlexGridSizer8->Add(StaticText9, 1, wxALL|wxEXPAND, 5);
	SpinCtrl6 = new wxSpinCtrl(Panel3, ID_SPINCTRL7, _T("45"), wxDefaultPosition, wxSize(70,21), 0, 0, 180, 45, _T("ID_SPINCTRL7"));
	SpinCtrl6->SetValue(_T("45"));
	FlexGridSizer8->Add(SpinCtrl6, 1, wxALL|wxEXPAND, 5);
	StaticBoxSizer2->Add(FlexGridSizer8, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer5->Add(StaticBoxSizer2, 1, wxALL|wxEXPAND, 5);
	Panel3->SetSizer(FlexGridSizer5);
	FlexGridSizer5->Fit(Panel3);
	FlexGridSizer5->SetSizeHints(Panel3);
	Notebook1->AddPage(Panel1, _("View"), false);
	Notebook1->AddPage(Panel2, _("Manipulate scenes"), false);
	Notebook1->AddPage(Panel3, _("Spherical travelling"), false);
	FlexGridSizer1->Add(Notebook1, 1, wxALL|wxEXPAND, 0);
	FlexGridSizer2 = new wxFlexGridSizer(1, 3, 0, 0);
	FlexGridSizer2->AddGrowableCol(0);
	FlexGridSizer2->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	btnOk = new wxButton(this, ID_BUTTON1, _("OK"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
	btnOk->SetDefault();
	FlexGridSizer2->Add(btnOk, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	btnCancel = new wxButton(this, ID_BUTTON2, _("Cancel"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
	FlexGridSizer2->Add(btnCancel, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer1->Add(FlexGridSizer2, 1, wxALL|wxEXPAND, 0);
	SetSizer(FlexGridSizer1);
	FlexGridSizer1->Fit(this);
	FlexGridSizer1->SetSizeHints(this);
	Center();

	Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CDialogOptions::OnbtnOkClick);
	Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CDialogOptions::OnbtnCancelClick);
	//*)
}

CDialogOptions::~CDialogOptions()
{
	//(*Destroy(CDialogOptions)
	//*)
}


void CDialogOptions::OnbtnOkClick(wxCommandEvent& event)
{
	EndModal( wxID_OK );
}

void CDialogOptions::OnbtnCancelClick(wxCommandEvent& event)
{
	EndModal( wxID_CANCEL );
}
