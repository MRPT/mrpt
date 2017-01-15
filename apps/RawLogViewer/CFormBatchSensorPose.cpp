/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "CFormBatchSensorPose.h"

//(*InternalHeaders(CFormBatchSensorPose)
#include <wx/string.h>
#include <wx/intl.h>
#include <wx/font.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/artprov.h>
//*)

#include <wx/filedlg.h>
#include <wx/progdlg.h>
#include <wx/imaglist.h>
#include <wx/busyinfo.h>
#include <wx/app.h>
#include <wx/msgdlg.h>

#include "xRawLogViewerMain.h"
// General global variables:

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;




//(*IdInit(CFormBatchSensorPose)
const long CFormBatchSensorPose::ID_STATICTEXT1 = wxNewId();
const long CFormBatchSensorPose::ID_TEXTCTRL1 = wxNewId();
const long CFormBatchSensorPose::ID_BITMAPBUTTON1 = wxNewId();
const long CFormBatchSensorPose::ID_BUTTON1 = wxNewId();
const long CFormBatchSensorPose::ID_BUTTON2 = wxNewId();
//*)

BEGIN_EVENT_TABLE(CFormBatchSensorPose,wxDialog)
	//(*EventTable(CFormBatchSensorPose)
	//*)
END_EVENT_TABLE()

CFormBatchSensorPose::CFormBatchSensorPose(wxWindow* parent,wxWindowID id)
{
	//(*Initialize(CFormBatchSensorPose)
	wxFlexGridSizer* FlexGridSizer1;
	wxFlexGridSizer* FlexGridSizer2;
	wxFlexGridSizer* FlexGridSizer3;

	Create(parent, id, _("Batch change sensor poses"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE|wxRESIZE_BORDER|wxCLOSE_BOX, _T("id"));
	SetClientSize(wxSize(520,434));
	FlexGridSizer1 = new wxFlexGridSizer(0, 1, 0, 0);
	FlexGridSizer1->AddGrowableCol(0);
	FlexGridSizer1->AddGrowableRow(0);
	FlexGridSizer2 = new wxFlexGridSizer(0, 1, 0, 0);
	FlexGridSizer2->AddGrowableCol(0);
	FlexGridSizer2->AddGrowableRow(1);
	StaticText1 = new wxStaticText(this, ID_STATICTEXT1, _("Configuration of sensors:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
	FlexGridSizer2->Add(StaticText1, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
	edText = new wxTextCtrl(this, ID_TEXTCTRL1, _("//  Any file in the format used by rawlog-grabber can be used here\n//  The name of the sections [xxx] can be arbitrary and \n//   independent of the sensor label. \n// ----------------------------------------------------------------\n\n//  SENSOR 1\n[OneCamera]\nsensorLabel\t\t= CAMERA1\n\npose_x\t\t\t= 0    ; position on the robot (meters)\npose_y\t\t\t= 0\npose_z\t\t\t= 0.5\npose_yaw\t\t= 0\t   ; Angles in degrees\npose_pitch\t\t= 0\npose_roll\t\t= 0\n\n//  SENSOR 2\n[AnotherSensor]\nsensorLabel\t\t= SICK_LASER\n\npose_x\t\t\t= 0.2    ; position on the robot (meters)\npose_y\t\t\t= 0\npose_z\t\t\t= 0.3\npose_yaw\t\t= 0\t   ; Angles in degrees\npose_pitch\t\t= 0\npose_roll\t\t= 0\n\n"), wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE|wxHSCROLL|wxVSCROLL|wxALWAYS_SHOW_SB, wxDefaultValidator, _T("ID_TEXTCTRL1"));
	edText->SetMinSize(wxSize(500,300));
	wxFont edTextFont(9,wxFONTFAMILY_TELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,_T("FreeMono"),wxFONTENCODING_DEFAULT);
	edText->SetFont(edTextFont);
	FlexGridSizer2->Add(edText, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer1->Add(FlexGridSizer2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer3 = new wxFlexGridSizer(0, 5, 0, 0);
	FlexGridSizer3->AddGrowableCol(1);
	btnOpen = new wxBitmapButton(this, ID_BITMAPBUTTON1, wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FILE_OPEN")),wxART_BUTTON), wxDefaultPosition, wxDefaultSize, wxBU_AUTODRAW, wxDefaultValidator, _T("ID_BITMAPBUTTON1"));
	btnOpen->SetToolTip(_("Load from text file..."));
	FlexGridSizer3->Add(btnOpen, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer3->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	btnApply = new wxButton(this, ID_BUTTON1, _("Apply"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
	btnApply->SetDefault();
	FlexGridSizer3->Add(btnApply, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	btnCancel = new wxButton(this, ID_BUTTON2, _("Cancel"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
	FlexGridSizer3->Add(btnCancel, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer1->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	SetSizer(FlexGridSizer1);
	FlexGridSizer1->SetSizeHints(this);
	Center();

	Connect(ID_BITMAPBUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormBatchSensorPose::OnbtnOpenClick);
	Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormBatchSensorPose::OnbtnApplyClick);
	Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormBatchSensorPose::OnbtnCancelClick);
	//*)
}

CFormBatchSensorPose::~CFormBatchSensorPose()
{
	//(*Destroy(CFormBatchSensorPose)
	//*)
}


void CFormBatchSensorPose::OnbtnOpenClick(wxCommandEvent& event)
{
    WX_START_TRY

    wxFileDialog dialog(
        this,
        _("Select file with sensor parameters") /* caption */,
        _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ),
        _("*.ini") /* defaultFilename */ ,
        _("rawlog-grabber config files (*.ini)|*.ini|All files (*.*)|*.*") /* wildcard */,
		wxFD_OPEN |  wxFD_FILE_MUST_EXIST );

    if (dialog.ShowModal() != wxID_OK)
        return;

    // Save the path
    WX_START_TRY
    iniFile->write(iniFileSect,"LastDir",std::string(dialog.GetDirectory().mb_str()));
    WX_END_TRY

	edText->LoadFile( dialog.GetPath() );

    WX_END_TRY
}

void CFormBatchSensorPose::OnbtnApplyClick(wxCommandEvent& event)
{
	EndModal(1);
}

void CFormBatchSensorPose::OnbtnCancelClick(wxCommandEvent& event)
{
	EndModal(0);
}
