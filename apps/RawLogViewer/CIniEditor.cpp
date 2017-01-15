/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "CIniEditor.h"

#include "xRawLogViewerMain.h"

//(*InternalHeaders(CIniEditor)
#include <wx/settings.h>
#include <wx/font.h>
#include <wx/intl.h>
#include <wx/string.h>
//*)

//(*IdInit(CIniEditor)
const long CIniEditor::ID_BUTTON1 = wxNewId();
const long CIniEditor::ID_BUTTON2 = wxNewId();
const long CIniEditor::ID_TEXTCTRL1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(CIniEditor,wxDialog)
	//(*EventTable(CIniEditor)
	//*)
END_EVENT_TABLE()

CIniEditor::CIniEditor(wxWindow* parent,wxWindowID id,const wxPoint& pos,const wxSize& size)
{
	//(*Initialize(CIniEditor)
	wxFlexGridSizer* FlexGridSizer2;
	wxFlexGridSizer* FlexGridSizer1;

	Create(parent, id, _("Edit "), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE|wxRESIZE_BORDER, _T("id"));
	SetClientSize(wxDefaultSize);
	Move(wxDefaultPosition);
	FlexGridSizer1 = new wxFlexGridSizer(0, 1, 0, 0);
	FlexGridSizer1->AddGrowableCol(0);
	FlexGridSizer1->AddGrowableRow(1);
	FlexGridSizer2 = new wxFlexGridSizer(0, 2, 0, 0);
	btnOK = new wxButton(this, ID_BUTTON1, _("OK"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
	FlexGridSizer2->Add(btnOK, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	btnCancel = new wxButton(this, ID_BUTTON2, _("Cancel"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
	FlexGridSizer2->Add(btnCancel, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer1->Add(FlexGridSizer2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	edText = new wxTextCtrl(this, ID_TEXTCTRL1, wxEmptyString, wxDefaultPosition, wxSize(516,372), wxTE_MULTILINE|wxHSCROLL|wxVSCROLL | wxTE_PROCESS_TAB, wxDefaultValidator, _T("ID_TEXTCTRL1"));
	wxFont edTextFont = wxSystemSettings::GetFont(wxSYS_OEM_FIXED_FONT);
	if ( !edTextFont.Ok() ) edTextFont = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
	edTextFont.SetPointSize((int)(edTextFont.GetPointSize() * 1.000000));
	edText->SetFont(edTextFont);
	FlexGridSizer1->Add(edText, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	SetSizer(FlexGridSizer1);
	FlexGridSizer1->Fit(this);
	FlexGridSizer1->SetSizeHints(this);
	Center();

	Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CIniEditor::OnbtnOKClick);
	Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CIniEditor::OnbtnCancelClick);
	//*)
}

CIniEditor::~CIniEditor()
{
	//(*Destroy(CIniEditor)
	//*)
}


void CIniEditor::OnbtnCancelClick(wxCommandEvent& event)
{
    EndModal( 0 );
}

void CIniEditor::OnbtnOKClick(wxCommandEvent& event)
{
    EndModal( 1 );
}
