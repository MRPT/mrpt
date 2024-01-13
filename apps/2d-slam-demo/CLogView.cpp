/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "CLogView.h"

#include "slamdemoApp.h"  // These are just to avoid weird WX compile issues with UNICODE, etc..
#include "slamdemoMain.h"

//(*InternalHeaders(CLogView)
#include <wx/font.h>
#include <wx/intl.h>
#include <wx/string.h>
//*)

//(*IdInit(CLogView)
const long CLogView::ID_TEXTCTRL1 = wxNewId();
const long CLogView::ID_BUTTON1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(CLogView, wxDialog)
//(*EventTable(CLogView)
//*)
END_EVENT_TABLE()

CLogView::CLogView(
	wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size)
{
	//(*Initialize(CLogView)
	wxFlexGridSizer* FlexGridSizer1;

	Create(
		parent, id, _("Log view"), wxDefaultPosition, wxDefaultSize,
		wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER | wxMAXIMIZE_BOX, _T("id"));
	SetClientSize(wxDefaultSize);
	Move(wxDefaultPosition);
	FlexGridSizer1 = new wxFlexGridSizer(2, 1, 0, 0);
	FlexGridSizer1->AddGrowableCol(0);
	FlexGridSizer1->AddGrowableRow(0);
	edLog = new wxTextCtrl(
		this, ID_TEXTCTRL1, wxEmptyString, wxDefaultPosition, wxSize(835, 437),
		wxTE_MULTILINE | wxTE_READONLY | wxHSCROLL | wxTE_DONTWRAP,
		wxDefaultValidator, _T("ID_TEXTCTRL1"));
	wxFont edLogFont(
		-1, wxFONTFAMILY_TELETYPE, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL);
	edLog->SetFont(edLogFont);
	FlexGridSizer1->Add(edLog, 1, wxEXPAND, 5);
	btnOk = new wxButton(
		this, ID_BUTTON1, _("Close"), wxDefaultPosition, wxDefaultSize, 0,
		wxDefaultValidator, _T("ID_BUTTON1"));
	btnOk->SetDefault();
	FlexGridSizer1->Add(
		btnOk, 1, wxALL | wxALIGN_TOP | wxALIGN_CENTER_HORIZONTAL, 5);
	SetSizer(FlexGridSizer1);
	FlexGridSizer1->Fit(this);
	FlexGridSizer1->SetSizeHints(this);
	Center();

	Bind(wxEVT_BUTTON, &CLogView::OnbtnOkClick, this, ID_BUTTON1);
	//*)
}

CLogView::~CLogView()
{
	//(*Destroy(CLogView)
	//*)
}

void CLogView::OnbtnOkClick(wxCommandEvent& event) { Close(); }
