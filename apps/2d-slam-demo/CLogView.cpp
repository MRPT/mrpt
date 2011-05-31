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


#include "slamdemoApp.h"   // These are just to avoid weird WX compile issues with UNICODE, etc..
#include "slamdemoMain.h"

#include "CLogView.h"

//(*InternalHeaders(CLogView)
#include <wx/string.h>
#include <wx/intl.h>
#include <wx/font.h>
//*)

//(*IdInit(CLogView)
const long CLogView::ID_TEXTCTRL1 = wxNewId();
const long CLogView::ID_BUTTON1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(CLogView,wxDialog)
	//(*EventTable(CLogView)
	//*)
END_EVENT_TABLE()

CLogView::CLogView(wxWindow* parent,wxWindowID id,const wxPoint& pos,const wxSize& size)
{
	//(*Initialize(CLogView)
	wxFlexGridSizer* FlexGridSizer1;
	
	Create(parent, id, _("Log view"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE|wxRESIZE_BORDER|wxMAXIMIZE_BOX, _T("id"));
	SetClientSize(wxDefaultSize);
	Move(wxDefaultPosition);
	FlexGridSizer1 = new wxFlexGridSizer(2, 1, 0, 0);
	FlexGridSizer1->AddGrowableCol(0);
	FlexGridSizer1->AddGrowableRow(0);
	edLog = new wxTextCtrl(this, ID_TEXTCTRL1, wxEmptyString, wxDefaultPosition, wxSize(835,437), wxTE_MULTILINE|wxTE_READONLY|wxHSCROLL|wxTE_DONTWRAP, wxDefaultValidator, _T("ID_TEXTCTRL1"));
	wxFont edLogFont(wxDEFAULT,wxTELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
	edLog->SetFont(edLogFont);
	FlexGridSizer1->Add(edLog, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	btnOk = new wxButton(this, ID_BUTTON1, _("Close"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
	btnOk->SetDefault();
	FlexGridSizer1->Add(btnOk, 1, wxALL|wxALIGN_TOP|wxALIGN_CENTER_HORIZONTAL, 5);
	SetSizer(FlexGridSizer1);
	FlexGridSizer1->Fit(this);
	FlexGridSizer1->SetSizeHints(this);
	Center();
	
	Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CLogView::OnbtnOkClick);
	//*)
}

CLogView::~CLogView()
{
	//(*Destroy(CLogView)
	//*)
}


void CLogView::OnbtnOkClick(wxCommandEvent& event)
{
	Close();
}
