/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include "CDlgLog.h"
#include "hmt_slam_guiMain.h"


//(*InternalHeaders(CDlgLog)
#include <wx/string.h>
#include <wx/intl.h>
#include <wx/font.h>
//*)

//(*IdInit(CDlgLog)
const long CDlgLog::ID_BUTTON1 = wxNewId();
const long CDlgLog::ID_BUTTON2 = wxNewId();
const long CDlgLog::ID_TEXTCTRL2 = wxNewId();
const long CDlgLog::ID_TIMER1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(CDlgLog,wxDialog)
	//(*EventTable(CDlgLog)
	//*)
END_EVENT_TABLE()


hmt_slam_guiFrame  *myParent=NULL;

CDlgLog::CDlgLog(wxWindow* parent,wxWindowID id,const wxPoint& pos,const wxSize& size)
{
	myParent = static_cast<hmt_slam_guiFrame*>(parent);


	//(*Initialize(CDlgLog)
	wxFlexGridSizer* FlexGridSizer1;
	wxFlexGridSizer* FlexGridSizer2;
	wxFlexGridSizer* FlexGridSizer3;

	Create(parent, id, _("Log messages"), wxDefaultPosition, wxDefaultSize, wxSYSTEM_MENU|wxRESIZE_BORDER|wxCLOSE_BOX|wxMAXIMIZE_BOX|wxMINIMIZE_BOX, _T("id"));
	SetClientSize(wxSize(344,143));
	Move(wxPoint(50,350));
	FlexGridSizer1 = new wxFlexGridSizer(2, 1, 0, 0);
	FlexGridSizer1->AddGrowableCol(0);
	FlexGridSizer1->AddGrowableRow(1);
	FlexGridSizer2 = new wxFlexGridSizer(2, 3, 0, 0);
	btnClear = new wxButton(this, ID_BUTTON1, _("Clear"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
	FlexGridSizer2->Add(btnClear, 1, wxALL|wxALIGN_LEFT|wxALIGN_BOTTOM, 5);
	btnSave = new wxButton(this, ID_BUTTON2, _("Save log.."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
	FlexGridSizer2->Add(btnSave, 1, wxALL|wxALIGN_LEFT|wxALIGN_BOTTOM, 5);
	FlexGridSizer1->Add(FlexGridSizer2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 0);
	FlexGridSizer3 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer3->AddGrowableCol(0);
	FlexGridSizer3->AddGrowableRow(0);
	edLog = new wxTextCtrl(this, ID_TEXTCTRL2, wxEmptyString, wxDefaultPosition, wxSize(408,96), wxTE_PROCESS_ENTER|wxTE_PROCESS_TAB|wxTE_MULTILINE|wxTE_READONLY|wxHSCROLL|wxALWAYS_SHOW_SB, wxDefaultValidator, _T("ID_TEXTCTRL2"));
	wxFont edLogFont(8,wxFONTFAMILY_TELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
	edLog->SetFont(edLogFont);
	FlexGridSizer3->Add(edLog, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 1);
	FlexGridSizer1->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 0);
	SetSizer(FlexGridSizer1);
	timDumpLog.SetOwner(this, ID_TIMER1);
	timDumpLog.Start(250, false);
	FlexGridSizer1->SetSizeHints(this);

	Connect(ID_TIMER1,wxEVT_TIMER,(wxObjectEventFunction)&CDlgLog::OntimDumpLogTrigger);
	Connect(wxID_ANY,wxEVT_CLOSE_WINDOW,(wxObjectEventFunction)&CDlgLog::OnClose);
	//*)


	m_redirector = new CMyRedirector(edLog,  false, 0, true,  /* thread_safe */ true,  /*Also dump to cout*/ true );
}

CDlgLog::~CDlgLog()
{
	mrpt::utils::delete_safe(m_redirector);

	//(*Destroy(CDlgLog)
	//*)
}


void CDlgLog::OnClose(wxCloseEvent& event)
{
	myParent->btnShowLogWin->SetValue(false);
	myParent->btnShowLogWin->Refresh();


	event.Skip(); // Continue processing this event
}

void CDlgLog::OntimDumpLogTrigger(wxTimerEvent& event)
{
	m_redirector->dumpNow();
}
