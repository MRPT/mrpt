/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "CDlgCamTracking.h"
#include "_DSceneViewerMain.h"

//(*InternalHeaders(CDlgCamTracking)
#include <wx/string.h>
#include <wx/intl.h>
//*)

//(*IdInit(CDlgCamTracking)
const long CDlgCamTracking::ID_BUTTON2 = wxNewId();
const long CDlgCamTracking::ID_BUTTON3 = wxNewId();
const long CDlgCamTracking::ID_BUTTON4 = wxNewId();
const long CDlgCamTracking::ID_CHECKBOX1 = wxNewId();
const long CDlgCamTracking::ID_TEXTCTRL1 = wxNewId();
const long CDlgCamTracking::ID_BUTTON6 = wxNewId();
const long CDlgCamTracking::ID_BUTTON5 = wxNewId();
const long CDlgCamTracking::ID_GRID1 = wxNewId();
const long CDlgCamTracking::ID_BUTTON1 = wxNewId();
const long CDlgCamTracking::ID_MENUITEM1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(CDlgCamTracking,wxDialog)
	//(*EventTable(CDlgCamTracking)
	//*)
END_EVENT_TABLE()

#include <mrpt/system/os.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::math;


CDlgCamTracking::CDlgCamTracking(_DSceneViewerFrame* parent,wxWindowID id,const wxPoint& pos,const wxSize& size)
	: m_main_win(parent)
{
	//(*Initialize(CDlgCamTracking)
	wxFlexGridSizer* FlexGridSizer1;
	wxFlexGridSizer* FlexGridSizer2;
	wxFlexGridSizer* FlexGridSizer4;
	wxFlexGridSizer* FlexGridSizer3;

	Create(parent, id, _("Define camera path"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE|wxRESIZE_BORDER, _T("id"));
	SetClientSize(wxDefaultSize);
	Move(wxDefaultPosition);
	FlexGridSizer1 = new wxFlexGridSizer(0, 1, 0, 0);
	FlexGridSizer1->AddGrowableCol(0);
	FlexGridSizer1->AddGrowableRow(2);
	FlexGridSizer2 = new wxFlexGridSizer(0, 5, 0, 0);
	FlexGridSizer2->AddGrowableCol(2);
	btnLoad = new wxButton(this, ID_BUTTON2, _("Load..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
	FlexGridSizer2->Add(btnLoad, 1, wxALL|wxALIGN_TOP|wxALIGN_BOTTOM, 5);
	btnSave = new wxButton(this, ID_BUTTON3, _("Save..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON3"));
	FlexGridSizer2->Add(btnSave, 1, wxALL|wxALIGN_TOP|wxALIGN_BOTTOM, 5);
	FlexGridSizer2->Add(0,0,1, wxALL|wxALIGN_TOP|wxALIGN_BOTTOM, 5);
	btnGrab = new wxButton(this, ID_BUTTON4, _("Grab current"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON4"));
	FlexGridSizer2->Add(btnGrab, 1, wxALL|wxALIGN_TOP|wxALIGN_BOTTOM, 5);
	FlexGridSizer1->Add(FlexGridSizer2, 1, wxALL|wxEXPAND|wxALIGN_TOP|wxALIGN_BOTTOM, 0);
	FlexGridSizer4 = new wxFlexGridSizer(0, 5, 0, 0);
	FlexGridSizer4->AddGrowableCol(2);
	cbConstVel = new wxCheckBox(this, ID_CHECKBOX1, _("Ignore time, with constant velocity (m/s):"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
	cbConstVel->SetValue(false);
	FlexGridSizer4->Add(cbConstVel, 1, wxALL|wxALIGN_TOP|wxALIGN_BOTTOM, 5);
	edVel = new wxTextCtrl(this, ID_TEXTCTRL1, _("0.2"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL1"));
	FlexGridSizer4->Add(edVel, 1, wxALL|wxALIGN_TOP|wxALIGN_BOTTOM, 5);
	FlexGridSizer4->Add(0,0,1, wxALL|wxALIGN_TOP|wxALIGN_BOTTOM, 5);
	btnStart = new wxButton(this, ID_BUTTON6, _("START"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON6"));
	FlexGridSizer4->Add(btnStart, 1, wxALL|wxALIGN_TOP|wxALIGN_BOTTOM, 5);
	btnStop = new wxButton(this, ID_BUTTON5, _("STOP"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON5"));
	btnStop->Disable();
	FlexGridSizer4->Add(btnStop, 1, wxALL|wxALIGN_TOP|wxALIGN_BOTTOM, 5);
	FlexGridSizer1->Add(FlexGridSizer4, 1, wxALL|wxEXPAND|wxALIGN_TOP|wxALIGN_BOTTOM|wxALIGN_CENTER_HORIZONTAL, 0);
	gridPoses = new wxGrid(this, ID_GRID1, wxDefaultPosition, wxDefaultSize, 0, _T("ID_GRID1"));
	gridPoses->CreateGrid(0,7);
	gridPoses->SetMinSize(wxSize(600,200));
	gridPoses->EnableEditing(true);
	gridPoses->EnableGridLines(true);
	gridPoses->SetColLabelValue(0, _("t (sec)"));
	gridPoses->SetColLabelValue(1, _("x"));
	gridPoses->SetColLabelValue(2, _("y"));
	gridPoses->SetColLabelValue(3, _("z"));
	gridPoses->SetColLabelValue(4, _("yaw (deg)"));
	gridPoses->SetColLabelValue(5, _("pitch (deg)"));
	gridPoses->SetColLabelValue(6, _("roll (deg)"));
	gridPoses->SetDefaultCellFont( gridPoses->GetFont() );
	gridPoses->SetDefaultCellTextColour( gridPoses->GetForegroundColour() );
	FlexGridSizer1->Add(gridPoses, 1, wxALL|wxEXPAND|wxALIGN_TOP|wxALIGN_BOTTOM, 5);
	FlexGridSizer3 = new wxFlexGridSizer(0, 3, 0, 0);
	btnClose = new wxButton(this, ID_BUTTON1, _("Close"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
	FlexGridSizer3->Add(btnClose, 1, wxALL|wxALIGN_TOP|wxALIGN_BOTTOM, 5);
	FlexGridSizer1->Add(FlexGridSizer3, 1, wxALL|wxALIGN_RIGHT|wxALIGN_TOP|wxALIGN_BOTTOM, 0);
	SetSizer(FlexGridSizer1);
	MenuItem1 = new wxMenuItem((&menuGrid), ID_MENUITEM1, _("Delete entry"), wxEmptyString, wxITEM_NORMAL);
	menuGrid.Append(MenuItem1);
	FlexGridSizer1->Fit(this);
	FlexGridSizer1->SetSizeHints(this);

	Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CDlgCamTracking::OnbtnLoadClick);
	Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CDlgCamTracking::OnbtnSaveClick);
	Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CDlgCamTracking::OnbtnGrabClick);
	Connect(ID_BUTTON6,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CDlgCamTracking::OnbtnStartClick);
	Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CDlgCamTracking::OnbtnStopClick);
	Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CDlgCamTracking::OnbtnCloseClick);
	Connect(ID_MENUITEM1,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&CDlgCamTracking::OnMenuItemDelete);
	//*)
}

CDlgCamTracking::~CDlgCamTracking()
{
	//(*Destroy(CDlgCamTracking)
	//*)
}


void CDlgCamTracking::OnbtnCloseClick(wxCommandEvent& event)
{
	Close();
}

void CDlgCamTracking::OnMenuItemDelete(wxCommandEvent& event)
{
}

void CDlgCamTracking::OnbtnSaveClick(wxCommandEvent& event)
{
	this->m_poses.saveToTextFile("a.txt");

}

void CDlgCamTracking::OnbtnLoadClick(wxCommandEvent& event)
{
}

void CDlgCamTracking::OnbtnGrabClick(wxCommandEvent& event)
{
	WX_START_TRY

	CPose3D p;
	m_main_win->m_canvas->m_openGLScene->getViewport("main")->getCurrentCameraPose(p);

	if (m_poses.empty())
	{
		m_poses.insert(now(), p);
	}
	else
	{
		m_poses.insert(now(), p);
	}
	UpdateTableFromPoses();

	WX_END_TRY
}

void CDlgCamTracking::OnbtnStartClick(wxCommandEvent& event)
{
	m_poses.setMaxTimeInterpolation(10000);
	m_poses.setInterpolationMethod( CPose3DInterpolator::imSSLLLL );
	//m_poses.setInterpolationMethod( CPose3DInterpolator::imLinear2Neig );

	m_main_win->m_travelling_is_arbitrary = true;
	m_main_win->m_travelling_start_time = now();
	m_main_win->m_tTravelling.Start(50);


}

void CDlgCamTracking::OnbtnStopClick(wxCommandEvent& event)
{
}


void CDlgCamTracking::UpdateTableFromPoses()
{
	gridPoses->BeginBatch();

	gridPoses->DeleteRows(0,gridPoses->GetNumberRows());

	const size_t N = m_poses.size();
	gridPoses->InsertRows(0, N );

	size_t i=0;
	TTimeStamp t0=INVALID_TIMESTAMP;
	for (CPose3DInterpolator::const_iterator it=m_poses.begin();it!=m_poses.end();++it, ++i)
	{
		const TTimeStamp t = it->first;
		const CPose3D &p = it->second;

		if (t0==INVALID_TIMESTAMP) t0=t;


		gridPoses->SetCellValue(i,0, wxString::Format(wxT("%.02f"), timeDifference(t0,t) ));

		gridPoses->SetCellValue(i,1, wxString::Format(wxT("%f"),p.x() ));
		gridPoses->SetCellValue(i,2, wxString::Format(wxT("%f"),p.y() ));
		gridPoses->SetCellValue(i,3, wxString::Format(wxT("%f"),p.z() ));
		gridPoses->SetCellValue(i,4, wxString::Format(wxT("%f"),RAD2DEG(p.yaw()) ));
		gridPoses->SetCellValue(i,5, wxString::Format(wxT("%f"),RAD2DEG(p.pitch()) ));
		gridPoses->SetCellValue(i,6, wxString::Format(wxT("%f"),RAD2DEG(p.roll()) ));
	}

	gridPoses->EndBatch();
}

