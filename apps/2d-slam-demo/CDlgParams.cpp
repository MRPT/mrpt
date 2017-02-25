/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "CDlgParams.h"
#include "slamdemoMain.h"

//(*InternalHeaders(CDlgParams)
#include <wx/string.h>
#include <wx/intl.h>
//*)

//(*IdInit(CDlgParams)
const long CDlgParams::ID_RADIOBUTTON1 = wxNewId();
const long CDlgParams::ID_RADIOBUTTON2 = wxNewId();
const long CDlgParams::ID_RADIOBUTTON3 = wxNewId();
const long CDlgParams::ID_STATICTEXT2 = wxNewId();
const long CDlgParams::ID_SPINCTRL1 = wxNewId();
const long CDlgParams::ID_RADIOBUTTON4 = wxNewId();
const long CDlgParams::ID_STATICTEXT25 = wxNewId();
const long CDlgParams::ID_CHECKBOX2 = wxNewId();
const long CDlgParams::ID_CHECKBOX3 = wxNewId();
const long CDlgParams::ID_STATICTEXT3 = wxNewId();
const long CDlgParams::ID_TEXTCTRL3 = wxNewId();
const long CDlgParams::ID_STATICTEXT4 = wxNewId();
const long CDlgParams::ID_TEXTCTRL4 = wxNewId();
const long CDlgParams::ID_STATICTEXT5 = wxNewId();
const long CDlgParams::ID_TEXTCTRL5 = wxNewId();
const long CDlgParams::ID_STATICTEXT6 = wxNewId();
const long CDlgParams::ID_TEXTCTRL6 = wxNewId();
const long CDlgParams::ID_RADIOBOX3 = wxNewId();
const long CDlgParams::ID_STATICTEXT17 = wxNewId();
const long CDlgParams::ID_TEXTCTRL13 = wxNewId();
const long CDlgParams::ID_STATICTEXT24 = wxNewId();
const long CDlgParams::ID_TEXTCTRL15 = wxNewId();
const long CDlgParams::ID_RADIOBOX1 = wxNewId();
const long CDlgParams::ID_RADIOBOX2 = wxNewId();
const long CDlgParams::ID_PANEL1 = wxNewId();
const long CDlgParams::ID_RADIOBUTTON5 = wxNewId();
const long CDlgParams::ID_STATICTEXT9 = wxNewId();
const long CDlgParams::ID_RADIOBUTTON6 = wxNewId();
const long CDlgParams::ID_STATICTEXT7 = wxNewId();
const long CDlgParams::ID_SPINCTRL2 = wxNewId();
const long CDlgParams::ID_STATICTEXT8 = wxNewId();
const long CDlgParams::ID_SPINCTRL3 = wxNewId();
const long CDlgParams::ID_RADIOBUTTON7 = wxNewId();
const long CDlgParams::ID_TEXTCTRL2 = wxNewId();
const long CDlgParams::ID_BUTTON3 = wxNewId();
const long CDlgParams::ID_CHECKBOX1 = wxNewId();
const long CDlgParams::ID_STATICTEXT11 = wxNewId();
const long CDlgParams::ID_TEXTCTRL7 = wxNewId();
const long CDlgParams::ID_STATICTEXT1 = wxNewId();
const long CDlgParams::ID_TEXTCTRL1 = wxNewId();
const long CDlgParams::ID_STATICTEXT12 = wxNewId();
const long CDlgParams::ID_TEXTCTRL8 = wxNewId();
const long CDlgParams::ID_STATICTEXT23 = wxNewId();
const long CDlgParams::ID_TEXTCTRL14 = wxNewId();
const long CDlgParams::ID_STATICTEXT10 = wxNewId();
const long CDlgParams::ID_TEXTCTRL12 = wxNewId();
const long CDlgParams::ID_STATICTEXT13 = wxNewId();
const long CDlgParams::ID_STATICTEXT14 = wxNewId();
const long CDlgParams::ID_STATICTEXT15 = wxNewId();
const long CDlgParams::ID_STATICTEXT16 = wxNewId();
const long CDlgParams::ID_TEXTCTRL9 = wxNewId();
const long CDlgParams::ID_TEXTCTRL10 = wxNewId();
const long CDlgParams::ID_TEXTCTRL11 = wxNewId();
const long CDlgParams::ID_STATICTEXT26 = wxNewId();
const long CDlgParams::ID_STATICTEXT27 = wxNewId();
const long CDlgParams::ID_STATICTEXT28 = wxNewId();
const long CDlgParams::ID_TEXTCTRL16 = wxNewId();
const long CDlgParams::ID_TEXTCTRL17 = wxNewId();
const long CDlgParams::ID_STATICTEXT18 = wxNewId();
const long CDlgParams::ID_SPINCTRL4 = wxNewId();
const long CDlgParams::ID_STATICTEXT19 = wxNewId();
const long CDlgParams::ID_STATICTEXT20 = wxNewId();
const long CDlgParams::ID_SPINCTRL5 = wxNewId();
const long CDlgParams::ID_STATICTEXT21 = wxNewId();
const long CDlgParams::ID_STATICTEXT22 = wxNewId();
const long CDlgParams::ID_BUTTON1 = wxNewId();
const long CDlgParams::ID_BUTTON2 = wxNewId();
//*)

BEGIN_EVENT_TABLE(CDlgParams,wxDialog)
	//(*EventTable(CDlgParams)
	//*)
END_EVENT_TABLE()

CDlgParams::CDlgParams(wxWindow* parent,wxWindowID id,const wxPoint& pos,const wxSize& size)
{
	//(*Initialize(CDlgParams)
	wxStaticBoxSizer* StaticBoxSizer2;
	wxFlexGridSizer* FlexGridSizer21;
	wxFlexGridSizer* FlexGridSizer8;
	wxFlexGridSizer* FlexGridSizer1;
	wxFlexGridSizer* FlexGridSizer2;
	wxFlexGridSizer* FlexGridSizer15;
	wxStaticBoxSizer* StaticBoxSizer7;
	wxStaticBoxSizer* StaticBoxSizer5;
	wxFlexGridSizer* FlexGridSizer17;
	wxFlexGridSizer* FlexGridSizer19;
	wxFlexGridSizer* FlexGridSizer11;
	wxFlexGridSizer* FlexGridSizer7;
	wxFlexGridSizer* FlexGridSizer4;
	wxFlexGridSizer* FlexGridSizer9;
	wxFlexGridSizer* FlexGridSizer14;
	wxStaticBoxSizer* StaticBoxSizer3;
	wxFlexGridSizer* FlexGridSizer6;
	wxFlexGridSizer* FlexGridSizer3;
	wxStaticBoxSizer* StaticBoxSizer4;
	wxStaticBoxSizer* StaticBoxSizer6;
	wxFlexGridSizer* FlexGridSizer16;
	wxFlexGridSizer* FlexGridSizer10;
	wxFlexGridSizer* FlexGridSizer13;
	wxFlexGridSizer* FlexGridSizer18;
	wxFlexGridSizer* FlexGridSizer12;
	wxFlexGridSizer* FlexGridSizer5;
	wxStaticBoxSizer* StaticBoxSizer1;
	wxFlexGridSizer* FlexGridSizer20;
	
	Create(parent, wxID_ANY, _("Simulation configuration"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE, _T("wxID_ANY"));
	FlexGridSizer1 = new wxFlexGridSizer(1, 2, 0, 0);
	FlexGridSizer16 = new wxFlexGridSizer(3, 1, 0, 0);
	StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Kalman Filter "));
	FlexGridSizer3 = new wxFlexGridSizer(0, 2, 0, 0);
	rbKFnaiv = new wxRadioButton(this, ID_RADIOBUTTON1, _("Standard EKF"), wxDefaultPosition, wxDefaultSize, wxRB_GROUP, wxDefaultValidator, _T("ID_RADIOBUTTON1"));
	FlexGridSizer3->Add(rbKFnaiv, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer3->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	rbKFdavison = new wxRadioButton(this, ID_RADIOBUTTON2, _("EKF (one scalar at a time)"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_RADIOBUTTON2"));
	rbKFdavison->SetValue(true);
	FlexGridSizer3->Add(rbKFdavison, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer3->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	rbIKF = new wxRadioButton(this, ID_RADIOBUTTON3, _("IKF"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_RADIOBUTTON3"));
	FlexGridSizer3->Add(rbIKF, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer4 = new wxFlexGridSizer(0, 3, 0, 0);
	StaticText2 = new wxStaticText(this, ID_STATICTEXT2, _("IKF iters:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
	FlexGridSizer4->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	edIKFiters = new wxSpinCtrl(this, ID_SPINCTRL1, _T("3"), wxDefaultPosition, wxSize(39,23), 0, 1, 30, 3, _T("ID_SPINCTRL1"));
	edIKFiters->SetValue(_T("3"));
	FlexGridSizer4->Add(edIKFiters, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer3->Add(FlexGridSizer4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	rbIKFdavison = new wxRadioButton(this, ID_RADIOBUTTON4, _("IKF (one scalar at a time)"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_RADIOBUTTON4"));
	FlexGridSizer3->Add(rbIKFdavison, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer3->Add(-1,-1,1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticText25 = new wxStaticText(this, ID_STATICTEXT25, _("Numeric Jacobians\n(tick: use numeric approximation)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT25"));
	FlexGridSizer3->Add(StaticText25, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer3->Add(-1,-1,1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
	cbJacobTran = new wxCheckBox(this, ID_CHECKBOX2, _("Transition model"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX2"));
	cbJacobTran->SetValue(false);
	FlexGridSizer3->Add(cbJacobTran, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer3->Add(-1,-1,1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
	cbJacobObs = new wxCheckBox(this, ID_CHECKBOX3, _("Observation model"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX3"));
	cbJacobObs->SetValue(false);
	FlexGridSizer3->Add(cbJacobObs, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer3->Add(-1,-1,1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticBoxSizer1->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer16->Add(StaticBoxSizer1, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticBoxSizer3 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Robot motion "));
	FlexGridSizer10 = new wxFlexGridSizer(0, 2, 0, 0);
	FlexGridSizer10->AddGrowableCol(0);
	StaticText3 = new wxStaticText(this, ID_STATICTEXT3, _("Square path length (m):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
	FlexGridSizer10->Add(StaticText3, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edPathLen = new wxTextCtrl(this, ID_TEXTCTRL3, _("Text"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL3"));
	FlexGridSizer10->Add(edPathLen, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText4 = new wxStaticText(this, ID_STATICTEXT4, _("Each step length (m):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT4"));
	FlexGridSizer10->Add(StaticText4, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edPathStepSize = new wxTextCtrl(this, ID_TEXTCTRL4, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL4"));
	FlexGridSizer10->Add(edPathStepSize, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText5 = new wxStaticText(this, ID_STATICTEXT5, _("Odometry error sigma in X/Y ( m):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT5"));
	FlexGridSizer10->Add(StaticText5, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edOdomStdXY = new wxTextCtrl(this, ID_TEXTCTRL5, _("Text"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL5"));
	FlexGridSizer10->Add(edOdomStdXY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText6 = new wxStaticText(this, ID_STATICTEXT6, _("Odometry error sigma in PHI (deg):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT6"));
	FlexGridSizer10->Add(StaticText6, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edStdOdomPhi = new wxTextCtrl(this, ID_TEXTCTRL6, _("Text"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL6"));
	FlexGridSizer10->Add(edStdOdomPhi, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticBoxSizer3->Add(FlexGridSizer10, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer16->Add(StaticBoxSizer3, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticBoxSizer5 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Data association "));
	panelDA = new wxPanel(this, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
	FlexGridSizer9 = new wxFlexGridSizer(0, 2, 0, 0);
	FlexGridSizer9->AddGrowableCol(0);
	wxString __wxRadioBoxChoices_1[2] = 
	{
	_("Mahalanobis"),
	_("Matching likelihood")
	};
	rbICmetric = new wxRadioBox(panelDA, ID_RADIOBOX3, _("Indiv. compat. gating"), wxDefaultPosition, wxDefaultSize, 2, __wxRadioBoxChoices_1, 1, wxRA_VERTICAL, wxDefaultValidator, _T("ID_RADIOBOX3"));
	FlexGridSizer9->Add(rbICmetric, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer9->Add(-1,-1,1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticText17 = new wxStaticText(panelDA, ID_STATICTEXT17, _("Chi2 threshold (inv.cdf) [0-1]"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT17"));
	FlexGridSizer9->Add(StaticText17, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edChi2 = new wxTextCtrl(panelDA, ID_TEXTCTRL13, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL13"));
	FlexGridSizer9->Add(edChi2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText24 = new wxStaticText(panelDA, ID_STATICTEXT24, _("ML threshold (log-likelihood)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT24"));
	FlexGridSizer9->Add(StaticText24, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edICMLrefDist = new wxTextCtrl(panelDA, ID_TEXTCTRL15, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL15"));
	FlexGridSizer9->Add(edICMLrefDist, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
	wxString __wxRadioBoxChoices_2[2] = 
	{
	_("Nearest neighbor"),
	_("JCBB")
	};
	rbDAMethod = new wxRadioBox(panelDA, ID_RADIOBOX1, _("D.A. method  "), wxDefaultPosition, wxDefaultSize, 2, __wxRadioBoxChoices_2, 1, wxRA_VERTICAL, wxDefaultValidator, _T("ID_RADIOBOX1"));
	FlexGridSizer9->Add(rbDAMethod, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer9->Add(-1,-1,1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
	wxString __wxRadioBoxChoices_3[2] = 
	{
	_("Mahalanobis"),
	_("Matching likelihood")
	};
	rbDAMetric = new wxRadioBox(panelDA, ID_RADIOBOX2, _("D.A. Distance measure "), wxDefaultPosition, wxDefaultSize, 2, __wxRadioBoxChoices_3, 1, wxRA_VERTICAL, wxDefaultValidator, _T("ID_RADIOBOX2"));
	FlexGridSizer9->Add(rbDAMetric, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	panelDA->SetSizer(FlexGridSizer9);
	FlexGridSizer9->Fit(panelDA);
	FlexGridSizer9->SetSizeHints(panelDA);
	StaticBoxSizer5->Add(panelDA, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer16->Add(StaticBoxSizer5, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer1->Add(FlexGridSizer16, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer17 = new wxFlexGridSizer(4, 1, 0, 0);
	FlexGridSizer17->AddGrowableRow(3);
	StaticBoxSizer2 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Ground truth map "));
	FlexGridSizer5 = new wxFlexGridSizer(0, 1, 0, 0);
	FlexGridSizer6 = new wxFlexGridSizer(0, 2, 0, 0);
	rbMapCorridor = new wxRadioButton(this, ID_RADIOBUTTON5, _("Square corridor"), wxDefaultPosition, wxDefaultSize, wxRB_GROUP, wxDefaultValidator, _T("ID_RADIOBUTTON5"));
	FlexGridSizer6->Add(rbMapCorridor, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer11 = new wxFlexGridSizer(0, 3, 0, 0);
	StaticText9 = new wxStaticText(this, ID_STATICTEXT9, _("(Note: seed=-1 means randomize with computer time)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT9"));
	FlexGridSizer11->Add(StaticText9, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer6->Add(FlexGridSizer11, 1, wxALL|wxALIGN_RIGHT|wxALIGN_BOTTOM, 0);
	rbMapRandom = new wxRadioButton(this, ID_RADIOBUTTON6, _("Random"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_RADIOBUTTON6"));
	rbMapRandom->SetValue(true);
	FlexGridSizer6->Add(rbMapRandom, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer7 = new wxFlexGridSizer(0, 5, 0, 0);
	StaticText7 = new wxStaticText(this, ID_STATICTEXT7, _("# landmarks:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT7"));
	FlexGridSizer7->Add(StaticText7, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	edLMs = new wxSpinCtrl(this, ID_SPINCTRL2, _T("60"), wxDefaultPosition, wxSize(52,23), 0, 1, 1000, 60, _T("ID_SPINCTRL2"));
	edLMs->SetValue(_T("60"));
	FlexGridSizer7->Add(edLMs, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText8 = new wxStaticText(this, ID_STATICTEXT8, _("Random seed:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT8"));
	FlexGridSizer7->Add(StaticText8, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	edSeed = new wxSpinCtrl(this, ID_SPINCTRL3, _T("-1"), wxDefaultPosition, wxSize(61,23), 0, -1, 10000000, -1, _T("ID_SPINCTRL3"));
	edSeed->SetValue(_T("-1"));
	FlexGridSizer7->Add(edSeed, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer6->Add(FlexGridSizer7, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	rbMapFile = new wxRadioButton(this, ID_RADIOBUTTON7, _("Text file..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_RADIOBUTTON7"));
	FlexGridSizer6->Add(rbMapFile, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer8 = new wxFlexGridSizer(0, 3, 0, 0);
	FlexGridSizer8->AddGrowableCol(0);
	edMapFile = new wxTextCtrl(this, ID_TEXTCTRL2, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL2"));
	FlexGridSizer8->Add(edMapFile, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	btnBrowse = new wxButton(this, ID_BUTTON3, _("Browse..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON3"));
	FlexGridSizer8->Add(btnBrowse, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer6->Add(FlexGridSizer8, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer5->Add(FlexGridSizer6, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	StaticBoxSizer2->Add(FlexGridSizer5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer17->Add(StaticBoxSizer2, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticBoxSizer4 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Robot sensor "));
	FlexGridSizer12 = new wxFlexGridSizer(0, 1, 0, 0);
	FlexGridSizer12->AddGrowableCol(0);
	FlexGridSizer13 = new wxFlexGridSizer(0, 3, 0, 0);
	cbSensorDistin = new wxCheckBox(this, ID_CHECKBOX1, _("Sensor distingishes landmarks (Checked: avoids data association problem)"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
	cbSensorDistin->SetValue(false);
	FlexGridSizer13->Add(cbSensorDistin, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer12->Add(FlexGridSizer13, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer14 = new wxFlexGridSizer(0, 4, 0, 0);
	StaticText11 = new wxStaticText(this, ID_STATICTEXT11, _("Range noise sigma (m):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT11"));
	FlexGridSizer14->Add(StaticText11, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edStdRange = new wxTextCtrl(this, ID_TEXTCTRL7, _("Text"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL7"));
	FlexGridSizer14->Add(edStdRange, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText1 = new wxStaticText(this, ID_STATICTEXT1, _("Max. Range (m):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
	FlexGridSizer14->Add(StaticText1, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edMaxR = new wxTextCtrl(this, ID_TEXTCTRL1, _("Text"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL1"));
	FlexGridSizer14->Add(edMaxR, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText12 = new wxStaticText(this, ID_STATICTEXT12, _("Angle noise sigma (deg):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT12"));
	FlexGridSizer14->Add(StaticText12, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edStdAngle = new wxTextCtrl(this, ID_TEXTCTRL8, _("Text"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL8"));
	FlexGridSizer14->Add(edStdAngle, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText23 = new wxStaticText(this, ID_STATICTEXT23, _("Min. Range (m):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT23"));
	FlexGridSizer14->Add(StaticText23, 1, wxALL|wxALIGN_RIGHT|wxALIGN_TOP, 5);
	edMinR = new wxTextCtrl(this, ID_TEXTCTRL14, _("Text"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL14"));
	FlexGridSizer14->Add(edMinR, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticText10 = new wxStaticText(this, ID_STATICTEXT10, _("FOV (deg):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT10"));
	FlexGridSizer14->Add(StaticText10, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edFOV = new wxTextCtrl(this, ID_TEXTCTRL12, _("Text"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL12"));
	FlexGridSizer14->Add(edFOV, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer12->Add(FlexGridSizer14, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	StaticText13 = new wxStaticText(this, ID_STATICTEXT13, _("Sensor pose on the robot:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT13"));
	FlexGridSizer12->Add(StaticText13, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer15 = new wxFlexGridSizer(0, 3, 0, 0);
	StaticText14 = new wxStaticText(this, ID_STATICTEXT14, _("x (m):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT14"));
	FlexGridSizer15->Add(StaticText14, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText15 = new wxStaticText(this, ID_STATICTEXT15, _("y (m):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT15"));
	FlexGridSizer15->Add(StaticText15, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText16 = new wxStaticText(this, ID_STATICTEXT16, _("phi (deg):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT16"));
	FlexGridSizer15->Add(StaticText16, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	edSenX = new wxTextCtrl(this, ID_TEXTCTRL9, _("Text"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL9"));
	FlexGridSizer15->Add(edSenX, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	edSenY = new wxTextCtrl(this, ID_TEXTCTRL10, _("Text"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL10"));
	FlexGridSizer15->Add(edSenY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	edSenPhi = new wxTextCtrl(this, ID_TEXTCTRL11, _("Text"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL11"));
	FlexGridSizer15->Add(edSenPhi, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer12->Add(FlexGridSizer15, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	StaticBoxSizer7 = new wxStaticBoxSizer(wxHORIZONTAL, this, _(" Spurious readings: "));
	FlexGridSizer20 = new wxFlexGridSizer(1, 2, 0, 0);
	FlexGridSizer20->AddGrowableCol(1);
	StaticText26 = new wxStaticText(this, ID_STATICTEXT26, _("Normally-distributeed number\nof spurious per \"observation\":\n(Both to 0 = disable)"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT26"));
	FlexGridSizer20->Add(StaticText26, 1, wxALL|wxALIGN_BOTTOM|wxALIGN_CENTER_HORIZONTAL, 5);
	FlexGridSizer21 = new wxFlexGridSizer(2, 2, 0, 0);
	FlexGridSizer21->AddGrowableCol(0);
	FlexGridSizer21->AddGrowableCol(1);
	StaticText27 = new wxStaticText(this, ID_STATICTEXT27, _("Mean (#readings):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT27"));
	FlexGridSizer21->Add(StaticText27, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText28 = new wxStaticText(this, ID_STATICTEXT28, _("Std.Dev. (#readings):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT28"));
	FlexGridSizer21->Add(StaticText28, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	edSpuriousMean = new wxTextCtrl(this, ID_TEXTCTRL16, _("Text"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL16"));
	FlexGridSizer21->Add(edSpuriousMean, 1, wxALL|wxALIGN_BOTTOM|wxALIGN_CENTER_HORIZONTAL, 5);
	edSpuriousStd = new wxTextCtrl(this, ID_TEXTCTRL17, _("Text"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL17"));
	FlexGridSizer21->Add(edSpuriousStd, 1, wxALL|wxALIGN_BOTTOM|wxALIGN_CENTER_HORIZONTAL, 5);
	FlexGridSizer20->Add(FlexGridSizer21, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 0);
	StaticBoxSizer7->Add(FlexGridSizer20, 1, wxALL|wxALIGN_LEFT|wxALIGN_BOTTOM, 0);
	FlexGridSizer12->Add(StaticBoxSizer7, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 2);
	StaticBoxSizer4->Add(FlexGridSizer12, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer17->Add(StaticBoxSizer4, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticBoxSizer6 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Uncertainty models over-estimation"));
	FlexGridSizer18 = new wxFlexGridSizer(1, 2, 0, 0);
	FlexGridSizer19 = new wxFlexGridSizer(2, 3, 0, 0);
	StaticText18 = new wxStaticText(this, ID_STATICTEXT18, _("Odometry errors:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT18"));
	FlexGridSizer19->Add(StaticText18, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edOverOdom = new wxSpinCtrl(this, ID_SPINCTRL4, _T("120"), wxDefaultPosition, wxSize(70,-1), 0, 1, 10000, 120, _T("ID_SPINCTRL4"));
	edOverOdom->SetValue(_T("120"));
	FlexGridSizer19->Add(edOverOdom, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticText19 = new wxStaticText(this, ID_STATICTEXT19, _("%"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT19"));
	FlexGridSizer19->Add(StaticText19, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
	StaticText20 = new wxStaticText(this, ID_STATICTEXT20, _("Sensor errors:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT20"));
	FlexGridSizer19->Add(StaticText20, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edOverSensor = new wxSpinCtrl(this, ID_SPINCTRL5, _T("120"), wxDefaultPosition, wxSize(70,-1), 0, 1, 10000, 120, _T("ID_SPINCTRL5"));
	edOverSensor->SetValue(_T("120"));
	FlexGridSizer19->Add(edOverSensor, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticText21 = new wxStaticText(this, ID_STATICTEXT21, _("%"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT21"));
	FlexGridSizer19->Add(StaticText21, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer18->Add(FlexGridSizer19, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 0);
	StaticText22 = new wxStaticText(this, ID_STATICTEXT22, _("Overestimating the uncertainty in filters is \nusually a good practice, i.e: the filter \nshould be pessimistic."), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT22"));
	FlexGridSizer18->Add(StaticText22, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticBoxSizer6->Add(FlexGridSizer18, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer17->Add(StaticBoxSizer6, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer2 = new wxFlexGridSizer(0, 3, 0, 0);
	btnOk = new wxButton(this, ID_BUTTON1, _("Accept"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
	FlexGridSizer2->Add(btnOk, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	btnCancel = new wxButton(this, ID_BUTTON2, _("Cancel"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
	FlexGridSizer2->Add(btnCancel, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer17->Add(FlexGridSizer2, 1, wxALL|wxALIGN_RIGHT|wxALIGN_BOTTOM, 5);
	FlexGridSizer1->Add(FlexGridSizer17, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	SetSizer(FlexGridSizer1);
	FlexGridSizer1->Fit(this);
	FlexGridSizer1->SetSizeHints(this);
	Center();
	
	Connect(ID_RADIOBUTTON1,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&CDlgParams::OnUpdateControlsState);
	Connect(ID_RADIOBUTTON2,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&CDlgParams::OnUpdateControlsState);
	Connect(ID_RADIOBUTTON3,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&CDlgParams::OnUpdateControlsState);
	Connect(ID_RADIOBUTTON4,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&CDlgParams::OnUpdateControlsState);
	Connect(ID_RADIOBOX3,wxEVT_COMMAND_RADIOBOX_SELECTED,(wxObjectEventFunction)&CDlgParams::OnUpdateControlsState);
	Connect(ID_RADIOBOX1,wxEVT_COMMAND_RADIOBOX_SELECTED,(wxObjectEventFunction)&CDlgParams::OnUpdateControlsState);
	Connect(ID_RADIOBUTTON5,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&CDlgParams::OnUpdateControlsState);
	Connect(ID_RADIOBUTTON6,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&CDlgParams::OnUpdateControlsState);
	Connect(ID_RADIOBUTTON7,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&CDlgParams::OnUpdateControlsState);
	Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CDlgParams::OnbtnBrowseClick);
	Connect(ID_CHECKBOX1,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&CDlgParams::OnUpdateControlsState);
	Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CDlgParams::OnbtnOkClick);
	Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CDlgParams::OnbtnCancelClick);
	//*)
}

CDlgParams::~CDlgParams()
{
	//(*Destroy(CDlgParams)
	//*)
}


void CDlgParams::OnbtnOkClick(wxCommandEvent& event)
{
	this->EndModal(wxID_OK);
}

void CDlgParams::OnbtnCancelClick(wxCommandEvent& event)
{
	this->EndModal(wxID_CANCEL);
}

void CDlgParams::OnbtnBrowseClick(wxCommandEvent& event)
{
}

void CDlgParams::OnrbIKFdavisonSelect(wxCommandEvent& event)
{
}

void CDlgParams::OnUpdateControlsState(wxCommandEvent& event)
{
	edIKFiters->Enable(
		rbIKF->GetValue() ||
		rbIKFdavison->GetValue() );

	edChi2->Enable( rbICmetric->GetSelection()==0 );
	edICMLrefDist->Enable( rbICmetric->GetSelection()==1 );

	edLMs->Enable( rbMapRandom->GetValue() );

	panelDA->Enable( !cbSensorDistin->GetValue() );
}
