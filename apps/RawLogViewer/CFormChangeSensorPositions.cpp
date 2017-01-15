/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "CFormChangeSensorPositions.h"

//(*InternalHeaders(CFormChangeSensorPositions)
#include <wx/intl.h>
#include <wx/string.h>
//*)
#include <wx/filedlg.h>
#include <wx/progdlg.h>
#include <wx/imaglist.h>
#include <wx/busyinfo.h>
#include <wx/app.h>
#include <wx/msgdlg.h>

#include "xRawLogViewerMain.h"
// General global variables:

#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;


// The index within the SF to process:
size_t	 idxToProcess=0;
string   labelToProcess;

bool	 sensorPoseReadOK;
TPose3D  sensorPoseToSet,sensorPoseToRead;
bool     changeOnlyXYZ=false;

bool	 camReadIsOk;
CMatrixDouble33     camIntrinsic;
std::vector<double> camDistortion;
float	 camFocalLen;



//(*IdInit(CFormChangeSensorPositions)
const long CFormChangeSensorPositions::ID_RADIOBUTTON1 = wxNewId();
const long CFormChangeSensorPositions::ID_RADIOBUTTON2 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT27 = wxNewId();
const long CFormChangeSensorPositions::ID_TEXTCTRL16 = wxNewId();
const long CFormChangeSensorPositions::ID_BUTTON9 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT28 = wxNewId();
const long CFormChangeSensorPositions::ID_TEXTCTRL17 = wxNewId();
const long CFormChangeSensorPositions::ID_BUTTON11 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICLINE1 = wxNewId();
const long CFormChangeSensorPositions::ID_RADIOBOX1 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT15 = wxNewId();
const long CFormChangeSensorPositions::ID_SPINCTRL1 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT30 = wxNewId();
const long CFormChangeSensorPositions::ID_COMBOBOX1 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT16 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT1 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT2 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT3 = wxNewId();
const long CFormChangeSensorPositions::ID_TEXTCTRL1 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT4 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT5 = wxNewId();
const long CFormChangeSensorPositions::ID_TEXTCTRL2 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT6 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT7 = wxNewId();
const long CFormChangeSensorPositions::ID_TEXTCTRL3 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT8 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT9 = wxNewId();
const long CFormChangeSensorPositions::ID_TEXTCTRL4 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT10 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT11 = wxNewId();
const long CFormChangeSensorPositions::ID_TEXTCTRL5 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT12 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT13 = wxNewId();
const long CFormChangeSensorPositions::ID_TEXTCTRL6 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT14 = wxNewId();
const long CFormChangeSensorPositions::ID_CHECKBOX1 = wxNewId();
const long CFormChangeSensorPositions::ID_BUTTON3 = wxNewId();
const long CFormChangeSensorPositions::ID_BUTTON1 = wxNewId();
const long CFormChangeSensorPositions::ID_PANEL1 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT17 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT18 = wxNewId();
const long CFormChangeSensorPositions::ID_TEXTCTRL7 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT19 = wxNewId();
const long CFormChangeSensorPositions::ID_TEXTCTRL8 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT20 = wxNewId();
const long CFormChangeSensorPositions::ID_TEXTCTRL9 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT21 = wxNewId();
const long CFormChangeSensorPositions::ID_TEXTCTRL10 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT23 = wxNewId();
const long CFormChangeSensorPositions::ID_TEXTCTRL12 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT24 = wxNewId();
const long CFormChangeSensorPositions::ID_TEXTCTRL13 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT25 = wxNewId();
const long CFormChangeSensorPositions::ID_TEXTCTRL14 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT26 = wxNewId();
const long CFormChangeSensorPositions::ID_TEXTCTRL15 = wxNewId();
const long CFormChangeSensorPositions::ID_STATICTEXT22 = wxNewId();
const long CFormChangeSensorPositions::ID_TEXTCTRL11 = wxNewId();
const long CFormChangeSensorPositions::ID_BUTTON4 = wxNewId();
const long CFormChangeSensorPositions::ID_BUTTON5 = wxNewId();
const long CFormChangeSensorPositions::ID_PANEL2 = wxNewId();
const long CFormChangeSensorPositions::ID_NOTEBOOK1 = wxNewId();
const long CFormChangeSensorPositions::ID_BUTTON2 = wxNewId();
//*)

BEGIN_EVENT_TABLE(CFormChangeSensorPositions,wxDialog)
	//(*EventTable(CFormChangeSensorPositions)
	//*)
END_EVENT_TABLE()

CFormChangeSensorPositions::CFormChangeSensorPositions(wxWindow* parent,wxWindowID id)
{
	//(*Initialize(CFormChangeSensorPositions)
	wxStaticBoxSizer* StaticBoxSizer2;
	wxFlexGridSizer* FlexGridSizer4;
	wxFlexGridSizer* FlexGridSizer16;
	wxFlexGridSizer* FlexGridSizer10;
	wxFlexGridSizer* FlexGridSizer3;
	wxFlexGridSizer* FlexGridSizer5;
	wxFlexGridSizer* FlexGridSizer9;
	wxFlexGridSizer* FlexGridSizer2;
	wxFlexGridSizer* FlexGridSizer7;
	wxStaticBoxSizer* StaticBoxSizer3;
	wxFlexGridSizer* FlexGridSizer15;
	wxFlexGridSizer* FlexGridSizer8;
	wxFlexGridSizer* FlexGridSizer13;
	wxFlexGridSizer* FlexGridSizer12;
	wxFlexGridSizer* FlexGridSizer6;
	wxStaticBoxSizer* StaticBoxSizer1;
	wxFlexGridSizer* FlexGridSizer1;
	wxFlexGridSizer* FlexGridSizer11;
	wxFlexGridSizer* FlexGridSizer17;

	Create(parent, id, _("Change sensor pose information:"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE, _T("id"));
	FlexGridSizer1 = new wxFlexGridSizer(3, 1, 0, 0);
	FlexGridSizer1->AddGrowableCol(0);
	StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Apply to:"));
	FlexGridSizer13 = new wxFlexGridSizer(0, 1, 0, 0);
	FlexGridSizer13->AddGrowableCol(0);
	FlexGridSizer13->AddGrowableRow(0);
	FlexGridSizer14 = new wxFlexGridSizer(2, 4, 0, 0);
	FlexGridSizer14->AddGrowableCol(2);
	rbLoaded = new wxRadioButton(this, ID_RADIOBUTTON1, _("Loaded rawlog"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_RADIOBUTTON1"));
	FlexGridSizer14->Add(rbLoaded, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer14->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer14->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer14->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	rbFile = new wxRadioButton(this, ID_RADIOBUTTON2, _("Rawlog in file:"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_RADIOBUTTON2"));
	FlexGridSizer14->Add(rbFile, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
	StaticText27 = new wxStaticText(this, ID_STATICTEXT27, _("Input file:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT27"));
	FlexGridSizer14->Add(StaticText27, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	txtInputFile = new wxTextCtrl(this, ID_TEXTCTRL16, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL16"));
	FlexGridSizer14->Add(txtInputFile, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	btnPickInput = new wxButton(this, ID_BUTTON9, _("Select..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON9"));
	FlexGridSizer14->Add(btnPickInput, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
	FlexGridSizer14->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText28 = new wxStaticText(this, ID_STATICTEXT28, _("Output file:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT28"));
	FlexGridSizer14->Add(StaticText28, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	txtOutputFile = new wxTextCtrl(this, ID_TEXTCTRL17, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL17"));
	FlexGridSizer14->Add(txtOutputFile, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	btnPickOut = new wxButton(this, ID_BUTTON11, _("Select..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON11"));
	FlexGridSizer14->Add(btnPickOut, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
	FlexGridSizer13->Add(FlexGridSizer14, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	StaticLine1 = new wxStaticLine(this, ID_STATICLINE1, wxDefaultPosition, wxSize(10,-1), wxLI_HORIZONTAL, _T("ID_STATICLINE1"));
	FlexGridSizer13->Add(StaticLine1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer16 = new wxFlexGridSizer(0, 2, 0, 0);
	FlexGridSizer16->AddGrowableCol(1);
	FlexGridSizer16->AddGrowableRow(0);
	FlexGridSizer3 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer3->AddGrowableCol(0);
	FlexGridSizer3->AddGrowableRow(0);
	wxString __wxRadioBoxChoices_1[2] =
	{
		_("Select by index within SF"),
		_("Select by label")
	};
	rbApply = new wxRadioBox(this, ID_RADIOBOX1, _("Apply to..."), wxDefaultPosition, wxSize(202,67), 2, __wxRadioBoxChoices_1, 1, wxRA_HORIZONTAL, wxDefaultValidator, _T("ID_RADIOBOX1"));
	FlexGridSizer3->Add(rbApply, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer16->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer17 = new wxFlexGridSizer(2, 2, 0, 0);
	FlexGridSizer17->AddGrowableCol(1);
	StaticText15 = new wxStaticText(this, ID_STATICTEXT15, _("Index within CSensoryFrame"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT15"));
	FlexGridSizer17->Add(StaticText15, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	scIndex = new wxSpinCtrl(this, ID_SPINCTRL1, _T("0"), wxDefaultPosition, wxDefaultSize, 0, 0, 100, 0, _T("ID_SPINCTRL1"));
	scIndex->SetValue(_T("0"));
	FlexGridSizer17->Add(scIndex, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticText30 = new wxStaticText(this, ID_STATICTEXT30, _("Observations by label:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT30"));
	FlexGridSizer17->Add(StaticText30, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edLabel = new wxComboBox(this, ID_COMBOBOX1, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, 0, 0, wxDefaultValidator, _T("ID_COMBOBOX1"));
	edLabel->Disable();
	FlexGridSizer17->Add(edLabel, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer16->Add(FlexGridSizer17, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer13->Add(FlexGridSizer16, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	StaticBoxSizer1->Add(FlexGridSizer13, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer1->Add(StaticBoxSizer1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticText16 = new wxStaticText(this, ID_STATICTEXT16, _("What should be changed in those observations\?"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT16"));
	FlexGridSizer1->Add(StaticText16, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 2);
	Notebook1 = new wxNotebook(this, ID_NOTEBOOK1, wxDefaultPosition, wxDefaultSize, 0, _T("ID_NOTEBOOK1"));
	Panel1 = new wxPanel(Notebook1, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
	FlexGridSizer4 = new wxFlexGridSizer(2, 1, 0, 0);
	FlexGridSizer4->AddGrowableCol(0);
	FlexGridSizer5 = new wxFlexGridSizer(4, 6, 0, 0);
	FlexGridSizer5->AddGrowableCol(1);
	FlexGridSizer5->AddGrowableCol(4);
	FlexGridSizer5->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText1 = new wxStaticText(Panel1, ID_STATICTEXT1, _("3D position:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
	FlexGridSizer5->Add(StaticText1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer5->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer5->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText2 = new wxStaticText(Panel1, ID_STATICTEXT2, _("3D angles (if applicable):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
	FlexGridSizer5->Add(StaticText2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer5->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText3 = new wxStaticText(Panel1, ID_STATICTEXT3, _("x:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
	FlexGridSizer5->Add(StaticText3, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edX = new wxTextCtrl(Panel1, ID_TEXTCTRL1, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL1"));
	FlexGridSizer5->Add(edX, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticText4 = new wxStaticText(Panel1, ID_STATICTEXT4, _("(meters)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT4"));
	FlexGridSizer5->Add(StaticText4, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText5 = new wxStaticText(Panel1, ID_STATICTEXT5, _("yaw:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT5"));
	FlexGridSizer5->Add(StaticText5, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edYaw = new wxTextCtrl(Panel1, ID_TEXTCTRL2, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL2"));
	FlexGridSizer5->Add(edYaw, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticText6 = new wxStaticText(Panel1, ID_STATICTEXT6, _("(deg)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT6"));
	FlexGridSizer5->Add(StaticText6, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText7 = new wxStaticText(Panel1, ID_STATICTEXT7, _("y:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT7"));
	FlexGridSizer5->Add(StaticText7, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edY = new wxTextCtrl(Panel1, ID_TEXTCTRL3, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL3"));
	FlexGridSizer5->Add(edY, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticText8 = new wxStaticText(Panel1, ID_STATICTEXT8, _("(meters)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT8"));
	FlexGridSizer5->Add(StaticText8, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText9 = new wxStaticText(Panel1, ID_STATICTEXT9, _("pitch:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT9"));
	FlexGridSizer5->Add(StaticText9, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edPitch = new wxTextCtrl(Panel1, ID_TEXTCTRL4, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL4"));
	FlexGridSizer5->Add(edPitch, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticText10 = new wxStaticText(Panel1, ID_STATICTEXT10, _("(deg)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT10"));
	FlexGridSizer5->Add(StaticText10, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText11 = new wxStaticText(Panel1, ID_STATICTEXT11, _("z:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT11"));
	FlexGridSizer5->Add(StaticText11, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edZ = new wxTextCtrl(Panel1, ID_TEXTCTRL5, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL5"));
	FlexGridSizer5->Add(edZ, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticText12 = new wxStaticText(Panel1, ID_STATICTEXT12, _("(meters)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT12"));
	FlexGridSizer5->Add(StaticText12, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText13 = new wxStaticText(Panel1, ID_STATICTEXT13, _("roll:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT13"));
	FlexGridSizer5->Add(StaticText13, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edRoll = new wxTextCtrl(Panel1, ID_TEXTCTRL6, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL6"));
	FlexGridSizer5->Add(edRoll, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticText14 = new wxStaticText(Panel1, ID_STATICTEXT14, _("(deg)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT14"));
	FlexGridSizer5->Add(StaticText14, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer4->Add(FlexGridSizer5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer15 = new wxFlexGridSizer(0, 3, 0, 0);
	cbOnlyXYZ = new wxCheckBox(Panel1, ID_CHECKBOX1, _("Change X,Y,Z only"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
	cbOnlyXYZ->SetValue(false);
	FlexGridSizer15->Add(cbOnlyXYZ, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer4->Add(FlexGridSizer15, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer6 = new wxFlexGridSizer(0, 3, 0, 0);
	FlexGridSizer6->AddGrowableCol(2);
	FlexGridSizer6->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	btnGetCurPose = new wxButton(Panel1, ID_BUTTON3, _("Get current values..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON3"));
	FlexGridSizer6->Add(btnGetCurPose, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	btnOK = new wxButton(Panel1, ID_BUTTON1, _("Apply changes..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
	FlexGridSizer6->Add(btnOK, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer4->Add(FlexGridSizer6, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	Panel1->SetSizer(FlexGridSizer4);
	FlexGridSizer4->Fit(Panel1);
	FlexGridSizer4->SetSizeHints(Panel1);
	Panel2 = new wxPanel(Notebook1, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
	FlexGridSizer7 = new wxFlexGridSizer(0, 1, 0, 0);
	FlexGridSizer7->AddGrowableCol(0);
	FlexGridSizer7->AddGrowableRow(1);
	StaticText17 = new wxStaticText(Panel2, ID_STATICTEXT17, _("This applies to CObservationImages (\"Monocular cameras\" only)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT17"));
	FlexGridSizer7->Add(StaticText17, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer9 = new wxFlexGridSizer(0, 2, 0, 0);
	FlexGridSizer9->AddGrowableCol(0);
	FlexGridSizer9->AddGrowableCol(1);
	FlexGridSizer9->AddGrowableRow(0);
	StaticBoxSizer2 = new wxStaticBoxSizer(wxHORIZONTAL, Panel2, _("Intrinsic parameters (in pixels)"));
	FlexGridSizer10 = new wxFlexGridSizer(0, 4, 0, 0);
	FlexGridSizer10->AddGrowableCol(1);
	FlexGridSizer10->AddGrowableCol(3);
	StaticText18 = new wxStaticText(Panel2, ID_STATICTEXT18, _("f_x="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT18"));
	FlexGridSizer10->Add(StaticText18, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edFX = new wxTextCtrl(Panel2, ID_TEXTCTRL7, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL7"));
	FlexGridSizer10->Add(edFX, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticText19 = new wxStaticText(Panel2, ID_STATICTEXT19, _("f_y="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT19"));
	FlexGridSizer10->Add(StaticText19, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edFY = new wxTextCtrl(Panel2, ID_TEXTCTRL8, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL8"));
	FlexGridSizer10->Add(edFY, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticText20 = new wxStaticText(Panel2, ID_STATICTEXT20, _("c_x="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT20"));
	FlexGridSizer10->Add(StaticText20, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edCX = new wxTextCtrl(Panel2, ID_TEXTCTRL9, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL9"));
	FlexGridSizer10->Add(edCX, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticText21 = new wxStaticText(Panel2, ID_STATICTEXT21, _("c_y="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT21"));
	FlexGridSizer10->Add(StaticText21, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edCY = new wxTextCtrl(Panel2, ID_TEXTCTRL10, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL10"));
	FlexGridSizer10->Add(edCY, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticBoxSizer2->Add(FlexGridSizer10, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer9->Add(StaticBoxSizer2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	StaticBoxSizer3 = new wxStaticBoxSizer(wxHORIZONTAL, Panel2, _("Distortion parameters"));
	FlexGridSizer11 = new wxFlexGridSizer(0, 4, 0, 0);
	StaticText23 = new wxStaticText(Panel2, ID_STATICTEXT23, _("k1="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT23"));
	FlexGridSizer11->Add(StaticText23, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	edK1 = new wxTextCtrl(Panel2, ID_TEXTCTRL12, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL12"));
	FlexGridSizer11->Add(edK1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText24 = new wxStaticText(Panel2, ID_STATICTEXT24, _("k2="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT24"));
	FlexGridSizer11->Add(StaticText24, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	edK2 = new wxTextCtrl(Panel2, ID_TEXTCTRL13, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL13"));
	FlexGridSizer11->Add(edK2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText25 = new wxStaticText(Panel2, ID_STATICTEXT25, _("p1="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT25"));
	FlexGridSizer11->Add(StaticText25, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	edP1 = new wxTextCtrl(Panel2, ID_TEXTCTRL14, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL14"));
	FlexGridSizer11->Add(edP1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText26 = new wxStaticText(Panel2, ID_STATICTEXT26, _("p2="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT26"));
	FlexGridSizer11->Add(StaticText26, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	edP2 = new wxTextCtrl(Panel2, ID_TEXTCTRL15, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL15"));
	FlexGridSizer11->Add(edP2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticBoxSizer3->Add(FlexGridSizer11, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer9->Add(StaticBoxSizer3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer7->Add(FlexGridSizer9, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer12 = new wxFlexGridSizer(0, 2, 0, 0);
	StaticText22 = new wxStaticText(Panel2, ID_STATICTEXT22, _("Focal length (in meters) ="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT22"));
	FlexGridSizer12->Add(StaticText22, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	edFocalLen = new wxTextCtrl(Panel2, ID_TEXTCTRL11, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL11"));
	FlexGridSizer12->Add(edFocalLen, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer7->Add(FlexGridSizer12, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer8 = new wxFlexGridSizer(0, 3, 0, 0);
	FlexGridSizer8->AddGrowableCol(2);
	FlexGridSizer8->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	btnGetCurCamModel = new wxButton(Panel2, ID_BUTTON4, _("Get current values..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON4"));
	FlexGridSizer8->Add(btnGetCurCamModel, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	btnApplyCameraParams = new wxButton(Panel2, ID_BUTTON5, _("Apply changes..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON5"));
	FlexGridSizer8->Add(btnApplyCameraParams, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer7->Add(FlexGridSizer8, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	Panel2->SetSizer(FlexGridSizer7);
	FlexGridSizer7->Fit(Panel2);
	FlexGridSizer7->SetSizeHints(Panel2);
	Notebook1->AddPage(Panel1, _("Sensor pose on robot"), false);
	Notebook1->AddPage(Panel2, _("Camera parameters"), false);
	FlexGridSizer1->Add(Notebook1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer2 = new wxFlexGridSizer(0, 3, 0, 0);
	FlexGridSizer2->AddGrowableCol(0);
	FlexGridSizer2->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	btnCancel = new wxButton(this, ID_BUTTON2, _("Cancel"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
	FlexGridSizer2->Add(btnCancel, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer1->Add(FlexGridSizer2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	SetSizer(FlexGridSizer1);
	FlexGridSizer1->Fit(this);
	FlexGridSizer1->SetSizeHints(this);
	Center();

	Connect(ID_RADIOBUTTON1,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&CFormChangeSensorPositions::OnrbLoadedSelect);
	Connect(ID_RADIOBUTTON2,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&CFormChangeSensorPositions::OnrbFileSelect);
	Connect(ID_BUTTON9,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormChangeSensorPositions::OnbtnPickInputClick);
	Connect(ID_BUTTON11,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormChangeSensorPositions::OnbtnPickOutClick);
	Connect(ID_RADIOBOX1,wxEVT_COMMAND_RADIOBOX_SELECTED,(wxObjectEventFunction)&CFormChangeSensorPositions::OnrbApplySelect);
	Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormChangeSensorPositions::OnbtnGetCurPoseClick1);
	Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormChangeSensorPositions::OnbtnOKClick);
	Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormChangeSensorPositions::OnbtnGetCurCamModelClick);
	Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormChangeSensorPositions::OnbtnApplyCameraParamsClick);
	Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormChangeSensorPositions::OnbtnCancelClick);
	Connect(wxID_ANY,wxEVT_INIT_DIALOG,(wxObjectEventFunction)&CFormChangeSensorPositions::OnInit);
	//*)
}

CFormChangeSensorPositions::~CFormChangeSensorPositions()
{
	//(*Destroy(CFormChangeSensorPositions)
	//*)
}


void CFormChangeSensorPositions::OnbtnCancelClick(wxCommandEvent& event)
{
	Close();
}

void CFormChangeSensorPositions::OnrbLoadedSelect(wxCommandEvent& event)
{
    btnPickInput->Disable();
    btnPickOut->Disable();
    txtOutputFile->Disable();
    txtInputFile->Disable();
}

void CFormChangeSensorPositions::OnrbFileSelect(wxCommandEvent& event)
{
    btnPickInput->Enable();
    btnPickOut->Enable();
    txtOutputFile->Enable();
    txtInputFile->Enable();
}

void CFormChangeSensorPositions::OnbtnPickInputClick(wxCommandEvent& event)
{
    WX_START_TRY

    wxFileDialog dialog(
        this,
        _("Select input rawlog file") /* caption */,
        _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ),
        _("*.rawlog") /* defaultFilename */ ,
        _("Rawlog files (*.rawlog,*.rawlog.gz)|*.rawlog;*.rawlog.gz|All files (*.*)|*.*") /* wildcard */,
        wxFD_OPEN |  wxFD_FILE_MUST_EXIST );

    if (dialog.ShowModal() != wxID_OK)
        return;

    // Save the path
    WX_START_TRY
    iniFile->write(iniFileSect,"LastDir",std::string(dialog.GetDirectory().mb_str()));
    WX_END_TRY

    txtInputFile->SetValue( dialog.GetPath() );

    WX_END_TRY
}

void CFormChangeSensorPositions::OnbtnPickOutClick(wxCommandEvent& event)
{
    WX_START_TRY

    wxFileDialog dialog(
        this,
        _("Select output rawlog file") /* caption */,
        _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ),
        _("*.rawlog") /* defaultFilename */ ,
        _("Rawlog files (*.rawlog,*.rawlog.gz)|*.rawlog;*.rawlog.gz|All files (*.*)|*.*") /* wildcard */,
        wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

    if (dialog.ShowModal() != wxID_OK)
        return;

    // Save the path
    WX_START_TRY
    iniFile->write(iniFileSect,"LastDir",std::string(dialog.GetDirectory().mb_str()));
    WX_END_TRY

    txtOutputFile->SetValue( dialog.GetPath() );

    WX_END_TRY
}

void CFormChangeSensorPositions::OnInit(wxInitDialogEvent& event)
{
    Center();
    wxCommandEvent  dumm;

    // Is there any rawlog loaded??
    if (!rawlog.size())
    {
        // NO: Disable "modify current":
        OnrbFileSelect( dumm );
        rbFile->SetValue(true);
        rbLoaded->Disable();
    }
    else
    {
        // Select the loaded rawlog by default:
        OnrbLoadedSelect( dumm );
        rbLoaded->SetValue(true);
    } // end there is loaded rawlog

}


/** This is the common function for all operations over a rawlog file ("filter" a rawlog file into a new one) or over the loaded rawlog (depending on the user selection in the GUI).
  */
void CFormChangeSensorPositions::executeOperationOnRawlog( TRawlogFilter operation, const char *endMsg )
{
    WX_START_TRY

    int   			processMax;
    bool			isInMemory;
    CStream 		*in_fil=NULL,*out_fil=NULL;


	sensorPoseReadOK = false;
	camReadIsOk = false;

    if (rbLoaded->GetValue())
    {
        // APPLY TO rawlog in memory:
        isInMemory = true;

        processMax = (int)rawlog.size();
    }
    else
    {
        // APPLY TO rawlog files:
        isInMemory = false;

        if ( !txtInputFile->GetValue().size() )
            THROW_EXCEPTION("An input rawlog file must be selected")
            if ( !txtOutputFile->GetValue().size() )
                THROW_EXCEPTION("An output rawlog file must be selected")

                string   fileName_IN( txtInputFile->GetValue().mbc_str() );
        if (!mrpt::system::fileExists(fileName_IN) )
            THROW_EXCEPTION("Input file does not exist!")

            string   fileName_OUT( txtOutputFile->GetValue().mbc_str() );

        if (!fileName_OUT.compare(fileName_IN))
            THROW_EXCEPTION("Input and output files must be different!")

		in_fil = new CFileGZInputStream(fileName_IN);
		out_fil = new CFileGZOutputStream(fileName_OUT);

        processMax = (int)in_fil->getTotalBytesCount();
    }

    wxProgressDialog    progDia(
        wxT("Modifying rawlog"),
        wxT("Processing..."),
        processMax, // range
        this, // parent
        wxPD_CAN_ABORT |
        wxPD_APP_MODAL |
        wxPD_SMOOTH |
        wxPD_AUTO_HIDE |
        wxPD_ELAPSED_TIME |
        wxPD_ESTIMATED_TIME |
        wxPD_REMAINING_TIME);

    wxTheApp->Yield();  // Let the app. process messages

    unsigned int		countLoop = 0;
    bool                keepLoading=true;
    string              errorMsg;
    wxString			auxStr;

    // Apply changes:
    int 	changes = 0;
    wxBusyCursor    cursor;

    while ((( !isInMemory && keepLoading ) ||
            (  isInMemory && countLoop < rawlog.size() ))&& !sensorPoseReadOK && !camReadIsOk )
    {
        CSerializablePtr newObj;
        try
        {
            if (isInMemory)
            {
                newObj = rawlog.getAsGeneric(countLoop);
            }
            else
            {
                (*in_fil) >> newObj;
            }

            // Check type:
            if ( newObj->GetRuntimeClass() == CLASS_ID(CSensoryFrame))
            {
                // A sensory frame:
                CSensoryFramePtr sf(newObj);

                // Process & save:
				operation(NULL,sf.pointer(),changes );

				if (!isInMemory)  (*out_fil) << *sf.pointer();
            }
            else if ( newObj->GetRuntimeClass() == CLASS_ID(CActionCollection))
            {
                // This is an action:
                CActionCollectionPtr acts =CActionCollectionPtr( newObj );

                // Process & save:
				operation( (CActionCollection*)acts.pointer(),NULL,changes);

                if (!isInMemory)  (*out_fil) << *acts;
            }
			else if ( newObj->GetRuntimeClass()->derivedFrom(CLASS_ID(CObservation)))
            {
                // A sensory frame:
                CObservationPtr o(newObj);

				static CSensoryFrame sf;
				sf.clear();
				sf.insert(o);

                // Process & save:
				operation(NULL,&sf,changes );

				if (!isInMemory)  (*out_fil) << *o;
            }
            else
            {   // Unknown class:
                THROW_EXCEPTION(format("Unexpected class found in the file: '%s'",newObj->GetRuntimeClass()->className) );
            }
        }
        catch (exception &e)
        {
            errorMsg = e.what();
            keepLoading = false;
        }
        catch (...)
        {
            keepLoading = false;
        }

        // Step counter & update progress dialog
        if (countLoop++ % 300 == 0)
        {
            auxStr.sprintf(wxT("Processing... (%u objects processed)"),countLoop);
            int curProgr =  isInMemory ? countLoop : (int)in_fil->getPosition();
            if (!progDia.Update( curProgr , auxStr ))
                keepLoading = false;
            wxTheApp->Yield();  // Let the app. process messages
        }

        // Delete only if processing file
        if (newObj && !isInMemory)
        {
            newObj.clear();
        }

    } // end while keep loading

    progDia.Update( processMax );	// Close dialog.

	if (strlen(endMsg))
	{
		char tmpStr[1000];
		os::sprintf(tmpStr,sizeof(tmpStr),"%s %i\n\nEnd message:\n%s", endMsg, changes, errorMsg.c_str() );
		wxMessageBox( _U(tmpStr), _("Result:"), wxOK,this);
	}

    if (in_fil) delete in_fil;
    if (out_fil) delete out_fil;

    WX_END_TRY
}


// ------------------------------------------------------------
//    Set the sensor pose
// ------------------------------------------------------------
void  exec_setPoseByIdx( mrpt::obs::CActionCollection *acts, mrpt::obs::CSensoryFrame *SF, int &changesCount  )
{
	if (SF)
		if (SF->size()>idxToProcess)
		{
		    CObservationPtr obs = SF->getObservationByIndex(idxToProcess);
		    if (changeOnlyXYZ)
		    {
                CPose3D   tmpPose;
                obs->getSensorPose(tmpPose);
                tmpPose.setFromValues(
                    sensorPoseToSet.x,
                    sensorPoseToSet.y,
                    sensorPoseToSet.z,
                    tmpPose.yaw(),
                    tmpPose.pitch(),
                    tmpPose.roll()
                    );
                obs->setSensorPose( tmpPose );
		    }
		    else
		    {
		        obs->setSensorPose( sensorPoseToSet );
		    }
			changesCount++;
		}
}

void  exec_setPoseByLabel( mrpt::obs::CActionCollection *acts, mrpt::obs::CSensoryFrame *SF, int &changesCount  )
{
	if (SF)
	{
		for (CSensoryFrame::iterator it= SF->begin();it!=SF->end();++it)
		{
		    CObservationPtr obs = *it;

		    if ( obs->sensorLabel == labelToProcess)
		    {
				if (changeOnlyXYZ)
				{
					CPose3D   tmpPose;
					obs->getSensorPose(tmpPose);
					tmpPose.setFromValues(
						sensorPoseToSet.x,
						sensorPoseToSet.y,
						sensorPoseToSet.z,
						tmpPose.yaw(),
						tmpPose.pitch(),
						tmpPose.roll()
						);
					obs->setSensorPose( tmpPose );
				}
				else
				{
					obs->setSensorPose( sensorPoseToSet );
				}
				changesCount++;
		    }
		} // end for each obs.
	}
}


void CFormChangeSensorPositions::OnbtnOKClick(wxCommandEvent& event)
{
	WX_START_TRY

	// Set: sensorPoseToSet
	sensorPoseToSet.x = atof( string(edX->GetValue().mb_str()).c_str());
	sensorPoseToSet.y = atof( string(edY->GetValue().mb_str()).c_str());
	sensorPoseToSet.z = atof( string(edZ->GetValue().mb_str()).c_str());
	sensorPoseToSet.yaw = DEG2RAD(atof( string(edYaw->GetValue().mb_str()).c_str() ));
	sensorPoseToSet.pitch = DEG2RAD(atof( string(edPitch->GetValue().mb_str()).c_str() ));
	sensorPoseToSet.roll = DEG2RAD(atof( string(edRoll->GetValue().mb_str()).c_str() ));

    changeOnlyXYZ = cbOnlyXYZ->GetValue();

    if ( rbApply->GetSelection() == 0 )
    {
		idxToProcess = scIndex->GetValue();
		executeOperationOnRawlog( exec_setPoseByIdx, "Sensor poses changed (by index): " );
    }
	else
	{
		labelToProcess = string(edLabel->GetValue().mb_str());
		executeOperationOnRawlog( exec_setPoseByLabel, "Sensor poses changed (by label): " );
	}

	WX_END_TRY
}

// ------------------------------------------------------------
//    Get the current sensor pose
// ------------------------------------------------------------
void  exec_getCurrentPoseByIdx( mrpt::obs::CActionCollection *acts, mrpt::obs::CSensoryFrame *SF, int &changesCount  )
{
	if (SF)
		if (SF->size()>idxToProcess)
		{
			SF->getObservationByIndex(idxToProcess)->getSensorPose( sensorPoseToRead );
			sensorPoseReadOK = true;
		}
}
void  exec_getCurrentPoseByLabel( mrpt::obs::CActionCollection *acts, mrpt::obs::CSensoryFrame *SF, int &changesCount  )
{
	if (SF && SF->size())
	{
		CObservationPtr o = SF->getObservationBySensorLabel(labelToProcess);
		if (o)
		{
			o->getSensorPose( sensorPoseToRead );
			sensorPoseReadOK = true;
		}
	}
}

void CFormChangeSensorPositions::OnbtnGetCurPoseClick1(wxCommandEvent& event)
{
	WX_START_TRY

	if ( rbApply->GetSelection() == 0 )
    {
		idxToProcess = scIndex->GetValue();
		executeOperationOnRawlog( exec_getCurrentPoseByIdx, "" );
    }
	else
	{
		labelToProcess = string(edLabel->GetValue().mb_str());
		executeOperationOnRawlog( exec_getCurrentPoseByLabel, "" );
	}

	if (sensorPoseReadOK )
	{
		// Put the sensor pose (stored in "sensorPoseToRead"):
		edX->SetValue( _U(format("%f",sensorPoseToRead.x).c_str()) );
		edY->SetValue( _U(format("%f",sensorPoseToRead.y).c_str()) );
		edZ->SetValue( _U(format("%f",sensorPoseToRead.z).c_str()) );
		edYaw->SetValue( _U(format("%f",RAD2DEG(sensorPoseToRead.yaw)).c_str()) );
		edPitch->SetValue( _U(format("%f",RAD2DEG(sensorPoseToRead.pitch)).c_str()) );
		edRoll->SetValue( _U(format("%f",RAD2DEG(sensorPoseToRead.roll)).c_str()) );
	}
	else
	{
		wxMessageBox(_("Error: No observation with the selected index has been found in the rawlog!"));
	}

	WX_END_TRY
}

// ------------------------------------------------------------
//    Get the current camera model
// ------------------------------------------------------------
void  exec_getCurrentCamCfgByIdx( mrpt::obs::CActionCollection *acts, mrpt::obs::CSensoryFrame *SF, int &changesCount  )
{
	if (SF)
		if (SF->size()>idxToProcess)
		{
			if ( SF->getObservationByIndex(idxToProcess)->GetRuntimeClass() == CLASS_ID( CObservationImage ))
			{
				CObservationImagePtr obsIm = SF->getObservationByIndexAs<CObservationImagePtr>(idxToProcess);
				camDistortion= obsIm->cameraParams.getDistortionParamsAsVector();
				camIntrinsic = obsIm->cameraParams.intrinsicParams;
				camFocalLen  = obsIm->cameraParams.focalLengthMeters;
				camReadIsOk = true;
			}
		}
}
void  exec_getCurrentCamCfgByLabel( mrpt::obs::CActionCollection *acts, mrpt::obs::CSensoryFrame *SF, int &changesCount  )
{
	if (SF && SF->size())
	{
		CObservationPtr o = SF->getObservationBySensorLabel(labelToProcess);

		if (o && o->GetRuntimeClass()==CLASS_ID(CObservationImage))
		{
			CObservationImagePtr obsIm = SF->getObservationBySensorLabelAs<CObservationImagePtr>(labelToProcess);
			camDistortion= obsIm->cameraParams.getDistortionParamsAsVector();
			camIntrinsic = obsIm->cameraParams.intrinsicParams;
			camFocalLen  = obsIm->cameraParams.focalLengthMeters;
			camReadIsOk = true;
		}
	}
}

void CFormChangeSensorPositions::OnbtnGetCurCamModelClick(wxCommandEvent& event)
{
	WX_START_TRY

    if ( rbApply->GetSelection() == 0 )
    {
		idxToProcess = scIndex->GetValue();
		executeOperationOnRawlog( exec_getCurrentCamCfgByIdx, "" );
    }
	else
	{
		labelToProcess = string(edLabel->GetValue().mb_str());
		executeOperationOnRawlog( exec_getCurrentCamCfgByLabel, "" );
	}

	if (camReadIsOk )
	{
		// Put the camara data:
		edFX->SetValue( _U(format("%f",camIntrinsic(0,0)).c_str()) );
		edFY->SetValue( _U(format("%f",camIntrinsic(1,1)).c_str()) );
		edCX->SetValue( _U(format("%f",camIntrinsic(0,2)).c_str()) );
		edCY->SetValue( _U(format("%f",camIntrinsic(1,2)).c_str()) );

		edK1->SetValue( _U(format("%f",camDistortion[0]).c_str()) );
		edK2->SetValue( _U(format("%f",camDistortion[1]).c_str()) );
		edP1->SetValue( _U(format("%f",camDistortion[1]).c_str()) );
		edP2->SetValue( _U(format("%f",camDistortion[2]).c_str()) );

		edFocalLen->SetValue( _U(format("%f",camFocalLen).c_str()) );
	}
	else
	{
		wxMessageBox(_("Error: No CObservationImage found in the rawlog with the selected index!"));
	}

	WX_END_TRY
}

// ------------------------------------------------------------
//    Set the camera model
// ------------------------------------------------------------
void  exec_setCurrentCamCfgByIdx( mrpt::obs::CActionCollection *acts, mrpt::obs::CSensoryFrame *SF, int &changesCount  )
{
	if (SF)
		if (SF->size()>idxToProcess)
		{
			if ( SF->getObservationByIndex(idxToProcess)->GetRuntimeClass() == CLASS_ID( CObservationImage ))
			{
				CObservationImagePtr obsIm = SF->getObservationByIndexAs<CObservationImagePtr>(idxToProcess);
				obsIm->cameraParams.setDistortionParamsVector( camDistortion );
				obsIm->cameraParams.intrinsicParams = camIntrinsic;
				obsIm->cameraParams.focalLengthMeters = camFocalLen;
				changesCount++;
			}
		}
}
void  exec_setCurrentCamCfgByLabel( mrpt::obs::CActionCollection *acts, mrpt::obs::CSensoryFrame *SF, int &changesCount  )
{
	if (SF && SF->size())
	{
		CObservationPtr o = SF->getObservationBySensorLabel(labelToProcess);
		if (o && o->GetRuntimeClass()==CLASS_ID(CObservationImage))
		{
			CObservationImagePtr obsIm = SF->getObservationBySensorLabelAs<CObservationImagePtr>(labelToProcess);
			obsIm->cameraParams.setDistortionParamsVector( camDistortion );
			obsIm->cameraParams.intrinsicParams = camIntrinsic;
			obsIm->cameraParams.focalLengthMeters = camFocalLen;
			changesCount++;
		}
	}
}

void CFormChangeSensorPositions::OnbtnApplyCameraParamsClick(wxCommandEvent& event)
{
	WX_START_TRY

	idxToProcess = scIndex->GetValue();

	// Put the camara data:
	camIntrinsic.setSize(3,3);
	camIntrinsic.zeros();
	camIntrinsic(2,2)=1;
	camIntrinsic(0,0) = atof( string(edFX->GetValue().mb_str()).c_str() );
	camIntrinsic(1,1) = atof( string(edFY->GetValue().mb_str()).c_str() );
	camIntrinsic(0,2) = atof( string(edCX->GetValue().mb_str()).c_str() );
	camIntrinsic(1,2) = atof( string(edCY->GetValue().mb_str()).c_str() );

	camDistortion.resize(4);
	camDistortion[0] = atof( string(edK1->GetValue().mb_str()).c_str() );
	camDistortion[1] = atof( string(edK2->GetValue().mb_str()).c_str() );
	camDistortion[2] = atof( string(edP1->GetValue().mb_str()).c_str() );
	camDistortion[3] = atof( string(edP2->GetValue().mb_str()).c_str() );

	camFocalLen = atof( string(edFocalLen->GetValue().mb_str()).c_str() );

    if ( rbApply->GetSelection() == 0 )
    {
		idxToProcess = scIndex->GetValue();
		executeOperationOnRawlog( exec_setCurrentCamCfgByIdx, "CObservationImage objects changed: " );
    }
	else
	{
		labelToProcess = string(edLabel->GetValue().mb_str());
		executeOperationOnRawlog( exec_setCurrentCamCfgByLabel, "CObservationImage objects changed: " );
	}


	WX_END_TRY
}


void CFormChangeSensorPositions::OnrbApplySelect(wxCommandEvent& event)
{
    if ( rbApply->GetSelection() == 0 )
    {
		scIndex->Enable();
		edLabel->Disable();
    }
	else
	{
		scIndex->Disable();
		edLabel->Enable();
	}
}
