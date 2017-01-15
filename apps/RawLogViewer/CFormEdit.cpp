/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "CFormEdit.h"

#include "xRawLogViewerMain.h"
#include <wx/msgdlg.h>

//(*InternalHeaders(CFormEdit)
#include <wx/intl.h>
#include <wx/string.h>
//*)
#include <wx/filedlg.h>
#include <wx/progdlg.h>
#include <wx/imaglist.h>
#include <wx/busyinfo.h>
#include <wx/app.h>

// General global variables:
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/stl_containers_utils.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CObservation2DRangeScan.h>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;



//(*IdInit(CFormEdit)
const long CFormEdit::ID_RADIOBUTTON1 = wxNewId();
const long CFormEdit::ID_RADIOBUTTON2 = wxNewId();
const long CFormEdit::ID_STATICTEXT22 = wxNewId();
const long CFormEdit::ID_TEXTCTRL11 = wxNewId();
const long CFormEdit::ID_BUTTON9 = wxNewId();
const long CFormEdit::ID_STATICTEXT23 = wxNewId();
const long CFormEdit::ID_TEXTCTRL12 = wxNewId();
const long CFormEdit::ID_BUTTON11 = wxNewId();
const long CFormEdit::ID_STATICTEXT1 = wxNewId();
const long CFormEdit::ID_SLIDER1 = wxNewId();
const long CFormEdit::ID_SPINCTRL1 = wxNewId();
const long CFormEdit::ID_STATICTEXT3 = wxNewId();
const long CFormEdit::ID_SLIDER2 = wxNewId();
const long CFormEdit::ID_SPINCTRL2 = wxNewId();
const long CFormEdit::ID_BUTTON1 = wxNewId();
const long CFormEdit::ID_BUTTON2 = wxNewId();
const long CFormEdit::ID_CHECKBOX1 = wxNewId();
const long CFormEdit::ID_CHECKBOX2 = wxNewId();
const long CFormEdit::ID_CHECKBOX3 = wxNewId();
const long CFormEdit::ID_CHECKBOX4 = wxNewId();
const long CFormEdit::ID_CHECKBOX5 = wxNewId();
const long CFormEdit::ID_BUTTON4 = wxNewId();
const long CFormEdit::ID_CHECKBOX6 = wxNewId();
const long CFormEdit::ID_CHECKBOX7 = wxNewId();
const long CFormEdit::ID_CHECKBOX8 = wxNewId();
const long CFormEdit::ID_CHECKBOX9 = wxNewId();
const long CFormEdit::ID_CHECKBOX10 = wxNewId();
const long CFormEdit::ID_BUTTON5 = wxNewId();
const long CFormEdit::ID_CHECKLISTBOX2 = wxNewId();
const long CFormEdit::ID_BUTTON7 = wxNewId();
const long CFormEdit::ID_BUTTON8 = wxNewId();
const long CFormEdit::ID_CHECKLISTBOX1 = wxNewId();
const long CFormEdit::ID_BUTTON10 = wxNewId();
const long CFormEdit::ID_BUTTON12 = wxNewId();
const long CFormEdit::ID_BUTTON13 = wxNewId();
const long CFormEdit::ID_STATICTEXT2 = wxNewId();
const long CFormEdit::ID_TEXTCTRL2 = wxNewId();
const long CFormEdit::ID_BUTTON3 = wxNewId();
const long CFormEdit::ID_BUTTON6 = wxNewId();
//*)

BEGIN_EVENT_TABLE(CFormEdit,wxDialog)
    //(*EventTable(CFormEdit)
    //*)
END_EVENT_TABLE()

CFormEdit::CFormEdit(wxWindow* parent,wxWindowID id)
{
    //(*Initialize(CFormEdit)
    wxFlexGridSizer* FlexGridSizer16;
    wxFlexGridSizer* FlexGridSizer10;
    wxFlexGridSizer* FlexGridSizer5;
    wxFlexGridSizer* FlexGridSizer15;
    wxFlexGridSizer* FlexGridSizer14;
    wxFlexGridSizer* FlexGridSizer13;
    wxFlexGridSizer* FlexGridSizer12;

    Create(parent, id, _("Edit the rawlog"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE|wxRESIZE_BORDER, _T("id"));
    FlexGridSizer1 = new wxFlexGridSizer(5, 1, 0, 0);
    FlexGridSizer1->AddGrowableCol(0);
    FlexGridSizer1->AddGrowableRow(0);
    FlexGridSizer1->AddGrowableRow(1);
    FlexGridSizer1->AddGrowableRow(2);
    FlexGridSizer1->AddGrowableRow(3);
    BoxSizer1 = new wxBoxSizer(wxHORIZONTAL);
    FlexGridSizer12 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer12->AddGrowableCol(0);
    FlexGridSizer12->AddGrowableCol(1);
    StaticBoxSizer6 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Apply to:"));
    BoxSizer4 = new wxBoxSizer(wxVERTICAL);
    BoxSizer5 = new wxBoxSizer(wxHORIZONTAL);
    FlexGridSizer7 = new wxFlexGridSizer(3, 4, 0, 0);
    FlexGridSizer7->AddGrowableCol(2);
    rbLoaded = new wxRadioButton(this, ID_RADIOBUTTON1, _("Loaded rawlog"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_RADIOBUTTON1"));
    FlexGridSizer7->Add(rbLoaded, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer7->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer7->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer7->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    rbFile = new wxRadioButton(this, ID_RADIOBUTTON2, _("Rawlog in file:"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_RADIOBUTTON2"));
    FlexGridSizer7->Add(rbFile, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    StaticText22 = new wxStaticText(this, ID_STATICTEXT22, _("Input file:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT22"));
    FlexGridSizer7->Add(StaticText22, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    txtInputFile = new wxTextCtrl(this, ID_TEXTCTRL11, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL11"));
    FlexGridSizer7->Add(txtInputFile, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnPickInput = new wxButton(this, ID_BUTTON9, _("Select..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON9"));
    FlexGridSizer7->Add(btnPickInput, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer7->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText23 = new wxStaticText(this, ID_STATICTEXT23, _("Output file:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT23"));
    FlexGridSizer7->Add(StaticText23, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    txtOutputFile = new wxTextCtrl(this, ID_TEXTCTRL12, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL12"));
    FlexGridSizer7->Add(txtOutputFile, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnPickOut = new wxButton(this, ID_BUTTON11, _("Select..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON11"));
    FlexGridSizer7->Add(btnPickOut, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    BoxSizer5->Add(FlexGridSizer7, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    BoxSizer4->Add(BoxSizer5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    StaticBoxSizer6->Add(BoxSizer4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer12->Add(StaticBoxSizer6, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Apply changes to this range:"));
    FlexGridSizer2 = new wxFlexGridSizer(2, 3, 0, 0);
    FlexGridSizer2->AddGrowableCol(1);
    FlexGridSizer2->AddGrowableRow(0);
    FlexGridSizer2->AddGrowableRow(1);
    StaticText1 = new wxStaticText(this, ID_STATICTEXT1, _("First:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    FlexGridSizer2->Add(StaticText1, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    slFrom = new wxSlider(this, ID_SLIDER1, 0, 0, 100, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_SLIDER1"));
    FlexGridSizer2->Add(slFrom, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    spinFirst = new wxSpinCtrl(this, ID_SPINCTRL1, _T("0"), wxDefaultPosition, wxDefaultSize, 0, 0, 100, 0, _T("ID_SPINCTRL1"));
    spinFirst->SetValue(_T("0"));
    FlexGridSizer2->Add(spinFirst, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    StaticText3 = new wxStaticText(this, ID_STATICTEXT3, _("Last:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
    FlexGridSizer2->Add(StaticText3, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    slTo = new wxSlider(this, ID_SLIDER2, 0, 0, 100, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_SLIDER2"));
    FlexGridSizer2->Add(slTo, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    spinLast = new wxSpinCtrl(this, ID_SPINCTRL2, _T("0"), wxDefaultPosition, wxDefaultSize, 0, 0, 100, 0, _T("ID_SPINCTRL2"));
    spinLast->SetValue(_T("0"));
    FlexGridSizer2->Add(spinLast, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer1->Add(FlexGridSizer2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer12->Add(StaticBoxSizer1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    BoxSizer1->Add(FlexGridSizer12, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer1->Add(BoxSizer1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    BoxSizer2 = new wxBoxSizer(wxHORIZONTAL);
    btnKeep = new wxButton(this, ID_BUTTON1, _("Keep the selected range only..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
    BoxSizer2->Add(btnKeep, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnDelete = new wxButton(this, ID_BUTTON2, _("Remove the selected range..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
    BoxSizer2->Add(btnDelete, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer1->Add(BoxSizer2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer8 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer8->AddGrowableCol(0);
    FlexGridSizer8->AddGrowableCol(1);
    StaticBoxSizer3 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Remove observations by index in sensorial frame:"));
    FlexGridSizer3 = new wxFlexGridSizer(1, 6, 0, 0);
    cbO0 = new wxCheckBox(this, ID_CHECKBOX1, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
    cbO0->SetValue(false);
    FlexGridSizer3->Add(cbO0, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbO1 = new wxCheckBox(this, ID_CHECKBOX2, _("1"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX2"));
    cbO1->SetValue(false);
    FlexGridSizer3->Add(cbO1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbO2 = new wxCheckBox(this, ID_CHECKBOX3, _("2"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX3"));
    cbO2->SetValue(false);
    FlexGridSizer3->Add(cbO2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbO3 = new wxCheckBox(this, ID_CHECKBOX4, _("3"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX4"));
    cbO3->SetValue(false);
    FlexGridSizer3->Add(cbO3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbO4 = new wxCheckBox(this, ID_CHECKBOX5, _("4"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX5"));
    cbO4->SetValue(false);
    FlexGridSizer3->Add(cbO4, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnDelObsIndx = new wxButton(this, ID_BUTTON4, _("Do it"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON4"));
    FlexGridSizer3->Add(btnDelObsIndx, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer3->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer8->Add(StaticBoxSizer3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    StaticBoxSizer4 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Remove actions by index in collection of actions:"));
    FlexGridSizer4 = new wxFlexGridSizer(1, 6, 0, 0);
    FlexGridSizer4->AddGrowableCol(5);
    FlexGridSizer4->AddGrowableRow(0);
    cbA0 = new wxCheckBox(this, ID_CHECKBOX6, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX6"));
    cbA0->SetValue(false);
    FlexGridSizer4->Add(cbA0, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbA1 = new wxCheckBox(this, ID_CHECKBOX7, _("1"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX7"));
    cbA1->SetValue(false);
    FlexGridSizer4->Add(cbA1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbA2 = new wxCheckBox(this, ID_CHECKBOX8, _("2"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX8"));
    cbA2->SetValue(false);
    FlexGridSizer4->Add(cbA2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbA3 = new wxCheckBox(this, ID_CHECKBOX9, _("3"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX9"));
    cbA3->SetValue(false);
    FlexGridSizer4->Add(cbA3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbA4 = new wxCheckBox(this, ID_CHECKBOX10, _("4"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX10"));
    cbA4->SetValue(false);
    FlexGridSizer4->Add(cbA4, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnRemActsIndx = new wxButton(this, ID_BUTTON5, _("Do it"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON5"));
    FlexGridSizer4->Add(btnRemActsIndx, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer4->Add(FlexGridSizer4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer8->Add(StaticBoxSizer4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer1->Add(FlexGridSizer8, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer9 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer9->AddGrowableCol(0);
    FlexGridSizer9->AddGrowableCol(1);
    StaticBoxSizer5 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Remove observations by CLASS:"));
    FlexGridSizer6 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer6->AddGrowableCol(0);
    FlexGridSizer5 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer5->AddGrowableCol(0);
    FlexGridSizer5->AddGrowableRow(0);
    cbObsClass = new wxCheckListBox(this, ID_CHECKLISTBOX2, wxDefaultPosition, wxSize(-1,200), 0, 0, 0, wxDefaultValidator, _T("ID_CHECKLISTBOX2"));
    cbObsClass->SetMaxSize(wxSize(-1,200));
    FlexGridSizer5->Add(cbObsClass, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer6->Add(FlexGridSizer5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer10 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer10->AddGrowableCol(0);
    FlexGridSizer10->AddGrowableCol(1);
    btnRemoveObsClass = new wxButton(this, ID_BUTTON7, _("Remove selected..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON7"));
    FlexGridSizer10->Add(btnRemoveObsClass, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnRemoveAllButByClass = new wxButton(this, ID_BUTTON8, _("Remove all but selected..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON8"));
    FlexGridSizer10->Add(btnRemoveAllButByClass, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer6->Add(FlexGridSizer10, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    StaticBoxSizer5->Add(FlexGridSizer6, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer9->Add(StaticBoxSizer5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    StaticBoxSizer7 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Remove observations by LABEL:"));
    FlexGridSizer11 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer11->AddGrowableCol(0);
    FlexGridSizer13 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer13->AddGrowableCol(0);
    FlexGridSizer13->AddGrowableRow(0);
    cbObsLabel = new wxCheckListBox(this, ID_CHECKLISTBOX1, wxDefaultPosition, wxSize(-1,200), 0, 0, 0, wxDefaultValidator, _T("ID_CHECKLISTBOX1"));
    cbObsLabel->SetMaxSize(wxSize(-1,200));
    FlexGridSizer13->Add(cbObsLabel, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer11->Add(FlexGridSizer13, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer14 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer14->AddGrowableCol(0);
    FlexGridSizer14->AddGrowableCol(1);
    btnRemByLabel = new wxButton(this, ID_BUTTON10, _("Remove matches..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON10"));
    FlexGridSizer14->Add(btnRemByLabel, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnRemByLabelNon = new wxButton(this, ID_BUTTON12, _("Remove non-matches..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON12"));
    FlexGridSizer14->Add(btnRemByLabelNon, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer11->Add(FlexGridSizer14, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    StaticBoxSizer7->Add(FlexGridSizer11, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer9->Add(StaticBoxSizer7, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer1->Add(FlexGridSizer9, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer16 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer16->AddGrowableCol(1);
    StaticBoxSizer2 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Miscellaneous"));
    FlexGridSizer15 = new wxFlexGridSizer(2, 3, 0, 0);
    btnLeaveHorizScans = new wxButton(this, ID_BUTTON13, _("Leave horizontal laser scans only..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON13"));
    FlexGridSizer15->Add(btnLeaveHorizScans, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    StaticText2 = new wxStaticText(this, ID_STATICTEXT2, _("Max. pitch (deg):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    FlexGridSizer15->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edMaxPitch = new wxTextCtrl(this, ID_TEXTCTRL2, _("1.5"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL2"));
    FlexGridSizer15->Add(edMaxPitch, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnImgSwap = new wxButton(this, ID_BUTTON3, _("Swap Red-Blue channels of all images..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON3"));
    FlexGridSizer15->Add(btnImgSwap, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticBoxSizer2->Add(FlexGridSizer15, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer16->Add(StaticBoxSizer2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnClose = new wxButton(this, ID_BUTTON6, _("Close"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON6"));
    FlexGridSizer16->Add(btnClose, 1, wxALL|wxALIGN_RIGHT|wxALIGN_BOTTOM, 5);
    FlexGridSizer1->Add(FlexGridSizer16, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    SetSizer(FlexGridSizer1);
    FlexGridSizer1->Fit(this);
    FlexGridSizer1->SetSizeHints(this);
    Center();

    Connect(ID_RADIOBUTTON1,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&CFormEdit::OnrbLoadedSelect);
    Connect(ID_RADIOBUTTON2,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&CFormEdit::OnrbFileSelect);
    Connect(ID_BUTTON9,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormEdit::OnbtnPickInputClick);
    Connect(ID_BUTTON11,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormEdit::OnbtnPickOutClick);
    Connect(ID_SLIDER1,wxEVT_SCROLL_THUMBTRACK,(wxObjectEventFunction)&CFormEdit::OnslFirstCmdScrollChanged);
    Connect(ID_SLIDER1,wxEVT_SCROLL_CHANGED,(wxObjectEventFunction)&CFormEdit::OnslFirstCmdScrollChanged);
    Connect(ID_SLIDER2,wxEVT_SCROLL_THUMBTRACK,(wxObjectEventFunction)&CFormEdit::OnslToCmdScrollChanged);
    Connect(ID_SLIDER2,wxEVT_SCROLL_CHANGED,(wxObjectEventFunction)&CFormEdit::OnslToCmdScrollChanged);
    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormEdit::OnbtnKeepClick);
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormEdit::OnbtnDeleteClick);
    Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormEdit::OnbtnDelObsIndxClick);
    Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormEdit::OnbtnRemActsIndxClick);
    Connect(ID_BUTTON7,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormEdit::OnbtnRemoveObsClassClick);
    Connect(ID_BUTTON8,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormEdit::OnbtnRemoveAllButByClassClick1);
    Connect(ID_BUTTON10,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormEdit::OnRemoveByLabel);
    Connect(ID_BUTTON12,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormEdit::OnRemoveButLabel);
    Connect(ID_BUTTON13,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormEdit::OnbtnLeaveHorizScansClick);
    Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormEdit::OnbtnImgSwapClick);
    Connect(ID_BUTTON6,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormEdit::OnbtnCloseClick);
    Connect(wxID_ANY,wxEVT_INIT_DIALOG,(wxObjectEventFunction)&CFormEdit::OnInit);
    //*)
}

CFormEdit::~CFormEdit()
{
    //(*Destroy(CFormEdit)
    //*)
}


// Load the selected items from list box:
void loadSelectionsFromListBox( vector_string &v, wxCheckListBox *c )
{
	v.clear();
	for (unsigned i=0;i<c->GetCount();i++)
		if (c->IsChecked(i))
			v.push_back( string(c->GetString(i).mb_str()) );
}


void CFormEdit::OnbtnCloseClick(wxCommandEvent& event)
{
    Close();
}

void CFormEdit::OnslFirstCmdScrollChanged(wxScrollEvent& event)
{
    int toVal = slTo->GetValue();
    int curVal = slFrom->GetValue();

    if (curVal>toVal) slFrom->SetValue(toVal);
    spinFirst->SetValue( slFrom->GetValue() );
    //lbFirst->SetLabel( wxString::Format(_("%d"),slFrom->GetValue()) );
}

void CFormEdit::OnslToCmdScrollChanged(wxScrollEvent& event)
{
    int fromVal = spinFirst->GetValue();
    int curVal = slTo->GetValue();

    if (curVal<fromVal) slTo->SetValue(fromVal);
//    lbLast->SetLabel( wxString::Format(_("%d"),slTo->GetValue()) );
    spinLast->SetValue( slTo->GetValue() );
}

void CFormEdit::OnbtnKeepClick(wxCommandEvent& event)
{
    WX_START_TRY

    size_t n = rawlog.size();
    size_t first = spinFirst->GetValue();
    size_t last = spinLast->GetValue();

    ASSERT_(last<n);
    ASSERT_(last>=first);

    wxBusyCursor wait;

    if (last<n-1)
        rawlog.remove(last+1,n-1);

    if (first>0)
        rawlog.remove(0,first-1);

    Close();

    WX_END_TRY
}

void CFormEdit::OnbtnDeleteClick(wxCommandEvent& event)
{
    WX_START_TRY

    size_t n = rawlog.size();
    size_t first = spinFirst->GetValue();
    size_t last = spinLast->GetValue();

    ASSERT_(last<n);
    ASSERT_(last>=first);

    wxBusyCursor wait;

    rawlog.remove(first,last);

    Close();

    WX_END_TRY

}


vector_bool     auxMask;

// Delete observations by index in their sensory frame.
void filter_delObsByIndex(
    mrpt::obs::CActionCollection *acts,
    mrpt::obs::CSensoryFrame *SF,
    int &changesCount  )
{
    if (SF)
    {
    	int j = 0;
        for (CSensoryFrame::iterator it=SF->begin();it!=SF->end();++j)
        {
            if (auxMask[j])
            {
                it = SF->erase(it);
                changesCount++;
            }
            else it++;
        }
    }
}

// Delete observations by index in their sensory frame.
void CFormEdit::OnbtnDelObsIndxClick(wxCommandEvent& event)
{
    WX_START_TRY

    auxMask.clear();
    auxMask.resize(5,false);
    auxMask[0] = cbO0->IsChecked();
    auxMask[1] = cbO1->IsChecked();
    auxMask[2] = cbO2->IsChecked();
    auxMask[3] = cbO3->IsChecked();
    auxMask[4] = cbO4->IsChecked();

    executeOperationOnRawlog( filter_delObsByIndex, "Observations deleted:" );

    WX_END_TRY

}


// Remove observations by class name:
vector_string classNameOfObsToRemove;

void filter_delObsByClass(
    mrpt::obs::CActionCollection *acts,
    mrpt::obs::CSensoryFrame *SF,
    int &changesCount  )
{
    if (SF)
    {
        for (CSensoryFrame::iterator it=SF->begin();it!=SF->end(); )
        {
            CObservationPtr obs = *it;
            if (string::npos!=find_in_vector(string(obs->GetRuntimeClass()->className), classNameOfObsToRemove) )
            {
                // A match: Delete it:
                it = SF->erase(it);
                changesCount++;
            }
            else it++;
        }
    }
}

void filter_remObsByClass(
    mrpt::obs::CActionCollection *acts,
    mrpt::obs::CSensoryFrame *SF,
    int &changesCount  )
{
    if (SF)
    {
        for (CSensoryFrame::iterator it=SF->begin();it!=SF->end(); )
        {
            CObservationPtr obs = *it;
            if (string::npos==find_in_vector(string(obs->GetRuntimeClass()->className), classNameOfObsToRemove) )
            {
                // A NO match: Delete it:
                it = SF->erase(it);
            }
            else it++;
        }
    }
}


// Remove observations by class name:
void CFormEdit::OnbtnRemoveObsClassClick(wxCommandEvent& event)
{
    WX_START_TRY

	loadSelectionsFromListBox( classNameOfObsToRemove, cbObsClass );
	if (classNameOfObsToRemove.empty()) THROW_EXCEPTION("You must select one class name!");

    executeOperationOnRawlog( filter_delObsByClass, "Observations deleted:" );
    Close();

    WX_END_TRY
}


// Delete actions by index.
void filter_delActsByIndex(
    mrpt::obs::CActionCollection *acts,
    mrpt::obs::CSensoryFrame *SF,
    int &changesCount )
{
    if (acts)
    {
    	int j=0;
        for (CActionCollection::iterator it=acts->begin();it!=acts->end(); ++j)
        {
            if (auxMask[j])
            {
                it = acts->erase(it);
                changesCount++;
            }
            else it++;
        }
    }
}


// Remove actions by index:
void CFormEdit::OnbtnRemActsIndxClick(wxCommandEvent& event)
{
    WX_START_TRY

    auxMask.clear();
    auxMask.resize(5,false);
    auxMask[0] = cbA0->IsChecked();
    auxMask[1] = cbA1->IsChecked();
    auxMask[2] = cbA2->IsChecked();
    auxMask[3] = cbA3->IsChecked();
    auxMask[4] = cbA4->IsChecked();

    executeOperationOnRawlog( filter_delActsByIndex, "Actions deleted:" );

    WX_END_TRY
}

void swapColors( CImage &img)
{
    int lx = img.getWidth();
    int ly = img.getHeight();
    for (int y=0;y<ly;y++)
    {
        for (int x=0;x<lx;x++)
        {
            unsigned char *pixels= img(x,y);
            unsigned char    temp = pixels[0];
            pixels[0] = pixels[2];
            pixels[2] = temp;
        }
    }
}


// Swap colors:
void filter_swapColors(
    mrpt::obs::CActionCollection *acts,
    mrpt::obs::CSensoryFrame *SF,
    int &changesCount )
{
    if (SF)
    {
        for (CSensoryFrame::iterator it=SF->begin();it!=SF->end();++it)
        {
            CObservationPtr obs = *it;

            if (IS_CLASS(obs,CObservationImage))
            {
                CObservationImage       *o = (CObservationImage*) obs.pointer();
                if (o->image.isColor())
                {
                    swapColors( o->image );
                    changesCount++;
                }
            }
            else
                if (IS_CLASS(obs,CObservationStereoImages))
                {
                    CObservationStereoImages    *o = (CObservationStereoImages*) obs.pointer();
                    if (o->imageLeft.isColor())
                    {
                        swapColors( o->imageLeft );
                        changesCount++;
                    }

                    if (o->imageRight.isColor())
                    {
                        swapColors( o->imageRight);
                        changesCount++;
                    }
                }
        }
    }
}

// Swap colors:
void CFormEdit::OnbtnImgSwapClick(wxCommandEvent& event)
{
    WX_START_TRY
    executeOperationOnRawlog( filter_swapColors, "Individual images modified:" );
    WX_END_TRY
}

void CFormEdit::OnInit(wxInitDialogEvent& event)
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

    FlexGridSizer1->RecalcSizes();

}

void CFormEdit::OnrbLoadedSelect(wxCommandEvent& event)
{
    btnPickInput->Disable();
    btnPickOut->Disable();
    txtOutputFile->Disable();
    txtInputFile->Disable();

    slFrom->Enable();
    slTo->Enable();
    spinFirst->Enable();
    spinLast->Enable();

    btnKeep->Enable();
    btnDelete->Enable();
}

void CFormEdit::OnrbFileSelect(wxCommandEvent& event)
{
    btnPickInput->Enable();
    btnPickOut->Enable();
    txtOutputFile->Enable();
    txtInputFile->Enable();

    slFrom->Disable();
    slTo->Disable();
    spinFirst->Disable();
    spinLast->Disable();

    btnKeep->Disable();
    btnDelete->Disable();
}

void CFormEdit::OnbtnRemoveAllButByClassClick1(wxCommandEvent& event)
{
    WX_START_TRY

	loadSelectionsFromListBox( classNameOfObsToRemove, cbObsClass );
	if (classNameOfObsToRemove.empty()) THROW_EXCEPTION("You must select one class name!");

    executeOperationOnRawlog( filter_remObsByClass, "Observations deleted:" );
    Close();

    WX_END_TRY
}

void CFormEdit::OnbtnPickInputClick(wxCommandEvent& event)
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

void CFormEdit::OnbtnPickOutClick(wxCommandEvent& event)
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


/** This is the common function for all operations over a rawlog file ("filter" a rawlog file into a new one) or over the loaded rawlog (depending on the user selection in the GUI).
  */
void CFormEdit::executeOperationOnRawlog( TRawlogFilter operation, const char *endMsg )
{
    WX_START_TRY

    unsigned int   	processMax;
    bool			isInMemory;
    CFileGZInputStream 	*in_fil=NULL;
    CFileGZOutputStream	*out_fil=NULL;
    size_t			first=0,last=0;

    if (rbLoaded->GetValue())
    {
        // APPLY TO rawlog in memory:
        isInMemory = true;

        processMax = (unsigned int)rawlog.size();
        first = spinFirst->GetValue();
        last  = spinLast->GetValue();

        ASSERT_(last<processMax);
        ASSERT_(last>=first);

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
        if (!fileExists(fileName_IN) )
            THROW_EXCEPTION("Input file does not exist!")

            string   fileName_OUT( txtOutputFile->GetValue().mbc_str() );

        if (!fileName_OUT.compare(fileName_IN))
            THROW_EXCEPTION("Input and output files must be different!")

		in_fil = new CFileGZInputStream(fileName_IN);
        out_fil = new CFileGZOutputStream	(fileName_OUT);

        processMax = (unsigned int)in_fil->getTotalBytesCount();
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

    unsigned int       countLoop = 0;
    bool                keepLoading=true;
    string              errorMsg;
    wxString			auxStr;

	CSensoryFramePtr dummy_sf = CSensoryFrame::Create();

    // Apply changes:
    int 	changes = 0;
    wxBusyCursor    cursor;

    while ( ( !isInMemory && keepLoading ) ||
            (  isInMemory && countLoop < rawlog.size() ) )
    {
        CSerializablePtr newObj;
        try
        {
            if (isInMemory)
            {
				try {
                newObj = rawlog.getAsGeneric(countLoop);
				} catch (std::exception &) {
					break;
				}
            }
            else
            {
                (*in_fil) >> newObj;
            }

            // Check type:
            if ( newObj->GetRuntimeClass() == CLASS_ID(CRawlog))
            {
                THROW_EXCEPTION("File is a 'CRawlog' object. Please save it as a sequence of actions/observations.")
            }
            else if ( newObj->GetRuntimeClass() == CLASS_ID(CSensoryFrame))
            {
                // A sensory frame:
                CSensoryFramePtr sf = CSensoryFramePtr( newObj );

                // Process & save:
                if (!isInMemory || (countLoop>=first && countLoop<=last) )
                    operation(NULL,sf.pointer(),changes );

                if (!isInMemory)  (*out_fil) << *sf;
            }
			else if ( newObj->GetRuntimeClass()->derivedFrom( CLASS_ID(CObservation)) )
            {
                // A single observation:
				dummy_sf->clear();
				dummy_sf->insert( CObservationPtr( newObj ) );

                // Process & save:
                if (!isInMemory || (countLoop>=first && countLoop<=last) )
                    operation(NULL,dummy_sf.pointer(),changes );

				// Still there?
				if (dummy_sf->size()==1)
				{
					if (!isInMemory)  (*out_fil) << *newObj;
				}
				else
				{
					if (isInMemory)
					{
						rawlog.remove(countLoop);
						countLoop--;
					}
				}

            }
            else if ( newObj->GetRuntimeClass() == CLASS_ID(CActionCollection))
            {
                // This is an action:
                CActionCollectionPtr acts = CActionCollectionPtr( newObj );

                // Process & save:
                if (!isInMemory || (countLoop>=first && countLoop<=last) )
                    operation((CActionCollection*)acts.pointer(),NULL,changes);

                if (!isInMemory)  (*out_fil) << *acts;
            }
            else
            {   // Unknown class:
                THROW_EXCEPTION(format("Unexpected class found in the file: '%s'",newObj->GetRuntimeClass()->className));
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
        if (countLoop++ % 20 == 0)
        {
            auxStr.sprintf(wxT("Processing... (%u objects processed)"),countLoop);
            int curProgr =  isInMemory ? countLoop : in_fil->getPosition();
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

    wxMessageBox(
		_U(format("%s %i\n\nEnd message:\n%s", endMsg, changes, errorMsg.c_str() ).c_str()),
		_("Result:"),
		wxOK,this);


    if (in_fil) delete in_fil;
    if (out_fil) delete out_fil;

    WX_END_TRY
}


// Remove observations by class name:
vector_string labelOfObsToRemove;

void filter_delObsByLabel(
    mrpt::obs::CActionCollection *acts,
    mrpt::obs::CSensoryFrame *SF,
    int &changesCount  )
{
    if (SF)
    {
        for (CSensoryFrame::iterator it=SF->begin();it!=SF->end(); )
        {
            CObservationPtr obs = *it;
            if (string::npos!=find_in_vector(obs->sensorLabel, labelOfObsToRemove) )
            {
                // A match: Delete it:
                it = SF->erase(it);
                changesCount++;
            }
            else it++;
        }
    }
}

void filter_NotDelObsByLabel(
    mrpt::obs::CActionCollection *acts,
    mrpt::obs::CSensoryFrame *SF,
    int &changesCount  )
{
    if (SF)
    {
        for (CSensoryFrame::iterator it=SF->begin();it!=SF->end(); )
        {
            CObservationPtr obs = *it;
            if (string::npos==find_in_vector(obs->sensorLabel, labelOfObsToRemove) )
            {
                // A match: Delete it:
                it = SF->erase(it);
                changesCount++;
            }
            else it++;
        }
    }
}

void CFormEdit::OnRemoveByLabel(wxCommandEvent& event)
{
    WX_START_TRY

	loadSelectionsFromListBox( labelOfObsToRemove, cbObsLabel);
	if (labelOfObsToRemove.empty()) THROW_EXCEPTION("You must select at least one label!");

    executeOperationOnRawlog( filter_delObsByLabel, "Observations deleted:" );
    Close();

    WX_END_TRY
}

void CFormEdit::OnRemoveButLabel(wxCommandEvent& event)
{
    WX_START_TRY

	loadSelectionsFromListBox( labelOfObsToRemove, cbObsLabel);
	if (labelOfObsToRemove.empty()) THROW_EXCEPTION("You must select at least one label!");

    executeOperationOnRawlog( filter_NotDelObsByLabel, "Observations deleted:" );
    Close();

    WX_END_TRY
}

void CFormEdit::OnslFromCmdScroll(wxScrollEvent& event)
{
}

void CFormEdit::OnslFromCmdScroll1(wxScrollEvent& event)
{
}


double  minPitchToDeleteLaserScan = DEG2RAD(1.2);

void leave_horizontalScans(
    mrpt::obs::CActionCollection *acts,
    mrpt::obs::CSensoryFrame *SF,
    int &changesCount  )
{
    if (SF)
    {
        for (CSensoryFrame::iterator it=SF->begin();it!=SF->end(); )
        {
            CObservationPtr obs = *it;

            if ( IS_CLASS(obs, CObservation2DRangeScan ) )
            {
            	CObservation2DRangeScan *o = static_cast<CObservation2DRangeScan*>( obs.pointer() );

            	if (fabs( o->sensorPose.pitch() )>minPitchToDeleteLaserScan )
            	{
					it = SF->erase(it);
            	}
            	else
            	{
					changesCount++; // Count success
					it++;
            	}
            }
            else it++;
        }
    }
}

// Leave horizontal scans only:
void CFormEdit::OnbtnLeaveHorizScansClick(wxCommandEvent& event)
{
   WX_START_TRY

    minPitchToDeleteLaserScan =  atof(string(edMaxPitch->GetValue().mb_str()).c_str());
    executeOperationOnRawlog( leave_horizontalScans, "Laser scans that passed the filter: " );

    WX_END_TRY
}
