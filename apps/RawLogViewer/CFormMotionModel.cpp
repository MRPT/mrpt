/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "CFormMotionModel.h"

//(*InternalHeaders(CFormMotionModel)
#include <wx/font.h>
#include <wx/intl.h>
#include <wx/string.h>
//*)

#include "xRawLogViewerMain.h"
#include <wx/msgdlg.h>
#include <wx/filedlg.h>
#include <wx/app.h>
#include <wx/progdlg.h>
#include <wx/imaglist.h>
#include <wx/busyinfo.h>
#include <wx/log.h>
#include <wx/textdlg.h>
#include <mrpt/gui/wx28-fixes.h>

// General global variables:
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;

#define MAX_READ_FOR_MODEL_SEARCH 100


// The new options:
CActionRobotMovement2D::TMotionModelOptions		options;


//(*IdInit(CFormMotionModel)
const long CFormMotionModel::ID_STATICTEXT1 = wxNewId();
const long CFormMotionModel::ID_BUTTON1 = wxNewId();
const long CFormMotionModel::ID_HYPERLINKCTRL1 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT4 = wxNewId();
const long CFormMotionModel::ID_TEXTCTRL1 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT2 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT3 = wxNewId();
const long CFormMotionModel::ID_TEXTCTRL2 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT5 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT6 = wxNewId();
const long CFormMotionModel::ID_TEXTCTRL3 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT7 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT26 = wxNewId();
const long CFormMotionModel::ID_TEXTCTRL15 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT27 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT8 = wxNewId();
const long CFormMotionModel::ID_TEXTCTRL4 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT9 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT10 = wxNewId();
const long CFormMotionModel::ID_TEXTCTRL5 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT11 = wxNewId();
const long CFormMotionModel::ID_BUTTON10 = wxNewId();
const long CFormMotionModel::ID_BUTTON2 = wxNewId();
const long CFormMotionModel::ID_BUTTON8 = wxNewId();
const long CFormMotionModel::ID_PANEL2 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT12 = wxNewId();
const long CFormMotionModel::ID_TEXTCTRL6 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT13 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT14 = wxNewId();
const long CFormMotionModel::ID_TEXTCTRL7 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT15 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT16 = wxNewId();
const long CFormMotionModel::ID_TEXTCTRL8 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT17 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT18 = wxNewId();
const long CFormMotionModel::ID_TEXTCTRL9 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT19 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT20 = wxNewId();
const long CFormMotionModel::ID_TEXTCTRL10 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT21 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT29 = wxNewId();
const long CFormMotionModel::ID_TEXTCTRL17 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT30 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT31 = wxNewId();
const long CFormMotionModel::ID_TEXTCTRL18 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT32 = wxNewId();
const long CFormMotionModel::ID_BUTTON11 = wxNewId();
const long CFormMotionModel::ID_BUTTON3 = wxNewId();
const long CFormMotionModel::ID_BUTTON9 = wxNewId();
const long CFormMotionModel::ID_PANEL3 = wxNewId();
const long CFormMotionModel::ID_NOTEBOOK1 = wxNewId();
const long CFormMotionModel::ID_RADIOBUTTON1 = wxNewId();
const long CFormMotionModel::ID_BUTTON6 = wxNewId();
const long CFormMotionModel::ID_CHECKBOX1 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT34 = wxNewId();
const long CFormMotionModel::ID_TEXTCTRL19 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT33 = wxNewId();
const long CFormMotionModel::ID_TEXTCTRL20 = wxNewId();
const long CFormMotionModel::ID_RADIOBUTTON2 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT22 = wxNewId();
const long CFormMotionModel::ID_TEXTCTRL11 = wxNewId();
const long CFormMotionModel::ID_BUTTON4 = wxNewId();
const long CFormMotionModel::ID_BUTTON7 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT23 = wxNewId();
const long CFormMotionModel::ID_TEXTCTRL12 = wxNewId();
const long CFormMotionModel::ID_BUTTON5 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT25 = wxNewId();
const long CFormMotionModel::ID_TEXTCTRL13 = wxNewId();
const long CFormMotionModel::ID_TEXTCTRL14 = wxNewId();
const long CFormMotionModel::ID_TEXTCTRL16 = wxNewId();
const long CFormMotionModel::ID_PANEL1 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT24 = wxNewId();
const long CFormMotionModel::ID_CUSTOM1 = wxNewId();
const long CFormMotionModel::ID_STATICTEXT28 = wxNewId();
const long CFormMotionModel::ID_CUSTOM2 = wxNewId();
//*)

BEGIN_EVENT_TABLE(CFormMotionModel,wxDialog)
    //(*EventTable(CFormMotionModel)
    //*)
END_EVENT_TABLE()

CFormMotionModel::CFormMotionModel(wxWindow* parent,wxWindowID id)
{
    //(*Initialize(CFormMotionModel)
    wxStaticBoxSizer* boxRanges;
    wxFlexGridSizer* FlexGridSizer10;
    wxFlexGridSizer* FlexGridSizer3;
    wxFlexGridSizer* FlexGridSizer9;

    Create(parent, wxID_ANY, _("Modify motion model parameters"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE|wxRESIZE_BORDER|wxMAXIMIZE_BOX, _T("wxID_ANY"));
    SetClientSize(wxSize(1032,558));
    FlexGridSizer1 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer1->AddGrowableCol(0);
    FlexGridSizer1->AddGrowableCol(1);
    FlexGridSizer1->AddGrowableRow(0);
    FlexGridSizer2 = new wxFlexGridSizer(3, 1, 0, 0);
    FlexGridSizer2->AddGrowableCol(0);
    FlexGridSizer2->AddGrowableRow(1);
    FlexGridSizer4 = new wxFlexGridSizer(2, 2, 0, 0);
    FlexGridSizer4->AddGrowableCol(1);
    StaticText1 = new wxStaticText(this, ID_STATICTEXT1, _("Select the motion model and its parameters:"), wxPoint(9,12), wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    FlexGridSizer4->Add(StaticText1, 1, wxALL|wxEXPAND, 2);
    btnOk = new wxButton(this, ID_BUTTON1, _("CLOSE"), wxPoint(338,9), wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
    btnOk->SetToolTip(_("Closes & cancel the dialog"));
    FlexGridSizer4->Add(btnOk, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    HyperlinkCtrl1 = new wxHyperlinkCtrl(this, ID_HYPERLINKCTRL1, _("http://www.mrpt.org/tutorials/programming/odometry-and-motion-models/probabilistic_motion_models/"), _("http://www.mrpt.org/tutorials/programming/odometry-and-motion-models/probabilistic_motion_models/"), wxDefaultPosition, wxDefaultSize, wxHL_CONTEXTMENU|wxHL_ALIGN_CENTRE|wxNO_BORDER, _T("ID_HYPERLINKCTRL1"));
    FlexGridSizer4->Add(HyperlinkCtrl1, 1, wxALL|wxEXPAND, 1);
    FlexGridSizer2->Add(FlexGridSizer4, 1, wxALL|wxEXPAND, 0);
    PageControl1 = new wxNotebook(this, ID_NOTEBOOK1, wxDefaultPosition, wxDefaultSize, 0, _T("ID_NOTEBOOK1"));
    Panel2 = new wxPanel(PageControl1, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
    FlexGridSizer3 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer11 = new wxFlexGridSizer(7, 3, 0, 0);
    StaticText4 = new wxStaticText(Panel2, ID_STATICTEXT4, _("Ratio motion to x/y std.dev (alpha_1)="), wxDefaultPosition, wxDefaultSize, wxALIGN_RIGHT, _T("ID_STATICTEXT4"));
    FlexGridSizer11->Add(StaticText4, 1, wxALL|wxEXPAND, 5);
    edG_A1 = new wxTextCtrl(Panel2, ID_TEXTCTRL1, _("0.01"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    FlexGridSizer11->Add(edG_A1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText2 = new wxStaticText(Panel2, ID_STATICTEXT2, _("(meter/meter)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    FlexGridSizer11->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText3 = new wxStaticText(Panel2, ID_STATICTEXT3, _("Ratio rotation to x/y std.dev (alpha_2)="), wxDefaultPosition, wxDefaultSize, wxALIGN_RIGHT, _T("ID_STATICTEXT3"));
    FlexGridSizer11->Add(StaticText3, 1, wxALL|wxEXPAND, 5);
    edG_A2 = new wxTextCtrl(Panel2, ID_TEXTCTRL2, _("0.001"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL2"));
    FlexGridSizer11->Add(edG_A2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText5 = new wxStaticText(Panel2, ID_STATICTEXT5, _("(meter/deg)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT5"));
    FlexGridSizer11->Add(StaticText5, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText6 = new wxStaticText(Panel2, ID_STATICTEXT6, _("Ratio motion x/y to phi std.dev (alpha_3)="), wxDefaultPosition, wxDefaultSize, wxALIGN_RIGHT, _T("ID_STATICTEXT6"));
    FlexGridSizer11->Add(StaticText6, 1, wxALL|wxEXPAND, 5);
    edG_A3 = new wxTextCtrl(Panel2, ID_TEXTCTRL3, _("1.0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL3"));
    FlexGridSizer11->Add(edG_A3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText7 = new wxStaticText(Panel2, ID_STATICTEXT7, _("(deg/meter)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT7"));
    FlexGridSizer11->Add(StaticText7, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText26 = new wxStaticText(Panel2, ID_STATICTEXT26, _("Ratio rotation to phi std.dev (alpha_4)="), wxDefaultPosition, wxDefaultSize, wxALIGN_RIGHT, _T("ID_STATICTEXT26"));
    FlexGridSizer11->Add(StaticText26, 1, wxALL|wxEXPAND, 5);
    edG_A4 = new wxTextCtrl(Panel2, ID_TEXTCTRL15, _("0.05"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL15"));
    FlexGridSizer11->Add(edG_A4, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText27 = new wxStaticText(Panel2, ID_STATICTEXT27, _("(deg/deg)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT27"));
    FlexGridSizer11->Add(StaticText27, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText8 = new wxStaticText(Panel2, ID_STATICTEXT8, _("Minimum std.dev. of x/y (min_std_XY)="), wxDefaultPosition, wxDefaultSize, wxALIGN_RIGHT, _T("ID_STATICTEXT8"));
    FlexGridSizer11->Add(StaticText8, 1, wxALL|wxEXPAND, 5);
    edMinStdXY = new wxTextCtrl(Panel2, ID_TEXTCTRL4, _("0.010"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL4"));
    FlexGridSizer11->Add(edMinStdXY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText9 = new wxStaticText(Panel2, ID_STATICTEXT9, _("(meters)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT9"));
    FlexGridSizer11->Add(StaticText9, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText10 = new wxStaticText(Panel2, ID_STATICTEXT10, _("Minimum std.dev. of phi (min_std_PHI)="), wxDefaultPosition, wxDefaultSize, wxALIGN_RIGHT, _T("ID_STATICTEXT10"));
    FlexGridSizer11->Add(StaticText10, 1, wxALL|wxEXPAND, 5);
    edMinStdPHI = new wxTextCtrl(Panel2, ID_TEXTCTRL5, _("0.2"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL5"));
    FlexGridSizer11->Add(edMinStdPHI, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText11 = new wxStaticText(Panel2, ID_STATICTEXT11, _("(degrees)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT11"));
    FlexGridSizer11->Add(StaticText11, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnResetGauss = new wxButton(Panel2, ID_BUTTON10, _("Reset default values"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON10"));
    FlexGridSizer11->Add(btnResetGauss, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnGaussOK = new wxButton(Panel2, ID_BUTTON2, _("Apply"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
    btnGaussOK->SetDefault();
    btnGaussOK->SetFocus();
    wxFont btnGaussOKFont(10,wxFONTFAMILY_SWISS,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    btnGaussOK->SetFont(btnGaussOKFont);
    FlexGridSizer11->Add(btnGaussOK, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    btnSimulate = new wxButton(Panel2, ID_BUTTON8, _("Draw samples"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON8"));
    FlexGridSizer11->Add(btnSimulate, 1, wxALL|wxEXPAND, 5);
    FlexGridSizer3->Add(FlexGridSizer11, 1, wxALL|wxEXPAND, 0);
    Panel2->SetSizer(FlexGridSizer3);
    FlexGridSizer3->Fit(Panel2);
    FlexGridSizer3->SetSizeHints(Panel2);
    Panel3 = new wxPanel(PageControl1, ID_PANEL3, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL3"));
    FlexGridSizer5 = new wxFlexGridSizer(8, 3, 0, 0);
    StaticText12 = new wxStaticText(Panel3, ID_STATICTEXT12, _("alpha1_rot_rot="), wxDefaultPosition, wxDefaultSize, wxALIGN_RIGHT, _T("ID_STATICTEXT12"));
    FlexGridSizer5->Add(StaticText12, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edA1 = new wxTextCtrl(Panel3, ID_TEXTCTRL6, _("0.05"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL6"));
    FlexGridSizer5->Add(edA1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText13 = new wxStaticText(Panel3, ID_STATICTEXT13, _("(degs/deg)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT13"));
    FlexGridSizer5->Add(StaticText13, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText14 = new wxStaticText(Panel3, ID_STATICTEXT14, _("alpha2_rot_trans="), wxDefaultPosition, wxDefaultSize, wxALIGN_RIGHT, _T("ID_STATICTEXT14"));
    FlexGridSizer5->Add(StaticText14, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edA2 = new wxTextCtrl(Panel3, ID_TEXTCTRL7, _("4"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL7"));
    FlexGridSizer5->Add(edA2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText15 = new wxStaticText(Panel3, ID_STATICTEXT15, _("(degs/meter)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT15"));
    FlexGridSizer5->Add(StaticText15, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText16 = new wxStaticText(Panel3, ID_STATICTEXT16, _("alpha3_trans_trans="), wxDefaultPosition, wxDefaultSize, wxALIGN_RIGHT, _T("ID_STATICTEXT16"));
    FlexGridSizer5->Add(StaticText16, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edA3 = new wxTextCtrl(Panel3, ID_TEXTCTRL8, _("0.01"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL8"));
    FlexGridSizer5->Add(edA3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText17 = new wxStaticText(Panel3, ID_STATICTEXT17, _("(meters/meter)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT17"));
    FlexGridSizer5->Add(StaticText17, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText18 = new wxStaticText(Panel3, ID_STATICTEXT18, _("alpha4_trans_rot="), wxDefaultPosition, wxDefaultSize, wxALIGN_RIGHT, _T("ID_STATICTEXT18"));
    FlexGridSizer5->Add(StaticText18, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edA4 = new wxTextCtrl(Panel3, ID_TEXTCTRL9, _("0.0001"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL9"));
    FlexGridSizer5->Add(edA4, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText19 = new wxStaticText(Panel3, ID_STATICTEXT19, _("(meters/deg)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT19"));
    FlexGridSizer5->Add(StaticText19, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText20 = new wxStaticText(Panel3, ID_STATICTEXT20, _("Number of particles to generate="), wxDefaultPosition, wxDefaultSize, wxALIGN_RIGHT, _T("ID_STATICTEXT20"));
    FlexGridSizer5->Add(StaticText20, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edNumParts = new wxTextCtrl(Panel3, ID_TEXTCTRL10, _("300"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL10"));
    FlexGridSizer5->Add(edNumParts, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText21 = new wxStaticText(Panel3, ID_STATICTEXT21, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT21"));
    FlexGridSizer5->Add(StaticText21, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText29 = new wxStaticText(Panel3, ID_STATICTEXT29, _("Additional std. dev. of x/y (min_std_XY)="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT29"));
    FlexGridSizer5->Add(StaticText29, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edAddXY = new wxTextCtrl(Panel3, ID_TEXTCTRL17, _("0.001"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL17"));
    FlexGridSizer5->Add(edAddXY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText30 = new wxStaticText(Panel3, ID_STATICTEXT30, _("(meters)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT30"));
    FlexGridSizer5->Add(StaticText30, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText31 = new wxStaticText(Panel3, ID_STATICTEXT31, _("Additional std.dev. of phi (min_std_PHI)="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT31"));
    FlexGridSizer5->Add(StaticText31, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edAddPhi = new wxTextCtrl(Panel3, ID_TEXTCTRL18, _("0.050"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL18"));
    FlexGridSizer5->Add(edAddPhi, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText32 = new wxStaticText(Panel3, ID_STATICTEXT32, _("(degrees)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT32"));
    FlexGridSizer5->Add(StaticText32, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnResetThrun = new wxButton(Panel3, ID_BUTTON11, _("Reset default values"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON11"));
    FlexGridSizer5->Add(btnResetThrun, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnThrunOk = new wxButton(Panel3, ID_BUTTON3, _("Apply"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON3"));
    btnThrunOk->SetDefault();
    btnThrunOk->SetFocus();
    wxFont btnThrunOkFont(10,wxFONTFAMILY_SWISS,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    btnThrunOk->SetFont(btnThrunOkFont);
    FlexGridSizer5->Add(btnThrunOk, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    btnSimulateThrun = new wxButton(Panel3, ID_BUTTON9, _("Draw samples"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON9"));
    FlexGridSizer5->Add(btnSimulateThrun, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel3->SetSizer(FlexGridSizer5);
    FlexGridSizer5->Fit(Panel3);
    FlexGridSizer5->SetSizeHints(Panel3);
    PageControl1->AddPage(Panel2, _("Gaussian model"), false);
    PageControl1->AddPage(Panel3, _("Thrun\'s book model"), false);
    FlexGridSizer2->Add(PageControl1, 1, wxALL|wxEXPAND, 1);
    BoxSizer3 = new wxBoxSizer(wxHORIZONTAL);
    StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Apply to:"));
    BoxSizer4 = new wxBoxSizer(wxVERTICAL);
    BoxSizer5 = new wxBoxSizer(wxHORIZONTAL);
    FlexGridSizer6 = new wxFlexGridSizer(3, 5, 0, 0);
    FlexGridSizer6->AddGrowableCol(2);
    rbLoaded = new wxRadioButton(this, ID_RADIOBUTTON1, _("Loaded rawlog"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_RADIOBUTTON1"));
    FlexGridSizer6->Add(rbLoaded, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer6->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnGetFromCurrent = new wxButton(this, ID_BUTTON6, _("<-- Get model"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON6"));
    FlexGridSizer6->Add(btnGetFromCurrent, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    boxRanges = new wxStaticBoxSizer(wxHORIZONTAL, this, _(" Apply to range: "));
    FlexGridSizer9 = new wxFlexGridSizer(0, 2, 0, 0);
    cbAll = new wxCheckBox(this, ID_CHECKBOX1, _("All"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
    cbAll->SetValue(true);
    FlexGridSizer9->Add(cbAll, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer9->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText34 = new wxStaticText(this, ID_STATICTEXT34, _("Range:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT34"));
    FlexGridSizer9->Add(StaticText34, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer10 = new wxFlexGridSizer(0, 3, 0, 0);
    edRangeFrom = new wxTextCtrl(this, ID_TEXTCTRL19, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL19"));
    edRangeFrom->Disable();
    FlexGridSizer10->Add(edRangeFrom, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText33 = new wxStaticText(this, ID_STATICTEXT33, _("-"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT33"));
    FlexGridSizer10->Add(StaticText33, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edRangeTo = new wxTextCtrl(this, ID_TEXTCTRL20, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL20"));
    edRangeTo->Disable();
    FlexGridSizer10->Add(edRangeTo, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer9->Add(FlexGridSizer10, 1, wxALL|wxEXPAND, 0);
    boxRanges->Add(FlexGridSizer9, 1, wxALL|wxEXPAND, 0);
    FlexGridSizer6->Add(boxRanges, 1, wxALL|wxEXPAND, 5);
    FlexGridSizer6->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    rbFile = new wxRadioButton(this, ID_RADIOBUTTON2, _("Rawlog in file:"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_RADIOBUTTON2"));
    FlexGridSizer6->Add(rbFile, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    StaticText22 = new wxStaticText(this, ID_STATICTEXT22, _("Input file:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT22"));
    FlexGridSizer6->Add(StaticText22, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    txtInputFile = new wxTextCtrl(this, ID_TEXTCTRL11, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL11"));
    FlexGridSizer6->Add(txtInputFile, 1, wxALL|wxEXPAND, 5);
    btnPickInput = new wxButton(this, ID_BUTTON4, _("Select..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON4"));
    FlexGridSizer6->Add(btnPickInput, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 0);
    btnGetFromFile = new wxButton(this, ID_BUTTON7, _("<-- Get model"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON7"));
    FlexGridSizer6->Add(btnGetFromFile, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer6->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText23 = new wxStaticText(this, ID_STATICTEXT23, _("Output file:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT23"));
    FlexGridSizer6->Add(StaticText23, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    txtOutputFile = new wxTextCtrl(this, ID_TEXTCTRL12, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL12"));
    FlexGridSizer6->Add(txtOutputFile, 1, wxALL|wxEXPAND, 5);
    btnPickOut = new wxButton(this, ID_BUTTON5, _("Select..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON5"));
    FlexGridSizer6->Add(btnPickOut, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer5->Add(FlexGridSizer6, 1, wxALL|wxEXPAND, 0);
    BoxSizer4->Add(BoxSizer5, 1, wxALL|wxEXPAND, 0);
    StaticBoxSizer1->Add(BoxSizer4, 1, wxALL|wxEXPAND, 0);
    BoxSizer3->Add(StaticBoxSizer1, 1, wxALL|wxEXPAND, 0);
    FlexGridSizer2->Add(BoxSizer3, 1, wxALL|wxEXPAND, 0);
    FlexGridSizer1->Add(FlexGridSizer2, 1, wxALL|wxEXPAND, 0);
    FlexGridSizer7 = new wxFlexGridSizer(5, 1, 0, 0);
    FlexGridSizer7->AddGrowableCol(0);
    FlexGridSizer7->AddGrowableRow(2);
    Panel1 = new wxPanel(this, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxDOUBLE_BORDER|wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    FlexGridSizer8 = new wxFlexGridSizer(2, 3, 0, 0);
    StaticText25 = new wxStaticText(Panel1, ID_STATICTEXT25, _("(Ax,Ay,Aphi_deg)="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT25"));
    FlexGridSizer8->Add(StaticText25, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
    FlexGridSizer8->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer8->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    txtAx = new wxTextCtrl(Panel1, ID_TEXTCTRL13, _("0.3"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL13"));
    txtAx->SetToolTip(_("Odometry X increment (meter)"));
    FlexGridSizer8->Add(txtAx, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    txtAy = new wxTextCtrl(Panel1, ID_TEXTCTRL14, _("0.1"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL14"));
    txtAy->SetToolTip(_("Odometry Y increment (meter)"));
    FlexGridSizer8->Add(txtAy, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    txtAphi = new wxTextCtrl(Panel1, ID_TEXTCTRL16, _("10"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL16"));
    txtAphi->SetToolTip(_("Odometry angular increment (degrees)"));
    FlexGridSizer8->Add(txtAphi, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel1->SetSizer(FlexGridSizer8);
    FlexGridSizer8->Fit(Panel1);
    FlexGridSizer8->SetSizeHints(Panel1);
    FlexGridSizer7->Add(Panel1, 1, wxALL|wxEXPAND, 0);
    StaticText24 = new wxStaticText(this, ID_STATICTEXT24, _("Random samples (X-Y view):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT24"));
    FlexGridSizer7->Add(StaticText24, 1, wxALL|wxEXPAND, 0);
    plotSamplesXY = new mpWindow(this,ID_CUSTOM1,wxDefaultPosition,wxSize(313,327),0);
    FlexGridSizer7->Add(plotSamplesXY, 1, wxALL|wxEXPAND, 0);
    StaticText28 = new wxStaticText(this, ID_STATICTEXT28, _("Random samples (PHI view):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT28"));
    FlexGridSizer7->Add(StaticText28, 1, wxALL|wxEXPAND, 0);
    plotSamplesPHI = new mpWindow(this,ID_CUSTOM2,wxDefaultPosition,wxSize(336,130),0);
    FlexGridSizer7->Add(plotSamplesPHI, 1, wxALL|wxEXPAND, 0);
    FlexGridSizer1->Add(FlexGridSizer7, 1, wxALL|wxEXPAND, 0);
    SetSizer(FlexGridSizer1);
    SetSizer(FlexGridSizer1);
    Layout();

    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormMotionModel::OnbtnOkClick);
    Connect(ID_BUTTON10,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormMotionModel::OnbtnResetGaussClick);
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormMotionModel::OnbtnGaussOKClick);
    Connect(ID_BUTTON8,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormMotionModel::OnbtnSimulateClick);
    Connect(ID_BUTTON11,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormMotionModel::OnbtnResetThrunClick);
    Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormMotionModel::OnbtnThrunOkClick);
    Connect(ID_BUTTON9,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormMotionModel::OnbtnSimulateThrunClick);
    Connect(ID_RADIOBUTTON1,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&CFormMotionModel::OnrbLoadedSelect);
    Connect(ID_BUTTON6,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormMotionModel::OnbtnGetFromCurrentClick);
    Connect(ID_CHECKBOX1,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&CFormMotionModel::OncbAllClick);
    Connect(ID_RADIOBUTTON2,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&CFormMotionModel::OnrbFileSelect);
    Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormMotionModel::OnbtnPickInputClick);
    Connect(ID_BUTTON7,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormMotionModel::OnbtnGetFromFileClick);
    Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormMotionModel::OnbtnPickOutClick);
    Connect(wxID_ANY,wxEVT_INIT_DIALOG,(wxObjectEventFunction)&CFormMotionModel::OnInit);
    //*)

    // The graph:
    // -----------------------------------
    lyAction2D_XY       = new mpFXYVector();
    lyAction2D_PHI      = new mpFXYVector();

    lyAction2D_XY->SetPen( wxPen(wxColour(0,0,255),2) );
    lyAction2D_PHI->SetPen( wxPen(wxColour(0,0,255),3) );

    plotSamplesXY->AddLayer( new mpScaleX() );
    plotSamplesXY->AddLayer( new mpScaleY() );
    plotSamplesXY->AddLayer( lyAction2D_XY );

    plotSamplesPHI->AddLayer( new mpScaleX() );
    plotSamplesPHI->AddLayer( new mpScaleY() );
    plotSamplesPHI->AddLayer( lyAction2D_PHI );
}

CFormMotionModel::~CFormMotionModel()
{
    //(*Destroy(CFormMotionModel)
    //*)
}


void CFormMotionModel::OnbtnOkClick(wxCommandEvent& event)
{
    // Do nothing
    EndModal(0);
}

void CFormMotionModel::loadFromGaussian()
{
    // Gaussian selected:
    options.modelSelection = CActionRobotMovement2D::mmGaussian;
    options.gaussianModel.a1  = atof( edG_A1->GetValue().mb_str() );
    options.gaussianModel.a2  = atof( edG_A2->GetValue().mb_str() ) * 180/M_PIf;
    options.gaussianModel.a3  = atof( edG_A3->GetValue().mb_str() ) * M_PIf/180;
    options.gaussianModel.a4  = atof( edG_A4->GetValue().mb_str() );
    options.gaussianModel.minStdXY = atof( edMinStdXY->GetValue().mb_str() );
    options.gaussianModel.minStdPHI = DEG2RAD(atof( edMinStdPHI->GetValue().mb_str() ));
}

void CFormMotionModel::loadFromThrun()
{
    // Thrun selected:
    options.modelSelection = CActionRobotMovement2D::mmThrun;
    options.thrunModel.nParticlesCount 	= atoi( edNumParts->GetValue().mb_str());
    options.thrunModel.alfa1_rot_rot 	= atof( edA1->GetValue().mb_str() );
    options.thrunModel.alfa2_rot_trans 	= DEG2RAD( atof( edA2->GetValue().mb_str() ) );
    options.thrunModel.alfa3_trans_trans= atof( edA3->GetValue().mb_str() );
    options.thrunModel.alfa4_trans_rot 	= RAD2DEG( atof( edA4->GetValue().mb_str() ) );
    options.thrunModel.additional_std_XY = atof( edAddXY->GetValue().mb_str() );
    options.thrunModel.additional_std_phi = DEG2RAD(atof( edAddPhi->GetValue().mb_str() ));
}

void CFormMotionModel::applyToLoadedRawlog()
{
    WX_START_TRY

    // Apply changes:
    size_t changes = 0;
    {
        wxBusyCursor    cursor;

        size_t  first = 0;
        size_t  last  = rawlog.size()-1;

        if (!cbAll->GetValue())
        {
        	first = atoi( string(edRangeFrom->GetValue().mb_str()).c_str() );
        	last = atoi( string(edRangeTo->GetValue().mb_str()).c_str() );
        }

        for (size_t i=first;i<=last;i++)
        {
            // Check type:
            if ( rawlog.getType(i)==CRawlog::etActionCollection )
            {
                // This is an action:
                CActionCollectionPtr acts = rawlog.getAsAction(i);

                CActionRobotMovement2DPtr firstActionMov = acts->getActionByClass<CActionRobotMovement2D>( );

                if (firstActionMov)
                {
					if (firstActionMov->estimationMethod==CActionRobotMovement2D::emOdometry)
					{
						// Use the kinematics motion model to estimate a pose change gaussian approximation:
						firstActionMov->computeFromOdometry( firstActionMov->rawOdometryIncrementReading, options);
					}
					else
					{
						// Take the mean value
						firstActionMov->computeFromOdometry( firstActionMov->poseChange->getMeanVal(), options);
					}
					changes++;
                }
            } // end is action
        } // end for i
    }

    wxString str;
    str.sprintf(_("Objects changed: %d"),changes);
    wxMessageBox( str, _("Result:"), wxOK,this);

    WX_END_TRY
}

// Change movement model of a rawlog file:
void CFormMotionModel::applyToRawlogFile()
{
    WX_START_TRY

    if ( !txtInputFile->GetValue().size() )
        THROW_EXCEPTION("An input rawlog file must be selected")
        if ( !txtOutputFile->GetValue().size() )
            THROW_EXCEPTION("An output rawlog file must be selected")

	string   fileName_IN( txtInputFile->GetValue().mbc_str() );

	ASSERT_FILE_EXISTS_(fileName_IN)

	string   fileName_OUT( txtOutputFile->GetValue().mbc_str() );

    if (!fileName_OUT.compare(fileName_IN))
        THROW_EXCEPTION("Input and output files must be different!")

	CFileGZInputStream 	in_fil(fileName_IN);
    CFileGZOutputStream	out_fil(fileName_OUT);

    unsigned int	filSize = (unsigned int)in_fil.getTotalBytesCount();

    wxProgressDialog    progDia(
        wxT("Modifying motion model"),
        wxT("Processing file..."),
        filSize, // range
        this, // parent
        wxPD_CAN_ABORT |
        wxPD_APP_MODAL |
        wxPD_SMOOTH |
        wxPD_AUTO_HIDE |
        wxPD_ELAPSED_TIME |
        wxPD_ESTIMATED_TIME |
        wxPD_REMAINING_TIME);

    wxTheApp->Yield();  // Let the app. process messages

    unsigned int        countLoop = 0;
    bool                keepLoading=true;
    string              errorMsg;
    wxString			auxStr;

    // Apply changes:
    size_t changes = 0;
    wxBusyCursor    cursor;

    while (keepLoading)
    {
        if (countLoop++ % 100 == 0)
        {
            auxStr.sprintf(wxT("Processing... (%u objects processed)"),rawlog.size());
            if (!progDia.Update( (int)in_fil.getPosition(), auxStr ))
                keepLoading = false;
            wxTheApp->Yield();  // Let the app. process messages
        }

        CSerializablePtr newObj;
        try
        {
            in_fil >> newObj;
            // Check type:
            if ( newObj->GetRuntimeClass() == CLASS_ID(CRawlog))
            {
                newObj.clear();
                THROW_EXCEPTION("File is a 'CRawlog' object. Please save it as a sequence of actions/observations.")
            }
            else if ( newObj->GetRuntimeClass() == CLASS_ID(CSensoryFrame))
            {
                newObj.clear();
            }
            else if ( newObj->GetRuntimeClass() == CLASS_ID(CActionCollection))
            {
                // This is an action:
                CActionCollectionPtr acts = CActionCollectionPtr( newObj );

                CActionRobotMovement2DPtr  firstActionMov = acts->getActionByClass<CActionRobotMovement2D >();
                if (firstActionMov)
                {
					if (firstActionMov->estimationMethod==CActionRobotMovement2D::emOdometry)
					{
						// Use the kinematics motion model to estimate a pose change gaussian approximation:
						firstActionMov->computeFromOdometry( firstActionMov->rawOdometryIncrementReading, options);
					}
					else
					{
						// Take the mean value
						firstActionMov->computeFromOdometry( firstActionMov->poseChange->getMeanVal(), options);
					}
					changes++;
                }

                out_fil << acts;
                newObj.clear();
            }
            else
            {   // Unknown class:
                THROW_EXCEPTION_CUSTOM_MSG1( "Unexpected class found in the file: '%s'",newObj->GetRuntimeClass()->className );
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
    } // end while keep loading

    progDia.Update( filSize );	// Close dialog.

    wxString str;
    str.sprintf(_("Objects changed: %d\n\nEnd message:\n%s"),changes, errorMsg.c_str() );
    wxMessageBox( str, _("Result:"), wxOK,this);

    WX_END_TRY
}


void CFormMotionModel::OnbtnGaussOKClick(wxCommandEvent& event)
{
    // Process the data:
    loadFromGaussian();

    if (rbLoaded->GetValue())
        applyToLoadedRawlog();
    else	applyToRawlogFile();

//    EndModal(0);
}

void CFormMotionModel::OnbtnThrunOkClick(wxCommandEvent& event)
{
    // Process the data:
    loadFromThrun();

    if (rbLoaded->GetValue())
        applyToLoadedRawlog();
    else	applyToRawlogFile();

//    EndModal(0);
}

void CFormMotionModel::OnInit(wxInitDialogEvent& event)
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

        OnbtnGetFromCurrentClick( dumm );

    } // end there is loaded rawlog


}

void CFormMotionModel::OnrbFileSelect(wxCommandEvent& event)
{
    btnGetFromCurrent->Disable();

    btnGetFromFile->Enable();
    btnPickInput->Enable();
    btnPickOut->Enable();
    txtOutputFile->Enable();
    txtInputFile->Enable();

	cbAll->Disable();
	edRangeFrom->Disable();
	edRangeTo->Disable();
}

void CFormMotionModel::OnrbLoadedSelect(wxCommandEvent& event)
{
    btnGetFromCurrent->Enable();

	cbAll->Enable();
	edRangeFrom->Enable( !cbAll->GetValue() );
	edRangeTo->Enable( !cbAll->GetValue() );

    btnGetFromFile->Disable();
    btnPickInput->Disable();
    btnPickOut->Disable();
    txtOutputFile->Disable();
    txtInputFile->Disable();
}

void CFormMotionModel::OnbtnGetFromCurrentClick(wxCommandEvent& event)
{
    WX_START_TRY

    CActionRobotMovement2DPtr acts;
    for (size_t i=0;!acts && i<rawlog.size() && i<MAX_READ_FOR_MODEL_SEARCH;i++)
    {
    	if ( rawlog.getType(i)==CRawlog::etActionCollection )
        {
            CActionCollectionPtr actss=rawlog.getAsAction(i);
            acts=actss->getBestMovementEstimation();
        }
    }

    if (!acts)
    {
        wxMessageBox(_("I couldn't find any 2D robot action!!"),_("Error"),wxOK,this);
    }
    else
    {
        options = acts->motionModelConfiguration;

        showOptionsInDialog();

        if (CActionRobotMovement2D::mmGaussian == options.modelSelection)
            wxMessageBox(_("Current model is Gaussian. Modify it as desired and press 'Apply'"),_("Done"),wxOK,this);
        else    wxMessageBox(_("Current model is Thrun's. Modify it as desired and press 'Apply'"),_("Done"),wxOK,this);
    }

    WX_END_TRY
}

void CFormMotionModel::showOptionsInDialog()
{
    char    str[1000];
    if (CActionRobotMovement2D::mmGaussian == options.modelSelection)
    {
        PageControl1->ChangeSelection(0);

        sprintf(str,"%.8f",options.gaussianModel.a1 );
        edG_A1->SetValue( _U(str) );

        sprintf(str,"%.8f",options.gaussianModel.a2 * M_PIf/180 );
        edG_A2->SetValue( _U(str) );

        sprintf(str,"%.8f",options.gaussianModel.a3 * 180/M_PIf );
        edG_A3->SetValue( _U(str) );

        sprintf(str,"%.8f",options.gaussianModel.a4 );
        edG_A4->SetValue( _U(str) );

        sprintf(str,"%.8f",options.gaussianModel.minStdXY);
        edMinStdXY->SetValue( _U(str) );

        sprintf(str,"%.8f",RAD2DEG( options.gaussianModel.minStdPHI ));
        edMinStdPHI->SetValue( _U(str) );
    }
    else
    {
        PageControl1->ChangeSelection(1);

        sprintf(str,"%.8f",options.thrunModel.alfa1_rot_rot );
        edA1->SetValue( _U(str) );
        sprintf(str,"%.8f",options.thrunModel.alfa2_rot_trans * 180/M_PIf );
        edA2->SetValue( _U(str) );
        sprintf(str,"%.8f",options.thrunModel.alfa3_trans_trans );
        edA3->SetValue( _U(str) );
        sprintf(str,"%.8f",options.thrunModel.alfa4_trans_rot * M_PIf/180 );
        edA4->SetValue( _U(str) );

        sprintf(str,"%.8f",options.thrunModel.additional_std_XY);
        edAddXY->SetValue( _U(str) );

        sprintf(str,"%.8f",RAD2DEG( options.thrunModel.additional_std_phi ));
        edAddPhi->SetValue( _U(str) );

        sprintf(str,"%d",options.thrunModel.nParticlesCount );
        edNumParts->SetValue( _U(str) );
    }
}

// Draw random samples from "options"
void CFormMotionModel::drawRandomSamples()
{
    WX_START_TRY

    // Plot the 2D pose samples:
    unsigned int                    N = 1000;
    vector<float>                    xs(N),ys(N),ps(N),dumm(N,0.1f);

    CActionRobotMovement2D          act;
    CPose2D  odo(
        atof(txtAx->GetValue().mb_str()),
        atof(txtAy->GetValue().mb_str()),
        DEG2RAD(atof(txtAphi->GetValue().mb_str())) );

    // Load in the action:
    act.computeFromOdometry( odo, options );

    // Draw a set of random (x,y,phi) samples:
    //poseChange->draw drawManySamples( N, samples );

    // Pass to vectors and draw them:
    CPose2D     tmpPose;
    for (unsigned int i=0;i<N;i++)
    {
        act.drawSingleSample( tmpPose );
        xs[i] = tmpPose.x();
        ys[i] = tmpPose.y();
        ps[i] = RAD2DEG( tmpPose.phi() );
    }

    lyAction2D_XY->SetData(xs,ys);
    lyAction2D_PHI->SetData(ps,dumm);

    plotSamplesXY->Fit();
    plotSamplesPHI->Fit();

    WX_END_TRY
}

// Simulate Gauss:
void CFormMotionModel::OnbtnSimulateClick(wxCommandEvent& event)
{
    loadFromGaussian();
    drawRandomSamples();
}

void CFormMotionModel::OnbtnSimulateThrunClick(wxCommandEvent& event)
{
    loadFromThrun();
    drawRandomSamples();
}

void CFormMotionModel::OnbtnResetGaussClick(wxCommandEvent& event)
{
    CActionRobotMovement2D::TMotionModelOptions		opt;
    opt.modelSelection = CActionRobotMovement2D::mmGaussian;
    options = opt;
    showOptionsInDialog();
}

void CFormMotionModel::OnButton1Click(wxCommandEvent& event)
{
    CActionRobotMovement2D::TMotionModelOptions		opt;
    opt.modelSelection = CActionRobotMovement2D::mmThrun;
    options = opt;
    showOptionsInDialog();
}


void CFormMotionModel::OnbtnResetThrunClick(wxCommandEvent& event)
{
    CActionRobotMovement2D::TMotionModelOptions		opt;
    opt.modelSelection = CActionRobotMovement2D::mmThrun;
    options = opt;
    showOptionsInDialog();
}

// Select input file:
void CFormMotionModel::OnbtnPickInputClick(wxCommandEvent& event)
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

// Select output file:
void CFormMotionModel::OnbtnPickOutClick(wxCommandEvent& event)
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

// Get from file:
void CFormMotionModel::OnbtnGetFromFileClick(wxCommandEvent& event)
{
    WX_START_TRY

    string   fileName( txtInputFile->GetValue().mbc_str() );
    ASSERT_FILE_EXISTS_(fileName)

    CFileGZInputStream	fil(fileName);

    bool                keepLoading=true;
    string              errorMsg;
    CActionRobotMovement2DPtr acts;
    int		nLoaded = 0;

    while (keepLoading)
    {
        CSerializablePtr newObj;

        fil >> newObj;

        // Check type:
        if ( newObj->GetRuntimeClass() == CLASS_ID(CActionCollection))
        {
            acts=CActionCollectionPtr(newObj)->getBestMovementEstimation();
            if (acts)
                keepLoading = false;
        }
        else
            if ( newObj->GetRuntimeClass() == CLASS_ID(CSensoryFrame))
            {
                // Ignore
            }
            else
            {   // Unknown class:
                THROW_EXCEPTION_CUSTOM_MSG1("Unexpected class found in the file: '%s'",newObj->GetRuntimeClass()->className);
            }

        newObj.clear();  // FREE MEMORY!

        if (nLoaded++>MAX_READ_FOR_MODEL_SEARCH)
            keepLoading=false;

    } // end while keep loading

    if (!acts)
    {
        wxMessageBox(_("I couldn't find any 2D robot action!!"),_("Error"),wxOK,this);
    }
    else
    {
        options = acts->motionModelConfiguration;

        showOptionsInDialog();

        if (CActionRobotMovement2D::mmGaussian == options.modelSelection)
            wxMessageBox(_("Current model is Gaussian. Modify it as desired and press 'Apply'"),_("Done"),wxOK,this);
        else    wxMessageBox(_("Current model is Thrun's. Modify it as desired and press 'Apply'"),_("Done"),wxOK,this);
    }

    WX_END_TRY
}

void CFormMotionModel::OncbAllClick(wxCommandEvent& event)
{
	edRangeFrom->Enable( !cbAll->GetValue() );
	edRangeTo->Enable( !cbAll->GetValue() );
}
