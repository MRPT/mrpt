/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "robotic_arm_kinematicsMain.h"
#include <wx/msgdlg.h>

//(*InternalHeaders(robotic_arm_kinematicsFrame)
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/intl.h>
#include <wx/font.h>
#include <wx/bitmap.h>
#include <wx/icon.h>
#include <wx/image.h>
#include <wx/artprov.h>
//*)
#include <mrpt/gui/wx28-fixes.h>

#include "CAboutBox.h"

#include <mrpt/gui/WxUtils.h>

#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/stock_objects.h>


#include "imgs/main_icon.xpm"
#include "../wx-common/mrpt_logo.xpm"

// A custom Art provider for customizing the icons:
class MyArtProvider : public wxArtProvider
{
protected:
    virtual wxBitmap CreateBitmap(const wxArtID& id,
                                  const wxArtClient& client,
                                  const wxSize& size);
};

// CreateBitmap function
wxBitmap MyArtProvider::CreateBitmap(const wxArtID& id,
                                     const wxArtClient& client,
                                     const wxSize& size)
{
    if (id == wxART_MAKE_ART_ID(MAIN_ICON))   return wxBitmap(main_icon_xpm);
    if (id == wxART_MAKE_ART_ID(IMG_MRPT_LOGO))  return wxBitmap(mrpt_logo_xpm);

    // Any wxWidgets icons not implemented here
    // will be provided by the default art provider.
    return wxNullBitmap;
}

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::kinematics;
using namespace std;


//(*IdInit(robotic_arm_kinematicsFrame)
const long robotic_arm_kinematicsFrame::ID_RADIOBOX2 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_STATICTEXT1 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_SIMPLEHTMLLISTBOX1 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_BUTTON5 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_BUTTON6 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_BUTTON7 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_RADIOBOX1 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_STATICLINE4 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_SIMPLEHTMLLISTBOX2 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_TEXTCTRL1 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_STATICTEXT3 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_BUTTON1 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_SLIDER1 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_STATICLINE1 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_SIMPLEHTMLLISTBOX3 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_TEXTCTRL2 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_STATICTEXT5 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_BUTTON2 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_SLIDER2 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_STATICLINE2 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_SIMPLEHTMLLISTBOX4 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_TEXTCTRL3 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_STATICTEXT7 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_BUTTON3 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_SLIDER3 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_STATICLINE3 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_SIMPLEHTMLLISTBOX5 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_TEXTCTRL4 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_STATICTEXT9 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_BUTTON4 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_SLIDER4 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_PANEL1 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_XY_GLCANVAS = wxNewId();
const long robotic_arm_kinematicsFrame::ID_STATICTEXT10 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_PANEL2 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_STATICTEXT2 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_LISTBOX1 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_TEXTCTRL5 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_PANEL3 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_NOTEBOOK1 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_MENUITEM3 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_MENUITEM1 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_MENUITEM2 = wxNewId();
const long robotic_arm_kinematicsFrame::idMenuQuit = wxNewId();
const long robotic_arm_kinematicsFrame::idMenuAbout = wxNewId();
//*)

BEGIN_EVENT_TABLE(robotic_arm_kinematicsFrame,wxFrame)
    //(*EventTable(robotic_arm_kinematicsFrame)
    //*)
END_EVENT_TABLE()


robotic_arm_kinematicsFrame *the_win=NULL;


robotic_arm_kinematicsFrame::robotic_arm_kinematicsFrame(wxWindow* parent,wxWindowID id)
{
	the_win = this;

    // Load my custom icons:
#if wxCHECK_VERSION(2, 8, 0)
    wxArtProvider::Push(new MyArtProvider);
#else
    wxArtProvider::PushProvider(new MyArtProvider);
#endif

    //(*Initialize(robotic_arm_kinematicsFrame)
    wxMenuItem* MenuItem2;
    wxMenuItem* MenuItem1;
    wxFlexGridSizer* FlexGridSizer8;
    wxFlexGridSizer* FlexGridSizer1;
    wxFlexGridSizer* FlexGridSizer2;
    wxFlexGridSizer* FlexGridSizer15;
    wxMenu* Menu1;
    wxFlexGridSizer* FlexGridSizer17;
    wxFlexGridSizer* FlexGridSizer19;
    wxFlexGridSizer* FlexGridSizer11;
    wxFlexGridSizer* FlexGridSizer7;
    wxFlexGridSizer* FlexGridSizer4;
    wxFlexGridSizer* FlexGridSizer9;
    wxFlexGridSizer* FlexGridSizer14;
    wxFlexGridSizer* FlexGridSizer6;
    wxFlexGridSizer* FlexGridSizer3;
    wxFlexGridSizer* FlexGridSizer16;
    wxFlexGridSizer* FlexGridSizer10;
    wxFlexGridSizer* FlexGridSizer13;
    wxMenuBar* MenuBar1;
    wxFlexGridSizer* FlexGridSizer18;
    wxFlexGridSizer* FlexGridSizer12;
    wxMenu* Menu2;
    wxFlexGridSizer* FlexGridSizer5;
    wxStaticBoxSizer* boxProperties;

    Create(parent, wxID_ANY, _("Robotic Arm Kinematic GUI - Part of MRPT"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("wxID_ANY"));
    SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BTNFACE));
    {
    wxIcon FrameIcon;
    FrameIcon.CopyFromBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("MAIN_ICON")),wxART_FRAME_ICON));
    SetIcon(FrameIcon);
    }
    FlexGridSizer1 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer1->AddGrowableCol(1);
    FlexGridSizer1->AddGrowableRow(0);
    FlexGridSizer2 = new wxFlexGridSizer(5, 1, 0, 0);
    FlexGridSizer2->AddGrowableCol(0);
    FlexGridSizer2->AddGrowableRow(2);
    wxString __wxRadioBoxChoices_1[3] =
    {
    _("Z"),
    _("Y"),
    _("X")
    };
    RadioBox1 = new wxRadioBox(this, ID_RADIOBOX2, _(" Orientation of first DOF "), wxDefaultPosition, wxDefaultSize, 3, __wxRadioBoxChoices_1, 1, wxRA_VERTICAL, wxDefaultValidator, _T("ID_RADIOBOX2"));
    FlexGridSizer2->Add(RadioBox1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    StaticText1 = new wxStaticText(this, ID_STATICTEXT1, _("List of kinematic links:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    FlexGridSizer2->Add(StaticText1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    listLinks = new wxSimpleHtmlListBox(this, ID_SIMPLEHTMLLISTBOX1, wxDefaultPosition, wxDefaultSize, 0, 0, wxHLB_DEFAULT_STYLE, wxDefaultValidator, _T("ID_SIMPLEHTMLLISTBOX1"));
    listLinks->Append(_("aasas"));
    listLinks->Append(_("a<font color=\"red\">s</font>asas"));
    listLinks->SetMinSize(wxSize(360,30));
    FlexGridSizer2->Add(listLinks, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    FlexGridSizer15 = new wxFlexGridSizer(1, 3, 0, 0);
    btnAddLink = new wxButton(this, ID_BUTTON5, _("Add new link"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON5"));
    FlexGridSizer15->Add(btnAddLink, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnClear = new wxButton(this, ID_BUTTON6, _("Clear all"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON6"));
    FlexGridSizer15->Add(btnClear, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnDelete = new wxButton(this, ID_BUTTON7, _("Delete selected"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON7"));
    FlexGridSizer15->Add(btnDelete, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer2->Add(FlexGridSizer15, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    panelProperties = new wxPanel(this, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    panelProperties->Disable();
    boxProperties = new wxStaticBoxSizer(wxHORIZONTAL, panelProperties, _(" Link properties: "));
    FlexGridSizer3 = new wxFlexGridSizer(9, 1, 0, 0);
    FlexGridSizer3->AddGrowableCol(0);
    wxString __wxRadioBoxChoices_2[2] =
    {
    _("Revolute"),
    _("Prismatic")
    };
    rbType = new wxRadioBox(panelProperties, ID_RADIOBOX1, _("Type"), wxDefaultPosition, wxDefaultSize, 2, __wxRadioBoxChoices_2, 2, wxRA_HORIZONTAL, wxDefaultValidator, _T("ID_RADIOBOX1"));
    FlexGridSizer3->Add(rbType, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    StaticLine4 = new wxStaticLine(panelProperties, ID_STATICLINE4, wxDefaultPosition, wxSize(10,-1), wxLI_HORIZONTAL, _T("ID_STATICLINE4"));
    FlexGridSizer3->Add(StaticLine4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    FlexGridSizer5 = new wxFlexGridSizer(3, 1, 0, 0);
    FlexGridSizer5->AddGrowableCol(0);
    SimpleHtmlListBox2 = new wxSimpleHtmlListBox(panelProperties, ID_SIMPLEHTMLLISTBOX2, wxDefaultPosition, wxSize(-1,25), 0, 0, wxNO_BORDER, wxDefaultValidator, _T("ID_SIMPLEHTMLLISTBOX2"));
    SimpleHtmlListBox2->Append(_("&theta;<sub>i</sub> (Angle between X<sub>i</sub> and  X<sub>i+1</sub>)"));
    SimpleHtmlListBox2->Disable();
    SimpleHtmlListBox2->SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BTNFACE));
    FlexGridSizer5->Add(SimpleHtmlListBox2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    FlexGridSizer6 = new wxFlexGridSizer(0, 3, 0, 0);
    FlexGridSizer6->AddGrowableCol(0);
    edTheta = new wxTextCtrl(panelProperties, ID_TEXTCTRL1, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    FlexGridSizer6->Add(edTheta, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    StaticText3 = new wxStaticText(panelProperties, ID_STATICTEXT3, _("(deg)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
    FlexGridSizer6->Add(StaticText3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnTh = new wxButton(panelProperties, ID_BUTTON1, _("Save"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
    btnTh->SetDefault();
    FlexGridSizer6->Add(btnTh, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer5->Add(FlexGridSizer6, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    slTheta = new wxSlider(panelProperties, ID_SLIDER1, 0, -180, 180, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_SLIDER1"));
    FlexGridSizer5->Add(slTheta, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    FlexGridSizer3->Add(FlexGridSizer5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    StaticLine1 = new wxStaticLine(panelProperties, ID_STATICLINE1, wxDefaultPosition, wxSize(10,-1), wxLI_HORIZONTAL, _T("ID_STATICLINE1"));
    FlexGridSizer3->Add(StaticLine1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    FlexGridSizer8 = new wxFlexGridSizer(3, 1, 0, 0);
    FlexGridSizer8->AddGrowableCol(0);
    SimpleHtmlListBox3 = new wxSimpleHtmlListBox(panelProperties, ID_SIMPLEHTMLLISTBOX3, wxDefaultPosition, wxSize(-1,25), 0, 0, wxNO_BORDER, wxDefaultValidator, _T("ID_SIMPLEHTMLLISTBOX3"));
    SimpleHtmlListBox3->Append(_("d<sub>i</sub> (Distance to common normal)"));
    SimpleHtmlListBox3->Disable();
    SimpleHtmlListBox3->SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BTNFACE));
    FlexGridSizer8->Add(SimpleHtmlListBox3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    FlexGridSizer9 = new wxFlexGridSizer(0, 3, 0, 0);
    FlexGridSizer9->AddGrowableCol(0);
    edD = new wxTextCtrl(panelProperties, ID_TEXTCTRL2, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL2"));
    FlexGridSizer9->Add(edD, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    StaticText5 = new wxStaticText(panelProperties, ID_STATICTEXT5, _("(mm)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT5"));
    FlexGridSizer9->Add(StaticText5, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnD = new wxButton(panelProperties, ID_BUTTON2, _("Save"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
    btnD->SetDefault();
    FlexGridSizer9->Add(btnD, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer8->Add(FlexGridSizer9, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    slD = new wxSlider(panelProperties, ID_SLIDER2, 0, -2000, 2000, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_SLIDER2"));
    FlexGridSizer8->Add(slD, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    FlexGridSizer3->Add(FlexGridSizer8, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    StaticLine2 = new wxStaticLine(panelProperties, ID_STATICLINE2, wxDefaultPosition, wxSize(10,-1), wxLI_HORIZONTAL, _T("ID_STATICLINE2"));
    FlexGridSizer3->Add(StaticLine2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    FlexGridSizer10 = new wxFlexGridSizer(3, 1, 0, 0);
    FlexGridSizer10->AddGrowableCol(0);
    SimpleHtmlListBox4 = new wxSimpleHtmlListBox(panelProperties, ID_SIMPLEHTMLLISTBOX4, wxDefaultPosition, wxSize(-1,25), 0, 0, wxNO_BORDER, wxDefaultValidator, _T("ID_SIMPLEHTMLLISTBOX4"));
    SimpleHtmlListBox4->Append(_("a<sub>i</sub> (Common normal length)"));
    SimpleHtmlListBox4->Disable();
    SimpleHtmlListBox4->SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BTNFACE));
    FlexGridSizer10->Add(SimpleHtmlListBox4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    FlexGridSizer11 = new wxFlexGridSizer(0, 3, 0, 0);
    FlexGridSizer11->AddGrowableCol(0);
    edA = new wxTextCtrl(panelProperties, ID_TEXTCTRL3, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL3"));
    FlexGridSizer11->Add(edA, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    StaticText7 = new wxStaticText(panelProperties, ID_STATICTEXT7, _("(mm)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT7"));
    FlexGridSizer11->Add(StaticText7, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnA = new wxButton(panelProperties, ID_BUTTON3, _("Save"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON3"));
    btnA->SetDefault();
    FlexGridSizer11->Add(btnA, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer10->Add(FlexGridSizer11, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    slA = new wxSlider(panelProperties, ID_SLIDER3, 0, -2000, 2000, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_SLIDER3"));
    FlexGridSizer10->Add(slA, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    FlexGridSizer3->Add(FlexGridSizer10, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    StaticLine3 = new wxStaticLine(panelProperties, ID_STATICLINE3, wxDefaultPosition, wxSize(10,-1), wxLI_HORIZONTAL, _T("ID_STATICLINE3"));
    FlexGridSizer3->Add(StaticLine3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    FlexGridSizer12 = new wxFlexGridSizer(3, 1, 0, 0);
    FlexGridSizer12->AddGrowableCol(0);
    SimpleHtmlListBox5 = new wxSimpleHtmlListBox(panelProperties, ID_SIMPLEHTMLLISTBOX5, wxDefaultPosition, wxSize(-1,25), 0, 0, wxNO_BORDER, wxDefaultValidator, _T("ID_SIMPLEHTMLLISTBOX5"));
    SimpleHtmlListBox5->Append(_("&alpha;<sub>i</sub> (Angle Z<sub>i</sub> to Z<sub>i+1</sub>)"));
    SimpleHtmlListBox5->Disable();
    SimpleHtmlListBox5->SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BTNFACE));
    FlexGridSizer12->Add(SimpleHtmlListBox5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    FlexGridSizer13 = new wxFlexGridSizer(0, 3, 0, 0);
    FlexGridSizer13->AddGrowableCol(0);
    edAlpha = new wxTextCtrl(panelProperties, ID_TEXTCTRL4, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL4"));
    FlexGridSizer13->Add(edAlpha, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    StaticText9 = new wxStaticText(panelProperties, ID_STATICTEXT9, _("(deg)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT9"));
    FlexGridSizer13->Add(StaticText9, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnAlpha = new wxButton(panelProperties, ID_BUTTON4, _("Save"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON4"));
    btnAlpha->SetDefault();
    FlexGridSizer13->Add(btnAlpha, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer12->Add(FlexGridSizer13, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    slAlpha = new wxSlider(panelProperties, ID_SLIDER4, 0, -180, 180, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_SLIDER4"));
    FlexGridSizer12->Add(slAlpha, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
    FlexGridSizer3->Add(FlexGridSizer12, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    boxProperties->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    panelProperties->SetSizer(boxProperties);
    boxProperties->Fit(panelProperties);
    boxProperties->SetSizeHints(panelProperties);
    FlexGridSizer2->Add(panelProperties, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer1->Add(FlexGridSizer2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer4 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer4->AddGrowableCol(0);
    FlexGridSizer4->AddGrowableRow(0);
    m_plot3D = new CMyGLCanvas(this,ID_XY_GLCANVAS,wxDefaultPosition,wxSize(450,350),wxTAB_TRAVERSAL,_T("ID_XY_GLCANVAS"));
    FlexGridSizer4->Add(m_plot3D, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer16 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer16->AddGrowableCol(0);
    FlexGridSizer16->AddGrowableRow(0);
    FlexGridSizer7 = new wxFlexGridSizer(2, 2, 0, 0);
    FlexGridSizer7->AddGrowableCol(1);
    FlexGridSizer7->AddGrowableRow(1);
    FlexGridSizer7->Add(1,15,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer14 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer14->AddGrowableCol(0);
    StaticText10 = new wxStaticText(this, ID_STATICTEXT10, _("Degrees of Freedom"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT10"));
    wxFont StaticText10Font = wxSystemSettings::GetFont(wxSYS_OEM_FIXED_FONT);
    if ( !StaticText10Font.Ok() ) StaticText10Font = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
    StaticText10Font.SetPointSize(8);
    StaticText10Font.SetWeight(wxFONTWEIGHT_BOLD);
    StaticText10->SetFont(StaticText10Font);
    FlexGridSizer14->Add(StaticText10, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer7->Add(FlexGridSizer14, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer7->Add(3,55,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    pnDOFs = new wxPanel(this, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
    boxSizerDOFs = new wxBoxSizer(wxHORIZONTAL);
    pnDOFs->SetSizer(boxSizerDOFs);
    boxSizerDOFs->Fit(pnDOFs);
    boxSizerDOFs->SetSizeHints(pnDOFs);
    FlexGridSizer7->Add(pnDOFs, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer16->Add(FlexGridSizer7, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer17 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer17->AddGrowableCol(0);
    FlexGridSizer17->AddGrowableRow(1);
    FlexGridSizer18 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer18->AddGrowableCol(0);
    StaticText2 = new wxStaticText(this, ID_STATICTEXT2, _("State details"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    wxFont StaticText2Font = wxSystemSettings::GetFont(wxSYS_OEM_FIXED_FONT);
    if ( !StaticText2Font.Ok() ) StaticText2Font = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
    StaticText2Font.SetPointSize(8);
    StaticText2Font.SetWeight(wxFONTWEIGHT_BOLD);
    StaticText2->SetFont(StaticText2Font);
    FlexGridSizer18->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer17->Add(FlexGridSizer18, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Notebook1 = new wxNotebook(this, ID_NOTEBOOK1, wxDefaultPosition, wxDefaultSize, wxNB_BOTTOM, _T("ID_NOTEBOOK1"));
    Panel1 = new wxPanel(Notebook1, ID_PANEL3, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL3"));
    FlexGridSizer19 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer19->AddGrowableCol(1);
    FlexGridSizer19->AddGrowableRow(0);
    lbXYZs = new wxListBox(Panel1, ID_LISTBOX1, wxDefaultPosition, wxSize(45,-1), 0, 0, 0, wxDefaultValidator, _T("ID_LISTBOX1"));
    wxFont lbXYZsFont(7,wxFONTFAMILY_TELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    lbXYZs->SetFont(lbXYZsFont);
    FlexGridSizer19->Add(lbXYZs, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    edMatrix = new wxTextCtrl(Panel1, ID_TEXTCTRL5, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE|wxTE_READONLY|wxTE_DONTWRAP|wxALWAYS_SHOW_SB, wxDefaultValidator, _T("ID_TEXTCTRL5"));
    edMatrix->SetMinSize(wxSize(300,-1));
    wxFont edMatrixFont(7,wxFONTFAMILY_TELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,_T("Monospace"),wxFONTENCODING_DEFAULT);
    edMatrix->SetFont(edMatrixFont);
    FlexGridSizer19->Add(edMatrix, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel1->SetSizer(FlexGridSizer19);
    FlexGridSizer19->Fit(Panel1);
    FlexGridSizer19->SetSizeHints(Panel1);
    Notebook1->AddPage(Panel1, _("Transf. matrices"), false);
    FlexGridSizer17->Add(Notebook1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer16->Add(FlexGridSizer17, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer4->Add(FlexGridSizer16, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer1->Add(FlexGridSizer4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    SetSizer(FlexGridSizer1);
    MenuBar1 = new wxMenuBar();
    Menu1 = new wxMenu();
    MenuItem5 = new wxMenuItem(Menu1, ID_MENUITEM3, _("New"), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(MenuItem5);
    Menu1->AppendSeparator();
    MenuItem3 = new wxMenuItem(Menu1, ID_MENUITEM1, _("Load (binary format)..."), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(MenuItem3);
    MenuItem4 = new wxMenuItem(Menu1, ID_MENUITEM2, _("Save (binary format)..."), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(MenuItem4);
    Menu1->AppendSeparator();
    MenuItem1 = new wxMenuItem(Menu1, idMenuQuit, _("Quit\tAlt-F4"), _("Quit the application"), wxITEM_NORMAL);
    Menu1->Append(MenuItem1);
    MenuBar1->Append(Menu1, _("&File"));
    Menu2 = new wxMenu();
    MenuItem2 = new wxMenuItem(Menu2, idMenuAbout, _("About\tF1"), _("Show info about this application"), wxITEM_NORMAL);
    Menu2->Append(MenuItem2);
    MenuBar1->Append(Menu2, _("Help"));
    SetMenuBar(MenuBar1);
    FlexGridSizer1->Fit(this);
    FlexGridSizer1->SetSizeHints(this);
    Center();

    Connect(ID_RADIOBOX2,wxEVT_COMMAND_RADIOBOX_SELECTED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::On1stXYZSelect);
    Connect(ID_SIMPLEHTMLLISTBOX1,wxEVT_COMMAND_LISTBOX_SELECTED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnlistLinksSelect);
    Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnbtnAddLinkClick);
    Connect(ID_BUTTON6,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnbtnClearClick);
    Connect(ID_BUTTON7,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnbtnDeleteClick);
    Connect(ID_RADIOBOX1,wxEVT_COMMAND_RADIOBOX_SELECTED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnrbTypeSelect);
    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnButtonSaveFromEdit);
    Connect(ID_SLIDER1,wxEVT_SCROLL_TOP|wxEVT_SCROLL_BOTTOM|wxEVT_SCROLL_LINEUP|wxEVT_SCROLL_LINEDOWN|wxEVT_SCROLL_PAGEUP|wxEVT_SCROLL_PAGEDOWN|wxEVT_SCROLL_THUMBTRACK|wxEVT_SCROLL_THUMBRELEASE|wxEVT_SCROLL_CHANGED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnSliderScroll);
    Connect(ID_SLIDER1,wxEVT_SCROLL_CHANGED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnSliderScroll);
    Connect(ID_SLIDER1,wxEVT_COMMAND_SLIDER_UPDATED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnSliderScroll);
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnButtonSaveFromEdit);
    Connect(ID_SLIDER2,wxEVT_SCROLL_TOP|wxEVT_SCROLL_BOTTOM|wxEVT_SCROLL_LINEUP|wxEVT_SCROLL_LINEDOWN|wxEVT_SCROLL_PAGEUP|wxEVT_SCROLL_PAGEDOWN|wxEVT_SCROLL_THUMBTRACK|wxEVT_SCROLL_THUMBRELEASE|wxEVT_SCROLL_CHANGED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnSliderScroll);
    Connect(ID_SLIDER2,wxEVT_SCROLL_CHANGED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnSliderScroll);
    Connect(ID_SLIDER2,wxEVT_COMMAND_SLIDER_UPDATED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnSliderScroll);
    Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnButtonSaveFromEdit);
    Connect(ID_SLIDER3,wxEVT_SCROLL_TOP|wxEVT_SCROLL_BOTTOM|wxEVT_SCROLL_LINEUP|wxEVT_SCROLL_LINEDOWN|wxEVT_SCROLL_PAGEUP|wxEVT_SCROLL_PAGEDOWN|wxEVT_SCROLL_THUMBTRACK|wxEVT_SCROLL_THUMBRELEASE|wxEVT_SCROLL_CHANGED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnSliderScroll);
    Connect(ID_SLIDER3,wxEVT_SCROLL_CHANGED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnSliderScroll);
    Connect(ID_SLIDER3,wxEVT_COMMAND_SLIDER_UPDATED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnSliderScroll);
    Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnButtonSaveFromEdit);
    Connect(ID_SLIDER4,wxEVT_SCROLL_TOP|wxEVT_SCROLL_BOTTOM|wxEVT_SCROLL_LINEUP|wxEVT_SCROLL_LINEDOWN|wxEVT_SCROLL_PAGEUP|wxEVT_SCROLL_PAGEDOWN|wxEVT_SCROLL_THUMBTRACK|wxEVT_SCROLL_THUMBRELEASE|wxEVT_SCROLL_CHANGED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnSliderScroll);
    Connect(ID_SLIDER4,wxEVT_SCROLL_CHANGED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnSliderScroll);
    Connect(ID_SLIDER4,wxEVT_COMMAND_SLIDER_UPDATED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnSliderScroll);
    Connect(ID_LISTBOX1,wxEVT_COMMAND_LISTBOX_SELECTED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnlbXYZsSelect);
    Connect(ID_MENUITEM3,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnbtnClearClick);
    Connect(ID_MENUITEM1,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnLoadBinary);
    Connect(ID_MENUITEM2,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnSaveBinary);
    Connect(idMenuQuit,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnQuit);
    Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnAbout);
    //*)

	// maximize:
	this->Maximize();

	// Load default robot:
	m_robot.addLink(0,0.4,0,DEG2RAD(90), false);
	m_robot.addLink(0,0.3,0,DEG2RAD(90), false);
	m_robot.addLink(0,0.2,0,0, false);


	// Initialize 3D scene:
	{
		mrpt::opengl::CGridPlaneXYPtr grid_10cm = mrpt::opengl::CGridPlaneXY::Create(-5,5, -5, 5, 0, 0.1f);
		mrpt::opengl::CGridPlaneXYPtr grid_1m = mrpt::opengl::CGridPlaneXY::Create(-5,5, -5, 5, 0.001f, 1);

		grid_10cm->setColor_u8( mrpt::utils::TColor(0xC0,0xC0,0xC0,0xA0) );
		grid_1m->setColor_u8( mrpt::utils::TColor(0xFF,0xFF,0xFF) );

		m_plot3D->m_openGLScene->insert(grid_10cm);
		m_plot3D->m_openGLScene->insert(grid_1m);
	}

	m_gl_robot = mrpt::opengl::CSetOfObjects::Create();
	m_plot3D->m_openGLScene->insert(m_gl_robot);

	this->Regenerate3DView();

	{
		mrpt::opengl::COpenGLViewportPtr gl_view = m_plot3D->m_openGLScene->createViewport("small-view");

		gl_view->setViewportPosition(0,0, 0.2,0.3);
		gl_view->setTransparent(true);
		{
			mrpt::opengl::CTextPtr obj=mrpt::opengl::CText::Create("X");
			obj->setLocation(1.1,0,0);
			gl_view->insert(obj);
		}
		{
			mrpt::opengl::CTextPtr obj=mrpt::opengl::CText::Create("Y");
			obj->setLocation(0,1.1,0);
			gl_view->insert(obj);
		}
		{
			mrpt::opengl::CTextPtr obj=mrpt::opengl::CText::Create("Z");
			obj->setLocation(0,0,1.1);
			gl_view->insert(obj);
		}
		gl_view->insert( mrpt::opengl::stock_objects::CornerXYZ() );
	}

	m_plot3D->cameraZoomDistance = 3;

	m_plot3D->Refresh();



	// Update controls:
	this->RegenerateDOFPanels();
	this->UpdateListLinks();

	this->OnListSelectionChange();
}

robotic_arm_kinematicsFrame::~robotic_arm_kinematicsFrame()
{
    //(*Destroy(robotic_arm_kinematicsFrame)
    //*)
}


void robotic_arm_kinematicsFrame::Regenerate3DView()
{
	m_gl_robot->clear();
	m_robot.getAs3DObject(m_gl_robot, &m_all_poses);
}

void robotic_arm_kinematicsFrame::OnQuit(wxCommandEvent& event)
{
    Close();
}

void robotic_arm_kinematicsFrame::OnAbout(wxCommandEvent& event)
{
	CAboutBox box(this);
	box.ShowModal();
}


/** Regenerate the left list from m_robot */
void robotic_arm_kinematicsFrame::UpdateListLinks()
{
	// Show orientation of 1st DOF:
	{
		const CPose3D &pose0 = m_robot.getOriginPose();
		const CMatrixDouble33 & R= pose0.getRotationMatrix();

		if (abs(R(0,2))>abs(R(1,2)) && abs(R(0,2))>abs(R(2,2)))
			RadioBox1->SetSelection(2); // X
		else if (abs(R(1,2))>abs(R(0,2)) && abs(R(1,2))>abs(R(2,2)))
			RadioBox1->SetSelection(1); // Y
		else if (abs(R(2,2))>abs(R(0,2)) && abs(R(2,2))>abs(R(1,2)))
			RadioBox1->SetSelection(0); // Z
	}

	listLinks->Freeze();

	const int oldSelect = listLinks->GetSelection();
	const int oldSelectXYZ = lbXYZs->GetSelection();
	listLinks->Clear();

	lbXYZs->Freeze();
	lbXYZs->Clear();

	for (unsigned int i=0;i<m_robot.size();i++)
	{
		const TKinematicLink & l = m_robot.getLink(i);

		std::stringstream ss;
		ss << "<b>"<< (i+1) << "</b>: ";
		ss << "Type: "; if (l.is_prismatic) ss << "<font color='blue'>P</font>"; else ss << "<font color='red'>R</font>";
		ss << mrpt::format(" &theta;=%.02f&deg; d=%.02fmm a=%.02fmm &alpha;=%.02f&deg;", RAD2DEG(l.theta), l.d*1e3, l.a*1e3,RAD2DEG(l.alpha));

		listLinks->Append(_U(ss.str().c_str()));
	}

	for (unsigned int i=0;i<=m_robot.size();i++)
	{
		lbXYZs->Append( _U( mrpt::format("XYZ%u",static_cast<unsigned int>(i)).c_str() ) );
	}

	if (oldSelect>=0) listLinks->SetSelection(oldSelect);
	if (oldSelectXYZ>=0) lbXYZs->SetSelection(oldSelectXYZ);

	listLinks->Thaw();
	lbXYZs->Thaw();

	// Update DOF panels:
	if (m_dof_panels.size()==m_robot.size())
	{
		pnDOFs->Freeze();

		for (size_t i=0;i<m_dof_panels.size();i++)
		{
			const TKinematicLink & l = m_robot.getLink(i);
			m_dof_panels[i]->Label1->Clear();

			{
				std::stringstream ss;
				if (l.is_prismatic)
					 ss << "d<sub>" << (i+1) << "</sub>";
				else ss << "&theta;<sub>" << (i+1) << "</sub>";

				m_dof_panels[i]->Label1->Append(_U(ss.str().c_str()));
			}

			if (l.is_prismatic)
			{
				m_dof_panels[i]->Slider1->SetMin(-2000);
				m_dof_panels[i]->Slider1->SetMax(2000);

				m_dof_panels[i]->Slider1->SetValue( l.d*1e3 );

				std::string s = mrpt::format("%.02f",l.d*1e3);
				m_dof_panels[i]->TextCtrl1->SetValue(_U(s.c_str()));
			}
			else
			{
				m_dof_panels[i]->Slider1->SetMin(-180);
				m_dof_panels[i]->Slider1->SetMax(180);
				m_dof_panels[i]->Slider1->SetValue( RAD2DEG(l.theta) );

				std::string s = mrpt::format("%.02f",RAD2DEG(l.theta));
				m_dof_panels[i]->TextCtrl1->SetValue(_U(s.c_str()));
			}

		}

		pnDOFs->Thaw();
	}
}

/** Regenerate the bottom controls from m_robot */
void robotic_arm_kinematicsFrame::RegenerateDOFPanels()
{
	// Clear:
	for (size_t i=0;i<m_dof_panels.size();i++)
		delete m_dof_panels[i];
	m_dof_panels.clear();

	// Create:
	const size_t N = m_robot.size();
	m_dof_panels.resize(N);
	for (unsigned int i=0;i<N;i++)
	{
		m_dof_panels[i] = new PanelDOF(pnDOFs);
		boxSizerDOFs->Add( m_dof_panels[i] );
	}

	boxSizerDOFs->Fit(pnDOFs);
    boxSizerDOFs->SetSizeHints(pnDOFs);
}

/** Just update the DOF panel status from m_robot */
void robotic_arm_kinematicsFrame::UpdateDOFPanels()
{

}

void robotic_arm_kinematicsFrame::OnListSelectionChange()
{
	UpdateMatrixView();

	const int sel = listLinks->GetSelection();
	if (sel<0 || size_t(sel)>=m_robot.size())
	{
		panelProperties->Enable(false);
		//panelProperties->Show(false);
		return;
	}

	panelProperties->Enable(true);
	panelProperties->Show(true);

	const TKinematicLink & l = m_robot.getLink(sel);
	edTheta->SetValue( wxString::Format(_("%.04f"), RAD2DEG(l.theta)) );
	edD->SetValue( wxString::Format(_("%.04f"), 1e3*l.d ) );
	edA->SetValue( wxString::Format(_("%.04f"), 1e3*l.a ) );
	edAlpha->SetValue( wxString::Format(_("%.04f"), RAD2DEG(l.alpha)) );

	rbType->SetSelection( l.is_prismatic ? 1:0);


	slTheta->SetValue(RAD2DEG(l.theta));
	slAlpha->SetValue(RAD2DEG(l.alpha));
	slD->SetValue(1e3*l.d);
	slA->SetValue(1e3*l.a);
}

void robotic_arm_kinematicsFrame::OnlistLinksSelect(wxCommandEvent& event)
{
	OnListSelectionChange();
}

// Sliders of the DOFs bottom panels:
void robotic_arm_kinematicsFrame::OnSliderDOFScroll(wxScrollEvent& event)
{
	for (size_t i=0;i<m_dof_panels.size();i++)
	{
		TKinematicLink & l = m_robot.getLinkRef(i);
		if (l.is_prismatic)
		     l.d = m_dof_panels[i]->Slider1->GetValue()*1e-3;
		else l.theta = DEG2RAD(m_dof_panels[i]->Slider1->GetValue());
	}

	UpdateListLinks();

	m_robot.update3DObject(&m_all_poses);
	m_plot3D->Refresh();

	UpdateMatrixView();
}

// Sliders of the left panel:
void robotic_arm_kinematicsFrame::OnSliderScroll(wxScrollEvent& event)
{
	UpdateMatrixView();

	const int sel = listLinks->GetSelection();
	if (sel<0 || size_t(sel)>=m_robot.size())
	{
		panelProperties->Enable(false);
		//panelProperties->Show(false);
		return;
	}

	TKinematicLink & l = m_robot.getLinkRef(sel);

	const double th = DEG2RAD( slTheta->GetValue() * 1.0 );
	const double alpha = DEG2RAD( slAlpha->GetValue() * 1.0 );
	const double d = slD->GetValue() * 1e-3;
	const double a = slA->GetValue() * 1e-3;

	l.theta = th;
	l.a = a;
	l.alpha = alpha;
	l.d = d;

	edTheta->SetValue( wxString::Format(_("%.04f"), RAD2DEG(l.theta)) );
	edD->SetValue( wxString::Format(_("%.04f"), 1e3*l.d ) );
	edA->SetValue( wxString::Format(_("%.04f"), 1e3*l.a ) );
	edAlpha->SetValue( wxString::Format(_("%.04f"), RAD2DEG(l.alpha)) );

	UpdateListLinks();

	m_robot.update3DObject(&m_all_poses);
	m_plot3D->Refresh();
}

void robotic_arm_kinematicsFrame::OnButtonSaveFromEdit(wxCommandEvent& event)
{
	const int sel = listLinks->GetSelection();
	if (sel>=0 && size_t(sel)<m_robot.size())
	{
		// Get ref to selected link and apply:
		TKinematicLink & l = m_robot.getLinkRef(sel);

		double d;
		edTheta->GetValue().ToDouble(&d);
		l.theta = DEG2RAD(d);

		edD->GetValue().ToDouble(&d);
		l.d = 1e-3*d;

		edA->GetValue().ToDouble(&d);
		l.a = 1e-3*d;

		edAlpha->GetValue().ToDouble(&d);
		l.alpha = DEG2RAD(d);

		l.is_prismatic = (rbType->GetSelection()==1);

		OnListSelectionChange();
		UpdateListLinks();
		UpdateMatrixView();
		m_robot.update3DObject(&m_all_poses);
		m_plot3D->Refresh();
	}
	else
	{
		panelProperties->Enable(false);
		return;
	}


}

void robotic_arm_kinematicsFrame::OnbtnClearClick(wxCommandEvent& event)
{
	m_robot.clear();

	this->RegenerateDOFPanels();
	this->UpdateListLinks();

	this->OnListSelectionChange();

	this->Regenerate3DView();
	m_plot3D->Refresh();
}

void robotic_arm_kinematicsFrame::OnbtnAddLinkClick(wxCommandEvent& event)
{
	m_robot.addLink(0,0,0,0,false);

	this->RegenerateDOFPanels();
	this->UpdateListLinks();

	listLinks->SetSelection( m_robot.size()-1 );

	this->OnListSelectionChange();
	this->Regenerate3DView();
	m_plot3D->Refresh();
}

void robotic_arm_kinematicsFrame::OnLoadBinary(wxCommandEvent& event)
{
	WX_START_TRY

	wxFileDialog dlg(
		this,
		_("Select kinematic chain to load"),
		_("."),
		_("*.kinbin"),
		wxT("Kinematic chains binary files (*.kinbin)|*.kinbin|All files (*.*)|*.*"),
		wxFD_OPEN | wxFD_FILE_MUST_EXIST );

	if (dlg.ShowModal() != wxID_OK)
		return;

	const wxString sFil =  dlg.GetPath();
	const std::string fil = std::string(sFil.mb_str());

	mrpt::utils::CFileGZInputStream f(fil);
	f >> m_robot;

	this->RegenerateDOFPanels();
	this->UpdateListLinks();

	this->OnListSelectionChange();

	this->Regenerate3DView();
	m_plot3D->Refresh();

	WX_END_TRY
}

void robotic_arm_kinematicsFrame::OnSaveBinary(wxCommandEvent& event)
{
	WX_START_TRY

	wxFileDialog dlg(
		this,
		_("Save kinematic chain"),
		_("."),
		_("*.kinbin"),
		wxT("Kinematic chains binary files (*.kinbin)|*.kinbin|All files (*.*)|*.*"),
		wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

	if (dlg.ShowModal() != wxID_OK)
		return;

	const wxString sFil =  dlg.GetPath();
	const std::string fil = std::string(sFil.mb_str());

	mrpt::utils::CFileOutputStream f(fil);
	f << m_robot;

	WX_END_TRY
}

void robotic_arm_kinematicsFrame::OnrbTypeSelect(wxCommandEvent& event)
{
	OnButtonSaveFromEdit(event);
}

void robotic_arm_kinematicsFrame::OnbtnDeleteClick(wxCommandEvent& event)
{
	const int sel = listLinks->GetSelection();
	if (sel<0 || size_t(sel)>=m_robot.size())
	{
		panelProperties->Enable(false);
		return;
	}

	m_robot.removeLink(sel);

	this->RegenerateDOFPanels();
	this->UpdateListLinks();

	listLinks->SetSelection( m_robot.size()-1 );

	this->OnListSelectionChange();
	this->Regenerate3DView();
	m_plot3D->Refresh();
}

void robotic_arm_kinematicsFrame::OnlbXYZsSelect(wxCommandEvent& event)
{
	UpdateMatrixView();
}

void robotic_arm_kinematicsFrame::UpdateMatrixView()
{
	const int sel = lbXYZs->GetSelection();
	if (sel<0 || size_t(sel)>m_all_poses.size())
	{
		return;
	}

	edMatrix->Freeze();

	const mrpt::math::CMatrixDouble44 M = m_all_poses[sel].getHomogeneousMatrixVal();

	const std::string s = format(
		"%6.03f %6.03f %6.03f %6.03f\n"
		"%6.03f %6.03f %6.03f %6.03f\n"
		"%6.03f %6.03f %6.03f %6.03f\n"
		"%6.03f %6.03f %6.03f %6.03f\n",
		M(0,0),M(0,1),M(0,2),M(0,3),
		M(1,0),M(1,1),M(1,2),M(1,3),
		M(2,0),M(2,1),M(2,2),M(2,3),
		M(3,0),M(3,1),M(3,2),M(3,3) );
	edMatrix->SetValue(_U( s.c_str() ));

	edMatrix->Thaw();
}

void robotic_arm_kinematicsFrame::On1stXYZSelect(wxCommandEvent& event)
{
	// Save orientation of 1st DOF:
	CPose3D pose0;

	switch( RadioBox1->GetSelection() )
	{
		// Z:
		default:
		case 0: pose0 = CPose3D(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0) ); break;
		// Y:
		case 1: pose0 = CPose3D(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(-90) ); break;
		// X:
		case 2: pose0 = CPose3D(0,0,0, DEG2RAD(0),DEG2RAD(90),DEG2RAD(0) ); break;
	};

	m_robot.setOriginPose(pose0);
	UpdateListLinks();
	UpdateMatrixView();
	m_robot.update3DObject(&m_all_poses);
	m_plot3D->Refresh();
}
