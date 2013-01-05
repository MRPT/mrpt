/***************************************************************
 * Name:      robotic_arm_kinematicsMain.cpp
 * Purpose:   Code for Application Frame
 * Author:     ()
 * Created:   2013-01-05
 * Copyright:  ()
 * License:
 **************************************************************/

#include "robotic_arm_kinematicsMain.h"
#include <wx/msgdlg.h>

//(*InternalHeaders(robotic_arm_kinematicsFrame)
#include <wx/settings.h>
#include <wx/intl.h>
#include <wx/string.h>
//*)

//helper functions
enum wxbuildinfoformat {
    short_f, long_f };

wxString wxbuildinfo(wxbuildinfoformat format)
{
    wxString wxbuild(wxVERSION_STRING);

    if (format == long_f )
    {
#if defined(__WXMSW__)
        wxbuild << _T("-Windows");
#elif defined(__UNIX__)
        wxbuild << _T("-Linux");
#endif

#if wxUSE_UNICODE
        wxbuild << _T("-Unicode build");
#else
        wxbuild << _T("-ANSI build");
#endif // wxUSE_UNICODE
    }

    return wxbuild;
}

//(*IdInit(robotic_arm_kinematicsFrame)
const long robotic_arm_kinematicsFrame::ID_STATICTEXT1 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_SIMPLEHTMLLISTBOX1 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_RADIOBOX1 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_SIMPLEHTMLLISTBOX2 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_STATICTEXT2 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_TEXTCTRL1 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_STATICTEXT3 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_SLIDER1 = wxNewId();
const long robotic_arm_kinematicsFrame::ID_XY_GLCANVAS = wxNewId();
const long robotic_arm_kinematicsFrame::idMenuQuit = wxNewId();
const long robotic_arm_kinematicsFrame::idMenuAbout = wxNewId();
const long robotic_arm_kinematicsFrame::ID_STATUSBAR1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(robotic_arm_kinematicsFrame,wxFrame)
    //(*EventTable(robotic_arm_kinematicsFrame)
    //*)
END_EVENT_TABLE()

robotic_arm_kinematicsFrame::robotic_arm_kinematicsFrame(wxWindow* parent,wxWindowID id)
{
    //(*Initialize(robotic_arm_kinematicsFrame)
    wxFlexGridSizer* FlexGridSizer4;
    wxMenuItem* MenuItem2;
    wxFlexGridSizer* FlexGridSizer3;
    wxMenuItem* MenuItem1;
    wxFlexGridSizer* FlexGridSizer5;
    wxFlexGridSizer* FlexGridSizer2;
    wxMenu* Menu1;
    wxMenuBar* MenuBar1;
    wxFlexGridSizer* FlexGridSizer6;
    wxStaticBoxSizer* StaticBoxSizer1;
    wxFlexGridSizer* FlexGridSizer1;
    wxMenu* Menu2;

    Create(parent, id, _("Robotic Arm Kinematic GUI - Part of MRPT"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("id"));
    SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BTNFACE));
    FlexGridSizer1 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer1->AddGrowableCol(1);
    FlexGridSizer1->AddGrowableRow(0);
    FlexGridSizer2 = new wxFlexGridSizer(3, 1, 0, 0);
    FlexGridSizer2->AddGrowableCol(0);
    FlexGridSizer2->AddGrowableRow(1);
    FlexGridSizer2->AddGrowableRow(2);
    StaticText1 = new wxStaticText(this, ID_STATICTEXT1, _("List of kinematic links:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    FlexGridSizer2->Add(StaticText1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    SimpleHtmlListBox1 = new wxSimpleHtmlListBox(this, ID_SIMPLEHTMLLISTBOX1, wxDefaultPosition, wxDefaultSize, 0, 0, wxHLB_DEFAULT_STYLE, wxDefaultValidator, _T("ID_SIMPLEHTMLLISTBOX1"));
    SimpleHtmlListBox1->Append(_("aasas"));
    SimpleHtmlListBox1->Append(_("a<font color=\"red\">s</font>asas"));
    FlexGridSizer2->Add(SimpleHtmlListBox1, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 2);
    StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, this, _(" Link properties: "));
    FlexGridSizer3 = new wxFlexGridSizer(3, 1, 0, 0);
    wxString __wxRadioBoxChoices_1[2] =
    {
    	_("Revolute"),
    	_("Prismatic")
    };
    RadioBox1 = new wxRadioBox(this, ID_RADIOBOX1, _("Type"), wxDefaultPosition, wxDefaultSize, 2, __wxRadioBoxChoices_1, 2, 0, wxDefaultValidator, _T("ID_RADIOBOX1"));
    FlexGridSizer3->Add(RadioBox1, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 2);
    SimpleHtmlListBox2 = new wxSimpleHtmlListBox(this, ID_SIMPLEHTMLLISTBOX2, wxDefaultPosition, wxSize(-1,30), 0, 0, wxNO_BORDER, wxDefaultValidator, _T("ID_SIMPLEHTMLLISTBOX2"));
    SimpleHtmlListBox2->Append(_("&theta;<sub>i</sub> (Angle between X<sub>i</sub> and  X<sub>i+1</sub>)"));
    SimpleHtmlListBox2->Disable();
    SimpleHtmlListBox2->SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_BTNFACE));
    FlexGridSizer3->Add(SimpleHtmlListBox2, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer5 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer5->AddGrowableCol(0);
    FlexGridSizer6 = new wxFlexGridSizer(0, 3, 0, 0);
    FlexGridSizer6->AddGrowableCol(1);
    StaticText2 = new wxStaticText(this, ID_STATICTEXT2, _("Value="), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    FlexGridSizer6->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edTheta = new wxTextCtrl(this, ID_TEXTCTRL1, _("0"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    FlexGridSizer6->Add(edTheta, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText3 = new wxStaticText(this, ID_STATICTEXT3, _("(deg)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
    FlexGridSizer6->Add(StaticText3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer5->Add(FlexGridSizer6, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    slTheta = new wxSlider(this, ID_SLIDER1, 0, 0, 100, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_SLIDER1"));
    FlexGridSizer5->Add(slTheta, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 2);
    FlexGridSizer3->Add(FlexGridSizer5, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    StaticBoxSizer1->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer2->Add(StaticBoxSizer1, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 2);
    FlexGridSizer1->Add(FlexGridSizer2, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer4 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer4->AddGrowableCol(0);
    FlexGridSizer4->AddGrowableRow(0);
    m_plot3D = new CMyGLCanvas(this,ID_XY_GLCANVAS,wxDefaultPosition,wxSize(450,350),wxTAB_TRAVERSAL,_T("ID_XY_GLCANVAS"));
    FlexGridSizer4->Add(m_plot3D, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer1->Add(FlexGridSizer4, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    SetSizer(FlexGridSizer1);
    MenuBar1 = new wxMenuBar();
    Menu1 = new wxMenu();
    MenuItem1 = new wxMenuItem(Menu1, idMenuQuit, _("Quit\tAlt-F4"), _("Quit the application"), wxITEM_NORMAL);
    Menu1->Append(MenuItem1);
    MenuBar1->Append(Menu1, _("&File"));
    Menu2 = new wxMenu();
    MenuItem2 = new wxMenuItem(Menu2, idMenuAbout, _("About\tF1"), _("Show info about this application"), wxITEM_NORMAL);
    Menu2->Append(MenuItem2);
    MenuBar1->Append(Menu2, _("Help"));
    SetMenuBar(MenuBar1);
    StatusBar1 = new wxStatusBar(this, ID_STATUSBAR1, 0, _T("ID_STATUSBAR1"));
    int __wxStatusBarWidths_1[1] = { -1 };
    int __wxStatusBarStyles_1[1] = { wxSB_NORMAL };
    StatusBar1->SetFieldsCount(1,__wxStatusBarWidths_1);
    StatusBar1->SetStatusStyles(1,__wxStatusBarStyles_1);
    SetStatusBar(StatusBar1);
    FlexGridSizer1->Fit(this);
    FlexGridSizer1->SetSizeHints(this);
    Center();

    Connect(idMenuQuit,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnQuit);
    Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&robotic_arm_kinematicsFrame::OnAbout);
    //*)

	// maximize:
	this->Maximize();

	// Initialize 3D scene:
	{
		mrpt::opengl::CGridPlaneXYPtr grid_10cm = mrpt::opengl::CGridPlaneXY::Create(-5,5, -5, 5, 0, 0.1);
		mrpt::opengl::CGridPlaneXYPtr grid_1m = mrpt::opengl::CGridPlaneXY::Create(-5,5, -5, 5, 0.001, 1);

		grid_10cm->setColor_u8( mrpt::utils::TColor(0xC0,0xC0,0xC0,0xA0) );
		grid_1m->setColor_u8( mrpt::utils::TColor(0xFF,0xFF,0xFF) );

		m_plot3D->m_openGLScene->insert(grid_10cm);
		m_plot3D->m_openGLScene->insert(grid_1m);
	}

	{
		mrpt::opengl::COpenGLViewportPtr gl_view = m_plot3D->m_openGLScene->createViewport("small-view");

		gl_view->setViewportPosition(0,0, 0.2,0.25);
		gl_view->setTransparent(true);
		gl_view->insert( mrpt::opengl::stock_objects::CornerXYZ() );
	}

	
	m_plot3D->Refresh();


}

robotic_arm_kinematicsFrame::~robotic_arm_kinematicsFrame()
{
    //(*Destroy(robotic_arm_kinematicsFrame)
    //*)
}

void robotic_arm_kinematicsFrame::OnQuit(wxCommandEvent& event)
{
    Close();
}

void robotic_arm_kinematicsFrame::OnAbout(wxCommandEvent& event)
{
    wxString msg = wxbuildinfo(long_f);
    wxMessageBox(msg, _("Welcome to..."));
}
