/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
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

#include "holonomic_navigator_demoMain.h"
#include <wx/msgdlg.h>
#include "CAboutBox.h"

//(*InternalHeaders(holonomic_navigator_demoFrame)
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/tglbtn.h>
#include <wx/font.h>
#include <wx/intl.h>
#include <wx/image.h>
#include <wx/string.h>
//*)

#include <mrpt/gui/WxUtils.h>
#include "imgs/main_icon.xpm"
#include "../wx-common/mrpt_logo.xpm"

#include "../ReactiveNavigationDemo/DEFAULT_GRIDMAP_DATA.h"

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

#include <mrpt/reactivenav.h>
#include <mrpt/gui.h>
#include <mrpt/opengl.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace std;


//(*IdInit(holonomic_navigator_demoFrame)
const long holonomic_navigator_demoFrame::ID_BUTTON1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_BUTTON2 = wxNewId();
const long holonomic_navigator_demoFrame::ID_BUTTON3 = wxNewId();
const long holonomic_navigator_demoFrame::ID_BUTTON6 = wxNewId();
const long holonomic_navigator_demoFrame::ID_BUTTON7 = wxNewId();
const long holonomic_navigator_demoFrame::ID_BUTTON4 = wxNewId();
const long holonomic_navigator_demoFrame::ID_BUTTON5 = wxNewId();
const long holonomic_navigator_demoFrame::ID_RADIOBOX1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_TEXTCTRL1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_PANEL1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_PANEL2 = wxNewId();
const long holonomic_navigator_demoFrame::ID_NOTEBOOK1 = wxNewId();
const long holonomic_navigator_demoFrame::ID_XY_GLCANVAS = wxNewId();
const long holonomic_navigator_demoFrame::ID_CUSTOM1 = wxNewId();
const long holonomic_navigator_demoFrame::idMenuQuit = wxNewId();
const long holonomic_navigator_demoFrame::idMenuAbout = wxNewId();
const long holonomic_navigator_demoFrame::ID_STATUSBAR1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(holonomic_navigator_demoFrame,wxFrame)
    //(*EventTable(holonomic_navigator_demoFrame)
    //*)
END_EVENT_TABLE()

holonomic_navigator_demoFrame::holonomic_navigator_demoFrame(wxWindow* parent,wxWindowID id)
{
    // Load my custom icons:
#if wxCHECK_VERSION(2, 8, 0)
    wxArtProvider::Push(new MyArtProvider);
#else
    wxArtProvider::PushProvider(new MyArtProvider);
#endif


    //(*Initialize(holonomic_navigator_demoFrame)
    wxFlexGridSizer* FlexGridSizer4;
    wxMenuItem* MenuItem2;
    wxFlexGridSizer* FlexGridSizer3;
    wxMenuItem* MenuItem1;
    wxFlexGridSizer* FlexGridSizer2;
    wxBoxSizer* BoxSizer2;
    wxMenu* Menu1;
    wxFlexGridSizer* FlexGridSizer7;
    wxBoxSizer* BoxSizer1;
    wxMenuBar* MenuBar1;
    wxFlexGridSizer* FlexGridSizer1;
    wxMenu* Menu2;
    
    Create(parent, id, _("Holonomic Navigation Demo - Part of MRPT"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("id"));
    FlexGridSizer1 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer1->AddGrowableCol(0);
    FlexGridSizer1->AddGrowableRow(1);
    FlexGridSizer2 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer2->AddGrowableCol(1);
    FlexGridSizer4 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer4->AddGrowableCol(0);
    BoxSizer1 = new wxBoxSizer(wxHORIZONTAL);
    btnLoadMap = new wxCustomButton(this,ID_BUTTON1,_("Load map..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FOLDER")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON1"));
    btnLoadMap->SetBitmapDisabled(btnLoadMap->CreateBitmapDisabled(btnLoadMap->GetBitmapLabel()));
    btnLoadMap->SetBitmapMargin(wxSize(2,4));
    BoxSizer1->Add(btnLoadMap, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
    btnHelp = new wxCustomButton(this,ID_BUTTON2,_("About..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_QUESTION")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON2"));
    btnHelp->SetBitmapDisabled(btnHelp->CreateBitmapDisabled(btnHelp->GetBitmapLabel()));
    btnHelp->SetBitmapMargin(wxSize(5,4));
    BoxSizer1->Add(btnHelp, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
    btnQuit = new wxCustomButton(this,ID_BUTTON3,_("Exit"),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_QUIT")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON3"));
    btnQuit->SetBitmapDisabled(btnQuit->CreateBitmapDisabled(btnQuit->GetBitmapLabel()));
    BoxSizer1->Add(btnQuit, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
    FlexGridSizer4->Add(BoxSizer1, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    BoxSizer2 = new wxBoxSizer(wxHORIZONTAL);
    Button1 = new wxCustomButton(this,ID_BUTTON6,_("Place robot..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FIND")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON6"));
    Button1->SetBitmapDisabled(Button1->CreateBitmapDisabled(Button1->GetBitmapLabel()));
    Button1->SetBitmapMargin(wxSize(2,4));
    BoxSizer2->Add(Button1, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 2);
    Button2 = new wxCustomButton(this,ID_BUTTON7,_("Set target..."),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FIND")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON7"));
    Button2->SetBitmapDisabled(Button2->CreateBitmapDisabled(Button2->GetBitmapLabel()));
    Button2->SetBitmapMargin(wxSize(2,4));
    BoxSizer2->Add(Button2, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 2);
    btnStart = new wxCustomButton(this,ID_BUTTON4,_("START"),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_GO_FORWARD")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON4"));
    btnStart->SetBitmapDisabled(btnStart->CreateBitmapDisabled(btnStart->GetBitmapLabel()));
    btnStart->SetBitmapMargin(wxSize(2,4));
    BoxSizer2->Add(btnStart, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 2);
    btnStop = new wxCustomButton(this,ID_BUTTON5,_("STOP"),wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_CROSS_MARK")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(wxEmptyString))),wxDefaultPosition,wxDefaultSize,wxCUSTBUT_BUTTON|wxCUSTBUT_BOTTOM,wxDefaultValidator,_T("ID_BUTTON5"));
    btnStop->SetBitmapDisabled(btnStop->CreateBitmapDisabled(btnStop->GetBitmapLabel()));
    btnStop->SetBitmapMargin(wxSize(2,4));
    BoxSizer2->Add(btnStop, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 2);
    FlexGridSizer4->Add(BoxSizer2, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer2->Add(FlexGridSizer4, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    Notebook1 = new wxNotebook(this, ID_NOTEBOOK1, wxDefaultPosition, wxDefaultSize, 0, _T("ID_NOTEBOOK1"));
    Panel1 = new wxPanel(Notebook1, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    FlexGridSizer7 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer7->AddGrowableCol(1);
    FlexGridSizer7->AddGrowableRow(0);
    wxString __wxRadioBoxChoices_1[2] = 
    {
    	_("VFF (Virtual Force Field)"),
    	_("ND (Nearnest  Diagram)")
    };
    rbHoloMethod = new wxRadioBox(Panel1, ID_RADIOBOX1, _(" Holonomic method "), wxDefaultPosition, wxDefaultSize, 2, __wxRadioBoxChoices_1, 1, 0, wxDefaultValidator, _T("ID_RADIOBOX1"));
    FlexGridSizer7->Add(rbHoloMethod, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edHoloParams = new wxTextCtrl(Panel1, ID_TEXTCTRL1, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER|wxTE_PROCESS_TAB|wxTE_MULTILINE|wxHSCROLL|wxVSCROLL|wxALWAYS_SHOW_SB, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    wxFont edHoloParamsFont(8,wxTELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    edHoloParams->SetFont(edHoloParamsFont);
    FlexGridSizer7->Add(edHoloParams, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 2);
    Panel1->SetSizer(FlexGridSizer7);
    FlexGridSizer7->Fit(Panel1);
    FlexGridSizer7->SetSizeHints(Panel1);
    Panel2 = new wxPanel(Notebook1, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
    Notebook1->AddPage(Panel1, _("Configuration"), true);
    Notebook1->AddPage(Panel2, _("Stats"), false);
    FlexGridSizer2->Add(Notebook1, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 2);
    FlexGridSizer1->Add(FlexGridSizer2, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer3 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer3->AddGrowableCol(0);
    FlexGridSizer3->AddGrowableCol(1);
    FlexGridSizer3->AddGrowableRow(0);
    m_plot3D = new CMyGLCanvas(this,ID_XY_GLCANVAS,wxDefaultPosition,wxSize(450,350),wxTAB_TRAVERSAL,_T("ID_XY_GLCANVAS"));
    FlexGridSizer3->Add(m_plot3D, 1, wxALL|wxEXPAND|wxFIXED_MINSIZE|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
    m_plotScan = new CMyGLCanvas(this,ID_CUSTOM1,wxDefaultPosition,wxSize(150,150),wxTAB_TRAVERSAL,_T("ID_CUSTOM1"));
    FlexGridSizer3->Add(m_plotScan, 1, wxALL|wxEXPAND|wxFIXED_MINSIZE|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
    FlexGridSizer1->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
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
    
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnAbout);
    Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnQuit);
    Connect(ID_BUTTON6,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnButton1Click);
    Connect(ID_BUTTON7,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnButton1Click);
    Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnButton1Click);
    Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnButton2Click);
    Connect(idMenuQuit,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnQuit);
    Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&holonomic_navigator_demoFrame::OnAbout);
    //*)


	// Populate 3D views:
	{
		mrpt::opengl::CGridPlaneXYPtr obj = mrpt::opengl::CGridPlaneXY::Create(-50,50, -50,50, 0, 1);
		obj->setColor_u8(TColor(30,30,30,50));
		m_plot3D->m_openGLScene->insert( obj );
	}
	{
		mrpt::opengl::CGridPlaneXYPtr obj = mrpt::opengl::CGridPlaneXY::Create(-20,20, -20,20, 0, 1);
		obj->setColor_u8(TColor(30,30,30,50));
		m_plotScan->m_openGLScene->insert( obj );
	}

	m_plotScan->clearColorR = 0.9f;
	m_plotScan->clearColorG = 0.9f;
	m_plotScan->clearColorB = 0.9f;

	//m_plotScan->useCameraFromScene = true;


	// Retrieve default parameters for holonomic methods:
	{
		mrpt::utils::CConfigFileMemory cfg;

		mrpt::reactivenav::CHolonomicVFF holo_VFF;
		holo_VFF.options.saveToConfigFile("VFF_CONFIG",cfg);

		mrpt::reactivenav::CHolonomicND holo_ND;
		holo_ND.options.saveToConfigFile("ND_CONFIG",cfg);

		this->edHoloParams->SetValue( _U( cfg.getContent().c_str() ) );
	}


}

holonomic_navigator_demoFrame::~holonomic_navigator_demoFrame()
{
    //(*Destroy(holonomic_navigator_demoFrame)
    //*)
}

void holonomic_navigator_demoFrame::OnQuit(wxCommandEvent& event)
{
    Close();
}

void holonomic_navigator_demoFrame::OnAbout(wxCommandEvent& event)
{
	CAboutBox dlg(this);
	dlg.ShowModal();
}

void holonomic_navigator_demoFrame::OnButton1Click(wxCommandEvent& event)
{
}

void holonomic_navigator_demoFrame::OnButton2Click(wxCommandEvent& event)
{
}
