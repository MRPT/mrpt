/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hmtMapViewerMain.h"
#include <wx/msgdlg.h>

//(*InternalHeaders(hmtMapViewerFrame)
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/intl.h>
#include <wx/font.h>
#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/artprov.h>
//*)

#include <wx/msgdlg.h>
#include <wx/filedlg.h>
#include <wx/progdlg.h>
#include <wx/imaglist.h>
#include <wx/busyinfo.h>
#include <wx/log.h>
#include <wx/textdlg.h>
#include <wx/dirdlg.h>
#include <wx/colordlg.h>
#include <wx/dcmemory.h>

#include <mrpt/gui/CMyRedirector.h>


const char *iniFileSect = "configuration";  // For the .ini file

// The file to open (from cmd line), or an empty string
extern std::string     global_fileToOpen;



#if !wxUSE_GLCANVAS
#error "OpenGL required: set wxUSE_GLCANVAS to 1 and rebuild wxWidgets"
#endif

#include <mrpt/gui/CMyGLCanvasBase.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/stock_objects.h>

#include <mrpt/hmtslam/CHMTSLAM.h>
#include <mrpt/hmtslam/CRobotPosesGraph.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::hmtslam;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::maps;
using namespace std;

// The configuration file:
extern CConfigFile      *iniFile;


// HMT map to be displayed.
CHMTSLAM       *hmt_map = NULL;



class CMyGLCanvas : public CMyGLCanvasBase
{
public:
    CMyGLCanvas( wxWindow *parent, wxWindowID id = wxID_ANY,
                 const wxPoint& pos = wxDefaultPosition,
                 const wxSize& size = wxDefaultSize,
                 long style = 0, const wxString& name = _T("CMyGLCanvasBase") )
		: CMyGLCanvasBase(parent,id,pos,size,style,name)
	{
	}

	virtual ~CMyGLCanvas()
	{ }

	//void OnKeyDownCustom( wxKeyEvent& event );

	void OnPreRender();
	void OnPostRender();
	void OnPostRenderSwapBuffers(double At, wxPaintDC &dc);
	void OnRenderError( const wxString &str );

};

void CMyGLCanvas::OnRenderError( const wxString &str )
{
	//theWindow->StatusBar1->SetStatusText(str,1);
}

void CMyGLCanvas::OnPreRender()
{
	// Do we have to update the scene??
/*	synch::CCriticalSectionLocker   lock( &critSec_UpdateScene );
	if (newOpenGLScene)
	{
		if (m_openGLScene) delete m_openGLScene;
		m_openGLScene = newOpenGLScene;
		newOpenGLScene = NULL;
	}*/
}

void CMyGLCanvas::OnPostRenderSwapBuffers(double At, wxPaintDC &dc)
{
}


void CMyGLCanvas::OnPostRender()
{
	// Show filename over the screen??
//	if (showFileNameInViewport)
	{
//		renderTextBitmap( 20,20, extractFileName(loadedFileName).c_str() );
	}
}


// Auxiliary class for the tree data
class CItemData : public wxTreeItemData
{
public:
	CSerializablePtr m_ptr;
	size_t          m_itemIndex;

	CItemData( CSerializablePtr ptr, size_t itemIndex) : m_ptr(ptr), m_itemIndex(itemIndex)
	{
	}
};


//(*IdInit(hmtMapViewerFrame)
const long hmtMapViewerFrame::ID_STATICTEXT1 = wxNewId();
const long hmtMapViewerFrame::ID_STATICTEXT2 = wxNewId();
const long hmtMapViewerFrame::ID_CHOICE1 = wxNewId();
const long hmtMapViewerFrame::ID_PANEL5 = wxNewId();
const long hmtMapViewerFrame::ID_TREECTRL1 = wxNewId();
const long hmtMapViewerFrame::ID_PANEL9 = wxNewId();
const long hmtMapViewerFrame::ID_TEXTCTRL1 = wxNewId();
const long hmtMapViewerFrame::ID_PANEL8 = wxNewId();
const long hmtMapViewerFrame::ID_SPLITTERWINDOW3 = wxNewId();
const long hmtMapViewerFrame::ID_PANEL1 = wxNewId();
const long hmtMapViewerFrame::ID_STATICTEXT3 = wxNewId();
const long hmtMapViewerFrame::ID_PANEL6 = wxNewId();
const long hmtMapViewerFrame::ID_PANEL3 = wxNewId();
const long hmtMapViewerFrame::ID_STATICTEXT4 = wxNewId();
const long hmtMapViewerFrame::ID_PANEL7 = wxNewId();
const long hmtMapViewerFrame::ID_PANEL4 = wxNewId();
const long hmtMapViewerFrame::ID_SPLITTERWINDOW2 = wxNewId();
const long hmtMapViewerFrame::ID_PANEL2 = wxNewId();
const long hmtMapViewerFrame::ID_SPLITTERWINDOW1 = wxNewId();
const long hmtMapViewerFrame::ID_MENUITEM1 = wxNewId();
const long hmtMapViewerFrame::ID_MENUITEM4 = wxNewId();
const long hmtMapViewerFrame::idMenuQuit = wxNewId();
const long hmtMapViewerFrame::ID_MENUITEM2 = wxNewId();
const long hmtMapViewerFrame::ID_MENUITEM3 = wxNewId();
const long hmtMapViewerFrame::ID_MENUITEM6 = wxNewId();
const long hmtMapViewerFrame::ID_MENUITEM7 = wxNewId();
const long hmtMapViewerFrame::ID_MENUITEM5 = wxNewId();
const long hmtMapViewerFrame::idMenuAbout = wxNewId();
const long hmtMapViewerFrame::ID_STATUSBAR1 = wxNewId();
const long hmtMapViewerFrame::ID_TOOLBARITEM1 = wxNewId();
const long hmtMapViewerFrame::ID_TOOLBARITEM2 = wxNewId();
const long hmtMapViewerFrame::ID_TOOLBAR1 = wxNewId();
//*)

const long hmtMapViewerFrame::ID_TIMER1 = wxNewId();


BEGIN_EVENT_TABLE(hmtMapViewerFrame,wxFrame)
    //(*EventTable(hmtMapViewerFrame)
    //*)
END_EVENT_TABLE()

hmtMapViewerFrame::hmtMapViewerFrame(wxWindow* parent,wxWindowID id)
{
	hmt_map = new CHMTSLAM();

    //(*Initialize(hmtMapViewerFrame)
    wxMenuItem* MenuItem2;
    wxMenuItem* MenuItem1;
    wxFlexGridSizer* FlexGridSizer8;
    wxFlexGridSizer* FlexGridSizer1;
    wxFlexGridSizer* FlexGridSizer2;
    wxMenu* Menu1;
    wxFlexGridSizer* FlexGridSizer7;
    wxBoxSizer* BoxSizer2;
    wxFlexGridSizer* FlexGridSizer4;
    wxFlexGridSizer* FlexGridSizer9;
    wxFlexGridSizer* FlexGridSizer6;
    wxFlexGridSizer* FlexGridSizer3;
    wxBoxSizer* BoxSizer1;
    wxMenuBar* MenuBar1;
    wxMenu* Menu2;
    wxFlexGridSizer* FlexGridSizer5;

    Create(parent, wxID_ANY, _("HMT Maps Viewer - Part of the MRPT project"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("wxID_ANY"));
    SetClientSize(wxSize(740,606));
    SplitterWindow1 = new wxSplitterWindow(this, ID_SPLITTERWINDOW1, wxPoint(176,320), wxDefaultSize, wxSP_3D|wxSP_LIVE_UPDATE, _T("ID_SPLITTERWINDOW1"));
    SplitterWindow1->SetMinSize(wxSize(10,10));
    SplitterWindow1->SetMinimumPaneSize(10);
    Panel1 = new wxPanel(SplitterWindow1, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    FlexGridSizer1 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer1->AddGrowableCol(0);
    FlexGridSizer1->AddGrowableRow(1);
    Panel5 = new wxPanel(Panel1, ID_PANEL5, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL5"));
    FlexGridSizer4 = new wxFlexGridSizer(5, 1, 0, 0);
    FlexGridSizer4->AddGrowableCol(0);
    BoxSizer2 = new wxBoxSizer(wxHORIZONTAL);
    StaticText1 = new wxStaticText(Panel5, ID_STATICTEXT1, _("Tree view of HMT map"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT1"));
    wxFont StaticText1Font = wxSystemSettings::GetFont(wxSYS_SYSTEM_FONT);
    if ( !StaticText1Font.Ok() ) StaticText1Font = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
    StaticText1Font.SetPointSize((int)(StaticText1Font.GetPointSize() * 1.000000));
    StaticText1Font.SetWeight(wxFONTWEIGHT_BOLD);
    StaticText1->SetFont(StaticText1Font);
    BoxSizer2->Add(StaticText1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer4->Add(BoxSizer2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer5 = new wxFlexGridSizer(0, 2, 0, 0);
    StaticText2 = new wxStaticText(Panel5, ID_STATICTEXT2, _("Select hypothesis:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    FlexGridSizer5->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbHypos = new wxChoice(Panel5, ID_CHOICE1, wxDefaultPosition, wxDefaultSize, 0, 0, 0, wxDefaultValidator, _T("ID_CHOICE1"));
    FlexGridSizer5->Add(cbHypos, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer4->Add(FlexGridSizer5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel5->SetSizer(FlexGridSizer4);
    FlexGridSizer4->Fit(Panel5);
    FlexGridSizer4->SetSizeHints(Panel5);
    FlexGridSizer1->Add(Panel5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    SplitterWindow3 = new wxSplitterWindow(Panel1, ID_SPLITTERWINDOW3, wxDefaultPosition, wxDefaultSize, wxSP_3D|wxSP_LIVE_UPDATE, _T("ID_SPLITTERWINDOW3"));
    SplitterWindow3->SetMinSize(wxSize(10,10));
    SplitterWindow3->SetMinimumPaneSize(10);
    Panel9 = new wxPanel(SplitterWindow3, ID_PANEL9, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL9"));
    FlexGridSizer9 = new wxFlexGridSizer(0, 0, 0, 0);
    FlexGridSizer9->AddGrowableCol(0);
    FlexGridSizer9->AddGrowableRow(0);
    treeView = new wxTreeCtrl(Panel9, ID_TREECTRL1, wxDefaultPosition, wxDefaultSize, wxTR_LINES_AT_ROOT|wxTR_MULTIPLE|wxTR_DEFAULT_STYLE|wxVSCROLL|wxHSCROLL, wxDefaultValidator, _T("ID_TREECTRL1"));
    FlexGridSizer9->Add(treeView, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel9->SetSizer(FlexGridSizer9);
    FlexGridSizer9->Fit(Panel9);
    FlexGridSizer9->SetSizeHints(Panel9);
    Panel8 = new wxPanel(SplitterWindow3, ID_PANEL8, wxPoint(0,305), wxSize(370,235), wxTAB_TRAVERSAL, _T("ID_PANEL8"));
    FlexGridSizer8 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer8->AddGrowableCol(0);
    FlexGridSizer8->AddGrowableRow(0);
    edLog = new wxTextCtrl(Panel8, ID_TEXTCTRL1, _("Log window"), wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER|wxTE_PROCESS_TAB|wxTE_MULTILINE|wxTE_READONLY|wxHSCROLL|wxVSCROLL, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    wxFont edLogFont = wxSystemSettings::GetFont(wxSYS_OEM_FIXED_FONT);
    if ( !edLogFont.Ok() ) edLogFont = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
    edLogFont.SetPointSize((int)(edLogFont.GetPointSize() * 1.000000));
    edLog->SetFont(edLogFont);
    FlexGridSizer8->Add(edLog, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    Panel8->SetSizer(FlexGridSizer8);
    FlexGridSizer8->SetSizeHints(Panel8);
    SplitterWindow3->SplitHorizontally(Panel9, Panel8);
    FlexGridSizer1->Add(SplitterWindow3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel1->SetSizer(FlexGridSizer1);
    FlexGridSizer1->Fit(Panel1);
    FlexGridSizer1->SetSizeHints(Panel1);
    Panel2 = new wxPanel(SplitterWindow1, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
    BoxSizer1 = new wxBoxSizer(wxHORIZONTAL);
    SplitterWindow2 = new wxSplitterWindow(Panel2, ID_SPLITTERWINDOW2, wxDefaultPosition, wxDefaultSize, wxSP_3D|wxSP_LIVE_UPDATE, _T("ID_SPLITTERWINDOW2"));
    SplitterWindow2->SetMinSize(wxSize(10,10));
    SplitterWindow2->SetMinimumPaneSize(10);
    Panel3 = new wxPanel(SplitterWindow2, ID_PANEL3, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL3"));
    FlexGridSizer2 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer2->AddGrowableCol(0);
    FlexGridSizer2->AddGrowableRow(1);
    StaticText3 = new wxStaticText(Panel3, ID_STATICTEXT3, _("Global HMT map"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT3"));
    wxFont StaticText3Font = wxSystemSettings::GetFont(wxSYS_SYSTEM_FONT);
    if ( !StaticText3Font.Ok() ) StaticText3Font = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
    StaticText3Font.SetPointSize((int)(StaticText3Font.GetPointSize() * 1.000000));
    StaticText3Font.SetWeight(wxFONTWEIGHT_BOLD);
    StaticText3->SetFont(StaticText3Font);
    FlexGridSizer2->Add(StaticText3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel6 = new wxPanel(Panel3, ID_PANEL6, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL6"));
    FlexGridSizer6 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer6->AddGrowableCol(0);
    FlexGridSizer6->AddGrowableRow(0);
    Panel6->SetSizer(FlexGridSizer6);
    FlexGridSizer6->Fit(Panel6);
    FlexGridSizer6->SetSizeHints(Panel6);
    FlexGridSizer2->Add(Panel6, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel3->SetSizer(FlexGridSizer2);
    FlexGridSizer2->Fit(Panel3);
    FlexGridSizer2->SetSizeHints(Panel3);
    Panel4 = new wxPanel(SplitterWindow2, ID_PANEL4, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL4"));
    FlexGridSizer3 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer3->AddGrowableCol(0);
    FlexGridSizer3->AddGrowableRow(1);
    StaticText4 = new wxStaticText(Panel4, ID_STATICTEXT4, _("Selected area local map"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT4"));
    wxFont StaticText4Font = wxSystemSettings::GetFont(wxSYS_SYSTEM_FONT);
    if ( !StaticText4Font.Ok() ) StaticText4Font = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
    StaticText4Font.SetPointSize((int)(StaticText4Font.GetPointSize() * 1.000000));
    StaticText4Font.SetWeight(wxFONTWEIGHT_BOLD);
    StaticText4->SetFont(StaticText4Font);
    FlexGridSizer3->Add(StaticText4, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel7 = new wxPanel(Panel4, ID_PANEL7, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL7"));
    FlexGridSizer7 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer7->AddGrowableCol(0);
    FlexGridSizer7->AddGrowableRow(0);
    Panel7->SetSizer(FlexGridSizer7);
    FlexGridSizer7->Fit(Panel7);
    FlexGridSizer7->SetSizeHints(Panel7);
    FlexGridSizer3->Add(Panel7, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel4->SetSizer(FlexGridSizer3);
    FlexGridSizer3->Fit(Panel4);
    FlexGridSizer3->SetSizeHints(Panel4);
    SplitterWindow2->SplitHorizontally(Panel3, Panel4);
    BoxSizer1->Add(SplitterWindow2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    Panel2->SetSizer(BoxSizer1);
    BoxSizer1->Fit(Panel2);
    BoxSizer1->SetSizeHints(Panel2);
    SplitterWindow1->SplitVertically(Panel1, Panel2);
    MenuBar1 = new wxMenuBar();
    Menu1 = new wxMenu();
    MenuItem3 = new wxMenuItem(Menu1, ID_MENUITEM1, _("Load..."), _("Loads a \"hmtmap\" file"), wxITEM_NORMAL);
    Menu1->Append(MenuItem3);
    Menu1->AppendSeparator();
    menuExportLocalMaps = new wxMenuItem(Menu1, ID_MENUITEM4, _("Export local maps..."), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(menuExportLocalMaps);
    Menu1->AppendSeparator();
    MenuItem1 = new wxMenuItem(Menu1, idMenuQuit, _("Quit\tAlt-F4"), _("Quit the application"), wxITEM_NORMAL);
    Menu1->Append(MenuItem1);
    MenuBar1->Append(Menu1, _("&File"));
    Menu3 = new wxMenu();
    MenuItem4 = new wxMenuItem(Menu3, ID_MENUITEM2, _("Translation btw. 2 areas..."), wxEmptyString, wxITEM_NORMAL);
    Menu3->Append(MenuItem4);
    MenuItem5 = new wxMenuItem(Menu3, ID_MENUITEM3, _("Overlap probability btw. 2 areas..."), wxEmptyString, wxITEM_NORMAL);
    Menu3->Append(MenuItem5);
    MenuItem6 = new wxMenu();
    MenuItem7 = new wxMenuItem(MenuItem6, ID_MENUITEM6, _("Grid matching"), wxEmptyString, wxITEM_NORMAL);
    MenuItem6->Append(MenuItem7);
    MenuItem8 = new wxMenuItem(MenuItem6, ID_MENUITEM7, _("FabMap"), wxEmptyString, wxITEM_NORMAL);
    MenuItem6->Append(MenuItem8);
    Menu3->Append(ID_MENUITEM5, _("Topological transition models..."), MenuItem6, wxEmptyString);
    MenuBar1->Append(Menu3, _("&Compute"));
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
    ToolBar1 = new wxToolBar(this, ID_TOOLBAR1, wxDefaultPosition, wxDefaultSize, wxTB_FLAT|wxTB_VERTICAL|wxTB_TEXT|wxNO_BORDER, _T("ID_TOOLBAR1"));
    ToolBarItem1 = ToolBar1->AddTool(ID_TOOLBARITEM1, _("Load..."), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FILE_OPEN")),wxART_TOOLBAR), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FILE_OPEN")),wxART_TOOLBAR), wxITEM_NORMAL, wxEmptyString, wxEmptyString);
    ToolBar1->AddSeparator();
    ToolBarItem2 = ToolBar1->AddTool(ID_TOOLBARITEM2, _("Exit"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_CROSS_MARK")),wxART_TOOLBAR), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_CROSS_MARK")),wxART_TOOLBAR), wxITEM_NORMAL, wxEmptyString, wxEmptyString);
    ToolBar1->Realize();
    SetToolBar(ToolBar1);
    Center();

    Connect(ID_TREECTRL1,wxEVT_COMMAND_TREE_SEL_CHANGED,(wxObjectEventFunction)&hmtMapViewerFrame::OntreeViewSelectionChanged);
    Connect(ID_MENUITEM1,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&hmtMapViewerFrame::OnMenuLoad);
    Connect(ID_MENUITEM4,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&hmtMapViewerFrame::OnmenuExportLocalMapsSelected);
    Connect(idMenuQuit,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&hmtMapViewerFrame::OnQuit);
    Connect(ID_MENUITEM2,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&hmtMapViewerFrame::OnMenuTranslationBtw2);
    Connect(ID_MENUITEM3,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&hmtMapViewerFrame::OnMenuOverlapBtw2);
    Connect(ID_MENUITEM6,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&hmtMapViewerFrame::OnTopologicalModel_Gridmap);
    Connect(ID_MENUITEM7,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&hmtMapViewerFrame::OnTopologicalModel_Fabmap);
    Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&hmtMapViewerFrame::OnAbout);
    Connect(ID_TOOLBARITEM1,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&hmtMapViewerFrame::OnMenuLoad);
    Connect(ID_TOOLBARITEM2,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&hmtMapViewerFrame::OnQuit);
    //*)

	// Fix sizes:
	SplitterWindow1->SetMinSize(wxSize(200,200));
	SplitterWindow2->SetMinSize(wxSize(200,200));
	SplitterWindow3->SetMinSize(wxSize(200,200));
	Fit();
	Refresh();

    // Create GL canvas:
    // -------------------------------
	m_canvas_HMAP = new CMyGLCanvas( Panel6, wxID_ANY, wxDefaultPosition, wxDefaultSize );
	Panel6->SetMinSize( wxSize(200,200) );
	m_canvas_HMAP->SetMinSize( wxSize(200,200) );
	m_canvas_HMAP->m_openGLScene->insert( opengl::CGridPlaneXY::Create() );

    FlexGridSizer6->Add(m_canvas_HMAP, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);


	m_canvas_LMH = new CMyGLCanvas( Panel7, wxID_ANY, wxDefaultPosition, wxDefaultSize );
	Panel7->SetMinSize( wxSize(200,200) );
	m_canvas_LMH->SetMinSize( wxSize(200,200) );
	m_canvas_LMH->m_openGLScene->insert( opengl::CGridPlaneXY::Create() );
    FlexGridSizer7->Add(m_canvas_LMH, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);

	m_canvas_LMH->cameraPointingX = 0;
	m_canvas_LMH->cameraPointingY = 0;
	m_canvas_LMH->cameraPointingZ = 0;
	m_canvas_LMH->cameraZoomDistance = 100;
	m_canvas_LMH->cameraElevationDeg=90;
	m_canvas_LMH->cameraAzimuthDeg=0;
	m_canvas_LMH->cameraIsProjective = false;


    Maximize();

	// Splitters:
	SplitterWindow1->SetSashPosition(300);
	SplitterWindow2->SetSashPosition(400);
	SplitterWindow3->SetSashPosition( GetSize().GetHeight() - 300);

	// Auto load file from command line?
	timAutoLoad.SetOwner(this, ID_TIMER1);
	if (!global_fileToOpen.empty())
	{
		Connect(ID_TIMER1,wxEVT_TIMER,(wxObjectEventFunction)&hmtMapViewerFrame::OntimAutoLoadTrigger);
		timAutoLoad.Start(250, true);
	}
}

hmtMapViewerFrame::~hmtMapViewerFrame()
{
    //(*Destroy(hmtMapViewerFrame)
    //*)

    WX_START_TRY

	delete hmt_map;
	hmt_map = NULL;

    WX_END_TRY
}

//------------------------------------------------------------------------
// Auto-load the file passed in the cmd line (if any)
//------------------------------------------------------------------------
void hmtMapViewerFrame::OntimAutoLoadTrigger(wxTimerEvent& event)
{
	// Now: Open it:
	if (!global_fileToOpen.empty())
		loadHTMSLAMFromFile( global_fileToOpen );
}


void hmtMapViewerFrame::OnQuit(wxCommandEvent& event)
{
    Close();
}

void hmtMapViewerFrame::OnAbout(wxCommandEvent& event)
{
//    wxString msg = wxbuildinfo(long_f);
//    wxMessageBox(msg, _("Welcome to..."));
}

void hmtMapViewerFrame::OnMenuLoad(wxCommandEvent& event)
{
	string 	fil;
	if ( AskForOpenHMTMap( fil ) )
		loadHTMSLAMFromFile( fil );
}


bool hmtMapViewerFrame::loadHTMSLAMFromFile( const std::string &filePath )
{
    WX_START_TRY

    if (!fileExists(filePath))
	{
		wxMessageBox(_U( string(string("File doesn't exist:\n")+filePath).c_str() ),_("Error loading file"),wxOK,this);
		return false;
	}

	wxBusyCursor busy;

	// Save the path
	WX_START_TRY
        string the_path( extractFileDirectory(filePath) );
        iniFile->write(iniFileSect,"LastDir", the_path );
	WX_END_TRY

    // Load
    CFileGZInputStream(filePath) >> *hmt_map;


    m_curFileOpen = filePath;

    // Refresh views:
    // ---------------------------

    // The tree:
    rebuildTreeView();

    // The global map:
    updateGlobalMapView();

    return true;

    WX_END_TRY

    return false;
}


void hmtMapViewerFrame::rebuildTreeView()
{
	WX_START_TRY

	wxBusyCursor        waitCursor;
	treeView->DeleteAllItems();

	treeView->SetQuickBestSize(true);

	// Root element & Areas:
	wxTreeItemId root = treeView->AddRoot(_("Areas"),0,-1,NULL);

	CHierarchicalMHMap::const_iterator it;
	size_t i;

	for (i=0,it=hmt_map->m_map.begin();it!=hmt_map->m_map.end();it++,i++)
	{
		string str = format( "Area %i", (int)it->second->getID() );

		//wxTreeItemId treeNode =
		treeView->AppendItem(
			root,
			_U(str.c_str()),
			0,-1,
			new CItemData(it->second,i));
	}

	treeView->ExpandAll();

	// List of hypotheses:
	cbHypos->Clear();

	for ( aligned_containers<THypothesisID, CLocalMetricHypothesis>::map_t::const_iterator l= hmt_map->m_LMHs.begin();l!=hmt_map->m_LMHs.end();++l)
		cbHypos->Append( _U( format("%i",(int)l->first).c_str()) );

	cbHypos->SetSelection(0);

    WX_END_TRY
}

//------------------------------------------------------------------------
//    Asks the user for a file, return false if user cancels
//------------------------------------------------------------------------
bool hmtMapViewerFrame::AskForOpenHMTMap( std::string &fil )
{
	wxString caption = wxT("Choose a file to open");
	wxString wildcard = wxT("HMT-SLAM files (*.hmtslam)|*.hmtslam|All files (*.*)|*.*");
	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
	wxString defaultFilename = wxT("");

	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_OPEN | wxFD_FILE_MUST_EXIST );

	if (dialog.ShowModal() == wxID_OK)
	{
		fil = string( dialog.GetPath().mb_str() );
		return true;
	}
	else return false;
}

void hmtMapViewerFrame::updateLocalMapView()
{
    WX_START_TRY

	CMyRedirector	redir( edLog );

	m_canvas_LMH->m_openGLScene->clear();

	// Get the hypothesis ID:
	THypothesisID	hypID = (THypothesisID	)atoi( cbHypos->GetStringSelection().mb_str() );
	if ( hmt_map->m_LMHs.find(hypID)==hmt_map->m_LMHs.end() )
	{
		wxMessageBox(_U( format("No LMH has hypothesis ID %i!", (int)hypID).c_str() ), _("Error with topological hypotesis"));
		return;
	}

	// Get the selected area or LMH in the tree view:
	wxArrayTreeItemIds	lstSelect;
	size_t	nSel = treeView->GetSelections(lstSelect);
	if (!nSel) return;

	CItemData	*data1 = static_cast<CItemData*>( treeView->GetItemData( lstSelect.Item(0) ) );
	if (!data1) return;
	if (!data1->m_ptr) return;

	CSerializablePtr obj = data1->m_ptr;
	if (obj->GetRuntimeClass()==CLASS_ID(CHMHMapNode))
	{
		// The 3D view:
		opengl::CSetOfObjectsPtr objs = opengl::CSetOfObjects::Create();

		// -------------------------------------------
		// Draw a grid on the ground:
		// -------------------------------------------
		{
			opengl::CGridPlaneXYPtr obj = opengl::CGridPlaneXY::Create(-100,100,-100,100,0,5);
			obj->setColor(0.4,0.4,0.4);
			objs->insert(obj);  // it will free the memory
		}


		// Two passes: 1st draw the map on the ground, then the rest.
		for (int nRound=0;nRound<2;nRound++)
		{
			CHMHMapNodePtr firstArea;
			CPose3DPDFGaussian		refPoseThisArea;

			for (size_t  nSelItem = 0; nSelItem<nSel;nSelItem++)
			{
				CItemData	*data1 = static_cast<CItemData*>( treeView->GetItemData( lstSelect.Item(nSelItem) ) );
				if (!data1) continue;
				if (!data1->m_ptr) continue;

				CHMHMapNodePtr area= CHMHMapNodePtr(data1->m_ptr);
				if (!area) continue;

				// Is this the first rendered area??
				if ( !firstArea )
				{
					firstArea = area;
				}
				else
				{
					// Compute the translation btw. ref. and current area:
					CPose3DPDFParticles		pdf;

					hmt_map->m_map.computeCoordinatesTransformationBetweenNodes(
						firstArea->getID(),
						area->getID(),
						pdf,
						hypID,
						200 );
						/*0.15f,
						DEG2RAD(5.0f) );*/

					refPoseThisArea.copyFrom( pdf );
					cout << "Pose " << firstArea->getID() << " - " << area->getID() << refPoseThisArea << endl;
				}

				CMultiMetricMapPtr obj_mmap = area->m_annotations.getAs<CMultiMetricMap>( NODE_ANNOTATION_METRIC_MAPS, hypID, false );

				CRobotPosesGraphPtr obj_robposes = area->m_annotations.getAs<CRobotPosesGraph>( NODE_ANNOTATION_POSES_GRAPH, hypID, false );

				TPoseID	refPoseID;
				area->m_annotations.getElemental( NODE_ANNOTATION_REF_POSEID, refPoseID, hypID, true);

				// ---------------------------------------------------------
				// The metric map:
				// ---------------------------------------------------------
				if (nRound==0)
				{
					opengl::CSetOfObjectsPtr objMap= opengl::CSetOfObjects::Create();
					obj_mmap->getAs3DObject(objMap);
					objMap->setPose( refPoseThisArea.mean );
					objs->insert(objMap);
				}

				if (nRound==1)
				{
					// ---------------------------------------------------------
					// Bounding boxes for grid maps:
					// ---------------------------------------------------------
					if (obj_mmap->m_gridMaps.size())
					{
						float x_min = obj_mmap->m_gridMaps[0]->getXMin();
						float x_max = obj_mmap->m_gridMaps[0]->getXMax();
						float y_min = obj_mmap->m_gridMaps[0]->getYMin();
						float y_max = obj_mmap->m_gridMaps[0]->getYMax();

						opengl::CSetOfLinesPtr objBB = opengl::CSetOfLines::Create();
						objBB->setColor(0,0,1);
						objBB->setLineWidth( 4.0f );

						objBB->appendLine(x_min,y_min,0,  x_max,y_min,0);
						objBB->appendLine(x_max,y_min,0,  x_max,y_max,0);
						objBB->appendLine(x_max,y_max,0,  x_min,y_max,0);
						objBB->appendLine(x_min,y_max,0,  x_min,y_min,0);

						objBB->setPose( refPoseThisArea.mean );
						objs->insert(objBB);
					}

					// -----------------------------------------------
					// Draw a 3D coordinates corner for the ref. pose
					// -----------------------------------------------
					{
						CPose3D	p;
						(*obj_robposes)[refPoseID].pdf.getMean(p);

						opengl::CSetOfObjectsPtr corner = stock_objects::CornerXYZ();
						corner->setPose( refPoseThisArea.mean + p);
						corner->setName(format("AREA %i",(int)area->getID() ));
						corner->enableShowName();

						objs->insert( corner );
					}

					// -----------------------------------------------
					// Draw ellipsoid with uncertainty of pose transformation
					// -----------------------------------------------
					if (refPoseThisArea.cov(0,0)!=0 || refPoseThisArea.cov(1,1)!=0)
					{
						opengl::CEllipsoidPtr ellip = opengl::CEllipsoid::Create();
						ellip->setPose( refPoseThisArea.mean );
						ellip->enableDrawSolid3D(false);

						CMatrixDouble C = CMatrixDouble(refPoseThisArea.cov);

						if (C(2,2)<1e6)
								C.setSize(2,2);
						else	C.setSize(3,3);

						ellip->setCovMatrix(C);
						ellip->setQuantiles(3);
						ellip->setLocation( ellip->getPoseX(), ellip->getPoseY(), ellip->getPoseZ()+0.5);
						ellip->setColor(1,0,0);
						ellip->setLineWidth(3);

						objs->insert( ellip );
					}

					// ---------------------------------------------------------
					// Draw each of the robot poses as 2D/3D ellipsoids
					// ---------------------------------------------------------
					for (CRobotPosesGraph::iterator it=obj_robposes->begin();it!=obj_robposes->end();++it)
					{

					}
				}

			} // end for nSelItem

		} // two pass

		// Add to the scene:
		m_canvas_LMH->m_openGLScene->insert(objs);
	}
	else
	if (obj->GetRuntimeClass()==CLASS_ID( CLocalMetricHypothesis ))
	{
		//CLocalMetricHypothesis *lmh = static_cast<CLocalMetricHypothesis*>( obj );

	}

	m_canvas_LMH->Refresh();

	WX_END_TRY
}

void hmtMapViewerFrame::updateGlobalMapView()
{
	m_canvas_HMAP->m_openGLScene->clear();

	wxBusyCursor busy;


	// Dump text representation to log window:
	{
		edLog->Clear();
		CMyRedirector	redir( edLog );

		CStringList	strLst;
		hmt_map->m_map.dumpAsText( strLst );
		string str;
		strLst.getText( str );
		cout << str << endl;
	}

	if (hmt_map->m_map.getFirstNode())
	{
		CHMHMapNode::TNodeID	refID = hmt_map->m_map.getFirstNode()->getID();

		THypothesisID	hypID = (THypothesisID	)atoi( cbHypos->GetStringSelection().mb_str() );

		if ( hmt_map->m_LMHs.find(hypID)==hmt_map->m_LMHs.end() )
		{
			wxMessageBox(_U( format("No LMH has hypothesis ID %i!", (int)hypID).c_str() ), _("Error with topological hypotesis"));
			return;
		}
//		cout << "Showing hypothesis ID: " << hypID  << endl;

		hmt_map->m_map.getAs3DScene(
			*m_canvas_HMAP->m_openGLScene,
			refID,
			hypID );

		m_canvas_HMAP->Refresh();
	}
}

void hmtMapViewerFrame::OnMenuOverlapBtw2(wxCommandEvent& event)
{
    WX_START_TRY

    wxArrayTreeItemIds	lstSelect;

	size_t	nSel = treeView->GetSelections(lstSelect);

	if (nSel!=2)
	{
		wxMessageBox(_("Please, select two areas from the tree first"),_("Error"));
		return;
	}

	CItemData	*data1 = static_cast<CItemData*>( treeView->GetItemData( lstSelect.Item(0) ) );
	CItemData	*data2 = static_cast<CItemData*>( treeView->GetItemData( lstSelect.Item(1) ) );

	ASSERT_(data1);
	ASSERT_(data2);
	ASSERT_(data1->m_ptr);
	ASSERT_(data2->m_ptr);

	CHMHMapNodePtr area1=CHMHMapNodePtr(data1->m_ptr);
	CHMHMapNodePtr area2=CHMHMapNodePtr(data2->m_ptr);

	THypothesisID	hypID = (THypothesisID	)atoi( cbHypos->GetStringSelection().mb_str() );

	const size_t monteCarloSamples = 100;

	double prob12 = hmt_map->m_map.computeOverlapProbabilityBetweenNodes(
		area1->getID(),
		area2->getID(),
		hypID,
		monteCarloSamples);

	double prob21 = hmt_map->m_map.computeOverlapProbabilityBetweenNodes(
		area2->getID(),
		area1->getID(),
		hypID,
		monteCarloSamples);


	{
		CMyRedirector	redir( edLog );

		cout << "Overlap probability: " << area1->getID() << " - " << area2->getID() << ": " << prob12 << endl;
		cout << "Overlap probability: " << area2->getID() << " - " << area1->getID() << ": " << prob21 << endl;
	}

    WX_END_TRY
}

void hmtMapViewerFrame::OnMenuTranslationBtw2(wxCommandEvent& event)
{
    WX_START_TRY

    wxArrayTreeItemIds	lstSelect;

	size_t	nSel = treeView->GetSelections(lstSelect);

	if (nSel!=2)
	{
		wxMessageBox(_("Please, select two areas from the tree first"),_("Error"));
		return;
	}

	CItemData	*data1 = static_cast<CItemData*>( treeView->GetItemData( lstSelect.Item(0) ) );
	CItemData	*data2 = static_cast<CItemData*>( treeView->GetItemData( lstSelect.Item(1) ) );

	ASSERT_(data1);
	ASSERT_(data2);
	ASSERT_(data1->m_ptr);
	ASSERT_(data2->m_ptr);

	CHMHMapNodePtr area1=CHMHMapNodePtr(data1->m_ptr);
	CHMHMapNodePtr area2=CHMHMapNodePtr (data2->m_ptr);

	THypothesisID	hypID = (THypothesisID	)atoi( cbHypos->GetStringSelection().mb_str() );

	const size_t monteCarloSamples = 100;

	CPose3DPDFParticles		pdf12;

	hmt_map->m_map.computeCoordinatesTransformationBetweenNodes(
		area1->getID(),
		area2->getID(),
		pdf12,
		hypID,
		monteCarloSamples,
		0,0);

	CPose3DPDFGaussian	pdfGauss;
	pdfGauss.copyFrom(pdf12);

	{
		CMyRedirector	redir( edLog );

		cout << "PDF between " << area1->getID() << " -> " << area2->getID() << ": " << endl;
		cout << pdfGauss << endl;
	}

    WX_END_TRY
}


void hmtMapViewerFrame::OntreeViewSelectionChanged(wxTreeEvent& event)
{
	updateLocalMapView();
}

void hmtMapViewerFrame::OnmenuExportLocalMapsSelected(wxCommandEvent& event)
{
    WX_START_TRY

	// Get the hypothesis ID:
	THypothesisID	hypID = (THypothesisID	)atoi( cbHypos->GetStringSelection().mb_str() );
	if ( hmt_map->m_LMHs.find(hypID)==hmt_map->m_LMHs.end() )
	{
		wxMessageBox(_U( format("No LMH has hypothesis ID %i!", (int)hypID).c_str() ), _("Error with topological hypotesis"));
		return;
	}

	wxString caption = wxT("Choose the target file prefix");
	wxString wildcard = wxT("simplemap files (*.simplemap)|*.hmtslam|All files (*.*)|*.*");
	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
	wxString defaultFilename = wxT("");

	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

	if (dialog.ShowModal() != wxID_OK) return;

	string user_fil = string( dialog.GetPath().mb_str() );
	string fil_dir  = mrpt::system::extractFileDirectory( user_fil );
	string fil_name = mrpt::system::extractFileName( user_fil );

	string map_prefix = fil_dir+string("/")+fil_name;

	CHierarchicalMHMap::const_iterator it;

	for (it=hmt_map->m_map.begin();it!=hmt_map->m_map.end();it++)
	{
		CSimpleMap 	simpleMap;

		string map_file = map_prefix + format( "_map_area_%03u.simplemap",  (unsigned)it->first ); //it->second->m_label.c_str() );
		CHMHMapNodePtr area= it->second;

		CRobotPosesGraphPtr obj_poseGraph = area->m_annotations.getAs<CRobotPosesGraph>( NODE_ANNOTATION_POSES_GRAPH, hypID, false );
		obj_poseGraph->convertIntoSimplemap( simpleMap );

		CFileGZOutputStream(map_file) << simpleMap;		// Save simplemap
	}

    WX_END_TRY
}


void hmtMapViewerFrame::OnTopologicalModel_Gridmap(wxCommandEvent& event)
{
	WX_START_TRY

	// Get the hypothesis ID:
	const THypothesisID	hypID = (THypothesisID	)atoi( cbHypos->GetStringSelection().mb_str() );
	if ( hmt_map->m_LMHs.find(hypID)==hmt_map->m_LMHs.end() )
	{
		wxMessageBox(_U( format("No LMH has hypothesis ID %i!", (int)hypID).c_str() ), _("Error with topological hypotesis"));
		return;
	}

    wxArrayTreeItemIds	lstSelect;

	size_t	nSel = treeView->GetSelections(lstSelect);
	if (nSel!=2)
	{
		wxMessageBox(_("Please, select two areas from the tree first"),_("Error"));
		return;
	}

	CItemData	*data1 = static_cast<CItemData*>( treeView->GetItemData( lstSelect.Item(0) ) );
	CItemData	*data2 = static_cast<CItemData*>( treeView->GetItemData( lstSelect.Item(1) ) );
	ASSERT_(data1);
	ASSERT_(data2);
	ASSERT_(data1->m_ptr);
	ASSERT_(data2->m_ptr);

	const CHMHMapNodePtr area1=CHMHMapNodePtr(data1->m_ptr);
	const CHMHMapNodePtr area2=CHMHMapNodePtr (data2->m_ptr);

	CTopLCDetectorBasePtr TLCD = CTopLCDetectorBasePtr( hmt_map->loopClosureDetector_factory("gridmaps") );

	CMyRedirector	redir( edLog );
	wxBusyCursor busy;


	double this_log_lik;

	// get the output from this LC detector:
	CPose3DPDFPtr pdf = TLCD->computeTopologicalObservationModel(
		hypID,
		area2,
		area1,
		this_log_lik );

	CPose3DPDFSOGPtr pdfSOG = CPose3DPDFSOGPtr(pdf);

	cout << "The relative estimated pose is: " << endl;
	for (CPose3DPDFSOG::iterator i=pdfSOG->begin();i!=pdfSOG->end();i++)
	{
		cout << " Mode: " << i->val.mean << endl;
	}


    WX_END_TRY
}

void hmtMapViewerFrame::OnTopologicalModel_Fabmap(wxCommandEvent& event)
{

}

