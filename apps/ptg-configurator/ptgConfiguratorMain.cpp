/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "ptgConfiguratorMain.h"
#include <wx/msgdlg.h>
#include "CAboutBox.h"

//(*InternalHeaders(ptgConfiguratorframe)
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/settings.h>
#include <wx/font.h>
#include <wx/intl.h>
#include <wx/image.h>
#include <wx/string.h>
//*)

#include <mrpt/gui/WxUtils.h>
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

#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/utils/CConfigFileMemory.h>
#include <mrpt/utils/CConfigFilePrefixer.h>

mrpt::nav::CParameterizedTrajectoryGenerator  * ptg = NULL;


//(*IdInit(ptgConfiguratorframe)
const long ptgConfiguratorframe::ID_STATICTEXT1 = wxNewId();
const long ptgConfiguratorframe::ID_CHOICE1 = wxNewId();
const long ptgConfiguratorframe::ID_STATICTEXT2 = wxNewId();
const long ptgConfiguratorframe::ID_SPINCTRL1 = wxNewId();
const long ptgConfiguratorframe::ID_BUTTON1 = wxNewId();
const long ptgConfiguratorframe::ID_TEXTCTRL1 = wxNewId();
const long ptgConfiguratorframe::ID_PANEL1 = wxNewId();
const long ptgConfiguratorframe::ID_XY_GLCANVAS = wxNewId();
const long ptgConfiguratorframe::ID_TEXTCTRL2 = wxNewId();
const long ptgConfiguratorframe::idMenuQuit = wxNewId();
const long ptgConfiguratorframe::idMenuAbout = wxNewId();
const long ptgConfiguratorframe::ID_STATUSBAR1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(ptgConfiguratorframe,wxFrame)
    //(*EventTable(ptgConfiguratorframe)
    //*)
END_EVENT_TABLE()


// Aux function
void add_robotShapeCirc_to_setOfLines(
	const double R,
	mrpt::opengl::CSetOfLines &gl_shape,
	const mrpt::poses::CPose2D &origin = mrpt::poses::CPose2D () )
{
	const int N = 10;
	// Transform coordinates:
	mrpt::math::CVectorDouble shap_x(N), shap_y(N),shap_z(N);
	for (int i=0;i<N;i++) {
		origin.composePoint(
			R*cos(i*2*M_PI/N-1),R*sin(i*2*M_PI/N-1), 0,
			shap_x[i],  shap_y[i],  shap_z[i]);
	}
	gl_shape.appendLine( shap_x[0], shap_y[0], shap_z[0], shap_x[1],shap_y[1],shap_z[1] );
	for (int i=0;i<=shap_x.size();i++) {
		const int idx = i % shap_x.size();
		gl_shape.appendLineStrip( shap_x[idx],shap_y[idx], shap_z[idx]);
	}
}

void add_robotShape_to_setOfLines(
	const mrpt::math::CVectorFloat &shap_x_,
	const mrpt::math::CVectorFloat &shap_y_,
	mrpt::opengl::CSetOfLines &gl_shape,
	const mrpt::poses::CPose2D &origin = mrpt::poses::CPose2D () )
{
	const int N = shap_x_.size();
	if (N>=2 && N==shap_y_.size() )
	{
		// Transform coordinates:
		mrpt::math::CVectorDouble shap_x(N), shap_y(N),shap_z(N);
		for (int i=0;i<N;i++) {
			origin.composePoint(
				shap_x_[i], shap_y_[i], 0,
				shap_x[i],  shap_y[i],  shap_z[i]);
		}

		gl_shape.appendLine( shap_x[0], shap_y[0], shap_z[0], shap_x[1],shap_y[1],shap_z[1] );
		for (int i=0;i<=shap_x.size();i++) {
			const int idx = i % shap_x.size();
			gl_shape.appendLineStrip( shap_x[idx],shap_y[idx], shap_z[idx]);
		}
	}
}

ptgConfiguratorframe::ptgConfiguratorframe(wxWindow* parent,wxWindowID id)
{
    // Load my custom icons:
#if wxCHECK_VERSION(2, 8, 0)
    wxArtProvider::Push(new MyArtProvider);
#else
    wxArtProvider::PushProvider(new MyArtProvider);
#endif


    //(*Initialize(ptgConfiguratorframe)
    wxFlexGridSizer* FlexGridSizer4;
    wxMenuItem* MenuItem2;
    wxFlexGridSizer* FlexGridSizer3;
    wxMenuItem* MenuItem1;
    wxFlexGridSizer* FlexGridSizer5;
    wxFlexGridSizer* FlexGridSizer2;
    wxMenu* Menu1;
    wxMenuBar* MenuBar1;
    wxFlexGridSizer* FlexGridSizer6;
    wxFlexGridSizer* FlexGridSizer1;
    wxMenu* Menu2;

    Create(parent, id, _("PTG configurator - Part of the MRPT project"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("id"));
    SetClientSize(wxSize(893,576));
    {
    	wxIcon FrameIcon;
    	FrameIcon.CopyFromBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("MAIN_ICON")),wxART_OTHER));
    	SetIcon(FrameIcon);
    }
    FlexGridSizer1 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer1->AddGrowableCol(0);
    FlexGridSizer1->AddGrowableRow(1);
    Panel1 = new wxPanel(this, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    FlexGridSizer2 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer2->AddGrowableCol(0);
    FlexGridSizer2->AddGrowableRow(0);
    FlexGridSizer3 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer3->AddGrowableCol(0);
    FlexGridSizer3->AddGrowableRow(3);
    StaticText1 = new wxStaticText(Panel1, ID_STATICTEXT1, _("Select a PTG class:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    FlexGridSizer3->Add(StaticText1, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    cbPTGClass = new wxChoice(Panel1, ID_CHOICE1, wxDefaultPosition, wxDefaultSize, 0, 0, wxCB_SORT, wxDefaultValidator, _T("ID_CHOICE1"));
    FlexGridSizer3->Add(cbPTGClass, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer4 = new wxFlexGridSizer(1, 0, 0, 0);
    StaticText2 = new wxStaticText(Panel1, ID_STATICTEXT2, _("PTG index for cfg file:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    FlexGridSizer4->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edPTGIndex = new wxSpinCtrl(Panel1, ID_SPINCTRL1, _T("0"), wxDefaultPosition, wxDefaultSize, 0, 0, 100, 0, _T("ID_SPINCTRL1"));
    edPTGIndex->SetValue(_T("0"));
    FlexGridSizer4->Add(edPTGIndex, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnReloadParams = new wxButton(Panel1, ID_BUTTON1, _("Initialize PTG from params..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
    wxFont btnReloadParamsFont(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxBOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    btnReloadParams->SetFont(btnReloadParamsFont);
    FlexGridSizer4->Add(btnReloadParams, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer3->Add(FlexGridSizer4, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    edCfg = new wxTextCtrl(Panel1, ID_TEXTCTRL1, wxEmptyString, wxDefaultPosition, wxSize(-1,150), wxTE_PROCESS_ENTER|wxTE_PROCESS_TAB|wxTE_MULTILINE|wxHSCROLL|wxTE_DONTWRAP|wxALWAYS_SHOW_SB, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    wxFont edCfgFont = wxSystemSettings::GetFont(wxSYS_OEM_FIXED_FONT);
    if ( !edCfgFont.Ok() ) edCfgFont = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
    edCfgFont.SetPointSize(8);
    edCfgFont.SetFamily(wxTELETYPE);
    edCfg->SetFont(edCfgFont);
    FlexGridSizer3->Add(edCfg, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 2);
    FlexGridSizer5 = new wxFlexGridSizer(0, 3, 0, 0);
    FlexGridSizer3->Add(FlexGridSizer5, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer2->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    FlexGridSizer6 = new wxFlexGridSizer(0, 3, 0, 0);
    FlexGridSizer2->Add(FlexGridSizer6, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    Panel1->SetSizer(FlexGridSizer2);
    FlexGridSizer2->Fit(Panel1);
    FlexGridSizer2->SetSizeHints(Panel1);
    FlexGridSizer1->Add(Panel1, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    m_plot = new CMyGLCanvas(this,ID_XY_GLCANVAS,wxDefaultPosition,wxSize(600,450),wxTAB_TRAVERSAL,_T("ID_XY_GLCANVAS"));
    FlexGridSizer1->Add(m_plot, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    edLog = new wxTextCtrl(this, ID_TEXTCTRL2, wxEmptyString, wxDefaultPosition, wxSize(-1,100), wxTE_PROCESS_ENTER|wxTE_PROCESS_TAB|wxTE_MULTILINE|wxTE_READONLY|wxHSCROLL|wxTE_DONTWRAP|wxALWAYS_SHOW_SB, wxDefaultValidator, _T("ID_TEXTCTRL2"));
    wxFont edLogFont = wxSystemSettings::GetFont(wxSYS_OEM_FIXED_FONT);
    if ( !edLogFont.Ok() ) edLogFont = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
    edLogFont.SetPointSize(8);
    edLogFont.SetFamily(wxTELETYPE);
    edLog->SetFont(edLogFont);
    FlexGridSizer1->Add(edLog, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
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
    int __wxStatusBarWidths_1[3] = { -2, -2, -3 };
    int __wxStatusBarStyles_1[3] = { wxSB_NORMAL, wxSB_NORMAL, wxSB_NORMAL };
    StatusBar1->SetFieldsCount(3,__wxStatusBarWidths_1);
    StatusBar1->SetStatusStyles(3,__wxStatusBarStyles_1);
    SetStatusBar(StatusBar1);
    FlexGridSizer1->SetSizeHints(this);
    Center();

    Connect(ID_CHOICE1,wxEVT_COMMAND_CHOICE_SELECTED,(wxObjectEventFunction)&ptgConfiguratorframe::OncbPTGClassSelect);
    Connect(ID_SPINCTRL1,wxEVT_COMMAND_SPINCTRL_UPDATED,(wxObjectEventFunction)&ptgConfiguratorframe::OnedPTGIndexChange);
    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&ptgConfiguratorframe::OnbtnReloadParamsClick);
    Connect(idMenuQuit,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&ptgConfiguratorframe::OnQuit);
    Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&ptgConfiguratorframe::OnAbout);
    //*)

//	m_plot->Connect(wxEVT_MOTION,(wxObjectEventFunction)&ptgConfiguratorframe::Onplot3DMouseMove,0,this);
//	m_plot->Connect(wxEVT_LEFT_DOWN,(wxObjectEventFunction)&ptgConfiguratorframe::Onplot3DMouseClick,0,this);

	// Redirect all output to control:
	m_myRedirector = new CMyRedirector( edLog, true, 100, true);

	WX_START_TRY


	// Populate 3D views:
	// Split in 2 views:
	gl_view_PTG      = m_plot->m_openGLScene->createViewport("main");
	gl_view_TPSpace  = m_plot->m_openGLScene->createViewport("TPSpace");

	gl_view_PTG->setViewportPosition(0,0,0.5,1.0);
	gl_view_TPSpace->setViewportPosition(0.5,0.0,0.5,1.0);

	gl_robot_path = mrpt::opengl::CSetOfLines::Create();
	gl_robot_path->setLineWidth(1);
	gl_robot_path->setColor_u8( mrpt::utils::TColor(40,40,40, 200));
	gl_view_PTG->insert(gl_robot_path);

	{
		gl_axis_WS = mrpt::opengl::CAxis::Create(-10.0,-10.0,0,10.0,10.0,0.0, 1.0 ,2.0);
		gl_axis_WS->setTextScale(0.20);
		gl_axis_WS->enableTickMarks(true,true,true);
		gl_axis_WS->setColor_u8(mrpt::utils::TColor(30,30,30,50));
		gl_axis_WS->setTextLabelOrientation(0,0,0,0);
		gl_axis_WS->setTextLabelOrientation(1,0,0,0);

		gl_view_PTG->insert( gl_axis_WS );
	}
	{
		gl_axis_TPS= mrpt::opengl::CAxis::Create(-1.0,-1.0,0,1.0,1.0,0.0, 0.1 ,2.0);
		gl_axis_TPS->setTextScale(0.02);
		gl_axis_TPS->enableTickMarks(true,true,false);
		gl_axis_TPS->setColor_u8(mrpt::utils::TColor(30,30,30,50));
		gl_axis_TPS->setTextLabelOrientation(0,0,0,0);
		gl_axis_TPS->setTextLabelOrientation(1,0,0,0);
		gl_view_TPSpace->insert( gl_axis_TPS );
	}

	// Set camera:
	m_plot->cameraPointingX=0;
	m_plot->cameraPointingY=0;
	m_plot->cameraPointingZ=0;
	m_plot->cameraZoomDistance = 40;
	m_plot->cameraElevationDeg = 70;
	m_plot->cameraAzimuthDeg = -100;
	m_plot->cameraIsProjective = true;

	gl_view_TPSpace_cam = mrpt::opengl::CCamera::Create();
	gl_view_TPSpace->insert ( gl_view_TPSpace_cam );
	gl_view_TPSpace_cam->setAzimuthDegrees( -90 );
	gl_view_TPSpace_cam->setElevationDegrees(90);
	gl_view_TPSpace_cam->setProjectiveModel( false );
	gl_view_TPSpace_cam->setZoomDistance(1.5);


	// Populate list of existing PTGs:
	{
		mrpt::utils::registerAllPendingClasses();
		const std::vector<const mrpt::utils::TRuntimeClassId*> &lstClasses = mrpt::utils::getAllRegisteredClasses();
		for (size_t i=0;i<lstClasses.size();i++)
		{
			if (!lstClasses[i]->derivedFrom( "CParameterizedTrajectoryGenerator") ||
				!strcmpi(lstClasses[i]->className,"CParameterizedTrajectoryGenerator") )
				continue;
			cbPTGClass->AppendString( _U( lstClasses[i]->className ));
		}
		if (cbPTGClass->GetCount()>0)
			cbPTGClass->SetSelection(0);
	}
	{
		wxCommandEvent e;
		OncbPTGClassSelect(e);
	}

	this->Maximize();

	WX_END_TRY
}

ptgConfiguratorframe::~ptgConfiguratorframe()
{
    //(*Destroy(ptgConfiguratorframe)
    //*)
	if (ptg) {
		delete ptg;
		ptg=NULL;
	}
}


void ptgConfiguratorframe::OnAbout(wxCommandEvent& event)
{
}

void ptgConfiguratorframe::OnQuit(wxCommandEvent& event)
{
	Close();
}

void ptgConfiguratorframe::OnbtnReloadParamsClick(wxCommandEvent& event)
{
	WX_START_TRY;
	if (!ptg) return;

	ptg->deinitialize();

	const std::string sKeyPrefix = mrpt::format("PTG%d_", (int)edPTGIndex->GetValue() );
	const std::string sSection = "PTG_PARAMS";

	mrpt::utils::CConfigFileMemory  cfg;
	mrpt::utils::CConfigFilePrefixer cfp;
	cfp.bind(cfg);
	cfp.setPrefixes("", sKeyPrefix );

	cfg.setContent( std::string( edCfg->GetValue().mb_str() ) );

	ptg->loadFromConfigFile(cfp,sSection);

	ptg->initialize();

	rebuild3Dview();

	WX_END_TRY;
}


void ptgConfiguratorframe::OncbPTGClassSelect(wxCommandEvent& event)
{
	WX_START_TRY;

	const int sel = cbPTGClass->GetSelection();
	if (sel<0) return;

	const std::string sSelPTG = std::string( cbPTGClass->GetString(sel).mb_str() );

	if (ptg) {
		delete ptg;
	}

	// Factory:
	const mrpt::utils::TRuntimeClassId *classId = mrpt::utils::findRegisteredClass( sSelPTG );
	if (!classId) {
		THROW_EXCEPTION_CUSTOM_MSG1("[CreatePTG] No PTG named `%s` is registered!",sSelPTG.c_str());
	}

	ptg = dynamic_cast<mrpt::nav::CParameterizedTrajectoryGenerator*>( classId->createObject() );
	if (!ptg) {
		THROW_EXCEPTION_CUSTOM_MSG1("[CreatePTG] Object of type `%s` seems not to be a PTG!",sSelPTG.c_str());
	}

	// Set some common defaults:
	ptg->loadDefaultParams();

	dumpPTGcfgToTextBox();

	WX_END_TRY;

	// Update graphs:
	rebuild3Dview();
}

void ptgConfiguratorframe::rebuild3Dview()
{
	WX_START_TRY;

	const double refDist = ptg ? ptg->getRefDistance() : 10.0;

	gl_axis_WS->setAxisLimits(-refDist,-refDist,.0f, refDist,refDist,.0f);

	m_plot->Refresh();
	WX_END_TRY;
}

void ptgConfiguratorframe::OnedPTGIndexChange(wxSpinEvent& event)
{
	dumpPTGcfgToTextBox();
}

void ptgConfiguratorframe::dumpPTGcfgToTextBox()
{
	if (!ptg) return;

	// Wrapper to transparently add prefixes to all config keys:
	const std::string sKeyPrefix = mrpt::format("PTG%d_", (int)edPTGIndex->GetValue() );
	const std::string sSection = "PTG_PARAMS";

	mrpt::utils::CConfigFileMemory  cfg;
	mrpt::utils::CConfigFilePrefixer cfp;
	cfp.bind(cfg);
	cfp.setPrefixes("", sKeyPrefix );

	const int WN = 40, WV = 20;
	cfp.write(sSection,"Type", ptg->GetRuntimeClass()->className,   WN,WV, "PTG C++ class name");

	// Dump default params:
	ptg->saveToConfigFile(cfp,sSection);

	edCfg->SetValue( _U( cfg.getContent().c_str() ));
}
