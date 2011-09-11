/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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

#include "_DSceneViewerMain.h"
#include "CDlgCamTracking.h"
#include "CDlgPLYOptions.h"
#include <wx/app.h>

//(*InternalHeaders(_DSceneViewerFrame)
#include <wx/string.h>
#include <wx/intl.h>
#include <wx/bitmap.h>
#include <wx/icon.h>
#include <wx/image.h>
#include <wx/artprov.h>
//*)

#include "CDialogOptions.h"
#include "CAboutBox.h"

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

#if defined(__WXMSW__)
    const std::string             iniFileSect("CONF_WIN");
#elif defined(__UNIX__)
    const std::string             iniFileSect("CONF_LIN");
#endif

#include "imgs/icono_main.xpm"
#include "../wx-common/mrpt_logo.xpm"

#if !wxUSE_GLCANVAS
#error "OpenGL required: set wxUSE_GLCANVAS to 1 and rebuild wxWidgets"
#endif

#include <mrpt/gui/CMyGLCanvasBase.h>


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
    if (id == wxART_MAKE_ART_ID(MAIN_ICON))   return wxBitmap(icono_main_xpm);
    if (id == wxART_MAKE_ART_ID(IMG_MRPT_LOGO))  return wxBitmap(mrpt_logo_xpm);

    // Any wxWidgets icons not implemented here
    // will be provided by the default art provider.
    return wxNullBitmap;
}

// Used for feedback from the glcanvas component to its parent.
_DSceneViewerFrame		*theWindow=NULL;

#include <mrpt/utils.h>
#include <mrpt/system.h>
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::poses;
using namespace mrpt::opengl;
using namespace std;

#ifdef MRPT_OS_WINDOWS
	// Windows:
    #include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>
//#ifdef HAVE_FREEGLUT_EXT_H
//	#include <GL/freeglut_ext.h>
//#endif


// Critical section for updating the scene:
synch::CCriticalSection	critSec_UpdateScene;

// For TCP streams...
extern opengl::COpenGLScenePtr newOpenGLScene;
opengl::COpenGLScenePtr newOpenGLScene;	// TEMP!!

// The file to open (from cmd line), or an empty string
extern std::string     global_fileToOpen;
// The configuration file:
extern CConfigFile      *iniFile;


bool    isCapturing = false;
string  capturingDir=".";
int     captureCount=0;

string 	loadedFileName;


bool	freeCameraAlways = false;
bool	freeCameraAlwaysNoAzimuth = false;

bool		showFileNameInViewport = false;
int         delayBetweenAutoplay = 5;


wxLogWindow *logWin=NULL;


void saveLastUsedDirectoryToCfgFile(const std::string &fil)
{
    try
    {
        iniFile->write(iniFileSect,"LastDir", extractFileDirectory(fil) );
    }
    catch(std::exception &e)
    {
        wxMessageBox( _U(e.what()), _("Exception"), wxOK );
    }
}



void CMyGLCanvas::OnRenderError( const wxString &str )
{
	wxLogError(str);
	logWin->Show();
}

void CMyGLCanvas::OnPreRender()
{
	// Do we have to update the scene??
	synch::CCriticalSectionLocker   lock( &critSec_UpdateScene );
	if (newOpenGLScene)
	{
		m_openGLScene.clear_unique();
		m_openGLScene = newOpenGLScene;
		newOpenGLScene.clear_unique();
	}
}

void CMyGLCanvas::OnPostRenderSwapBuffers(double At, wxPaintDC &dc)
{
	// Capture window image?
	// --------------------------
	if ( isCapturing )
	{
		int w,h;
		dc.GetSize(&w, &h);

		// create a memory DC and bitmap to capture the DC
		wxMemoryDC memDC;
		wxBitmap memBmp(w, h);
		memDC.SelectObject(memBmp);
		memDC.Blit(0,0, w,h, &dc, 0,0);

		string fileName( format("%s/screenshot_%07i.png",capturingDir.c_str(),captureCount++) );

		memBmp.SaveFile(_U(fileName.c_str()),wxBITMAP_TYPE_PNG);
	}


	// Estimate FPS:
	// --------------------------
	double				estimatedFPS;
	static double 		meanEstimatedFPS = 1;

	if (At>0)
		estimatedFPS = 1/At;
	else	estimatedFPS = 0;

	meanEstimatedFPS = 0.8*meanEstimatedFPS + 0.2*estimatedFPS;

	string str = format("Center=(%.02f,%.02f,%.02f) Zoom:%.02f AZ=%.02f deg EL:%.02f deg",
						cameraPointingX,
						cameraPointingY,
						cameraPointingZ,
						cameraZoomDistance,
						cameraAzimuthDeg,
						cameraElevationDeg);
	theWindow->StatusBar1->SetStatusText( _U(str.c_str()) ,1);

	str = format("%.02f FPS", meanEstimatedFPS );
	theWindow->StatusBar1->SetStatusText( _U(str.c_str()) ,2);

	str = format("%u viewports", (unsigned)m_openGLScene->viewportsCount() );
	theWindow->StatusBar1->SetStatusText( _U(str.c_str()) ,3);
}


void CMyGLCanvas::OnPostRender()
{
	// Show filename over the screen??
	if (showFileNameInViewport)
	{
		renderTextBitmap( 20,20, extractFileName(loadedFileName).c_str() );
	}
}



void CMyGLCanvas::OnCharCustom( wxKeyEvent& event )
{
    long evkey = event.GetKeyCode();

    if (evkey==WXK_LEFT || evkey==WXK_RIGHT)
    {
    	try
    	{
    		CTicTac		tictac;
    		tictac.Tic();

			// Load a different file:

			// First, build a list of files in the directory:
			static CDirectoryExplorer::TFileInfoList lstFiles;
			static TTimeStamp lastUpdateOfList = INVALID_TIMESTAMP;
			TTimeStamp curTime = getCurrentTime();

			if ( lastUpdateOfList == INVALID_TIMESTAMP || timeDifference(lastUpdateOfList,curTime)>2)
			{
				CDirectoryExplorer::explore(
					extractFileDirectory(loadedFileName),	// path
					FILE_ATTRIB_ARCHIVE, // mask
					lstFiles );

				// Sort and remove non 3Dscene files:
				CDirectoryExplorer::sortByName(lstFiles);
				CDirectoryExplorer::filterByExtension(lstFiles,"3Dscene");

				lastUpdateOfList=curTime;
			}

			//cout << "Time to build file list: " << tictac.Tac() << endl;

			string curFileName = extractFileName(loadedFileName)+string(".")+extractFileExtension(loadedFileName);

			// Find the current file:
			CDirectoryExplorer::TFileInfoList::iterator it;
			size_t  i;
			for (i=0,it=lstFiles.begin();it!=lstFiles.end();it++,i++)
			{
				if (!os::_strcmpi(it->name.c_str(),curFileName.c_str()))
				{
					// Look for the desired file:
					if (evkey==WXK_LEFT)
					{
						if (i>0)  theWindow->loadFromFile( lstFiles[i-1].wholePath, true );
						return;
					}
					else
					{
						if (i<(lstFiles.size()-1))
							theWindow->loadFromFile( lstFiles[i+1].wholePath, true );
						else
						{
							// This was the last file
						}
						return;
					}
				}
			}
		}
    	catch (std::exception &)
    	{
    		//cerr << "*EXCEPTION* while determining next/previous file:\n" << e.what() << endl;
    	}
    	catch(...)
    	{
    	}
    }

    if (evkey==WXK_UP || evkey==WXK_DOWN)
    {
    	// Move the camera forward-backward:
    	// ...
    	//Refresh(false);
    }

}

//(*IdInit(_DSceneViewerFrame)
const long _DSceneViewerFrame::ID_MENUITEM1 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM2 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM5 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM7 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM6 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM20 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM19 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM22 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM21 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM12 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM18 = wxNewId();
const long _DSceneViewerFrame::idMenuQuit = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM4 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM3 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM15 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM17 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM16 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM11 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM9 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM8 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM10 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM14 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM13 = wxNewId();
const long _DSceneViewerFrame::idMenuAbout = wxNewId();
const long _DSceneViewerFrame::ID_STATUSBAR1 = wxNewId();
const long _DSceneViewerFrame::ID_TOOLBARITEM7 = wxNewId();
const long _DSceneViewerFrame::ID_TOOLBARITEM1 = wxNewId();
const long _DSceneViewerFrame::ID_TOOLBARITEM5 = wxNewId();
const long _DSceneViewerFrame::ID_TOOLBARITEM6 = wxNewId();
const long _DSceneViewerFrame::ID_TOOLBARITEM11 = wxNewId();
const long _DSceneViewerFrame::ID_TOOLBARITEM2 = wxNewId();
const long _DSceneViewerFrame::ID_TOOLBARITEM10 = wxNewId();
const long _DSceneViewerFrame::ID_TOOLBARITEM8 = wxNewId();
const long _DSceneViewerFrame::ID_TOOLBARITEM9 = wxNewId();
const long _DSceneViewerFrame::ID_TOOLBARITEM3 = wxNewId();
const long _DSceneViewerFrame::ID_TOOLBARITEM4 = wxNewId();
const long _DSceneViewerFrame::ID_TOOLBAR1 = wxNewId();
const long _DSceneViewerFrame::ID_TIMER1 = wxNewId();
//*)

const long _DSceneViewerFrame::ID_TRAVELLING_TIMER = wxNewId();

const long _DSceneViewerFrame::ID_TIMER_AUTOPLAY = wxNewId();


BEGIN_EVENT_TABLE(_DSceneViewerFrame,wxFrame)
    //(*EventTable(_DSceneViewerFrame)
    //*)
END_EVENT_TABLE()


_DSceneViewerFrame::_DSceneViewerFrame(wxWindow* parent,wxWindowID id)
	: m_travelling_start_time(INVALID_TIMESTAMP), maxv(0)
{
	//wxLogNull logQuiet;  // There're some issues with the toolbar icons.. just ignore them
	logWin = new wxLogWindow(this,wxT("Log window"),false);

	theWindow = this;

	// Load my custom icons:
#if wxCHECK_VERSION(2, 8, 0)
    wxArtProvider::Push(new MyArtProvider);
#else
    wxArtProvider::PushProvider(new MyArtProvider);
#endif

    //(*Initialize(_DSceneViewerFrame)
    wxMenuItem* MenuItem2;
    wxMenu* MenuItem15;
    wxMenuItem* MenuItem1;
    wxMenu* Menu1;
    wxMenuItem* MenuItem12;
    wxMenuItem* MenuItem3;
    wxMenuBar* MenuBar1;
    wxMenuItem* MenuItem4;
    wxMenuItem* MenuItem13;
    wxMenu* Menu2;

    Create(parent, id, _("3DSceneViewer - Part of the MRPT project - Jose Luis Blanco (C) 2005-2008"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("id"));
    SetClientSize(wxSize(672,539));
    {
    wxIcon FrameIcon;
    FrameIcon.CopyFromBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("MAIN_ICON")),wxART_FRAME_ICON));
    SetIcon(FrameIcon);
    }
    MenuBar1 = new wxMenuBar();
    Menu1 = new wxMenu();
    MenuItem3 = new wxMenuItem(Menu1, ID_MENUITEM1, _("New scene"), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(MenuItem3);
    MenuItem4 = new wxMenuItem(Menu1, ID_MENUITEM2, _("Open..."), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(MenuItem4);
    MenuItem7 = new wxMenuItem(Menu1, ID_MENUITEM5, _("Reload file\tF5"), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(MenuItem7);
    MenuItem9 = new wxMenuItem(Menu1, ID_MENUITEM7, _("Save..."), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(MenuItem9);
    Menu1->AppendSeparator();
    MenuItem18 = new wxMenu();
    MenuItem8 = new wxMenuItem(MenuItem18, ID_MENUITEM6, _("a 3DStudio object..."), wxEmptyString, wxITEM_NORMAL);
    MenuItem18->Append(MenuItem8);
    MenuItem19 = new wxMenuItem(MenuItem18, ID_MENUITEM20, _("a PLY point cloud..."), wxEmptyString, wxITEM_NORMAL);
    MenuItem18->Append(MenuItem19);
    Menu1->Append(ID_MENUITEM19, _("Import"), MenuItem18, wxEmptyString);
    MenuItem20 = new wxMenu();
    MenuItem21 = new wxMenuItem(MenuItem20, ID_MENUITEM22, _("point clouds to PLY file..."), wxEmptyString, wxITEM_NORMAL);
    MenuItem20->Append(MenuItem21);
    Menu1->Append(ID_MENUITEM21, _("Export"), MenuItem20, wxEmptyString);
    Menu1->AppendSeparator();
    MenuItem14 = new wxMenuItem(Menu1, ID_MENUITEM12, _("Take snapshot...\tF2"), _("Saves the current window image to a file"), wxITEM_NORMAL);
    Menu1->Append(MenuItem14);
    mnuSceneStats = new wxMenuItem(Menu1, ID_MENUITEM18, _("Scene stats"), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(mnuSceneStats);
    Menu1->AppendSeparator();
    MenuItem1 = new wxMenuItem(Menu1, idMenuQuit, _("Quit\tAlt-F4"), _("Quit the application"), wxITEM_NORMAL);
    Menu1->Append(MenuItem1);
    MenuBar1->Append(Menu1, _("&File"));
    Menu3 = new wxMenu();
    MenuItem6 = new wxMenuItem(Menu3, ID_MENUITEM4, _("Background color..."), wxEmptyString, wxITEM_NORMAL);
    Menu3->Append(MenuItem6);
    MenuItem5 = new wxMenuItem(Menu3, ID_MENUITEM3, _("Options..."), wxEmptyString, wxITEM_NORMAL);
    Menu3->Append(MenuItem5);
    Menu3->AppendSeparator();
    MenuItem17 = new wxMenu();
    mnuItemShowCloudOctrees = new wxMenuItem(MenuItem17, ID_MENUITEM15, _("Show/hide bounding boxes"), wxEmptyString, wxITEM_CHECK);
    MenuItem17->Append(mnuItemShowCloudOctrees);
    mnuItemChangeMaxPointsPerOctreeNode = new wxMenuItem(MenuItem17, ID_MENUITEM17, _("Change octree parameters..."), wxEmptyString, wxITEM_NORMAL);
    MenuItem17->Append(mnuItemChangeMaxPointsPerOctreeNode);
    Menu3->Append(ID_MENUITEM16, _("Point clouds octrees..."), MenuItem17, wxEmptyString);
    Menu3->AppendSeparator();
    MenuItem13 = new wxMenuItem(Menu3, ID_MENUITEM11, _("Delete all objects"), wxEmptyString, wxITEM_NORMAL);
    Menu3->Append(MenuItem13);
    MenuItem11 = new wxMenu();
    MenuItem12 = new wxMenuItem(MenuItem11, ID_MENUITEM9, _("Simple robot model"), wxEmptyString, wxITEM_NORMAL);
    MenuItem11->Append(MenuItem12);
    Menu3->Append(ID_MENUITEM8, _("Insert stock object"), MenuItem11, wxEmptyString);
    Menu3->AppendSeparator();
    MenuItem15 = new wxMenu();
    MenuItem10 = new wxMenuItem(MenuItem15, ID_MENUITEM10, _("Circular"), wxEmptyString, wxITEM_NORMAL);
    MenuItem15->Append(MenuItem10);
    MenuItem16 = new wxMenuItem(MenuItem15, ID_MENUITEM14, _("Arbitrary path..."), wxEmptyString, wxITEM_NORMAL);
    MenuItem15->Append(MenuItem16);
    Menu3->Append(ID_MENUITEM13, _("Camera tracking"), MenuItem15, wxEmptyString);
    MenuBar1->Append(Menu3, _("Tools"));
    Menu2 = new wxMenu();
    MenuItem2 = new wxMenuItem(Menu2, idMenuAbout, _("About\tF1"), _("Show info about this application"), wxITEM_NORMAL);
    Menu2->Append(MenuItem2);
    MenuBar1->Append(Menu2, _("Help"));
    SetMenuBar(MenuBar1);
    StatusBar1 = new wxStatusBar(this, ID_STATUSBAR1, 0, _T("ID_STATUSBAR1"));
    int __wxStatusBarWidths_1[4] = { -10, -10, -4, -5 };
    int __wxStatusBarStyles_1[4] = { wxSB_NORMAL, wxSB_NORMAL, wxSB_NORMAL, wxSB_NORMAL };
    StatusBar1->SetFieldsCount(4,__wxStatusBarWidths_1);
    StatusBar1->SetStatusStyles(4,__wxStatusBarStyles_1);
    SetStatusBar(StatusBar1);
    ToolBar1 = new wxToolBar(this, ID_TOOLBAR1, wxDefaultPosition, wxDefaultSize, wxTB_FLAT|wxTB_VERTICAL|wxTB_TEXT|wxNO_BORDER, _T("ID_TOOLBAR1"));
    ToolBarItem1 = ToolBar1->AddTool(ID_TOOLBARITEM7, _("New"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_NORMAL_FILE")),wxART_TOOLBAR), wxNullBitmap, wxITEM_NORMAL, _("New scene"), _("New scene"));
    ToolBarItem2 = ToolBar1->AddTool(ID_TOOLBARITEM1, _("Open"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FILE_OPEN")),wxART_TOOLBAR), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FILE_OPEN")),wxART_TOOLBAR), wxITEM_NORMAL, _("Open a \"3Dscene\" file..."), _("Open a \"3Dscene\" file..."));
    ToolBar1->AddSeparator();
    ToolBarItem3 = ToolBar1->AddTool(ID_TOOLBARITEM5, _("Previous"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_GO_BACK")),wxART_TOOLBAR), wxNullBitmap, wxITEM_NORMAL, _("Open previous file in directory..."), _("Open previous file in directory..."));
    ToolBarItem4 = ToolBar1->AddTool(ID_TOOLBARITEM6, _("Next"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_GO_FORWARD")),wxART_TOOLBAR), wxNullBitmap, wxITEM_NORMAL, _("Open next file in directory..."), _("Open next file in directory..."));
    ToolBarItem5 = ToolBar1->AddTool(ID_TOOLBARITEM11, _("Reload"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_REDO")),wxART_MAKE_CLIENT_ID_FROM_STR(wxString(_T("wxART_REDO")))), wxNullBitmap, wxITEM_NORMAL, _("Reload current file (F5)"), wxEmptyString);
    ToolBar1->AddSeparator();
    ToolBarItem6 = ToolBar1->AddTool(ID_TOOLBARITEM2, _("Options"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FIND")),wxART_TOOLBAR), wxNullBitmap, wxITEM_NORMAL, _("View the options of 3DSceneViewer"), _("View the options of 3DSceneViewer"));
    btnOrtho = ToolBar1->AddTool(ID_TOOLBARITEM10, _("Ortho"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_TICK_MARK")),wxART_TOOLBAR), wxNullBitmap, wxITEM_CHECK, _("Toogle projective/orthogonal view..."), _("Toogle projective/orthogonal view"));
    btnAutoplay = ToolBar1->AddTool(ID_TOOLBARITEM8, _("Autoplay"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_REMOVABLE")),wxART_TOOLBAR), wxNullBitmap, wxITEM_CHECK, _("Play a sequence of files"), _("Play consecutive files in the current directory"));
    btnRecord = ToolBar1->AddTool(ID_TOOLBARITEM9, _("Capture..."), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_HARDDISK")),wxART_TOOLBAR), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_HARDDISK")),wxART_TOOLBAR), wxITEM_CHECK, _("Start capturing screenshots to a directory..."), _("Start capturing screenshots to a directory..."));
    ToolBar1->AddSeparator();
    ToolBarItem10 = ToolBar1->AddTool(ID_TOOLBARITEM3, _("About..."), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_HELP_BOOK")),wxART_TOOLBAR), wxNullBitmap, wxITEM_NORMAL, _("About..."), _("Show program information"));
    ToolBarItem11 = ToolBar1->AddTool(ID_TOOLBARITEM4, _("Quit"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_QUIT")),wxART_TOOLBAR), wxNullBitmap, wxITEM_NORMAL, _("Close the application"), _("Close the application"));
    ToolBar1->Realize();
    SetToolBar(ToolBar1);
    timLoadFileCmdLine.SetOwner(this, ID_TIMER1);
    timLoadFileCmdLine.Start(50, true);
    Center();

    Connect(ID_MENUITEM1,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&_DSceneViewerFrame::OnNewScene);
    Connect(ID_MENUITEM2,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&_DSceneViewerFrame::OnOpenFile);
    Connect(ID_MENUITEM5,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&_DSceneViewerFrame::OnReload);
    Connect(ID_MENUITEM7,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&_DSceneViewerFrame::OnMenuSave);
    Connect(ID_MENUITEM6,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&_DSceneViewerFrame::OnInsert3DS);
    Connect(ID_MENUITEM20,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&_DSceneViewerFrame::OnMenuItemImportPLYPointCloud);
    Connect(ID_MENUITEM22,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&_DSceneViewerFrame::OnMenuItemExportPointsPLY);
    Connect(ID_MENUITEM12,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&_DSceneViewerFrame::OnMenuItem14Selected);
    Connect(ID_MENUITEM18,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&_DSceneViewerFrame::OnmnuSceneStatsSelected);
    Connect(idMenuQuit,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&_DSceneViewerFrame::OnQuit);
    Connect(ID_MENUITEM4,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&_DSceneViewerFrame::OnMenuBackColor);
    Connect(ID_MENUITEM3,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&_DSceneViewerFrame::OnMenuOptions);
    Connect(ID_MENUITEM15,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&_DSceneViewerFrame::OnmnuItemShowCloudOctreesSelected);
    Connect(ID_MENUITEM17,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&_DSceneViewerFrame::OnmnuItemChangeMaxPointsPerOctreeNodeSelected);
    Connect(ID_MENUITEM11,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&_DSceneViewerFrame::OnMenuDeleteAll);
    Connect(ID_MENUITEM9,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&_DSceneViewerFrame::OnMenuAddSICK);
    Connect(ID_MENUITEM10,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&_DSceneViewerFrame::OnStartCameraTravelling);
    Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&_DSceneViewerFrame::OnAbout);
    Connect(ID_TOOLBARITEM7,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&_DSceneViewerFrame::OnNewScene);
    Connect(ID_TOOLBARITEM1,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&_DSceneViewerFrame::OnOpenFile);
    Connect(ID_TOOLBARITEM5,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&_DSceneViewerFrame::OnPrevious);
    Connect(ID_TOOLBARITEM6,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&_DSceneViewerFrame::OnNext);
    Connect(ID_TOOLBARITEM11,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&_DSceneViewerFrame::OnReload);
    Connect(ID_TOOLBARITEM2,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&_DSceneViewerFrame::OnMenuOptions);
    Connect(ID_TOOLBARITEM10,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&_DSceneViewerFrame::OnbtnOrthoClicked);
    Connect(ID_TOOLBARITEM8,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&_DSceneViewerFrame::OnbtnAutoplayClicked);
    Connect(ID_TOOLBARITEM9,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&_DSceneViewerFrame::OnBtnRecordClicked);
    Connect(ID_TOOLBARITEM3,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&_DSceneViewerFrame::OnAbout);
    Connect(ID_TOOLBARITEM4,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&_DSceneViewerFrame::OnQuit);
    Connect(ID_TIMER1,wxEVT_TIMER,(wxObjectEventFunction)&_DSceneViewerFrame::OntimLoadFileCmdLineTrigger);
    //*)

    Connect(ID_MENUITEM14,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&_DSceneViewerFrame::OnMenuCameraTrackingArbitrary);


    Connect(ID_TIMER_AUTOPLAY,wxEVT_TIMER,(wxObjectEventFunction)&_DSceneViewerFrame::OntimAutoplay);
    Connect(ID_TRAVELLING_TIMER,wxEVT_TIMER,(wxObjectEventFunction)&_DSceneViewerFrame::OnTravellingTrigger);

    // Create the wxCanvas object:
    m_canvas = new CMyGLCanvas( this, wxID_ANY, wxDefaultPosition, wxDefaultSize );

	// Load an empty scene:
    wxCommandEvent dummEvent;
    OnNewScene(dummEvent);


    m_autoplayTimer = new wxTimer(this, ID_TIMER_AUTOPLAY );

    m_tTravelling.SetOwner(this, ID_TRAVELLING_TIMER);

    m_dlg_tracking= new CDlgCamTracking(this);


    Maximize();
}

_DSceneViewerFrame::~_DSceneViewerFrame()
{
	theWindow = NULL;

	delete m_autoplayTimer;

    //(*Destroy(_DSceneViewerFrame)
    //*)
}

void _DSceneViewerFrame::OnQuit(wxCommandEvent& event)
{
    Close();
}

void _DSceneViewerFrame::OnAbout(wxCommandEvent& event)
{
	CAboutBox  dialog(this);
	dialog.ShowModal();
}

void _DSceneViewerFrame::OnNewScene(wxCommandEvent& event)
{
	m_canvas->m_openGLScene->clear();

	// Default file name:
	loadedFileName = "no-name.3Dscene";
	updateTitle();

    {
		mrpt::opengl::CGridPlaneXYPtr obj = mrpt::opengl::CGridPlaneXY::Create( -50,50,-50,50,0,1 );
	    obj->setColor(0.3,0.3,0.3);
	    m_canvas->m_openGLScene->insert( obj );
    }

    m_canvas->cameraPointingX 	= 0;
    m_canvas->cameraPointingY 	= 0;
    m_canvas->cameraPointingZ 	= 0;

    m_canvas->cameraZoomDistance	= 20;

    m_canvas->cameraElevationDeg 	= 45;
    m_canvas->cameraAzimuthDeg	= 45;

    m_canvas->m_openGLScene->insert( stock_objects::CornerXYZ() );

	// Add a clone viewport:
	COpenGLViewportPtr vi= m_canvas->m_openGLScene->createViewport("clone");
	vi->setViewportPosition(0.05,0.05,0.2,0.2);
	vi->setCloneView("main");
	vi->setCloneCamera(true);
	vi->setBorderSize(1);

    Refresh(false);
}

void _DSceneViewerFrame::OnOpenFile(wxCommandEvent& event)
{
    wxString caption = wxT("Choose a file to open");
    wxString wildcard = wxT("3D scene files (*.3Dscene)|*.3Dscene|All files (*.*)|*.*");

    wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );

    wxString defaultFilename = wxT("");
    wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_OPEN | wxFD_FILE_MUST_EXIST );

    if (dialog.ShowModal() == wxID_OK)
    {
        wxString fileName = dialog.GetPath();

        loadFromFile( std::string(fileName.mb_str()) );
    }

}

void _DSceneViewerFrame::loadFromFile( const std::string &fil, bool isInASequence )
{
    try
    {
        // Save the path
		saveLastUsedDirectoryToCfgFile(fil);

		//static float	old_cam_pX,old_cam_pY,old_cam_pZ,old_cam_d,old_cam_az,old_cam_el;
		static bool first = true;

		if (first)
		{
			first = false;
			//old_cam_pX=old_cam_pY=old_cam_pZ=old_cam_d=old_cam_az=old_cam_el=0.0f;
		}

		CFileGZInputStream	   	f( fil);

        static utils::CTicTac		tictac;
        {
            synch::CCriticalSectionLocker   lock(&critSec_UpdateScene);

            tictac.Tic();

            m_canvas->m_openGLScene->clear();
            f >> m_canvas->m_openGLScene;
        }

		double timeToLoad = tictac.Tac();

		if (m_canvas->m_openGLScene->viewportsCount()==0  ||
			!m_canvas->m_openGLScene->getViewport("main") ||
			m_canvas->m_openGLScene->getViewport("main")->size()==0
			)
		{
			wxMessageBox( _("File is empty or format unrecognized."), _("Warning"), wxOK, this);
			theWindow->ToolBar1->ToggleTool(ID_TOOLBARITEM8, false);
		}


        // Change the camera if necesary:
		if ( m_canvas->m_openGLScene->followCamera() )
		{
			COpenGLViewportPtr view = m_canvas->m_openGLScene->getViewport("main");
			if (!view)
				THROW_EXCEPTION("Fatal error: there is no 'main' viewport in the 3D scene!");

			CCameraPtr cam = m_canvas->m_openGLScene->getByClass<CCamera>();

			bool  camIsCCameraObj = cam.present();
			if ( !camIsCCameraObj )
				cam = CCameraPtr( new CCamera( view->getCamera() ) );

			m_canvas->cameraPointingX = cam->getPointingAtX();
			m_canvas->cameraPointingY = cam->getPointingAtY();
			m_canvas->cameraPointingZ = cam->getPointingAtZ();

			// If it's not loaded thru arrow keys or a timed sequence, take camera:
			if (!isInASequence || !freeCameraAlways || !freeCameraAlwaysNoAzimuth)
			{
				if (!freeCameraAlwaysNoAzimuth)
				{
					m_canvas->cameraZoomDistance = cam->getZoomDistance();
					m_canvas->cameraElevationDeg = cam->getElevationDegrees();
				}
				m_canvas->cameraAzimuthDeg = cam->getAzimuthDegrees();
			}

			// Remove the camera from the object:
			if ( camIsCCameraObj )
				m_canvas->m_openGLScene->removeObject( cam );
		}

        loadedFileName = fil;

        // Set the file name as window title:
        updateTitle();

		theWindow->StatusBar1->SetStatusText( _U(format("File loaded in %.03fs", timeToLoad ).c_str()) ,0);

        Refresh(false);
    }
    catch(std::exception &e)
    {
        std::cerr << e.what() << std::endl;
		theWindow->ToolBar1->ToggleTool(ID_TOOLBARITEM8, false);
        wxMessageBox( _U(e.what()), _("Exception"), wxOK, this);
    }
    catch(...)
    {
		theWindow->ToolBar1->ToggleTool(ID_TOOLBARITEM8, false);
        wxMessageBox( _("Runtime error!"), _("Exception"), wxOK, this);
    }
}

void _DSceneViewerFrame::updateTitle()
{
	SetTitle(_U(format("3DSceneViewer - Part of the MRPT project [%s]",
		( extractFileName(loadedFileName) + string(".") + extractFileExtension(loadedFileName)
		).c_str() ).c_str() ));
}

void _DSceneViewerFrame::OntimLoadFileCmdLineTrigger(wxTimerEvent& event)
{
    // Open file if passed by the command line:
    if (global_fileToOpen.size())
        loadFromFile( global_fileToOpen );
}


void _DSceneViewerFrame::OnbtnAutoplayClicked(wxCommandEvent& event)
{
	if ( btnAutoplay->IsToggled() )
		m_autoplayTimer->Start(5,true); // One-shot:
}

void _DSceneViewerFrame::OntimAutoplay(wxTimerEvent& event)
{
	// Load next file:
	wxKeyEvent  dummyEvent;
	dummyEvent.m_keyCode = WXK_RIGHT;
	m_canvas->OnCharCustom( dummyEvent );

	// Continue?
	if ( btnAutoplay->IsToggled() )
		m_autoplayTimer->Start( delayBetweenAutoplay  ,true); // One-shot:
}

void _DSceneViewerFrame::OnMenuBackColor(wxCommandEvent& event)
{
	wxColourData	colourData;
	wxColour			color;
	color.Set( (int)(255*m_canvas->clearColorR),(int)(255*m_canvas->clearColorG),(int)(255*m_canvas->clearColorB) );

	colourData.SetColour( color );
	colourData.SetChooseFull(true);

	wxColourDialog  colDial(this, &colourData);

	if (wxID_OK==colDial.ShowModal())
	{
		wxColour col = colDial.GetColourData().GetColour();
		m_canvas->clearColorR = col.Red()/255.0f;
		m_canvas->clearColorG = col.Green()/255.0f;
		m_canvas->clearColorB = col.Blue()/255.0f;

		int w,h;
		m_canvas->GetSize(&w,&h);
		RefreshRect( wxRect(0,0,w,h), false );
	}
}

// See options:
void _DSceneViewerFrame::OnMenuOptions(wxCommandEvent& event)
{
	CDialogOptions	dial(this);

	// Put current options:
	dial.cbViewFileName->SetValue( showFileNameInViewport );
	dial.cbFreeCamera->SetValue( freeCameraAlways );
	dial.cbFreeCameraNoAzimuth->SetValue ( freeCameraAlwaysNoAzimuth );
	dial.edDelay->SetValue( delayBetweenAutoplay );

	// Show
	if (dial.ShowModal() == wxID_OK)
	{
		// If OK, get new options:
		showFileNameInViewport = dial.cbViewFileName->GetValue( );
		freeCameraAlways = dial.cbFreeCamera->GetValue( );
		freeCameraAlwaysNoAzimuth = dial.cbFreeCameraNoAzimuth->GetValue();
		delayBetweenAutoplay = dial.edDelay->GetValue( );
		m_canvas->Refresh(false);
	}
}

void _DSceneViewerFrame::OnPrevious(wxCommandEvent& event)
{
    wxKeyEvent  evn;
    evn.m_keyCode = WXK_LEFT;
    m_canvas->OnChar(evn);
}

void _DSceneViewerFrame::OnNext(wxCommandEvent& event)
{
    wxKeyEvent  evn;
    evn.m_keyCode = WXK_RIGHT;
    m_canvas->OnChar(evn);
}

void _DSceneViewerFrame::OnClose(wxCloseEvent& event)
{
}

void _DSceneViewerFrame::OnBtnRecordClicked(wxCommandEvent& event)
{
	if ( btnRecord->IsToggled() )
	{
	    // Starting recording:
        wxDirDialog dirDialog(
            this,
            _("Choose the directory where to save the screenshots:"),
            _U( extractFileDirectory(loadedFileName).c_str() ), 0, wxDefaultPosition );

        if (dirDialog.ShowModal()==wxID_OK)
        {
            capturingDir = string( dirDialog.GetPath().mb_str() );
            isCapturing = true;
            captureCount=0;
        }
        else
        {
            btnRecord->SetToggle(false);
        }
	}
	else
	{
        isCapturing = false;
	}
}

void _DSceneViewerFrame::OnbtnOrthoClicked(wxCommandEvent& event)
{
	bool ortho = btnOrtho->IsToggled();
	m_canvas->cameraIsProjective = !ortho;

    m_canvas->Refresh(false);
}

void _DSceneViewerFrame::OnReload(wxCommandEvent& event)
{
	if ( fileExists(loadedFileName) )
		loadFromFile( loadedFileName );
}

void _DSceneViewerFrame::OnInsert3DS(wxCommandEvent& event)
{
	try
	{
		wxString caption = wxT("Choose a file to import");
		wxString wildcard = wxT("3DS files (*.3DS, *.3DS.gz)|*.3DS;*.3DS.gz;*.3ds;*.3ds.gz|All files (*.*)|*.*");
		wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
		wxString defaultFilename = wxT("");

		wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_OPEN | wxFD_FILE_MUST_EXIST );

		if (dialog.ShowModal() != wxID_OK)
			return;

		wxString fileName = dialog.GetPath();
		std::string	fil = string(fileName.mb_str());

		saveLastUsedDirectoryToCfgFile(fil);

		mrpt::opengl::C3DSScenePtr	obj3D = mrpt::opengl::C3DSScene::Create();
		obj3D->loadFrom3DSFile( fil );
		m_canvas->m_openGLScene->insert( obj3D );

		m_canvas->Refresh();
    }
    catch(std::exception &e)
    {
        std::cerr << e.what() << std::endl;
		ToolBar1->ToggleTool(ID_TOOLBARITEM8, false);
        wxMessageBox( _U(e.what()), _("Exception"), wxOK, this);
    }
    catch(...)
    {
		ToolBar1->ToggleTool(ID_TOOLBARITEM8, false);
        wxMessageBox( _("Runtime error!"), _("Exception"), wxOK, this);
    }
}

void _DSceneViewerFrame::OnMenuSave(wxCommandEvent& event)
{
	try
	{
		wxString caption = wxT("Save scene to file");
		wxString wildcard = wxT("3Dscene files (*.3Dscene)|*.3Dscene|All files (*.*)|*.*");
		wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
		wxString defaultFilename = wxT("");

		wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT  );

		if (dialog.ShowModal() != wxID_OK)
			return;

		wxString fileName = dialog.GetPath();

		CFileGZOutputStream	fo( string( fileName.mb_str() ) );
		fo << *m_canvas->m_openGLScene;

    }
    catch(std::exception &e)
    {
        std::cerr << e.what() << std::endl;
		ToolBar1->ToggleTool(ID_TOOLBARITEM8, false);
        wxMessageBox( _U(e.what()), _("Exception"), wxOK, this);
    }
    catch(...)
    {
		ToolBar1->ToggleTool(ID_TOOLBARITEM8, false);
        wxMessageBox( _("Runtime error!"), _("Exception"), wxOK, this);
    }
}

void _DSceneViewerFrame::OnTravellingTrigger(wxTimerEvent& event)
{
	try
	{
		if ((m_canvas->m_openGLScene->viewportsCount() == 0) ||
			!m_canvas->m_openGLScene->getViewport("main") ||
			(m_canvas->m_openGLScene->getViewport("main")->size() == 0))
		{
			wxMessageBox( _("Canvas is empty"), _("Warning"), wxOK, this);
		}
		else
		{
			// Change the camera
			COpenGLViewportPtr view = m_canvas->m_openGLScene->getViewport("main");

			if (!view)
				THROW_EXCEPTION("Fatal error: there is no 'main' viewport in the 3D scene!");

			// Do (in Spanish) "travelling":
			if (m_travelling_is_arbitrary)
			{
				// ===============================
				// Arbitrary camera tracking
				// ===============================
				if (m_dlg_tracking->m_poses.empty())
				{
					m_tTravelling.Stop();
					return;
				}

				TTimeStamp t= m_dlg_tracking->m_poses.begin()->first + (mrpt::system::now()-m_travelling_start_time);
				bool valid=false;
				CPose3D p;
				while (!valid && t<m_dlg_tracking->m_poses.rbegin()->first)
				{
					m_dlg_tracking->m_poses.interpolate(t,p,valid);
					if (!valid)
					{
						TTimeStamp At = secondsToTimestamp(0.1);
						t+=At;
						m_travelling_start_time-=At;
					}
				}

				if (valid)
				{
					m_canvas->cameraPointingX = p.x();
					m_canvas->cameraPointingY = p.y();
					m_canvas->cameraPointingZ = p.z();

				}
				else
				{
					// end of path:
					m_tTravelling.Stop();
					return;
				}
			}
			else
			{
				// ===============================
				// Circular camera tracking
				// ===============================
				double step = atof(iniFile->read_string("Spherical travelling","Step","5").c_str());

				if ((m_canvas->cameraAzimuthDeg + step) < maxv)
					m_canvas->cameraAzimuthDeg += step/10;
				else
					m_tTravelling.Stop();

			}

			Refresh(false);
		}
    }
    catch(std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        wxMessageBox( _U(e.what()), _("Exception"), wxOK, this);
    }
    catch(...)
    {
        wxMessageBox( _("Runtime error!"), _("Exception"), wxOK, this);
    }
}

void _DSceneViewerFrame::OnStartCameraTravelling(wxCommandEvent& event)
{
	try
	{
		if ((m_canvas->m_openGLScene->viewportsCount() == 0) ||
			!m_canvas->m_openGLScene->getViewport("main") ||
			(m_canvas->m_openGLScene->getViewport("main")->size() == 0))
		{
			wxMessageBox( _("Canvas is empty"), _("Warning"), wxOK, this);
		}
		else
		{
			// Change the camera
			COpenGLViewportPtr view = m_canvas->m_openGLScene->getViewport("main");

			if (!view)
				THROW_EXCEPTION("Fatal error: there is no 'main' viewport in the 3D scene!");

			double target_x = atof(iniFile->read_string("Spherical travelling","X","0").c_str());
			double target_y = atof(iniFile->read_string("Spherical travelling","Y","0").c_str());
			double target_z = atof(iniFile->read_string("Spherical travelling","Z","0").c_str());

			double zoom      = atof(iniFile->read_string("Spherical travelling","Zoom distance","25").c_str());
			double elevation = atof(iniFile->read_string("Spherical travelling","Elevation degrees","15").c_str());
			double azimuth   = atof(iniFile->read_string("Spherical travelling","Azimuth degrees","90").c_str());

			double min_value = atof(iniFile->read_string("Spherical travelling","Min value","90").c_str());
			double max_value = atof(iniFile->read_string("Spherical travelling","Max value","90").c_str());

			m_canvas->cameraPointingX = target_x;
			m_canvas->cameraPointingY = target_y;
			m_canvas->cameraPointingZ = target_z;
			m_canvas->cameraZoomDistance = zoom;
			m_canvas->cameraElevationDeg = elevation;
			m_canvas->cameraAzimuthDeg   = azimuth - min_value;

			maxv = azimuth + max_value;

			m_travelling_is_arbitrary=false; // Circular
			m_tTravelling.Start(100);

			Refresh(false);
		}
    }
    catch(std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        wxMessageBox( _U(e.what()), _("Exception"), wxOK, this);
    }
    catch(...)
    {
        wxMessageBox( _("Runtime error!"), _("Exception"), wxOK, this);
    }
}


void _DSceneViewerFrame::OnClose1(wxCloseEvent& event)
{
}

void _DSceneViewerFrame::OnMenuAddSICK(wxCommandEvent& event)
{
	try
	{
		mrpt::opengl::CSetOfObjectsPtr obj = mrpt::opengl::stock_objects::RobotPioneer();
		m_canvas->m_openGLScene->insert( obj );
		m_canvas->Refresh();
    }
    catch(std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        wxMessageBox( _U(e.what()), _("Exception"), wxOK, this);
    }
    catch(...)
    {
        wxMessageBox( _("Runtime error!"), _("Exception"), wxOK, this);
    }
}

void _DSceneViewerFrame::OnMenuDeleteAll(wxCommandEvent& event)
{
	m_canvas->m_openGLScene->clear();
	m_canvas->Refresh();
}

void _DSceneViewerFrame::OnMenuItem14Selected(wxCommandEvent& event)
{
	m_canvas->Refresh();
	wxApp::GetInstance()->Yield();


	wxClientDC  dc(this);

	int w,h;
	dc.GetSize(&w, &h);

	// create a memory DC and bitmap to capture the DC
	wxMemoryDC memDC;
	wxBitmap memBmp(w, h);
	memDC.SelectObject(memBmp);
	memDC.Blit(0,0, w,h, &dc, 0,0);

	// Save:
	wxString caption = wxT("Save snapshot to file");
	wxString wildcard = wxT("Image files (*.png)|*.png|All files (*.*)|*.*");
	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
	wxString defaultFilename = wxT("snapshot.png");

	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT  );

	if (dialog.ShowModal() != wxID_OK)
		return;

	memBmp.SaveFile( dialog.GetPath(), wxBITMAP_TYPE_PNG);
}


void _DSceneViewerFrame::OnMenuCameraTrackingArbitrary(wxCommandEvent& event)
{
	m_dlg_tracking->Show();
}

void _DSceneViewerFrame::OnmnuItemChangeMaxPointsPerOctreeNodeSelected(wxCommandEvent& event)
{
	wxString sRet1 = wxGetTextFromUser(
		_("Max. number of points in an octree node before split:"),
		_("Enter new value"),
		wxString::Format(_("%e"),(double)mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE),
		this);

	wxString sRet2 = wxGetTextFromUser(
		_("Max. density of points in each octree (points/pixel^2):"),
		_("Enter new value"),
		wxString::Format(_("%e"),(double)mrpt::global_settings::OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL),
		this);

	double N1,N2;
	if (sRet1.ToDouble(&N1) && sRet2.ToDouble(&N2))
	{
		mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE = N1;
		mrpt::global_settings::OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL = N2;

		// Redo the octrees:
		clear_all_octrees_in_scene();
		Refresh(false);

		wxCommandEvent dumm;  // Redraw bounding-boxes:
		OnmnuItemShowCloudOctreesSelected(dumm);
	}
	else
		wxMessageBox(_("Invalid number!"));
}

void func_clear_octrees(const mrpt::opengl::CRenderizablePtr &o)
{
	if (IS_CLASS(o,CPointCloud))
	{
		CPointCloudPtr obj = CPointCloudPtr(o);
		obj->octree_mark_as_outdated();
	}
	else
	if (IS_CLASS(o,CPointCloudColoured))
	{
		CPointCloudColouredPtr obj = CPointCloudColouredPtr(o);
		obj->octree_mark_as_outdated();
	}
}

void _DSceneViewerFrame::clear_all_octrees_in_scene()
{
	{
		mrpt::synch::CCriticalSectionLocker lock(&critSec_UpdateScene);
		m_canvas->m_openGLScene->visitAllObjects( &func_clear_octrees );
	}
}

struct TSceneStats
{
	void clear()
	{
		nPoints=0;
		nObjects=0;
		nOctreeVisible=0;
		nOctreeTotal=0;
	}

	size_t  nPoints;
	size_t  nObjects;
	size_t  nOctreeVisible,nOctreeTotal;
};

TSceneStats sceneStats;

void func_gather_stats(const mrpt::opengl::CRenderizablePtr &o)
{
	sceneStats.nObjects++;

	if (IS_CLASS(o,CPointCloud))
	{
		CPointCloudPtr obj = CPointCloudPtr(o);
		sceneStats.nPoints +=obj->size();
		sceneStats.nOctreeVisible+=obj->octree_get_visible_nodes();
		sceneStats.nOctreeTotal+=obj->octree_get_node_count();
	}
	else
	if (IS_CLASS(o,CPointCloudColoured))
	{
		CPointCloudColouredPtr obj = CPointCloudColouredPtr(o);
		sceneStats.nPoints +=obj->size();
		sceneStats.nOctreeVisible+=obj->octree_get_visible_nodes();
		sceneStats.nOctreeTotal+=obj->octree_get_node_count();
	}
}

// Gather stats on the scene:
void _DSceneViewerFrame::OnmnuSceneStatsSelected(wxCommandEvent& event)
{
    try
    {
		wxBusyCursor wait;
		sceneStats.clear();

		{
			mrpt::synch::CCriticalSectionLocker lock(&critSec_UpdateScene);
			m_canvas->m_openGLScene->visitAllObjects( &func_gather_stats );
		}

		std::stringstream  ss;
		ss << "Number of objects: " << sceneStats.nObjects << endl
		   << format("Overall points (in point clouds): %e",double(sceneStats.nPoints)) << endl
		   << "Total octree nodes   (in point clouds): " << sceneStats.nOctreeTotal << endl
		   << "Visible octree nodes (in point clouds): " << sceneStats.nOctreeVisible << endl;

		wxMessageBox(_U(ss.str().c_str()), _("Scene statistics"));
    }
    catch(std::exception &e)
    {
        wxMessageBox( _U(e.what()), _("Exception"), wxOK, this);
    }
}


static const std::string name_octrees_bb_globj = "__3dsceneviewer_gl_octree_bb__";
CSetOfObjectsPtr aux_gl_octrees_bb;

void func_get_octbb(const mrpt::opengl::CRenderizablePtr &o)
{
	if (IS_CLASS(o,CPointCloud))
	{
		CPointCloudPtr obj = CPointCloudPtr(o);
		CSetOfObjectsPtr new_bb = CSetOfObjects::Create();
		obj->octree_get_graphics_boundingboxes(*new_bb);
		aux_gl_octrees_bb->insert(new_bb );
	}
	else
	if (IS_CLASS(o,CPointCloudColoured))
	{
		CPointCloudColouredPtr obj = CPointCloudColouredPtr(o);
		CSetOfObjectsPtr new_bb = CSetOfObjects::Create();
		obj->octree_get_graphics_boundingboxes(*new_bb);
		aux_gl_octrees_bb->insert(new_bb );
	}
}


// Show/hide the octree bounding boxes of the point clouds:
void _DSceneViewerFrame::OnmnuItemShowCloudOctreesSelected(wxCommandEvent& event)
{
	const bool show_hide = mnuItemShowCloudOctrees->IsChecked();

    try
    {
		wxBusyCursor wait;

		{
			mrpt::synch::CCriticalSectionLocker lock(&critSec_UpdateScene);
			m_canvas->m_openGLScene->visitAllObjects( &func_gather_stats );

			CSetOfObjectsPtr gl_octrees_bb;

			// Get object from scene, or creat upon first usage:
			{
				CRenderizablePtr obj = m_canvas->m_openGLScene->getByName(name_octrees_bb_globj);
				if (obj)
					gl_octrees_bb = CSetOfObjectsPtr(obj);
				else
				{
					gl_octrees_bb = CSetOfObjects::Create();
					gl_octrees_bb->setName( name_octrees_bb_globj );
					m_canvas->m_openGLScene->insert(gl_octrees_bb);
				}
			}

			// Show or hide, clear first anyway:
			gl_octrees_bb->clear();

			if (show_hide)
			{
				// Show:
				aux_gl_octrees_bb = gl_octrees_bb;

				m_canvas->m_openGLScene->visitAllObjects( func_get_octbb );

				aux_gl_octrees_bb.clear_unique();
			}
		}

		Refresh(false);
    }
    catch(std::exception &e)
    {
        wxMessageBox( _U(e.what()), _("Exception"), wxOK, this);
    }
}

// ----------------------------------------------------------
// Import a point cloud from the PLY file format
// ----------------------------------------------------------
void _DSceneViewerFrame::OnMenuItemImportPLYPointCloud(wxCommandEvent& event)
{
	try
	{
		wxFileDialog dialog(
			this,
			_("Choose the PLY file to import"),
			_U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ),
			_("*.ply"),
			_("PLY files (*.ply, *.PLY)|*.ply;*.PLY|All files (*.*)|*.*"),
			wxFD_OPEN | wxFD_FILE_MUST_EXIST );

		if (dialog.ShowModal() != wxID_OK)
			return;

		const std::string fil = string(dialog.GetPath().mb_str());
		saveLastUsedDirectoryToCfgFile(fil);

		CDlgPLYOptions dlgPLY(this);
		if (dlgPLY.ShowModal()!=wxID_OK)
			return;

		opengl::CPointCloudPtr gl_points;
		opengl::CPointCloudColouredPtr gl_points_col;
		mrpt::utils::PLY_Importer *ply_obj=NULL;

		if (dlgPLY.rbClass->GetSelection()==0)
		{
		     gl_points     = opengl::CPointCloud::Create();
			 ply_obj = gl_points.pointer();
		}
		else
		{
			gl_points_col = opengl::CPointCloudColoured::Create();
			ply_obj = gl_points_col.pointer();
		}

		CStringList file_comments, file_info;

		bool res;
		{
			wxBusyCursor  busy;
			res	= ply_obj->loadFromPlyFile(fil, &file_comments, &file_info);
		}
		if (!res)
		{
	        wxMessageBox( _("Error loading or parsing the PLY file"), _("Exception"), wxOK, this);
		}
		else
		{
			// Set the point cloud as the only object in scene:
			m_canvas->m_openGLScene = opengl::COpenGLScene::Create();

			if (dlgPLY.cbXYGrid->GetValue())
			{
				mrpt::opengl::CGridPlaneXYPtr obj = mrpt::opengl::CGridPlaneXY::Create( -50,50,-50,50,0,1 );
				obj->setColor(0.3,0.3,0.3);
				m_canvas->m_openGLScene->insert( obj );
			}

			if (dlgPLY.cbXYZ->GetValue())
				m_canvas->m_openGLScene->insert( mrpt::opengl::stock_objects::CornerXYZ() );

			double ptSize;
			dlgPLY.cbPointSize->GetStringSelection().ToDouble(&ptSize);
			if (gl_points)     gl_points->setPointSize( ptSize );
			if (gl_points_col) gl_points_col->setPointSize( ptSize );

			if (gl_points)
			{
				switch(dlgPLY.rbIntFromXYZ->GetSelection())
				{
					case 1: gl_points->enableColorFromX(); break;
					case 2: gl_points->enableColorFromY(); break;
					case 3: gl_points->enableColorFromZ(); break;
				};
			}

			TPose3D ptCloudPose(0,0,0, 0,0,0);

			dlgPLY.edYaw->GetValue().ToDouble(&ptCloudPose.yaw);
			dlgPLY.edPitch->GetValue().ToDouble(&ptCloudPose.pitch);
			dlgPLY.edRoll->GetValue().ToDouble(&ptCloudPose.roll);
			ptCloudPose.yaw   = DEG2RAD(ptCloudPose.yaw);
			ptCloudPose.pitch = DEG2RAD(ptCloudPose.pitch);
			ptCloudPose.roll  = DEG2RAD(ptCloudPose.roll);

			if (gl_points)     gl_points->setPose(CPose3D(ptCloudPose));
			if (gl_points_col) gl_points_col->setPose(CPose3D(ptCloudPose));

			// Insert point cloud into scene:
            if (gl_points)     m_canvas->m_openGLScene->insert(gl_points);
            if (gl_points_col) m_canvas->m_openGLScene->insert(gl_points_col);


			m_canvas->cameraPointingX = 0;
			m_canvas->cameraPointingY = 0;
			m_canvas->cameraPointingZ = 0;

			m_canvas->cameraZoomDistance = 4;
			m_canvas->cameraAzimuthDeg   = 45;
			m_canvas->cameraElevationDeg = 30;

			loadedFileName = std::string("Imported_")+fil+std::string(".3Dscene");
			updateTitle();

			Refresh(false);

			wxMessageBox(
				_U(format(
					"Comments:\n--------------------\n%s\nObject info:\n--------------------\n%s",
					file_comments.getText().c_str(), file_info.getText().c_str()).c_str()),
				_("File info"),
				wxOK,
				this);
		}
    }
    catch(std::exception &e)
    {
        wxMessageBox( _U(e.what()), _("Exception"), wxOK, this);
    }
}


struct visitor_export_PLY : public unary_function<mrpt::opengl::CRenderizablePtr,void>
{
	const string &filename;
	unsigned int &count;

	visitor_export_PLY(const string &fil,unsigned int &counter) : filename(fil), count(counter) { }

	void operator()(const mrpt::opengl::CRenderizablePtr &obj)
	{
		if (IS_CLASS(obj,CPointCloud))
		{
			CPointCloudPtr o = CPointCloudPtr(obj);
			o->saveToPlyFile(format("%s_%03u.ply",filename.c_str(),++count));
		}
		else
		if (IS_CLASS(obj,CPointCloudColoured))
		{
			CPointCloudColouredPtr o = CPointCloudColouredPtr(obj);
			o->saveToPlyFile(format("%s_%03u.ply",filename.c_str(),++count));
		}
	}
};

// ----------------------------------------------------------
// Export point clouds to the PLY file format
// ----------------------------------------------------------
void _DSceneViewerFrame::OnMenuItemExportPointsPLY(wxCommandEvent& event)
{
	try
	{
		wxMessageBox(_("Each point cloud object in the scene will be exported as a separate PLY file."),_("Notice") );

		wxFileDialog dialog(
			this,
			_("Choose the target PLY filename"),
			_U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ),
			_("*.ply"),
			_("PLY files (*.ply, *.PLY)|*.ply;*.PLY|All files (*.*)|*.*"),
			wxFD_SAVE );

		if (dialog.ShowModal() != wxID_OK)
			return;

		const std::string fil = string(dialog.GetPath().mb_str());
		saveLastUsedDirectoryToCfgFile(fil);

		// Go thru all point clouds and save them:
		unsigned int counter=0;
		visitor_export_PLY visitor(fil,counter);

		{
			wxBusyCursor busy;
			m_canvas->m_openGLScene->visitAllObjects( visitor );
		}

		wxMessageBox(wxString::Format(_("%u point cloud(s) exported to PLY files."),counter), _("Result"));

    }
    catch(std::exception &e)
    {
        wxMessageBox( _U(e.what()), _("Exception"), wxOK, this);
    }
}
