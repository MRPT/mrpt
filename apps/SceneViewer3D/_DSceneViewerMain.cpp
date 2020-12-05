/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "_DSceneViewerMain.h"
#include <wx/app.h>
#include "CDlgCamTracking.h"
#include "CDlgPLYOptions.h"

//(*InternalHeaders(_DSceneViewerFrame)
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/icon.h>
#include <wx/image.h>
#include <wx/intl.h>
#include <wx/string.h>
#include <wx/tglbtn.h>
//*)

#include "CDialogOptions.h"

#include <wx/busyinfo.h>
#include <wx/choicdlg.h>
#include <wx/colordlg.h>
#include <wx/dcmemory.h>
#include <wx/dirdlg.h>
#include <wx/filedlg.h>
#include <wx/imaglist.h>
#include <wx/log.h>
#include <wx/msgdlg.h>
#include <wx/numdlg.h>
#include <wx/progdlg.h>
#include <wx/textdlg.h>

#if defined(__WXMSW__)
const std::string iniFileSect("CONF_WIN");
#elif defined(__UNIX__)
const std::string iniFileSect("CONF_LIN");
#endif

#include "../wx-common/mrpt_logo.xpm"
#include "imgs/icono_main.xpm"

#if !wxUSE_GLCANVAS
#error "OpenGL required: set wxUSE_GLCANVAS to 1 and rebuild wxWidgets"
#endif

#include <mrpt/config/CConfigFile.h>
#include <mrpt/gui/CWxGLCanvasBase.h>
#include <mrpt/gui/about_box.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/opengl/CAngularObservationMesh.h>  // It's in lib mrpt-maps
#include <mrpt/opengl/CAssimpModel.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPlanarLaserScan.h>  // It's in lib mrpt-maps
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/CDirectoryExplorer.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>

#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CPointsMap.h>
#if MRPT_HAS_LIBLAS
#include <mrpt/maps/CPointsMap_liblas.h>
#endif

const mrpt::maps::CColouredPointsMap dummy_map;  // this is to enforce to load
// the mrpt-maps DLL, then
// register all OpenGL classes
// defined there.

// A custom Art provider for customizing the icons:
class MyArtProvider : public wxArtProvider
{
   protected:
	wxBitmap CreateBitmap(
		const wxArtID& id, const wxArtClient& client,
		const wxSize& size) override;
};

// CreateBitmap function
wxBitmap MyArtProvider::CreateBitmap(
	const wxArtID& id, const wxArtClient& client, const wxSize& size)
{
	if (id == wxART_MAKE_ART_ID(MAIN_ICON)) return wxBitmap(icono_main_xpm);
	if (id == wxART_MAKE_ART_ID(IMG_MRPT_LOGO)) return wxBitmap(mrpt_logo_xpm);

	// Any wxWidgets icons not implemented here
	// will be provided by the default art provider.
	return wxNullBitmap;
}

// Used for feedback from the glcanvas component to its parent.
_DSceneViewerFrame* theWindow = nullptr;

#ifdef MRPT_OS_WINDOWS
// Windows:
#include <windows.h>
#endif

#ifdef MRPT_OS_APPLE
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <mutex>

using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::io;
using namespace mrpt::config;
using namespace mrpt::rtti;
using namespace mrpt::opengl;
using namespace mrpt::img;
using namespace std;

// Critical section for updating the scene:
std::mutex critSec_UpdateScene;

// The file to open (from cmd line), or an empty string
extern std::string global_fileToOpen;
// The configuration file:
extern std::unique_ptr<CConfigFile> iniFile;

bool isCapturing = false;
string capturingDir = ".";
int captureCount = 0;

string loadedFileName;

bool freeCameraAlways = false;
bool freeCameraAlwaysNoAzimuth = false;

bool showFileNameInViewport = false;
int delayBetweenAutoplay = 5;

wxLogWindow* logWin = nullptr;

void saveLastUsedDirectoryToCfgFile(const std::string& fil)
{
	try
	{
		iniFile->write(iniFileSect, "LastDir", extractFileDirectory(fil));
	}
	catch (const std::exception& e)
	{
		wxMessageBox(mrpt::exception_to_str(e), _("Exception"), wxOK);
	}
}

void CMyGLCanvas::OnRenderError(const wxString& str)
{
	if (!logWin) logWin = new wxLogWindow(this, wxT("Log window"), false);

	wxLogError(str);
	logWin->Show();
}

void CMyGLCanvas::OnPreRender() {}

void CMyGLCanvas::OnPostRenderSwapBuffers(double At, wxPaintDC& dc)
{
	// Capture window image?
	// --------------------------
	if (isCapturing)
	{
		int w, h;
		dc.GetSize(&w, &h);

		// Save image directly from OpenGL
		CImage frame(w, h, CH_RGB);

		glPixelStorei(GL_PACK_ALIGNMENT, 1);
		glPixelStorei(GL_PACK_ROW_LENGTH, 0);

		glReadBuffer(GL_FRONT);
		glReadPixels(0, 0, w, h, GL_BGR_EXT, GL_UNSIGNED_BYTE, frame(0, 0));
		frame.flipVertical();

		string fileName(format(
			"%s/screenshot_%07i.png", capturingDir.c_str(), captureCount++));

		frame.saveToFile(fileName);
	}

	// Estimate FPS:
	// --------------------------
	double estimatedFPS;
	static double meanEstimatedFPS = 1;

	if (At > 0)
		estimatedFPS = 1 / At;
	else
		estimatedFPS = 0;

	meanEstimatedFPS = 0.8 * meanEstimatedFPS + 0.2 * estimatedFPS;

	string str = format(
		"Center=(%.02f,%.02f,%.02f) Zoom:%.02f AZ=%.02f deg EL:%.02f deg",
		getCameraPointingX(), getCameraPointingY(), getCameraPointingZ(),
		getZoomDistance(), getAzimuthDegrees(), getElevationDegrees());
	theWindow->StatusBar1->SetStatusText(str.c_str(), 1);

	str = format("%.02f FPS", meanEstimatedFPS);
	theWindow->StatusBar1->SetStatusText(str.c_str(), 2);

	str =
		format("%u viewports", (unsigned)getOpenGLSceneRef()->viewportsCount());
	theWindow->StatusBar1->SetStatusText(str.c_str(), 3);
}

void CMyGLCanvas::OnPostRender() {}

void CMyGLCanvas::OnCharCustom(wxKeyEvent& event)
{
	long evkey = event.GetKeyCode();

	if (evkey == WXK_LEFT || evkey == WXK_RIGHT)
	{
		try
		{
			CTicTac tictac;
			tictac.Tic();

			// Load a different file:

			// First, build a list of files in the directory:
			static CDirectoryExplorer::TFileInfoList lstFiles;
			static TTimeStamp lastUpdateOfList = INVALID_TIMESTAMP;
			TTimeStamp curTime = getCurrentTime();

			if (lastUpdateOfList == INVALID_TIMESTAMP ||
				timeDifference(lastUpdateOfList, curTime) > 2)
			{
				CDirectoryExplorer::explore(
					extractFileDirectory(loadedFileName),  // path
					FILE_ATTRIB_ARCHIVE,  // mask
					lstFiles);

				// Sort and remove non 3Dscene files:
				CDirectoryExplorer::sortByName(lstFiles);
				CDirectoryExplorer::filterByExtension(lstFiles, "3Dscene");

				lastUpdateOfList = curTime;
			}

			// cout << "Time to build file list: " << tictac.Tac() << endl;

			string curFileName = extractFileName(loadedFileName) + string(".") +
								 extractFileExtension(loadedFileName);

			// Find the current file:
			CDirectoryExplorer::TFileInfoList::iterator it;
			size_t i;
			for (i = 0, it = lstFiles.begin(); it != lstFiles.end(); it++, i++)
			{
				if (!os::_strcmpi(it->name.c_str(), curFileName.c_str()))
				{
					// Look for the desired file:
					if (evkey == WXK_LEFT)
					{
						if (i > 0)
							theWindow->loadFromFile(
								lstFiles[i - 1].wholePath, true);
						return;
					}
					else
					{
						if (i < (lstFiles.size() - 1))
							theWindow->loadFromFile(
								lstFiles[i + 1].wholePath, true);
						else
						{
							// This was the last file
						}
						return;
					}
				}
			}
		}
		catch (std::exception&)
		{
			// cerr << "*EXCEPTION* while determining next/previous file:\n" <<
			// mrpt::exception_to_str(e) << endl;
		}
		catch (...)
		{
		}
	}

	if (evkey == WXK_UP || evkey == WXK_DOWN)
	{
		// Move the camera forward-backward:
		// ...
		// Refresh(false);
	}
}

//(*IdInit(_DSceneViewerFrame)
const long _DSceneViewerFrame::ID_BUTTON1 = wxNewId();
const long _DSceneViewerFrame::ID_BUTTON2 = wxNewId();
const long _DSceneViewerFrame::ID_STATICLINE1 = wxNewId();
const long _DSceneViewerFrame::ID_BUTTON3 = wxNewId();
const long _DSceneViewerFrame::ID_BUTTON4 = wxNewId();
const long _DSceneViewerFrame::ID_BUTTON5 = wxNewId();
const long _DSceneViewerFrame::ID_STATICLINE2 = wxNewId();
const long _DSceneViewerFrame::ID_BUTTON6 = wxNewId();
const long _DSceneViewerFrame::ID_BUTTON7 = wxNewId();
const long _DSceneViewerFrame::ID_BUTTON8 = wxNewId();
const long _DSceneViewerFrame::ID_BUTTON9 = wxNewId();
const long _DSceneViewerFrame::ID_STATICLINE3 = wxNewId();
const long _DSceneViewerFrame::ID_BUTTON10 = wxNewId();
const long _DSceneViewerFrame::ID_BUTTON11 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM1 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM2 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM5 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM7 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM6 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM_ImportImage = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM20 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM25 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM19 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM22 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM21 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM29 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM30 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM12 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM23 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM18 = wxNewId();
const long _DSceneViewerFrame::idMenuQuit = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM24 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM26 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM27 = wxNewId();
const long _DSceneViewerFrame::ID_MENUITEM28 = wxNewId();
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
const long _DSceneViewerFrame::ID_TIMER1 = wxNewId();
//*)

const long _DSceneViewerFrame::ID_TRAVELLING_TIMER = wxNewId();

const long _DSceneViewerFrame::ID_TIMER_AUTOPLAY = wxNewId();

BEGIN_EVENT_TABLE(_DSceneViewerFrame, wxFrame)
//(*EventTable(_DSceneViewerFrame)
//*)
END_EVENT_TABLE()

_DSceneViewerFrame::_DSceneViewerFrame(wxWindow* parent, wxWindowID id)
	: maxv(0)
{
	theWindow = this;

	// Load my custom icons:
	wxArtProvider::Push(new MyArtProvider);

	//(*Initialize(_DSceneViewerFrame)
	wxMenuItem* MenuItem2;
	wxMenu* MenuItem15;
	wxMenuItem* MenuItem1;
	wxMenuItem* MenuItem4;
	wxFlexGridSizer* FlexGridSizer2;
	wxMenuItem* MenuItem13;
	wxMenu* Menu1;
	wxMenuItem* MenuItem12;
	wxMenuItem* MenuItem3;
	wxMenuBar* MenuBar1;
	wxFlexGridSizer* FlexGridSizer1;
	wxMenu* Menu2;

	Create(
		parent, id, _("3DSceneViewer - Part of the MRPT project"),
		wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("id"));
	SetMinSize(wxSize(150, 100));
	{
		wxIcon FrameIcon;
		FrameIcon.CopyFromBitmap(wxArtProvider::GetBitmap(
			wxART_MAKE_ART_ID_FROM_STR(_T("MAIN_ICON")), wxART_FRAME_ICON));
		SetIcon(FrameIcon);
	}
	FlexGridSizer1 = new wxFlexGridSizer(1, 2, 0, 0);
	FlexGridSizer1->AddGrowableCol(1);
	FlexGridSizer1->AddGrowableRow(0);
	FlexGridSizer2 = new wxFlexGridSizer(0, 1, 0, 0);
	btnNew = new wxCustomButton(
		this, ID_BUTTON1, _("  New  "),
		wxArtProvider::GetBitmap(
			wxART_MAKE_ART_ID_FROM_STR(_T("wxART_NORMAL_FILE")), wxART_TOOLBAR),
		wxDefaultPosition, wxDefaultSize, wxCUSTBUT_BUTTON | wxCUSTBUT_BOTTOM,
		wxDefaultValidator, _T("ID_BUTTON1"));
	btnNew->SetBitmapDisabled(wxArtProvider::GetBitmap(
		wxART_MAKE_ART_ID_FROM_STR(_T("wxART_NORMAL_FILE")), wxART_TOOLBAR));
	btnNew->SetMargins(wxSize(5, 5));
	FlexGridSizer2->Add(
		btnNew, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 1);
	btnToolbarOpen = new wxCustomButton(
		this, ID_BUTTON2, _("  Open... "),
		wxArtProvider::GetBitmap(
			wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FILE_OPEN")), wxART_TOOLBAR),
		wxDefaultPosition, wxDefaultSize, wxCUSTBUT_BUTTON | wxCUSTBUT_BOTTOM,
		wxDefaultValidator, _T("ID_BUTTON2"));
	btnToolbarOpen->SetBitmapDisabled(wxArtProvider::GetBitmap(
		wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FILE_OPEN")), wxART_TOOLBAR));
	btnToolbarOpen->SetMargins(wxSize(5, 5));
	FlexGridSizer2->Add(
		btnToolbarOpen, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 1);
	StaticLine1 = new wxStaticLine(
		this, ID_STATICLINE1, wxDefaultPosition, wxSize(50, -1),
		wxLI_HORIZONTAL, _T("ID_STATICLINE1"));
	FlexGridSizer2->Add(
		StaticLine1, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 2);
	btnPrev = new wxCustomButton(
		this, ID_BUTTON3, _("  Previous  "),
		wxArtProvider::GetBitmap(
			wxART_MAKE_ART_ID_FROM_STR(_T("wxART_GO_BACK")), wxART_TOOLBAR),
		wxDefaultPosition, wxDefaultSize, wxCUSTBUT_BUTTON | wxCUSTBUT_BOTTOM,
		wxDefaultValidator, _T("ID_BUTTON3"));
	btnPrev->SetBitmapDisabled(wxArtProvider::GetBitmap(
		wxART_MAKE_ART_ID_FROM_STR(_T("wxART_GO_BACK")), wxART_TOOLBAR));
	btnPrev->SetMargins(wxSize(5, 5));
	FlexGridSizer2->Add(
		btnPrev, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 1);
	btnNext = new wxCustomButton(
		this, ID_BUTTON4, _("  Next  "),
		wxArtProvider::GetBitmap(
			wxART_MAKE_ART_ID_FROM_STR(_T("wxART_GO_FORWARD")), wxART_TOOLBAR),
		wxDefaultPosition, wxDefaultSize, wxCUSTBUT_BUTTON | wxCUSTBUT_BOTTOM,
		wxDefaultValidator, _T("ID_BUTTON4"));
	btnNext->SetBitmapDisabled(wxArtProvider::GetBitmap(
		wxART_MAKE_ART_ID_FROM_STR(_T("wxART_GO_FORWARD")), wxART_TOOLBAR));
	btnNext->SetMargins(wxSize(5, 5));
	FlexGridSizer2->Add(
		btnNext, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 1);
	btnReload = new wxCustomButton(
		this, ID_BUTTON5, _("  Reload  "),
		wxArtProvider::GetBitmap(
			wxART_MAKE_ART_ID_FROM_STR(_T("wxART_REDO")), wxART_TOOLBAR),
		wxDefaultPosition, wxDefaultSize, wxCUSTBUT_BUTTON | wxCUSTBUT_BOTTOM,
		wxDefaultValidator, _T("ID_BUTTON5"));
	btnReload->SetBitmapDisabled(wxArtProvider::GetBitmap(
		wxART_MAKE_ART_ID_FROM_STR(_T("wxART_REDO")), wxART_TOOLBAR));
	btnReload->SetMargins(wxSize(5, 5));
	FlexGridSizer2->Add(
		btnReload, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP,
		wxDLG_UNIT(this, wxSize(1, 0)).GetWidth());
	StaticLine2 = new wxStaticLine(
		this, ID_STATICLINE2, wxDefaultPosition, wxSize(50, -1),
		wxLI_HORIZONTAL, _T("ID_STATICLINE2"));
	FlexGridSizer2->Add(
		StaticLine2, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 2);
	btnOptions = new wxCustomButton(
		this, ID_BUTTON6, _("  Options  "),
		wxArtProvider::GetBitmap(
			wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FIND")), wxART_TOOLBAR),
		wxDefaultPosition, wxDefaultSize, wxCUSTBUT_BUTTON | wxCUSTBUT_BOTTOM,
		wxDefaultValidator, _T("ID_BUTTON6"));
	btnOptions->SetBitmapDisabled(wxArtProvider::GetBitmap(
		wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FIND")), wxART_TOOLBAR));
	btnOptions->SetMargins(wxSize(5, 5));
	FlexGridSizer2->Add(
		btnOptions, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 1);
	btnOrtho = new wxCustomButton(
		this, ID_BUTTON7, _("  Ortho  "),
		wxArtProvider::GetBitmap(
			wxART_MAKE_ART_ID_FROM_STR(_T("wxART_TICK_MARK")), wxART_TOOLBAR),
		wxDefaultPosition, wxDefaultSize, wxCUSTBUT_TOGGLE | wxCUSTBUT_BOTTOM,
		wxDefaultValidator, _T("ID_BUTTON7"));
	btnOrtho->SetBitmapDisabled(wxArtProvider::GetBitmap(
		wxART_MAKE_ART_ID_FROM_STR(_T("wxART_TICK_MARK")), wxART_TOOLBAR));
	btnOrtho->SetMargins(wxSize(5, 5));
	FlexGridSizer2->Add(
		btnOrtho, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 1);
	btnAutoplay = new wxCustomButton(
		this, ID_BUTTON8, _("  Autoplay  "),
		wxArtProvider::GetBitmap(
			wxART_MAKE_ART_ID_FROM_STR(_T("wxART_REMOVABLE")), wxART_TOOLBAR),
		wxDefaultPosition, wxDefaultSize, wxCUSTBUT_TOGGLE | wxCUSTBUT_BOTTOM,
		wxDefaultValidator, _T("ID_BUTTON8"));
	btnAutoplay->SetBitmapDisabled(wxArtProvider::GetBitmap(
		wxART_MAKE_ART_ID_FROM_STR(_T("wxART_REMOVABLE")), wxART_TOOLBAR));
	btnAutoplay->SetMargins(wxSize(5, 5));
	FlexGridSizer2->Add(
		btnAutoplay, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 1);
	btnCapture = new wxCustomButton(
		this, ID_BUTTON9, _("  Capture  "),
		wxArtProvider::GetBitmap(
			wxART_MAKE_ART_ID_FROM_STR(_T("wxART_HARDDISK")), wxART_TOOLBAR),
		wxDefaultPosition, wxDefaultSize, wxCUSTBUT_TOGGLE | wxCUSTBUT_BOTTOM,
		wxDefaultValidator, _T("ID_BUTTON9"));
	btnCapture->SetBitmapDisabled(wxArtProvider::GetBitmap(
		wxART_MAKE_ART_ID_FROM_STR(_T("wxART_HARDDISK")), wxART_TOOLBAR));
	btnCapture->SetMargins(wxSize(5, 5));
	FlexGridSizer2->Add(
		btnCapture, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 1);
	StaticLine3 = new wxStaticLine(
		this, ID_STATICLINE3, wxDefaultPosition, wxSize(50, -1),
		wxLI_HORIZONTAL, _T("ID_STATICLINE3"));
	FlexGridSizer2->Add(
		StaticLine3, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 2);
	btnAbout = new wxCustomButton(
		this, ID_BUTTON10, _("  About..."),
		wxArtProvider::GetBitmap(
			wxART_MAKE_ART_ID_FROM_STR(_T("wxART_HELP_BOOK")), wxART_TOOLBAR),
		wxDefaultPosition, wxDefaultSize, wxCUSTBUT_BUTTON | wxCUSTBUT_BOTTOM,
		wxDefaultValidator, _T("ID_BUTTON10"));
	btnAbout->SetBitmapDisabled(wxArtProvider::GetBitmap(
		wxART_MAKE_ART_ID_FROM_STR(_T("wxART_HELP_BOOK")), wxART_TOOLBAR));
	btnAbout->SetMargins(wxSize(5, 5));
	FlexGridSizer2->Add(
		btnAbout, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 1);
	btnQuit = new wxCustomButton(
		this, ID_BUTTON11, _("  Quit  "),
		wxArtProvider::GetBitmap(
			wxART_MAKE_ART_ID_FROM_STR(_T("wxART_QUIT")), wxART_TOOLBAR),
		wxDefaultPosition, wxDefaultSize, wxCUSTBUT_BUTTON | wxCUSTBUT_BOTTOM,
		wxDefaultValidator, _T("ID_BUTTON11"));
	btnQuit->SetBitmapDisabled(wxArtProvider::GetBitmap(
		wxART_MAKE_ART_ID_FROM_STR(_T("wxART_QUIT")), wxART_TOOLBAR));
	btnQuit->SetMargins(wxSize(5, 5));
	FlexGridSizer2->Add(
		btnQuit, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 1);
	FlexGridSizer1->Add(
		FlexGridSizer2, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	SetSizer(FlexGridSizer1);
	MenuBar1 = new wxMenuBar();
	Menu1 = new wxMenu();
	MenuItem3 = new wxMenuItem(
		Menu1, ID_MENUITEM1, _("New scene"), wxEmptyString, wxITEM_NORMAL);
	Menu1->Append(MenuItem3);
	MenuItem4 = new wxMenuItem(
		Menu1, ID_MENUITEM2, _("Open..."), wxEmptyString, wxITEM_NORMAL);
	Menu1->Append(MenuItem4);
	MenuItem7 = new wxMenuItem(
		Menu1, ID_MENUITEM5, _("Reload file\tF5"), wxEmptyString,
		wxITEM_NORMAL);
	Menu1->Append(MenuItem7);
	MenuItem9 = new wxMenuItem(
		Menu1, ID_MENUITEM7, _("Save..."), wxEmptyString, wxITEM_NORMAL);
	Menu1->Append(MenuItem9);
	Menu1->AppendSeparator();
	MenuItem18 = new wxMenu();
	MenuItem8 = new wxMenuItem(
		MenuItem18, ID_MENUITEM6, _("a 3D model (any Assimp format)..."),
		wxEmptyString, wxITEM_NORMAL);
	MenuItem18->Append(MenuItem8);
	MenuItem19 = new wxMenuItem(
		MenuItem18, ID_MENUITEM20, _("a PLY point cloud..."), wxEmptyString,
		wxITEM_NORMAL);
	MenuItem18->Append(MenuItem19);
	mnuImportLAS = new wxMenuItem(
		MenuItem18, ID_MENUITEM25, _("a LAS LiDAR file..."), wxEmptyString,
		wxITEM_NORMAL);
	MenuItem18->Append(mnuImportLAS);
	MenuItemImportImage = new wxMenuItem(
		MenuItem18, ID_MENUITEM_ImportImage, _("an image..."), wxEmptyString,
		wxITEM_NORMAL);
	MenuItem18->Append(MenuItemImportImage);
	Menu1->Append(ID_MENUITEM19, _("Import"), MenuItem18, wxEmptyString);
	MenuItem20 = new wxMenu();
	MenuItem21 = new wxMenuItem(
		MenuItem20, ID_MENUITEM22, _("point clouds to PLY file..."),
		wxEmptyString, wxITEM_NORMAL);
	MenuItem20->Append(MenuItem21);
	Menu1->Append(ID_MENUITEM21, _("Export"), MenuItem20, wxEmptyString);
	Menu1->AppendSeparator();
	MenuItem23 = new wxMenuItem(
		Menu1, ID_MENUITEM29, _("Previous file\tF7"), wxEmptyString,
		wxITEM_NORMAL);
	Menu1->Append(MenuItem23);
	MenuItem24 = new wxMenuItem(
		Menu1, ID_MENUITEM30, _("Next file\tF8"), wxEmptyString, wxITEM_NORMAL);
	Menu1->Append(MenuItem24);
	Menu1->AppendSeparator();
	MenuItem14 = new wxMenuItem(
		Menu1, ID_MENUITEM12, _("Take snapshot...\tF2"),
		_("Saves the current window image to a file"), wxITEM_NORMAL);
	Menu1->Append(MenuItem14);
	MenuItem22 = new wxMenuItem(
		Menu1, ID_MENUITEM23, _("High-resolution render to file..."),
		wxEmptyString, wxITEM_NORMAL);
	Menu1->Append(MenuItem22);
	mnuSceneStats = new wxMenuItem(
		Menu1, ID_MENUITEM18, _("Scene stats"), wxEmptyString, wxITEM_NORMAL);
	Menu1->Append(mnuSceneStats);
	Menu1->AppendSeparator();
	MenuItem1 = new wxMenuItem(
		Menu1, idMenuQuit, _("Quit\tAlt-F4"), _("Quit the application"),
		wxITEM_NORMAL);
	Menu1->Append(MenuItem1);
	MenuBar1->Append(Menu1, _("&File"));
	Menu4 = new wxMenu();
	mnuSelectNone = new wxMenuItem(
		Menu4, ID_MENUITEM24, _("Select none"), wxEmptyString, wxITEM_NORMAL);
	Menu4->Append(mnuSelectNone);
	mnuSelectByClass = new wxMenuItem(
		Menu4, ID_MENUITEM26, _("Select by class..."), wxEmptyString,
		wxITEM_NORMAL);
	Menu4->Append(mnuSelectByClass);
	Menu4->AppendSeparator();
	mnuSelectionScale = new wxMenuItem(
		Menu4, ID_MENUITEM27, _("Re-scale selected..."), wxEmptyString,
		wxITEM_NORMAL);
	Menu4->Append(mnuSelectionScale);
	mnuSelectionDelete = new wxMenuItem(
		Menu4, ID_MENUITEM28, _("Delete selected"), wxEmptyString,
		wxITEM_NORMAL);
	Menu4->Append(mnuSelectionDelete);
	MenuBar1->Append(Menu4, _("&Edit"));
	Menu3 = new wxMenu();
	MenuItem6 = new wxMenuItem(
		Menu3, ID_MENUITEM4, _("Background color..."), wxEmptyString,
		wxITEM_NORMAL);
	Menu3->Append(MenuItem6);
	MenuItem5 = new wxMenuItem(
		Menu3, ID_MENUITEM3, _("Options..."), wxEmptyString, wxITEM_NORMAL);
	Menu3->Append(MenuItem5);
	Menu3->AppendSeparator();
	MenuItem17 = new wxMenu();
	mnuItemShowCloudOctrees = new wxMenuItem(
		MenuItem17, ID_MENUITEM15, _("Show/hide bounding boxes"), wxEmptyString,
		wxITEM_CHECK);
	MenuItem17->Append(mnuItemShowCloudOctrees);
	mnuItemChangeMaxPointsPerOctreeNode = new wxMenuItem(
		MenuItem17, ID_MENUITEM17, _("Change octree parameters..."),
		wxEmptyString, wxITEM_NORMAL);
	MenuItem17->Append(mnuItemChangeMaxPointsPerOctreeNode);
	Menu3->Append(
		ID_MENUITEM16, _("Point clouds octrees..."), MenuItem17, wxEmptyString);
	Menu3->AppendSeparator();
	MenuItem13 = new wxMenuItem(
		Menu3, ID_MENUITEM11, _("Delete all objects"), wxEmptyString,
		wxITEM_NORMAL);
	Menu3->Append(MenuItem13);
	MenuItem11 = new wxMenu();
	MenuItem12 = new wxMenuItem(
		MenuItem11, ID_MENUITEM9, _("Simple robot model"), wxEmptyString,
		wxITEM_NORMAL);
	MenuItem11->Append(MenuItem12);
	Menu3->Append(
		ID_MENUITEM8, _("Insert stock object"), MenuItem11, wxEmptyString);
	Menu3->AppendSeparator();
	MenuItem15 = new wxMenu();
	MenuItem10 = new wxMenuItem(
		MenuItem15, ID_MENUITEM10, _("Circular"), wxEmptyString, wxITEM_NORMAL);
	MenuItem15->Append(MenuItem10);
	MenuItem16 = new wxMenuItem(
		MenuItem15, ID_MENUITEM14, _("Arbitrary path..."), wxEmptyString,
		wxITEM_NORMAL);
	MenuItem15->Append(MenuItem16);
	Menu3->Append(
		ID_MENUITEM13, _("Camera tracking"), MenuItem15, wxEmptyString);
	MenuBar1->Append(Menu3, _("&Tools"));
	Menu2 = new wxMenu();
	MenuItem2 = new wxMenuItem(
		Menu2, idMenuAbout, _("About\tF1"),
		_("Show info about this application"), wxITEM_NORMAL);
	Menu2->Append(MenuItem2);
	MenuBar1->Append(Menu2, _("&Help"));
	SetMenuBar(MenuBar1);
	StatusBar1 = new wxStatusBar(this, ID_STATUSBAR1, 0, _T("ID_STATUSBAR1"));
	int __wxStatusBarWidths_1[4] = {-10, -10, -4, -5};
	int __wxStatusBarStyles_1[4] = {wxSB_NORMAL, wxSB_NORMAL, wxSB_NORMAL,
									wxSB_NORMAL};
	StatusBar1->SetFieldsCount(4, __wxStatusBarWidths_1);
	StatusBar1->SetStatusStyles(4, __wxStatusBarStyles_1);
	SetStatusBar(StatusBar1);
	timLoadFileCmdLine.SetOwner(this, ID_TIMER1);
	timLoadFileCmdLine.Start(50, false);
	FlexGridSizer1->Fit(this);
	FlexGridSizer1->SetSizeHints(this);
	Center();

	using svf = _DSceneViewerFrame;

	Bind(wxEVT_BUTTON, &svf::OnNewScene, this, ID_BUTTON1);
	Bind(wxEVT_BUTTON, &svf::OnOpenFile, this, ID_BUTTON2);
	Bind(wxEVT_BUTTON, &svf::OnPrevious, this, ID_BUTTON3);
	Bind(wxEVT_BUTTON, &svf::OnNext, this, ID_BUTTON4);
	Bind(wxEVT_BUTTON, &svf::OnReload, this, ID_BUTTON5);
	Bind(wxEVT_BUTTON, &svf::OnMenuOptions, this, ID_BUTTON6);
	Bind(
		wxEVT_COMMAND_TOGGLEBUTTON_CLICKED, &svf::OnbtnOrthoClicked, this,
		ID_BUTTON7);
	Bind(
		wxEVT_COMMAND_TOGGLEBUTTON_CLICKED, &svf::OnbtnAutoplayClicked, this,
		ID_BUTTON8);
	Bind(
		wxEVT_COMMAND_TOGGLEBUTTON_CLICKED, &svf::OnBtnRecordClicked, this,
		ID_BUTTON9);
	Bind(wxEVT_BUTTON, &svf::OnAbout, this, ID_BUTTON10);
	Bind(wxEVT_BUTTON, &svf::OnQuit, this, ID_BUTTON11);
	Bind(wxEVT_MENU, &svf::OnNewScene, this, ID_MENUITEM1);
	Bind(wxEVT_MENU, &svf::OnOpenFile, this, ID_MENUITEM2);
	Bind(wxEVT_MENU, &svf::OnReload, this, ID_MENUITEM5);
	Bind(wxEVT_MENU, &svf::OnMenuSave, this, ID_MENUITEM7);
	Bind(wxEVT_MENU, &svf::OnInsert3DS, this, ID_MENUITEM6);
	Bind(wxEVT_MENU, &svf::OnMenuItemImportPLYPointCloud, this, ID_MENUITEM20);
	Bind(wxEVT_MENU, &svf::OnmnuImportLASSelected, this, ID_MENUITEM25);
	Bind(wxEVT_MENU, &svf::OnmnuImportImageView, this, ID_MENUITEM_ImportImage);
	Bind(wxEVT_MENU, &svf::OnMenuItemExportPointsPLY, this, ID_MENUITEM22);
	Bind(wxEVT_MENU, &svf::OnPrevious, this, ID_MENUITEM29);
	Bind(wxEVT_MENU, &svf::OnNext, this, ID_MENUITEM30);
	Bind(wxEVT_MENU, &svf::OnMenuItem14Selected, this, ID_MENUITEM12);
	Bind(wxEVT_MENU, &svf::OnMenuItemHighResRender, this, ID_MENUITEM23);
	Bind(wxEVT_MENU, &svf::OnmnuSceneStatsSelected, this, ID_MENUITEM18);
	Bind(wxEVT_MENU, &svf::OnQuit, this, idMenuQuit);
	Bind(wxEVT_MENU, &svf::OnmnuSelectNoneSelected, this, ID_MENUITEM24);
	Bind(wxEVT_MENU, &svf::OnmnuSelectByClassSelected, this, ID_MENUITEM26);
	Bind(wxEVT_MENU, &svf::OnmnuSelectionScaleSelected, this, ID_MENUITEM27);
	Bind(wxEVT_MENU, &svf::OnmnuSelectionDeleteSelected, this, ID_MENUITEM28);
	Bind(wxEVT_MENU, &svf::OnMenuBackColor, this, ID_MENUITEM4);
	Bind(wxEVT_MENU, &svf::OnMenuOptions, this, ID_MENUITEM3);
	Bind(
		wxEVT_MENU, &svf::OnmnuItemShowCloudOctreesSelected, this,
		ID_MENUITEM15);
	Bind(
		wxEVT_MENU, &svf::OnmnuItemChangeMaxPointsPerOctreeNodeSelected, this,
		ID_MENUITEM17);
	Bind(wxEVT_MENU, &svf::OnMenuDeleteAll, this, ID_MENUITEM11);
	Bind(wxEVT_MENU, &svf::OnMenuAddSICK, this, ID_MENUITEM9);
	Bind(wxEVT_MENU, &svf::OnStartCameraTravelling, this, ID_MENUITEM10);
	Bind(wxEVT_MENU, &svf::OnAbout, this, idMenuAbout);
	Bind(wxEVT_TIMER, &svf::OntimLoadFileCmdLineTrigger, this, ID_TIMER1);
	//*)

	Bind(wxEVT_MENU, &svf::OnMenuCameraTrackingArbitrary, this, ID_MENUITEM14);
	Bind(wxEVT_TIMER, &svf::OntimAutoplay, this, ID_TIMER_AUTOPLAY);
	Bind(wxEVT_TIMER, &svf::OnTravellingTrigger, this, ID_TRAVELLING_TIMER);

	// Create the wxCanvas object:
	m_canvas =
		new CMyGLCanvas(this, wxID_ANY, wxDefaultPosition, wxDefaultSize);
#if wxCHECK_VERSION(2, 9, 0)
	m_canvas->SetMinClientSize(wxSize(100, 100));
#endif
	FlexGridSizer1->Add(
		m_canvas, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);

	// Load an empty scene:
	wxCommandEvent dummEvent;
	OnNewScene(dummEvent);

	m_autoplayTimer = std::make_unique<wxTimer>(this, ID_TIMER_AUTOPLAY);

	m_tTravelling.SetOwner(this, ID_TRAVELLING_TIMER);

	m_dlg_tracking = new CDlgCamTracking(this);

	Maximize();
}

_DSceneViewerFrame::~_DSceneViewerFrame()
{
	theWindow = nullptr;

	//(*Destroy(_DSceneViewerFrame)
	//*)
}

void _DSceneViewerFrame::OnQuit(wxCommandEvent&) { Close(); }
void _DSceneViewerFrame::OnAbout(wxCommandEvent&)
{
	mrpt::gui::show_mrpt_about_box_wxWidgets(this, "SceneViewer3D");
}

void _DSceneViewerFrame::OnNewScene(wxCommandEvent& event)
{
	auto openGLSceneRef = m_canvas->getOpenGLSceneRef();
	openGLSceneRef->clear();

	// Default file name:
	loadedFileName = "no-name.3Dscene";
	updateTitle();

	{
		mrpt::opengl::CGridPlaneXY::Ptr obj =
			mrpt::opengl::CGridPlaneXY::Create(-50, 50, -50, 50, 0, 1);
		obj->setColor(0.3f, 0.3f, 0.3f);
		openGLSceneRef->insert(obj);
	}

	m_canvas->setCameraPointing(0.0f, 0.0f, 0.0f);
	m_canvas->setZoomDistance(20.0f);
	m_canvas->setElevationDegrees(45.0f);
	m_canvas->setAzimuthDegrees(45.0f);

	openGLSceneRef->insert(stock_objects::CornerXYZ());

	// Add a clone viewport:
	COpenGLViewport::Ptr vi = openGLSceneRef->createViewport("clone");
	vi->setViewportPosition(0.05, 0.05, 0.2, 0.2);
	vi->setCloneView("main");
	vi->setCloneCamera(true);
	vi->setBorderSize(1);

	Refresh(false);
}

void _DSceneViewerFrame::OnOpenFile(wxCommandEvent& event)
{
	wxString caption = wxT("Choose a file to open");
	wxString wildcard =
		wxT("3D scene files (*.3Dscene)|*.3Dscene|All files (*.*)|*.*");

	wxString defaultDir(
		(iniFile->read_string(iniFileSect, "LastDir", ".").c_str()));

	wxString defaultFilename;
	wxFileDialog dialog(
		this, caption, defaultDir, defaultFilename, wildcard,
		wxFD_OPEN | wxFD_FILE_MUST_EXIST);

	if (dialog.ShowModal() == wxID_OK)
	{
		wxString fileName = dialog.GetPath();

		loadFromFile(std::string(fileName.mb_str()));
	}
}

void _DSceneViewerFrame::loadFromFile(
	const std::string& fil, bool isInASequence)
{
	try
	{
		// Save the path
		saveLastUsedDirectoryToCfgFile(fil);

		// static float
		// old_cam_pX,old_cam_pY,old_cam_pZ,old_cam_d,old_cam_az,old_cam_el;
		static bool first = true;

		if (first)
		{
			first = false;
			// old_cam_pX=old_cam_pY=old_cam_pZ=old_cam_d=old_cam_az=old_cam_el=0.0f;
		}

		CFileGZInputStream f(fil);

		static mrpt::system::CTicTac tictac;
		auto openGLSceneRef = m_canvas->getOpenGLSceneRef();
		{
			std::lock_guard<std::mutex> lock(critSec_UpdateScene);

			tictac.Tic();

			openGLSceneRef->clear();
			mrpt::serialization::archiveFrom(f) >> *openGLSceneRef;
		}

		double timeToLoad = tictac.Tac();

		if (openGLSceneRef->viewportsCount() == 0 ||
			!openGLSceneRef->getViewport("main") ||
			openGLSceneRef->getViewport("main")->size() == 0)
		{
			wxMessageBox(
				_("File is empty or format unrecognized."), _("Warning"), wxOK,
				this);
			btnAutoplay->SetValue(false);
		}

		// Change the camera if necesary:
		if (openGLSceneRef->followCamera())
		{
			COpenGLViewport::Ptr view = openGLSceneRef->getViewport("main");
			if (!view)
				THROW_EXCEPTION(
					"Fatal error: there is no 'main' viewport in the 3D "
					"scene!");

			CCamera::Ptr cam = openGLSceneRef->getByClass<CCamera>();

			bool camIsCCameraObj = cam ? true : false;
			if (!camIsCCameraObj) cam = CCamera::Create(view->getCamera());

			m_canvas->setCameraPointing(
				cam->getPointingAtX(), cam->getPointingAtY(),
				cam->getPointingAtZ());

			// If it's not loaded thru arrow keys or a timed sequence, take
			// camera:
			if (!isInASequence || !freeCameraAlways ||
				!freeCameraAlwaysNoAzimuth)
			{
				if (!freeCameraAlwaysNoAzimuth)
				{
					m_canvas->setZoomDistance(cam->getZoomDistance());
					m_canvas->setElevationDegrees(cam->getElevationDegrees());
				}
				m_canvas->setAzimuthDegrees(cam->getAzimuthDegrees());
			}

			// Remove the camera from the object:
			if (camIsCCameraObj) openGLSceneRef->removeObject(cam);
		}

		loadedFileName = fil;

		if (showFileNameInViewport)
		{
			openGLSceneRef->getViewport()->addTextMessage(
				20, 20, extractFileName(loadedFileName));
		}

		// Set the file name as window title:
		updateTitle();

		theWindow->StatusBar1->SetStatusText(
			(format("File loaded in %.03fs", timeToLoad).c_str()), 0);

		Refresh(false);
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e) << std::endl;
		btnAutoplay->SetValue(false);
		wxMessageBox(mrpt::exception_to_str(e), _("Exception"), wxOK, this);
	}
	catch (...)
	{
		btnAutoplay->SetValue(false);
		wxMessageBox(_("Runtime error!"), _("Exception"), wxOK, this);
	}
}

void _DSceneViewerFrame::updateTitle()
{
	SetTitle((format(
				  "3DSceneViewer - Part of the MRPT project [%s]",
				  (extractFileName(loadedFileName) + string(".") +
				   extractFileExtension(loadedFileName))
					  .c_str())
				  .c_str()));
}

void _DSceneViewerFrame::OntimLoadFileCmdLineTrigger(wxTimerEvent&)
{
	timLoadFileCmdLine.Stop();  // One shot only.
	// Open file if passed by the command line:
	if (!global_fileToOpen.empty())
	{
		if (mrpt::system::strCmpI(
				"3Dscene", mrpt::system::extractFileExtension(
							   global_fileToOpen, true /*ignore .gz*/)))
		{
			loadFromFile(global_fileToOpen);
		}
		else
		{
			std::cout << "Filename extension does not match `3Dscene`, "
						 "importing as an ASSIMP model...\n";
			try
			{
				auto obj3D = mrpt::opengl::CAssimpModel::Create();
				obj3D->loadScene(global_fileToOpen);
				// obj3D->setPose(mrpt::math::TPose3D(0, 0, 0, .0_deg,
				// 0._deg, 90.0_deg));
				m_canvas->getOpenGLSceneRef()->insert(obj3D);

				m_canvas->Refresh();
			}
			catch (const std::exception& e)
			{
				std::cerr << mrpt::exception_to_str(e) << std::endl;
				wxMessageBox(
					mrpt::exception_to_str(e), _("Exception"), wxOK, this);
			}
		}
	}
}

void _DSceneViewerFrame::OnbtnAutoplayClicked(wxCommandEvent& event)
{
	if (btnAutoplay->GetValue()) m_autoplayTimer->Start(5, true);  // One-shot:
}

void _DSceneViewerFrame::OntimAutoplay(wxTimerEvent& event)
{
	// Load next file:
	wxKeyEvent dummyEvent;
	dummyEvent.m_keyCode = WXK_RIGHT;
	m_canvas->OnCharCustom(dummyEvent);

	// Continue?
	if (btnAutoplay->GetValue())
		m_autoplayTimer->Start(delayBetweenAutoplay, true);  // One-shot:
}

void _DSceneViewerFrame::OnMenuBackColor(wxCommandEvent& event)
{
	wxColourData colourData;
	wxColour color;
	color.Set(
		(int)(255 * m_canvas->getClearColorR()),
		(int)(255 * m_canvas->getClearColorG()),
		(int)(255 * m_canvas->getClearColorB()));

	colourData.SetColour(color);
	colourData.SetChooseFull(true);

	wxColourDialog colDial(this, &colourData);

	if (wxID_OK == colDial.ShowModal())
	{
		wxColour col = colDial.GetColourData().GetColour();
		m_canvas->setClearColors(
			col.Red() / 255.0f, col.Green() / 255.0f, col.Blue() / 255.0f);

		int w, h;
		m_canvas->GetSize(&w, &h);
		RefreshRect(wxRect(0, 0, w, h), false);
	}
}

// See options:
void _DSceneViewerFrame::OnMenuOptions(wxCommandEvent& event)
{
	CDialogOptions dial(this);

	// Put current options:
	dial.cbViewFileName->SetValue(showFileNameInViewport);
	dial.cbFreeCamera->SetValue(freeCameraAlways);
	dial.cbFreeCameraNoAzimuth->SetValue(freeCameraAlwaysNoAzimuth);
	dial.edDelay->SetValue(delayBetweenAutoplay);

	// Show
	if (dial.ShowModal() == wxID_OK)
	{
		// If OK, get new options:
		showFileNameInViewport = dial.cbViewFileName->GetValue();
		freeCameraAlways = dial.cbFreeCamera->GetValue();
		freeCameraAlwaysNoAzimuth = dial.cbFreeCameraNoAzimuth->GetValue();
		delayBetweenAutoplay = dial.edDelay->GetValue();
		m_canvas->Refresh(false);
	}
}

void _DSceneViewerFrame::OnPrevious(wxCommandEvent& event)
{
	wxKeyEvent evn;
	evn.m_keyCode = WXK_LEFT;
	m_canvas->OnChar(evn);
}

void _DSceneViewerFrame::OnNext(wxCommandEvent& event)
{
	wxKeyEvent evn;
	evn.m_keyCode = WXK_RIGHT;
	m_canvas->OnChar(evn);
}

void _DSceneViewerFrame::OnClose(wxCloseEvent& event) {}
void _DSceneViewerFrame::OnBtnRecordClicked(wxCommandEvent& event)
{
	if (btnCapture->GetValue())
	{
		// Starting recording:
		wxDirDialog dirDialog(
			this, _("Choose the directory where to save the screenshots:"),
			extractFileDirectory(loadedFileName).c_str(), 0, wxDefaultPosition);

		if (dirDialog.ShowModal() == wxID_OK)
		{
			capturingDir = string(dirDialog.GetPath().mb_str());
			isCapturing = true;
			captureCount = 0;
			btnCapture->SetValue(true);
		}
		else
		{
			btnCapture->SetValue(false);
		}
	}
	else
	{
		isCapturing = false;
	}
}

void _DSceneViewerFrame::OnbtnOrthoClicked(wxCommandEvent& event)
{
	bool ortho = btnOrtho->GetValue();
	m_canvas->setCameraProjective(!ortho);

	m_canvas->Refresh(false);
}

void _DSceneViewerFrame::OnReload(wxCommandEvent& event)
{
	if (fileExists(loadedFileName)) loadFromFile(loadedFileName);
}

void _DSceneViewerFrame::OnInsert3DS(wxCommandEvent& event)
{
	try
	{
		wxString caption = wxT("Choose a file to import");
		wxString wildcard =
			wxT("3D models (All Assimp "
				"formats)|*.dae;*.blend;*.3ds;*.ase;*.obj;*.ifc;*.xgl;.zgl;*."
				"ply;*.dxf;.lwo;*.lws;*.lxo;*.stl;*.x;*.ac;*.ms3d;*.cob;.scn;*."
				"bvh;*.csm;*.xml;*.irrmesh;*.irr;*.mdl;*.md2;*.md3;*.pk3;*.mdc;"
				"*.md5*;*.smd;.vta;*.m3;*.3d;*.b3d;*.q3d;*.q3s;*.nff;*.nff;*."
				"off;*.raw;*.ter;*.mdl;*.hmp;*.ndo;|All files (*.*)|*.*");
		wxString defaultDir(
			(iniFile->read_string(iniFileSect, "LastDir", ".").c_str()));
		wxString defaultFilename;

		wxFileDialog dialog(
			this, caption, defaultDir, defaultFilename, wildcard,
			wxFD_OPEN | wxFD_FILE_MUST_EXIST);

		if (dialog.ShowModal() != wxID_OK) return;

		wxString fileName = dialog.GetPath();
		std::string fil = string(fileName.mb_str());

		saveLastUsedDirectoryToCfgFile(fil);

		mrpt::opengl::CAssimpModel::Ptr obj3D =
			mrpt::opengl::CAssimpModel::Create();
		obj3D->loadScene(fil);
		// obj3D->setPose(mrpt::math::TPose3D(0, 0, 0, .0_deg,
		// 0._deg, 90.0_deg));
		m_canvas->getOpenGLSceneRef()->insert(obj3D);

		m_canvas->Refresh();
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e) << std::endl;
		btnAutoplay->SetValue(false);
		wxMessageBox(mrpt::exception_to_str(e), _("Exception"), wxOK, this);
	}
	catch (...)
	{
		btnAutoplay->SetValue(false);
		wxMessageBox(_("Runtime error!"), _("Exception"), wxOK, this);
	}
}

void _DSceneViewerFrame::OnMenuSave(wxCommandEvent& event)
{
	try
	{
		wxString caption = wxT("Save scene to file");
		wxString wildcard =
			wxT("3Dscene files (*.3Dscene)|*.3Dscene|All files (*.*)|*.*");
		wxString defaultDir(
			(iniFile->read_string(iniFileSect, "LastDir", ".").c_str()));
		wxString defaultFilename;

		wxFileDialog dialog(
			this, caption, defaultDir, defaultFilename, wildcard,
			wxFD_SAVE | wxFD_OVERWRITE_PROMPT);

		if (dialog.ShowModal() != wxID_OK) return;

		wxString fileName = dialog.GetPath();

		CFileGZOutputStream fo(string(fileName.mb_str()));
		mrpt::serialization::archiveFrom(fo) << *m_canvas->getOpenGLSceneRef();
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e) << std::endl;
		btnAutoplay->SetValue(false);
		wxMessageBox(mrpt::exception_to_str(e), _("Exception"), wxOK, this);
	}
	catch (...)
	{
		btnAutoplay->SetValue(false);
		wxMessageBox(_("Runtime error!"), _("Exception"), wxOK, this);
	}
}

void _DSceneViewerFrame::OnTravellingTrigger(wxTimerEvent& event)
{
	try
	{
		auto openGLSceneRef = m_canvas->getOpenGLSceneRef();
		if ((openGLSceneRef->viewportsCount() == 0) ||
			!openGLSceneRef->getViewport("main") ||
			(openGLSceneRef->getViewport("main")->size() == 0))
		{
			wxMessageBox(_("Canvas is empty"), _("Warning"), wxOK, this);
		}
		else
		{
			// Change the camera
			COpenGLViewport::Ptr view = openGLSceneRef->getViewport("main");

			if (!view)
				THROW_EXCEPTION(
					"Fatal error: there is no 'main' viewport in the 3D "
					"scene!");

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

				mrpt::Clock::time_point t =
					m_dlg_tracking->m_poses.begin()->first +
					(mrpt::Clock::now() - m_travelling_start_time.value());
				bool valid = false;
				CPose3D p;
				while (!valid && t < m_dlg_tracking->m_poses.rbegin()->first)
				{
					m_dlg_tracking->m_poses.interpolate(t, p, valid);
					if (!valid)
					{
						using namespace std::chrono_literals;
						auto At = 100ms;
						t += At;
						m_travelling_start_time.value() -= At;
					}
				}

				if (valid)
					m_canvas->setCameraPointing(p.x(), p.y(), p.z());
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
				double step = atof(
					iniFile->read_string("Spherical travelling", "Step", "5")
						.c_str());

				if ((m_canvas->getAzimuthDegrees() + step) < maxv)
					m_canvas->setAzimuthDegrees(
						m_canvas->getAzimuthDegrees() + step / 10.0);
				else
					m_tTravelling.Stop();
			}

			Refresh(false);
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e) << std::endl;
		wxMessageBox(mrpt::exception_to_str(e), _("Exception"), wxOK, this);
	}
	catch (...)
	{
		wxMessageBox(_("Runtime error!"), _("Exception"), wxOK, this);
	}
}

void _DSceneViewerFrame::OnStartCameraTravelling(wxCommandEvent& event)
{
	try
	{
		auto openGLSceneRef = m_canvas->getOpenGLSceneRef();
		if ((openGLSceneRef->viewportsCount() == 0) ||
			!openGLSceneRef->getViewport("main") ||
			(openGLSceneRef->getViewport("main")->size() == 0))
		{
			wxMessageBox(_("Canvas is empty"), _("Warning"), wxOK, this);
		}
		else
		{
			// Change the camera
			COpenGLViewport::Ptr view = openGLSceneRef->getViewport("main");

			if (!view)
				THROW_EXCEPTION(
					"Fatal error: there is no 'main' viewport in the 3D "
					"scene!");

			double target_x = atof(
				iniFile->read_string("Spherical travelling", "X", "0").c_str());
			double target_y = atof(
				iniFile->read_string("Spherical travelling", "Y", "0").c_str());
			double target_z = atof(
				iniFile->read_string("Spherical travelling", "Z", "0").c_str());

			double zoom = atof(
				iniFile
					->read_string("Spherical travelling", "Zoom distance", "25")
					.c_str());
			double elevation =
				atof(iniFile
						 ->read_string(
							 "Spherical travelling", "Elevation degrees", "15")
						 .c_str());
			double azimuth =
				atof(iniFile
						 ->read_string(
							 "Spherical travelling", "Azimuth degrees", "90")
						 .c_str());

			double min_value = atof(
				iniFile->read_string("Spherical travelling", "Min value", "90")
					.c_str());
			double max_value = atof(
				iniFile->read_string("Spherical travelling", "Max value", "90")
					.c_str());

			m_canvas->setCameraPointing(target_x, target_y, target_z);
			m_canvas->setZoomDistance(zoom);
			m_canvas->setElevationDegrees(elevation);
			m_canvas->setAzimuthDegrees(azimuth - min_value);

			maxv = azimuth + max_value;

			m_travelling_is_arbitrary = false;  // Circular
			m_tTravelling.Start(100);

			Refresh(false);
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e) << std::endl;
		wxMessageBox(mrpt::exception_to_str(e), _("Exception"), wxOK, this);
	}
	catch (...)
	{
		wxMessageBox(_("Runtime error!"), _("Exception"), wxOK, this);
	}
}

void _DSceneViewerFrame::OnClose1(wxCloseEvent& event) {}
void _DSceneViewerFrame::OnMenuAddSICK(wxCommandEvent& event)
{
	try
	{
		mrpt::opengl::CSetOfObjects::Ptr obj =
			mrpt::opengl::stock_objects::RobotPioneer();
		m_canvas->getOpenGLSceneRef()->insert(obj);
		m_canvas->Refresh();
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e) << std::endl;
		wxMessageBox(mrpt::exception_to_str(e), _("Exception"), wxOK, this);
	}
	catch (...)
	{
		wxMessageBox(_("Runtime error!"), _("Exception"), wxOK, this);
	}
}

void _DSceneViewerFrame::OnMenuDeleteAll(wxCommandEvent& event)
{
	m_canvas->getOpenGLSceneRef()->clear();
	m_canvas->Refresh();
}

void _DSceneViewerFrame::OnMenuItem14Selected(wxCommandEvent& event)
{
	m_canvas->Refresh();
	wxApp::GetInstance()->Yield();

	wxClientDC dc(this);

	int w, h;
	dc.GetSize(&w, &h);

	// Save image directly from OpenGL
	CImage frame(w, h, CH_RGB);

	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glPixelStorei(GL_PACK_ROW_LENGTH, 0);

	glReadBuffer(GL_FRONT);
	glReadPixels(0, 0, w, h, GL_BGR_EXT, GL_UNSIGNED_BYTE, frame(0, 0));
	frame.flipVertical();

	// Save:
	wxString caption = wxT("Save snapshot to file");
	wxString wildcard = wxT("Image files (*.png)|*.png|All files (*.*)|*.*");
	wxString defaultDir(
		(iniFile->read_string(iniFileSect, "LastDir", ".").c_str()));
	wxString defaultFilename = wxT("snapshot.png");

	wxFileDialog dialog(
		this, caption, defaultDir, defaultFilename, wildcard,
		wxFD_SAVE | wxFD_OVERWRITE_PROMPT);

	if (dialog.ShowModal() != wxID_OK) return;

	frame.saveToFile(std::string(dialog.GetPath().mb_str()));
}

void _DSceneViewerFrame::OnMenuCameraTrackingArbitrary(wxCommandEvent& event)
{
	m_dlg_tracking->Show();
}

void _DSceneViewerFrame::OnmnuItemChangeMaxPointsPerOctreeNodeSelected(
	wxCommandEvent& event)
{
	wxString sRet1 = wxGetTextFromUser(
		_("Max. number of points in an octree node before split:"),
		_("Enter new value"),
		wxString::Format(
			_("%e"),
			(double)mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE()),
		this);

	wxString sRet2 = wxGetTextFromUser(
		_("Max. density of points in each octree (points/pixel^2):"),
		_("Enter new value"),
		wxString::Format(
			_("%e"), (double)mrpt::global_settings::
						 OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL()),
		this);

	double N1, N2;
	if (sRet1.ToDouble(&N1) && sRet2.ToDouble(&N2))
	{
		mrpt::global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE(N1);
		mrpt::global_settings::OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL(N2);

		// Redo the octrees:
		clear_all_octrees_in_scene();
		Refresh(false);

		wxCommandEvent dumm;  // Redraw bounding-boxes:
		OnmnuItemShowCloudOctreesSelected(dumm);
	}
	else
		wxMessageBox(_("Invalid number!"));
}

void func_clear_octrees(const mrpt::opengl::CRenderizable::Ptr& o)
{
	if (IS_CLASS(*o, CPointCloud))
	{
		CPointCloud::Ptr obj = std::dynamic_pointer_cast<CPointCloud>(o);
		obj->octree_mark_as_outdated();
	}
	else if (IS_CLASS(*o, CPointCloudColoured))
	{
		CPointCloudColoured::Ptr obj =
			std::dynamic_pointer_cast<CPointCloudColoured>(o);
		obj->octree_mark_as_outdated();
	}
}

void _DSceneViewerFrame::clear_all_octrees_in_scene()
{
	{
		std::lock_guard<std::mutex> lock(critSec_UpdateScene);
		m_canvas->getOpenGLSceneRef()->visitAllObjects(&func_clear_octrees);
	}
}

struct TSceneStats
{
	void clear()
	{
		nPoints = 0;
		nObjects = 0;
		nOctreeVisible = 0;
		nOctreeTotal = 0;
	}

	size_t nPoints;
	size_t nObjects;
	size_t nOctreeVisible, nOctreeTotal;
};

TSceneStats sceneStats;

void func_gather_stats(const mrpt::opengl::CRenderizable::Ptr& o)
{
	sceneStats.nObjects++;

	if (IS_CLASS(*o, CPointCloud))
	{
		CPointCloud::Ptr obj = std::dynamic_pointer_cast<CPointCloud>(o);
		sceneStats.nPoints += obj->size();
		sceneStats.nOctreeVisible += obj->octree_get_visible_nodes();
		sceneStats.nOctreeTotal += obj->octree_get_node_count();
	}
	else if (IS_CLASS(*o, CPointCloudColoured))
	{
		CPointCloudColoured::Ptr obj =
			std::dynamic_pointer_cast<CPointCloudColoured>(o);
		sceneStats.nPoints += obj->size();
		sceneStats.nOctreeVisible += obj->octree_get_visible_nodes();
		sceneStats.nOctreeTotal += obj->octree_get_node_count();
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
			std::lock_guard<std::mutex> lock(critSec_UpdateScene);
			m_canvas->getOpenGLSceneRef()->visitAllObjects(&func_gather_stats);
		}

		std::stringstream ss;
		ss << "Number of objects: " << sceneStats.nObjects << endl
		   << format(
				  "Overall points (in point clouds): %e",
				  double(sceneStats.nPoints))
		   << endl
		   << "Total octree nodes   (in point clouds): "
		   << sceneStats.nOctreeTotal << endl
		   << "Visible octree nodes (in point clouds): "
		   << sceneStats.nOctreeVisible << endl;

		wxMessageBox(ss.str().c_str(), _("Scene statistics"));
	}
	catch (const std::exception& e)
	{
		wxMessageBox(mrpt::exception_to_str(e), _("Exception"), wxOK, this);
	}
}

static const std::string name_octrees_bb_globj =
	"__3dsceneviewer_gl_octree_bb__";
CSetOfObjects::Ptr aux_gl_octrees_bb;

void func_get_octbb(const mrpt::opengl::CRenderizable::Ptr& o)
{
	if (IS_CLASS(*o, CPointCloud))
	{
		CPointCloud::Ptr obj = std::dynamic_pointer_cast<CPointCloud>(o);
		CSetOfObjects::Ptr new_bb = std::make_shared<CSetOfObjects>();
		obj->octree_get_graphics_boundingboxes(*new_bb);
		aux_gl_octrees_bb->insert(new_bb);
	}
	else if (IS_CLASS(*o, CPointCloudColoured))
	{
		CPointCloudColoured::Ptr obj =
			std::dynamic_pointer_cast<CPointCloudColoured>(o);
		CSetOfObjects::Ptr new_bb = std::make_shared<CSetOfObjects>();
		obj->octree_get_graphics_boundingboxes(*new_bb);
		aux_gl_octrees_bb->insert(new_bb);
	}
}

// Show/hide the octree bounding boxes of the point clouds:
void _DSceneViewerFrame::OnmnuItemShowCloudOctreesSelected(
	wxCommandEvent& event)
{
	const bool show_hide = mnuItemShowCloudOctrees->IsChecked();

	try
	{
		wxBusyCursor wait;

		{
			auto openGLSceneRef = m_canvas->getOpenGLSceneRef();
			std::lock_guard<std::mutex> lock(critSec_UpdateScene);
			openGLSceneRef->visitAllObjects(&func_gather_stats);

			CSetOfObjects::Ptr gl_octrees_bb;

			// Get object from scene, or creat upon first usage:
			{
				CRenderizable::Ptr obj =
					openGLSceneRef->getByName(name_octrees_bb_globj);
				if (obj)
					gl_octrees_bb =
						std::dynamic_pointer_cast<CSetOfObjects>(obj);
				else
				{
					gl_octrees_bb = std::make_shared<CSetOfObjects>();
					gl_octrees_bb->setName(name_octrees_bb_globj);
					openGLSceneRef->insert(gl_octrees_bb);
				}
			}

			// Show or hide, clear first anyway:
			gl_octrees_bb->clear();

			if (show_hide)
			{
				// Show:
				aux_gl_octrees_bb = gl_octrees_bb;

				openGLSceneRef->visitAllObjects(func_get_octbb);

				aux_gl_octrees_bb.reset();
			}
		}

		Refresh(false);
	}
	catch (const std::exception& e)
	{
		wxMessageBox(mrpt::exception_to_str(e), _("Exception"), wxOK, this);
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
			this, _("Choose the PLY file to import"),
			(iniFile->read_string(iniFileSect, "LastDir", ".").c_str()),
			_("*.ply"),
			_("PLY files (*.ply, *.PLY)|*.ply;*.PLY|All files (*.*)|*.*"),
			wxFD_OPEN | wxFD_FILE_MUST_EXIST);

		if (dialog.ShowModal() != wxID_OK) return;

		const std::string fil = string(dialog.GetPath().mb_str());
		saveLastUsedDirectoryToCfgFile(fil);

		CDlgPLYOptions dlgPLY(this);
		if (dlgPLY.ShowModal() != wxID_OK) return;

		opengl::CPointCloud::Ptr gl_points;
		opengl::CPointCloudColoured::Ptr gl_points_col;
		mrpt::opengl::PLY_Importer* ply_obj = nullptr;

		if (dlgPLY.rbClass->GetSelection() == 0)
		{
			gl_points = std::make_shared<opengl::CPointCloud>();
			ply_obj = gl_points.get();
		}
		else
		{
			gl_points_col = std::make_shared<opengl::CPointCloudColoured>();
			ply_obj = gl_points_col.get();
		}

		std::vector<std::string> file_comments, file_info;

		bool res;
		{
			wxBusyCursor busy;
			res = ply_obj->loadFromPlyFile(fil, &file_comments, &file_info);
		}
		if (!res)
		{
			wxMessageBox(
				_("Error loading or parsing the PLY file"), _("Exception"),
				wxOK, this);
		}
		else
		{
			auto openGLSceneRef = m_canvas->getOpenGLSceneRef();
			// Set the point cloud as the only object in scene:
			openGLSceneRef = std::make_shared<opengl::COpenGLScene>();

			if (dlgPLY.cbXYGrid->GetValue())
			{
				mrpt::opengl::CGridPlaneXY::Ptr obj =
					mrpt::opengl::CGridPlaneXY::Create(-50, 50, -50, 50, 0, 1);
				obj->setColor(0.3f, 0.3f, 0.3f);
				openGLSceneRef->insert(obj);
			}

			if (dlgPLY.cbXYZ->GetValue())
				openGLSceneRef->insert(
					mrpt::opengl::stock_objects::CornerXYZ());

			double ptSize;
			dlgPLY.cbPointSize->GetStringSelection().ToDouble(&ptSize);
			if (gl_points) gl_points->setPointSize(ptSize);
			if (gl_points_col) gl_points_col->setPointSize(ptSize);

			if (gl_points)
			{
				switch (dlgPLY.rbIntFromXYZ->GetSelection())
				{
					case 1:
						gl_points->enableColorFromX();
						break;
					case 2:
						gl_points->enableColorFromY();
						break;
					case 3:
						gl_points->enableColorFromZ();
						break;
				};
			}

			TPose3D ptCloudPose(0, 0, 0, 0, 0, 0);

			dlgPLY.edYaw->GetValue().ToDouble(&ptCloudPose.yaw);
			dlgPLY.edPitch->GetValue().ToDouble(&ptCloudPose.pitch);
			dlgPLY.edRoll->GetValue().ToDouble(&ptCloudPose.roll);
			ptCloudPose.yaw = DEG2RAD(ptCloudPose.yaw);
			ptCloudPose.pitch = DEG2RAD(ptCloudPose.pitch);
			ptCloudPose.roll = DEG2RAD(ptCloudPose.roll);

			if (gl_points) gl_points->setPose(CPose3D(ptCloudPose));
			if (gl_points_col) gl_points_col->setPose(CPose3D(ptCloudPose));

			// Insert point cloud into scene:
			if (gl_points) openGLSceneRef->insert(gl_points);
			if (gl_points_col) openGLSceneRef->insert(gl_points_col);

			m_canvas->setCameraPointing(0.0f, 0.0f, 0.0f);
			m_canvas->setZoomDistance(4.0f);
			m_canvas->setAzimuthDegrees(45.0f);
			m_canvas->setElevationDegrees(30.0f);

			loadedFileName =
				std::string("Imported_") + fil + std::string(".3Dscene");
			updateTitle();

			Refresh(false);

			string sC, sI;
			mrpt::system::stringListAsString(file_comments, sC);
			mrpt::system::stringListAsString(file_info, sI);
			wxMessageBox(
				(format(
					 "Comments:\n--------------------\n%s\nObject "
					 "info:\n--------------------\n%s",
					 sC.c_str(), sI.c_str())
					 .c_str()),
				_("File info"), wxOK, this);
		}
	}
	catch (const std::exception& e)
	{
		wxMessageBox(mrpt::exception_to_str(e), _("Exception"), wxOK, this);
	}
}

struct visitor_export_PLY
{
	const string& filename;
	unsigned int& count;

	visitor_export_PLY(const string& fil, unsigned int& counter)
		: filename(fil), count(counter)
	{
	}

	void operator()(const mrpt::opengl::CRenderizable::Ptr& obj)
	{
		if (IS_CLASS(*obj, CPointCloud))
		{
			CPointCloud::Ptr o = std::dynamic_pointer_cast<CPointCloud>(obj);
			o->saveToPlyFile(format("%s_%03u.ply", filename.c_str(), ++count));
		}
		else if (IS_CLASS(*obj, CPointCloudColoured))
		{
			CPointCloudColoured::Ptr o =
				std::dynamic_pointer_cast<CPointCloudColoured>(obj);
			o->saveToPlyFile(format("%s_%03u.ply", filename.c_str(), ++count));
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
		wxMessageBox(
			_("Each point cloud object in the scene will be exported as a "
			  "separate PLY file."),
			_("Notice"));

		wxFileDialog dialog(
			this, _("Choose the target PLY filename"),
			(iniFile->read_string(iniFileSect, "LastDir", ".").c_str()),
			_("*.ply"),
			_("PLY files (*.ply, *.PLY)|*.ply;*.PLY|All files (*.*)|*.*"),
			wxFD_SAVE | wxFD_OVERWRITE_PROMPT);

		if (dialog.ShowModal() != wxID_OK) return;

		const std::string fil = string(dialog.GetPath().mb_str());
		saveLastUsedDirectoryToCfgFile(fil);

		// Go thru all point clouds and save them:
		unsigned int counter = 0;
		visitor_export_PLY visitor(fil, counter);

		{
			wxBusyCursor busy;
			m_canvas->getOpenGLSceneRef()->visitAllObjects(visitor);
		}

		wxMessageBox(
			wxString::Format(
				_("%u point cloud(s) exported to PLY files."), counter),
			_("Result"));
	}
	catch (const std::exception& e)
	{
		wxMessageBox(mrpt::exception_to_str(e), _("Exception"), wxOK, this);
	}
}

// Off-screen high-resolution render to image file:
void _DSceneViewerFrame::OnMenuItemHighResRender(wxCommandEvent& event)
{
	try
	{
		wxFileDialog dialog(
			this, _("Choose target image file"),
			(iniFile->read_string(iniFileSect, "LastDir", ".").c_str()),
			_("render.png"),
			_("Image files "
			  "(*.png,*.tif,*.jpg,...)|*.png;*.tif;*.jpg;*.bmp|All "
			  "files (*.*)|*.*"),
			wxFD_SAVE | wxFD_OVERWRITE_PROMPT);
		if (dialog.ShowModal() != wxID_OK) return;

		const std::string sTargetFil = string(dialog.GetPath().mb_str());
		saveLastUsedDirectoryToCfgFile(sTargetFil);

		{
			wxBusyCursor busy;

			// It seems wxGetNumberFromUser() leads to a crash with wxGTK
			// (tested with wx2.8.11)
			// So use wxGetTextFromUser() instead
			wxString ssW = wxGetTextFromUser(
				_("Render image size"), _("Width in pixels:"), _("1024"), this);
			std::string sW = std::string(ssW.mb_str());
			const long width = atoi(sW.c_str());

			wxString ssH = wxGetTextFromUser(
				_("Render image size"), _("Height in pixels:"), _("768"), this);
			std::string sH = std::string(ssH.mb_str());
			const long height = atoi(sH.c_str());

			CFBORender render(width, height, true /* skip Glut extra window */);
			CImage frame(width, height, CH_RGB);

			render.setBackgroundColor(mrpt::img::TColorf(
				m_canvas->getClearColorR(), m_canvas->getClearColorG(),
				m_canvas->getClearColorB(), 1.0));

			// render the scene
			render.getFrame(*m_canvas->getOpenGLSceneRef(), frame);

			frame.saveToFile(sTargetFil);
		}
	}
	catch (const std::exception& e)
	{
		wxMessageBox(mrpt::exception_to_str(e), _("Exception"), wxOK, this);
	}
}

// Select: none
void _DSceneViewerFrame::OnmnuSelectNoneSelected(wxCommandEvent& event)
{
	m_selected_gl_objects.clear();
}

class OpenGlObjectsFilterVirtual
{
   public:
	OpenGlObjectsFilterVirtual(
		std::vector<mrpt::opengl::CRenderizable::Ptr>& out_list)
		: m_out_list(out_list)
	{
	}

	virtual void operator()(const mrpt::opengl::CRenderizable::Ptr& obj)
	{
		if (checkObj(obj)) m_out_list.push_back(obj);
	}

	std::vector<mrpt::opengl::CRenderizable::Ptr>& m_out_list;

   protected:
	virtual bool checkObj(const mrpt::opengl::CRenderizable::Ptr& obj) = 0;
};

class OpenGlObjectsFilter_ByClass : public OpenGlObjectsFilterVirtual
{
   public:
	OpenGlObjectsFilter_ByClass(
		std::vector<mrpt::opengl::CRenderizable::Ptr>& out_list,
		const vector<const TRuntimeClassId*>& selected_classes)
		: OpenGlObjectsFilterVirtual(out_list),
		  m_selected_classes(selected_classes)
	{
	}

   protected:
	bool checkObj(const mrpt::opengl::CRenderizable::Ptr& obj) override
	{
		for (auto m_selected_classe : m_selected_classes)
		{
			if (obj->GetRuntimeClass() == m_selected_classe) return true;
		}
		return false;
	}

	const vector<const TRuntimeClassId*>& m_selected_classes;
};

// Select: by class
void _DSceneViewerFrame::OnmnuSelectByClassSelected(wxCommandEvent& event)
{
	static bool init_list = true;
	static wxArrayString glClassNames;
	if (init_list)
	{
		init_list = false;
		vector<const TRuntimeClassId*> all_mrpt_classes =
			mrpt::rtti::getAllRegisteredClasses();
		for (auto& all_mrpt_classe : all_mrpt_classes)
			if (all_mrpt_classe->derivedFrom(CLASS_ID(CRenderizable)))
				glClassNames.Add(all_mrpt_classe->className);
	}

	wxArrayInt selections;
#if wxCHECK_VERSION(2, 9, 0)
	wxGetSelectedChoices(
		selections, _("Select by class:"), _("Select objects"), glClassNames,
		this);
#else
	wxGetMultipleChoices(
		selections, _("Select by class:"), _("Select objects"), glClassNames,
		this);
#endif

	// Build list of classes IDs:
	vector<const TRuntimeClassId*> selected_classes;
	for (size_t i = 0; i < selections.Count(); i++)
	{
		const std::string sName =
			std::string(glClassNames[selections[i]].mb_str());
		selected_classes.push_back(mrpt::rtti::findRegisteredClass(sName));
	}

	// Go thru objects and do filter:
	OpenGlObjectsFilter_ByClass filter(m_selected_gl_objects, selected_classes);
	m_canvas->getOpenGLSceneRef()->visitAllObjects(filter);

	theWindow->StatusBar1->SetStatusText(
		(mrpt::format(
			 "%u objects selected",
			 static_cast<unsigned int>(m_selected_gl_objects.size()))
			 .c_str()),
		0);
}

// On selection: delete
void _DSceneViewerFrame::OnmnuSelectionDeleteSelected(wxCommandEvent& event)
{
	for (auto& m_selected_gl_object : m_selected_gl_objects)
		m_selected_gl_object.reset();
	Refresh(false);
}

// On selection: re-scale
void _DSceneViewerFrame::OnmnuSelectionScaleSelected(wxCommandEvent& event)
{
	wxString ssScale = wxGetTextFromUser(
		_("Multiply scale of selected objects"), _("Scale factor:"), _("1.0"),
		this);
	std::string sScale = std::string(ssScale.mb_str());
	const float s = atof(sScale.c_str());

	for (auto& m_selected_gl_object : m_selected_gl_objects)
		if (m_selected_gl_object)
		{
			const float sx = m_selected_gl_object->getScaleX();
			const float sy = m_selected_gl_object->getScaleY();
			const float sz = m_selected_gl_object->getScaleZ();
			m_selected_gl_object->setScale(s * sx, s * sy, s * sz);
		}
	Refresh(false);
}

// Import a LAS LiDAR file:
void _DSceneViewerFrame::OnmnuImportLASSelected(wxCommandEvent& event)
{
	try
	{
#if MRPT_HAS_LIBLAS
		wxFileDialog dialog(
			this, _("Choose the LAS file to import"),
			(iniFile->read_string(iniFileSect, "LastDir", ".").c_str()),
			_("*.las"),
			_("LAS files (*.las, "
			  "*.laz)|*.las;*.LAS;*.laz;*.LAZ|All files (*.*)|*.*"),
			wxFD_OPEN | wxFD_FILE_MUST_EXIST);

		if (dialog.ShowModal() != wxID_OK) return;

		const std::string fil = string(dialog.GetPath().mb_str());
		saveLastUsedDirectoryToCfgFile(fil);

		CDlgPLYOptions dlgPLY(this);

		// Default values for LAS:
		dlgPLY.SetTitle(_("LAS import options"));
		dlgPLY.edRoll->SetValue(_("0"));
		dlgPLY.rbIntFromXYZ->SetSelection(3);

		if (dlgPLY.ShowModal() != wxID_OK) return;

		opengl::CPointCloud::Ptr gl_points;
		opengl::CPointCloudColoured::Ptr gl_points_col;

		if (dlgPLY.rbClass->GetSelection() == 0)
			gl_points = std::make_shared<opengl::CPointCloud>();
		else
			gl_points_col = std::make_shared<opengl::CPointCloudColoured>();

		mrpt::maps::CColouredPointsMap pts_map;
		mrpt::maps::LAS_HeaderInfo las_hdr;

		bool res;
		{
			wxBusyCursor busy;
			res = mrpt::maps::loadLASFile(pts_map, fil, las_hdr);
		}

		if (!res)
		{
			wxMessageBox(
				_("Error loading or parsing the LAS file"), _("Exception"),
				wxOK, this);
			return;
		}

		mrpt::math::TPoint3D bb_min, bb_max;
		{
			wxBusyCursor busy;
			if (gl_points_col)
			{
				gl_points_col->loadFromPointsMap(&pts_map);
				gl_points_col->getBoundingBox(bb_min, bb_max);
			}
			else
			{
				gl_points->loadFromPointsMap(&pts_map);
				gl_points->getBoundingBox(bb_min, bb_max);
			}
		}

		const double scene_size = bb_min.distanceTo(bb_max);

		// Set the point cloud as the only object in scene:
		auto scene = std::make_shared<opengl::COpenGLScene>();
		m_canvas->setOpenGLSceneRef(scene);

		if (dlgPLY.cbXYGrid->GetValue())
		{
			mrpt::opengl::CGridPlaneXY::Ptr obj =
				mrpt::opengl::CGridPlaneXY::Create(
					bb_min.x, bb_max.x, bb_min.y, bb_max.y, 0,
					scene_size * 0.02);
			obj->setColor(0.3f, 0.3f, 0.3f);
			scene->insert(obj);
		}

		if (dlgPLY.cbXYZ->GetValue())
			scene->insert(mrpt::opengl::stock_objects::CornerXYZ());

		double ptSize;
		dlgPLY.cbPointSize->GetStringSelection().ToDouble(&ptSize);
		if (gl_points) gl_points->setPointSize(ptSize);
		if (gl_points_col) gl_points_col->setPointSize(ptSize);

		if (gl_points)
		{
			switch (dlgPLY.rbIntFromXYZ->GetSelection())
			{
				case 1:
					gl_points->enableColorFromX();
					break;
				case 2:
					gl_points->enableColorFromY();
					break;
				case 3:
					gl_points->enableColorFromZ();
					break;
			};
		}

		TPose3D ptCloudPose(0, 0, 0, 0, 0, 0);

		dlgPLY.edYaw->GetValue().ToDouble(&ptCloudPose.yaw);
		dlgPLY.edPitch->GetValue().ToDouble(&ptCloudPose.pitch);
		dlgPLY.edRoll->GetValue().ToDouble(&ptCloudPose.roll);
		ptCloudPose.yaw = DEG2RAD(ptCloudPose.yaw);
		ptCloudPose.pitch = DEG2RAD(ptCloudPose.pitch);
		ptCloudPose.roll = DEG2RAD(ptCloudPose.roll);

		if (gl_points) gl_points->setPose(CPose3D(ptCloudPose));
		if (gl_points_col) gl_points_col->setPose(CPose3D(ptCloudPose));

		// Insert point cloud into scene:
		if (gl_points) scene->insert(gl_points);
		if (gl_points_col) scene->insert(gl_points_col);

		m_canvas->setCameraPointing(
			(bb_min.x + bb_max.x) * 0.5, (bb_min.y + bb_max.y) * 0.5,
			(bb_min.z + bb_max.z) * 0.5);

		m_canvas->setZoomDistance(2 * scene_size);
		m_canvas->setAzimuthDegrees(45);
		m_canvas->setElevationDegrees(30);

		COpenGLViewport::Ptr gl_view = scene->getViewport();
		gl_view->setViewportClipDistances(0.01, 10 * scene_size);

		loadedFileName =
			std::string("Imported_") + fil + std::string(".3Dscene");
		updateTitle();

		// Typically, LAS scenes have millions of points:
		mrpt::global_settings::OCTREE_RENDER_MAX_DENSITY_POINTS_PER_SQPIXEL(
			1000);

		Refresh(false);

		std::stringstream ss;
		ss << pts_map.size() << " points loaded.\n"
		   << "Bounding box:\n"
		   << " X: " << bb_min.x << " <=> " << bb_max.x << "\n"
		   << " Y: " << bb_min.y << " <=> " << bb_max.y << "\n"
		   << " Z: " << bb_min.z << " <=> " << bb_max.z << "\n"
		   << "LAS header info:\n"
		   << "---------------------------------\n"
		   << "FileSignature      : " << las_hdr.FileSignature << endl
		   << "SystemIdentifier   : " << las_hdr.SystemIdentifier << endl
		   << "SoftwareIdentifier : " << las_hdr.SoftwareIdentifier << endl
		   << "project GUID       : " << las_hdr.project_guid << endl
		   << "SRS Proj.4         : " << las_hdr.spatial_reference_proj4 << endl
		   << "Creation date      : Year=" << las_hdr.creation_year
		   << " DOY=" << las_hdr.creation_DOY << endl;

		wxMessageBox(ss.str().c_str(), _("File info"), wxOK, this);
#else
		throw std::runtime_error("MRPT was built without libLAS support!");
#endif
	}
	catch (const std::exception& e)
	{
		wxMessageBox(mrpt::exception_to_str(e), _("Exception"), wxOK, this);
	}
}

void _DSceneViewerFrame::OnmnuImportImageView(wxCommandEvent&)
{
	try
	{
		auto scene = std::make_shared<opengl::COpenGLScene>();
		m_canvas->setOpenGLSceneRef(scene);
		wxFileDialog dialog(
			this, _("Choose the LAS file to import"),
			(iniFile->read_string(iniFileSect, "LastDir", ".").c_str()),
			_("*.png"),
			_("Image files (*.png, "
			  "*.jpg, *.bmp)|*.png;*.jpg;*.jpeg;*.bmp|All files (*.*)|*.*"),
			wxFD_OPEN | wxFD_FILE_MUST_EXIST);

		if (dialog.ShowModal() != wxID_OK) return;

		const std::string fil = string(dialog.GetPath().mb_str());
		saveLastUsedDirectoryToCfgFile(fil);

		mrpt::img::CImage im;
		if (!im.loadFromFile(fil))
			THROW_EXCEPTION_FMT("Error loading image file: '%s'", fil.c_str());

		COpenGLViewport::Ptr gl_view = scene->getViewport();
		gl_view->setImageView(im);

		Refresh();
	}
	catch (const std::exception& e)
	{
		wxMessageBox(mrpt::exception_to_str(e), _("Exception"), wxOK, this);
	}
}
