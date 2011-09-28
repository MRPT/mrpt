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

#include "xRawLogViewerMain.h"
#include "CAboutBox.h"
#include "CFormMotionModel.h"
#include "CFormPlayVideo.h"
#include "CFormRawMap.h"
#include "CFormEdit.h"
#include "CScanMatching.h"
#include "CFormChangeSensorPositions.h"
#include "COdometryParams.h"
#include "CScanAnimation.h"
#include "CFormBatchSensorPose.h"
#include "CIniEditor.h"

#include "../wx-common/CMyRedirector.h"

#include <wx/msgdlg.h>
#include <wx/filedlg.h>
#include <wx/progdlg.h>
#include <wx/imaglist.h>
#include <wx/busyinfo.h>
#include <wx/log.h>
#include <wx/textdlg.h>
#include <wx/dirdlg.h>
#include <wx/tipdlg.h>
#include <wx/statbox.h>

#include "xRawLogViewerApp.h"

#include <mrpt/gui/WxUtils.h>
#include <mrpt/vision/CVideoFileWriter.h>

//(*InternalHeaders(xRawLogViewerFrame)
#include <wx/string.h>
#include <wx/intl.h>
#include <wx/font.h>
#include <wx/bitmap.h>
#include <wx/icon.h>
#include <wx/image.h>
#include <wx/artprov.h>
//*)

#include <wx/tooltip.h>
#include <wx/config.h>
#include <wx/choicdlg.h>
#include <wx/choice.h>
#include <wx/image.h>

#include "imgs/tree_icon1.xpm"
#include "imgs/tree_icon2.xpm"
#include "imgs/tree_icon3.xpm"
#include "imgs/tree_icon4.xpm"
#include "imgs/tree_icon5.xpm"
#include "imgs/tree_icon6.xpm"
#include "imgs/tree_icon7.xpm"
#include "imgs/tree_icon8.xpm"
#include "imgs/tree_icon9.xpm"


//-----------------------------------------------------------------------------
// wxStaticBitmapPopup
//-----------------------------------------------------------------------------
const long wxStaticBitmapPopup::ID_MENUITEM_IMG_LOAD = wxNewId();
const long wxStaticBitmapPopup::ID_MENUITEM_IMG_SAVE = wxNewId();

IMPLEMENT_DYNAMIC_CLASS(wxStaticBitmapPopup, wxStaticBitmap)

BEGIN_EVENT_TABLE(wxStaticBitmapPopup, wxStaticBitmap)
	EVT_MIDDLE_UP( wxStaticBitmapPopup::OnShowPopupMenu)
	EVT_RIGHT_UP ( wxStaticBitmapPopup::OnShowPopupMenu)
	EVT_MENU( ID_MENUITEM_IMG_SAVE, wxStaticBitmapPopup::OnPopupSaveImage  )
	EVT_MENU( ID_MENUITEM_IMG_LOAD, wxStaticBitmapPopup::OnPopupLoadImage  )
END_EVENT_TABLE()

// Constructors
wxStaticBitmapPopup::wxStaticBitmapPopup( wxWindow *parent, wxWindowID id, const wxBitmap&img, const wxPoint &pos, const wxSize &size, int flag,const wxString &name )
		: wxStaticBitmap( parent, id, img, pos, size, flag, name )
{
	wxMenuItem *mnu1 = new wxMenuItem((&mnuImages), ID_MENUITEM_IMG_SAVE, _("Save image to file..."), wxEmptyString, wxITEM_NORMAL);
	wxMenuItem *mnu2 = new wxMenuItem((&mnuImages), ID_MENUITEM_IMG_LOAD, _("Replace image by another one from file..."), wxEmptyString, wxITEM_NORMAL);

	mnuImages.Append( mnu1 );
	mnuImages.Append( mnu2 );
}
wxStaticBitmapPopup::~wxStaticBitmapPopup( )
{
}
void wxStaticBitmapPopup::OnShowPopupMenu(wxMouseEvent &event)
{
	PopupMenu( &mnuImages );
}

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace mrpt::vision;
using namespace std;

CRawlog			rawlog;
TTimeStamp		rawlog_first_timestamp = INVALID_TIMESTAMP;

CTicTac                         crono_Loading;
string                     		loadedFileName = "noname.rawlog";
float                           timeToLoad =0;
double							experimentLenght=0;

xRawLogViewerFrame				*theMainWindow = NULL;

extern CConfigFile              *iniFile;

// As a global variable to keep the user selections in memory.
CFormRawMap                     *formRawMap = NULL;
CScanMatching       			*scanMatchingDialog=NULL;

extern string              global_fileToOpen;




#if defined(__WXMSW__)
string             iniFileSect("CONF_WIN");
#elif defined(__UNIX__)
string             iniFileSect("CONF_LIN");
#endif

//helper functions
enum wxbuildinfoformat
{
	short_f, long_f
};

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

//(*IdInit(xRawLogViewerFrame)
const long xRawLogViewerFrame::ID_CUSTOM5 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL1 = wxNewId();
const long xRawLogViewerFrame::ID_TEXTCTRL1 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL3 = wxNewId();
const long xRawLogViewerFrame::ID_BUTTON1 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL18 = wxNewId();
const long xRawLogViewerFrame::ID_TEXTCTRL2 = wxNewId();
const long xRawLogViewerFrame::ID_TEXTCTRL3 = wxNewId();
const long xRawLogViewerFrame::ID_NOTEBOOK3 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL6 = wxNewId();
const long xRawLogViewerFrame::ID_CUSTOM2 = wxNewId();
const long xRawLogViewerFrame::ID_CUSTOM3 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL7 = wxNewId();
const long xRawLogViewerFrame::ID_CUSTOM1 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL4 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL8 = wxNewId();
const long xRawLogViewerFrame::ID_STATICTEXT2 = wxNewId();
const long xRawLogViewerFrame::ID_STATICBITMAP1 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL9 = wxNewId();
const long xRawLogViewerFrame::ID_STATICTEXT1 = wxNewId();
const long xRawLogViewerFrame::ID_STATICBITMAP2 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL13 = wxNewId();
const long xRawLogViewerFrame::ID_STATICBITMAP3 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL14 = wxNewId();
const long xRawLogViewerFrame::ID_STATICBITMAP7 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL24 = wxNewId();
const long xRawLogViewerFrame::ID_NOTEBOOK2 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL10 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL11 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL12 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL15 = wxNewId();
const long xRawLogViewerFrame::ID_CUSTOM4 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL17 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL16 = wxNewId();
const long xRawLogViewerFrame::ID_XY_GLCANVAS = wxNewId();
const long xRawLogViewerFrame::ID_STATICTEXT3 = wxNewId();
const long xRawLogViewerFrame::ID_SLIDER1 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL20 = wxNewId();
const long xRawLogViewerFrame::ID_STATICBITMAP4 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL21 = wxNewId();
const long xRawLogViewerFrame::ID_STATICBITMAP5 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL22 = wxNewId();
const long xRawLogViewerFrame::ID_STATICBITMAP6 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL23 = wxNewId();
const long xRawLogViewerFrame::ID_NOTEBOOK4 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL19 = wxNewId();
const long xRawLogViewerFrame::ID_NOTEBOOK1 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL5 = wxNewId();
const long xRawLogViewerFrame::ID_SPLITTERWINDOW3 = wxNewId();
const long xRawLogViewerFrame::ID_PANEL2 = wxNewId();
const long xRawLogViewerFrame::ID_SPLITTERWINDOW1 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM1 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM2 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM11 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM4 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM76 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM7 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM8 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM10 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM62 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM64 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM13 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM60 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM61 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM6 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM5 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM47 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM56 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM63 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM87 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM3 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM58 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM55 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM54 = wxNewId();
const long xRawLogViewerFrame::idMenuQuit = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM14 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM51 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM69 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM15 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM70 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM16 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM59 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM57 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM75 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM67 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM68 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM82 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM20 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM22 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM53 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM23 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM41 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM84 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM12 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM17 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM44 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM19 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM25 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM73 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM74 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM77 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM79 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM18 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM86 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM90 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM85 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM29 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM9 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM28 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM71 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM72 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM78 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM83 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM21 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM30 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM24 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM35 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM31 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM34 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM65 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM66 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM52 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM80 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM36 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM33 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM38 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM37 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM40 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM81 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM39 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM46 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM45 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM43 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM42 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM89 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM88 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM26 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM32 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM27 = wxNewId();
const long xRawLogViewerFrame::idMenuAbout = wxNewId();
const long xRawLogViewerFrame::ID_STATUSBAR1 = wxNewId();
const long xRawLogViewerFrame::ID_TOOLBARITEM1 = wxNewId();
const long xRawLogViewerFrame::ID_TOOLBARITEM2 = wxNewId();
const long xRawLogViewerFrame::ID_TOOLBARITEM3 = wxNewId();
const long xRawLogViewerFrame::ID_TOOLBARITEM4 = wxNewId();
const long xRawLogViewerFrame::ID_TOOLBARITEM5 = wxNewId();
const long xRawLogViewerFrame::ID_TOOLBARITEM9 = wxNewId();
const long xRawLogViewerFrame::ID_TOOLBARITEM10 = wxNewId();
const long xRawLogViewerFrame::ID_TOOLBARITEM6 = wxNewId();
const long xRawLogViewerFrame::ID_TOOLBARITEM7 = wxNewId();
const long xRawLogViewerFrame::ID_TOOLBARITEM8 = wxNewId();
const long xRawLogViewerFrame::ID_TOOLBAR1 = wxNewId();
const long xRawLogViewerFrame::MNU_1 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM49 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM50 = wxNewId();
const long xRawLogViewerFrame::ID_MENUITEM48 = wxNewId();
const long xRawLogViewerFrame::ID_TIMER1 = wxNewId();
//*)

const long xRawLogViewerFrame::ID_COMBO_IMG_DIRS = wxNewId();


BEGIN_EVENT_TABLE(xRawLogViewerFrame,wxFrame)
	//(*EventTable(xRawLogViewerFrame)
	//*)
	EVT_MENU_RANGE(wxID_FILE1, wxID_FILE9, xRawLogViewerFrame::OnMRUFile)
END_EVENT_TABLE()


xRawLogViewerFrame::xRawLogViewerFrame(wxWindow* parent,wxWindowID id)
		: m_fileHistory( 9 /* Max file list*/ )
{
	theMainWindow = this;

	// Load my custom icons:
	wxArtProvider::Push(new MyArtProvider);

	//(*Initialize(xRawLogViewerFrame)
	wxMenuItem* MenuItem2;
	wxMenu* MenuItem81;
	wxMenuItem* MenuItem1;
	wxFlexGridSizer* FlexGridSizer8;
	wxFlexGridSizer* FlexGridSizer1;
	wxFlexGridSizer* FlexGridSizer2;
	wxMenuItem* MenuItem43;
	wxMenu* Menu39;
	wxMenuItem* MenuItem75;
	wxMenu* Menu1;
	wxMenuItem* MenuItem12;
	wxMenuItem* MenuItem25;
	wxMenuItem* MenuItem55;
	wxMenuItem* MenuItem19;
	wxFlexGridSizer* FlexGridSizer11;
	wxMenuItem* MenuItem65;
	wxMenuItem* MenuItem56;
	wxMenuItem* MenuItem67;
	wxMenuItem* MenuItem33;
	wxFlexGridSizer* FlexGridSizer7;
	wxMenuItem* MenuItem24;
	wxFlexGridSizer* FlexGridSizer4;
	wxMenuItem* MenuItem69;
	wxFlexGridSizer* FlexGridSizer9;
	wxFlexGridSizer* FlexGridSizer6;
	wxMenuItem* MenuItem17;
	wxFlexGridSizer* FlexGridSizer3;
	wxMenuItem* MenuItem60;
	wxMenuItem* MenuItem22;
	wxMenuItem* MenuItem41;
	wxFlexGridSizer* FlexGridSizer10;
	wxMenuItem* MenuItem82;
	wxFlexGridSizer* FlexGridSizer13;
	wxMenuBar* MenuBar1;
	wxMenuItem* MenuItem27;
	wxMenuItem* MenuItem18;
	wxMenuItem* MenuItem26;
	wxMenuItem* MenuItem70;
	wxFlexGridSizer* FlexGridSizer12;
	wxMenu* Menu2;
	wxFlexGridSizer* FlexGridSizer5;

	Create(parent, id, _("RawlogViewer - Part of the MRPT project"), wxDefaultPosition, wxDefaultSize, wxCAPTION|wxDEFAULT_FRAME_STYLE|wxSYSTEM_MENU|wxRESIZE_BORDER|wxCLOSE_BOX|wxMAXIMIZE_BOX|wxMINIMIZE_BOX, _T("id"));
	SetClientSize(wxSize(700,500));
	{
	wxIcon FrameIcon;
	FrameIcon.CopyFromBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("MAIN_ICON")),wxART_OTHER));
	SetIcon(FrameIcon);
	}
	SplitterWindow1 = new wxSplitterWindow(this, ID_SPLITTERWINDOW1, wxDefaultPosition, wxDefaultSize, wxSP_3D|wxSP_3DBORDER|wxSP_LIVE_UPDATE, _T("ID_SPLITTERWINDOW1"));
	SplitterWindow1->SetMinSize(wxSize(10,10));
	SplitterWindow1->SetMinimumPaneSize(10);
	Panel1 = new wxPanel(SplitterWindow1, ID_PANEL1, wxDefaultPosition, wxSize(337,-1), wxTAB_TRAVERSAL, _T("ID_PANEL1"));
	FlexGridSizer1 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer1->AddGrowableCol(0);
	FlexGridSizer1->AddGrowableRow(0);
	tree_view = new CRawlogTreeView(Panel1,ID_CUSTOM5,wxDefaultPosition,wxDefaultSize,wxVSCROLL,_T("ID_CUSTOM5"));
	FlexGridSizer1->Add(tree_view, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
	Panel1->SetSizer(FlexGridSizer1);
	FlexGridSizer1->SetSizeHints(Panel1);
	Panel2 = new wxPanel(SplitterWindow1, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
	BoxSizer1 = new wxBoxSizer(wxHORIZONTAL);
	SplitterWindow3 = new wxSplitterWindow(Panel2, ID_SPLITTERWINDOW3, wxDefaultPosition, wxDefaultSize, wxSP_3D, _T("ID_SPLITTERWINDOW3"));
	SplitterWindow3->SetMinSize(wxSize(10,10));
	SplitterWindow3->SetMinimumPaneSize(10);
	Panel3 = new wxPanel(SplitterWindow3, ID_PANEL3, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL3"));
	BoxSizer2 = new wxBoxSizer(wxHORIZONTAL);
	memo = new wxTextCtrl(Panel3, ID_TEXTCTRL1, wxEmptyString, wxDefaultPosition, wxSize(327,140), wxTE_MULTILINE|wxTE_READONLY|wxTE_WORDWRAP|wxNO_BORDER|wxVSCROLL, wxDefaultValidator, _T("ID_TEXTCTRL1"));
	wxFont memoFont(10,wxTELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,_T("Courier 10 Pitch"),wxFONTENCODING_DEFAULT);
	memo->SetFont(memoFont);
	BoxSizer2->Add(memo, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
	Panel3->SetSizer(BoxSizer2);
	BoxSizer2->Fit(Panel3);
	BoxSizer2->SetSizeHints(Panel3);
	Panel5 = new wxPanel(SplitterWindow3, ID_PANEL5, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL5"));
	BoxSizer3 = new wxBoxSizer(wxHORIZONTAL);
	Notebook1 = new wxNotebook(Panel5, ID_NOTEBOOK1, wxDefaultPosition, wxDefaultSize, wxNB_BOTTOM, _T("ID_NOTEBOOK1"));
	pn_CSensorialFrame = new wxPanel(Notebook1, ID_PANEL6, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL6"));
	FlexGridSizer6 = new wxFlexGridSizer(2, 1, 0, 0);
	FlexGridSizer6->AddGrowableCol(0);
	FlexGridSizer6->AddGrowableRow(1);
	Panel9 = new wxPanel(pn_CSensorialFrame, ID_PANEL18, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL18"));
	btnEditComments = new wxButton(Panel9, ID_BUTTON1, _("Edit comments..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
	FlexGridSizer6->Add(Panel9, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
	Notebook3 = new wxNotebook(pn_CSensorialFrame, ID_NOTEBOOK3, wxDefaultPosition, wxSize(-1,150), 0, _T("ID_NOTEBOOK3"));
	Notebook3->SetMinSize(wxSize(-1,150));
	memStats = new wxTextCtrl(Notebook3, ID_TEXTCTRL2, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE|wxTE_READONLY|wxVSCROLL, wxDefaultValidator, _T("ID_TEXTCTRL2"));
	wxFont memStatsFont(10,wxTELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,_T("Monospace"),wxFONTENCODING_DEFAULT);
	memStats->SetFont(memStatsFont);
	memStats->SetToolTip(_("Statistics of the rawlog load"));
	txtException = new wxTextCtrl(Notebook3, ID_TEXTCTRL3, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE|wxTE_READONLY, wxDefaultValidator, _T("ID_TEXTCTRL3"));
	wxFont txtExceptionFont(10,wxTELETYPE,wxFONTSTYLE_NORMAL,wxNORMAL,false,_T("Monospace"),wxFONTENCODING_DEFAULT);
	txtException->SetFont(txtExceptionFont);
	Notebook3->AddPage(memStats, _("Loaded rawlog info"), false);
	Notebook3->AddPage(txtException, _("End of load message"), false);
	FlexGridSizer6->Add(Notebook3, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
	pn_CSensorialFrame->SetSizer(FlexGridSizer6);
	FlexGridSizer6->Fit(pn_CSensorialFrame);
	FlexGridSizer6->SetSizeHints(pn_CSensorialFrame);
	pn_Action = new wxPanel(Notebook1, ID_PANEL7, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL7"));
	BoxSizer8 = new wxBoxSizer(wxHORIZONTAL);
	plotAct2D_XY = new mpWindow(pn_Action,ID_CUSTOM2,wxDefaultPosition,wxDefaultSize,0);
	BoxSizer8->Add(plotAct2D_XY, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
	plotAct2D_PHI = new mpWindow(pn_Action,ID_CUSTOM3,wxDefaultPosition,wxDefaultSize,0);
	BoxSizer8->Add(plotAct2D_PHI, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
	pn_Action->SetSizer(BoxSizer8);
	BoxSizer8->Fit(pn_Action);
	BoxSizer8->SetSizeHints(pn_Action);
	pn_CObservation2DRangeScan = new wxPanel(Notebook1, ID_PANEL8, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL8"));
	BoxSizer5 = new wxBoxSizer(wxVERTICAL);
	plotScan2D = new mpWindow(pn_CObservation2DRangeScan,ID_CUSTOM1,wxDefaultPosition,wxDefaultSize,0);
	BoxSizer5->Add(plotScan2D, 10, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
	Panel4 = new wxPanel(pn_CObservation2DRangeScan, ID_PANEL4, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL4"));
	BoxSizer5->Add(Panel4, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
	pn_CObservation2DRangeScan->SetSizer(BoxSizer5);
	BoxSizer5->Fit(pn_CObservation2DRangeScan);
	BoxSizer5->SetSizeHints(pn_CObservation2DRangeScan);
	pn_CObservationImage = new wxPanel(Notebook1, ID_PANEL9, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL9"));
	FlexGridSizer3 = new wxFlexGridSizer(2, 1, 0, 0);
	FlexGridSizer3->AddGrowableCol(0);
	FlexGridSizer3->AddGrowableRow(1);
	StaticText2 = new wxStaticText(pn_CObservationImage, ID_STATICTEXT2, _("Right click on the images to see the popup menu:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
	FlexGridSizer3->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	bmpObsImage = new wxStaticBitmapPopup(pn_CObservationImage, ID_STATICBITMAP1, wxNullBitmap, wxPoint(0,0), wxDefaultSize, wxFULL_REPAINT_ON_RESIZE, _T("ID_STATICBITMAP1"));
	FlexGridSizer3->Add(bmpObsImage, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
	pn_CObservationImage->SetSizer(FlexGridSizer3);
	FlexGridSizer3->Fit(pn_CObservationImage);
	FlexGridSizer3->SetSizeHints(pn_CObservationImage);
	pn_CObservationStereoImage = new wxPanel(Notebook1, ID_PANEL10, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL|wxFULL_REPAINT_ON_RESIZE, _T("ID_PANEL10"));
	FlexGridSizer2 = new wxFlexGridSizer(2, 1, 0, 0);
	FlexGridSizer2->AddGrowableCol(0);
	FlexGridSizer2->AddGrowableRow(1);
	StaticText1 = new wxStaticText(pn_CObservationStereoImage, ID_STATICTEXT1, _("Right click on the images to see the popup menu:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
	FlexGridSizer2->Add(StaticText1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	Notebook2 = new wxNotebook(pn_CObservationStereoImage, ID_NOTEBOOK2, wxDefaultPosition, wxDefaultSize, 0, _T("ID_NOTEBOOK2"));
	Panel6 = new wxPanel(Notebook2, ID_PANEL13, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL13"));
	FlexGridSizer4 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer4->AddGrowableCol(0);
	FlexGridSizer4->AddGrowableRow(0);
	bmpObsStereoLeft = new wxStaticBitmapPopup(Panel6, ID_STATICBITMAP2, wxNullBitmap, wxPoint(0,0), wxDefaultSize, wxFULL_REPAINT_ON_RESIZE, _T("ID_STATICBITMAP2"));
	FlexGridSizer4->Add(bmpObsStereoLeft, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 0);
	Panel6->SetSizer(FlexGridSizer4);
	FlexGridSizer4->Fit(Panel6);
	FlexGridSizer4->SetSizeHints(Panel6);
	Panel7 = new wxPanel(Notebook2, ID_PANEL14, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL14"));
	FlexGridSizer5 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer5->AddGrowableCol(0);
	FlexGridSizer5->AddGrowableRow(0);
	bmpObsStereoRight = new wxStaticBitmapPopup(Panel7, ID_STATICBITMAP3, wxNullBitmap, wxPoint(0,0), wxDefaultSize, wxFULL_REPAINT_ON_RESIZE, _T("ID_STATICBITMAP3"));
	FlexGridSizer5->Add(bmpObsStereoRight, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 0);
	Panel7->SetSizer(FlexGridSizer5);
	FlexGridSizer5->Fit(Panel7);
	FlexGridSizer5->SetSizeHints(Panel7);
	Panel10 = new wxPanel(Notebook2, ID_PANEL24, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL24"));
	FlexGridSizer13 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer13->AddGrowableCol(0);
	FlexGridSizer13->AddGrowableRow(0);
	bmpObsStereoDisp = new wxStaticBitmapPopup(Panel10, ID_STATICBITMAP7, wxNullBitmap, wxPoint(0,0), wxDefaultSize, wxFULL_REPAINT_ON_RESIZE, _T("ID_STATICBITMAP7"));
	FlexGridSizer13->Add(bmpObsStereoDisp, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 0);
	Panel10->SetSizer(FlexGridSizer13);
	FlexGridSizer13->Fit(Panel10);
	FlexGridSizer13->SetSizeHints(Panel10);
	Notebook2->AddPage(Panel6, _("Left"), false);
	Notebook2->AddPage(Panel7, _("Right"), false);
	Notebook2->AddPage(Panel10, _("Disparity"), false);
	FlexGridSizer2->Add(Notebook2, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
	pn_CObservationStereoImage->SetSizer(FlexGridSizer2);
	FlexGridSizer2->Fit(pn_CObservationStereoImage);
	FlexGridSizer2->SetSizeHints(pn_CObservationStereoImage);
	pn_CObservationBeaconRanges = new wxPanel(Notebook1, ID_PANEL11, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL11"));
	pn_CObservationGasSensors = new wxPanel(Notebook1, ID_PANEL12, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL12"));
	pn_CObservationGPS = new wxPanel(Notebook1, ID_PANEL15, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL15"));
	pn_CObservationBearingRange = new wxPanel(Notebook1, ID_PANEL16, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL16"));
	BoxSizer4 = new wxBoxSizer(wxVERTICAL);
	plotRangeBearing = new mpWindow(pn_CObservationBearingRange,ID_CUSTOM4,wxDefaultPosition,wxDefaultSize,0);
	BoxSizer4->Add(plotRangeBearing, 10, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
	Panel8 = new wxPanel(pn_CObservationBearingRange, ID_PANEL17, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL17"));
	BoxSizer4->Add(Panel8, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
	pn_CObservationBearingRange->SetSizer(BoxSizer4);
	BoxSizer4->Fit(pn_CObservationBearingRange);
	BoxSizer4->SetSizeHints(pn_CObservationBearingRange);
	pn_CObservation3DRangeScan = new wxPanel(Notebook1, ID_PANEL19, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL19"));
	FlexGridSizer7 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer7->AddGrowableCol(0);
	FlexGridSizer7->AddGrowableRow(0);
	nb_3DObsChannels = new wxNotebook(pn_CObservation3DRangeScan, ID_NOTEBOOK4, wxDefaultPosition, wxDefaultSize, 0, _T("ID_NOTEBOOK4"));
	pn3Dobs_3D = new wxPanel(nb_3DObsChannels, ID_PANEL20, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL20"));
	FlexGridSizer8 = new wxFlexGridSizer(1, 2, 0, 0);
	FlexGridSizer8->AddGrowableCol(0);
	FlexGridSizer8->AddGrowableRow(0);
	m_gl3DRangeScan = new CMyGLCanvas(pn3Dobs_3D,ID_XY_GLCANVAS,wxDefaultPosition,wxDefaultSize,wxTAB_TRAVERSAL,_T("ID_XY_GLCANVAS"));
	FlexGridSizer8->Add(m_gl3DRangeScan, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
	FlexGridSizer12 = new wxFlexGridSizer(2, 1, 0, 0);
	FlexGridSizer12->AddGrowableCol(0);
	FlexGridSizer12->AddGrowableRow(1);
	StaticText3 = new wxStaticText(pn3Dobs_3D, ID_STATICTEXT3, _("Min.conf."), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
	FlexGridSizer12->Add(StaticText3, 1, wxALL|wxALIGN_LEFT|wxALIGN_BOTTOM, 5);
	slid3DcamConf = new wxSlider(pn3Dobs_3D, ID_SLIDER1, 127, 0, 255, wxDefaultPosition, wxDefaultSize, wxSL_VERTICAL|wxSL_INVERSE, wxDefaultValidator, _T("ID_SLIDER1"));
	FlexGridSizer12->Add(slid3DcamConf, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 5);
	FlexGridSizer8->Add(FlexGridSizer12, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 0);
	pn3Dobs_3D->SetSizer(FlexGridSizer8);
	FlexGridSizer8->Fit(pn3Dobs_3D);
	FlexGridSizer8->SetSizeHints(pn3Dobs_3D);
	pn3Dobs_Depth = new wxPanel(nb_3DObsChannels, ID_PANEL21, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL21"));
	FlexGridSizer9 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer9->AddGrowableCol(0);
	FlexGridSizer9->AddGrowableRow(0);
	bmp3Dobs_depth = new wxStaticBitmapPopup(pn3Dobs_Depth, ID_STATICBITMAP4, wxNullBitmap, wxPoint(0,0), wxDefaultSize, wxFULL_REPAINT_ON_RESIZE, _T("ID_STATICBITMAP4"));
	FlexGridSizer9->Add(bmp3Dobs_depth, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
	pn3Dobs_Depth->SetSizer(FlexGridSizer9);
	FlexGridSizer9->Fit(pn3Dobs_Depth);
	FlexGridSizer9->SetSizeHints(pn3Dobs_Depth);
	pn3Dobs_Int = new wxPanel(nb_3DObsChannels, ID_PANEL22, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL22"));
	FlexGridSizer10 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer10->AddGrowableCol(0);
	FlexGridSizer10->AddGrowableRow(0);
	bmp3Dobs_int = new wxStaticBitmapPopup(pn3Dobs_Int, ID_STATICBITMAP5, wxNullBitmap, wxPoint(0,0), wxDefaultSize, wxFULL_REPAINT_ON_RESIZE, _T("ID_STATICBITMAP5"));
	FlexGridSizer10->Add(bmp3Dobs_int, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
	pn3Dobs_Int->SetSizer(FlexGridSizer10);
	FlexGridSizer10->Fit(pn3Dobs_Int);
	FlexGridSizer10->SetSizeHints(pn3Dobs_Int);
	pn3Dobs_Conf = new wxPanel(nb_3DObsChannels, ID_PANEL23, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL23"));
	FlexGridSizer11 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer11->AddGrowableCol(0);
	FlexGridSizer11->AddGrowableRow(0);
	bmp3Dobs_conf = new wxStaticBitmapPopup(pn3Dobs_Conf, ID_STATICBITMAP6, wxNullBitmap, wxPoint(0,0), wxDefaultSize, wxFULL_REPAINT_ON_RESIZE, _T("ID_STATICBITMAP6"));
	FlexGridSizer11->Add(bmp3Dobs_conf, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 5);
	pn3Dobs_Conf->SetSizer(FlexGridSizer11);
	FlexGridSizer11->Fit(pn3Dobs_Conf);
	FlexGridSizer11->SetSizeHints(pn3Dobs_Conf);
	nb_3DObsChannels->AddPage(pn3Dobs_3D, _("3D points"), false);
	nb_3DObsChannels->AddPage(pn3Dobs_Depth, _("Depth"), false);
	nb_3DObsChannels->AddPage(pn3Dobs_Int, _("Intensity"), false);
	nb_3DObsChannels->AddPage(pn3Dobs_Conf, _("Confidence"), false);
	FlexGridSizer7->Add(nb_3DObsChannels, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 1);
	pn_CObservation3DRangeScan->SetSizer(FlexGridSizer7);
	FlexGridSizer7->Fit(pn_CObservation3DRangeScan);
	FlexGridSizer7->SetSizeHints(pn_CObservation3DRangeScan);
	Notebook1->AddPage(pn_CSensorialFrame, _("Rawlog information"), true);
	Notebook1->AddPage(pn_Action, _("2D Mov. Action"), false);
	Notebook1->AddPage(pn_CObservation2DRangeScan, _("Obs: 2D scan"), false);
	Notebook1->AddPage(pn_CObservationImage, _("Obs: Image"), false);
	Notebook1->AddPage(pn_CObservationStereoImage, _("Obs: Stereo Image"), false);
	Notebook1->AddPage(pn_CObservationBeaconRanges, _("Obs: Beacon Ranges"), false);
	Notebook1->AddPage(pn_CObservationGasSensors, _("Obs: e-Noses"), false);
	Notebook1->AddPage(pn_CObservationGPS, _("Obs: GPS"), false);
	Notebook1->AddPage(pn_CObservationBearingRange, _("Obs: RangeBearing"), false);
	Notebook1->AddPage(pn_CObservation3DRangeScan, _("Obs: 3D range scan"), false);
	BoxSizer3->Add(Notebook1, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
	Panel5->SetSizer(BoxSizer3);
	BoxSizer3->Fit(Panel5);
	BoxSizer3->SetSizeHints(Panel5);
	SplitterWindow3->SplitHorizontally(Panel3, Panel5);
	BoxSizer1->Add(SplitterWindow3, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
	Panel2->SetSizer(BoxSizer1);
	BoxSizer1->Fit(Panel2);
	BoxSizer1->SetSizeHints(Panel2);
	SplitterWindow1->SplitVertically(Panel1, Panel2);
	MenuBar1 = new wxMenuBar();
	Menu1 = new wxMenu();
	MenuItem3 = new wxMenuItem(Menu1, ID_MENUITEM1, _("Open..."), _("Loads a whole rawlog file in memory"), wxITEM_NORMAL);
	MenuItem3->SetBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FOLDER")),wxART_MENU));
	Menu1->Append(MenuItem3);
	MenuItem4 = new wxMenuItem(Menu1, ID_MENUITEM2, _("Save as..."), _("Saves the current rawlog in a file"), wxITEM_NORMAL);
	MenuItem4->SetBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FILE_SAVE_AS")),wxART_MENU));
	Menu1->Append(MenuItem4);
	MenuItem6 = new wxMenu();
	MenuItem13 = new wxMenuItem(MenuItem6, ID_MENUITEM11, _("New Menu"), wxEmptyString, wxITEM_NORMAL);
	MenuItem6->Append(MenuItem13);
	Menu1->Append(ID_MENUITEM4, _("Open recent"), MenuItem6, wxEmptyString);
	MenuItem73 = new wxMenuItem(Menu1, ID_MENUITEM76, _("Revert"), _("Reload the currently open file, discarding changes"), wxITEM_NORMAL);
	Menu1->Append(MenuItem73);
	MenuItem8 = new wxMenu();
	MenuItem9 = new wxMenuItem(MenuItem8, ID_MENUITEM7, _("Load a part only..."), _("Load a section of a long rawlog"), wxITEM_NORMAL);
	MenuItem8->Append(MenuItem9);
	MenuItem10 = new wxMenuItem(MenuItem8, ID_MENUITEM8, _("Count entries..."), _("Count the entries in a rawlog file"), wxITEM_NORMAL);
	MenuItem8->Append(MenuItem10);
	MenuItem12 = new wxMenuItem(MenuItem8, ID_MENUITEM10, _("Save all images to files.."), _("Save all the images in a rawlog to a given directory"), wxITEM_NORMAL);
	MenuItem8->Append(MenuItem12);
	MenuItem59 = new wxMenuItem(MenuItem8, ID_MENUITEM62, _("Conver to externally stored images..."), wxEmptyString, wxITEM_NORMAL);
	MenuItem8->Append(MenuItem59);
	MenuItem61 = new wxMenuItem(MenuItem8, ID_MENUITEM64, _("Convert to observations-only rawlog..."), wxEmptyString, wxITEM_NORMAL);
	MenuItem8->Append(MenuItem61);
	MenuItem8->AppendSeparator();
	MenuItem15 = new wxMenuItem(MenuItem8, ID_MENUITEM13, _("Append visual landmarks from stereo imgs..."), _("Precompute 3D landmarks from stereo images and save in another rawlog"), wxITEM_NORMAL);
	MenuItem8->Append(MenuItem15);
	MenuItem8->AppendSeparator();
	MenuItem57 = new wxMenuItem(MenuItem8, ID_MENUITEM60, _("Loss-less decimation..."), _("Accumulate observations from many timesteps in one"), wxITEM_NORMAL);
	MenuItem8->Append(MenuItem57);
	MenuItem58 = new wxMenuItem(MenuItem8, ID_MENUITEM61, _("Compact rawlog..."), _("Group consecutive actions & observations"), wxITEM_NORMAL);
	MenuItem8->Append(MenuItem58);
	Menu1->Append(ID_MENUITEM6, _("Operations on files"), MenuItem8, wxEmptyString);
	Menu1->AppendSeparator();
	MenuItem5 = new wxMenu();
	MenuItem7 = new wxMenuItem(MenuItem5, ID_MENUITEM5, _("a CARMEN log..."), _("Import a CARMEN \"log\" file"), wxITEM_NORMAL);
	MenuItem5->Append(MenuItem7);
	MenuItem44 = new wxMenuItem(MenuItem5, ID_MENUITEM47, _("a sequence of image files..."), wxEmptyString, wxITEM_NORMAL);
	MenuItem5->Append(MenuItem44);
	MenuItem53 = new wxMenuItem(MenuItem5, ID_MENUITEM56, _("a MOOS alog..."), wxEmptyString, wxITEM_NORMAL);
	MenuItem5->Append(MenuItem53);
	MenuItem60 = new wxMenuItem(MenuItem5, ID_MENUITEM63, _("a RTL log..."), wxEmptyString, wxITEM_NORMAL);
	MenuItem5->Append(MenuItem60);
	MenuItem83 = new wxMenuItem(MenuItem5, ID_MENUITEM87, _("a Bremen DLR log..."), wxEmptyString, wxITEM_NORMAL);
	MenuItem5->Append(MenuItem83);
	Menu1->Append(ID_MENUITEM3, _("Import"), MenuItem5, wxEmptyString);
	MenuItem51 = new wxMenu();
	MenuItem55 = new wxMenuItem(MenuItem51, ID_MENUITEM58, _("Plain text files..."), _("Generate text files for odometry and laser"), wxITEM_NORMAL);
	MenuItem51->Append(MenuItem55);
	MenuItem52 = new wxMenuItem(MenuItem51, ID_MENUITEM55, _("As a MOOS alog file..."), wxEmptyString, wxITEM_NORMAL);
	MenuItem51->Append(MenuItem52);
	Menu1->Append(ID_MENUITEM54, _("Export"), MenuItem51, wxEmptyString);
	Menu1->AppendSeparator();
	MenuItem1 = new wxMenuItem(Menu1, idMenuQuit, _("Quit\tAlt-F4"), _("Quit the application"), wxITEM_NORMAL);
	MenuItem1->SetBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_QUIT")),wxART_MENU));
	Menu1->Append(MenuItem1);
	MenuBar1->Append(Menu1, _("&File"));
	Menu3 = new wxMenu();
	MenuItem16 = new wxMenuItem(Menu3, ID_MENUITEM14, _("Edit rawlog..."), _("Manipulate the rawlog entries"), wxITEM_NORMAL);
	MenuItem16->SetBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_COPY")),wxART_MENU));
	Menu3->Append(MenuItem16);
	MenuItem48 = new wxMenuItem(Menu3, ID_MENUITEM51, _("Insert a field for comments"), wxEmptyString, wxITEM_NORMAL);
	Menu3->Append(MenuItem48);
	MenuItem66 = new wxMenuItem(Menu3, ID_MENUITEM69, _("Rename a sensor..."), wxEmptyString, wxITEM_NORMAL);
	Menu3->Append(MenuItem66);
	Menu3->AppendSeparator();
	MenuItem17 = new wxMenuItem(Menu3, ID_MENUITEM15, _("Change sensor/camera parameters..."), _("Change the poses of the sensors"), wxITEM_NORMAL);
	Menu3->Append(MenuItem17);
	MenuItem67 = new wxMenuItem(Menu3, ID_MENUITEM70, _("Batch change sensor poses..."), _("Change the pose of all sensors from a ini-like config file"), wxITEM_NORMAL);
	Menu3->Append(MenuItem67);
	Menu3->AppendSeparator();
	MenuItem18 = new wxMenuItem(Menu3, ID_MENUITEM16, _("Decimate records..."), _("Reduce the number of observations in the rawlog"), wxITEM_NORMAL);
	Menu3->Append(MenuItem18);
	MenuItem56 = new wxMenuItem(Menu3, ID_MENUITEM59, _("Loss-less decimation ..."), _("Accumulate observations from many timesteps in one"), wxITEM_NORMAL);
	Menu3->Append(MenuItem56);
	MenuItem54 = new wxMenuItem(Menu3, ID_MENUITEM57, _("Compact rawlog..."), _("Group consecutive actions & observations"), wxITEM_NORMAL);
	Menu3->Append(MenuItem54);
	MenuItem72 = new wxMenuItem(Menu3, ID_MENUITEM75, _("Convert into SF format..."), _("Convert a rawlog of observations-only into the actions-sensory frames format"), wxITEM_NORMAL);
	Menu3->Append(MenuItem72);
	Menu3->AppendSeparator();
	MenuItem64 = new wxMenuItem(Menu3, ID_MENUITEM67, _("Re-sort by timestamp"), _("Re order all the observations in the rawlog using their timestamps"), wxITEM_NORMAL);
	Menu3->Append(MenuItem64);
	MenuItem65 = new wxMenuItem(Menu3, ID_MENUITEM68, _("Shift timestamp by sensorLabel..."), _("Displaces in time all the observations of a given label by some amount"), wxITEM_NORMAL);
	Menu3->Append(MenuItem65);
	MenuItem79 = new wxMenuItem(Menu3, ID_MENUITEM82, _("Regenerate timestamps in SF"), wxEmptyString, wxITEM_NORMAL);
	Menu3->Append(MenuItem79);
	MenuBar1->Append(Menu3, _("&Edit"));
	Menu6 = new wxMenu();
	Menu14 = new wxMenu();
	MenuItem22 = new wxMenuItem(Menu14, ID_MENUITEM20, _("Modify motion model..."), _("Manipulate the uncertainty of odometry"), wxITEM_NORMAL);
	Menu14->Append(MenuItem22);
	MenuItem24 = new wxMenuItem(Menu14, ID_MENUITEM22, _("Recalculate actions with ICP..."), _("Compute ICP-based odometry"), wxITEM_NORMAL);
	Menu14->Append(MenuItem24);
	MenuItem50 = new wxMenuItem(Menu14, ID_MENUITEM53, _("Modify uncertainty of ICP-based actions..."), wxEmptyString, wxITEM_NORMAL);
	Menu14->Append(MenuItem50);
	MenuItem25 = new wxMenuItem(Menu14, ID_MENUITEM23, _("Recompute odometry from encoders config..."), _("Recompute odometry increments from encoders"), wxITEM_NORMAL);
	Menu14->Append(MenuItem25);
	MenuItem38 = new wxMenuItem(Menu14, ID_MENUITEM41, _("Force encoders info to \'false\'"), _("Can be used to force ignoring corrupted encoders data in old datasets"), wxITEM_NORMAL);
	Menu14->Append(MenuItem38);
	MenuItem80 = new wxMenuItem(Menu14, ID_MENUITEM84, _("Regenerate invalid odometry timestamps..."), wxEmptyString, wxITEM_NORMAL);
	Menu14->Append(MenuItem80);
	Menu6->Append(ID_MENUITEM12, _("&Odometry (actions)"), Menu14, wxEmptyString);
	Menu20 = new wxMenu();
	MenuItem19 = new wxMenuItem(Menu20, ID_MENUITEM17, _("Show the ICP module..."), _("A demonstration of scan-matching with ICP"), wxITEM_NORMAL);
	MenuItem19->SetBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_ICP")),wxART_MENU));
	Menu20->Append(MenuItem19);
	MenuItem41 = new wxMenuItem(Menu20, ID_MENUITEM44, _("Show an animation with the scans..."), wxEmptyString, wxITEM_NORMAL);
	Menu20->Append(MenuItem41);
	MenuItem21 = new wxMenuItem(Menu20, ID_MENUITEM19, _("Count laser scans with zero valid ranges"), _("Count the numnber of scans without a single valid range"), wxITEM_NORMAL);
	Menu20->Append(MenuItem21);
	MenuItem27 = new wxMenuItem(Menu20, ID_MENUITEM25, _("Filter out erroneous range measurements..."), _("Heuristic to remove erroneous laser sensor readings"), wxITEM_NORMAL);
	Menu20->Append(MenuItem27);
	MenuItem70 = new wxMenuItem(Menu20, ID_MENUITEM73, _("Mark as invalid ranges larger than maximum..."), wxEmptyString, wxITEM_NORMAL);
	Menu20->Append(MenuItem70);
	MenuItem71 = new wxMenuItem(Menu20, ID_MENUITEM74, _("Change max. range for a given laser scan..."), wxEmptyString, wxITEM_NORMAL);
	Menu20->Append(MenuItem71);
	MenuItem74 = new wxMenuItem(Menu20, ID_MENUITEM77, _("Batch exclusion zone filtering..."), wxEmptyString, wxITEM_NORMAL);
	Menu20->Append(MenuItem74);
	MenuItem76 = new wxMenuItem(Menu20, ID_MENUITEM79, _("Batch exclusion angles filtering..."), wxEmptyString, wxITEM_NORMAL);
	Menu20->Append(MenuItem76);
	Menu6->Append(ID_MENUITEM18, _("&Laser scans"), Menu20, wxEmptyString);
	MenuItem81 = new wxMenu();
	MenuItem82 = new wxMenuItem(MenuItem81, ID_MENUITEM86, _("Recover camera params..."), _("Executes a Levenberg-Marquart optimization to recover the camera calibration matrix"), wxITEM_NORMAL);
	MenuItem81->Append(MenuItem82);
	mnuItemEnable3DCamAutoGenPoints = new wxMenuItem(MenuItem81, ID_MENUITEM90, _("Enable on-the-fly generate 3D point cloud"), wxEmptyString, wxITEM_CHECK);
	MenuItem81->Append(mnuItemEnable3DCamAutoGenPoints);
	Menu6->Append(ID_MENUITEM85, _("&3D depth cameras"), MenuItem81, wxEmptyString);
	Menu23 = new wxMenu();
	MenuItem31 = new wxMenuItem(Menu23, ID_MENUITEM29, _("Sequence of PNG files with images..."), _("Extract all the images of the rawlog to a given directory"), wxITEM_NORMAL);
	Menu23->Append(MenuItem31);
	MenuItem11 = new wxMenuItem(Menu23, ID_MENUITEM9, _("Show images as a video..."), _("Show the sequence of images as a video"), wxITEM_NORMAL);
	Menu23->Append(MenuItem11);
	MenuItem30 = new wxMenuItem(Menu23, ID_MENUITEM28, _("Generate 3D actions from visual odometry..."), wxEmptyString, wxITEM_NORMAL);
	Menu23->Append(MenuItem30);
	MenuItem68 = new wxMenuItem(Menu23, ID_MENUITEM71, _("Convert pairs of mono into stereo...\tCreate stereo images observations from pairs of monocular images"), wxEmptyString, wxITEM_NORMAL);
	Menu23->Append(MenuItem68);
	MenuItem69 = new wxMenuItem(Menu23, ID_MENUITEM72, _("Batch rectify images..."), wxEmptyString, wxITEM_NORMAL);
	Menu23->Append(MenuItem69);
	MenuItem75 = new wxMenuItem(Menu23, ID_MENUITEM78, _("Rename externally stored image files..."), wxEmptyString, wxITEM_NORMAL);
	Menu23->Append(MenuItem75);
	mnuCreateAVI = new wxMenuItem(Menu23, ID_MENUITEM83, _("Create AVI video file..."), wxEmptyString, wxITEM_NORMAL);
	Menu23->Append(mnuCreateAVI);
	Menu6->Append(ID_MENUITEM21, _("&Images (Mono && Stereo)"), Menu23, wxEmptyString);
	Menu38 = new wxMenu();
	MenuItem32 = new wxMenuItem(Menu38, ID_MENUITEM30, _("Generate text file with gas sensor readings..."), _("Extract gas readings to a text file"), wxITEM_NORMAL);
	Menu38->Append(MenuItem32);
	MenuItem26 = new wxMenuItem(Menu38, ID_MENUITEM24, _("Filter out spureous gas readings..."), _("Heuristic to remove erroneous gas sensor readings"), wxITEM_NORMAL);
	Menu38->Append(MenuItem26);
	Menu6->Append(ID_MENUITEM35, _("Ga&s sensors"), Menu38, wxEmptyString);
	Menu39 = new wxMenu();
	MenuItem33 = new wxMenuItem(Menu39, ID_MENUITEM31, _("Generate text file with GPS readings..."), _("Extract GPS readings to a text file"), wxITEM_NORMAL);
	Menu39->Append(MenuItem33);
	MenuItem36 = new wxMenuItem(Menu39, ID_MENUITEM34, _("Summary of DGPS modes"), _("Show a summary of GPS modes found in the rawlog"), wxITEM_NORMAL);
	Menu39->Append(MenuItem36);
	MenuItem62 = new wxMenuItem(Menu39, ID_MENUITEM65, _("&Measure distance between GPS\'s..."), wxEmptyString, wxITEM_NORMAL);
	Menu39->Append(MenuItem62);
	MenuItem63 = new wxMenuItem(Menu39, ID_MENUITEM66, _("Regenerate &timestamp from sat time"), wxEmptyString, wxITEM_NORMAL);
	Menu39->Append(MenuItem63);
	MenuItem49 = new wxMenuItem(Menu39, ID_MENUITEM52, _("&Draw 3D path..."), wxEmptyString, wxITEM_NORMAL);
	Menu39->Append(MenuItem49);
	MenuItem77 = new wxMenuItem(Menu39, ID_MENUITEM80, _("Delete entries with NaN"), wxEmptyString, wxITEM_NORMAL);
	Menu39->Append(MenuItem77);
	Menu6->Append(ID_MENUITEM36, _("&GPS sensors"), Menu39, wxEmptyString);
	Menu40 = new wxMenu();
	MenuItem35 = new wxMenuItem(Menu40, ID_MENUITEM33, _("Generate text file with Beacon ranges..."), _("Extract range readings to a text file"), wxITEM_NORMAL);
	Menu40->Append(MenuItem35);
	MenuItem14 = new wxMenuItem(Menu40, ID_MENUITEM38, _("Remove specific measurements"), wxEmptyString, wxITEM_NORMAL);
	Menu40->Append(MenuItem14);
	Menu6->Append(ID_MENUITEM37, _("Range-only (&beacon detector) sensors"), Menu40, wxEmptyString);
	MenuItem20 = new wxMenu();
	MenuItem23 = new wxMenuItem(MenuItem20, ID_MENUITEM40, _("Generate text file with measurements..."), wxEmptyString, wxITEM_NORMAL);
	MenuItem20->Append(MenuItem23);
	MenuItem78 = new wxMenuItem(MenuItem20, ID_MENUITEM81, _("Remove specific landmark by ID..."), wxEmptyString, wxITEM_NORMAL);
	MenuItem20->Append(MenuItem78);
	Menu6->Append(ID_MENUITEM39, _("Range-bearing (landmark detector) sensors"), MenuItem20, wxEmptyString);
	MenuItem42 = new wxMenu();
	MenuItem43 = new wxMenuItem(MenuItem42, ID_MENUITEM46, _("Generate a text file with measurements..."), wxEmptyString, wxITEM_NORMAL);
	MenuItem42->Append(MenuItem43);
	Menu6->Append(ID_MENUITEM45, _("1D range finders (Ultrasonic,IR)"), MenuItem42, wxEmptyString);
	MenuItem39 = new wxMenu();
	MenuItem40 = new wxMenuItem(MenuItem39, ID_MENUITEM43, _("Generate text file with measurementes..."), wxEmptyString, wxITEM_NORMAL);
	MenuItem39->Append(MenuItem40);
	Menu6->Append(ID_MENUITEM42, _("Inertial Measurement Units"), MenuItem39, wxEmptyString);
	MenuItem84 = new wxMenu();
	MenuItem85 = new wxMenuItem(MenuItem84, ID_MENUITEM89, _("Generate &text file with all observations..."), wxEmptyString, wxITEM_NORMAL);
	MenuItem84->Append(MenuItem85);
	Menu6->Append(ID_MENUITEM88, _("&WiFi Strength"), MenuItem84, wxEmptyString);
	MenuBar1->Append(Menu6, _("&Sensors"));
	Menu4 = new wxMenu();
	MenuItem28 = new wxMenuItem(Menu4, ID_MENUITEM26, _("Open paths and &map generation module..."), _("Open the raw map/path samples dialog"), wxITEM_NORMAL);
	MenuItem28->SetBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_RAWMAP")),wxART_MENU));
	Menu4->Append(MenuItem28);
	MenuItem34 = new wxMenuItem(Menu4, ID_MENUITEM32, _("&Generate odometry and laser text files..."), _("Extract odometry and laser scans to text files"), wxITEM_NORMAL);
	Menu4->Append(MenuItem34);
	MenuBar1->Append(Menu4, _("&Tools"));
	Menu2 = new wxMenu();
	MenuItem29 = new wxMenuItem(Menu2, ID_MENUITEM27, _("Show &tips..."), wxEmptyString, wxITEM_NORMAL);
	Menu2->Append(MenuItem29);
	MenuItem2 = new wxMenuItem(Menu2, idMenuAbout, _("&About\tF1"), _("Show info about this application"), wxITEM_NORMAL);
	MenuItem2->SetBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_ABOUT")),wxART_MENU));
	Menu2->Append(MenuItem2);
	MenuBar1->Append(Menu2, _("&Help"));
	SetMenuBar(MenuBar1);
	StatusBar1 = new wxStatusBar(this, ID_STATUSBAR1, 0, _T("ID_STATUSBAR1"));
	int __wxStatusBarWidths_1[1] = { -1 };
	int __wxStatusBarStyles_1[1] = { wxSB_NORMAL };
	StatusBar1->SetFieldsCount(1,__wxStatusBarWidths_1);
	StatusBar1->SetStatusStyles(1,__wxStatusBarStyles_1);
	SetStatusBar(StatusBar1);
	ToolBar1 = new wxToolBar(this, ID_TOOLBAR1, wxDefaultPosition, wxDefaultSize, wxTB_FLAT|wxTB_HORIZONTAL|wxTB_3DBUTTONS|wxTB_TEXT|wxNO_BORDER, _T("ID_TOOLBAR1"));
	ToolBar1->SetToolBitmapSize(wxSize(32,32));
	ToolBarItem1 = ToolBar1->AddTool(ID_TOOLBARITEM1, _("Load..."), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FOLDER")),wxART_TOOLBAR), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FOLDER")),wxART_TOOLBAR), wxITEM_NORMAL, _("Open a rawlog..."), _("Open a rawlog"));
	ToolBarItem2 = ToolBar1->AddTool(ID_TOOLBARITEM2, _("Save as..."), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FILE_SAVE_AS")),wxART_TOOLBAR), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FILE_SAVE_AS")),wxART_TOOLBAR), wxITEM_NORMAL, _("Save the rawlog in memory as..."), _("Save the current rawlog to a file"));
	ToolBar1->AddSeparator();
	ToolBarItem3 = ToolBar1->AddTool(ID_TOOLBARITEM3, _("Edit..."), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_COPY")),wxART_TOOLBAR), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_COPY")),wxART_TOOLBAR), wxITEM_NORMAL, _("Edit the rawlog: Cut, batch remove,..."), _("Edit the rawlog (cut, remove observations,...)"));
	ToolBarItem4 = ToolBar1->AddTool(ID_TOOLBARITEM4, _("\"Raw-map\""), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_RAWMAP")),wxART_TOOLBAR), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_RAWMAP")),wxART_TOOLBAR), wxITEM_NORMAL, _("Generate a map of laser points from odometry..."), _("Generate a map of laser points from odometry"));
	ToolBarItem5 = ToolBar1->AddTool(ID_TOOLBARITEM5, _("Motion model"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_MOTION")),wxART_TOOLBAR), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_MOTION")),wxART_TOOLBAR), wxITEM_NORMAL, _("Shows/modifies the probabilistic motion model"), _("Shows/modifies the probabilistic motion model"));
	ToolBarItem6 = ToolBar1->AddTool(ID_TOOLBARITEM9, _("ICP..."), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_ICP")),wxART_TOOLBAR), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_ICP")),wxART_TOOLBAR), wxITEM_NORMAL, _("Opens the scan matching experimenting module..."), _("Opens the scan matching experimenting module"));
	ToolBar1->AddSeparator();
	ToolBarItem7 = ToolBar1->AddTool(ID_TOOLBARITEM10, _("Animate..."), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_ANIMATE_SCANS")),wxART_TOOLBAR), wxNullBitmap, wxITEM_NORMAL, _("Shows an animation with the laser scans"), _("Shows an animation with the laser scans"));
	ToolBarItem8 = ToolBar1->AddTool(ID_TOOLBARITEM6, _("Video..."), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_TIP")),wxART_TOOLBAR), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_TIP")),wxART_TOOLBAR), wxITEM_NORMAL, _("View images as a video..."), _("View images as a video..."));
	ToolBar1->AddSeparator();
	ToolBarItem9 = ToolBar1->AddTool(ID_TOOLBARITEM7, _("About..."), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_ABOUT")),wxART_TOOLBAR), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_ABOUT")),wxART_TOOLBAR), wxITEM_NORMAL, _("Shows the application info"), _("Show information about this program"));
	ToolBarItem10 = ToolBar1->AddTool(ID_TOOLBARITEM8, _("Quit"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_QUIT")),wxART_TOOLBAR), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_QUIT")),wxART_TOOLBAR), wxITEM_NORMAL, _("Close the application"), _("Close the application"));
	ToolBar1->Realize();
	SetToolBar(ToolBar1);
	MenuItem37 = new wxMenuItem((&mnuTree), MNU_1, _("Delete element"), _("Erases the selected element"), wxITEM_NORMAL);
	mnuTree.Append(MenuItem37);
	MenuItem45 = new wxMenu();
	MenuItem46 = new wxMenuItem(MenuItem45, ID_MENUITEM49, _("2D increment (from odometry)"), wxEmptyString, wxITEM_NORMAL);
	MenuItem45->Append(MenuItem46);
	MenuItem47 = new wxMenuItem(MenuItem45, ID_MENUITEM50, _("2D increment (from scan-matching)"), wxEmptyString, wxITEM_NORMAL);
	MenuItem45->Append(MenuItem47);
	mnuTree.Append(ID_MENUITEM48, _("Add action"), MenuItem45, wxEmptyString);
	timAutoLoad.SetOwner(this, ID_TIMER1);
	timAutoLoad.Start(50, true);

	Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&xRawLogViewerFrame::OnbtnEditCommentsClick1);
	Connect(ID_SLIDER1,wxEVT_SCROLL_TOP|wxEVT_SCROLL_BOTTOM|wxEVT_SCROLL_LINEUP|wxEVT_SCROLL_LINEDOWN|wxEVT_SCROLL_PAGEUP|wxEVT_SCROLL_PAGEDOWN|wxEVT_SCROLL_THUMBTRACK|wxEVT_SCROLL_THUMBRELEASE|wxEVT_SCROLL_CHANGED,(wxObjectEventFunction)&xRawLogViewerFrame::Onslid3DcamConfCmdScrollChanged);
	Connect(ID_SLIDER1,wxEVT_SCROLL_THUMBTRACK,(wxObjectEventFunction)&xRawLogViewerFrame::Onslid3DcamConfCmdScrollChanged);
	Connect(ID_SLIDER1,wxEVT_SCROLL_CHANGED,(wxObjectEventFunction)&xRawLogViewerFrame::Onslid3DcamConfCmdScrollChanged);
	Connect(ID_NOTEBOOK1,wxEVT_COMMAND_NOTEBOOK_PAGE_CHANGING,(wxObjectEventFunction)&xRawLogViewerFrame::OnNotebook1PageChanging);
	Connect(ID_MENUITEM1,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnFileOpen);
	Connect(ID_MENUITEM2,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnSaveFile);
	Connect(ID_MENUITEM76,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuRevert);
	Connect(ID_MENUITEM7,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnLoadAPartOnly);
	Connect(ID_MENUITEM8,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnFileCountEntries);
	Connect(ID_MENUITEM10,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnFileSaveImages);
	Connect(ID_MENUITEM62,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuConvertExternallyStored);
	Connect(ID_MENUITEM64,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuConvertObservationOnly);
	Connect(ID_MENUITEM13,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnFileGenVisualLMFromStereoImages);
	Connect(ID_MENUITEM60,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuLossLessDecFILE);
	Connect(ID_MENUITEM61,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenCompactFILE);
	Connect(ID_MENUITEM5,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnImportCARMEN);
	Connect(ID_MENUITEM47,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnImportSequenceOfImages);
	Connect(ID_MENUITEM56,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuImportALOG);
	Connect(ID_MENUITEM63,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnImportRTL);
	Connect(ID_MENUITEM87,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuItemImportBremenDLRLog);
	Connect(ID_MENUITEM58,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnGenOdoLaser);
	Connect(ID_MENUITEM55,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuExportALOG);
	Connect(idMenuQuit,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnQuit);
	Connect(ID_MENUITEM14,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnEditRawlog);
	Connect(ID_MENUITEM51,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuInsertComment);
	Connect(ID_MENUITEM69,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuRenameSensor);
	Connect(ID_MENUITEM15,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnChangeSensorPositions);
	Connect(ID_MENUITEM70,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuChangePosesBatch);
	Connect(ID_MENUITEM16,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnDecimateRecords);
	Connect(ID_MENUITEM59,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuLossLessDecimate);
	Connect(ID_MENUITEM57,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuCompactRawlog);
	Connect(ID_MENUITEM75,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuConvertSF);
	Connect(ID_MENUITEM67,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuResortByTimestamp);
	Connect(ID_MENUITEM68,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuShiftTimestampsByLabel);
	Connect(ID_MENUITEM82,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuRegenerateTimestampBySF);
	Connect(ID_MENUITEM20,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnChangeMotionModel);
	Connect(ID_MENUITEM22,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnRecalculateActionsICP);
	Connect(ID_MENUITEM53,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuModifyICPActionsUncertainty);
	Connect(ID_MENUITEM23,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnRecomputeOdometry);
	Connect(ID_MENUITEM41,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnForceEncodersFalse);
	Connect(ID_MENUITEM84,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuRegenerateOdometryTimes);
	Connect(ID_MENUITEM17,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnShowICP);
	Connect(ID_MENUITEM44,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnShowAnimateScans);
	Connect(ID_MENUITEM19,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnCountBadScans);
	Connect(ID_MENUITEM25,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnFilterErroneousScans);
	Connect(ID_MENUITEM73,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuMarkLaserScanInvalid);
	Connect(ID_MENUITEM74,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuChangeMaxRangeLaser);
	Connect(ID_MENUITEM77,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuBatchLaserExclusionZones);
	Connect(ID_MENUITEM79,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnLaserFilterAngles);
	Connect(ID_MENUITEM86,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuItem3DObsRecoverParams);
	Connect(ID_MENUITEM29,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnGenerateSeqImgs);
	Connect(ID_MENUITEM9,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnShowImagesAsVideo);
	Connect(ID_MENUITEM28,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuVisualOdometry);
	Connect(ID_MENUITEM71,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuMono2Stereo);
	Connect(ID_MENUITEM72,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuRectifyImages);
	Connect(ID_MENUITEM78,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuRenameImageFiles);
	Connect(ID_MENUITEM83,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnmnuCreateAVISelected);
	Connect(ID_MENUITEM30,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnGenGasTxt);
	Connect(ID_MENUITEM24,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnFilterSpureousGas);
	Connect(ID_MENUITEM31,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnGenGPSTxt);
	Connect(ID_MENUITEM34,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnSummaryGPS);
	Connect(ID_MENUITEM65,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuDistanceBtwGPSs);
	Connect(ID_MENUITEM66,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuRegenerateGPSTimestamps);
	Connect(ID_MENUITEM52,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuDrawGPSPath);
	Connect(ID_MENUITEM80,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuGPSDeleteNaN);
	Connect(ID_MENUITEM33,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuGenerateBeaconList);
	Connect(ID_MENUITEM38,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnRemoveSpecificRangeMeas);
	Connect(ID_MENUITEM40,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnGenerateTextFileRangeBearing);
	Connect(ID_MENUITEM81,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuRangeBearFilterIDs);
	Connect(ID_MENUITEM46,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnRangeFinder1DGenTextFile);
	Connect(ID_MENUITEM43,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnGenerateIMUTextFile);
	Connect(ID_MENUITEM89,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnGenWifiTxt);
	Connect(ID_MENUITEM26,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnRawMapOdo);
	Connect(ID_MENUITEM32,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnGenOdoLaser);
	Connect(ID_MENUITEM27,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuShowTips);
	Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnAbout);
	Connect(ID_TOOLBARITEM1,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&xRawLogViewerFrame::OnFileOpen);
	Connect(ID_TOOLBARITEM2,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&xRawLogViewerFrame::OnSaveFile);
	Connect(ID_TOOLBARITEM3,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&xRawLogViewerFrame::OnEditRawlog);
	Connect(ID_TOOLBARITEM4,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&xRawLogViewerFrame::OnRawMapOdo);
	Connect(ID_TOOLBARITEM5,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&xRawLogViewerFrame::OnChangeMotionModel);
	Connect(ID_TOOLBARITEM9,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&xRawLogViewerFrame::OnShowICP);
	Connect(ID_TOOLBARITEM10,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&xRawLogViewerFrame::OnShowAnimateScans);
	Connect(ID_TOOLBARITEM6,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&xRawLogViewerFrame::OnShowImagesAsVideo);
	Connect(ID_TOOLBARITEM7,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&xRawLogViewerFrame::OnAbout);
	Connect(ID_TOOLBARITEM8,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&xRawLogViewerFrame::OnQuit);
	Connect(MNU_1,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuItem37Selected);
	Connect(ID_MENUITEM49,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuItem46Selected);
	Connect(ID_MENUITEM50,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnMenuItem47Selected);
	Connect(ID_TIMER1,wxEVT_TIMER,(wxObjectEventFunction)&xRawLogViewerFrame::OntimAutoLoadTrigger);
	//*)

	// "Manually" added code:
	// ----------------------------
	imgList = new wxImageList(16,16,true,0);
	imgList->Add( wxIcon(tree_icon1_xpm) );
	imgList->Add( wxIcon(tree_icon2_xpm) );
	imgList->Add( wxIcon(tree_icon3_xpm) );
	imgList->Add( wxIcon(tree_icon4_xpm) );
	imgList->Add( wxIcon(tree_icon5_xpm) );
	imgList->Add( wxIcon(tree_icon6_xpm) );
	imgList->Add( wxIcon(tree_icon7_xpm) );
	imgList->Add( wxIcon(tree_icon8_xpm) );
	imgList->Add( wxIcon(tree_icon9_xpm) );

	tree_view->AssignImageList( imgList );

	tree_view->ConnectSelectedItemChange( OntreeViewSelectionChanged );
	tree_view->setWinParent(this);


	// Force this menu item starts checked.
	mnuItemEnable3DCamAutoGenPoints->Check(true);


	// The graphs:
	// -----------------------------------
	lyScan2D            = new mpFXYVector();
	lyAction2D_XY       = new mpFXYVector();
	lyAction2D_PHI      = new mpFXYVector();

	plotScan2D->AddLayer( new mpScaleX() );
	plotScan2D->AddLayer( new mpScaleY() );
	plotScan2D->AddLayer( lyScan2D );

	plotScan2D->LockAspect(true);
	wxPen   penBlue(wxColour(0,0,255),3);
	lyScan2D->SetPen( penBlue );

	plotAct2D_XY->AddLayer( new mpScaleX() );
	plotAct2D_XY->AddLayer( new mpScaleY() );
	plotAct2D_XY->AddLayer(  lyAction2D_XY );
	plotAct2D_XY->LockAspect(true);
	lyAction2D_XY->SetPen( penBlue );

	plotAct2D_PHI->AddLayer( new mpScaleX(_("PHI (deg)")) );
	plotAct2D_PHI->AddLayer(  lyAction2D_PHI );
	plotAct2D_PHI->LockAspect(true);
	lyAction2D_PHI->SetPen( penBlue );


	//  Range-bearing plot:
	lyRangeBearingLandmarks  = new mpFXYVector();
	lyRangeBearingLandmarks->SetPen( penBlue );
	lyRangeBearingLandmarks->SetContinuity(false);


	plotRangeBearing->AddLayer( new mpScaleX() );
	plotRangeBearing->AddLayer( new mpScaleY() );
	plotRangeBearing->AddLayer( lyRangeBearingLandmarks );


	plotScan2D->EnableDoubleBuffer(true);
	plotAct2D_XY->EnableDoubleBuffer(true);
	plotAct2D_PHI->EnableDoubleBuffer(true);
	plotRangeBearing->EnableDoubleBuffer(true);


	Maximize(); // Maximize the main window

	// Set sliders:
	// ----------------------
	SplitterWindow1->SetSashPosition(400);
	SplitterWindow3->SetSashPosition(400);


	// Set recent file list:
	// -------------------------
	MenuItem6->Destroy(MenuItem13);
	m_fileHistory.UseMenu( MenuItem6  );

	for (int i=m_fileHistory.GetMaxFiles()-1;i>=0;i--)
	{
		string fil = iniFile->read_string("RecentFiles",format("file_%03u",i),"");
		if (fil.size() && mrpt::system::fileExists(fil))
			m_fileHistory.AddFileToHistory( _U( fil.c_str() ) );
	}


	// Image directory selector on the toolbar:
	// --------------------------------------------
	{
		ToolBar1->AddSeparator();

		toolbarcomboImages =new wxComboBox(ToolBar1, ID_COMBO_IMG_DIRS, _(""), wxDefaultPosition, wxDefaultSize,0, NULL, wxCB_DROPDOWN );

		Connect(ID_COMBO_IMG_DIRS, wxEVT_COMMAND_COMBOBOX_SELECTED,(wxObjectEventFunction)&xRawLogViewerFrame::OnComboImageDirsChange);

		toolbarcomboImages->SetToolTip(_("Found external images paths"));
		toolbarcomboImages->SetMinSize(wxSize(300,-1));

		const long ID_TOOLBAR_TXT = wxNewId();
		ToolBar1->AddControl( new wxStaticText(ToolBar1,ID_TOOLBAR_TXT,_("Image dirs:")));
		ToolBar1->AddControl( toolbarcomboImages );

		ToolBar1->Realize();
	}

	// Construction of "global" dialog variables:
	// ----------------------------------------------
	formRawMap = new CFormRawMap(this);
	scanMatchingDialog = new CScanMatching(this);
}

xRawLogViewerFrame::~xRawLogViewerFrame()
{
	//(*Destroy(xRawLogViewerFrame)
	//*)

	// Destroy dialogs:
	delete formRawMap;
	formRawMap = NULL;
	delete scanMatchingDialog;
	scanMatchingDialog = NULL;


	// Close extra windows, if any:
	winGPSPath.clear();

}

void xRawLogViewerFrame::OnQuit(wxCommandEvent& event)
{
	Close();
}

void xRawLogViewerFrame::OnAbout(wxCommandEvent& event)
{
	CAboutBox   aboutbox(this);
	aboutbox.ShowModal();
}


//------------------------------------------------------------------------
//    Asks the user for a rawlog file, return false if user cancels
//------------------------------------------------------------------------
bool xRawLogViewerFrame::AskForOpenRawlog( std::string &fil )
{
	wxString caption = wxT("Choose a file to open");
	wxString wildcard = wxT("RawLog files (*.rawlog,*.rawlog.gz)|*.rawlog;*.rawlog.gz|All files (*.*)|*.*");

	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );

	wxString defaultFilename = wxT("");
	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_OPEN | wxFD_FILE_MUST_EXIST );

	if (dialog.ShowModal() == wxID_OK)
	{
		wxString fileName = dialog.GetPath();
		wxString filePath = dialog.GetDirectory();

		fil = string( fileName.mb_str() );
		return true;
	}
	return false;
}

//------------------------------------------------------------------------
//    Asks the user for a rawlog file, return false if user cancels
//------------------------------------------------------------------------
bool xRawLogViewerFrame::AskForSaveRawlog( std::string &fil )
{
	wxString caption = wxT("Save file...");
	wxString wildcard = wxT("RawLog files (*.rawlog,*.rawlog.gz)|*.rawlog;*.rawlog.gz|All files (*.*)|*.*");

	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );

	wxString defaultFilename = wxT("");
	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

	if (dialog.ShowModal() == wxID_OK)
	{
		wxString fileName = dialog.GetPath();
		wxString filePath = dialog.GetDirectory();

		fil = string( fileName.mb_str() );
		return true;
	}
	return false;
}


//------------------------------------------------------------------------
//                      Open a file dialog
//------------------------------------------------------------------------
void xRawLogViewerFrame::OnFileOpen(wxCommandEvent& event)
{
	string 	fil;
	if ( AskForOpenRawlog( fil ) )
		loadRawlogFile( fil );
}

//------------------------------------------------------------------------
//                      Save as...
//------------------------------------------------------------------------
void xRawLogViewerFrame::OnSaveFile(wxCommandEvent& event)
{
	wxString caption = wxT("Save as...");
	wxString wildcard = wxT("RawLog files (*.rawlog,*.rawlog.gz)|*.rawlog;*.rawlog.gz|All files (*.*)|*.*");

	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );

	wxString defaultFilename = _U( loadedFileName.c_str() );
	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT);

	if (dialog.ShowModal() == wxID_OK)
	{
		wxString fileName = dialog.GetPath();
		wxString filePath = dialog.GetDirectory();

		// Save the path
		WX_START_TRY

		iniFile->write(iniFileSect,"LastDir",string(filePath.mb_str()));

		// Save the file:
		loadedFileName = fileName.mb_str();

		CFileGZOutputStream	fs( loadedFileName.c_str() );

		int          countLoop=0, i, n = (int)rawlog.size();

		wxBusyCursor    waitCursor;

		wxProgressDialog    progDia(
			wxT("Saving rawlog to file"),
			wxT("Saving..."),
			n, // range
			this, // parent
			wxPD_CAN_ABORT |
			wxPD_APP_MODAL |
			wxPD_SMOOTH |
			wxPD_AUTO_HIDE |
			wxPD_ELAPSED_TIME |
			wxPD_ESTIMATED_TIME |
			wxPD_REMAINING_TIME);

		wxTheApp->Yield();  // Let the app. process messages

		// Comments:
		string comts;
		rawlog.getCommentText(comts);

		if (!comts.empty())
		{
			CObservationComment	o;
			o.text = comts;
			fs << o;
		}

		for (i=0;i<n;i++)
		{
			if (countLoop++ % 100 == 0)
			{
				if (!progDia.Update( i )) break; // Exit the loop
				wxTheApp->Yield();  // Let the app. process messages
			}

			fs << *rawlog.getAsGeneric(i);
		} // end for i

		progDia.Update( n );

		WX_END_TRY
	}
}

//------------------------------------------------------------------------
//               Edit rawlog dialog
//------------------------------------------------------------------------
void xRawLogViewerFrame::OnEditRawlog(wxCommandEvent& event)
{
	CFormEdit   dialog(this);


	dialog.cbObsLabel->Clear();
	for (std::map<std::string,TInfoPerSensorLabel>::iterator i=listOfSensorLabels.begin();i!=listOfSensorLabels.end();++i)
		dialog.cbObsLabel->Append( _U( i->first.c_str() ) );

	dialog.slFrom->SetRange(0,(int)rawlog.size()-1);
	dialog.slFrom->SetValue(0);

	dialog.spinFirst->SetRange(0,(int)rawlog.size()-1);
	dialog.spinFirst->SetValue( dialog.slFrom->GetValue() );

	dialog.slTo->SetRange(0,(int)rawlog.size()-1);
	dialog.slTo->SetValue((int)rawlog.size()-1);

	dialog.spinLast->SetRange(0,(int)rawlog.size()-1);
	dialog.spinLast->SetValue( dialog.slTo->GetValue() );

	// Fill all the observation classes:
	dialog.cbObsClass->Clear();
	vector<const utils::TRuntimeClassId*> lstClasses;
	vector<const utils::TRuntimeClassId*>::iterator it;
	lstClasses = utils::getAllRegisteredClasses();
	for (it=lstClasses.begin();it!=lstClasses.end();it++)
		if ((*it)->derivedFrom(CLASS_ID(CObservation) ) )
			dialog.cbObsClass->Append( _U( (*it)->className ) );

	dialog.Fit();
	dialog.ShowModal();

	// Update the views:
	rebuildTreeView();
}

//------------------------------------------------------------------------
//               Loads the rawlog into the application
//------------------------------------------------------------------------
void xRawLogViewerFrame::loadRawlogFile(
	const string &str,
	int		first,
	int		last
)
{
	WX_START_TRY

	if (!fileExists(str))
	{
		wxMessageBox(_U( string(string("File doesn't exist:\n")+str).c_str() ),_("Error loading file"),wxOK,this);
		return;
	}

	// Add to MR files, and save the list:
	WX_START_TRY

	m_fileHistory.AddFileToHistory( _U(str.c_str()) );

	for (size_t i=0;i<m_fileHistory.GetCount();i++)
		iniFile->write("RecentFiles",format("file_%03u",static_cast<unsigned>(i)), string(m_fileHistory.GetHistoryFile(i).mb_str()) );
	WX_END_TRY


	// Save the path
	string rawlog_path = extractFileDirectory(str);
	if (rawlog_path.empty())
		rawlog_path = "./";
	else if(*rawlog_path.rbegin()!='/' && *rawlog_path.rbegin()!='\\')
		rawlog_path += string("/");

	WX_START_TRY
	iniFile->write(iniFileSect,"LastDir", rawlog_path );
	WX_END_TRY

	// Set default delayed-load images base path:
	toolbarcomboImages->Clear();
	CImage::IMAGES_PATH_BASE = CRawlog::detectImagesDirectory(str);

	// Add found dir to the combo:
	toolbarcomboImages->Append( _U(CImage::IMAGES_PATH_BASE.c_str()) );
	toolbarcomboImages->SetSelection(0);

	// An extra rectified images dir??
	// Other "Images*" directories??
	if (mrpt::system::directoryExists(rawlog_path))
	{
		CDirectoryExplorer::TFileInfoList	lstFiles;
		CDirectoryExplorer::explore(rawlog_path , FILE_ATTRIB_DIRECTORY, lstFiles);
		for (CDirectoryExplorer::TFileInfoList::iterator i=lstFiles.begin();i!=lstFiles.end();++i)
			if ( 0== os::_strcmpi( i->name.substr(0,6).c_str(), "Images"))
			{
				wxString s = _U( i->wholePath.c_str() );
				if (toolbarcomboImages->FindString(s)==wxNOT_FOUND)
					toolbarcomboImages->Append( s );
			}
	}
#ifdef __WXMSW__
	SendMessage( (HWND)toolbarcomboImages->GetHandle(), CB_SETDROPPEDWIDTH, toolbarcomboImages->GetBestSize().GetWidth()  , 0);
#endif
	ToolBar1->Realize();



	wxBusyCursor        waitCursor;

	CFileGZInputStream	fil(str);

	uint64_t filSize = fil.getTotalBytesCount();

	const uint64_t progDialogMax = filSize>>10; // Size, in Kb's (to avoid saturatin the "int" in wxProgressDialog)

	loadedFileName = str;

	wxString            auxStr;
	wxProgressDialog    progDia(
		wxT("Progress of rawlog load"),
		wxT("Loading..."),
		progDialogMax, // range, in Kb's
		this, // parent
		wxPD_CAN_ABORT |
		wxPD_APP_MODAL |
		wxPD_SMOOTH |
		wxPD_AUTO_HIDE |
		wxPD_ELAPSED_TIME |
		wxPD_ESTIMATED_TIME |
		wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages
	progDia.SetSize(500,progDia.GetSize().GetHeight());
	progDia.Center();

	wxTheApp->Yield();  // Let the app. process messages

	// Clear first:
	rawlog.clear();

	crono_Loading.Tic();

	size_t              countLoop = 0;
	int					entryIndex = 0;
	bool                keepLoading=true;
	bool 				alreadyWarnedTooLargeFile=false;
	string              errorMsg;

	while (keepLoading)
	{
		if (countLoop++ % 10 == 0)
		{
			uint64_t fil_pos = fil.getPosition();
			static double last_ratio = -1;
			double ratio = fil_pos/(1.0*filSize);

			if (ratio-last_ratio>=0.006)
			{
				last_ratio = ratio;

				unsigned long	memUsg = getMemoryUsage();
				double			memUsg_Mb = memUsg / (1024.0*1024.0);

				const uint64_t  progPos = fil_pos>>10;

				auxStr.sprintf(wxT("Loading... %u objects / Memory usage: %.03fMb"),rawlog.size(), memUsg_Mb);
				if (!progDia.Update( progPos<(progDialogMax-1) ? progPos : (progDialogMax-1), auxStr ))
					keepLoading = false;
				progDia.Fit();
				wxTheApp->Yield();  // Let the app. process messages

				if (memUsg_Mb>1600 && !alreadyWarnedTooLargeFile)
				{
					alreadyWarnedTooLargeFile=true;
					string msg;
					msg += "It seems that the file you are loading will consume a large amount of memory if loaded in memory completely.\n";
					msg += "Note that large rawlogs are better managed by selecting 'Process rawlog in files' in the corresponding \n";
					msg += "RawLogViewer dialogs, or in File -> Operations on files -> ...\n \n";
					msg += "Do you want to continue loading this file? (Not recommended)";
					if (wxNO==wxMessageBox( _U(msg.c_str()),_("Warning"),wxYES_NO | wxICON_EXCLAMATION))
						keepLoading = false;
				}

			}

		}

		CSerializablePtr newObj;
		try
		{
			fil >> newObj;
			// Check type:
			if ( newObj->GetRuntimeClass() == CLASS_ID(CSensoryFrame))
			{
				if ( entryIndex>=first && (last==-1 || entryIndex<=last)  )
					rawlog.addObservationsMemoryReference( CSensoryFramePtr(newObj) );
				entryIndex++;
			}
			else if ( newObj->GetRuntimeClass() == CLASS_ID(CActionCollection))
			{
				if ( entryIndex>=first && (last==-1 || entryIndex<=last)  )
					rawlog.addActionsMemoryReference( CActionCollectionPtr(newObj) );
				entryIndex++;
			}
			/* Added in MRPT 0.6.0: The new "observations only" format: */
			else if ( newObj->GetRuntimeClass()->derivedFrom( CLASS_ID(CObservation)) )
			{
				if ( entryIndex>=first && (last==-1 || entryIndex<=last)  )
					rawlog.addObservationMemoryReference( CObservationPtr(newObj) );
				entryIndex++;
			}
			/* FOR BACKWARD COMPATIBILITY: CPose2D was used previously instead of an "ActionCollection" object
			                                                                    26-JAN-2006	*/
			else if ( newObj->GetRuntimeClass() == CLASS_ID(CPose2D))
			{
				if ( entryIndex>=first && (last==-1 || entryIndex<=last)  )
				{
					CPose2DPtr poseChange = CPose2DPtr(newObj);
					CActionCollectionPtr temp = CActionCollection::Create();
					CActionRobotMovement2D		action;
					CActionRobotMovement2D::TMotionModelOptions	options;
					action.computeFromOdometry( *poseChange, options);
					temp->insert( action );

					rawlog.addActionsMemoryReference( temp );
				}
				entryIndex++;
			}
			else if ( newObj->GetRuntimeClass() == CLASS_ID(CRawlog))
			{
				CRawlogPtr rw = CRawlogPtr(newObj);
				rawlog.moveFrom(*rw);
			}
			else
			{
				// Unknown class:
				THROW_EXCEPTION("Unknown class found in the file!");
			}

			// Passed last?
			if (last!=-1 && entryIndex>last)
				keepLoading = false;
		}
		catch (std::bad_alloc &)
		{
			// Probably we're in a 32 bit machine and we rose up to 2Gb of mem... free
			//  some, give a warning and go on.
			if ( rawlog.size()>10000 )
			{
				size_t NN = rawlog.size() - 10000;
				while (rawlog.size() > NN)
					rawlog.remove( NN );
			}
			else rawlog.clear();

			wxString	s = _("OUT OF MEMORY: The last part of the rawlog has been freed to allow the program to continue.\n");
#if MRPT_WORD_SIZE==32
			s << _("  This is a 32bit machine, so the maximum memory available is 2Gb in despite of the real RAM installed.");
#endif
			wxMessageBox(s);

			keepLoading = false;
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

	timeToLoad = crono_Loading.Tac();
	progDia.Update( progDialogMax );

	// Update the views:
	rebuildTreeView();

	// Set error msg:
	StatusBar1->SetStatusText( _U( format("Loading file: %s",str.c_str()).c_str() ));
	txtException->SetValue( _U( errorMsg.c_str() ) );

	// Seems something bad happened?
	if (!rawlog.size())
	{
		wxMessageBox( _("If the file is not really empty, perhaps the format is invalid or a more recent\nversion of RawLogViewer is required to load the file.\n Please check the tab \"End of load message\" for further details."), _("Zero entries loaded"), wxOK, this );
	}

	WX_END_TRY
}

//------------------------------------------------------------------------
//           Rebuilds the tree view with data in "rawlog"
//------------------------------------------------------------------------
void xRawLogViewerFrame::rebuildTreeView()
{
	wxString		s;
	float			totalDistance=0;
	bool			firstSF = true;
	TTimeStamp		t1=INVALID_TIMESTAMP,t2=INVALID_TIMESTAMP;
	int				countLoop=0;

	Notebook1->ChangeSelection( 0 );
	curSelectedObservation.clear_unique();// = NULL;
	curSelectedObject.clear_unique(); // = NULL;

	wxProgressDialog    progDia(
		wxT("Constructing the tree view"),
		wxT("Creating the tree..."),
		(int)rawlog.size(), // range
		this, // parent
		wxPD_APP_MODAL |
		wxPD_AUTO_HIDE );

	wxTheApp->Yield();  // Let the app. process messages

	WX_START_TRY

	wxBusyCursor        waitCursor;

	listOfSensorLabels.clear();

	// Refresh the custom tree view:
	tree_view->setRawlogName( loadedFileName );
	tree_view->setRawlogSource( &rawlog );


	// Does this need to be so complicated? -> Yes: In Linux (UNICODE) the straightforward
	//  implementation runs *very* slow.
	map<const TRuntimeClassId*,wxString>             mapStrings;
	map<const TRuntimeClassId*,wxString>::iterator   it;

	size_t updateProgressBarSteps = (rawlog.size() / 20)+1;

	std::map<const TRuntimeClassId*,size_t>		listOfObjects;

	// The elements:
	for (unsigned int i=0;i<rawlog.size();i++)
	{
		if (countLoop++ % updateProgressBarSteps == 0)
		{
			progDia.Update( i );
			wxTheApp->Yield();  // Let the app. process messages
		}

		// Process element:
		s = wxT("");
		switch ( rawlog.getType(i) )
		{
			case CRawlog::etActionCollection:
			{
				CActionCollectionPtr acts = rawlog.getAsAction(i);

				// Distance:
				CPose2D  est;
				if (acts->getBestMovementEstimation())
				{
					acts->getBestMovementEstimation()->poseChange->getMean(est);
					totalDistance += est.norm();
				}

			}
			break;

			case CRawlog::etSensoryFrame:
			{
				CSerializablePtr obj = rawlog.getAsObservations(i);
				if (CLASS_ID(CSensoryFrame)!=obj->GetRuntimeClass())
					THROW_EXCEPTION("Expected an object of class CSensoryFrame!!");

				CSensoryFramePtr sf = CSensoryFramePtr( obj );

				if (firstSF)
				{
					if (sf->size())
					{
						firstSF = false;
						t1 = (*sf->begin())->timestamp;
					}
				}
				if (sf->size())
					t2 = (*sf->begin())->timestamp;     // Keep the last one

				size_t  j,n = sf->size();

				for (j=0;j<n;j++)
				{
					CObservationPtr obs = sf->getObservationByIndex(j);

					// Stats:
					listOfObjects[obs->GetRuntimeClass()]++;
					TInfoPerSensorLabel	&dd = listOfSensorLabels[obs->sensorLabel];
					dd.occurences++;
					if (dd.first==INVALID_TIMESTAMP) dd.first = obs->timestamp;
					dd.last = obs->timestamp;
				}

			} // end SensoryFrame
			break;

			case CRawlog::etObservation:
			{
				CObservationPtr obs = rawlog.getAsObservation(i);

				if (t1==INVALID_TIMESTAMP)
					t1 = obs->timestamp;

				t2 = obs->timestamp;     // Keep the last one

				// Stats:
				listOfObjects[obs->GetRuntimeClass()]++;

				TInfoPerSensorLabel	&dd = listOfSensorLabels[obs->sensorLabel];
				dd.occurences++;
				if (dd.first==INVALID_TIMESTAMP) dd.first = obs->timestamp;
				dd.last = obs->timestamp;

				// For odometry measurements: total distance:
				if ( obs->GetRuntimeClass() == CLASS_ID(CObservationOdometry) )
				{
					CObservationOdometryPtr odoObs = CObservationOdometryPtr(obs);

					static CPose2D   oldOdo;
					static bool      oldOdo_first = true;

					if (oldOdo_first)
					{
						oldOdo_first = false;
						oldOdo = odoObs->odometry;
					}
					else
					{
						CPose2D   inc = odoObs->odometry - oldOdo;
						oldOdo = odoObs->odometry;

						totalDistance += inc.norm();
					}
				}

			} // end Observation
			break;

		}; // end switch type

	} // end for i

	if (t1!=INVALID_TIMESTAMP && t2!=INVALID_TIMESTAMP)
		experimentLenght = timeDifference(t1,t2);

	// Statistics:
	// ---------------------------
	memStats->Clear();

	memStats->AppendText( _U(format("Time to load file:                  %.03fms\n",1000*timeToLoad).c_str()) );
	memStats->AppendText( _U(format("Records loaded:                     %u\n",(unsigned)rawlog.size()).c_str() ) );
	memStats->AppendText( _U(format("Traveled distance (from odometry):  %.02f meters\n",totalDistance).c_str() ) );

	if (tree_view->getFirstTimestamp()!=INVALID_TIMESTAMP)
	{
		rawlog_first_timestamp = tree_view->getFirstTimestamp();
		memStats->AppendText( _U(format("Dataset first time-stamp (UTC):     %s\n", mrpt::system::dateTimeToString(tree_view->getFirstTimestamp()).c_str()).c_str() ) );
	}

	memStats->AppendText( _U(format("Dataset length:                     %s (hh:mm:ss),  %.03f secs.\n", formatTimeInterval(experimentLenght).c_str(), experimentLenght ).c_str()) );

	// Stats of object classes:
	memStats->AppendText( _("\nSummary of classes found in the rawlog:\n-----------------------------------------\n") );

	if (experimentLenght==0) experimentLenght=1;

	for (std::map<const TRuntimeClassId*,size_t>::iterator it=listOfObjects.begin();it!=listOfObjects.end();++it)
	{
		const char *className = it->first->className;
		size_t      count = it->second;
		memStats->AppendText( _U(format(" %8u %25s : %5.03f Hz\n",(unsigned)count,className, double(count>1 ? count-1 : 1)/experimentLenght ).c_str() ) );
	}

	// Stats of object classes:
	memStats->AppendText( _("\nSummary of 'sensorLabels' found in the rawlog:\n-----------------------------------------\n") );

	for (std::map<std::string,TInfoPerSensorLabel>::iterator it=listOfSensorLabels.begin();it!=listOfSensorLabels.end();++it)
	{
		size_t      count = it->second.occurences;
		TTimeStamp	tf = it->second.first;
		TTimeStamp	tl = it->second.last;
		double Hz = 0, dur = 0;
		if (tf!=INVALID_TIMESTAMP && tl!=INVALID_TIMESTAMP)
		{
			dur = mrpt::system::timeDifference(tf,tl);
			Hz = double(count>1 ? count-1 : 1)/dur;
		}

		memStats->AppendText( _U(format(" %8u %25s : %5.03f Hz during %.04f seconds\n",(unsigned)count,it->first.c_str(), Hz, dur ).c_str() ) );
	}

	memStats->ShowPosition(0);

	SelectObjectInTreeView( CSerializablePtr() );
	tree_view->Refresh();

	WX_END_TRY
}

// Selection has changed:
void xRawLogViewerFrame::OntreeViewSelectionChanged(
	wxWindow*				me,
	CRawlogTreeView*		the_tree,
	TRawlogTreeViewEvent	ev,
	int					item_index,
	const mrpt::utils::CSerializablePtr &item_data)
{
	xRawLogViewerFrame	*win = (xRawLogViewerFrame*)me;
	win->SelectObjectInTreeView(item_data);
	the_tree->SetFocus();
}

// Update selected item display:
void xRawLogViewerFrame::SelectObjectInTreeView( const CSerializablePtr & sel_obj )
{
	WX_START_TRY

	if (sel_obj.present())
	{
		// -----------------------------------------------------------
		// 	Write to the memo:
		// 	Redirect all the console output (cout) to the "memo":
		// ------------------------------------------------------------
		memo->Clear();

		{ // scope for redirector
		CMyRedirector myRedirector( memo );

		// For contextualize the popup menu, etc...
		curSelectedObject = sel_obj;

		// Select the panel by class (cannot make "wxWindow::FindWindowByName" to run right!!! :-(
		//  And update the required data:
		const TRuntimeClassId *classID = sel_obj->GetRuntimeClass();

		// Common data:
		if ( classID->derivedFrom( CLASS_ID( CObservation ) ) )
		{
			CObservationPtr obs( sel_obj );
			cout << "Timestamp (UTC): " << dateTimeToString(obs->timestamp) << endl;
			cout << "  (as time_t): " <<  std::fixed << std::setprecision(5) << mrpt::system::timestampTotime_t(obs->timestamp) << endl;
			cout << "  (as TTimestamp): " << obs->timestamp <<  endl;
			cout << "Sensor label: '" << obs->sensorLabel << "'" << endl;
		}
		else
		if ( classID->derivedFrom( CLASS_ID( CAction ) ) )
		{
			CActionPtr obs( sel_obj );
			cout << "Timestamp (UTC): " << dateTimeToString(obs->timestamp) << endl;
		}

		// Specific data:
		if ( classID  == CLASS_ID(CObservation2DRangeScan) )
		{
			// ----------------------------------------------------------------------
			//              CObservation2DRangeScan
			// ----------------------------------------------------------------------
			Notebook1->ChangeSelection( 2 );
			CObservation2DRangeScanPtr obs = CObservation2DRangeScanPtr( sel_obj );
			curSelectedObservation = CObservationPtr( sel_obj );

			cout << "Homogeneous matrix for the sensor's 3D pose, relative to robot base:\n";
			cout << obs->sensorPose.getHomogeneousMatrixVal()
				 << obs->sensorPose << endl;

			cout << format("Samples direction: %s\n", (obs->rightToLeft) ? "Right->Left" : "Left->Right");
			cout << format("Points in the scan: %u\n", (unsigned)obs->scan.size());
			cout << format("Estimated sensor 'sigma': %f\n", obs->stdError );
			cout << format("Increment in pitch during the scan: %f deg\n", RAD2DEG( obs->deltaPitch ));

			size_t i,inval = 0;
			for (i=0;i<obs->scan.size();i++) if (!obs->validRange[i]) inval++;
			cout << format("Invalid points in the scan: %u\n", (unsigned)inval);

			cout << format("Sensor maximum range: %.02f m\n", obs->maxRange );
			cout << format("Sensor field-of-view (\"aperture\"): %.01f deg\n", RAD2DEG(obs->aperture) );

			cout << "Raw scan values: [";
			for (i=0;i<obs->scan.size();i++) cout << format("%.03f ", obs->scan[i] );
			cout << "]\n";

			cout << "Raw valid-scan values: [";
			for (i=0;i<obs->scan.size();i++) cout << format("%u ", obs->validRange[i] ? 1:0 );
			cout << "]\n\n";

			// The plot:
			mrpt::slam::CSimplePointsMap  dummMap;
			dummMap.insertionOptions.minDistBetweenLaserPoints = 0;
			dummMap.insertObservation( obs.pointer() );

			vector<float>    Xs,Ys;
			dummMap.getAllPoints(Xs,Ys);

			lyScan2D->SetData(Xs,Ys);
			plotScan2D->Fit();      // Update the window to show the new data fitted.
		}
		else
			if ( classID  == CLASS_ID(CObservationImage) )
			{
				// ----------------------------------------------------------------------
				//              CObservationImage
				// ----------------------------------------------------------------------
				Notebook1->ChangeSelection( 3 );
				CObservationImagePtr obs = CObservationImagePtr( sel_obj );
				curSelectedObservation = CObservationPtr( sel_obj );

				cout << "Homogeneous matrix for the sensor's 3D pose, relative to robot base:\n";
				cout << obs->cameraPose.getHomogeneousMatrixVal()
				<< obs->cameraPose << endl;

				cout << format("Focal length: %.03f mm\n",obs->cameraParams.focalLengthMeters*1000);

				cout << "Intrinsic parameters matrix for the camera:"<< endl
				<< obs->cameraParams.intrinsicParams.inMatlabFormat() << endl << obs->cameraParams.intrinsicParams << endl;

				cout << "Distorsion parameters for the camera: "
				<< obs->cameraParams.getDistortionParamsAsVector() << endl;

				if (obs->image.isExternallyStored())
					cout << " Image is stored externally in file: " << obs->image.getExternalStorageFile() << endl;

				cout << format(" Image size: %ux%u pixels\n", (unsigned int)obs->image.getWidth(), (unsigned int)obs->image.getHeight() );

				cout << " Channels order: " << obs->image.getChannelsOrder() << endl;

				cout << format(" Rows are stored in top-bottom order: %s\n",
							   obs->image.isOriginTopLeft() ? "YES" : "NO");

				// Get bitmap:
				// ----------------------
				wxImage *img = mrpt::gui::MRPTImage2wxImage( obs->image );
				bmpObsImage->SetBitmap( wxBitmap(*img) );
				bmpObsImage->Refresh();
				delete img;
				obs->image.unload(); // For externally-stored datasets
			}
			else
				if ( classID  == CLASS_ID(CObservationStereoImages) )
				{
					// ----------------------------------------------------------------------
					//              CObservationStereoImages
					// ----------------------------------------------------------------------
					Notebook1->ChangeSelection( 4);
					CObservationStereoImagesPtr obs = CObservationStereoImagesPtr(sel_obj);
					curSelectedObservation = CObservationPtr( sel_obj );


					cout << "Homogeneous matrix for the sensor's 3D pose, relative to robot base:\n";
					cout << obs->cameraPose.getHomogeneousMatrixVal()
					<< obs->cameraPose << endl;

					cout << "Homogeneous matrix for the RIGHT camera's 3D pose, relative to LEFT camera reference system:\n";
					cout << obs->rightCameraPose.getHomogeneousMatrixVal()
					<< obs->rightCameraPose << endl;

					cout << "Intrinsic parameters matrix for the camera:"<< endl;
					CMatrixDouble33 aux = obs->leftCamera.intrinsicParams;
					cout << aux.inMatlabFormat() << endl << aux << endl;

					if (obs->imageLeft.isExternallyStored())
						cout << " Left image is stored externally in file: " << obs->imageLeft.getExternalStorageFile() << endl;

					cout << " Right image";
					if (obs->hasImageRight )
					{
						if (obs->imageRight.isExternallyStored())
							cout << " is stored externally in file: " << obs->imageRight.getExternalStorageFile() << endl;
					}
					else cout << " : No.\n";

					cout << " Disparity image";
					if (obs->hasImageDisparity )
					{
						if (obs->imageDisparity.isExternallyStored())
							cout << " is stored externally in file: " << obs->imageDisparity.getExternalStorageFile() << endl;
					}
					else cout << " : No.\n";

					cout << format(" Image size: %ux%u pixels\n", (unsigned int)obs->imageLeft.getWidth(), (unsigned int)obs->imageLeft.getHeight() );

					cout << " Channels order: " << obs->imageLeft.getChannelsOrder() << endl;

					cout << format(" Rows are stored in top-bottom order: %s\n",
								   obs->imageLeft.isOriginTopLeft() ? "YES" : "NO");

					// Images:
					// ----------------------
					wxImage *imgLeft = mrpt::gui::MRPTImage2wxImage( obs->imageLeft );
					bmpObsStereoLeft->SetBitmap( wxBitmap(*imgLeft) );
					bmpObsStereoLeft->Refresh();
					delete imgLeft;

					wxImage *imgRight = mrpt::gui::MRPTImage2wxImage( obs->imageRight );
					bmpObsStereoRight->SetBitmap( wxBitmap(*imgRight) );
					bmpObsStereoRight->Refresh();
					delete imgRight;

					wxImage *imgDisp = mrpt::gui::MRPTImage2wxImage( obs->imageDisparity );
					bmpObsStereoDisp->SetBitmap( wxBitmap(*imgDisp) );
					bmpObsStereoDisp->Refresh();
					delete imgDisp;

				}
					else
					if ( classID == CLASS_ID(CActionRobotMovement2D) )
					{
						// ----------------------------------------------------------------------
						//              CActionRobotMovement2D
						// ----------------------------------------------------------------------
						Notebook1->ChangeSelection( 1 );

						CActionRobotMovement2DPtr act = CActionRobotMovement2DPtr( sel_obj );

						CPose2D			Ap;
						CMatrixDouble33 mat;
						act->poseChange->getCovarianceAndMean(mat,Ap);

						cout << "Robot Movement (as a gaussian pose change):\n";
						cout << " Mean = " << Ap << endl;

						cout << format(" Covariance:     DET=%e\n", mat.det());

						cout << format("      %e %e %e\n", mat(0,0), mat(0,1), mat(0,2) );
						cout << format("      %e %e %e\n", mat(1,0), mat(1,1), mat(1,2) );
						cout << format("      %e %e %e\n", mat(2,0), mat(2,1), mat(2,2) );

						cout << endl;

						cout << " Actual reading from the odometry increment = " << act->rawOdometryIncrementReading << endl;

						cout << format("Actual PDF class is: '%s'\n",
									   act->poseChange->GetRuntimeClass()->className );

						if (act->poseChange->GetRuntimeClass()==CLASS_ID(CPosePDFParticles))
						{
							CPosePDFParticlesPtr aux = CPosePDFParticlesPtr( act->poseChange );
							cout << format (" (Particle count = %u)\n", (unsigned)aux->m_particles.size() );
						}
						cout << endl;

						cout << "Estimation method: ";
						switch (act->estimationMethod)
						{
						case CActionRobotMovement2D::emOdometry:
							cout << "emOdometry\n";
							break;
						case CActionRobotMovement2D::emScan2DMatching:
							cout << "emScan2DMatching\n";
							break;
						default:
							cout <<  "(Unknown ID!)\n";
							break;
						};

						// Additional data:
						if (act->hasEncodersInfo)
						{
							cout << format(" Encoder info: deltaL=%i deltaR=%i\n", act->encoderLeftTicks, act->encoderRightTicks );
						}
						else    cout << "Encoder info: Not available!\n";

						if (act->hasVelocities)
						{
							cout << format(" Velocity info: v=%.03f m/s  w=%.03f deg/s\n", act->velocityLin, RAD2DEG(act->velocityAng) );
						}
						else    cout << "Velocity info: Not available!\n";


						// Plot the 2D pose samples:
						unsigned int                    N = 1000;
						vector<vector_double>       samples;
						vector<float>                    xs(N),ys(N),ps(N),dumm(N,0.1f);

						// Draw a set of random (x,y,phi) samples:
						act->poseChange->drawManySamples( N, samples );

						// Pass to vectors and draw them:
						for (unsigned int i=0;i<N;i++)
						{
							xs[i] = samples[i][0];
							ys[i] = samples[i][1];
							ps[i] = RAD2DEG(samples[i][2]);
						}

						lyAction2D_XY->SetData(xs,ys);
						lyAction2D_PHI->SetData(ps,dumm);

						plotAct2D_XY->Fit();
						plotAct2D_PHI->Fit();
					}
					else
						if ( classID  == CLASS_ID(CObservationBeaconRanges) )
						{
							// ----------------------------------------------------------------------
							//              CObservationBeaconRanges
							// ----------------------------------------------------------------------
							Notebook1->ChangeSelection( 5 );
							CObservationBeaconRangesPtr obs = CObservationBeaconRangesPtr( sel_obj );
							curSelectedObservation = CObservationPtr( sel_obj );


							cout << "Auxiliary estimated pose (if available): " << obs->auxEstimatePose << endl;

							cout << format("minSensorDistance=%f m\n",obs->minSensorDistance);
							cout << format("maxSensorDistance=%f m\n",obs->maxSensorDistance);
							cout << format("stdError=%f m\n\n",obs->stdError);

							cout << format("There are %u range measurements:\n\n",(unsigned)obs->sensedData.size());

							cout << "  BEACON   RANGE     SENSOR POSITION ON ROBOT \n";
							cout << "------------------------------------------------\n";

							deque<CObservationBeaconRanges::TMeasurement>::iterator it;

							for (it=obs->sensedData.begin(); it!=obs->sensedData.end(); it++)
							{
								cout << format("   %i      %.04f      (%.03f,%.03f,%.03f)\n",
											   (int)it->beaconID,it->sensedDistance,
											   it->sensorLocationOnRobot.x(),it->sensorLocationOnRobot.y(),it->sensorLocationOnRobot.z());
							}
						}
						else
							if ( classID  == CLASS_ID(CObservationGasSensors) )
							{
								// ----------------------------------------------------------------------
								//              CObservationGasSensors
								// ----------------------------------------------------------------------
								Notebook1->ChangeSelection( 6 );
								CObservationGasSensorsPtr obs = CObservationGasSensorsPtr( sel_obj );
								curSelectedObservation = CObservationPtr( sel_obj );



								for (size_t j=0;j<obs->m_readings.size();j++)
								{
									cout << format("e-nose #%u:\n",(unsigned)j);

									vector<float>::iterator it;
									vector_int::iterator   itKind;

									ASSERT_( obs->m_readings[j].readingsVoltage.size() == obs->m_readings[j].sensorTypes.size());

									for (it=obs->m_readings[j].readingsVoltage.begin(),itKind=obs->m_readings[j].sensorTypes.begin();it!=obs->m_readings[j].readingsVoltage.end();it++,itKind++)
										cout << format( "%04X: %.03f ", *itKind, *it);

									cout << endl;

									cout << format("  Sensor pose on robot: (x,y,z)=(%.02f,%.02f,%.02f)\n",
												   obs->m_readings[j].eNosePoseOnTheRobot.x,
												   obs->m_readings[j].eNosePoseOnTheRobot.y,
												   obs->m_readings[j].eNosePoseOnTheRobot.z );

									cout << "Measured temperature: ";
									if (obs->m_readings[j].hasTemperature)
										cout << format("%.03f degC\n", obs->m_readings[j].temperature );
									else
										cout << "NOT AVAILABLE\n";
								}
							}
							else
								if ( classID  == CLASS_ID(CObservationGPS) )
								{
									// ----------------------------------------------------------------------
									//              CObservationGPS
									// ----------------------------------------------------------------------
									Notebook1->ChangeSelection( 7 );
									CObservationGPSPtr obs = CObservationGPSPtr( sel_obj );
									curSelectedObservation = CObservationPtr( sel_obj );

									if (obs->has_GGA_datum)
										cout << endl << "Satellite time: " << format("%02u:%02u:%02.3f",obs->GGA_datum.UTCTime.hour,obs->GGA_datum.UTCTime.minute,obs->GGA_datum.UTCTime.sec) << endl;

									cout << "Sensor position on the robot: " << obs->sensorPose << endl;

									obs->dumpToConsole();
								}
								else
									if ( classID  == CLASS_ID(CObservationBearingRange) )
									{
										// ----------------------------------------------------------------------
										//              CObservationBearingRange
										// ----------------------------------------------------------------------
										Notebook1->ChangeSelection( 8 );
										CObservationBearingRangePtr obs = CObservationBearingRangePtr( sel_obj );
										curSelectedObservation = CObservationPtr( sel_obj );


										cout << "Homogeneous matrix for the sensor's 3D pose, relative to robot base:\n";
										cout << obs->sensorLocationOnRobot.getHomogeneousMatrixVal()
										<< obs->sensorLocationOnRobot << endl << endl;

										cout << "Do observations have individual covariance matrices? " << (obs->validCovariances ? "YES":"NO") << endl << endl;

										cout << "Default noise sigmas:" << endl;
										cout << "sensor_std_range (m)   : " << obs->sensor_std_range << endl;
										cout << "sensor_std_yaw   (deg) : " << RAD2DEG(obs->sensor_std_yaw) << endl;
										cout << "sensor_std_pitch (deg) : " << RAD2DEG(obs->sensor_std_pitch) << endl;

										cout << endl;

										// For each entry in this sequence:
										cout << "  LANDMARK_ID    RANGE (m)    YAW (deg)    PITCH (deg)   COV. MATRIX (optional)" << endl;
										cout << "--------------------------------------------------------------------------------------" << endl;
										for (size_t q=0;q<obs->sensedData.size();q++)
										{

											cout << "      ";
											if (obs->sensedData[q].landmarkID==INVALID_LANDMARK_ID)
												cout << "(NO ID)";
											else cout << format("%7u",obs->sensedData[q].landmarkID);

											cout << format("   %10.03f  %10.03f %10.03f        ",
														   obs->sensedData[q].range,
														   RAD2DEG( wrapToPi( obs->sensedData[q].yaw)),
														   RAD2DEG( wrapToPi(obs->sensedData[q].pitch)) );

											if (obs->validCovariances)
												cout << obs->sensedData[q].covariance.inMatlabFormat() << endl;
											else
												cout << "  (N/A)\n";
										}

										// The plot:
										size_t nPts = obs->sensedData.size();
										vector<float>    Xs(nPts),Ys(nPts),Zs(nPts);
										for (size_t k=0;k<nPts;k++)
										{
											float R      = obs->sensedData[k].range;
											float yaw    = obs->sensedData[k].yaw;
											float pitch  = obs->sensedData[k].pitch;

											CPoint3D   local(
												R * cos(yaw) * cos(pitch),
												R * sin(yaw) * cos(pitch),
												R * sin(pitch) );
											CPoint3D relRobot( obs->sensorLocationOnRobot + local);
											Xs[k] = relRobot.x();
											Ys[k] = relRobot.y();
											Zs[k] = relRobot.z();
										}
										lyRangeBearingLandmarks->SetData(Xs,Ys);
										plotRangeBearing->LockAspect();
										plotRangeBearing->Fit();      // Update the window to show the new data fitted.
									}
									else
										if ( classID  == CLASS_ID(CObservationBatteryState) )
										{
											// ----------------------------------------------------------------------
											//              CObservationBatteryState
											// ----------------------------------------------------------------------
											CObservationBatteryStatePtr obs = CObservationBatteryStatePtr( sel_obj);
											curSelectedObservation = CObservationPtr( sel_obj );
											cout << endl;

											cout << format("Measured VoltageMainRobotBattery: %.02fV  isValid= %s \n",
												obs->voltageMainRobotBattery,
												(obs->voltageMainRobotBatteryIsValid == true)? "True":"False" );

											cout << format("Measured VoltageMainRobotComputer: %.02fV  isValid= %s \n",
												obs->voltageMainRobotComputer,
												(obs->voltageMainRobotComputerIsValid == true)? "True":"False" );

											cout << "VoltageOtherBatteries: \n";
											for(vector_double::Index i=0; i<obs->voltageOtherBatteries.size(); i++)
											{
												cout << format("Index: %d --> %.02fV  isValid= %s \n",
												int(i),
												obs->voltageOtherBatteries[i],
												(obs->voltageOtherBatteriesValid[i] == true)? "True":"False" );
											}



										}
										else
										if ( classID  == CLASS_ID(CObservationWirelessPower) )
										{
											// ----------------------------------------------------------------------
											//              CObservationBatteryState
											// ----------------------------------------------------------------------
											CObservationWirelessPowerPtr obs = CObservationWirelessPowerPtr( sel_obj);
											curSelectedObservation = CObservationPtr( sel_obj );
											cout << endl;

											cout << format("Measured Power: %.02f/100\n",
												obs->power);
										}
										else
											if ( classID  == CLASS_ID(CObservationIMU) )
											{
												// ----------------------------------------------------------------------
												//              CObservationIMU
												// ----------------------------------------------------------------------
												Notebook1->ChangeSelection( 0 );
												CObservationIMUPtr obs = CObservationIMUPtr( sel_obj);
												curSelectedObservation = CObservationPtr( sel_obj );

												cout << "Sensor pose on the robot: " << obs->sensorPose << endl;

												cout << format("Orientation (degrees): (yaw,pitch,roll)=(%.06f, %.06f, %.06f)\n\n",
													RAD2DEG( obs->rawMeasurements[IMU_YAW] ),
													RAD2DEG( obs->rawMeasurements[IMU_PITCH] ),
													RAD2DEG( obs->rawMeasurements[IMU_ROLL] ) );

	#define DUMP_IMU_DATA(x)  \
		cout << #x << " = "; \
		if (obs->dataIsPresent[x]) \
			cout << obs->rawMeasurements[x]; \
		else  	cout << "(not present)"; \
		cout << endl;

												DUMP_IMU_DATA(IMU_X_ACC)
												DUMP_IMU_DATA(IMU_Y_ACC)
												DUMP_IMU_DATA(IMU_Z_ACC)
												DUMP_IMU_DATA(IMU_YAW_VEL)
												DUMP_IMU_DATA(IMU_PITCH_VEL)
												DUMP_IMU_DATA(IMU_ROLL_VEL)
												DUMP_IMU_DATA(IMU_X_VEL)
												DUMP_IMU_DATA(IMU_Y_VEL)
												DUMP_IMU_DATA(IMU_Z_VEL)
												DUMP_IMU_DATA(IMU_YAW)
												DUMP_IMU_DATA(IMU_PITCH)
												DUMP_IMU_DATA(IMU_ROLL)
												DUMP_IMU_DATA(IMU_X)
												DUMP_IMU_DATA(IMU_Y)
												DUMP_IMU_DATA(IMU_Z)


											}
											else
												if ( classID  == CLASS_ID(CObservationOdometry) )
												{
													// ----------------------------------------------------------------------
													//              CObservationOdometry
													// ----------------------------------------------------------------------
													Notebook1->ChangeSelection( 0 );
													CObservationOdometryPtr obs = CObservationOdometryPtr( sel_obj );
													curSelectedObservation = CObservationPtr( sel_obj );

													cout << endl << "Odometry reading: " << obs->odometry << endl;

													// Additional data:
													if (obs->hasEncodersInfo)
													{
														cout << format(" Encoder info: deltaL=%i deltaR=%i\n", obs->encoderLeftTicks, obs->encoderRightTicks );
													}
													else    cout << "Encoder info: Not available!\n";

													if (obs->hasVelocities)
													{
														cout << format(" Velocity info: v=%.03f m/s  w=%.03f deg/s\n", obs->velocityLin, RAD2DEG(obs->velocityAng) );
													}
													else   cout << "Velocity info: Not available!\n";

												}
												else
													if ( classID  == CLASS_ID(CActionRobotMovement3D) )
													{
														// ----------------------------------------------------------------------
														//              CActionRobotMovement2D
														// ----------------------------------------------------------------------
														//Notebook1->ChangeSelection( 1 );
														CActionRobotMovement3DPtr act = CActionRobotMovement3DPtr( sel_obj );
														cout << "Robot Movement (as a gaussian pose change):\n";
														cout << act->poseChange << endl;
													}
													else
													if ( classID  == CLASS_ID(CObservationRange) )
													{
														// ----------------------------------------------------------------------
														//              CObservationRange
														// ----------------------------------------------------------------------
														Notebook1->ChangeSelection( 0 );
														CObservationRangePtr obs = CObservationRangePtr( sel_obj );
														curSelectedObservation = CObservationPtr( sel_obj );

														cout << endl;
														cout << "minSensorDistance   = " << obs->minSensorDistance << " m" << endl;
														cout << "maxSensorDistance   = " << obs->maxSensorDistance << " m" << endl;
														cout << "sensorConeApperture = " << RAD2DEG(obs->sensorConeApperture) << " deg" << endl;

														// For each entry in this sequence:
														cout << "  SENSOR_ID    RANGE (m)    SENSOR POSE (on the robot)" << endl;
														cout << "-------------------------------------------------------" << endl;
														for (size_t q=0;q<obs->sensedData.size();q++)
														{
															cout << format("     %7u",(unsigned int)obs->sensedData[q].sensorID );
															cout << format("    %4.03f   ",obs->sensedData[q].sensedDistance);
															cout << obs->sensedData[q].sensorPose << endl;

														}
													}
													else
													if ( classID  == CLASS_ID(CObservation3DRangeScan) )
													{
														// ----------------------------------------------------------------------
														//              CObservation3DRangeScan
														// ----------------------------------------------------------------------
														Notebook1->ChangeSelection( 9 );
														CObservation3DRangeScanPtr obs = CObservation3DRangeScanPtr( sel_obj );

														obs->load(); // Make sure the 3D point cloud, etc... are all loaded in memory.

														curSelectedObservation = CObservationPtr( sel_obj );


														cout << endl;
														cout << "maxRange = " << obs->maxRange << " m" << endl;

														const bool generate3Donthefly = !obs->hasPoints3D && mnuItemEnable3DCamAutoGenPoints->IsChecked();
														if (generate3Donthefly)
														{
															obs->project3DPointsFromDepthImage();
														}

														cout << "Has 3D point cloud? ";
														if (obs->hasPoints3D)
														{
															cout << "YES: " << obs->points3D_x.size() << " points";
															if (obs->points3D_isExternallyStored())
																cout << ". External file: " << obs->points3D_getExternalStorageFile() << endl;
															else cout << " (embedded)." << endl;
														}
														else	cout << "NO" << endl;

														if (generate3Donthefly)
															cout << "NOTICE: The stored observation didn't contain 3D points, but these have been generated on-the-fly just for visualization purposes.\n"
															"(You can disable this behavior from the menu Sensors->3D depth cameras\n\n";

														cout << "Has raw range data? " << (obs->hasRangeImage ? "YES": "NO");
														if (obs->hasRangeImage)
														{
															if (obs->rangeImage_isExternallyStored())
																 cout << ". External file: " << obs->rangeImage_getExternalStorageFile() << endl;
															else cout << " (embedded)." << endl;
														}

														cout << endl << "Has intensity data? " << (obs->hasIntensityImage ? "YES": "NO");
														if (obs->hasIntensityImage)
														{
															if (obs->intensityImage.isExternallyStored())
																cout << ". External file: " << obs->intensityImage.getExternalStorageFile() << endl;
															else cout << " (embedded).\n";
															// Channel?
															cout << "Source channel: " << mrpt::utils::TEnumType<CObservation3DRangeScan::TIntensityChannelID>::value2name(obs->intensityImageChannel) << endl;
														}

														cout << endl << "Has confidence data? " << (obs->hasConfidenceImage ? "YES": "NO");
														if (obs->hasConfidenceImage)
														{
															if (obs->confidenceImage.isExternallyStored())
																cout << ". External file: " << obs->confidenceImage.getExternalStorageFile() << endl;
															else cout << " (embedded)." << endl;
														}

														cout << endl << endl;
														cout << "Depth camera calibration parameters:" << endl;
														{
															CConfigFileMemory cfg;
															obs->cameraParams.saveToConfigFile("DEPTH_CAM_PARAMS",cfg);
															cout << cfg.getContent() << endl;
														}
														cout << endl << "Intensity camera calibration parameters:" << endl;
														{
															CConfigFileMemory cfg;
															obs->cameraParamsIntensity.saveToConfigFile("INTENSITY_CAM_PARAMS",cfg);
															cout << cfg.getContent() << endl;
														}
														cout << endl << endl << "Pose of the intensity cam. wrt the depth cam:\n"
															<< obs->relativePoseIntensityWRTDepth << endl
															<< obs->relativePoseIntensityWRTDepth.getHomogeneousMatrixVal() << endl;

														// Update 3D view ==========
													#if RAWLOGVIEWER_HAS_3D
														this->m_gl3DRangeScan->m_openGLScene->clear();
														//this->m_gl3DRangeScan->m_openGLScene->insert( mrpt::opengl::stock_objects::CornerXYZ() );
														this->m_gl3DRangeScan->m_openGLScene->insert( mrpt::opengl::CAxis::Create(-20,-20,-20,20,20,20,1,2,true ));

														mrpt::opengl::CPointCloudColouredPtr pnts = mrpt::opengl::CPointCloudColoured::Create();
														CColouredPointsMap  pointMap;
														pointMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;

														if (obs->hasPoints3D)
														{
															// Assign only those points above a certain threshold:
															const int confThreshold =   obs->hasConfidenceImage ? slid3DcamConf->GetValue() : 0;

															if (confThreshold==0) // This includes when there is no confidence image.
															{
																pointMap.insertionOptions.minDistBetweenLaserPoints = 0; // don't drop any point
																pointMap.insertObservation(obs.pointer());
																pnts->loadFromPointsMap(&pointMap);
															}
															else
															{
																pnts->clear();

																const vector<float>  &obs_xs = obs->points3D_x;
																const vector<float>  &obs_ys = obs->points3D_y;
																const vector<float>  &obs_zs = obs->points3D_z;

																size_t i=0;

																const size_t W = obs->confidenceImage.getWidth();
																const size_t H = obs->confidenceImage.getHeight();

																ASSERT_(obs->confidenceImage.isColor()==false)
																ASSERT_(obs_xs.size() == H*W)

																for(size_t r=0;r<H;r++)
																{
																	unsigned char const * ptr_lin = obs->confidenceImage.get_unsafe(0,r,0);
																	for(size_t c=0;c<W;c++,  i++ )
																	{
																		unsigned char conf = *ptr_lin++;
																		if (conf>=confThreshold)
																			pnts->push_back(obs_xs[i],obs_ys[i],obs_zs[i],1,1,1);
																	}
																}
															}

															pnts->setPointSize(4.0);

															// Translate the 3D cloud since sensed points are relative to the camera, but the camera may be translated wrt the robot (our 0,0,0 here):
															pnts->setPose( obs->sensorPose );
														}
														this->m_gl3DRangeScan->m_openGLScene->insert(pnts);
														this->m_gl3DRangeScan->Refresh();

														// Free memory:
														if (generate3Donthefly)
														{
															obs->hasPoints3D = false;
															obs->resizePoints3DVectors(0);
														}

													#endif

														// Update intensity image ======
														{
															CImage im;
															if (obs->hasIntensityImage)
																 im = obs->intensityImage;
															else im.resize(10,10,CH_GRAY, true);
															wxImage *img = mrpt::gui::MRPTImage2wxImage( im );
															if (img->IsOk())
																bmp3Dobs_int->SetBitmap( wxBitmap(*img) );
															bmp3Dobs_int->Refresh();
															delete img;
															obs->intensityImage.unload(); // For externally-stored datasets
														}
														// Update depth image ======
														{
															CImage  auxImg;
															if (obs->hasRangeImage)
															{
																// Convert to range [0,255]
																mrpt::math::CMatrix normalized_range = obs->rangeImage;
																const float max_rang = std::max(obs->maxRange, normalized_range.maximum() );
																if (max_rang>0) normalized_range *= 255./max_rang;
																auxImg.setFromMatrix(normalized_range, false /* it's in range [0,255] */);
															}
															else auxImg.resize(10,10, CH_GRAY, true );

															wxImage *img = mrpt::gui::MRPTImage2wxImage( auxImg );
															if (img->IsOk())
																bmp3Dobs_depth->SetBitmap( wxBitmap(*img) );
															bmp3Dobs_depth->Refresh();
															delete img;
														}
														// Update confidence image ======
														{
															wxImage *img;
															if (obs->hasConfidenceImage)
																img = mrpt::gui::MRPTImage2wxImage( obs->confidenceImage );
															else
															{
																mrpt::utils::CImage dumm(10,10);
																img = mrpt::gui::MRPTImage2wxImage( dumm );
															}
															if (img->IsOk())
																bmp3Dobs_conf->SetBitmap( wxBitmap(*img) );
															bmp3Dobs_conf->Refresh();
															delete img;
															obs->confidenceImage.unload(); // For externally-stored datasets
														}

														obs->unload();

													}
													else
													if ( classID  == CLASS_ID(CObservationStereoImagesFeatures) )
													{
														// ----------------------------------------------------------------------
														//              CObservationStereoImagesFeatures
														// ----------------------------------------------------------------------
														CObservationStereoImagesFeaturesPtr obs = CObservationStereoImagesFeaturesPtr( sel_obj );
														curSelectedObservation = CObservationPtr( sel_obj );

														cout << "Homogeneous matrix for the sensor's 3D pose, relative to robot base:\n";
														cout << obs->cameraPoseOnRobot.getHomogeneousMatrixVal()
														<< obs->cameraPoseOnRobot << endl;

														cout << "Homogeneous matrix for the RIGHT camera's 3D pose, relative to LEFT camera reference system:\n";
														cout << obs->rightCameraPose.getHomogeneousMatrixVal()
														<< obs->rightCameraPose << endl;

														cout << "Intrinsic parameters matrix for the LEFT camera:"<< endl;
														CMatrixDouble33 aux = obs->cameraLeft.intrinsicParams;
														cout << aux.inMatlabFormat() << endl << aux << endl;

														cout << "Distortion parameters vector for the LEFT camera:"<< endl << "[ ";
														for( unsigned int i = 0; i < 5; ++i )
															cout << obs->cameraLeft.dist[i] << " ";
														cout << "]" << endl;

														cout << "Intrinsic parameters matrix for the RIGHT camera:"<< endl;
														aux = obs->cameraRight.intrinsicParams;
														cout << aux.inMatlabFormat() << endl << aux << endl;

														cout << "Distortion parameters vector for the RIGHT camera:"<< endl << "[ ";
														for( unsigned int i = 0; i < 5; ++i )
															cout << obs->cameraRight.dist[i] << " ";
														cout << "]"<< endl;

														cout << endl << format(" Image size: %ux%u pixels\n", (unsigned int)obs->cameraLeft.ncols, (unsigned int)obs->cameraLeft.nrows );
														cout << endl << format(" Number of features in images: %u\n", (unsigned int)obs->theFeatures.size() );

													}
													else
													{
														// Other selections:
														Notebook1->ChangeSelection( 0 );
													}



		} // scope for redirector
		memo->ShowPosition(0);
	}
	else
	{
		Notebook1->ChangeSelection( 0 );

		memo->Freeze();                 // Freeze the window to prevent scrollbar jumping
		memo->Clear();
		{
			CMyRedirector myRedirector( memo );

			// Show comments of the rawlog:
			string s;
			rawlog.getCommentText( s );

			if (s.empty())
			{
				cout << "(The rawlog has no comments)" << endl;
			}
			else
			{
				cout << s;
			}
		}
		// Set focus on the first line:
		memo->ShowPosition(0);
		memo->Thaw();  // Allow the window to redraw
	}

  }
  catch( utils::CExceptionExternalImageNotFound &e )
  {
	  wxMessageBox( _U(e.what()), _("Error with a delayed load image"), wxOK, this );

	  if (wxYES==wxMessageBox(
		  _U(format( "The current directory for relative images is:\n%s\n\nDo you want to set it to a different one?", CImage::IMAGES_PATH_BASE.c_str() ).c_str()),
		_("Error with delayed loading image"), wxYES_NO, this) )
	  {
			// Change CImage::IMAGES_PATH_BASE
			wxDirDialog dirDialog(
				this,
				_("Choose the base directory for relative image paths"),
				_U(CImage::IMAGES_PATH_BASE.c_str()), 0, wxDefaultPosition );
			if (dirDialog.ShowModal()==wxID_OK)
			{
				CImage::IMAGES_PATH_BASE = string( dirDialog.GetPath().mb_str() );
			}
	  }
  }
  catch(std::exception &e)
  {
		wxMessageBox( wxString(e.what(),wxConvUTF8), wxT("Exception"), wxOK, this);
  }
}


#if wxUSE_STARTUP_TIPS
	class CMyTips : public wxTipProvider
	{
	public:
		CMyTips(size_t n) : wxTipProvider(n) { }

		virtual wxString GetTip()
		{
			size_t idx = m_currentTip++;

			static const size_t N_TIPS = 5;

			switch ( idx % N_TIPS )
			{
            case 0: return _("To have a first overview of a dataset with odometry and laser scans, select 'Tools' -> 'maps & paths generation module' -> 'Map from odometry'.\n If the dataset is very large, select a portion of it, or use the 'decimation' slide.");
            case 1: return _("When observations are selected in the tree-view at the left, extended information will be shown for that observation: timestamp, a visualization of the laser scan/image, etc.");
            case 2: return _("Portions of a rawlog can be stripped out in Edit ->Edit Rawlog");
            case 3: return _("When a rawlog is loaded, the panel at the bottom displays a summary of the log, including approximate overall distance traveled by the vehicle, overall time, and the frequency of each sensor.");
            case 4: return _("There is an ICP experimenting module that can be opened from the toolbar button 'ICP'");
			}
			return wxString();
		}
	};
#endif

//------------------------------------------------------------------------
// Auto-load the file passed in the cmd line (if any)
//------------------------------------------------------------------------
void xRawLogViewerFrame::OntimAutoLoadTrigger(wxTimerEvent& event)
{
	// To fix a strange bug in windows!: The window is not drawn correctly:
	Panel5->Refresh();

	// Now: Open it:
	if (global_fileToOpen.size())
		loadRawlogFile( global_fileToOpen );
	else	rebuildTreeView();

	// Tips:
	showNextTip();
}

//------------------------------------------------------------------------
// Show next tip
//------------------------------------------------------------------------
void xRawLogViewerFrame::showNextTip(bool forceShow)
{
#if wxUSE_STARTUP_TIPS
	// Show tips?
	bool    user_wants_to_show = iniFile->read_bool("tips","show",true);

	if (forceShow || user_wants_to_show)
	{
		size_t  last_tip = iniFile->read_int("tips","next",0);
		CMyTips	*myTips = new CMyTips(last_tip);
		bool cont_showing = ::wxShowTip(this, myTips);

		// save cont_showing:
		iniFile->write("tips","show",cont_showing);
		size_t next_tip = myTips->GetCurrentTip();
		iniFile->write("tips","next",(int)next_tip);

		delete myTips;
	}
#endif
}


//------------------------------------------------------------------------
// Tabs can only be changed programatically.
//------------------------------------------------------------------------
void xRawLogViewerFrame::OnNotebook1PageChanging(wxNotebookEvent& event)
{
	event.Veto();
}


//------------------------------------------------------------------------
// Changes the motion model parameters.
//------------------------------------------------------------------------
void xRawLogViewerFrame::OnChangeMotionModel(wxCommandEvent& event)
{
	// Create the dialog:
	CFormMotionModel    formMotionModel(this);
	formMotionModel.ShowModal();
}

//------------------------------------------------------------------------
// Menu cmd: Show images of a file as a video:
//------------------------------------------------------------------------
void xRawLogViewerFrame::OnShowImagesAsVideo(wxCommandEvent& event)
{
	CFormPlayVideo    diag(this);

	diag.getImageDirsCombo()->Clear();
	for ( unsigned int i=0;i<toolbarcomboImages->GetCount();i++ )
		diag.getImageDirsCombo()->Append( toolbarcomboImages->GetString(i) );

	diag.getImageDirsCombo()->SetSelection( toolbarcomboImages->GetSelection() );

	diag.ShowModal();

	toolbarcomboImages->SetSelection( diag.getImageDirsCombo()->GetSelection() );
}

//------------------------------------------------------------------------
//               Dialog: Build map directly from odometry/gps
//------------------------------------------------------------------------
void xRawLogViewerFrame::OnRawMapOdo(wxCommandEvent& event)
{
	if (rawlog.size()<1)
	{
		wxMessageBox(_("Please load a rawlog first!"),_("Rawlog is empty"),wxOK,this);
		return;
	}

	// Set slider values:
	//  If they have changed, we have a different rawlog loaded, thus we select
	//  the whole range by default:
	bool selectMaxRange = ((size_t)formRawMap->slFrom->GetMax()) != rawlog.size()-1;
	formRawMap->slFrom->SetRange( 0,(int)rawlog.size()-1 );
	formRawMap->slTo->SetRange( 0,(int)rawlog.size()-1 );

	formRawMap->edFirst->SetRange( 0,(int)rawlog.size()-1 );
	formRawMap->edLast->SetRange( 0,(int)rawlog.size()-1 );

	if (selectMaxRange)
	{
		formRawMap->slFrom->SetValue(0);
		formRawMap->slTo->SetValue((int)rawlog.size()-1);
	}

	// Clear the graphs:
	formRawMap->plotMap->DelAllLayers(true,false);

	// Show:
	formRawMap->edFirst->SetValue( formRawMap->slFrom->GetValue() );
	formRawMap->edLast->SetValue( formRawMap->slTo->GetValue() );

	// Enable "results" buttons:
	formRawMap->btnSaveTxt->Disable();
	formRawMap->btnSave3D->Disable();
	formRawMap->btnSavePath->Disable();
	formRawMap->btnSaveTxt->Disable();
	formRawMap->btnSaveObsPath->Disable();
	formRawMap->btnView3D->Disable();


	formRawMap->ShowModal();
}

void xRawLogViewerFrame::OnMenuGenerateBeaconList(wxCommandEvent& event)
{
	WX_START_TRY

	wxString caption = wxT("Save as...");
	wxString wildcard = wxT("Text files (*.txt)|*.txt|All files (*.*)|*.*");
	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
	wxString defaultFilename = _U( (loadedFileName+string("_beacons.txt")).c_str() );
	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

	if (dialog.ShowModal() == wxID_OK)
	{
		wxString fileName = dialog.GetPath();
		string        fil( fileName.mbc_str() );

		int             i, M = 0,  n = (int)rawlog.size();
		//            bool	    ref_valid = false;

		FILE            *f = os::fopen( fil.c_str(), "wt");
		if (!f)
			THROW_EXCEPTION("Cannot open output file for write.");

		for (i=0;i<n;i++)
		{
			switch( rawlog.getType(i) )
			{
				case CRawlog::etSensoryFrame:
				{
					CSensoryFramePtr sf = rawlog.getAsObservations(i);
					CObservationBeaconRangesPtr obs = sf->getObservationByClass<CObservationBeaconRanges>();

					if (obs)
					{
						::fprintf(f,"%u ", i);

						for (unsigned int k = 0;k< 10; k++)
						{
							float	rng = 0;
							if (k<obs->sensedData.size())
								rng = obs->sensedData[k].sensedDistance;

							if (isNaN(rng)) rng = 0;

							::fprintf(f,"%f ", rng);
						}
						::fprintf(f,"\n");
						M++;
					}
				}
				break;

				case CRawlog::etObservation:
				{
					CObservationPtr o = rawlog.getAsObservation(i);
					if (o->GetRuntimeClass()!=CLASS_ID(CObservationBeaconRanges)) break;
					CObservationBeaconRangesPtr obs = CObservationBeaconRangesPtr( o );
					::fprintf(f,"%u ", i);

					for (unsigned int k = 0;k< 10; k++)
					{
						float	rng = 0;
						if (k<obs->sensedData.size())
							rng = obs->sensedData[k].sensedDistance;

						if (isNaN(rng)) rng = 0;

						::fprintf(f,"%f ", rng);
					}
					::fprintf(f,"\n");
					M++;
				}
				break;

				default:
					break;
			}
		}

		os::fclose(f);

		char auxStr[100];
		os::sprintf(auxStr,sizeof(auxStr),"%u entries saved!",M);
		wxMessageBox(_U(auxStr),_("Done"),wxOK,this);
	}


	WX_END_TRY
}


string xRawLogViewerFrame::AskForImageFileFormat()
{
	#define SIZE_lstImgFormats 5
	wxString lstImgFormats[SIZE_lstImgFormats] = { _("jpg"),_("png"),_("bmp"),_("tif"),_("pgm") };

	int ret = wxGetSingleChoiceIndex(
		_("Choose the format of the output images:"),
		_("Images format"),
		SIZE_lstImgFormats,
		lstImgFormats,
		this );

	if (ret==-1)
			return string("");
	else	return string( lstImgFormats[ret].mb_str() );
}



void xRawLogViewerFrame::OnGenOdoLaser(wxCommandEvent& event)
{
	WX_START_TRY

	if (!rawlog.size())
	{
		::wxMessageBox(_("Load a rawlog first."));
		return;
	}

	const wxString &target_dir_wx = ::wxDirSelector(
		_("Select the directory where the files will be saved"),
		_U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ),
		wxDD_DEFAULT_STYLE, wxDefaultPosition, this);

	if (target_dir_wx.empty()) return;

	const string target_dir = string( target_dir_wx.mb_str() );

	const string fil_odo       = target_dir + string("/") + mrpt::system::extractFileName(loadedFileName) + string("_ODO.txt");
	const string fil_odo_times = target_dir + string("/") + mrpt::system::extractFileName(loadedFileName) + string("_ODO_times.txt");

	const string prefix_laser = target_dir + string("/") + mrpt::system::extractFileName(loadedFileName) + string("_LASER_");

	unsigned int      i, n = (unsigned int)rawlog.size();

	bool	genTimes = rawlog_first_timestamp != INVALID_TIMESTAMP;
	if (!genTimes)
		::wxMessageBox(_("It seems that there are no valid timestamps in the rawlog. The time files will not be generated."));

	FILE	*f_odo = os::fopen( fil_odo, "wt");
	ASSERT_(f_odo)

	FILE	*f_odo_times = NULL;
	if (genTimes)
	{
		f_odo_times = os::fopen( fil_odo_times, "wt");
		ASSERT_(f_odo_times)
	}

	// Prepare for possibly several lasers: label -> < file: data, file: times, file: pose >
	std::map<string , std::pair<FILE*,std::pair<FILE*,FILE*> > >   lstFiles;

	CPose2D		lastOdo;
	bool		lastOdo_ok = false;

	unsigned int nOdo=0, nLaser=0;

	wxBusyCursor	cursor_wait;

	wxProgressDialog    progDia(
		wxT("Exporting as text files"),
		wxT("Saving..."),
		n, // range
		this, // parent
		wxPD_CAN_ABORT |
		wxPD_APP_MODAL |
		wxPD_SMOOTH |
		wxPD_AUTO_HIDE |
		wxPD_ELAPSED_TIME |
		wxPD_ESTIMATED_TIME |
		wxPD_REMAINING_TIME);

	for (i=0;i<n;i++)
	{
		if (i % 50 == 0)
		{
			if (!progDia.Update(i) )
				break;
			wxTheApp->Yield();  // Let the app. process messages
		}

		switch (rawlog.getType(i))
		{
		case CRawlog::etActionCollection:
			{
				CActionCollectionPtr  acts = rawlog.getAsAction(i);
				CActionRobotMovement2DPtr action = acts->getBestMovementEstimation();
				if (!action)
					THROW_EXCEPTION_CUSTOM_MSG1("No odometry action found in rawlog entry %i!",i)

				CPose2D                 poseIncrement = action->poseChange->getMeanVal();

				::fprintf(f_odo,"%f %f %f\n", poseIncrement.x(),poseIncrement.y(), poseIncrement.phi() );
				nOdo++;

				if (genTimes)
				{
					double t = 0;
					if (action->timestamp!=INVALID_TIMESTAMP)
						//t = mrpt::system::timeDifference(rawlog_first_timestamp, action->timestamp);
						t = timestampTotime_t(action->timestamp);

					::fprintf(f_odo_times,"%f\n", t );
				}
			}
			break;
		case CRawlog::etSensoryFrame:
			{
				CSensoryFramePtr sf = rawlog.getAsObservations(i);
				//int nScans=0;
				for (unsigned int k=0;k<sf->size();k++)
				{
					if ( sf->getObservationByIndex(k)->GetRuntimeClass()==CLASS_ID(CObservation2DRangeScan) )
					{
						CObservation2DRangeScanPtr obs = sf->getObservationByIndexAs<CObservation2DRangeScanPtr>(k);

						// Get files from list, or create them the first time:
						std::pair<FILE*,std::pair<FILE*,FILE*> > *files=NULL;
						std::map<string , std::pair<FILE*,std::pair<FILE*,FILE*> > >::iterator it;
						if ((it = lstFiles.find(obs->sensorLabel)) == lstFiles.end())
						{
							files = &lstFiles[obs->sensorLabel];
							// Open files:
							files->first = os::fopen( prefix_laser+obs->sensorLabel+string(".txt"),"wt" );
							ASSERT_(files->first);
							files->second.first = os::fopen( prefix_laser+obs->sensorLabel+string("_times.txt"),"wt" );
							ASSERT_(files->second.first);
							files->second.second = os::fopen( prefix_laser+obs->sensorLabel+string("_poses.txt"),"wt" );
							ASSERT_(files->second.second);
						}
						else
						{
							files = &lstFiles[obs->sensorLabel];
						}

						for (size_t j=0;j<obs->scan.size();j++)
							::fprintf(files->first,"%f ", obs->validRange[j] ? obs->scan[j] : 0 );
						::fprintf(files->first,"\n");

						nLaser++;

						if (genTimes)
						{
							double t = 0;
							if (obs->timestamp!=INVALID_TIMESTAMP)
								//t = mrpt::system::timeDifference(rawlog_first_timestamp, obs->timestamp);
								t = timestampTotime_t(obs->timestamp);
							::fprintf(files->second.first,"%f\n", t );
						}

						// dump pose data
						double y,p,r;
						obs->sensorPose.getYawPitchRoll(y, p, r);
						::fprintf(files->second.second,"%f\t%f\t%f\n", y, p, r);
					}
				}
			}
			break;

		case CRawlog::etObservation:
			{
				CObservationPtr o = rawlog.getAsObservation(i);

				if (IS_CLASS(o,CObservation2DRangeScan))
				{
					CObservation2DRangeScanPtr obs = CObservation2DRangeScanPtr( o );

					// Get files from list, or create them the first time:
					std::pair<FILE*,std::pair<FILE*,FILE*> > *files=NULL;
					std::map<string , std::pair<FILE*,std::pair<FILE*,FILE*> > >::iterator it;
					if ((it = lstFiles.find(obs->sensorLabel)) == lstFiles.end())
					{
						files = &lstFiles[obs->sensorLabel];
						// Open files:
						files->first = os::fopen( prefix_laser+obs->sensorLabel+string(".txt"),"wt" );
						ASSERT_(files->first);
						files->second.first = os::fopen( prefix_laser+obs->sensorLabel+string("_times.txt"),"wt" );
						ASSERT_(files->second.first);
						files->second.second = os::fopen( prefix_laser+obs->sensorLabel+string("_poses.txt"),"wt" );
						ASSERT_(files->second.second);
					}
					else
					{
						files = &lstFiles[obs->sensorLabel];
					}

					for (size_t j=0;j<obs->scan.size();j++)
						::fprintf(files->first,"%f ", obs->validRange[j] ? obs->scan[j] : 0 );
					::fprintf(files->first,"\n");

					nLaser++;

					if (genTimes)
					{
						double t = 0;
						if (obs->timestamp!=INVALID_TIMESTAMP)
							//t = mrpt::system::timeDifference(rawlog_first_timestamp, obs->timestamp);
							t = timestampTotime_t(obs->timestamp);

						::fprintf(files->second.first,"%f\n", t );
					}

					// dump pose data
					double y,p,r;
					obs->sensorPose.getYawPitchRoll(y, p, r);
					::fprintf(files->second.second,"%f\t%f\t%f\n", y, p, r);
				}
				else if (IS_CLASS(o,CObservationOdometry))
				{
					CObservationOdometryPtr odo = CObservationOdometryPtr(o);

					CPose2D		poseIncrement(0,0,0);
					if (!lastOdo_ok)
					{
						lastOdo_ok = true;
					}
					else
					{
						poseIncrement = odo->odometry - lastOdo;
						lastOdo = odo->odometry;
					}

					::fprintf(f_odo,"%f %f %f\n", poseIncrement.x(),poseIncrement.y(), poseIncrement.phi() );
					nOdo++;

					if (genTimes)
					{
						double t = 0;
						if (odo->timestamp!=INVALID_TIMESTAMP)
							//t = mrpt::system::timeDifference(rawlog_first_timestamp, odo->timestamp);
							t = timestampTotime_t(odo->timestamp);
						::fprintf(f_odo_times,"%f\n", t );
					}
				}
			}
			break;


			// Error:
		default:
			THROW_EXCEPTION("Unknown element type in the rawlog")

		} // end switch.
	}

	progDia.Update( n );


	os::fclose(f_odo);
	if (f_odo_times)
		os::fclose(f_odo_times);

	for (std::map<string, std::pair<FILE*,std::pair<FILE*,FILE*> > >::iterator i=lstFiles.begin();i!=lstFiles.end();++i)
	{
		os::fclose(i->second.first);
		os::fclose(i->second.second.first);
		os::fclose(i->second.second.second);
	}

	wxMessageBox( wxString::Format(_("%u odometry & %u laser entries saved."), nOdo, nLaser ),_("Done"),wxOK,this);

	WX_END_TRY
}

void xRawLogViewerFrame::OnShowICP(wxCommandEvent& event)
{
	if (rawlog.size()<1)
	{
		wxMessageBox(_("Please load a rawlog first!"),_("Rawlog is empty"),wxOK,this);
		return;
	}

	scanMatchingDialog->Hide();
	scanMatchingDialog->Show();
	scanMatchingDialog->Maximize();
}

void xRawLogViewerFrame::OnLoadAPartOnly(wxCommandEvent& event)
{
	string 	fil;
	if ( !AskForOpenRawlog( fil ) ) return;

	// Query the first and last entries to load:
	wxString strFirst = wxGetTextFromUser(
							_("Enter the first entry index to load:"),
							_("Load a part of rawlog:"),
							_("0"));
	wxString strLast = wxGetTextFromUser(
						   _("Enter the last entry index to load (-1:Last):"),
						   _("Load a part of rawlog:"),
						   _("-1"));
	long first;
	strFirst.ToLong( &first );
	long last;
	strLast.ToLong( &last );

	loadRawlogFile( fil, first,last);

}

void xRawLogViewerFrame::OnFileCountEntries(wxCommandEvent& event)
{
	WX_START_TRY

	string 	str;
	if ( !AskForOpenRawlog( str ) ) return;

	wxBusyCursor        waitCursor;
	CFileGZInputStream	fil(str);
	unsigned int              filSize = (unsigned int)fil.getTotalBytesCount();

	wxString            auxStr;
	wxProgressDialog    progDia(
		wxT("Progress of rawlog read"),
		wxT("Parsing file..."),
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
	int					entryIndex = 0;
	bool                keepLoading=true;
	string              errorMsg;

	while (keepLoading)
	{
		if (countLoop++ % 100 == 0)
		{
			auxStr.sprintf(wxT("Parsing file... %u objects"),entryIndex );
			if (!progDia.Update( (int)fil.getPosition(), auxStr ))
				keepLoading = false;
			wxTheApp->Yield();  // Let the app. process messages
		}

		CSerializablePtr newObj;
		try
		{
			fil >> newObj;
			// Check type:
			if ( newObj->GetRuntimeClass() == CLASS_ID(CSensoryFrame) ||
					newObj->GetRuntimeClass() == CLASS_ID(CActionCollection) ||
					newObj->GetRuntimeClass() == CLASS_ID(CPose2D) )
			{
				entryIndex++;
			}
			else
			{
				// Unknown class:
				THROW_EXCEPTION("Unknown class found in the file!");
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

	progDia.Update( filSize );

	// Set error msg:
	char auxStr2[100];
	os::sprintf(auxStr2,sizeof(auxStr2),"There are %i entries in the rawlog file",entryIndex);
	wxMessageBox(_U(auxStr2),_("Count done"),wxOK,this);

	WX_END_TRY

}

void xRawLogViewerFrame::OnFileSaveImages(wxCommandEvent& event)
{
	WX_START_TRY

	string 	str;
	if ( !AskForOpenRawlog( str ) ) return;

	// ask for the output directory:
	wxDirDialog dirDialog( this, _("Choose the output directory for the images"),
						   _("."), 0, wxDefaultPosition );

	if (dirDialog.ShowModal()!=wxID_OK) return;
	string outDir( dirDialog.GetPath().mb_str() );

	// Let the user choose the image format:
	string imgFileExtension = AskForImageFileFormat();
	if (imgFileExtension.empty()) return;

	wxBusyCursor        waitCursor;
	CFileGZInputStream	fil(str);
	unsigned int        filSize = (unsigned int)fil.getTotalBytesCount();

	wxString            auxStr;
	wxProgressDialog    progDia(
		wxT("Progress"),
		wxT("Parsing file..."),
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

	unsigned int              countLoop = 0;
	int					imgSaved = 0;
	bool                keepLoading=true;
	string              errorMsg;

	while (keepLoading)
	{
		if (countLoop++ % 5 == 0)
		{
			auxStr.sprintf(wxT("Parsing file... %u objects"),countLoop );
			if (!progDia.Update( (int)fil.getPosition(), auxStr ))
				keepLoading = false;
			wxTheApp->Yield();  // Let the app. process messages
		}

		CSerializablePtr newObj;
		try
		{
			fil >> newObj;
			// Check type:
			if ( newObj->GetRuntimeClass() == CLASS_ID(CSensoryFrame) )
			{
				CSensoryFramePtr SF = CSensoryFramePtr( CSerializablePtr(newObj) );

				for (unsigned k=0;k<SF->size();k++)
				{
					if (SF->getObservationByIndex(k)->GetRuntimeClass()==CLASS_ID(CObservationStereoImages ) )
					{
						CObservationStereoImagesPtr obsSt = SF->getObservationByIndexAs<CObservationStereoImagesPtr>(k);
						obsSt->imageLeft.saveToFile( format("%s/img_stereo_%u_left_%05u.%s",outDir.c_str(),k,imgSaved,imgFileExtension.c_str()) );
						if (obsSt->hasImageRight) obsSt->imageRight.saveToFile( format("%s/img_stereo_%u_right_%05u.%s",outDir.c_str(),k,imgSaved,imgFileExtension.c_str()) );
						if (obsSt->hasImageDisparity) obsSt->imageDisparity.saveToFile( format("%s/img_stereo_%u_disp_%05u.%s",outDir.c_str(),k,imgSaved,imgFileExtension.c_str()) );
						imgSaved++;
					}
					if (SF->getObservationByIndex(k)->GetRuntimeClass()==CLASS_ID(CObservationImage ) )
					{
						CObservationImagePtr obsIm = SF->getObservationByIndexAs<CObservationImagePtr>(k);

						obsIm->image.saveToFile( format("%s/img_monocular_%u_%05u.%s",outDir.c_str(),k,imgSaved,imgFileExtension.c_str()) );
						imgSaved++;
					}
				}
			}
			else
				if ( newObj->GetRuntimeClass() == CLASS_ID(CActionCollection) ||
						newObj->GetRuntimeClass() == CLASS_ID(CPose2D) )
				{

				}
				else
				{
					// Unknown class:
					THROW_EXCEPTION("Unknown class found in the file!");
				}

			newObj.clear();
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

	progDia.Update( filSize );

	// Set error msg:
	wxMessageBox(_U(format("Images saved: %i",imgSaved).c_str()),_("Done"),wxOK,this);

	WX_END_TRY
}

void xRawLogViewerFrame::OnFileGenVisualLMFromStereoImages(wxCommandEvent& event)
{

}

void xRawLogViewerFrame::OnMRUFile(wxCommandEvent& event)
{
	wxString f(m_fileHistory.GetHistoryFile(event.GetId() - wxID_FILE1));
	if (!f.empty())
		loadRawlogFile( string( f.mb_str() ) );
}





void wxStaticBitmapPopup::OnPopupSaveImage(wxCommandEvent& event)
{
	try
	{
		if (!theMainWindow || !theMainWindow->curSelectedObservation) return;
		CObservationPtr curSelectedObservation = theMainWindow->curSelectedObservation;

		CImage *imgToSave = NULL;

		cout << "[xRawLogViewerFrame::OnPopupSaveImage] Current observation class: " << curSelectedObservation->GetRuntimeClass()->className << endl;

		if (curSelectedObservation->GetRuntimeClass()==CLASS_ID(CObservationImage))
		{
			CObservationImage	*obs = (CObservationImage*) curSelectedObservation.pointer();
			imgToSave = &obs->image;
		}
		else if ( curSelectedObservation->GetRuntimeClass()==CLASS_ID(CObservationStereoImages) )
		{
			CObservationStereoImages *obs = (CObservationStereoImages*) curSelectedObservation.pointer();

			switch(theMainWindow->Notebook2->GetSelection())
			{
				case 0: imgToSave = &obs->imageLeft; break;
				case 1: imgToSave = &obs->imageRight; break;
				case 2: imgToSave = &obs->imageDisparity; break;
			}
		}

		if (imgToSave)
		{
			wxFileDialog dialog(
				this,
				_("Save image as...") /*caption*/,
				_U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) /* defaultDir */,
				_("image.png") /* defaultFilename */,
				_("Image files (*.bmp;*.gif;*.jpg;*.jpeg;*.png;*.tif)|*.bmp;*.gif;*.jpg;*.jpeg;*.png;*.tif") /* wildcard */,
				wxFD_SAVE | wxFD_OVERWRITE_PROMPT );
			if (dialog.ShowModal() != wxID_OK) return;

			// Save image:
			string filName( dialog.GetPath().mb_str() );
			imgToSave->saveToFile( filName );
		}
	}
	catch (...)
	{
		cerr << "[xRawLogViewerFrame::OnPopupSaveImage] Ignoring untyped exception!" << endl;
	}
}

void wxStaticBitmapPopup::OnPopupLoadImage(wxCommandEvent& event)
{
	try
	{
		if (!theMainWindow || !theMainWindow->curSelectedObservation) return;
		CObservationPtr curSelectedObservation = theMainWindow->curSelectedObservation;

		CImage *imgToLoad = NULL;

		if (curSelectedObservation->GetRuntimeClass()==CLASS_ID(CObservationImage))
		{
			CObservationImage	*obs = (CObservationImage*) curSelectedObservation.pointer();
			imgToLoad = &obs->image;
		}
		else if ( curSelectedObservation->GetRuntimeClass()==CLASS_ID(CObservationStereoImages) )
		{
			CObservationStereoImages *obs = (CObservationStereoImages*) curSelectedObservation.pointer();

			switch (theMainWindow->Notebook2->GetSelection())
			{
				case 0: imgToLoad = &obs->imageLeft; break;
				case 1: imgToLoad = &obs->imageRight; break;
				case 2: imgToLoad = &obs->imageDisparity; break;
			}
		}

		if (imgToLoad)
		{
			wxFileDialog dialog(
				this,
				_("Load image...") /*caption*/,
				_U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) /* defaultDir */,
				_("image.png") /* defaultFilename */,
				_("Image files (*.bmp;*.gif;*.jpg;*.jpeg;*.png;*.tif)|*.bmp;*.gif;*.jpg;*.jpeg;*.png;*.tif|All files (*.*)|*.*") /* wildcard */,
				wxFD_OPEN | wxFD_FILE_MUST_EXIST );
			if (dialog.ShowModal() != wxID_OK) return;

			// Replace image:
			string filName( dialog.GetPath().mb_str() );
			imgToLoad->loadFromFile( filName );
		}
	}
	catch (...)
	{
		cerr << "[xRawLogViewerFrame::OnPopupSaveImage] Ignoring untyped exception!" << endl;
	}
}


void xRawLogViewerFrame::OnChangeSensorPositions(wxCommandEvent& event)
{
	CFormChangeSensorPositions	dialog(this);

	dialog.edLabel->Clear();
	for (std::map<std::string,TInfoPerSensorLabel>::iterator i=listOfSensorLabels.begin();i!=listOfSensorLabels.end();++i)
		dialog.edLabel->Append( _U( i->first.c_str() ) );

	dialog.ShowModal();
}

void xRawLogViewerFrame::OnDecimateRecords(wxCommandEvent& event)
{
	WX_START_TRY

	CRawlog	newRawLog;
	newRawLog.setCommentText( rawlog.getCommentText() );

	wxString strDecimation = wxGetTextFromUser(
								 _("The number of observations will be decimated (only 1 out of M will be kept). Enter the decimation ratio M:"),
								 _("Decimation"),
								 _("1") );
	long	DECIMATE_RATIO;
	strDecimation.ToLong( &DECIMATE_RATIO );

	ASSERT_(DECIMATE_RATIO>=1)

	wxBusyCursor	busyCursor;
	wxTheApp->Yield();  // Let the app. process messages

	size_t	i, N = rawlog.size();

	// ------------------------------------------------------------------------------
	// METHOD TO BE MEMORY EFFICIENT:
	//  To free the memory of the current rawlog entries as we create the new one,
	//  then call "clearWithoutDelete" at the end.
	// ------------------------------------------------------------------------------
	CSensoryFramePtr last_sf ( CSerializablePtr( static_cast<CSerializable*>(NULL)) );
	CActionRobotMovement2D::TMotionModelOptions	odometryOptions;
	bool					cummMovementInit = false;
	long 					SF_counter = 0;

	// Reset cummulative pose change:
	CPose2D		accumMovement(0,0,0);

	// For each entry:
	for (i=0;i<N;i++)
	{
		CSerializablePtr obj = rawlog.getAsGeneric(i);
		bool objToBeDeleted = true; // Will be set to false if "obj" cannot be deleted now.

		if (rawlog.getType(i)==CRawlog::etActionCollection)
		{
			// Accumulate Actions
			// ----------------------
			CActionCollectionPtr curActs = CActionCollectionPtr( obj);
			CActionRobotMovement2DPtr mov = curActs->getBestMovementEstimation();
			if (mov)
			{
				// Accumulate from odometry:
				accumMovement = accumMovement + mov->poseChange->getMeanVal();

				// Copy the probabilistic options from the first entry we find:
				if (!cummMovementInit)
				{
					odometryOptions = mov->motionModelConfiguration;
					cummMovementInit = true;
				}
			}
		}
		else if (rawlog.getType(i)==CRawlog::etSensoryFrame)
		{
			// Decimate Observations
			// ---------------------------
			if (!last_sf)
			{
				last_sf = CSensoryFramePtr(obj);
				objToBeDeleted = false; // Do not delete this one
			}

			if ( ++SF_counter >= DECIMATE_RATIO )
			{
				SF_counter = 0;

				// INSERT OBSERVATION:
				newRawLog.addObservationsMemoryReference( last_sf );
				last_sf.clear_unique(); // = NULL;

				// INSERT ACTIONS:
				CActionCollection	actsCol;
				if (cummMovementInit)
				{
					CActionRobotMovement2D	cummMovement;
					cummMovement.computeFromOdometry(accumMovement, odometryOptions );
					actsCol.insert( cummMovement );
					// Reset odometry accumulation:
					accumMovement = CPose2D(0,0,0);
				}
				newRawLog.addActions( actsCol );
			}
		}
		else
		{
			THROW_EXCEPTION("This is only for rawlogs based on sensory frames.");
		}

		// Delete object?
		if (objToBeDeleted) obj.clear_unique(); //delete obj;

	} // end for i each entry

	// Clear the list only (objects already deleted)
	rawlog.clearWithoutDelete();

	// Copy as new log:
	rawlog = newRawLog;

	rebuildTreeView();

	WX_END_TRY
}

void xRawLogViewerFrame::OnCountBadScans(wxCommandEvent& event)
{
	WX_START_TRY

	wxBusyCursor        waitCursor;
	int				nEntries = (int)rawlog.size();

	wxString            auxStr;
	wxProgressDialog    progDia(
		wxT("Progress"),
		wxT("Parsing rawlog..."),
		nEntries, // range
		this, // parent
		wxPD_CAN_ABORT |
		wxPD_APP_MODAL |
		wxPD_SMOOTH |
		wxPD_AUTO_HIDE |
		wxPD_ELAPSED_TIME |
		wxPD_ESTIMATED_TIME |
		wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages

	int					invalidScans = 0;
	string              errorMsg;

	for (int countLoop=0;countLoop<nEntries;countLoop++)
	{
		if (countLoop % 5 == 0)
		{
			auxStr.sprintf(wxT("Parsing rawlog... %u objects"),countLoop );
			if (!progDia.Update( countLoop, auxStr ))
				break;
			wxTheApp->Yield();  // Let the app. process messages
		}

		try
		{
			if ( rawlog.getType(countLoop) == CRawlog::etSensoryFrame )
			{
				CSensoryFramePtr SF = rawlog.getAsObservations(countLoop);

				for (size_t k=0;k<SF->size();k++)
				{
					if (SF->getObservationByIndex(k)->GetRuntimeClass()==CLASS_ID(CObservation2DRangeScan) )
					{
						CObservation2DRangeScanPtr obsScan = SF->getObservationByIndexAs<CObservation2DRangeScanPtr>(k);
						bool thisValid=false;

						for (size_t k=0;k<obsScan->validRange.size();k++)
						{
							if (obsScan->validRange[k])
								thisValid = true;
							if ( mrpt::math::isNaN(obsScan->scan[k]))
								thisValid = false;
						}

						if (!thisValid)  invalidScans++;
					}
				}
			} // end for each entry
			else if ( rawlog.getType(countLoop) == CRawlog::etObservation )
			{
				CObservationPtr o = rawlog.getAsObservation(countLoop);

				if (o->GetRuntimeClass()== CLASS_ID( CObservation2DRangeScan ) )
				{
					CObservation2DRangeScanPtr obsScan = CObservation2DRangeScanPtr(o);
					bool thisValid=false;

					for (size_t k=0;k<obsScan->validRange.size();k++)
					{
						if (obsScan->validRange[k])
							thisValid = true;
						if ( mrpt::math::isNaN(obsScan->scan[k]))
							thisValid = false;
					}

					if (!thisValid)  invalidScans++;
				}
			} // end for each entry
		}
		catch (exception &e)
		{
			errorMsg = e.what();
			break;
		}
		catch (...)
		{
			break;
		}
	} // end while keep loading

	progDia.Update( nEntries );

	wxMessageBox(_U( format("Found %u range scans with no valid range values.",invalidScans).c_str() ),_("Done"),wxOK,this);

	WX_END_TRY
}

void xRawLogViewerFrame::OnFilterSpureousGas(wxCommandEvent& event)
{
	WX_START_TRY

	wxString strMaxChange = wxGetTextFromUser(
								_("Maximum change between readings (volts):"),
								_("Filter out gas readings 'spikes':"),
								_("0.05"));
	double						maxChange;
	strMaxChange.ToDouble( &maxChange );

	wxBusyCursor        waitCursor;
	int				nEntries = (int)rawlog.size();

	wxString            auxStr;
	wxProgressDialog    progDia(
		wxT("Progress"),
		wxT("Parsing rawlog..."),
		nEntries, // range
		this, // parent
		wxPD_CAN_ABORT |
		wxPD_APP_MODAL |
		wxPD_SMOOTH |
		wxPD_AUTO_HIDE |
		wxPD_ELAPSED_TIME |
		wxPD_ESTIMATED_TIME |
		wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages

	int			nFilt = 0, nReadings = 0;
	string		errorMsg;
	CObservationGasSensorsPtr obs_1, obs_2;

	for (int countLoop=0;countLoop<nEntries;countLoop++)
	{
		if (countLoop % 5 == 0)
		{
			auxStr.sprintf(wxT("Parsing rawlog... %u objects"),countLoop );
			if (!progDia.Update( countLoop, auxStr ))
				break;
			wxTheApp->Yield();  // Let the app. process messages
		}

		try
		{
		    switch ( rawlog.getType(countLoop))
			{
            case CRawlog::etSensoryFrame:
                {
				CSensoryFramePtr	SF = rawlog.getAsObservations(countLoop);

				for (size_t k=0;k<SF->size();k++)
				{
					if (SF->getObservationByIndex(k)->GetRuntimeClass()==CLASS_ID(CObservationGasSensors) )
					{
						CObservationGasSensorsPtr obs = SF->getObservationByIndexAs<CObservationGasSensorsPtr>(k);

						// Do we have 3 consecutive readings??
						if (obs_1 && obs_2)
						{
							// Process:
							ASSERT_( obs->m_readings.size() == obs_1->m_readings.size() );
							ASSERT_( obs->m_readings.size() == obs_2->m_readings.size() );

							for (size_t j=0;j<obs->m_readings.size();j++)
							{
								ASSERT_( obs->m_readings[j].readingsVoltage.size() == obs_1->m_readings[j].readingsVoltage.size() );
								ASSERT_( obs->m_readings[j].readingsVoltage.size() == obs_2->m_readings[j].readingsVoltage.size() );

								for (size_t k = 0;k<obs->m_readings[j].readingsVoltage.size();k++)
								{
									nReadings++;
									// Compute difference for "t-1":
									if ( fabs( obs_1->m_readings[j].readingsVoltage[k] - obs->m_readings[j].readingsVoltage[k] ) > maxChange &&
											fabs( obs_1->m_readings[j].readingsVoltage[k] - obs_2->m_readings[j].readingsVoltage[k] ) > maxChange )
									{
										obs_1->m_readings[j].readingsVoltage[k] = 0.5f*(obs->m_readings[j].readingsVoltage[k]+obs_2->m_readings[j].readingsVoltage[k]);
										nFilt++;
									}
								}
							}
						}

						// Shift:
						obs_2 = obs_1;
						obs_1 = obs;
					}
				}
                } break;

            case CRawlog::etObservation:
                {
                    CObservationPtr	o = rawlog.getAsObservation(countLoop);

					if (IS_CLASS(o,CObservationGasSensors) )
					{
						CObservationGasSensorsPtr obs = CObservationGasSensorsPtr(o);

						// Do we have 3 consecutive readings??
						if (obs_1 && obs_2)
						{
							// Process:
							ASSERT_( obs->m_readings.size() == obs_1->m_readings.size() );
							ASSERT_( obs->m_readings.size() == obs_2->m_readings.size() );

							for (size_t j=0;j<obs->m_readings.size();j++)
							{
								ASSERT_( obs->m_readings[j].readingsVoltage.size() == obs_1->m_readings[j].readingsVoltage.size() );
								ASSERT_( obs->m_readings[j].readingsVoltage.size() == obs_2->m_readings[j].readingsVoltage.size() );

								for (size_t k = 0;k<obs->m_readings[j].readingsVoltage.size();k++)
								{
									nReadings++;
									// Compute difference for "t-1":
									if ( fabs( obs_1->m_readings[j].readingsVoltage[k] - obs->m_readings[j].readingsVoltage[k] ) > maxChange &&
											fabs( obs_1->m_readings[j].readingsVoltage[k] - obs_2->m_readings[j].readingsVoltage[k] ) > maxChange )
									{
										obs_1->m_readings[j].readingsVoltage[k] = 0.5f*(obs->m_readings[j].readingsVoltage[k]+obs_2->m_readings[j].readingsVoltage[k]);
										nFilt++;
									}
								}
							}
						}

						// Shift:
						obs_2 = obs_1;
						obs_1 = obs;
					}
                } break;

                default:
                    break;
			} // end for each entry
		}
		catch (exception &e)
		{
			errorMsg = e.what();
			break;
		}
		catch (...)
		{
			break;
		}
	} // end while keep loading

	progDia.Update( nEntries );

	wxMessageBox(_U( format("%u out of %u readings have been filtered out!",nFilt,nReadings).c_str() ),_("Done"),wxOK,this);

	WX_END_TRY
}

void xRawLogViewerFrame::OnRemoveSpecificRangeMeas(wxCommandEvent& event)
{
	WX_START_TRY

	wxString strStart = wxGetTextFromUser(
							_("First index of the rawlog to remove from:"),
							_("Remove range-only measurements."),
							_("0"));
	long start_filt;
	strStart.ToLong( &start_filt );

	wxString strEnd = wxGetTextFromUser(
						  _("Last index of the rawlog to remove from (-1:cancel & exit):"),
						  _("Remove range-only measurements."),
						  _("-1"));
	long end_filt;
	strEnd.ToLong( &end_filt );

	if (end_filt<0) return;

	wxString strIndex = wxGetTextFromUser(
							_("Index of the range within the observation (-1:None)"),
							_("Remove range-only measurements."),
							_("0"));
	unsigned long indx_filt;
	strIndex.ToULong( &indx_filt );

	vector<float>	lastValidRanges;

	wxBusyCursor        waitCursor;
	long				i,n= (long)rawlog.size();

	int			nFilt = 0, nReadings = 0;
	string		errorMsg;
	CObservationBeaconRangesPtr obs_1,obs_2;
	CObservationBeaconRangesPtr obs;
	size_t		        q;

	for (i=start_filt;i<=end_filt;i++)
	{
		switch (rawlog.getType(i))
		{
		case CRawlog::etSensoryFrame:
			{
				CSensoryFramePtr sf = rawlog.getAsObservations(i);
				obs = sf->getObservationByClass<CObservationBeaconRanges>();
				if (obs)
				{
					ASSERT_( indx_filt<obs->sensedData.size() );
					obs->sensedData[indx_filt].sensedDistance = 0;
				}
			}
			break;
		case CRawlog::etObservation:
			{
				CObservationPtr o = rawlog.getAsObservation(i);
				if (o->GetRuntimeClass()->derivedFrom( CLASS_ID(CObservation)) )
				{
					CObservationBeaconRangesPtr obs = CObservationBeaconRangesPtr( o );
					ASSERT_( indx_filt<obs->sensedData.size() );
					obs->sensedData[indx_filt].sensedDistance = 0;
				}
			}
			break;

		default:
			break;
		}; // end
	}

	for (i=0;i<n;i++)
	{
		switch ( rawlog.getType(i) )
		{
		case CRawlog::etSensoryFrame:
			{
				CSensoryFramePtr sf = rawlog.getAsObservations(i);
				obs = sf->getObservationByClass<CObservationBeaconRanges>();

				if (obs)
				{
					nReadings++;

					// Do we have 2 consecutive readings??
					if (obs_1 && obs_2)
					{
						ASSERT_( obs->sensedData.size() == obs_1->sensedData.size() );
						ASSERT_( obs_2->sensedData.size() == obs_1->sensedData.size() );

						for (q=0;q<obs->sensedData.size();q++)
						{
							bool filter = false;

							if ( isNaN(obs->sensedData[q].sensedDistance) )
								obs->sensedData[q].sensedDistance = 0;

							filter = isNaN(obs_2->sensedData[q].sensedDistance);

							if (!filter)
								filter = (obs->sensedData[q].sensedDistance   == obs_1->sensedData[q].sensedDistance &&
										  obs_1->sensedData[q].sensedDistance == obs_2->sensedData[q].sensedDistance );

							if (filter)
							{
								obs_2->sensedData[q].sensedDistance = 0;
							}
							nFilt++;
						}
					}

					// Shift:
					obs_2 = obs_1;
					obs_1 = obs;
				}
			}
		default:
			break;
		};
	}

	wxMessageBox(_U( format("%u out of %u readings have been filtered out!",nFilt,nReadings).c_str() ),_("Done"),wxOK,this);

	WX_END_TRY
}

void xRawLogViewerFrame::OnForceEncodersFalse(wxCommandEvent& event)
{
	WX_START_TRY

	bool clearEncoders = wxYES == wxMessageBox( _("Set 'hasEncodersInfo' fields to false?"),_("Select operations"),wxYES_NO,this );
	bool clearVelocities = wxYES == wxMessageBox( _("Set 'hasVelocities' fields to false?"),_("Select operations"),wxYES_NO,this );

	wxBusyCursor        waitCursor;
	int					nEntries = (int)rawlog.size();

	wxString            auxStr;
	wxProgressDialog    progDia(
		wxT("Progress"),
		wxT("Parsing rawlog..."),
		nEntries, // range
		this, // parent
		wxPD_CAN_ABORT |
		wxPD_APP_MODAL |
		wxPD_SMOOTH |
		wxPD_AUTO_HIDE |
		wxPD_ELAPSED_TIME |
		wxPD_ESTIMATED_TIME |
		wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages

	int			nChanges = 0;
	string		errorMsg;

	for (int countLoop=0;countLoop<nEntries;countLoop++)
	{
		if (countLoop % 20 == 0)
		{
			auxStr.sprintf(wxT("Parsing rawlog... %u objects"),countLoop );
			if (!progDia.Update( countLoop, auxStr ))
				break;
			wxTheApp->Yield();  // Let the app. process messages
		}

		try
		{
			if (rawlog.getType(countLoop) == CRawlog::etActionCollection )
			{
				CActionCollectionPtr acts = rawlog.getAsAction( countLoop );
				for (size_t k=0;k<acts->size();k++)
				{
					CActionPtr act = acts->get(k);
					if (act->GetRuntimeClass() == CLASS_ID( CActionRobotMovement2D ) )
					{
						CActionRobotMovement2DPtr a = CActionRobotMovement2DPtr( act );
						if (a->hasEncodersInfo && clearEncoders)
						{
							a->hasEncodersInfo = false;
							nChanges++;
						}
						if (a->hasVelocities && clearVelocities)
						{
							a->hasVelocities = false;
							nChanges++;
						}
					}
				}
			} // end for each entry
		}
		catch (exception &e)
		{
			errorMsg = e.what();
			break;
		}
		catch (...)
		{
			break;
		}
	} // end while keep loading

	progDia.Update( nEntries );

	wxMessageBox(_U( format("%u entries have been modified",nChanges).c_str() ),_("Done"),wxOK,this);

	WX_END_TRY
}

void doFilterErrScans(CObservationPtr &obs, size_t &invalidSegments, size_t &invalidRanges, bool &touched)
{
	if (obs->GetRuntimeClass()->derivedFrom( CLASS_ID( CObservation2DRangeScan ) ))
	{
		CObservation2DRangeScanPtr obsScan = CObservation2DRangeScanPtr(obs);

		// Try to filter our spureous ranges:
		// --------------------------------------------

		// Build a vector: each element is true if
		//    element[k] is the element[k] -/+ a value in [0.10-2.0], and viceversa with [k-1]
		std::vector<bool> 	ringing( obsScan->scan.size(), false );
		unsigned int k;

		for (k=1;k<(obsScan->scan.size()-1);k++)
		{
			if (obsScan->validRange[k] && obsScan->validRange[k-1] && obsScan->validRange[k+1])
			{
				int		dirPrior = 0 , dirPost =0;

				if (  obsScan->scan[k]>(obsScan->scan[k-1]+0.03f) &&
						obsScan->scan[k]<(obsScan->scan[k-1]+2.00f) )
					dirPrior = 1;
				if (  obsScan->scan[k]<(obsScan->scan[k-1]-0.03f) &&
						obsScan->scan[k]>(obsScan->scan[k-1]-2.00f) )
					dirPrior = -1;

				if (  obsScan->scan[k]>(obsScan->scan[k+1]+0.03f) &&
						obsScan->scan[k]<(obsScan->scan[k+1]+2.00f) )
					dirPost = -1;
				if (  obsScan->scan[k]<(obsScan->scan[k+1]-0.03f) &&
						obsScan->scan[k]>(obsScan->scan[k+1]-2.00f) )
					dirPost = 1;

				if (  (dirPrior==1 && dirPost==-1) || (dirPrior==-1 && dirPost==1) )
					ringing[k] = true;
			}
		}


		// Look for segments of 'K' consecutive ringing ranges, and mark them as not valid!!
		int		ringingStart = -1;
		for (k=1;k<(obsScan->scan.size()-1);k++)
		{
			if ( ringing[k] )
			{
				if (ringingStart==-1)
				{
					// First one in a segment:
					ringingStart = k;
				}
			}
			else
			{
				// End of segment?
				if (ringingStart!=-1)
				{
					// Length:
					size_t		len = (k-1) - ringingStart;
					if (len>2)
					{
						// Mark them as invalid!!
						for (size_t p=ringingStart;p<k;p++,invalidRanges++)
						{
							obsScan->validRange[p]=false;
							touched = true;
						}

						invalidSegments++;
					}
				}
				// We are out of any segment just now:
				ringingStart = -1;
			}
		} // end for "k" & mark as invalid
	} // end-if is a laser scan
}

void xRawLogViewerFrame::OnFilterErroneousScans(wxCommandEvent& event)
{
	WX_START_TRY


	wxMessageBox(_("This process will look for spurious ranges, e.g. caused by \ndirect sunlight in an indoor laser scanner."),_("Filter 2D range scans"),wxOK,this);

	wxBusyCursor        waitCursor;
	int					nEntries = (int)rawlog.size();

	wxString            auxStr;
	wxProgressDialog    progDia(
		wxT("Progress"),
		wxT("Parsing rawlog..."),
		nEntries, // range
		this, // parent
		wxPD_CAN_ABORT |
		wxPD_APP_MODAL |
		wxPD_SMOOTH |
		wxPD_AUTO_HIDE |
		wxPD_ELAPSED_TIME |
		wxPD_ESTIMATED_TIME |
		wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages

	size_t invalidSegments=0,invalidRanges=0;
	string 		lstTouched;
	string		errorMsg;

	for (int countLoop=0;countLoop<nEntries;countLoop++)
	{
		if (countLoop % 5 == 0)
		{
			auxStr.sprintf(wxT("Parsing rawlog... %u objects"),countLoop );
			if (!progDia.Update( countLoop, auxStr ))
				break;
			wxTheApp->Yield();  // Let the app. process messages
		}

		try
		{
			// Check type:
			switch (rawlog.getType(countLoop))
			{
			case CRawlog::etSensoryFrame:
				{
					// This is a SF:
					CSensoryFramePtr SF = rawlog.getAsObservations(countLoop);
					bool touched = false;

					for (size_t j=0;j<SF->size();j++)
					{
						CObservationPtr obs = SF->getObservationByIndex(j);
						doFilterErrScans(obs, invalidSegments, invalidRanges, touched);
					} // end for each Observation in the SF

					// Add to the list of modified entries:
					if (touched)
					{
						if (lstTouched.size()<1000)
							lstTouched += format("%u ",countLoop);
						else
						{
							static bool noMore = false;
							if (!noMore) lstTouched += "...";
							noMore=true;
						}
					}
				} break;

			case CRawlog::etObservation:
				{
					// This is a SF:
					CObservationPtr obs = rawlog.getAsObservation(countLoop);
					bool touched = false;

					doFilterErrScans(obs, invalidSegments, invalidRanges, touched);

					// Add to the list of modified entries:
					if (touched)
					{
						if (lstTouched.size()<1000)
							lstTouched += format("%u ",countLoop);
						else
						{
							static bool noMore = false;
							if (!noMore) lstTouched += "...";
							noMore=true;
						}
					}
				} break;

				default:
					break;
			}; // end switch
		}
		catch (exception &e)
		{
			errorMsg = e.what();
			break;
		}
		catch (...)
		{
			break;
		}
	} // end while keep loading

	progDia.Update( nEntries );

	wxMessageBox(_U( format("Number of bad segments detected & marked as invalid: %u \n(%u individual ranges). Rawlog indexes modified:\n%s",(unsigned)invalidSegments,(unsigned)invalidRanges, lstTouched.c_str()).c_str() ),_("Done"),wxOK,this);

	WX_END_TRY
}

void xRawLogViewerFrame::OnRecalculateActionsICP(wxCommandEvent& event)
{
	WX_START_TRY

	wxMessageBox(_("Please, modify as desired all the ICP algorithm options\nand the type of reference map from the following dialog, then close it to start recomputing the actions from scan matching."));

	// Take options from "scanMatchingDialog"
	scanMatchingDialog->btnRunICP->Hide();
	scanMatchingDialog->ShowModal();
	scanMatchingDialog->btnRunICP->Show();
	scanMatchingDialog->btnRunICP->Fit();


	// Ask for the min. ICP goodness:
	float minICPGoodness = 0.4f;

	// The reference and the new maps:
	CSimplePointsMap		refMapPt;
	COccupancyGridMap2D 	refMapGrid;
	CSimplePointsMap        newMapPt;
	CICP					icp;
	CICP::TReturnInfo		icpInfo;

	// The 2 SFs and the action between:
	CSensoryFramePtr		SF_ref;// = NULL;
	CSensoryFramePtr		SF_new;// = NULL;
	CActionCollectionPtr	act_between;// = NULL;

	CPosePDFPtr	poseEst;
	float     runTime;

	// Load ICP options:
	// ------------------------------------------
	CConfigFileMemory	icpCfg( CStringList( string( scanMatchingDialog->edOptICP->GetValue().mb_str() ) ) );
	icp.options.loadFromConfigFile( icpCfg,"ICP" );

	// EXTRA options:
	CPose2D  initialEst(0,0,0);

	// Create reference map & load its options:
	// ------------------------------------------
	bool  useGridMap = scanMatchingDialog->rbGrid->GetValue();
	if (!useGridMap)
	{
		CConfigFileMemory	refCfg( CStringList( string( scanMatchingDialog->edOptRefPnt->GetValue().mb_str() ) ) );
		refMapPt.insertionOptions.loadFromConfigFile(refCfg,"InsertionOptions");
	}
	else
	{
		CConfigFileMemory	refCfg( CStringList( string( scanMatchingDialog->edOptRefGrid->GetValue().mb_str() ) ) );
		float		gridRes = refCfg.read_float("Construction","resolution",0.05f);
		refMapGrid.setSize(-10,10,-10,10,gridRes);
		refMapGrid.insertionOptions.loadFromConfigFile(refCfg,"InsertionOptions");
	}

	CMetricMap *refMap = useGridMap ? (CMetricMap*)&refMapGrid : (CMetricMap*)&refMapPt;

	// Load new map options:
	// ----------------------------
	{
		CConfigFileMemory	refCfg( CStringList( string( scanMatchingDialog->edOptAlignMap->GetValue().mb_str() ) ) );
		newMapPt.insertionOptions.loadFromConfigFile(refCfg,"InsertionOptions");
	}



	wxBusyCursor        waitCursor;
	int				nEntries = (int)rawlog.size();

	wxString            auxStr;
	wxProgressDialog    progDia(
		wxT("Progress"),
		wxT("Parsing rawlog..."),
		nEntries, // range
		this, // parent
		wxPD_CAN_ABORT |
		wxPD_APP_MODAL |
		wxPD_SMOOTH |
		wxPD_AUTO_HIDE |
		wxPD_ELAPSED_TIME |
		wxPD_ESTIMATED_TIME |
		wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages

	string		errorMsg;

	for (int countLoop=0;countLoop<nEntries;countLoop++)
	{
		if (countLoop % 20 == 0)
		{
			auxStr.sprintf(wxT("Parsing rawlog... %u objects"),countLoop );
			if (!progDia.Update( countLoop, auxStr ))
				break;
			wxTheApp->Yield();  // Let the app. process messages
		}

		try
		{
			// Check type:
			if (rawlog.getType(countLoop)==CRawlog::etActionCollection)
			{
				act_between = rawlog.getAsAction(countLoop);
			}
			else if (rawlog.getType(countLoop)== CRawlog::etSensoryFrame )
			{
				// This is a SF:
				SF_new = rawlog.getAsObservations(countLoop);

				if (SF_new && SF_ref && act_between)
				{
					// Insert the observations:
					// --------------------------------------
					refMap->clear();
					newMapPt.clear();

					SF_ref->insertObservationsInto(refMap);
					CPose3D	newMapRobotPose( initialEst );
					SF_new->insertObservationsInto((CMetricMap*)&newMapPt, &newMapRobotPose);

					poseEst = icp.Align(
								  refMap,
								  (CMetricMap*)&newMapPt,
								  initialEst,
								  &runTime,
								  &icpInfo );


					// The final estimation:
					// --------------------------------------
					CPose2D	estMean;
					CMatrixDouble33 estCov;
					poseEst->getCovarianceAndMean(estCov,estMean);

					if (icpInfo.goodness>minICPGoodness)
					{
						// Add SM-based action or overwrite:
						CActionRobotMovement2DPtr act=act_between->getMovementEstimationByType(CActionRobotMovement2D::emScan2DMatching);
						if (!act)
						{
							// Create:
							CActionRobotMovement2D newAct;
							newAct.estimationMethod = CActionRobotMovement2D::emScan2DMatching;
							newAct.poseChange = CPosePDFPtr( new CPosePDFGaussian( estMean,estCov ) );
							act_between->insert( newAct );
						}
						else
						{
							// Overwrite:
							act->estimationMethod = CActionRobotMovement2D::emScan2DMatching;
							act->poseChange = CPosePDFPtr( new CPosePDFGaussian( estMean,estCov ) );
						}
					}
					else
					{
						// Remove SM-based action if it existed
						for (int k=((int)act_between->size())-1;k>=0;k--)
						{
							if (act_between->get(k)->GetRuntimeClass()==CLASS_ID(CActionRobotMovement2D))
							{
								CActionRobotMovement2DPtr act= CActionRobotMovement2DPtr( act_between->get(k) );
								if (act->estimationMethod==CActionRobotMovement2D::emScan2DMatching)
									act_between->eraseByIndex(k);
							}
						}
					}

				} // end if both new/ref SFs.

				// For the next iteration:
				SF_ref = SF_new;

			} // end if it's a SF
		}
		catch (exception &e)
		{
			errorMsg = e.what();
			break;
		}
		catch (...)
		{
			break;
		}
	} // end while keep loading

	progDia.Update( nEntries );

	rebuildTreeView();

	WX_END_TRY
}

// --------------------------------------------------------------
//
//    			POPUP MENU ON TREE VIEW
//
// --------------------------------------------------------------
//Right click on the tree view:
/*
void xRawLogViewerFrame::OntreeViewItemRightClick(wxTreeEvent& event)
{
	try
	{
		if (curSelectedObject)
		{
			// MNU: Delete
			MenuItem37->Enable( curSelectedObject->GetRuntimeClass() != CLASS_ID(CActionCollection) && curSelectedObject->GetRuntimeClass() != CLASS_ID(CSensoryFrame));
			// MNU: Add action
			MenuItem46->Enable( curSelectedObject->GetRuntimeClass() == CLASS_ID(CActionCollection) );
			MenuItem47->Enable( curSelectedObject->GetRuntimeClass() == CLASS_ID(CActionCollection) );
		}
	}
	catch (...){}

	treeView->PopupMenu( &mnuTree,event.GetPoint() );
}
*/

// Menu: Delete an element from the rawlog
void xRawLogViewerFrame::OnMenuItem37Selected(wxCommandEvent& event)
{
	WX_START_TRY

	if (curSelectedObject)
	{
		if (curSelectedObject->GetRuntimeClass()->derivedFrom(CLASS_ID(CObservation)))
		{
			bool done = false;
			// Look for its SF:
			for (size_t i=0;!done && i<rawlog.size();i++)
			{
				if (rawlog.getType(i)==CRawlog::etSensoryFrame)
				{
					CSensoryFramePtr	sf = rawlog.getAsObservations(i);
					for (size_t k=0;!done && k<sf->size();k++)
					{
						if (sf->getObservationByIndex(k)==curSelectedObject)
						{
							sf->eraseByIndex(k);
							done = true;
							break;
						}
					}
				}
				else if (rawlog.getType(i)==CRawlog::etActionCollection )
				{
					CActionCollectionPtr acts = rawlog.getAsAction(i);
					for (size_t k=0;!done && k<acts->size();k++)
					{
						if (acts->get(k)==curSelectedObject)
						{
							acts->eraseByIndex(k);
							done = true;
							break;
						}
					}
				}
			}
		}
	}

	rebuildTreeView();
	WX_END_TRY
}
// Menu: New action 2D (SM)
void xRawLogViewerFrame::OnMenuItem47Selected(wxCommandEvent& event)
{
	WX_START_TRY
	if (curSelectedObject)
	{
		if (curSelectedObject->GetRuntimeClass() == CLASS_ID(CActionCollection) )
		{
			CActionCollectionPtr acts = CActionCollectionPtr( curSelectedObject );

			float odo_x = atof( string( wxGetTextFromUser(
											_("Estimated displacement, X (meters):"),
											_("Enter new action parameter:"),
											_("0")).mb_str()).c_str() );
			float odo_y = atof( string( wxGetTextFromUser(
											_("Estimated displacement, Y (meters):"),
											_("Enter new action parameter:"),
											_("0")).mb_str()).c_str() );
			float odo_phi = DEG2RAD(atof( string( wxGetTextFromUser(
													  _("Estimated displacement, PHI (degrees):"),
													  _("Enter new action parameter:"),
													  _("0")).mb_str()).c_str() ));

			CPose2D estMean(odo_x,odo_y,odo_phi);
			CMatrixDouble33 estCov;
			estCov.unit(3,1e-6);

			CActionRobotMovement2D newAct;
			newAct.estimationMethod = CActionRobotMovement2D::emScan2DMatching;
			newAct.poseChange = CPosePDFPtr( new CPosePDFGaussian( estMean,estCov ) );

			acts->insert(newAct);
		}
	}
	rebuildTreeView();
	WX_END_TRY
}
// Menu: New action 2D (odometry)
void xRawLogViewerFrame::OnMenuItem46Selected(wxCommandEvent& event)
{
	WX_START_TRY
	if (curSelectedObject)
	{
		if (curSelectedObject->GetRuntimeClass() == CLASS_ID(CActionCollection) )
		{
			CActionCollectionPtr acts = CActionCollectionPtr( curSelectedObject );

			float odo_x = atof( string( wxGetTextFromUser(
											_("Estimated displacement, X (meters):"),
											_("Enter new action parameter:"),
											_("0")).mb_str()).c_str() );
			float odo_y = atof( string( wxGetTextFromUser(
											_("Estimated displacement, Y (meters):"),
											_("Enter new action parameter:"),
											_("0")).mb_str()).c_str() );
			float odo_phi = DEG2RAD(atof( string( wxGetTextFromUser(
													  _("Estimated displacement, PHI (degrees):"),
													  _("Enter new action parameter:"),
													  _("0")).mb_str()).c_str() ));

			CPose2D odo(odo_x,odo_y,odo_phi);

			CActionRobotMovement2D newAct;
			CActionRobotMovement2D::TMotionModelOptions odoOpts;  // Default values
			newAct.computeFromOdometry( odo, odoOpts);

			acts->insert(newAct);
		}
	}
	rebuildTreeView();
	WX_END_TRY
}

void xRawLogViewerFrame::OnMenuExpandAll(wxCommandEvent& event)
{
	//treeView->ExpandAll();
}

void xRawLogViewerFrame::OnMenuCollapseAll(wxCommandEvent& event)
{
	//treeView->CollapseAll();
}

void xRawLogViewerFrame::OnRecomputeOdometry(wxCommandEvent& event)
{
	WX_START_TRY

	COdometryParams		dialog(this);

	if ( dialog.ShowModal() )
	{
		float K_left,K_right, D;

		K_left = atof( string(dialog.edKL->GetValue().mb_str()).c_str() );
		K_right = atof( string(dialog.edKR->GetValue().mb_str()).c_str() );
		D = atof( string(dialog.edD->GetValue().mb_str()).c_str() );

		size_t  M = 0;

		{
			wxBusyCursor    waitCursor;

			for (size_t i=0;i<rawlog.size();i++)
			{
				if (rawlog.getType(i)==CRawlog::etActionCollection)
				{
					CActionCollectionPtr acts = CActionCollectionPtr( rawlog.getAsAction(i) );
					CActionRobotMovement2DPtr act = acts->getMovementEstimationByType( CActionRobotMovement2D::emOdometry );
					if (act)
					{
						if ( !act->hasEncodersInfo )
						{
							wxMessageBox( _U( format("An odometry measurement was found at entry %i which does not\ncontain encoders info: Cannot recompute odometry without this information!",(unsigned)i).c_str() ) );
							return;
						}
						act->computeFromEncoders( K_left,K_right, D );
						M++;
					}
				}
			}
		}

		wxMessageBox( _U( format("%u entries modified!",(unsigned)M).c_str() ) );
	}
	WX_END_TRY
}

// Generate text file with 1D range measurements
void xRawLogViewerFrame::OnRangeFinder1DGenTextFile(wxCommandEvent& event)
{
	WX_START_TRY

	wxString caption = wxT("Save as...");
	wxString wildcard = wxT("Text files (*.txt)|*.txt|All files (*.*)|*.*");
	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
	wxString defaultFilename = _U( (loadedFileName+string("_1DrangeFinders.txt")).c_str() );
	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

	if (dialog.ShowModal() == wxID_OK)
	{
		wxString fileName = dialog.GetPath();
		string        fil( fileName.mbc_str() );

		int             i, M = 0,  n = (int)rawlog.size();
		FILE            *f = os::fopen( fil.c_str(), "wt");
		if (!f)
			THROW_EXCEPTION("Cannot open output file for write.");

		for (i=0;i<n;i++)
		{
			if ( rawlog.getType(i) == CRawlog::etSensoryFrame )
			{
				CSensoryFramePtr sf = rawlog.getAsObservations(i);
				for (size_t k=0;k<sf->size();k++)
				{
					if (sf->getObservationByIndex(k)->GetRuntimeClass()==CLASS_ID(CObservationRange))
					{
						CObservationRangePtr obs = sf->getObservationByIndexAs<CObservationRangePtr>(k);

						vector<float>   rowOfRangesByID(100,0);
						for (CObservationRange::iterator it=obs->begin();it!=obs->end();++it)
						{
							ASSERT_(it->sensorID<rowOfRangesByID.size());
							rowOfRangesByID[it->sensorID]=it->sensedDistance;
						}

						::fprintf(f,"%06u ",i);
						for (size_t q=0;q<rowOfRangesByID.size();q++)
							::fprintf(f,"%03.04f ",rowOfRangesByID[q]);
						::fprintf(f,"\n");
						M++;
					}
				} // end for k
			}
			else if ( rawlog.getType(i) == CRawlog::etObservation )
			{
				CObservationPtr o = rawlog.getAsObservation(i);

				if (IS_CLASS(o,CObservationRange))
				{
					CObservationRangePtr obs = CObservationRangePtr(o);

					deque<CObservationRange::TMeasurement>::iterator it;
					vector<float>   rowOfRangesByID(100,0);
					for (it=obs->sensedData.begin();it!=obs->sensedData.end();it++)
					{
						ASSERT_(it->sensorID<rowOfRangesByID.size());
						rowOfRangesByID[it->sensorID]=it->sensedDistance;
					}

					::fprintf(f,"%06u ",i);
					for (size_t q=0;q<rowOfRangesByID.size();q++)
						::fprintf(f,"%03.04f ",rowOfRangesByID[q]);
					::fprintf(f,"\n");
					M++;
				}
			}

		}

		os::fclose(f);

		wxMessageBox(_U( format("%u entries saved!",M).c_str() ),_("Done"),wxOK,this);
	}

	WX_END_TRY
}

void xRawLogViewerFrame::OnMenuModifyICPActionsUncertainty(wxCommandEvent& event)
{
	WX_START_TRY

	float std_xy = atof( string( wxGetTextFromUser(
									 _("Standard deviation of x and y (meters):"),
									 _("ICP-based odometry uncertainty:"),
									 _("0.01")).mb_str()).c_str() );
	float std_phi = DEG2RAD( atof( string( wxGetTextFromUser(
											   _("Standard deviation of phi (degrees):"),
											   _("ICP-based odometry uncertainty:"),
											   _("0.2")).mb_str()).c_str() ) );

	float std_xy2 = square(std_xy);
	float std_phi2 = square(std_phi);

	size_t  M = 0;

	{
		wxBusyCursor    waitCursor;

		for (size_t i=0;i<rawlog.size();i++)
		{
			if (rawlog.getType(i)==CRawlog::etActionCollection)
			{
				CActionCollectionPtr acts = CActionCollectionPtr( rawlog.getAsAction(i) );
				for (unsigned int j=0;j<acts->size();j++)
				{
					CActionPtr act = acts->get(j);

					if (act->GetRuntimeClass()->derivedFrom( CLASS_ID( CActionRobotMovement2D ) ))
					{
						CActionRobotMovement2DPtr actMov = CActionRobotMovement2DPtr( act );

						if (actMov->estimationMethod == CActionRobotMovement2D::emScan2DMatching )
						{
							ASSERT_( actMov->poseChange->GetRuntimeClass() == CLASS_ID(CPosePDFGaussian) );

							CPosePDFGaussianPtr aux = CPosePDFGaussianPtr( actMov->poseChange );
							aux->cov.zeros();
							aux->cov(0,0) =
								aux->cov(1,1) = std_xy2;
							aux->cov(2,2) = std_phi2;

							actMov->motionModelConfiguration.modelSelection = CActionRobotMovement2D::mmGaussian;
							actMov->rawOdometryIncrementReading = aux->mean;
							M++;
						}
					}
				}
			}
		}
	}

	wxMessageBox( _U( format("%u entries modified!",(unsigned)M).c_str() ) );

	WX_END_TRY
}


void xRawLogViewerFrame::OnGenerateIMUTextFile(wxCommandEvent& event)
{
	WX_START_TRY

	wxString caption = wxT("Save as...");
	wxString wildcard = wxT("Text files (*.txt)|*.txt|All files (*.*)|*.*");
	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
	wxString defaultFilename = _U( (loadedFileName+string("_IMU.txt")).c_str() );
	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

	if ( dialog.ShowModal() == wxID_OK )
	{
		wxString fileName = dialog.GetPath();
		string        fil( fileName.mbc_str() );

		int             i, M = 0,  n = (int)rawlog.size();
		FILE            *f = os::fopen( fil.c_str(), "wt");
		if (!f)
			THROW_EXCEPTION("Cannot open output file for write.");

		::fprintf(f,
		"%% TIMESTAMP IMU_X_ACC IMU_Y_ACC IMU_Z_ACC IMU_YAW_VEL IMU_PITCH_VEL IMU_ROLL_VEL IMU_X_VEL IMU_Y_VEL IMU_Z_VEL IMU_YAW IMU_PITCH IMU_ROLL IMU_X IMU_Y IMU_Z\n"
		"%% ---------------------------------------------------------------------------------------------------------------------------------------------------------\n");

		for (i=0;i<n;i++)
		{
			switch (rawlog.getType(i))
			{
				default:
					break;

				case CRawlog::etSensoryFrame:
				{
					CSensoryFramePtr sf = rawlog.getAsObservations(i);
					size_t k = 0;
					CObservationIMUPtr obs;
					do {
						obs =sf->getObservationByClass<CObservationIMU>(k++);
						if (obs)
						{
							// For each entry in this sequence:
							if (obs->rawMeasurements.size()>0)
							{
								ASSERT_( obs->dataIsPresent.size()==obs->rawMeasurements.size() );
								size_t nValuesPerRow = obs->dataIsPresent.size();

								// For each entry in this sequence: Compute the timestamp and save all 15 values:
								ASSERT_(obs->timestamp!=INVALID_TIMESTAMP);
								TTimeStamp	t  = obs->timestamp;

								//double 	sampleTime = timeDifference( rawlog_first_timestamp ,t);
								double 	sampleTime = timestampTotime_t(t);

								// Time:
								::fprintf(f,"%f ",sampleTime);
								ASSERT_( obs->rawMeasurements.size()==obs->rawMeasurements.size() );
								for (size_t idx=0;idx<nValuesPerRow;idx++)
									::fprintf(f,"%f ",obs->dataIsPresent[idx] ? obs->rawMeasurements[idx] : 0);
								::fprintf(f,"\n");
								M++;
							} // end if
						} // end if
					} while(obs);
				}
				break;

				case CRawlog::etObservation:
				{
					CObservationPtr oo = rawlog.getAsObservation(i);

					if ( oo->GetRuntimeClass() == CLASS_ID(CObservationIMU) )
					{
						CObservationIMUPtr obs = CObservationIMUPtr(oo);
						// For each entry in this sequence:
						if (obs->rawMeasurements.size()>0)
						{
							ASSERT_( obs->dataIsPresent.size()==obs->rawMeasurements.size() );
							size_t nValuesPerRow = obs->dataIsPresent.size();

							// For each entry in this sequence: Compute the timestamp and save all 15 values:
							ASSERT_(obs->timestamp!=INVALID_TIMESTAMP);
							TTimeStamp	t  = obs->timestamp;

							double 	sampleTime = timestampTotime_t(t); //timeDifference(rawlog_first_timestamp,t);

							// Time:
							::fprintf(f,"%f ",sampleTime);
							ASSERT_( obs->rawMeasurements.size()==obs->rawMeasurements.size() );
							for (size_t idx=0;idx<nValuesPerRow;idx++)
								::fprintf(f,"%f ",obs->dataIsPresent[idx] ? obs->rawMeasurements[idx] : 0);
							::fprintf(f,"\n");
							M++;
						} // end if
					}
				}
				break;

			} // end switch.

		}

		os::fclose(f);

		wxMessageBox(_U( format("%u IMU data entries saved!",M).c_str() ),_("Done"),wxOK,this);
	}

	WX_END_TRY
}

void xRawLogViewerFrame::OnGenerateTextFileRangeBearing(wxCommandEvent& event)
{
	WX_START_TRY

	wxString caption = wxT("Save as...");
	wxString wildcard = wxT("Text files (*.txt)|*.txt|All files (*.*)|*.*");
	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
	wxString defaultFilename = _U( (loadedFileName+string("_RANGE_BEARING.txt")).c_str() );
	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

	if ( dialog.ShowModal() == wxID_OK )
	{
		wxString fileName = dialog.GetPath();
		string        fil( fileName.mbc_str() );

		int             i, M = 0,  n = (int)rawlog.size();
		FILE            *f = os::fopen( fil.c_str(), "wt");
		if (!f)
			THROW_EXCEPTION("Cannot open output file for write.");

		for (i=0;i<n;i++)
		{
			if ( rawlog.getType(i)==CRawlog::etSensoryFrame )
			{
				CSensoryFramePtr sf = rawlog.getAsObservations(i);
				CObservationBearingRangePtr obs = sf->getObservationByClass<CObservationBearingRange>();
				if (obs)
				{
					// For each entry in this sequence:
					for (size_t q=0;q<obs->sensedData.size();q++)
					{
						M++;
						::fprintf(f,"%u %i %f %f %f\n",
								i,
								obs->sensedData[q].landmarkID,
								obs->sensedData[q].range,
								obs->sensedData[q].yaw,
								obs->sensedData[q].pitch);
					}
				}
			}
			else if ( rawlog.getType(i)==CRawlog::etObservation )
			{
				CObservationPtr o = rawlog.getAsObservation(i);

				if (IS_CLASS(o,CObservationBearingRange))
				{
					CObservationBearingRangePtr obs = CObservationBearingRangePtr(o);
					// For each entry in this sequence:
					for (size_t q=0;q<obs->sensedData.size();q++)
					{
						M++;
						::fprintf(f,"%u %i %f %f %f\n",
								i,
								obs->sensedData[q].landmarkID,
								obs->sensedData[q].range,
								obs->sensedData[q].yaw,
								obs->sensedData[q].pitch);
					}
				}
			}
		}

		os::fclose(f);

		wxMessageBox(_U( format("%u bearing-range data entries saved!",M).c_str() ),_("Done"),wxOK,this);
	}

	WX_END_TRY
}


void xRawLogViewerFrame::OnShowAnimateScans(wxCommandEvent& event)
{
	CScanAnimation	scanAnimation(this);
	scanAnimation.Maximize();

	scanAnimation.ShowModal();
}



void xRawLogViewerFrame::OnMenuShowTips(wxCommandEvent& event)
{
    showNextTip(true);
}


void xRawLogViewerFrame::OnbtnEditCommentsClick(wxCommandEvent& event)
{

}

void xRawLogViewerFrame::OnMenuInsertComment(wxCommandEvent& event)
{

}

// Asks for a sensor label:
vector_string xRawLogViewerFrame::AskForObservationByLabelMultiple(const std::string & title)
{
	vector_string	labels;

	if (listOfSensorLabels.empty())
	{
	    wxMessageBox(_("No sensors were found with proper sensor labels. Labels are required for this operation."));
	    return vector_string();
	}

    // List of labels:
	wxArrayString lstLabels;
	vector_string lstLabelsStd;
    for (std::map<std::string,TInfoPerSensorLabel>::iterator i=listOfSensorLabels.begin();i!=listOfSensorLabels.end();++i)
    {
    	lstLabelsStd.push_back( i->first );
        lstLabels.Add( _U( i->first.c_str() ) );
    }

	wxArrayInt sels;

#if (wxMAJOR_VERSION>=3) || ((wxMAJOR_VERSION==2) && (wxMINOR_VERSION>=9))
	wxGetSelectedChoices(
		sels,
		_U(title.c_str()),
		_("Sensor Labels"),
		lstLabels,
		this );
#else
	wxGetMultipleChoices(
		sels,
		_U(title.c_str()),
		_("Sensor Labels"),
		lstLabels,
		this );
#endif

	labels.resize( sels.Count() );
	for (size_t i=0;i<labels.size();i++)
		labels[i] = lstLabelsStd[ sels[i] ];

	return labels;
}



// Asks for a sensor label:
std::string xRawLogViewerFrame::AskForObservationByLabel(const std::string & title)
{
	if (listOfSensorLabels.empty())
	{
	    wxMessageBox(_("No sensors were found with proper sensor labels. Labels are required for this operation."));
	    return string();
	}

    // List of labels:
	wxArrayString lstLabels;
    for (std::map<std::string,TInfoPerSensorLabel>::iterator i=listOfSensorLabels.begin();i!=listOfSensorLabels.end();++i)
        lstLabels.Add( _U( i->first.c_str() ) );

	wxString ret = wxGetSingleChoice(
		_U(title.c_str()),
		_("Sensor Labels"),
		lstLabels,
		this );
	if (ret.IsEmpty()) return string();

	return string(ret.mb_str());
}

// Changes the label of a sensor:
void xRawLogViewerFrame::OnMenuRenameSensor(wxCommandEvent& event)
{
	WX_START_TRY

	const string  the_label = AskForObservationByLabel("Choose the label of the sensor to change:");

	wxString new_label = wxGetTextFromUser(
		_("Enter the new sensor label"),
		_("New label:"),
		_U(the_label.c_str()), this );
	if (new_label.IsEmpty()) return;

	const string  the_new_label = string(new_label.mb_str());

	if (the_new_label==the_label) return;

	size_t i,n = rawlog.size();
	unsigned int nChanges = 0;

    for (i=0;i<n;i++)
    {
        switch ( rawlog.getType(i) )
        {
        case CRawlog::etSensoryFrame:
            {
                CSensoryFramePtr sf = rawlog.getAsObservations(i);
				CObservationPtr o;
				while ( (o=sf->getObservationBySensorLabel(the_label,0)).present() )
				{
					o->sensorLabel = the_new_label;
					nChanges++;
				}
            }
            break;

        case CRawlog::etObservation:
            {
                CObservationPtr o = rawlog.getAsObservation(i);

				if (o->sensorLabel==the_label)
				{
					o->sensorLabel = the_new_label;
					nChanges++;
				}
            }
            break;

            default:
                break;
        } // end switch type

    } // end for

	wxMessageBox( wxString::Format(_("%u changes"),nChanges), _("Done"),wxOK, this );

	// Update the views:
	rebuildTreeView();

	WX_END_TRY
}

void xRawLogViewerFrame::OnMenuChangePosesBatch(wxCommandEvent& event)
{
	CFormBatchSensorPose	dialog(this);
	if (dialog.ShowModal())
	{
		WX_START_TRY

		wxBusyCursor	busy;

		// Load the "ini-file" from the text control:
		CConfigFileMemory	cfg( CStringList( string( dialog.edText->GetValue().mb_str() ) ) );

		// make a list  "sensor_label -> sensor_pose" by parsing the ini-file:
		std::map<std::string,mrpt::poses::CPose3D>	desiredSensorPoses;
		std::map<std::string, mrpt::slam::CObservationImage > desiredCamParams;

		vector_string	sections;
		cfg.getAllSections( sections );

		for (vector_string::iterator it=sections.begin();it!=sections.end();++it)
		{
			if (it->empty()) continue;

			// Get sensor label:
			string label = cfg.read_string(*it,"sensorLabel","");
			if (label.empty()) continue;

			CPose3D  the_pose(
				cfg.read_double(*it,"pose_x",0,true),
				cfg.read_double(*it,"pose_y",0,true),
				cfg.read_double(*it,"pose_z",0,true),
				DEG2RAD( cfg.read_double(*it,"pose_yaw",0 ) ),
				DEG2RAD( cfg.read_double(*it,"pose_pitch",0 ) ),
				DEG2RAD( cfg.read_double(*it,"pose_roll",0 ) ) );

			// insert:
			desiredSensorPoses[label] = the_pose;

			// Camera params?
			vector_double calib, distort;
			cfg.read_vector(*it,"calib_params",vector_double(),calib);
			cfg.read_vector(*it,"distort_params",vector_double(),distort);

			if (calib.empty() || distort.empty()) continue;

			ASSERT_(calib.size()==4);
			ASSERT_(distort.size()==4);

			CMatrixDouble33 &I = desiredCamParams[label].cameraParams.intrinsicParams;
			I.zeros(3,3);
			I(2,2)=1;
			I(0,0)=calib[0];
			I(1,1)=calib[1];
			I(0,2)=calib[2];
			I(1,2)=calib[3];

			desiredCamParams[label].cameraParams.setDistortionParamsVector(distort);
		} // end for sections

		if (desiredSensorPoses.empty())
		{
			wxMessageBox(_("No valid 'sensorLabel' entry was found in the text"),_("Error"),wxOK,this);
			return;
		}

		// now apply the changes:
		wxProgressDialog    progDia(
			wxT("Modifying rawlog"),
			wxT("Processing..."),
			rawlog.size(), // range
			this, // parent
			wxPD_CAN_ABORT |
			wxPD_APP_MODAL |
			wxPD_SMOOTH |
			wxPD_AUTO_HIDE |
			wxPD_ELAPSED_TIME |
			wxPD_ESTIMATED_TIME |
			wxPD_REMAINING_TIME);

		wxTheApp->Yield();  // Let the app. process messages


		size_t changes = 0;
		int    countLoop = 0;
		bool   keepDoing = true;
		for (CRawlog::iterator it = rawlog.begin(); it!=rawlog.end() && keepDoing; it++,countLoop++)
		{
			CObservationPtr obs;

			// Go thru all the obs. of a SF, or just the obs. for non-SF rawlogs:
			size_t idx_in_sf = 0;
			bool  isSingleObs;
			while (true)
			{
				isSingleObs = false;

				if (it.getType()==CRawlog::etObservation)
				{
					obs = CObservationPtr(*it);
					isSingleObs = true;
				}
				else if (it.getType()==CRawlog::etSensoryFrame)
				{
					CSensoryFramePtr sf = CSensoryFramePtr(*it);

					if ( idx_in_sf>= sf->size()) break;
					obs = sf->getObservationByIndex(idx_in_sf++);
				}
				else break;

				if (obs)
				{
					// Check the sensor label:
					std::map<std::string,mrpt::poses::CPose3D>::iterator i = desiredSensorPoses.find(obs->sensorLabel);
					if (i!=desiredSensorPoses.end())
					{
						obs->setSensorPose( i->second );
						changes ++;
					}

					// Check for camera params:
					std::map<std::string, mrpt::slam::CObservationImage >::iterator c= desiredCamParams.find(obs->sensorLabel);
					if (c!=desiredCamParams.end())
					{
						if (!IS_CLASS(obs,CObservationImage))
							THROW_EXCEPTION_CUSTOM_MSG1("Camera parameters found for non-image observation class: %s",obs->sensorLabel.c_str());

						CObservationImagePtr img = CObservationImagePtr(obs);
						img->cameraParams = c->second.cameraParams;
					}

					if (isSingleObs) break;
				}
				else break;
			}

			if (countLoop++ % 100 == 0)
			{
				if (!progDia.Update( countLoop, wxString::Format(wxT("Processing... (%i objects processed)"),countLoop) ))
					keepDoing = false;
				wxTheApp->Yield();  // Let the app. process messages
			}
		}

		progDia.Update( rawlog.size() );	// Close dialog.

		wxMessageBox( wxString::Format(_("%i entries modified for %i sensor labels."), (int)changes, (int)desiredSensorPoses.size()), _("Done"), wxOK, this );

		WX_END_TRY
	}
}


void doFilterInvalidRange(CObservationPtr &obs, size_t &invalidRanges)
{
	if (obs->GetRuntimeClass()->derivedFrom( CLASS_ID( CObservation2DRangeScan ) ))
	{
		CObservation2DRangeScanPtr obsScan = CObservation2DRangeScanPtr(obs);
		for (size_t k=0;k<obsScan->scan.size();k++)
			if (obsScan->scan[k]>=obsScan->maxRange)
			{
				obsScan->validRange[k] = false;
				invalidRanges++;
			}
	}
}

void xRawLogViewerFrame::OnMenuMarkLaserScanInvalid(wxCommandEvent& event)
{
	WX_START_TRY

	wxBusyCursor        waitCursor;
	int					nEntries = (int)rawlog.size();

	wxString            auxStr;
	wxProgressDialog    progDia(
		wxT("Progress"),
		wxT("Parsing rawlog..."),
		nEntries, // range
		this, // parent
		wxPD_CAN_ABORT |
		wxPD_APP_MODAL |
		wxPD_SMOOTH |
		wxPD_AUTO_HIDE |
		wxPD_ELAPSED_TIME |
		wxPD_ESTIMATED_TIME |
		wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages

	size_t 	invalidRanges=0;

	for (int countLoop=0;countLoop<nEntries;countLoop++)
	{
		if (countLoop % 50 == 0)
		{
			auxStr.sprintf(wxT("Parsing rawlog... %u objects"),countLoop );
			if (!progDia.Update( countLoop, auxStr ))
				break;
			wxTheApp->Yield();  // Let the app. process messages
		}

		// Check type:
		switch (rawlog.getType(countLoop))
		{
		case CRawlog::etSensoryFrame:
			{
				// This is a SF:
				CSensoryFramePtr SF = rawlog.getAsObservations(countLoop);

				for (size_t j=0;j<SF->size();j++)
				{
					CObservationPtr obs = SF->getObservationByIndex(j);
					doFilterInvalidRange(obs, invalidRanges);
				} // end for each Observation in the SF

			} break;

		case CRawlog::etObservation:
			{
				// This is a SF:
				CObservationPtr obs = rawlog.getAsObservation(countLoop);
				doFilterInvalidRange(obs, invalidRanges);
			} break;

			default:
				break;
		}; // end switch

	} // end while keep loading

	progDia.Update( nEntries );

	wxMessageBox(_U( format("Number of invalid ranges marked: %i",(int)invalidRanges).c_str() ),_("Done"),wxOK,this);

	WX_END_TRY
}

void xRawLogViewerFrame::OnMenuChangeMaxRangeLaser(wxCommandEvent& event)
{
	WX_START_TRY

	std::string lab = AskForObservationByLabel("Select the laser sensor");
	if (lab.empty()) return;

	wxString strMaxR = wxGetTextFromUser(
						   _("Enter the new maximum range (in meters):"),
						   _("Maximum range:"),
						   _("81.0"));
	double	maxR;
	strMaxR.ToDouble( &maxR );

	wxBusyCursor        waitCursor;
	int					nEntries = (int)rawlog.size();

	wxString            auxStr;
	wxProgressDialog    progDia(
		wxT("Progress"),
		wxT("Parsing rawlog..."),
		nEntries, // range
		this, // parent
		wxPD_CAN_ABORT |
		wxPD_APP_MODAL |
		wxPD_SMOOTH |
		wxPD_AUTO_HIDE |
		wxPD_ELAPSED_TIME |
		wxPD_ESTIMATED_TIME |
		wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages

	size_t 	N=0;

	for (int countLoop=0;countLoop<nEntries;countLoop++)
	{
		if (countLoop % 50 == 0)
		{
			auxStr.sprintf(wxT("Parsing rawlog... %u objects"),countLoop );
			if (!progDia.Update( countLoop, auxStr ))
				break;
			wxTheApp->Yield();  // Let the app. process messages
		}

		// Check type:
		switch (rawlog.getType(countLoop))
		{
		case CRawlog::etSensoryFrame:
			{
				// This is a SF:
				CSensoryFramePtr SF = rawlog.getAsObservations(countLoop);

				for (size_t j=0;j<SF->size();j++)
				{
					CObservationPtr obs = SF->getObservationByIndex(j);
					if (obs->sensorLabel==lab && IS_CLASS(obs,CObservation2DRangeScan))
					{
						CObservation2DRangeScanPtr o = CObservation2DRangeScanPtr(obs);
						o->maxRange = maxR;
						N++;
					}
				} // end for each Observation in the SF

			} break;

		case CRawlog::etObservation:
			{
				// This is a SF:
				CObservationPtr obs = rawlog.getAsObservation(countLoop);
				if (obs->sensorLabel==lab && IS_CLASS(obs,CObservation2DRangeScan))
				{
					CObservation2DRangeScanPtr o = CObservation2DRangeScanPtr(obs);
					o->maxRange = maxR;
					N++;
				}
			} break;

			default:
				break;
		}; // end switch

	} // end while keep loading

	progDia.Update( nEntries );

	wxMessageBox(_U( format("Number of changes: %i",(int)N).c_str() ),_("Done"),wxOK,this);

	WX_END_TRY
}

void xRawLogViewerFrame::OnbtnEditCommentsClick1(wxCommandEvent& event)
{
	CIniEditor	dlg(this);

	string s;
	rawlog.getCommentText(s);
	dlg.edText->SetValue(_U(s.c_str()));

	if (dlg.ShowModal() )
	{
		rawlog.setCommentText( string( dlg.edText->GetValue().mb_str() ) );
		rebuildTreeView();
	}
}


void xRawLogViewerFrame::OnMenuRevert(wxCommandEvent& event)
{
	if ( mrpt::system::fileExists( loadedFileName ) )
		loadRawlogFile( loadedFileName );
}

void xRawLogViewerFrame::OnMenuBatchLaserExclusionZones(wxCommandEvent& event)
{
	CFormBatchSensorPose	dialog(this);

	if (dialog.ShowModal())
	{
		WX_START_TRY

		wxBusyCursor	busy;

		// Load the "ini-file" from the text control:
		CConfigFileMemory	cfg( CStringList( string( dialog.edText->GetValue().mb_str() ) ) );

		// make a list  "sensor_label -> list of exclusion polygons" by parsing the ini-file:
		typedef map<string, vector<CPolygon> > TPolygonList;
		TPolygonList	lstExclusions;

		vector_string	sections;
		cfg.getAllSections( sections );

		unsigned int nExclZones = 0;

		for (vector_string::iterator it=sections.begin();it!=sections.end();++it)
		{
			if (it->empty()) continue;

			// Get sensor label:
			string label = cfg.read_string(*it,"sensorLabel","");
			if (label.empty()) continue;

			unsigned int N = 1;

			for(;;)
			{
				vector<double> x,y;
				cfg.read_vector( *it, format("exclusionZone%u_x",N), vector<double>(0), x);
				cfg.read_vector( *it, format("exclusionZone%u_y",N++), vector<double>(0), y);

				if (!x.empty() && !y.empty())
				{
					ASSERT_(x.size()==y.size())

					mrpt::math::CPolygon	p;
					p.setAllVertices(x,y);
					lstExclusions[label].push_back(p);
					nExclZones++;
				}
				else break;
			}

		} // end for sections

		if (lstExclusions.empty())
		{
			wxMessageBox(_("No valid exclusion zones found in the text"),_("Error"),wxOK,this);
			return;
		}

		// now apply the changes:
		wxProgressDialog    progDia(
			wxT("Modifying rawlog"),
			wxT("Processing..."),
			rawlog.size(), // range
			this, // parent
			wxPD_CAN_ABORT |
			wxPD_APP_MODAL |
			wxPD_SMOOTH |
			wxPD_AUTO_HIDE |
			wxPD_ELAPSED_TIME |
			wxPD_ESTIMATED_TIME |
			wxPD_REMAINING_TIME);

		wxTheApp->Yield();  // Let the app. process messages


		size_t changes = 0;
		int    countLoop = 0;
		bool   keepDoing = true;
		for (CRawlog::iterator it = rawlog.begin(); it!=rawlog.end() && keepDoing; it++,countLoop++)
		{
			CObservation2DRangeScanPtr obs;

			// Go thru all the obs. of a SF, or just the obs. for non-SF rawlogs:
			size_t idx_in_sf = 0;
			bool  isSingleObs;
			while (true)
			{
				isSingleObs = false;

				if (it.getType()==CRawlog::etObservation)
				{
					if (IS_CLASS(*it,CObservation2DRangeScan))
					{
						obs = CObservation2DRangeScanPtr(*it);
						isSingleObs = true;
					}
					else break; // not a laser scan
				}
				else if (it.getType()==CRawlog::etSensoryFrame)
				{
					CSensoryFramePtr sf = CSensoryFramePtr(*it);

					obs = sf->getObservationByClass<CObservation2DRangeScan>(idx_in_sf++);
					if (!obs)
					{
						idx_in_sf = 0;
						break;
					}
				}
				else break;

				if (obs)
				{
					// Check the sensor label:
					TPolygonList::iterator i = lstExclusions.find(obs->sensorLabel);
					if (i!=lstExclusions.end())
					{
						obs->filterByExclusionAreas(i->second);
						changes++;
					}
					if (isSingleObs) break;
				}
				else break;
			}

			if (countLoop++ % 100 == 0)
			{
				if (!progDia.Update( countLoop, wxString::Format(wxT("Processing... (%i objects processed)"),countLoop) ))
					keepDoing = false;
				wxTheApp->Yield();  // Let the app. process messages
			}
		}

		progDia.Update( rawlog.size() );	// Close dialog.

		wxMessageBox( wxString::Format(_("%i entries modified for %i sensor labels and %u exclusion areas."),
			(int)changes, (int)lstExclusions.size(), nExclZones ), _("Done"), wxOK, this );

		WX_END_TRY
	}
}



void xRawLogViewerFrame::OnComboImageDirsChange(wxCommandEvent& event)
{
	wxString dir = toolbarcomboImages->GetStringSelection();

	string 	 dirc = string( dir.mb_str() );

	if ( mrpt::system::fileExists(dirc) )
	{
		CImage::IMAGES_PATH_BASE = dirc;
		//wxMessageBox( _("The current directory for external images has been set to:\n")+dir , _("External images"));

		tree_view->SetSelectedItem( tree_view->GetSelectedItem(), true );
	}
	else
	{
		wxMessageBox( _("The directory:\n")+dir+_("does not exist.") , _("External images"));
	}
}


void xRawLogViewerFrame::OnLaserFilterAngles(wxCommandEvent& event)
{
	CFormBatchSensorPose	dialog(this);

	if (dialog.ShowModal())
	{
		WX_START_TRY

		wxBusyCursor	busy;

		// Load the "ini-file" from the text control:
		CConfigFileMemory	cfg( CStringList( string( dialog.edText->GetValue().mb_str() ) ) );

		// make a list  "sensor_label -> list of exclusion polygons" by parsing the ini-file:
		typedef map<string, vector<pair<double,double> > > TExclAreasList;
		TExclAreasList  lstExclusions;

		vector_string	sections;
		cfg.getAllSections( sections );

		unsigned int nExclZones = 0;

		for (vector_string::iterator it=sections.begin();it!=sections.end();++it)
		{
			if (it->empty()) continue;

			// Get sensor label:
			string label = cfg.read_string(*it,"sensorLabel","");
			if (label.empty()) continue;

			// Load forbiden angles;
			unsigned int N = 1;
			for(;;)
			{
				const double ini = DEG2RAD( cfg.read_double( *it, format("exclusionAngles%u_ini",N), -1000 ) );
				const double end = DEG2RAD( cfg.read_double( *it, format("exclusionAngles%u_end",N++), -1000 ) );

				if (ini>-M_PI && end>-M_PI)
				{
					 lstExclusions[label].push_back(make_pair(ini,end));
					 nExclZones++;
				}
				else break;
			}
		} // end for sections

		if (lstExclusions.empty())
		{
			wxMessageBox(_("No valid exclusion zones found in the text"),_("Error"),wxOK,this);
			return;
		}

		// now apply the changes:
		wxProgressDialog    progDia(
			wxT("Modifying rawlog"),
			wxT("Processing..."),
			rawlog.size(), // range
			this, // parent
			wxPD_CAN_ABORT |
			wxPD_APP_MODAL |
			wxPD_SMOOTH |
			wxPD_AUTO_HIDE |
			wxPD_ELAPSED_TIME |
			wxPD_ESTIMATED_TIME |
			wxPD_REMAINING_TIME);

		wxTheApp->Yield();  // Let the app. process messages


		size_t changes = 0;
		int    countLoop = 0;
		bool   keepDoing = true;
		for (CRawlog::iterator it = rawlog.begin(); it!=rawlog.end() && keepDoing; it++,countLoop++)
		{
			CObservation2DRangeScanPtr obs;

			// Go thru all the obs. of a SF, or just the obs. for non-SF rawlogs:
			size_t idx_in_sf = 0;
			bool  isSingleObs;
			while (true)
			{
				isSingleObs = false;

				if (it.getType()==CRawlog::etObservation)
				{
					if (IS_CLASS(*it,CObservation2DRangeScan))
					{
						obs = CObservation2DRangeScanPtr(*it);
						isSingleObs = true;
					}
					else break; // not a laser scan
				}
				else if (it.getType()==CRawlog::etSensoryFrame)
				{
					CSensoryFramePtr sf = CSensoryFramePtr(*it);

					obs = sf->getObservationByClass<CObservation2DRangeScan>(idx_in_sf++);
					if (!obs)
					{
						idx_in_sf = 0;
						break;
					}
				}
				else break;

				if (obs)
				{
					// Check the sensor label:
					TExclAreasList::iterator i = lstExclusions.find(obs->sensorLabel);
					if (i!=lstExclusions.end())
					{
						obs->filterByExclusionAngles(i->second);
						changes++;
					}
					if (isSingleObs) break;
				}
				else break;
			}

			if (countLoop++ % 100 == 0)
			{
				if (!progDia.Update( countLoop, wxString::Format(wxT("Processing... (%i objects processed)"),countLoop) ))
					keepDoing = false;
				wxTheApp->Yield();  // Let the app. process messages
			}
		}

		progDia.Update( rawlog.size() );	// Close dialog.

		wxMessageBox( wxString::Format(_("%i entries modified for %i sensor labels and %u exclusion areas."),
			(int)changes, (int)lstExclusions.size(), nExclZones ), _("Done"), wxOK, this );

		WX_END_TRY
	}
}

void xRawLogViewerFrame::OnMenuRangeBearFilterIDs(wxCommandEvent& event)
{
	WX_START_TRY

	wxString strID = wxGetTextFromUser(
						   _("Enter the landmark ID to remove:"),
						   _("Landmark ID"),
						   _("0"));
	long ID;
	strID.ToLong( &ID );

	wxBusyCursor        waitCursor;
	int					nEntries = (int)rawlog.size();

	wxString            auxStr;
	wxProgressDialog    progDia(
		wxT("Progress"),
		wxT("Parsing rawlog..."),
		nEntries, // range
		this, // parent
		wxPD_CAN_ABORT |
		wxPD_APP_MODAL |
		wxPD_SMOOTH |
		wxPD_AUTO_HIDE |
		wxPD_ELAPSED_TIME |
		wxPD_ESTIMATED_TIME |
		wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages

	size_t 	N=0;

	for (int countLoop=0;countLoop<nEntries;countLoop++)
	{
		if (countLoop % 50 == 0)
		{
			auxStr.sprintf(wxT("Parsing rawlog... %u objects"),countLoop );
			if (!progDia.Update( countLoop, auxStr ))
				break;
			wxTheApp->Yield();  // Let the app. process messages
		}

		// Check type:
		switch (rawlog.getType(countLoop))
		{
		case CRawlog::etSensoryFrame:
			{
				// This is a SF:
				CSensoryFramePtr SF = rawlog.getAsObservations(countLoop);

				for (size_t j=0;j<SF->size();j++)
				{
					CObservationPtr obs = SF->getObservationByIndex(j);
					if (IS_CLASS(obs,CObservationBearingRange))
					{
						CObservationBearingRangePtr o = CObservationBearingRangePtr(obs);
						for (size_t k=0;k<o->sensedData.size();k++)
						{
							if (ID==o->sensedData[k].landmarkID)
							{
								o->sensedData.erase( o->sensedData.begin()+k );
								N++;
							}
						}
					}
				} // end for each Observation in the SF

			} break;

		case CRawlog::etObservation:
			{
				// This is a SF:
				CObservationPtr obs = rawlog.getAsObservation(countLoop);
				if (IS_CLASS(obs,CObservationBearingRange))
				{
					CObservationBearingRangePtr o = CObservationBearingRangePtr(obs);
					for (size_t k=0;k<o->sensedData.size();k++)
					{
						if (ID==o->sensedData[k].landmarkID)
						{
							o->sensedData.erase( o->sensedData.begin()+k );
							N++;
						}
					}
				}
			} break;

			default:
				break;
		}; // end switch

	} // end while keep loading

	progDia.Update( nEntries );

	wxMessageBox(_U( format("Number of changes: %i",(int)N).c_str() ),_("Done"),wxOK,this);

	WX_END_TRY
}

void xRawLogViewerFrame::OnMenuRegenerateTimestampBySF(wxCommandEvent& event)
{
	WX_START_TRY

	wxBusyCursor        waitCursor;
	const size_t nEntries = rawlog.size();

	wxString            auxStr;
	wxProgressDialog    progDia(
		wxT("Progress"),
		wxT("Parsing rawlog..."),
		nEntries, // range
		this, // parent
		wxPD_CAN_ABORT |
		wxPD_APP_MODAL |
		wxPD_SMOOTH |
		wxPD_AUTO_HIDE |
		wxPD_ELAPSED_TIME |
		wxPD_ESTIMATED_TIME |
		wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages

	size_t 	N=0;

	for (size_t countLoop=0;countLoop<nEntries;countLoop++)
	{
		if (countLoop % 50 == 0)
		{
			auxStr.sprintf(wxT("Parsing rawlog... %u objects"),countLoop );
			if (!progDia.Update( countLoop, auxStr ))
				break;
			wxTheApp->Yield();  // Let the app. process messages
		}

		// Check type:
		switch (rawlog.getType(countLoop))
		{
		case CRawlog::etSensoryFrame:
			{
				// This is a SF:
				CSensoryFramePtr SF = rawlog.getAsObservations(countLoop);

				// Take a timestamp for this SF:
				TTimeStamp tim = INVALID_TIMESTAMP;
				for (size_t j=0;j<SF->size();j++)
				{
					CObservationPtr obs = SF->getObservationByIndex(j);
					if (obs->timestamp!=INVALID_TIMESTAMP)
					{
						tim=obs->timestamp;
						break;
					}
				} // end for each Observation in the SF
				if (tim != INVALID_TIMESTAMP)
				{
					for (size_t j=0;j<SF->size();j++)
					{
						CObservationPtr obs = SF->getObservationByIndex(j);
						if (obs->timestamp==INVALID_TIMESTAMP)
						{
							obs->timestamp = tim;
							N++;
						}
					} // end for each Observation in the SF
				}

			} break;

		default:
				break;
		}; // end switch

	} // end while keep loading

	progDia.Update( nEntries );

	wxMessageBox(_U( format("Number of changes: %i",(int)N).c_str() ),_("Done"),wxOK,this);

	WX_END_TRY
}

// Used below. Must be at global scope for usage within STL.
struct TImageToSaveData
{
	TImageToSaveData() : img(NULL) { }
	TImageToSaveData(mrpt::utils::CImage *_img,const char* str) : img(_img),channel_desc(str) { }
	mrpt::utils::CImage *img;
	std::string          channel_desc; // LEFT, RIGHT, etc...
};

bool operator <(const TImageToSaveData&a, const TImageToSaveData&b)
{
	return a.channel_desc < b.channel_desc;
}

void xRawLogViewerFrame::OnmnuCreateAVISelected(wxCommandEvent& event)
{
	WX_START_TRY

	std::string  senLabel = AskForObservationByLabel("Select the camera:");
	if (senLabel.empty()) return;

	wxString caption = wxT("Save AVI video...");
	wxString wildcard = wxT("AVI files (*.avi)|*.avi|All files (*.*)|*.*");
	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );
	wxString defaultFilename = _U( (loadedFileName+format("_%s.avi",senLabel.c_str())).c_str() );
	wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

	if ( dialog.ShowModal() != wxID_OK )
		return;

	const string outAviFilename = string( dialog.GetPath().mb_str() );

	float FPS = 20.0; // For the AVI

	wxBusyCursor        waitCursor;
	int				nEntries = (int)rawlog.size();

	wxString            auxStr;
	wxProgressDialog    progDia(
		wxT("Progress"),
		wxT("Parsing rawlog..."),
		nEntries, // range
		this, // parent
		wxPD_CAN_ABORT |
		wxPD_APP_MODAL |
		wxPD_SMOOTH |
		wxPD_AUTO_HIDE |
		wxPD_ELAPSED_TIME |
		wxPD_ESTIMATED_TIME |
		wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages


	//  possible <channel_desc>: left, right, disparity (more in the future?)
	std::vector<std::string>  		outVideosIdx;
	mrpt::vision::CVideoFileWriter  outVideos[20];

	int    nFrames  = 0;
	string errorMsg;

	for (int countLoop=0;countLoop<nEntries;countLoop++)
	{
		if (countLoop % 5 == 0)
		{
			auxStr.sprintf(wxT("Processing rawlog... %u image frames"),countLoop );
			if (!progDia.Update( countLoop, auxStr ))
				break;
			progDia.Fit();
			wxTheApp->Yield();  // Let the app. process messages
		}

		try
		{
			std::set<TImageToSaveData>  imgsForVideo;


			if ( rawlog.getType(countLoop) == CRawlog::etSensoryFrame )
			{
				CSensoryFramePtr SF = rawlog.getAsObservations(countLoop);

				CObservationImagePtr obsImg = SF->getObservationByClass<CObservationImage>();
				if (obsImg)
				{
					imgsForVideo.insert( TImageToSaveData(&obsImg->image, "IMAGE") );
				}
				else
				{
					CObservationStereoImagesPtr obsStereoImg = SF->getObservationByClass<CObservationStereoImages>();
					if (obsStereoImg)
					{
						imgsForVideo.insert( TImageToSaveData(&obsStereoImg->imageLeft, "LEFT") );
						if (obsStereoImg->hasImageRight) imgsForVideo.insert( TImageToSaveData(&obsStereoImg->imageRight, "RIGHT") );
						if (obsStereoImg->hasImageDisparity) imgsForVideo.insert( TImageToSaveData(&obsStereoImg->imageDisparity, "DISP") );
					}
					else
					{
						CObservation3DRangeScanPtr obs3D = SF->getObservationByClass<CObservation3DRangeScan>();
						if ( obs3D )
						{
							imgsForVideo.insert( TImageToSaveData(&obs3D->intensityImage, "INTENSITY") );
						}
					}
				}
			} // end for each entry
			else if ( rawlog.getType(countLoop) == CRawlog::etObservation )
			{
				CObservationPtr o = rawlog.getAsObservation(countLoop);
				if (IS_CLASS(o,CObservationImage))
				{
					CObservationImagePtr obsImg = CObservationImagePtr(o);
					imgsForVideo.insert( TImageToSaveData(&obsImg->image, "IMAGE") );
				}
				else if (IS_CLASS(o,CObservationStereoImages))
				{
					CObservationStereoImagesPtr obsStereoImg = CObservationStereoImagesPtr(o);
					imgsForVideo.insert( TImageToSaveData(&obsStereoImg->imageLeft, "LEFT") );
					if (obsStereoImg->hasImageRight) imgsForVideo.insert( TImageToSaveData(&obsStereoImg->imageRight, "RIGHT") );
					if (obsStereoImg->hasImageDisparity) imgsForVideo.insert( TImageToSaveData(&obsStereoImg->imageDisparity, "DISP") );
				}
				else if (IS_CLASS( o, CObservation3DRangeScan ))
				{
					CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(o);
					imgsForVideo.insert( TImageToSaveData(&obs3D->intensityImage, "INTENSITY") );
				}
			} // end for each entry

			// If we have images in "imgsForVideo", save them to their video (AVI) files,
			//  and create them upon first usage:
			for (std::set<TImageToSaveData>::const_iterator itIm=imgsForVideo.begin();itIm!=imgsForVideo.end();++itIm)
			{
				const TImageToSaveData &d = *itIm;

				size_t idx = mrpt::utils::find_in_vector(d.channel_desc,outVideosIdx);
				if (string::npos == idx )  // new?
				{
					idx = outVideosIdx.size();
					outVideosIdx.push_back(d.channel_desc);
				}

				// The video writter for this channel:
				mrpt::vision::CVideoFileWriter &vid = outVideos[idx];

				if (!vid.isOpen())	// Open file upon first usage:
				{
					const string filname =
						mrpt::system::extractFileDirectory(outAviFilename) + string("/") +
						mrpt::system::extractFileName(outAviFilename) + string("_") +
						d.channel_desc + string(".avi");

					if (!vid.open(filname,FPS,TImageSize(d.img->getWidth(),d.img->getHeight()),
#ifdef MRPT_OS_WINDOWS
							""
#else
							"XVID"
#endif
						))
						throw std::runtime_error("Error creating the AVI video file...");
				}
				// and save video frame:
				CImage  imgAux;
				d.img->colorImage(imgAux);
				vid << imgAux;
				nFrames++;
				d.img->unload();
			} // end for each image
		}
		catch (exception &e)
		{
			errorMsg = e.what();
			break;
		}
		catch (...)
		{
			break;
		}
	} // end while keep loading

	progDia.Update( nEntries );

	if (!errorMsg.empty())
		wxMessageBox(_U(errorMsg.c_str() ), _("Error"));

	wxMessageBox(_U( format("Saved %u images.",nFrames).c_str() ),_("Done"),wxOK,this);

	WX_END_TRY
}

void xRawLogViewerFrame::OnMenuRegenerateOdometryTimes(wxCommandEvent& event)
{
	WX_START_TRY

	wxBusyCursor        waitCursor;
	int					nEntries = (int)rawlog.size();

	wxString            auxStr;
	wxProgressDialog    progDia(
		wxT("Progress"),
		wxT("Parsing rawlog..."),
		nEntries, // range
		this, // parent
		wxPD_CAN_ABORT |
		wxPD_APP_MODAL |
		wxPD_SMOOTH |
		wxPD_AUTO_HIDE |
		wxPD_ELAPSED_TIME |
		wxPD_ESTIMATED_TIME |
		wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages

	int			nChanges = 0;
	string		errorMsg;

	mrpt::system::TTimeStamp   lastObsTime = INVALID_TIMESTAMP;
	CActionRobotMovement2DPtr  lastOdo;


	for (int countLoop=0;countLoop<nEntries;countLoop++)
	{
		if (countLoop % 20 == 0)
		{
			auxStr.sprintf(wxT("Parsing rawlog... %u objects"),countLoop );
			if (!progDia.Update( countLoop, auxStr ))
				break;
			wxTheApp->Yield();  // Let the app. process messages
		}

		try
		{
			if (rawlog.getType(countLoop) == CRawlog::etActionCollection )
			{
				CActionCollectionPtr acts = rawlog.getAsAction( countLoop );
				for (size_t k=0;k<acts->size();k++)
				{
					CActionPtr act = acts->get(k);
					if (act->GetRuntimeClass() == CLASS_ID( CActionRobotMovement2D ) )
						lastOdo = CActionRobotMovement2DPtr( act );
				}
			}
			else if (rawlog.getType(countLoop) == CRawlog::etSensoryFrame)
			{
				CSensoryFramePtr sf = rawlog.getAsObservations(countLoop);
				if (sf->size()>0)
				{
					TTimeStamp  thisObsTime = (*sf->begin())->timestamp;
					if (thisObsTime!=INVALID_TIMESTAMP)
					{
						if (lastOdo && lastObsTime!=INVALID_TIMESTAMP)
						{	// Do average:
							lastOdo->timestamp = (lastObsTime>>1)+(thisObsTime>>1);
							lastOdo.clear_unique();
							nChanges++;
						}
						lastObsTime = thisObsTime;
					}
				}
			}
		}
		catch (exception &e)
		{
			errorMsg = e.what();
			break;
		}
		catch (...)
		{
			break;
		}
	} // end while keep loading

	progDia.Update( nEntries );

	wxMessageBox(_U( format("%u entries have been modified",nChanges).c_str() ),_("Done"),wxOK,this);

	WX_END_TRY
}

// Recover 3D camera params from range data:
void xRawLogViewerFrame::OnMenuItem3DObsRecoverParams(wxCommandEvent& event)
{
	WX_START_TRY

	wxBusyCursor        waitCursor;
	int					nEntries = (int)rawlog.size();

	wxString            auxStr;
	wxProgressDialog    progDia(
		wxT("Progress"),
		wxT("Parsing rawlog..."),
		nEntries, // range
		this, // parent
		wxPD_CAN_ABORT |
		wxPD_APP_MODAL |
		wxPD_SMOOTH |
		wxPD_AUTO_HIDE |
		wxPD_ELAPSED_TIME |
		wxPD_ESTIMATED_TIME |
		wxPD_REMAINING_TIME);

	wxTheApp->Yield();  // Let the app. process messages

	int			nChanges = 0;
	string		errorMsg;

	bool firstObs = true;
	TCamera optimal_params;

	for (int countLoop=0;countLoop<nEntries;countLoop++)
	{
		if (countLoop % 20 == 0)
		{
			auxStr.sprintf(wxT("Parsing rawlog... %u objects"),countLoop );
			if (!progDia.Update( countLoop, auxStr ))
				break;
			wxTheApp->Yield();  // Let the app. process messages
		}

		try
		{
			if (rawlog.getType(countLoop) == CRawlog::etObservation)
			{
				CObservationPtr obs = rawlog.getAsObservation(countLoop);
				if (IS_CLASS(obs,CObservation3DRangeScan))
				{
					CObservation3DRangeScanPtr o = CObservation3DRangeScanPtr(obs);

					if (firstObs)
					{
						const double avrErr = CObservation3DRangeScan::recoverCameraCalibrationParameters(
							*o,
							optimal_params);

						if (wxNO==wxMessageBox( _U(format("Calibration with first observation:\nAverage reprojection error=%.04fpx.\n Accept and apply to ALL 3D observations?",avrErr).c_str()),_("Warning"), wxYES_NO | wxICON_EXCLAMATION))
							break;

						firstObs = false;
					}

					// For the rest (including the first obs):
					o->cameraParams = optimal_params;
					nChanges++;
				}
			}
		}
		catch (exception &e)
		{
			errorMsg = e.what();
			break;
		}
		catch (...)
		{
			break;
		}
	} // end while keep loading

	progDia.Update( nEntries );

	wxMessageBox(_U( format("%u entries have been modified",nChanges).c_str() ),_("Done"),wxOK,this);

	WX_END_TRY

}

void xRawLogViewerFrame::Onslid3DcamConfCmdScrollChanged(wxScrollEvent& event)
{
	// Refresh:
	SelectObjectInTreeView(curSelectedObject);
}


