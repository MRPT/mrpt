/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "slamdemoApp.h"
#include "slamdemoMain.h"
#include "CAboutBox.h"
#include "CDlgParams.h"
#include "CLogView.h"
#include <wx/msgdlg.h>
#include <wx/filedlg.h>

//(*InternalHeaders(slamdemoFrame)
#include <wx/string.h>
#include <wx/intl.h>
#include <wx/font.h>
#include <wx/bitmap.h>
#include <wx/icon.h>
#include <wx/image.h>
#include <wx/artprov.h>
//*)
#include <mrpt/gui/wx28-fixes.h>

#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/system/vector_loadsave.h>
#include <mrpt/random.h>
#include <mrpt/obs/CObservationComment.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::bayes;
using namespace mrpt::utils;
using namespace mrpt::random;
using namespace mrpt::slam;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::poses;

#include "../wx-common/mrpt_logo.xpm"
#include "imgs/icono_main.xpm"
#include "imgs/icon_batch.xpm"
#include "imgs/icon_config.xpm"
#include "imgs/icon_help.xpm"
#include "imgs/icon_exit.xpm"
#include "imgs/icon_play.xpm"
#include "imgs/icon_reset.xpm"
#include "imgs/icon_step.xpm"
#include "imgs/icon_stop.xpm"

static const double RAD2DEGSQ = square(180/M_PI);


// A custom Art provider for customizing the icons:
class MyArtProvider : public wxArtProvider
{
protected:
    virtual wxBitmap CreateBitmap(const wxArtID& id,
                                  const wxArtClient& client,
                                  const wxSize& size);
};
wxBitmap MyArtProvider::CreateBitmap(const wxArtID& id,
                                     const wxArtClient& client,
                                     const wxSize& size)
{
    if (id == wxART_MAKE_ART_ID(MAIN_ICON))   return wxBitmap(icono_main_xpm);
    if (id == wxART_MAKE_ART_ID(IMG_MRPT_LOGO))  return wxBitmap(mrpt_logo_xpm);

	if (id == wxART_MAKE_ART_ID(ICON_BATCH))   return wxBitmap(icon_batch_xpm );
	if (id == wxART_MAKE_ART_ID(ICON_CONFIG))   return wxBitmap(icon_config_xpm );
	if (id == wxART_MAKE_ART_ID(ICON_HELP))   return wxBitmap(icon_help_xpm );
	if (id == wxART_MAKE_ART_ID(ICON_EXIT))   return wxBitmap(icon_exit_xpm );
	if (id == wxART_MAKE_ART_ID(ICON_PLAY))   return wxBitmap(icon_play_xpm );
	if (id == wxART_MAKE_ART_ID(ICON_RESET))   return wxBitmap(icon_reset_xpm );
	if (id == wxART_MAKE_ART_ID(ICON_STEP))   return wxBitmap(icon_step_xpm );
	if (id == wxART_MAKE_ART_ID(ICON_STOP))   return wxBitmap(icon_stop_xpm );

    // Any wxWidgets icons not implemented here
    // will be provided by the default art provider.
    return wxNullBitmap;
}





//(*IdInit(slamdemoFrame)
const long slamdemoFrame::ID_STATICTEXT1 = wxNewId();
const long slamdemoFrame::ID_PANEL3 = wxNewId();
const long slamdemoFrame::ID_CUSTOM1 = wxNewId();
const long slamdemoFrame::ID_STATICTEXT2 = wxNewId();
const long slamdemoFrame::ID_PANEL4 = wxNewId();
const long slamdemoFrame::ID_CUSTOM2 = wxNewId();
const long slamdemoFrame::ID_STATICTEXT10 = wxNewId();
const long slamdemoFrame::ID_PANEL5 = wxNewId();
const long slamdemoFrame::ID_CUSTOM3 = wxNewId();
const long slamdemoFrame::ID_STATICTEXT3 = wxNewId();
const long slamdemoFrame::ID_PANEL6 = wxNewId();
const long slamdemoFrame::ID_CUSTOM4 = wxNewId();
const long slamdemoFrame::ID_STATICTEXT4 = wxNewId();
const long slamdemoFrame::ID_PANEL7 = wxNewId();
const long slamdemoFrame::ID_GRID1 = wxNewId();
const long slamdemoFrame::ID_STATICTEXT5 = wxNewId();
const long slamdemoFrame::ID_PANEL9 = wxNewId();
const long slamdemoFrame::ID_CUSTOM7 = wxNewId();
const long slamdemoFrame::ID_STATICTEXT6 = wxNewId();
const long slamdemoFrame::ID_PANEL10 = wxNewId();
const long slamdemoFrame::ID_CUSTOM8 = wxNewId();
const long slamdemoFrame::ID_STATICTEXT7 = wxNewId();
const long slamdemoFrame::ID_PANEL11 = wxNewId();
const long slamdemoFrame::ID_CUSTOM9 = wxNewId();
const long slamdemoFrame::ID_PANEL1 = wxNewId();
const long slamdemoFrame::ID_STATICTEXT9 = wxNewId();
const long slamdemoFrame::ID_PANEL8 = wxNewId();
const long slamdemoFrame::ID_CUSTOM5 = wxNewId();
const long slamdemoFrame::ID_STATICTEXT11 = wxNewId();
const long slamdemoFrame::ID_PANEL14 = wxNewId();
const long slamdemoFrame::ID_CUSTOM6 = wxNewId();
const long slamdemoFrame::ID_STATICTEXT12 = wxNewId();
const long slamdemoFrame::ID_PANEL15 = wxNewId();
const long slamdemoFrame::ID_CUSTOM11 = wxNewId();
const long slamdemoFrame::ID_STATICTEXT13 = wxNewId();
const long slamdemoFrame::ID_PANEL16 = wxNewId();
const long slamdemoFrame::ID_CUSTOM12 = wxNewId();
const long slamdemoFrame::ID_PANEL2 = wxNewId();
const long slamdemoFrame::ID_STATICTEXT8 = wxNewId();
const long slamdemoFrame::ID_PANEL13 = wxNewId();
const long slamdemoFrame::ID_CUSTOM10 = wxNewId();
const long slamdemoFrame::ID_STATICTEXT14 = wxNewId();
const long slamdemoFrame::ID_PANEL17 = wxNewId();
const long slamdemoFrame::ID_CUSTOM13 = wxNewId();
const long slamdemoFrame::ID_PANEL12 = wxNewId();
const long slamdemoFrame::ID_NOTEBOOK1 = wxNewId();
const long slamdemoFrame::ID_MENUITEM1 = wxNewId();
const long slamdemoFrame::ID_MENUITEM2 = wxNewId();
const long slamdemoFrame::ID_MENUITEM3 = wxNewId();
const long slamdemoFrame::ID_MENUITEM6 = wxNewId();
const long slamdemoFrame::ID_MENUITEM4 = wxNewId();
const long slamdemoFrame::ID_MENUITEM5 = wxNewId();
const long slamdemoFrame::idMenuQuit = wxNewId();
const long slamdemoFrame::ID_MENUITEM8 = wxNewId();
const long slamdemoFrame::ID_MENUITEM11 = wxNewId();
const long slamdemoFrame::ID_MENUITEM_SAVE_RAWLOG = wxNewId();
const long slamdemoFrame::ID_MENUITEM9 = wxNewId();
const long slamdemoFrame::ID_MENUITEM10 = wxNewId();
const long slamdemoFrame::ID_MENUITEM7 = wxNewId();
const long slamdemoFrame::idMenuAbout = wxNewId();
const long slamdemoFrame::ID_STATUSBAR1 = wxNewId();
const long slamdemoFrame::ID_TOOLBARITEM1 = wxNewId();
const long slamdemoFrame::ID_TOOLBARITEM2 = wxNewId();
const long slamdemoFrame::ID_BTNRUN = wxNewId();
const long slamdemoFrame::ID_BTNSTOP = wxNewId();
const long slamdemoFrame::ID_TOOLBARITEM4 = wxNewId();
const long slamdemoFrame::ID_TOOLBARITEM3 = wxNewId();
const long slamdemoFrame::ID_TOOLBARITEM6 = wxNewId();
const long slamdemoFrame::ID_TOOLBARITEM7 = wxNewId();
const long slamdemoFrame::ID_TOOLBAR1 = wxNewId();
const long slamdemoFrame::ID_TIMER1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(slamdemoFrame,wxFrame)
    //(*EventTable(slamdemoFrame)
    //*)
END_EVENT_TABLE()

slamdemoFrame::slamdemoFrame(wxWindow* parent,wxWindowID id)
{
	// Load my custom icons:
#if wxCHECK_VERSION(2, 8, 0)
    wxArtProvider::Push(new MyArtProvider);
#else
    wxArtProvider::PushProvider(new MyArtProvider);
#endif


    //(*Initialize(slamdemoFrame)
    wxMenuItem* MenuItem2;
    wxGridSizer* GridSizer13;
    wxMenuItem* MenuItem1;
    wxFlexGridSizer* FlexGridSizer8;
    wxGridSizer* GridSizer1;
    wxFlexGridSizer* FlexGridSizer1;
    wxFlexGridSizer* FlexGridSizer2;
    wxMenu* Menu1;
    wxGridSizer* GridSizer4;
    wxGridSizer* GridSizer7;
    wxGridSizer* GridSizer3;
    wxGridSizer* GridSizer2;
    wxGridSizer* GridSizer6;
    wxFlexGridSizer* FlexGridSizer7;
    wxFlexGridSizer* FlexGridSizer4;
    wxFlexGridSizer* FlexGridSizer9;
    wxFlexGridSizer* FlexGridSizer6;
    wxGridSizer* GridSizer11;
    wxGridSizer* GridSizer5;
    wxFlexGridSizer* FlexGridSizer3;
    wxGridSizer* GridSizer14;
    wxGridSizer* GridSizer10;
    wxGridSizer* GridSizer8;
    wxMenuBar* MenuBar1;
    wxGridSizer* GridSizer9;
    wxMenuItem* MenuItem7;
    wxMenu* Menu2;
    wxGridSizer* GridSizer12;
    wxFlexGridSizer* FlexGridSizer5;

    Create(parent, wxID_ANY, _("2D SLAM Demo - Part of the MRPT project"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("wxID_ANY"));
    {
    wxIcon FrameIcon;
    FrameIcon.CopyFromBitmap(wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("MAIN_ICON")),wxART_FRAME_ICON));
    SetIcon(FrameIcon);
    }
    FlexGridSizer1 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer1->AddGrowableCol(0);
    FlexGridSizer1->AddGrowableRow(0);
    FlexGridSizer3 = new wxFlexGridSizer(1, 4, 0, 0);
    FlexGridSizer3->AddGrowableCol(0);
    FlexGridSizer3->AddGrowableCol(1);
    FlexGridSizer3->AddGrowableCol(2);
    FlexGridSizer3->AddGrowableCol(3);
    FlexGridSizer3->AddGrowableRow(0);
    FlexGridSizer4 = new wxFlexGridSizer(4, 1, 0, 0);
    FlexGridSizer4->AddGrowableCol(0);
    FlexGridSizer4->AddGrowableRow(1);
    FlexGridSizer4->AddGrowableRow(3);
    Panel3 = new wxPanel(this, ID_PANEL3, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL3"));
    Panel3->SetBackgroundColour(wxColour(255,255,0));
    GridSizer1 = new wxGridSizer(0, 1, 0, 0);
    lbGT = new wxStaticText(Panel3, ID_STATICTEXT1, _("Ground truth"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    wxFont lbGTFont(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    lbGT->SetFont(lbGTFont);
    GridSizer1->Add(lbGT, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel3->SetSizer(GridSizer1);
    GridSizer1->Fit(Panel3);
    GridSizer1->SetSizeHints(Panel3);
    FlexGridSizer4->Add(Panel3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    plotGT = new mpWindow(this,ID_CUSTOM1,wxDefaultPosition,wxDefaultSize,0);
    FlexGridSizer4->Add(plotGT, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 3);
    Panel4 = new wxPanel(this, ID_PANEL4, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL4"));
    Panel4->SetBackgroundColour(wxColour(255,255,0));
    GridSizer2 = new wxGridSizer(0, 1, 0, 0);
    lbObs = new wxStaticText(Panel4, ID_STATICTEXT2, _("Observation"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT2"));
    wxFont lbObsFont(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    lbObs->SetFont(lbObsFont);
    GridSizer2->Add(lbObs, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel4->SetSizer(GridSizer2);
    GridSizer2->Fit(Panel4);
    GridSizer2->SetSizeHints(Panel4);
    FlexGridSizer4->Add(Panel4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    plotObs = new mpWindow(this,ID_CUSTOM2,wxDefaultPosition,wxDefaultSize,0);
    FlexGridSizer4->Add(plotObs, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 3);
    FlexGridSizer3->Add(FlexGridSizer4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer5 = new wxFlexGridSizer(4, 1, 0, 0);
    FlexGridSizer5->AddGrowableCol(0);
    FlexGridSizer5->AddGrowableRow(1);
    FlexGridSizer5->AddGrowableRow(3);
    Panel5 = new wxPanel(this, ID_PANEL5, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL5"));
    Panel5->SetBackgroundColour(wxColour(255,255,0));
    GridSizer3 = new wxGridSizer(0, 1, 0, 0);
    lbMap = new wxStaticText(Panel5, ID_STATICTEXT10, _("Map"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT10"));
    wxFont lbMapFont(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    lbMap->SetFont(lbMapFont);
    GridSizer3->Add(lbMap, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel5->SetSizer(GridSizer3);
    GridSizer3->Fit(Panel5);
    GridSizer3->SetSizeHints(Panel5);
    FlexGridSizer5->Add(Panel5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    plotMap = new mpWindow(this,ID_CUSTOM3,wxDefaultPosition,wxDefaultSize,0);
    FlexGridSizer5->Add(plotMap, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 3);
    Panel6 = new wxPanel(this, ID_PANEL6, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL6"));
    Panel6->SetBackgroundColour(wxColour(255,255,0));
    GridSizer4 = new wxGridSizer(0, 1, 0, 0);
    lmIndCompat = new wxStaticText(Panel6, ID_STATICTEXT3, _("Indiv. compat (95% ellips.)"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT3"));
    wxFont lmIndCompatFont(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    lmIndCompat->SetFont(lmIndCompatFont);
    GridSizer4->Add(lmIndCompat, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel6->SetSizer(GridSizer4);
    GridSizer4->Fit(Panel6);
    GridSizer4->SetSizeHints(Panel6);
    FlexGridSizer5->Add(Panel6, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    plotIndivCompat = new mpWindow(this,ID_CUSTOM4,wxDefaultPosition,wxDefaultSize,0);
    FlexGridSizer5->Add(plotIndivCompat, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 3);
    FlexGridSizer3->Add(FlexGridSizer5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer6 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer6->AddGrowableCol(0);
    FlexGridSizer6->AddGrowableRow(1);
    Panel7 = new wxPanel(this, ID_PANEL7, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL7"));
    Panel7->SetBackgroundColour(wxColour(255,255,0));
    GridSizer5 = new wxGridSizer(0, 1, 0, 0);
    lbDatAssoc = new wxStaticText(Panel7, ID_STATICTEXT4, _("Dat assoc"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT4"));
    wxFont lbDatAssocFont(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    lbDatAssoc->SetFont(lbDatAssocFont);
    GridSizer5->Add(lbDatAssoc, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel7->SetSizer(GridSizer5);
    GridSizer5->Fit(Panel7);
    GridSizer5->SetSizeHints(Panel7);
    FlexGridSizer6->Add(Panel7, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    gridDA = new wxGrid(this, ID_GRID1, wxDefaultPosition, wxDefaultSize, 0, _T("ID_GRID1"));
    gridDA->CreateGrid(3,3);
    gridDA->EnableEditing(false);
    gridDA->EnableGridLines(true);
    gridDA->SetColLabelSize(20);
    gridDA->SetRowLabelSize(40);
    gridDA->SetDefaultColSize(40, true);
    gridDA->SetDefaultCellFont( gridDA->GetFont() );
    gridDA->SetDefaultCellTextColour( gridDA->GetForegroundColour() );
    FlexGridSizer6->Add(gridDA, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer3->Add(FlexGridSizer6, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer7 = new wxFlexGridSizer(0, 1, 0, 0);
    FlexGridSizer7->AddGrowableCol(0);
    FlexGridSizer7->AddGrowableRow(0);
    Notebook1 = new wxNotebook(this, ID_NOTEBOOK1, wxDefaultPosition, wxDefaultSize, 0, _T("ID_NOTEBOOK1"));
    Panel1 = new wxPanel(Notebook1, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL1"));
    FlexGridSizer8 = new wxFlexGridSizer(6, 1, 0, 0);
    FlexGridSizer8->AddGrowableCol(0);
    FlexGridSizer8->AddGrowableRow(1);
    FlexGridSizer8->AddGrowableRow(3);
    FlexGridSizer8->AddGrowableRow(5);
    Panel8 = new wxPanel(Panel1, ID_PANEL9, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL9"));
    Panel8->SetBackgroundColour(wxColour(255,255,0));
    GridSizer7 = new wxGridSizer(0, 1, 0, 0);
    StaticText2 = new wxStaticText(Panel8, ID_STATICTEXT5, _("Vehicle X (err,99% bounds)"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT5"));
    wxFont StaticText2Font(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    StaticText2->SetFont(StaticText2Font);
    GridSizer7->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel8->SetSizer(GridSizer7);
    GridSizer7->Fit(Panel8);
    GridSizer7->SetSizeHints(Panel8);
    FlexGridSizer8->Add(Panel8, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    plotErrorX = new mpWindow(Panel1,ID_CUSTOM7,wxDefaultPosition,wxDefaultSize,0);
    FlexGridSizer8->Add(plotErrorX, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 3);
    Panel9 = new wxPanel(Panel1, ID_PANEL10, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL10"));
    Panel9->SetBackgroundColour(wxColour(255,255,0));
    GridSizer8 = new wxGridSizer(0, 1, 0, 0);
    StaticText3 = new wxStaticText(Panel9, ID_STATICTEXT6, _("Vehicle Y (err,99% bounds)"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT6"));
    wxFont StaticText3Font(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    StaticText3->SetFont(StaticText3Font);
    GridSizer8->Add(StaticText3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel9->SetSizer(GridSizer8);
    GridSizer8->Fit(Panel9);
    GridSizer8->SetSizeHints(Panel9);
    FlexGridSizer8->Add(Panel9, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    plotErrorY = new mpWindow(Panel1,ID_CUSTOM8,wxDefaultPosition,wxDefaultSize,0);
    FlexGridSizer8->Add(plotErrorY, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 3);
    Panel10 = new wxPanel(Panel1, ID_PANEL11, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL11"));
    Panel10->SetBackgroundColour(wxColour(255,255,0));
    GridSizer9 = new wxGridSizer(0, 1, 0, 0);
    StaticText4 = new wxStaticText(Panel10, ID_STATICTEXT7, _("Vehicle Phi (err,99% bounds)"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT7"));
    wxFont StaticText4Font(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    StaticText4->SetFont(StaticText4Font);
    GridSizer9->Add(StaticText4, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel10->SetSizer(GridSizer9);
    GridSizer9->Fit(Panel10);
    GridSizer9->SetSizeHints(Panel10);
    FlexGridSizer8->Add(Panel10, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    plotErrorPhi = new mpWindow(Panel1,ID_CUSTOM9,wxDefaultPosition,wxDefaultSize,0);
    FlexGridSizer8->Add(plotErrorPhi, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 3);
    Panel1->SetSizer(FlexGridSizer8);
    FlexGridSizer8->Fit(Panel1);
    FlexGridSizer8->SetSizeHints(Panel1);
    Panel2 = new wxPanel(Notebook1, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL2"));
    FlexGridSizer9 = new wxFlexGridSizer(8, 1, 0, 0);
    FlexGridSizer9->AddGrowableCol(0);
    FlexGridSizer9->AddGrowableRow(1);
    FlexGridSizer9->AddGrowableRow(3);
    FlexGridSizer9->AddGrowableRow(5);
    FlexGridSizer9->AddGrowableRow(7);
    Panel13 = new wxPanel(Panel2, ID_PANEL8, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL8"));
    Panel13->SetBackgroundColour(wxColour(255,255,0));
    GridSizer6 = new wxGridSizer(0, 1, 0, 0);
    lbDaTP = new wxStaticText(Panel13, ID_STATICTEXT9, _("True positives:"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT9"));
    wxFont lbDaTPFont(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    lbDaTP->SetFont(lbDaTPFont);
    GridSizer6->Add(lbDaTP, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel13->SetSizer(GridSizer6);
    GridSizer6->Fit(Panel13);
    GridSizer6->SetSizeHints(Panel13);
    FlexGridSizer9->Add(Panel13, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    plotDaTP = new mpWindow(Panel2,ID_CUSTOM5,wxDefaultPosition,wxDefaultSize,0);
    FlexGridSizer9->Add(plotDaTP, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 3);
    Panel14 = new wxPanel(Panel2, ID_PANEL14, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL14"));
    Panel14->SetBackgroundColour(wxColour(255,255,0));
    GridSizer11 = new wxGridSizer(0, 1, 0, 0);
    lbDaTN = new wxStaticText(Panel14, ID_STATICTEXT11, _("True negatives:"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT11"));
    wxFont lbDaTNFont(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    lbDaTN->SetFont(lbDaTNFont);
    GridSizer11->Add(lbDaTN, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel14->SetSizer(GridSizer11);
    GridSizer11->Fit(Panel14);
    GridSizer11->SetSizeHints(Panel14);
    FlexGridSizer9->Add(Panel14, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    plotDaTN = new mpWindow(Panel2,ID_CUSTOM6,wxDefaultPosition,wxDefaultSize,0);
    FlexGridSizer9->Add(plotDaTN, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    Panel15 = new wxPanel(Panel2, ID_PANEL15, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL15"));
    Panel15->SetBackgroundColour(wxColour(255,255,0));
    GridSizer12 = new wxGridSizer(0, 1, 0, 0);
    StaticText6 = new wxStaticText(Panel15, ID_STATICTEXT12, _("False positives:"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT12"));
    wxFont StaticText6Font(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    StaticText6->SetFont(StaticText6Font);
    GridSizer12->Add(StaticText6, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel15->SetSizer(GridSizer12);
    GridSizer12->Fit(Panel15);
    GridSizer12->SetSizeHints(Panel15);
    FlexGridSizer9->Add(Panel15, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    plotDaFP = new mpWindow(Panel2,ID_CUSTOM11,wxDefaultPosition,wxDefaultSize,0);
    FlexGridSizer9->Add(plotDaFP, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    Panel16 = new wxPanel(Panel2, ID_PANEL16, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL16"));
    Panel16->SetBackgroundColour(wxColour(255,255,0));
    GridSizer13 = new wxGridSizer(0, 1, 0, 0);
    StaticText7 = new wxStaticText(Panel16, ID_STATICTEXT13, _("False negatives:"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT13"));
    wxFont StaticText7Font(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    StaticText7->SetFont(StaticText7Font);
    GridSizer13->Add(StaticText7, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel16->SetSizer(GridSizer13);
    GridSizer13->Fit(Panel16);
    GridSizer13->SetSizeHints(Panel16);
    FlexGridSizer9->Add(Panel16, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    plotDaFN = new mpWindow(Panel2,ID_CUSTOM12,wxDefaultPosition,wxDefaultSize,0);
    FlexGridSizer9->Add(plotDaFN, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    Panel2->SetSizer(FlexGridSizer9);
    FlexGridSizer9->Fit(Panel2);
    FlexGridSizer9->SetSizeHints(Panel2);
    Panel11 = new wxPanel(Notebook1, ID_PANEL12, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL12"));
    FlexGridSizer2 = new wxFlexGridSizer(4, 1, 0, 0);
    FlexGridSizer2->AddGrowableCol(0);
    FlexGridSizer2->AddGrowableRow(1);
    Panel12 = new wxPanel(Panel11, ID_PANEL13, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL13"));
    Panel12->SetBackgroundColour(wxColour(255,255,0));
    GridSizer10 = new wxGridSizer(0, 1, 0, 0);
    StaticText1 = new wxStaticText(Panel12, ID_STATICTEXT8, _("Computation time (ms)"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT8"));
    wxFont StaticText1Font(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    StaticText1->SetFont(StaticText1Font);
    GridSizer10->Add(StaticText1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel12->SetSizer(GridSizer10);
    GridSizer10->Fit(Panel12);
    GridSizer10->SetSizeHints(Panel12);
    FlexGridSizer2->Add(Panel12, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    plotStatTime = new mpWindow(Panel11,ID_CUSTOM10,wxDefaultPosition,wxDefaultSize,0);
    FlexGridSizer2->Add(plotStatTime, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 3);
    Panel17 = new wxPanel(Panel11, ID_PANEL17, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL, _T("ID_PANEL17"));
    Panel17->SetBackgroundColour(wxColour(255,255,0));
    GridSizer14 = new wxGridSizer(0, 1, 0, 0);
    StaticText5 = new wxStaticText(Panel17, ID_STATICTEXT14, _("JCBB iterations (if used)"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT14"));
    wxFont StaticText5Font(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
    StaticText5->SetFont(StaticText5Font);
    GridSizer14->Add(StaticText5, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    Panel17->SetSizer(GridSizer14);
    GridSizer14->Fit(Panel17);
    GridSizer14->SetSizeHints(Panel17);
    FlexGridSizer2->Add(Panel17, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 0);
    plotDaJCBB = new mpWindow(Panel11,ID_CUSTOM13,wxDefaultPosition,wxDefaultSize,0);
    FlexGridSizer2->Add(plotDaJCBB, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_BOTTOM, 3);
    Panel11->SetSizer(FlexGridSizer2);
    FlexGridSizer2->Fit(Panel11);
    FlexGridSizer2->SetSizeHints(Panel11);
    Notebook1->AddPage(Panel1, _("Errors"), true);
    Notebook1->AddPage(Panel2, _("Data assoc."), false);
    Notebook1->AddPage(Panel11, _("Stats"), false);
    FlexGridSizer7->Add(Notebook1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer3->Add(FlexGridSizer7, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer1->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    SetSizer(FlexGridSizer1);
    MenuBar1 = new wxMenuBar();
    Menu1 = new wxMenu();
    MenuItem3 = new wxMenuItem(Menu1, ID_MENUITEM1, _("Reset simulation\tCtrl+F2"), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(MenuItem3);
    Menu1->AppendSeparator();
    mnuOneStep = new wxMenuItem(Menu1, ID_MENUITEM2, _("Run one step\tF11"), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(mnuOneStep);
    mnuRun = new wxMenuItem(Menu1, ID_MENUITEM3, _("Run (interactive)...\tF5"), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(mnuRun);
    mnuStop = new wxMenuItem(Menu1, ID_MENUITEM6, _("Stop\tF2"), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(mnuStop);
    mnuRunBatch = new wxMenuItem(Menu1, ID_MENUITEM4, _("Run (non-interactive)...\tCtrl-F5"), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(mnuRunBatch);
    Menu1->AppendSeparator();
    MenuItem7 = new wxMenuItem(Menu1, ID_MENUITEM5, _("Parameters..."), wxEmptyString, wxITEM_NORMAL);
    Menu1->Append(MenuItem7);
    Menu1->AppendSeparator();
    MenuItem1 = new wxMenuItem(Menu1, idMenuQuit, _("Quit\tAlt-F4"), _("Quit the application"), wxITEM_NORMAL);
    Menu1->Append(MenuItem1);
    MenuBar1->Append(Menu1, _("&Simulation"));
    Menu3 = new wxMenu();
    MenuItem5 = new wxMenuItem(Menu3, ID_MENUITEM8, _("Save filter state..."), wxEmptyString, wxITEM_NORMAL);
    Menu3->Append(MenuItem5);
    mnuSaveLastDA = new wxMenuItem(Menu3, ID_MENUITEM11, _("Save last data association state..."), wxEmptyString, wxITEM_NORMAL);
    Menu3->Append(mnuSaveLastDA);
    Menu3->AppendSeparator();
    mnuItemSaveRawlog = new wxMenuItem(Menu3, ID_MENUITEM_SAVE_RAWLOG, _("Enable save rawlog..."), wxEmptyString, wxITEM_CHECK);
    Menu3->Append(mnuItemSaveRawlog);
    Menu3->AppendSeparator();
    MenuItem4 = new wxMenu();
    MenuItem6 = new wxMenuItem(MenuItem4, ID_MENUITEM9, _("View stats"), wxEmptyString, wxITEM_NORMAL);
    MenuItem4->Append(MenuItem6);
    MenuItem8 = new wxMenuItem(MenuItem4, ID_MENUITEM10, _("Reset stats"), wxEmptyString, wxITEM_NORMAL);
    MenuItem4->Append(MenuItem8);
    Menu3->Append(ID_MENUITEM7, _("Profiler"), MenuItem4, wxEmptyString);
    MenuBar1->Append(Menu3, _("&Tools"));
    Menu2 = new wxMenu();
    MenuItem2 = new wxMenuItem(Menu2, idMenuAbout, _("&About\tF1"), _("Show info about this application"), wxITEM_NORMAL);
    Menu2->Append(MenuItem2);
    MenuBar1->Append(Menu2, _("&Help"));
    SetMenuBar(MenuBar1);
    StatusBar1 = new wxStatusBar(this, ID_STATUSBAR1, 0, _T("ID_STATUSBAR1"));
    int __wxStatusBarWidths_1[1] = { -1 };
    int __wxStatusBarStyles_1[1] = { wxSB_NORMAL };
    StatusBar1->SetFieldsCount(1,__wxStatusBarWidths_1);
    StatusBar1->SetStatusStyles(1,__wxStatusBarStyles_1);
    SetStatusBar(StatusBar1);
    ToolBar1 = new wxToolBar(this, ID_TOOLBAR1, wxDefaultPosition, wxDefaultSize, wxTB_FLAT|wxTB_HORIZONTAL|wxTB_TEXT|wxNO_BORDER, _T("ID_TOOLBAR1"));
    ToolBarItem1 = ToolBar1->AddTool(ID_TOOLBARITEM1, _("Reset"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_RESET")),wxART_TOOLBAR), wxNullBitmap, wxITEM_NORMAL, _("Reset the simulation"), _("Reset the simulation"));
    ToolBar1->AddSeparator();
    ToolBarItem2 = ToolBar1->AddTool(ID_TOOLBARITEM2, _("One step"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_STEP")),wxART_TOOLBAR), wxNullBitmap, wxITEM_NORMAL, _("Run one step"), _("Run one step"));
    ToolBarItem3 = ToolBar1->AddTool(ID_BTNRUN, _("Run..."), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_PLAY")),wxART_TOOLBAR), wxNullBitmap, wxITEM_CHECK, _("Continuous run with animations..."), _("Continuous run with animations..."));
    ToolBarItem4 = ToolBar1->AddTool(ID_BTNSTOP, _("Stop"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_STOP")),wxART_TOOLBAR), wxNullBitmap, wxITEM_NORMAL, _("Stop continuous run"), _("Stop continuous run"));
    ToolBarItem5 = ToolBar1->AddTool(ID_TOOLBARITEM4, _("Run (batch)"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_BATCH")),wxART_TOOLBAR), wxNullBitmap, wxITEM_NORMAL, _("Run the whole simulation without animate"), _("Run the whole simulation without animate"));
    ToolBar1->AddSeparator();
    ToolBarItem6 = ToolBar1->AddTool(ID_TOOLBARITEM3, _("Config..."), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_CONFIG")),wxART_TOOLBAR), wxNullBitmap, wxITEM_NORMAL, _("Change simulation parameters..."), _("Change simulation parameters..."));
    ToolBar1->AddSeparator();
    ToolBarItem7 = ToolBar1->AddTool(ID_TOOLBARITEM6, _("About..."), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_HELP")),wxART_TOOLBAR), wxNullBitmap, wxITEM_NORMAL, _("About this program"), _("About this program"));
    ToolBarItem8 = ToolBar1->AddTool(ID_TOOLBARITEM7, _("Quit"), wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("ICON_EXIT")),wxART_TOOLBAR), wxNullBitmap, wxITEM_NORMAL, _("Exit the application"), _("Exit the application"));
    ToolBar1->Realize();
    SetToolBar(ToolBar1);
    timSimul.SetOwner(this, ID_TIMER1);
    FlexGridSizer1->Fit(this);
    FlexGridSizer1->SetSizeHints(this);

    Connect(ID_MENUITEM1,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&slamdemoFrame::OnbtnResetClicked);
    Connect(ID_MENUITEM2,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&slamdemoFrame::OnbtnOneStepClicked);
    Connect(ID_MENUITEM3,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&slamdemoFrame::OnbtnRunClicked);
    Connect(ID_MENUITEM6,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&slamdemoFrame::OnbtnStopClicked);
    Connect(ID_MENUITEM4,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&slamdemoFrame::OnbtnRunBatchClicked);
    Connect(ID_MENUITEM5,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&slamdemoFrame::OnConfigClicked);
    Connect(idMenuQuit,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&slamdemoFrame::OnQuit);
    Connect(ID_MENUITEM8,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&slamdemoFrame::OnMenuSaveFilterState);
    Connect(ID_MENUITEM11,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&slamdemoFrame::OnmnuSaveLastDASelected);
    Connect(ID_MENUITEM_SAVE_RAWLOG,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&slamdemoFrame::OnmnuItemSaveRawlogSelected);
    Connect(ID_MENUITEM9,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&slamdemoFrame::OnMenuProfilerViewStats);
    Connect(ID_MENUITEM10,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&slamdemoFrame::OnMenuProfilerReset);
    Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&slamdemoFrame::OnAbout);
    Connect(ID_TOOLBARITEM1,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&slamdemoFrame::OnbtnResetClicked);
    Connect(ID_TOOLBARITEM2,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&slamdemoFrame::OnbtnOneStepClicked);
    Connect(ID_BTNRUN,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&slamdemoFrame::OnbtnRunClicked);
    Connect(ID_BTNSTOP,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&slamdemoFrame::OnbtnStopClicked);
    Connect(ID_TOOLBARITEM4,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&slamdemoFrame::OnbtnRunBatchClicked);
    Connect(ID_TOOLBARITEM3,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&slamdemoFrame::OnConfigClicked);
    Connect(ID_TOOLBARITEM6,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&slamdemoFrame::OnAbout);
    Connect(ID_TOOLBARITEM7,wxEVT_COMMAND_TOOL_CLICKED,(wxObjectEventFunction)&slamdemoFrame::OnQuit);
    Connect(ID_TIMER1,wxEVT_TIMER,(wxObjectEventFunction)&slamdemoFrame::OntimSimulTrigger);
    //*)

	gridDA->SetSelectionMode( wxGrid::wxGridSelectCells );


	// buttons ------------
	this->ToolBar1->EnableTool(ID_BTNSTOP,false);
	mnuStop->Enable(false);

	// Init graphs:
#define INIT_PLOT_LABELS(_PL,_LBX,_LBY)  \
	_PL->AddLayer( new mpScaleX(wxT(_LBX)) ); \
	_PL->AddLayer( new mpScaleY(wxT(_LBY)) ); \
	_PL->LockAspect(true); \
	_PL->Fit(-10,10,-10,10);

#define INIT_PLOT(_PL)  INIT_PLOT_LABELS(_PL,"x","y")
#define INIT_PLOT_TIME(_PL)  INIT_PLOT_LABELS(_PL,"time","")

	INIT_PLOT(plotGT)
	INIT_PLOT(plotMap)
	INIT_PLOT_TIME(plotErrorX)
	INIT_PLOT_TIME(plotErrorY)
	INIT_PLOT_TIME(plotErrorPhi)
	INIT_PLOT_LABELS(plotIndivCompat,"Bearing","Range")
	INIT_PLOT(plotObs)

	INIT_PLOT_TIME(plotStatTime)

	INIT_PLOT_TIME(plotDaFN)
	INIT_PLOT_TIME(plotDaFP)
	INIT_PLOT_TIME(plotDaTN)
	INIT_PLOT_TIME(plotDaTP)
	INIT_PLOT_TIME(plotDaJCBB)

	vector<float> robot_shape_xs(3);
	vector<float> robot_shape_ys(3);
	robot_shape_xs[0] =0;	robot_shape_ys[0] =-0.10f;
	robot_shape_xs[1] =0;	robot_shape_ys[1] = 0.10f;
	robot_shape_xs[2] =0.2f;	robot_shape_ys[2] = 0;


	// GT plot ------------
	m_lyGTMap = new mpFXYVector();
	m_lyGTMap->SetPen( wxPen(wxColour(0,0,255),5) );
	m_lyGTMap->SetContinuity( false );
	plotGT->AddLayer( m_lyGTMap );

	m_lyGTRobot= new mpPolygon();
	m_lyGTRobot->SetPen( wxPen(wxColour(255,0,0),2) );
	m_lyGTRobot->SetContinuity( true );
	m_lyGTRobot->setPoints(robot_shape_xs,robot_shape_ys);
	m_lyGTRobot->SetCoordinateBase(0,0,0);
	plotGT->AddLayer( m_lyGTRobot );

	m_lyGTvisibleRange= new mpPolygon();
	m_lyGTvisibleRange->SetPen( wxPen(wxColour(0,0,0),1) );
	m_lyGTvisibleRange->SetContinuity( true );
	m_lyGTvisibleRange->SetCoordinateBase(0,0,0);
	plotGT->AddLayer( m_lyGTvisibleRange );



	plotGT->LockAspect();
	plotGT->EnableDoubleBuffer(true);


	// Map plot ------------
	m_lyMapRobot= new mpPolygon();
	m_lyMapRobot->SetPen( wxPen(wxColour(255,0,0),2) );
	m_lyMapRobot->SetContinuity( true );
	m_lyMapRobot->setPoints(robot_shape_xs,robot_shape_ys);
	m_lyMapRobot->SetCoordinateBase(0,0,0);
	plotMap->AddLayer( m_lyMapRobot );
	plotMap->LockAspect();
	plotMap->EnableDoubleBuffer(true);


	// Observations plot ------------
	m_lyObsRobot= new mpPolygon();
	m_lyObsRobot->SetPen( wxPen(wxColour(255,0,0),2) );
	m_lyObsRobot->SetContinuity( true );
	m_lyObsRobot->setPoints(robot_shape_xs,robot_shape_ys);
	m_lyObsRobot->SetCoordinateBase(0,0,0);
	plotObs->AddLayer( m_lyObsRobot );

	m_lyObsvisibleRange= new mpPolygon();
	m_lyObsvisibleRange->SetPen( wxPen(wxColour(0,0,0),1) );
	m_lyObsvisibleRange->SetContinuity( true );
	m_lyObsvisibleRange->SetCoordinateBase(0,0,0);
	plotObs->AddLayer( m_lyObsvisibleRange );

	plotObs->LockAspect();
	plotObs->EnableDoubleBuffer(true);

	// IC plot ------------
	m_lyICvisibleRange= new mpPolygon();
	m_lyICvisibleRange->SetPen( wxPen(wxColour(0,0,0),1,wxLONG_DASH) );
	m_lyICvisibleRange->SetContinuity( true );
	m_lyICvisibleRange->SetCoordinateBase(0,0,0);
	plotIndivCompat->AddLayer( m_lyICvisibleRange );

	plotIndivCompat->LockAspect(false);
	plotIndivCompat->EnableDoubleBuffer(true);


	// X ERROR plot ------------
	m_lyERRX_err = new mpFXYVector();
	m_lyERRX_err->SetPen( wxPen(wxColour(0,0,0),4) );
	m_lyERRX_err->SetContinuity( true );
	plotErrorX->AddLayer( m_lyERRX_err );

	m_lyERRX_boundUp = new mpFXYVector();
	m_lyERRX_boundUp->SetPen( wxPen(wxColour(0,255,0),2) );
	m_lyERRX_boundUp->SetContinuity( true );
	plotErrorX->AddLayer( m_lyERRX_boundUp );

	m_lyERRX_boundDown= new mpFXYVector();
	m_lyERRX_boundDown->SetPen( wxPen(wxColour(0,255,0),2) );
	m_lyERRX_boundDown->SetContinuity( true );
	plotErrorX->AddLayer( m_lyERRX_boundDown );

	plotErrorX->LockAspect(false);
	plotErrorX->EnableDoubleBuffer(true);

	// Y ERROR plot ------------
	m_lyERRY_err = new mpFXYVector();
	m_lyERRY_err->SetPen( wxPen(wxColour(0,0,0),4) );
	m_lyERRY_err->SetContinuity( true );
	plotErrorY->AddLayer( m_lyERRY_err );

	m_lyERRY_boundUp = new mpFXYVector();
	m_lyERRY_boundUp->SetPen( wxPen(wxColour(0,255,0),2) );
	m_lyERRY_boundUp->SetContinuity( true );
	plotErrorY->AddLayer( m_lyERRY_boundUp );

	m_lyERRY_boundDown= new mpFXYVector();
	m_lyERRY_boundDown->SetPen( wxPen(wxColour(0,255,0),2) );
	m_lyERRY_boundDown->SetContinuity( true );
	plotErrorY->AddLayer( m_lyERRY_boundDown );

	plotErrorY->LockAspect(false);
	plotErrorY->EnableDoubleBuffer(true);

	// Phi ERROR plot ------------
	m_lyERRPHI_err = new mpFXYVector();
	m_lyERRPHI_err->SetPen( wxPen(wxColour(0,0,0),4) );
	m_lyERRPHI_err->SetContinuity( true );
	plotErrorPhi->AddLayer( m_lyERRPHI_err );

	m_lyERRPHI_boundUp = new mpFXYVector();
	m_lyERRPHI_boundUp->SetPen( wxPen(wxColour(0,255,0),2) );
	m_lyERRPHI_boundUp->SetContinuity( true );
	plotErrorPhi->AddLayer( m_lyERRPHI_boundUp );

	m_lyERRPHI_boundDown= new mpFXYVector();
	m_lyERRPHI_boundDown->SetPen( wxPen(wxColour(0,255,0),2) );
	m_lyERRPHI_boundDown->SetContinuity( true );
	plotErrorPhi->AddLayer( m_lyERRPHI_boundDown );

	plotErrorPhi->LockAspect(false);
	plotErrorPhi->EnableDoubleBuffer(true);


	// Stats Time plot ------------
	m_lyStatTimes = new mpFXYVector();
	m_lyStatTimes->SetPen( wxPen(wxColour(0,0,255),3) );
	m_lyStatTimes->SetContinuity( true );
	plotStatTime->AddLayer( m_lyStatTimes );

	plotStatTime->LockAspect(false);
	plotStatTime->EnableDoubleBuffer(true);

	// DA Stats plots ------------
#define INIT_DA_PLOT(CODE) \
	m_lyDa##CODE = new mpFXYVector(); \
	m_lyDa##CODE->SetPen( wxPen(wxColour(0,0,0),5) ); \
	m_lyDa##CODE->SetContinuity( false ); \
	plotDa##CODE->AddLayer( m_lyDa##CODE ); \
	plotDa##CODE->LockAspect(false); \
	plotDa##CODE->EnableDoubleBuffer(true);

	INIT_DA_PLOT(FP);
	INIT_DA_PLOT(FN);
	INIT_DA_PLOT(TP);
	INIT_DA_PLOT(TN);

	INIT_DA_PLOT(JCBB);

	// Set some default params:
	m_SLAM.options.std_sensor_range = 0.03f;
	m_SLAM.options.std_sensor_yaw   = DEG2RAD(0.5f);

	options.sensor_max_range = 5;
	options.sensor_min_range = 0.50;
	options.sensor_fov		= DEG2RAD(140);

	options.sensorDistingishesLandmarks = false;

	m_SLAM.KF_options.method = kfEKFNaive;
//	m_SLAM.KF_options.method = kfEKFAlaDavison;
//	m_SLAM.KF_options.method = kfIKFFull;
//	m_SLAM.KF_options.IKF_iterations = 4;

	m_SLAM.KF_options.enable_profiler	= true;
	//m_SLAM.KF_options.debug_verify_analytic_jacobians = true;

    // Init everything:
	resetSimulator(options.map_generator);
	updateAllGraphs();

    Maximize();
}

slamdemoFrame::~slamdemoFrame()
{
    //(*Destroy(slamdemoFrame)
    //*)
}

/*---------------------------------------------------------------
						Quit
  ---------------------------------------------------------------*/
void slamdemoFrame::OnQuit(wxCommandEvent& event)
{
	// Stop simulation
	wxCommandEvent dumm;
	OnbtnStopClicked(dumm);

    Close();
}

/*---------------------------------------------------------------
						About
  ---------------------------------------------------------------*/
void slamdemoFrame::OnAbout(wxCommandEvent& event)
{
	CAboutBox	dlg(this);
	dlg.ShowModal();
}

/*---------------------------------------------------------------
						Reset btn
  ---------------------------------------------------------------*/
void slamdemoFrame::OnbtnResetClicked(wxCommandEvent& event)
{
	resetSimulator(options.map_generator);
	updateAllGraphs();
}

/*---------------------------------------------------------------
						1 step btn
  ---------------------------------------------------------------*/
void slamdemoFrame::OnbtnOneStepClicked(wxCommandEvent& event)
{
	static CTicTac tictac;
	tictac.Tic();
	executeOneStep();
	const double T = tictac.Tac();
	StatusBar1->SetStatusText( wxString::Format(_("Step %u done in %.03fms"),(unsigned)m_historicData.size(),1e3*T) );
	updateAllGraphs();
}

/*---------------------------------------------------------------
						Run btn
  ---------------------------------------------------------------*/
void slamdemoFrame::OnbtnRunClicked(wxCommandEvent& event)
{
	this->ToolBar1->EnableTool(ID_BTNSTOP,true);
	mnuStop->Enable(true);

	this->ToolBar1->ToggleTool(ID_BTNRUN,true);
	this->ToolBar1->EnableTool(ID_BTNRUN,false);
	mnuRun->Enable(false);

	// Prepare next step:
	timSimul.Start(10,true);
}

/*---------------------------------------------------------------
						Stop btn
  ---------------------------------------------------------------*/
void slamdemoFrame::OnbtnStopClicked(wxCommandEvent& event)
{
	this->ToolBar1->EnableTool(ID_BTNSTOP,false);
	mnuStop->Enable(false);

	this->ToolBar1->ToggleTool(ID_BTNRUN,false);
	this->ToolBar1->EnableTool(ID_BTNRUN,true);
	mnuRun->Enable(true);
}

/*---------------------------------------------------------------
						batch run btn
  ---------------------------------------------------------------*/
void slamdemoFrame::OnbtnRunBatchClicked(wxCommandEvent& event)
{
	static CTicTac tictac;

	wxBusyCursor  info;
	wxTheApp->Yield();  // Let the app. process messages

	tictac.Tic();
	const size_t N = (options.path_square_len/options.robot_step_length)*4+50;
	for (size_t i=0;i<N;i++)
		executeOneStep();

	const double T = tictac.Tac();

	updateAllGraphs();

	StatusBar1->SetStatusText( wxString::Format(_("%u steps done in %.03f secs"),(unsigned)N,T) );
}


/*---------------------------------------------------------------
						Reset simulator
  ---------------------------------------------------------------*/
void slamdemoFrame::resetSimulator( const std::string &map_type )
{
	if (options.random_seed>=0)
			randomGenerator.randomize(options.random_seed);
	else	randomGenerator.randomize();

	m_SLAM.reset();
	m_historicData.clear();
	m_estimatedIDX2realIDX.clear();
	m_realIDX_already_mapped.clear();

	m_lastObservation.sensedData.clear();
	m_lastObservation_GT_indices.clear();

	m_GT_pose = CPose2D(0,0,0);

	// The map:
	// -----------------
	m_GT_map.clear();

	if (map_type=="1")
	{
		// Default map:
		size_t ID =0;
		CLandmark newLM;
		newLM.pose_mean.z = 0;

		for (int i=0;i<=mrpt::utils::round((options.path_square_len)/2.0);i++)
		{
			// Bottom & top  corridors:
			newLM.pose_mean.x = 1+2*i;
			newLM.pose_mean.y =-0.35;
			newLM.ID = ID++;
			m_GT_map.landmarks.push_back(newLM);

			newLM.pose_mean.y +=options.path_square_len;  // Top corridor
			newLM.ID = ID++;
			m_GT_map.landmarks.push_back(newLM);

			newLM.pose_mean.x = 1+2*i;
			newLM.pose_mean.y =-0.6;
			newLM.ID = ID++;
			m_GT_map.landmarks.push_back(newLM);

			newLM.pose_mean.y +=options.path_square_len;  // Top corridor
			newLM.ID = ID++;
			m_GT_map.landmarks.push_back(newLM);

			newLM.pose_mean.x = 2*(1+i);
			newLM.pose_mean.y = 0.35;
			newLM.ID = ID++;
			m_GT_map.landmarks.push_back(newLM);

			newLM.pose_mean.x = 2*i;
			newLM.pose_mean.y +=options.path_square_len;  // Top corridor
			newLM.ID = ID++;
			m_GT_map.landmarks.push_back(newLM);

			newLM.pose_mean.x = 2*(1+i);
			newLM.pose_mean.y = 0.6;
			newLM.ID = ID++;
			m_GT_map.landmarks.push_back(newLM);

			newLM.pose_mean.x = 2*i;
			newLM.pose_mean.y +=options.path_square_len;  // Top corridor
			newLM.ID = ID++;
			m_GT_map.landmarks.push_back(newLM);


			// Left & right corridor:
			newLM.pose_mean.x =-0.35;
			newLM.pose_mean.y = 2*i;
			newLM.ID = ID++;
			m_GT_map.landmarks.push_back(newLM);

			newLM.pose_mean.x +=options.path_square_len;  // Right corridor
			//newLM.pose_mean.y -=1;
			newLM.ID = ID++;
			m_GT_map.landmarks.push_back(newLM);

			newLM.pose_mean.x =-0.6;
			newLM.pose_mean.y = 2*i;
			newLM.ID = ID++;
			m_GT_map.landmarks.push_back(newLM);

			newLM.pose_mean.x +=options.path_square_len;  // Right corridor
			//newLM.pose_mean.y -=1;
			newLM.ID = ID++;
			m_GT_map.landmarks.push_back(newLM);

			newLM.pose_mean.x = 0.35;
			newLM.pose_mean.y = 2*i-1;
			newLM.ID = ID++;
			m_GT_map.landmarks.push_back(newLM);

			newLM.pose_mean.x +=options.path_square_len;  // Right  corridor
			newLM.pose_mean.y +=1+1;
			newLM.ID = ID++;
			m_GT_map.landmarks.push_back(newLM);

			newLM.pose_mean.x = 0.6;
			newLM.pose_mean.y = 2*i-1;
			newLM.ID = ID++;
			m_GT_map.landmarks.push_back(newLM);

			newLM.pose_mean.x +=options.path_square_len;  // Right  corridor
			newLM.pose_mean.y +=1+1;
			newLM.ID = ID++;
			m_GT_map.landmarks.push_back(newLM);

		}

	}
	else if (map_type=="2")
	{
		const double extra_width = 5;
		// Random map:
		for (size_t i=0;i<options.randomMap_nLMs;i++)
		{
			CLandmark newLM;
			newLM.pose_mean.x = randomGenerator.drawUniform(-extra_width,options.path_square_len+extra_width);
			newLM.pose_mean.y = randomGenerator.drawUniform(-extra_width,options.path_square_len+extra_width);
			newLM.pose_mean.z = 0;
			newLM.ID = i;
			m_GT_map.landmarks.push_back(newLM);
		}
	}

	// Refresh all:
	updateAllGraphs(true);
}


/*---------------------------------------------------------------
						updateAllGraphs
  ---------------------------------------------------------------*/
void slamdemoFrame::updateAllGraphs(bool alsoGTMap )
{
	// The area visible by the sensor (used in multiple graphs):
	const size_t AREA_SEGS = 100;
	vector<float> xs_area(AREA_SEGS+1);
	vector<float> ys_area(AREA_SEGS+1);
	for (size_t i=0;i<AREA_SEGS;i++)
	{
		double a = options.sensor_fov*(-0.5+double(i)/(AREA_SEGS-1));
		xs_area[i] = options.sensorOnTheRobot.x() + options.sensor_max_range * cos(a+options.sensorOnTheRobot.phi());
		ys_area[i] = options.sensorOnTheRobot.y() + options.sensor_max_range * sin(a+options.sensorOnTheRobot.phi());
	}
	xs_area[AREA_SEGS] = options.sensorOnTheRobot.x();
	ys_area[AREA_SEGS] = options.sensorOnTheRobot.y();


	// GT Map ----------------------
	if (alsoGTMap)
	{
		vector<float> xs,ys;
		for (CLandmarksMap::TCustomSequenceLandmarks::iterator i=m_GT_map.landmarks.begin();i!=m_GT_map.landmarks.end();++i)
		{
			xs.push_back( i->pose_mean.x );
			ys.push_back( i->pose_mean.y );
		}

		lbGT->SetLabel(_U(format("Ground truth (%u landmarks)",(unsigned)m_GT_map.landmarks.size()).c_str()));

		m_lyGTMap->Clear();
		m_lyGTMap->SetData(xs,ys);

		plotGT->Fit();
	}

	m_lyGTvisibleRange->setPoints(xs_area,ys_area);

	m_lyGTRobot->SetCoordinateBase( m_GT_pose.x(),m_GT_pose.y(),m_GT_pose.phi());
	m_lyGTvisibleRange->SetCoordinateBase( m_GT_pose.x(),m_GT_pose.y(),m_GT_pose.phi());
	plotGT->Refresh();


	// Observation ----------------------
	m_lyObsvisibleRange->setPoints(xs_area,ys_area);

	lbObs->SetLabel(_U(format("Observation (%u landmarks)",(unsigned)m_lastObservation.sensedData.size()).c_str()));

	for (size_t i=0;i<m_lyObsLMs.size();i++)
		plotObs->DelLayer(m_lyObsLMs[i],true);
	m_lyObsLMs.clear();

	CMatrixDouble22 NOISE;
	NOISE(0,0) = square( m_SLAM.options.std_sensor_range );
	NOISE(1,1) = square( m_SLAM.options.std_sensor_yaw );

	// Create an ellipse for each observed landmark:
	for (size_t i=0;i<m_lastObservation.sensedData.size();i++)
	{
		mpCovarianceEllipse	*cov = new mpCovarianceEllipse();
		cov->SetPen( wxPen(wxColour(255,0,0),2) );
		if (m_lastObservation.sensedData[i].landmarkID!=INVALID_LANDMARK_ID)
				cov->SetName(wxString::Format(_("#%u"),(unsigned)m_lastObservation.sensedData[i].landmarkID));
		else	cov->SetName(_("?"));


		// Compute mean & cov:
		const double hr = m_lastObservation.sensedData[i].range;
		const double ha = m_lastObservation.sensedData[i].yaw;
		const double cphi_0sa = cos(options.sensorOnTheRobot.phi() + ha);
		const double sphi_0sa = sin(options.sensorOnTheRobot.phi() + ha);

		const CPoint2D lm_xy = options.sensorOnTheRobot + CPoint2D(hr*cos(ha), hr*sin(ha) );

		// Jacobian wrt hn:
		CMatrixDouble22  dyn_dhn;
		dyn_dhn(0,0) = cphi_0sa;
		dyn_dhn(0,1) = -hr*sphi_0sa;
		dyn_dhn(1,0) = sphi_0sa;
		dyn_dhn(1,1) = hr*cphi_0sa;

		CMatrixDouble22 COV; // COV = H * NOISE * H^T
		dyn_dhn.multiply_HCHt(NOISE,COV);

		cov->SetQuantiles(3);
		cov->SetCoordinateBase(lm_xy.x(),lm_xy.y());
		cov->SetCovarianceMatrix(COV(0,0),COV(0,1),COV(1,1));

		plotObs->AddLayer( cov );
		m_lyObsLMs.push_back(cov);
	}

	plotObs->Fit();
	plotObs->Refresh();

	// Map estimated by the SLAM filter ----------------------
	{
		CPosePDFGaussian estRobotPose;
		vector<TPoint2D>  LMs;
		map<unsigned int,CLandmark::TLandmarkID> landmarkIDs;

		CVectorDouble     Xkk;  // Full mean & cov
		CMatrixDouble     Pkk;

		m_SLAM.getCurrentState(
			estRobotPose,
			LMs,
			landmarkIDs,
			Xkk, Pkk);

		lbMap->SetLabel(wxString::Format(_("Estimated map (%u landmarks)"),(unsigned)LMs.size()));

		// mean robot pose:
		m_lyMapRobot->SetCoordinateBase(estRobotPose.mean.x(),estRobotPose.mean.y(),estRobotPose.mean.phi());

		// Delete old covs:
		for (size_t i=0;i<m_lyMapEllipses.size();i++)
			plotMap->DelLayer(m_lyMapEllipses[i],true);
		m_lyMapEllipses.clear();

		// Robot ellipse:
		{
			mpCovarianceEllipse	*cov = new mpCovarianceEllipse();
			cov->SetPen( wxPen(wxColour(255,0,0),2) );
			cov->SetName(_("Robot"));
			cov->SetQuantiles(3);

			cov->SetCoordinateBase(estRobotPose.mean.x(),estRobotPose.mean.y(),estRobotPose.mean.phi());
			cov->SetCovarianceMatrix( estRobotPose.cov(0,0),estRobotPose.cov(0,1),estRobotPose.cov(1,1) );

			plotMap->AddLayer( cov );
			m_lyMapEllipses.push_back(cov);
		}

		// Landmarks:
		for (size_t i=0;i<LMs.size();i++)
		{
			const size_t idx_in_real_map = m_estimatedIDX2realIDX.find(i)->second;

			mpCovarianceEllipse	*cov = new mpCovarianceEllipse();
			cov->SetPen( wxPen(wxColour(0,0,255),2) );

			if (options.show_map_real_correspondences)
				 cov->SetName(wxString::Format(_("#%u->%u"),(unsigned)i,(unsigned)idx_in_real_map ));
			else cov->SetName(wxString::Format(_("#%u"),(unsigned)i ));

			cov->SetQuantiles(3);

			cov->SetCoordinateBase(LMs[i].x,LMs[i].y);
			const size_t idx = 3 + 2 * i;
			cov->SetCovarianceMatrix( Pkk(idx+0,idx+0),Pkk(idx+0,idx+1),Pkk(idx+1,idx+1) );

			plotMap->AddLayer( cov );
			m_lyMapEllipses.push_back(cov);
		}
	}

	plotMap->Fit();
	plotMap->Refresh();


	// Error plots -----------
	if (!m_historicData.empty())
	{
		const size_t N = m_historicData.size();

		// ERRORS IN X --------------
		if (m_lyERRX_err->GetDataLength()>=N)
		{
			m_lyERRX_err->Clear();
			m_lyERRX_boundUp->Clear();
			m_lyERRX_boundDown->Clear();
		}
		for (size_t i=m_lyERRX_err->GetDataLength();i<N;i++)
		{
			THistoric &h = m_historicData[i];
			m_lyERRX_err->AppendDataPoint( i, h.GT_robot_pose.x() - h.estimate_robot_pose.mean.x() );
			const double std_x = sqrt(h.estimate_robot_pose.cov(0,0));
			m_lyERRX_boundUp->AppendDataPoint( i, 3*std_x );
			m_lyERRX_boundDown->AppendDataPoint( i, -3*std_x );
		}

		// ERRORS IN Y --------------
		if (m_lyERRY_err->GetDataLength()>=N)
		{
			m_lyERRY_err->Clear();
			m_lyERRY_boundUp->Clear();
			m_lyERRY_boundDown->Clear();
		}
		for (size_t i=m_lyERRY_err->GetDataLength();i<N;i++)
		{
			THistoric &h = m_historicData[i];
			m_lyERRY_err->AppendDataPoint( i, h.GT_robot_pose.y() - h.estimate_robot_pose.mean.y() );
			const double std_y = sqrt(h.estimate_robot_pose.cov(1,1));
			m_lyERRY_boundUp->AppendDataPoint( i, 3*std_y );
			m_lyERRY_boundDown->AppendDataPoint( i, -3*std_y );
		}

		// ERRORS IN PHI --------------
		if (m_lyERRPHI_err->GetDataLength()>=N)
		{
			m_lyERRPHI_err->Clear();
			m_lyERRPHI_boundUp->Clear();
			m_lyERRPHI_boundDown->Clear();
		}
		for (size_t i=m_lyERRPHI_err->GetDataLength();i<N;i++)
		{
			THistoric &h = m_historicData[i];
			m_lyERRPHI_err->AppendDataPoint( i, RAD2DEG( mrpt::math::wrapToPi(h.GT_robot_pose.phi() - h.estimate_robot_pose.mean.phi()) ));
			const double std_p = sqrt(h.estimate_robot_pose.cov(2,2));
			m_lyERRPHI_boundUp->AppendDataPoint( i, RAD2DEG(3*std_p) );
			m_lyERRPHI_boundDown->AppendDataPoint( i, RAD2DEG(-3*std_p) );
		}

		plotErrorX->Fit(-0.05*N,N*1.05, m_lyERRX_boundDown->GetMinY()*1.05,m_lyERRX_boundUp->GetMaxY()*1.05 );
		plotErrorX->Refresh();
		plotErrorY->Fit(-0.05*N,N*1.05, m_lyERRY_boundDown->GetMinY()*1.05,m_lyERRY_boundUp->GetMaxY()*1.05 );
		plotErrorY->Refresh();
		plotErrorPhi->Fit(-0.05*N,N*1.05, m_lyERRPHI_boundDown->GetMinY()*1.05,m_lyERRPHI_boundUp->GetMaxY()*1.05 );
		plotErrorPhi->Refresh();

		// Execution times: --------------
		if (m_lyStatTimes->GetDataLength()>=N)
		{
			m_lyStatTimes->Clear();
		}
		for (size_t i=m_lyStatTimes->GetDataLength();i<N;i++)
		{
			THistoric &h = m_historicData[i];
			m_lyStatTimes->AppendDataPoint( i, 1e3*h.run_time );
		}
		plotStatTime->Fit(-0.15*N,N*1.05, -0.10*m_lyStatTimes->GetMaxY(),m_lyStatTimes->GetMaxY()*1.05 );
		plotStatTime->Refresh();
	}


	// Data association graphs ------------------------
	const CRangeBearingKFSLAM2D::TDataAssocInfo & da =  m_SLAM.getLastDataAssociation();

	// IC graph ------------------------------------------
	{
		// Draw sensor ranges in the R-B plane:
		vector<float> xs_area_RG(5);
		vector<float> ys_area_RG(5);
		xs_area_RG[0] = -RAD2DEG(options.sensor_fov)*0.5; ys_area_RG[0] = 0;
		xs_area_RG[1] =  RAD2DEG(options.sensor_fov)*0.5; ys_area_RG[1] = 0;
		xs_area_RG[2] =  RAD2DEG(options.sensor_fov)*0.5; ys_area_RG[2] = options.sensor_max_range;
		xs_area_RG[3] = -RAD2DEG(options.sensor_fov)*0.5; ys_area_RG[3] = options.sensor_max_range;
		xs_area_RG[4] = xs_area_RG[0]; ys_area_RG[4] = ys_area_RG[0];

		m_lyICvisibleRange->setPoints(xs_area_RG,ys_area_RG);

		// Delete old ellipses:
		for (size_t i=0;i<m_lyIC_LMs.size();i++)
			plotIndivCompat->DelLayer(m_lyIC_LMs[i],true);
		m_lyIC_LMs.clear();

		// Create an ellipse for each observed landmark, in
		//  the RANGE-BEARING plane:
		for (size_t i=0;i<m_lastObservation.sensedData.size();i++)
		{
			mpCovarianceEllipse	*cov = new mpCovarianceEllipse();
			cov->SetPen( wxPen(wxColour(255,0,0),2) );
			if (m_lastObservation.sensedData[i].landmarkID!=INVALID_LANDMARK_ID)
					cov->SetName(wxString::Format(_("O(%u)"),(unsigned)m_lastObservation.sensedData[i].landmarkID));
			else	cov->SetName(wxString::Format(_("O%u"),(unsigned)i));

			// Compute mean & cov:
			const double hr = m_lastObservation.sensedData[i].range;
			const double ha = m_lastObservation.sensedData[i].yaw;

			cov->SetQuantiles(2);
			cov->SetCoordinateBase(RAD2DEG(ha),hr);
			cov->SetCovarianceMatrix(RAD2DEGSQ*NOISE(1,1),0,NOISE(0,0));

			plotIndivCompat->AddLayer(cov);
			m_lyIC_LMs.push_back(cov);
		}

		const size_t obs_size = m_SLAM.get_observation_size();

		// Create an ellipse for each PREDICTED map landmark, in
		//  the RANGE-BEARING plane:
		for (size_t i=0;i<da.predictions_IDs.size();i++)
		{
			mpCovarianceEllipse	*cov = new mpCovarianceEllipse();
			cov->SetPen( wxPen(wxColour(0,0,255),2) );
			cov->SetName(wxString::Format(_("P%u"),(unsigned)da.predictions_IDs[i]));

			const double hr = da.Y_pred_means(i,0);
			const double ha = da.Y_pred_means(i,1);

			cov->SetQuantiles(2);
			cov->SetCoordinateBase(RAD2DEG(ha),hr);
			if (da.Y_pred_covs.getColCount()==obs_size )
			{	// Independent predictions:
				ASSERT_(da.Y_pred_covs.getRowCount()==obs_size*da.predictions_IDs.size());
				cov->SetCovarianceMatrix(
					RAD2DEGSQ*da.Y_pred_covs(obs_size*i+1,1),
					RAD2DEG( da.Y_pred_covs(obs_size*i+1,0) ),
					da.Y_pred_covs(obs_size*i+0,0));
			}
			else
			{	// Full cov. predictions:
				ASSERT_(da.Y_pred_covs.isSquare() && da.Y_pred_covs.getColCount()==obs_size*da.predictions_IDs.size());
				cov->SetCovarianceMatrix(
					RAD2DEGSQ*da.Y_pred_covs(obs_size*i+1,obs_size*i+1),
					RAD2DEG( da.Y_pred_covs(obs_size*i+1,obs_size*i+0) ),
					da.Y_pred_covs(obs_size*i+0,obs_size*i+0));
			}

			plotIndivCompat->AddLayer(cov);
			m_lyIC_LMs.push_back(cov);
		}

		// Draw lines between individually compatible covs:
		for (unsigned int o=0;o<m_lastObservation.sensedData.size();o++)
		{
			const double hr_o = m_lastObservation.sensedData[o].range;
			const double ha_o = m_lastObservation.sensedData[o].yaw;

			for (unsigned int p=0;p<da.predictions_IDs.size();p++)
			{
				bool is_final_assoc =
					(da.results.associations.find(o)!=da.results.associations.end() &&
					da.results.associations.find(o)->second== da.predictions_IDs[p] );

				if (da.results.indiv_compatibility(p,o))
				{
					mpFXYVector *v = new mpFXYVector(wxEmptyString, mpALIGN_SW);
					v->SetPen( wxPen(wxColour(0,0,0), is_final_assoc ? 4:2  ) );
					v->SetName(wxString::Format(_("%f"),da.results.indiv_distances(p,o)));

					const double hr = da.Y_pred_means(p,0);
					const double ha = da.Y_pred_means(p,1);

					vector<float> xs(2);
					vector<float> ys(2);
					xs[0] = RAD2DEG(ha); ys[0] = hr;
					xs[1] = RAD2DEG(ha_o); ys[1] = hr_o;

					v->SetData(xs,ys);
					v->SetContinuity(true);

					plotIndivCompat->AddLayer(v);
					m_lyIC_LMs.push_back(v);
				}
				// Set background if it's a final association:
				if (is_final_assoc)
				{
					gridDA->SetCellBackgroundColour(p,o, wxColor(255,255,0) );
				}
			}
		}



		plotIndivCompat->Fit(
			-RAD2DEG(options.sensor_fov)*0.5-10,
			 RAD2DEG(options.sensor_fov)*0.5+10,
			 -0.10,
			 options.sensor_max_range+0.5);

		plotIndivCompat->Refresh();
	}

	// DA distances ------------------------------------------
	{
		gridDA->BeginBatch();

		// Rows: predictions; Cols: observations
		if (gridDA->GetNumberCols()) gridDA->DeleteCols(0,gridDA->GetNumberCols());
		gridDA->AppendCols( m_lastObservation.sensedData.size() );

		for (unsigned int i=0;i<m_lastObservation.sensedData.size();i++)
			gridDA->SetColLabelValue(i,wxString::Format(wxT("O%u"),i));

		if (gridDA->GetNumberRows()) gridDA->DeleteRows(0,gridDA->GetNumberRows());
		gridDA->AppendRows( da.predictions_IDs.size() );

		for (unsigned int i=0;i<da.predictions_IDs.size();i++)
			gridDA->SetRowLabelValue(i,wxString::Format(_("P%u"),(unsigned)da.predictions_IDs[i]));

		for (unsigned int o=0;o<m_lastObservation.sensedData.size();o++)
			for (unsigned int p=0;p<da.predictions_IDs.size();p++)
			{
				gridDA->SetCellAlignment(p,o,wxALIGN_RIGHT,wxALIGN_CENTRE);

				const double v= da.results.indiv_distances(p,o);
				if (v>500)
						gridDA->SetCellValue(p,o, wxT("\u221E")); // Infinity
				else if (v<-950)
						gridDA->SetCellValue(p,o, wxT("-\u221E")); // -Infinity
				else	gridDA->SetCellValue(p,o, wxString::Format(wxT("%.02f"),v));

				// Set background if it's a final association:
				if (da.results.associations.find(o)!=da.results.associations.end() &&
					da.results.associations.find(o)->second== da.predictions_IDs[p] )
				{
					gridDA->SetCellBackgroundColour(p,o, wxColor(255,255,0) );
				}
			}

		gridDA->EndBatch();
	}

	// DA stats ------------------------------------------
	if (!m_historicData.empty())
	{
		const size_t N = m_historicData.size();

		// DA True positives --------------
		if (m_lyDaTP->GetDataLength()>=N)
			m_lyDaTP->Clear();
		for (size_t i=m_lyDaTP->GetDataLength();i<N;i++)
			m_lyDaTP->AppendDataPoint( i, m_historicData[i].da_true_pos );

		unsigned int totalTP=0;
		for (size_t i=0;i<N;i++)
			totalTP+=m_historicData[i].da_true_pos;
		lbDaTP->SetLabel(wxString::Format(wxT("True positives: %u"),totalTP));

		// DA True negatives --------------
		if (m_lyDaTN->GetDataLength()>=N)
			m_lyDaTN->Clear();
		for (size_t i=m_lyDaTN->GetDataLength();i<N;i++)
			m_lyDaTN->AppendDataPoint( i, m_historicData[i].da_true_neg );

		unsigned int totalTN=0;
		for (size_t i=0;i<N;i++)
			totalTN+=m_historicData[i].da_true_neg;
		lbDaTN->SetLabel(wxString::Format(wxT("True negatives: %u"),totalTN));


		// DA false positives --------------
		if (m_lyDaFP->GetDataLength()>=N)
			m_lyDaFP->Clear();
		for (size_t i=m_lyDaFP->GetDataLength();i<N;i++)
			m_lyDaFP->AppendDataPoint( i, m_historicData[i].da_false_pos);

		unsigned int totalFP=0;
		for (size_t i=0;i<N;i++)
			totalFP+=m_historicData[i].da_false_pos;
		StaticText6->SetLabel(wxString::Format(wxT("False positives: %u"),totalFP));

		// DA false negatives --------------
		if (m_lyDaFN->GetDataLength()>=N)
			m_lyDaFN->Clear();
		for (size_t i=m_lyDaFN->GetDataLength();i<N;i++)
			m_lyDaFN->AppendDataPoint( i, m_historicData[i].da_false_neg );

		unsigned int totalFN=0;
		for (size_t i=0;i<N;i++)
			totalFN+=m_historicData[i].da_false_neg;
		StaticText7->SetLabel(wxString::Format(wxT("False negatives: %u"),totalFN));

		// JCBB iterations --------------
		if (m_lyDaJCBB->GetDataLength()>=N)
			m_lyDaJCBB->Clear();
		for (size_t i=m_lyDaJCBB->GetDataLength();i<N;i++)
			m_lyDaJCBB->AppendDataPoint( i, m_historicData[i].jcbb_iters );

		plotDaFN->Fit(); plotDaFN->Refresh();
		plotDaFP->Fit(); plotDaFP->Refresh();
		plotDaTN->Fit(); plotDaTN->Refresh();
		plotDaTP->Fit(); plotDaTP->Refresh();

		plotDaJCBB->Fit(); plotDaJCBB->Refresh();
	}

}


/*---------------------------------------------------------------
						TSimulationOptions
  ---------------------------------------------------------------*/
slamdemoFrame::TSimulationOptions::TSimulationOptions() :
	random_seed		( -1 ),
	map_generator	("1"),
	randomMap_nLMs	( 70 ),
	sensorOnTheRobot(0,0,0),
	sensor_max_range ( 5 ),
	sensor_min_range ( 0.50 ),
	sensor_fov		 ( DEG2RAD(140) ),
	sensorDistingishesLandmarks (false),
	path_square_len	( 8 ),
	robot_step_length(0.3),
	odometry_noise_std_xy( 0.02 ),
	odometry_noise_std_phi( DEG2RAD(0.2) ),
	uncert_overestim_odom(1.2),
	uncert_overestim_sensor(1.2),
	show_map_real_correspondences(false),
	spurious_count_mean(0),
	spurious_count_std(0)
{
}


void  slamdemoFrame::TSimulationOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase	&f,
	const std::string		&c)
{
	MRPT_LOAD_CONFIG_VAR(random_seed,int,  		f,c)
	MRPT_LOAD_CONFIG_VAR(map_generator,string,	f,c)
	MRPT_LOAD_CONFIG_VAR(randomMap_nLMs,int,	f,c)

	sensorOnTheRobot.x( f.read_double(c,"sensorOnTheRobot.x",sensorOnTheRobot.x()));
	sensorOnTheRobot.y( f.read_double(c,"sensorOnTheRobot.y",sensorOnTheRobot.y()));
	sensorOnTheRobot.phi( DEG2RAD(f.read_double(c,"sensorOnTheRobot.phi",RAD2DEG(sensorOnTheRobot.phi()))));

	MRPT_LOAD_CONFIG_VAR(sensor_max_range,double,	f,c)
	MRPT_LOAD_CONFIG_VAR(sensor_min_range,double,	f,c)
	MRPT_LOAD_CONFIG_VAR_DEGREES(sensor_fov,	f,c)

	MRPT_LOAD_CONFIG_VAR(sensorDistingishesLandmarks,bool,	f,c)

	MRPT_LOAD_CONFIG_VAR(path_square_len,double,	f,c)
	MRPT_LOAD_CONFIG_VAR(robot_step_length,double,	f,c)

	MRPT_LOAD_CONFIG_VAR(odometry_noise_std_xy,double,	f,c)
	MRPT_LOAD_CONFIG_VAR_DEGREES(odometry_noise_std_phi,	f,c)

	MRPT_LOAD_CONFIG_VAR(uncert_overestim_odom,double,	f,c)
	MRPT_LOAD_CONFIG_VAR(uncert_overestim_sensor,double,	f,c)

	MRPT_LOAD_CONFIG_VAR(show_map_real_correspondences,bool,	f,c)

	MRPT_LOAD_CONFIG_VAR(spurious_count_mean,double,	f,c)
	MRPT_LOAD_CONFIG_VAR(spurious_count_std ,double,	f,c)
}

void  slamdemoFrame::TSimulationOptions::saveToConfigFile(
	mrpt::utils::CConfigFileBase	&f,
	const std::string		&c) const
{
	MRPT_SAVE_CONFIG_VAR(random_seed,  	f,c)
	MRPT_SAVE_CONFIG_VAR(map_generator,	f,c)
	MRPT_SAVE_CONFIG_VAR(randomMap_nLMs,f,c)

	f.write(c,"sensorOnTheRobot.x",sensorOnTheRobot.x());
	f.write(c,"sensorOnTheRobot.y",sensorOnTheRobot.y());
	f.write(c,"sensorOnTheRobot.phi",RAD2DEG(sensorOnTheRobot.phi()));

	MRPT_SAVE_CONFIG_VAR(sensor_max_range,	f,c)
	MRPT_SAVE_CONFIG_VAR(sensor_min_range,	f,c)
	MRPT_SAVE_CONFIG_VAR_DEGREES(sensor_fov,	f,c)

	MRPT_SAVE_CONFIG_VAR(sensorDistingishesLandmarks,	f,c)

	MRPT_SAVE_CONFIG_VAR(path_square_len,f,c)
	MRPT_SAVE_CONFIG_VAR(robot_step_length,f,c)

	MRPT_SAVE_CONFIG_VAR(odometry_noise_std_xy,f,c)
	MRPT_SAVE_CONFIG_VAR_DEGREES(odometry_noise_std_phi,f,c)

	MRPT_SAVE_CONFIG_VAR(uncert_overestim_odom,f,c)
	MRPT_SAVE_CONFIG_VAR(uncert_overestim_sensor,f,c)

	MRPT_SAVE_CONFIG_VAR(show_map_real_correspondences,f,c)

	MRPT_SAVE_CONFIG_VAR(spurious_count_mean,f,c)
	MRPT_SAVE_CONFIG_VAR(spurious_count_std,f,c)
}



void  slamdemoFrame::TSimulationOptions::dumpToTextStream( CStream		&out) const
{

}


/*---------------------------------------------------------------
						executeOneStep
  ---------------------------------------------------------------*/
void slamdemoFrame::executeOneStep()
{
	try
	{

	// Move the robot   ------------------------
	// Skip in the first step:
	CPose2D  poseIncr(0,0,0);

	const int turnSteps = 10;
	const double Aphi = DEG2RAD(90.0/turnSteps);

	if (!m_historicData.empty())
	{ // Ok, move:
		const double PATH_SQUARE_LEN = options.path_square_len;

		if ( fabs(fmod(m_GT_pose.phi(),DEG2RAD(90.0)))<1e-2 )
		{
			int dir = mrpt::utils::round(m_GT_pose.phi()/Aphi);

			// Continue in a straight line, unless we reach a corner:
			if (( m_GT_pose.x()>PATH_SQUARE_LEN && m_GT_pose.y()<PATH_SQUARE_LEN && dir==0 ) ||
				( m_GT_pose.x()>PATH_SQUARE_LEN && m_GT_pose.y()>PATH_SQUARE_LEN && dir==turnSteps ) ||
				( m_GT_pose.x()<=0 && abs(dir)==2*turnSteps ) ||
				( m_GT_pose.y()<=0 && dir==-turnSteps )
				)
			{ // Turn
				poseIncr = CPose2D(0,0,Aphi);
			}
			else
			{ // Straight:
				poseIncr = CPose2D(options.robot_step_length ,0,0);
			}
		}
		else
		{ // Continue turning:
			poseIncr = CPose2D(0,0,Aphi);
		}
	}

	m_GT_pose = m_GT_pose + poseIncr;

	// Round phi so we have always perfect square paths:
	m_GT_pose.phi(  mrpt::utils::round(m_GT_pose.phi()/Aphi)*Aphi );



	// Simulate observation ------------------------
	{
		const CPose3D robotPose = CPose3D( this->m_GT_pose );
		const CPose3D sensorOnRobot = CPose3D(options.sensorOnTheRobot);

		m_lastObservation.fieldOfView_yaw = options.sensor_fov;
		m_lastObservation.maxSensorDistance = options.sensor_max_range;
		m_lastObservation.minSensorDistance = options.sensor_min_range;

		m_GT_map.simulateRangeBearingReadings(
			robotPose,
			sensorOnRobot,
			m_lastObservation,
			options.sensorDistingishesLandmarks,
			m_SLAM.options.std_sensor_range / options.uncert_overestim_sensor,
			m_SLAM.options.std_sensor_yaw / options.uncert_overestim_sensor,
			0, // sigma_pitch: we are in 2D
			&m_lastObservation_GT_indices,
			options.spurious_count_mean,
			options.spurious_count_std
			);
	}

	// before processing the observation:
	const std::set<size_t> old_realIDX_already_mapped = m_realIDX_already_mapped;


	// SLAM ------------------------
	double executionTime;
	static CTicTac tictac;
	{
		CActionCollectionPtr act = CActionCollection::Create();
		CActionRobotMovement2D  actmov;
		CActionRobotMovement2D::TMotionModelOptions odo_opts;
		odo_opts.modelSelection = CActionRobotMovement2D::mmGaussian;

		// Model as a constant noise in X,Y,PHI:
		odo_opts.gaussianModel.a1  = 0; //0.01f;
		odo_opts.gaussianModel.a2  = 0; //RAD2DEG( 0.0001f );
		odo_opts.gaussianModel.a3  = 0; //DEG2RAD( 0.1f );
		odo_opts.gaussianModel.a4  = 0; // 0.01; //0.05f;
		odo_opts.gaussianModel.minStdXY  = options.odometry_noise_std_xy;
		odo_opts.gaussianModel.minStdPHI = options.odometry_noise_std_phi;

		// Add noise:
		CPose2D  noisyPoseIncr = poseIncr;
		noisyPoseIncr.x_incr( randomGenerator.drawGaussian1D(0,options.odometry_noise_std_xy/options.uncert_overestim_odom) );
		noisyPoseIncr.y_incr( randomGenerator.drawGaussian1D(0,options.odometry_noise_std_xy/options.uncert_overestim_odom) );
		noisyPoseIncr.phi_incr( randomGenerator.drawGaussian1D(0,options.odometry_noise_std_phi/options.uncert_overestim_odom) );

		actmov.computeFromOdometry(noisyPoseIncr,odo_opts);
		actmov.timestamp = mrpt::system::now();
		act->insert(actmov);

		CSensoryFramePtr sf = CSensoryFrame::Create();
		m_lastObservation.timestamp = mrpt::system::now();
		m_lastObservation.sensorLabel = "SIMUL_2D_RB";

		sf->insert( CObservationBearingRangePtr( new CObservationBearingRange(m_lastObservation) ));

		tictac.Tic();

		m_SLAM.processActionObservation(act,sf);

		executionTime = tictac.Tac();

		// Save dataset to file?
		if (m_rawlog_out_file.fileOpenCorrectly())
		{
			m_rawlog_out_file << act << sf;
		}
	}


	// For the case of doing D.A., save the correspondences REAL_MAP <-> ESTIMATED_MAP ----------
	const CRangeBearingKFSLAM2D::TDataAssocInfo & da =  m_SLAM.getLastDataAssociation();

	for (std::map<size_t,size_t>::const_iterator it=da.newly_inserted_landmarks.begin();it!=da.newly_inserted_landmarks.end();++it)
	{
		const size_t obs_idx = it->first;
		const size_t est_map_idx = it->second;
		const size_t real_map_idx = m_lastObservation_GT_indices[obs_idx];

		m_estimatedIDX2realIDX[est_map_idx] = real_map_idx;

		// Do NOT remember all spureous readings, since they all look alike:
		if (real_map_idx!=std::string::npos)
			m_realIDX_already_mapped.insert(real_map_idx);
	}

	// Save historic data ------------------------
	m_historicData.push_back(THistoric());
	THistoric & hist = m_historicData.back();

	hist.GT_robot_pose = m_GT_pose;
	m_SLAM.getCurrentRobotPose( hist.estimate_robot_pose );
	hist.run_time = executionTime;
	hist.jcbb_iters = da.results.nNodesExploredInJCBB;

	// D.A. history data:
	if (!options.sensorDistingishesLandmarks)
	{
		ASSERT_( m_lastObservation.sensedData.size() == m_lastObservation_GT_indices.size() );

		for (unsigned int o=0;o<m_lastObservation.sensedData.size();o++)
		{
			const bool    o_has_assoc   = (da.results.associations.find(o)!=da.results.associations.end());
			const size_t  o_realmap_idx = m_lastObservation_GT_indices[o]; // Note: This can be "-1" for spurious readings!
			const bool    o_was_mapped  =
				(o_realmap_idx==std::string::npos) ?
					false
					:
					old_realIDX_already_mapped.find(o_realmap_idx)!=old_realIDX_already_mapped.end();
			const bool    o_has_been_just_inserted = da.newly_inserted_landmarks.find(o)!=da.newly_inserted_landmarks.end();

			if ( o_was_mapped )
			{ // ALREADY OBSERVED IN THE PAST

				if (o_has_assoc)
				{
					if (m_estimatedIDX2realIDX[da.results.associations.find(o)->second]==o_realmap_idx)
					{
						// True positive: If the LM was already known and it's been recognized as such...
						hist.da_true_pos++;
					}
					else
					{
						// False positive:
						hist.da_false_pos++;
					}
				}
				else
				{
					ASSERT_(o_has_been_just_inserted)
					// False negative: It was an already known LM but has been wrongly classified as new:
					hist.da_false_neg++;
				}
			}
			else
			{ // OBSERVED FOR THE FIRST TIME

				if (o_has_assoc)
				{
					// False positive:
					hist.da_false_pos++;
				}
				else
				{
					ASSERT_( o_has_been_just_inserted );
					// True negative: If the LM is a new one and it's been inserted as such...
					hist.da_true_neg++;
				}
			}

		}

	}

	}
	catch(std::exception &e)
	{
		try { wxCommandEvent  dum; this->OnbtnStopClicked(dum); } catch(...) {}
		cerr << endl << e.what() << endl;
		wxMessageBox(_U(e.what()),_("Exception"));
	}
}

/*---------------------------------------------------------------
						OntimSimulTrigger
  ---------------------------------------------------------------*/
void slamdemoFrame::OntimSimulTrigger(wxTimerEvent& event)
{
	static CTicTac tictac;

	if (!ToolBar1->GetToolState(ID_BTNRUN) ) return;

	// Simulate one step:
	tictac.Tic();

	executeOneStep();

	const double T = tictac.Tac();

	updateAllGraphs();

	StatusBar1->SetStatusText( wxString::Format(_("Step %u done in %.03fms"),(unsigned)m_historicData.size(),1e3*T) );

	wxTheApp->Yield(true);  // Let the app. process messages

	// Prepare next step:
	timSimul.Start(20,true);
}

/*---------------------------------------------------------------
						OnConfigClicked
  ---------------------------------------------------------------*/
void slamdemoFrame::OnConfigClicked(wxCommandEvent& event)
{
	CDlgParams  dlg(this);

	// Put all params
	// ---------------------------------------
	dlg.rbKFnaiv->SetValue( m_SLAM.KF_options.method == kfEKFNaive );
	dlg.rbKFdavison->SetValue( m_SLAM.KF_options.method == kfEKFAlaDavison );
	dlg.rbIKF->SetValue( m_SLAM.KF_options.method == kfIKFFull );
	dlg.rbIKFdavison->SetValue( m_SLAM.KF_options.method == kfIKF );
	dlg.edIKFiters->SetValue( m_SLAM.KF_options.IKF_iterations );

	dlg.cbJacobTran->SetValue( !m_SLAM.KF_options.use_analytic_transition_jacobian  );
	dlg.cbJacobObs->SetValue(  !m_SLAM.KF_options.use_analytic_observation_jacobian );

	//dlg.rbFusion->SetSelection( (int)m_SLAM.KF_options.fusion_strategy );

	dlg.rbMapCorridor->SetValue( options.map_generator=="1" );
	dlg.rbMapRandom->SetValue( options.map_generator=="2" );

	if (options.map_generator!="1" && options.map_generator!="2")
		dlg.rbMapFile->SetValue(true);

	dlg.edLMs->SetValue( options.randomMap_nLMs );

	dlg.edPathLen->SetValue( wxString::Format(_("%f"),options.path_square_len) );
	dlg.edPathStepSize->SetValue( wxString::Format(_("%f"),options.robot_step_length) );
	dlg.edOdomStdXY->SetValue( wxString::Format(_("%f"),options.odometry_noise_std_xy ) );
	dlg.edStdOdomPhi->SetValue( wxString::Format(_("%f"),RAD2DEG(options.odometry_noise_std_phi) ));

	dlg.cbSensorDistin->SetValue( options.sensorDistingishesLandmarks );
	dlg.edSeed->SetValue( options.random_seed );

	dlg.edStdRange->SetValue( wxString::Format(_("%f"),m_SLAM.options.std_sensor_range) );
	dlg.edStdAngle->SetValue( wxString::Format(_("%f"),RAD2DEG(m_SLAM.options.std_sensor_yaw)) );

	dlg.edSpuriousMean->SetValue( wxString::Format(_("%f"),options.spurious_count_mean) );
	dlg.edSpuriousStd->SetValue( wxString::Format(_("%f"),options.spurious_count_std) );

	dlg.edSenX->SetValue( wxString::Format(_("%f"),options.sensorOnTheRobot.x() ) );
	dlg.edSenY->SetValue( wxString::Format(_("%f"),options.sensorOnTheRobot.y() ) );
	dlg.edSenPhi->SetValue( wxString::Format(_("%f"),RAD2DEG(options.sensorOnTheRobot.phi()) ) );

	dlg.edMaxR->SetValue( wxString::Format(_("%f"),options.sensor_max_range ) );
	dlg.edMinR->SetValue( wxString::Format(_("%f"),options.sensor_min_range ) );
	dlg.edFOV->SetValue( wxString::Format(_("%f"),RAD2DEG(options.sensor_fov )) );

	dlg.edChi2->SetValue(wxString::Format(_("%.04f"),m_SLAM.options.data_assoc_IC_chi2_thres ) );
	dlg.rbDAMethod->SetSelection( int(m_SLAM.options.data_assoc_method) );
	dlg.rbDAMetric->SetSelection( int(m_SLAM.options.data_assoc_metric) );
	dlg.rbICmetric->SetSelection( int(m_SLAM.options.data_assoc_IC_metric) );
	dlg.edICMLrefDist->SetValue(wxString::Format(_("%.04f"),m_SLAM.options.data_assoc_IC_ml_threshold ) );


	dlg.edOverOdom->SetValue( options.uncert_overestim_odom * 100 );
	dlg.edOverSensor->SetValue( options.uncert_overestim_sensor * 100 );

	{
		wxCommandEvent dum;
		dlg.OnUpdateControlsState(dum);
	}


	if (dlg.ShowModal()==wxID_OK)
	{
		// Get all params
		// ---------------------------------------
		if (dlg.rbKFnaiv->GetValue())  m_SLAM.KF_options.method = kfEKFNaive;
		if (dlg.rbKFdavison->GetValue()) m_SLAM.KF_options.method = kfEKFAlaDavison;
		if (dlg.rbIKF->GetValue()) m_SLAM.KF_options.method = kfIKFFull;
		if (dlg.rbIKFdavison->GetValue()) m_SLAM.KF_options.method = kfIKF;
		m_SLAM.KF_options.IKF_iterations = dlg.edIKFiters->GetValue();

		m_SLAM.KF_options.use_analytic_transition_jacobian  = !dlg.cbJacobTran->GetValue();
		m_SLAM.KF_options.use_analytic_observation_jacobian = !dlg.cbJacobObs->GetValue();

//		m_SLAM.KF_options.fusion_strategy = TKFFusionMethod(dlg.rbFusion->GetSelection());


		if (dlg.rbMapCorridor->GetValue())
			options.map_generator="1";
		else if (dlg.rbMapRandom->GetValue())
			options.map_generator="2";
		else options.map_generator = string( dlg.edMapFile->GetValue().mb_str() );

		options.randomMap_nLMs = dlg.edLMs->GetValue();
		options.random_seed = dlg.edSeed->GetValue();

		options.path_square_len = atof( dlg.edPathLen->GetValue().mb_str() );
		options.robot_step_length = atof( dlg.edPathStepSize->GetValue().mb_str() );
		options.odometry_noise_std_xy = atof( dlg.edOdomStdXY->GetValue().mb_str() );
		options.odometry_noise_std_phi = DEG2RAD( atof( dlg.edStdOdomPhi->GetValue().mb_str() ));

		options.sensorDistingishesLandmarks = dlg.cbSensorDistin->GetValue();

		m_SLAM.options.std_sensor_range = atof( dlg.edStdRange->GetValue().mb_str() );
		m_SLAM.options.std_sensor_yaw   = DEG2RAD( atof( dlg.edStdAngle->GetValue().mb_str() ) );

		options.spurious_count_mean = atof( dlg.edSpuriousMean->GetValue().mb_str() );
		options.spurious_count_std = atof( dlg.edSpuriousStd->GetValue().mb_str() );

		options.sensorOnTheRobot.x( atof( dlg.edSenX->GetValue().mb_str() ) );
		options.sensorOnTheRobot.y( atof( dlg.edSenY->GetValue().mb_str() ) );
		options.sensorOnTheRobot.phi( DEG2RAD( atof( dlg.edSenPhi->GetValue().mb_str() ) ) );

		options.sensor_max_range = atof( dlg.edMaxR->GetValue().mb_str() );
		options.sensor_min_range = atof( dlg.edMinR->GetValue().mb_str() );
		options.sensor_fov  = DEG2RAD( atof( dlg.edFOV->GetValue().mb_str() ) );

		m_SLAM.options.data_assoc_IC_chi2_thres = atof( dlg.edChi2->GetValue().mb_str() );
		m_SLAM.options.data_assoc_method = TDataAssociationMethod(dlg.rbDAMethod->GetSelection());
		m_SLAM.options.data_assoc_metric = TDataAssociationMetric(dlg.rbDAMetric->GetSelection());
		m_SLAM.options.data_assoc_IC_metric= TDataAssociationMetric(dlg.rbICmetric->GetSelection());
		m_SLAM.options.data_assoc_IC_ml_threshold = atof( dlg.edICMLrefDist->GetValue().mb_str() );


		options.uncert_overestim_odom   = 0.01*dlg.edOverOdom->GetValue();
		options.uncert_overestim_sensor = 0.01*dlg.edOverSensor->GetValue();
	}
}

void slamdemoFrame::OnMenuSaveFilterState(wxCommandEvent& event)
{
	CPosePDFGaussian estRobotPose;
	vector<TPoint2D>  LMs;
	map<unsigned int,CLandmark::TLandmarkID> landmarkIDs;

	CVectorDouble     Xkk;  // Full mean & cov
	CMatrixDouble     Pkk;

	m_SLAM.getCurrentState(
		estRobotPose,
		LMs,
		landmarkIDs,
		Xkk, Pkk);

	{
		wxFileDialog dialog(
			this,
			_("Save state vector...") /*caption*/,
			_(".") /* defaultDir */,
			_("kf_mean.txt") /* defaultFilename */,
			_("Text files (*.txt)|*.txt|All files (*.*)|*.*") /* wildcard */,
			wxFD_SAVE | wxFD_OVERWRITE_PROMPT  );

		if (dialog.ShowModal() != wxID_OK) return;
		string filName( dialog.GetPath().mb_str() );

		Xkk.saveToTextFile(filName);
	}
	{
		wxFileDialog dialog(
			this,
			_("Save covariance matrix...") /*caption*/,
			_(".") /* defaultDir */,
			_("kf_cov.txt") /* defaultFilename */,
			_("Text files (*.txt)|*.txt|All files (*.*)|*.*") /* wildcard */,
			wxFD_SAVE | wxFD_OVERWRITE_PROMPT  );

		if (dialog.ShowModal() != wxID_OK) return;
		string filName( dialog.GetPath().mb_str() );

		Pkk.saveToTextFile(filName);
	}

	{
		wxFileDialog dialog(
			this,
			_("Save as 3D opengl scene...") /*caption*/,
			_(".") /* defaultDir */,
			_("slam.3Dscene") /* defaultFilename */,
			_("MRPT 3D scenes (*.3Dscene)|*.3Dscene|All files (*.*)|*.*") /* wildcard */,
			wxFD_SAVE | wxFD_OVERWRITE_PROMPT  );

		if (dialog.ShowModal() != wxID_OK) return;
		string filName( dialog.GetPath().mb_str() );

		// Save as 3D objects:
		mrpt::opengl::CSetOfObjectsPtr obj3D = mrpt::opengl::CSetOfObjects::Create();
		m_SLAM.getAs3DObject(obj3D);

		mrpt::opengl::COpenGLScene scene;
		scene.insert(obj3D);

		CFileGZOutputStream(filName) << scene;
	}

}

void slamdemoFrame::OnMenuProfilerViewStats(wxCommandEvent& event)
{
	string profStats = m_SLAM.getProfiler().getStatsAsText();
	CLogView  dlg(this);
	dlg.edLog->SetValue( _U(profStats.c_str()) );
	dlg.ShowModal();
}

void slamdemoFrame::OnMenuProfilerReset(wxCommandEvent& event)
{
	m_SLAM.getProfiler().clear();
}


void slamdemoFrame::OnmnuSaveLastDASelected(wxCommandEvent& event)
{
	// Data association graphs ------------------------
	const CRangeBearingKFSLAM2D::TDataAssocInfo & da =  m_SLAM.getLastDataAssociation();

	{
		wxFileDialog dialog(
			this,
			_("Save prediction landmark IDs...") /*caption*/,
			_(".") /* defaultDir */,
			_("prediction_IDs.txt") /* defaultFilename */,
			_("Text files (*.txt)|*.txt|All files (*.*)|*.*") /* wildcard */,
			wxFD_SAVE | wxFD_OVERWRITE_PROMPT  );

		if (dialog.ShowModal() != wxID_OK) return;
		string filName( dialog.GetPath().mb_str() );

		mrpt::system::vectorToTextFile(da.predictions_IDs,filName);
	}
	{
		wxFileDialog dialog(
			this,
			_("Save prediction means...") /*caption*/,
			_(".") /* defaultDir */,
			_("prediction_means.txt") /* defaultFilename */,
			_("Text files (*.txt)|*.txt|All files (*.*)|*.*") /* wildcard */,
			wxFD_SAVE | wxFD_OVERWRITE_PROMPT  );

		if (dialog.ShowModal() != wxID_OK) return;
		string filName( dialog.GetPath().mb_str() );

		da.Y_pred_means.saveToTextFile(filName);
	}
	{
		wxFileDialog dialog(
			this,
			_("Save prediction covariance...") /*caption*/,
			_(".") /* defaultDir */,
			_("prediction_cov.txt") /* defaultFilename */,
			_("Text files (*.txt)|*.txt|All files (*.*)|*.*") /* wildcard */,
			wxFD_SAVE | wxFD_OVERWRITE_PROMPT  );

		if (dialog.ShowModal() != wxID_OK) return;
		string filName( dialog.GetPath().mb_str() );

		da.Y_pred_covs.saveToTextFile(filName);
	}
}

void slamdemoFrame::OnmnuItemSaveRawlogSelected(wxCommandEvent& event)
{
	const bool saveMnuChkd = mnuItemSaveRawlog->IsChecked();

	if (m_rawlog_out_file.fileOpenCorrectly())
		m_rawlog_out_file.close();

	if (saveMnuChkd)
	{
		// Start saving:
		wxFileDialog dialog(
			this,
			_("Save data set...") /*caption*/,
			_(".") /* defaultDir */,
			_("simul-range-bearing_dataset.rawlog") /* defaultFilename */,
			_("Rawlogs (*.rawlog)|*.rawlog|All files (*.*)|*.*") /* wildcard */,
			wxFD_SAVE | wxFD_OVERWRITE_PROMPT  );

		if (dialog.ShowModal() != wxID_OK) return;
		const string filName( dialog.GetPath().mb_str() );

		if (!m_rawlog_out_file.open(filName))
		{
			mnuItemSaveRawlog->Check(false);
			wxMessageBox(_("Cannot open output file..."),_("Error"));
			return;
		}

		// Save a first "observation" with historic data:
		CObservationComment obs;
		obs.text =
			std::string("Rawlog generated by 2d-slam-demo\n"
						" MRPT version: ") + mrpt::system::MRPT_getVersion() + std::string("\n"
						" Creation date: ") + mrpt::system::dateTimeLocalToString(mrpt::system::now()) + std::string("\n");

		m_rawlog_out_file << obs;
	}

}
