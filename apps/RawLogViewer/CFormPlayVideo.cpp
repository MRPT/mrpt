/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "CFormPlayVideo.h"
#include "xRawLogViewerMain.h"

#include <wx/filedlg.h>
#include <wx/msgdlg.h>
#include <wx/app.h>
#include <wx/dcmemory.h>
#include <wx/dirdlg.h>
#include <wx/dcclient.h>

//(*InternalHeaders(CFormPlayVideo)
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/intl.h>
#include <wx/image.h>
#include <wx/string.h>
//*)

#include <mrpt/gui/WxUtils.h>


//(*IdInit(CFormPlayVideo)
const long CFormPlayVideo::ID_RADIOBUTTON1 = wxNewId();
const long CFormPlayVideo::ID_RADIOBUTTON2 = wxNewId();
const long CFormPlayVideo::ID_STATICTEXT22 = wxNewId();
const long CFormPlayVideo::ID_TEXTCTRL11 = wxNewId();
const long CFormPlayVideo::ID_BUTTON4 = wxNewId();
const long CFormPlayVideo::ID_CHECKBOX1 = wxNewId();
const long CFormPlayVideo::ID_CHECKBOX2 = wxNewId();
const long CFormPlayVideo::ID_CHECKBOX3 = wxNewId();
const long CFormPlayVideo::ID_CHECKBOX4 = wxNewId();
const long CFormPlayVideo::ID_STATICTEXT8 = wxNewId();
const long CFormPlayVideo::ID_SPINCTRL2 = wxNewId();
const long CFormPlayVideo::ID_STATICTEXT6 = wxNewId();
const long CFormPlayVideo::ID_COMBOBOX1 = wxNewId();
const long CFormPlayVideo::ID_BUTTON2 = wxNewId();
const long CFormPlayVideo::ID_BUTTON3 = wxNewId();
const long CFormPlayVideo::ID_STATICTEXT7 = wxNewId();
const long CFormPlayVideo::ID_TEXTCTRL2 = wxNewId();
const long CFormPlayVideo::ID_BUTTON5 = wxNewId();
const long CFormPlayVideo::ID_SLIDER1 = wxNewId();
const long CFormPlayVideo::ID_STATICTEXT1 = wxNewId();
const long CFormPlayVideo::ID_SPINCTRL1 = wxNewId();
const long CFormPlayVideo::ID_BUTTON1 = wxNewId();
const long CFormPlayVideo::ID_STATICTEXT3 = wxNewId();
const long CFormPlayVideo::ID_STATICTEXT5 = wxNewId();
const long CFormPlayVideo::ID_BITMAPBUTTON1 = wxNewId();
const long CFormPlayVideo::ID_STATICTEXT4 = wxNewId();
const long CFormPlayVideo::ID_BITMAPBUTTON2 = wxNewId();
const long CFormPlayVideo::ID_STATICTEXT2 = wxNewId();
const long CFormPlayVideo::ID_BITMAPBUTTON3 = wxNewId();
const long CFormPlayVideo::ID_PANEL3 = wxNewId();
const long CFormPlayVideo::ID_PANEL4 = wxNewId();
const long CFormPlayVideo::ID_PANEL2 = wxNewId();
//*)

BEGIN_EVENT_TABLE(CFormPlayVideo,wxDialog)
    //(*EventTable(CFormPlayVideo)
    //*)
END_EVENT_TABLE()

// Global variables:
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/gui/WxUtils.h>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;


extern TTimeStamp		rawlog_first_timestamp;

std::vector<CObservationPtr> 	displayedImgs(3);


CFormPlayVideo::CFormPlayVideo(wxWindow* parent,wxWindowID id)
{
	WX_START_TRY

    m_nowPlaying = false;
	firstFit = true;

    //(*Initialize(CFormPlayVideo)
    wxFlexGridSizer* FlexGridSizer4;
    wxFlexGridSizer* FlexGridSizer10;
    wxFlexGridSizer* FlexGridSizer3;
    wxFlexGridSizer* FlexGridSizer5;
    wxFlexGridSizer* FlexGridSizer9;
    wxFlexGridSizer* FlexGridSizer7;
    wxFlexGridSizer* FlexGridSizer12;
    wxFlexGridSizer* FlexGridSizer11;

    Create(parent, wxID_ANY, _("Play images in a Rawlog as a video"), wxDefaultPosition, wxDefaultSize, wxCAPTION|wxDEFAULT_DIALOG_STYLE|wxSYSTEM_MENU|wxRESIZE_BORDER|wxCLOSE_BOX|wxMAXIMIZE_BOX, _T("wxID_ANY"));
    FlexGridSizer1 = new wxFlexGridSizer(5, 1, 0, 0);
    FlexGridSizer1->AddGrowableCol(0);
    FlexGridSizer1->AddGrowableRow(4);
    StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Apply to:"));
    BoxSizer4 = new wxBoxSizer(wxVERTICAL);
    BoxSizer5 = new wxBoxSizer(wxHORIZONTAL);
    FlexGridSizer6 = new wxFlexGridSizer(2, 4, 0, 0);
    FlexGridSizer6->AddGrowableCol(2);
    rbLoaded = new wxRadioButton(this, ID_RADIOBUTTON1, _("Loaded rawlog"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_RADIOBUTTON1"));
    rbLoaded->SetValue(true);
    FlexGridSizer6->Add(rbLoaded, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer6->Add(-1,-1,1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer6->Add(-1,-1,1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer6->Add(-1,-1,1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    rbFile = new wxRadioButton(this, ID_RADIOBUTTON2, _("Rawlog in file:"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_RADIOBUTTON2"));
    FlexGridSizer6->Add(rbFile, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    StaticText22 = new wxStaticText(this, ID_STATICTEXT22, _("Input file:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT22"));
    FlexGridSizer6->Add(StaticText22, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edFile = new wxTextCtrl(this, ID_TEXTCTRL11, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL11"));
    FlexGridSizer6->Add(edFile, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnPickInput = new wxButton(this, ID_BUTTON4, _("Select..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON4"));
    FlexGridSizer6->Add(btnPickInput, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
    BoxSizer5->Add(FlexGridSizer6, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    BoxSizer4->Add(BoxSizer5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    StaticBoxSizer1->Add(BoxSizer4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer1->Add(StaticBoxSizer1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer9 = new wxFlexGridSizer(3, 3, 0, 0);
    cbOrderByYaw = new wxCheckBox(this, ID_CHECKBOX1, _("Order images by Yaw (<0:left,=0:middle,>0:right)"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
    cbOrderByYaw->SetValue(false);
    FlexGridSizer9->Add(cbOrderByYaw, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbOrderByY = new wxCheckBox(this, ID_CHECKBOX2, _("Order by \'y\' (<0: left, >0:right)"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX2"));
    cbOrderByY->SetValue(false);
    FlexGridSizer9->Add(cbOrderByY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbReduceLarge = new wxCheckBox(this, ID_CHECKBOX3, _("Reduce large images (w>640px)"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX3"));
    cbReduceLarge->SetValue(true);
    FlexGridSizer9->Add(cbReduceLarge, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    cbDrawStereoRules = new wxCheckBox(this, ID_CHECKBOX4, _("Draw horizontal rules\?"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX4"));
    cbDrawStereoRules->SetValue(false);
    FlexGridSizer9->Add(cbDrawStereoRules, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText3 = new wxStaticText(this, ID_STATICTEXT8, _("Rule vertical spacing (pixels):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT8"));
    FlexGridSizer9->Add(StaticText3, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    edHorzRuleSpace = new wxSpinCtrl(this, ID_SPINCTRL2, _T("50"), wxDefaultPosition, wxDefaultSize, 0, 3, 1000, 50, _T("ID_SPINCTRL2"));
    edHorzRuleSpace->SetValue(_T("50"));
    FlexGridSizer9->Add(edHorzRuleSpace, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    StaticText2 = new wxStaticText(this, ID_STATICTEXT6, _("Directory for delayed-load (external) images:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT6"));
    FlexGridSizer9->Add(StaticText2, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
    cbImageDirs = new wxComboBox(this, ID_COMBOBOX1, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, 0, wxCB_READONLY|wxCB_DROPDOWN, wxDefaultValidator, _T("ID_COMBOBOX1"));
    FlexGridSizer9->Add(cbImageDirs, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer1->Add(FlexGridSizer9, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer8 = new wxFlexGridSizer(2, 7, 0, 0);
    FlexGridSizer8->AddGrowableCol(5);
    btnPlay = new wxButton(this, ID_BUTTON2, _("Play"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
    FlexGridSizer8->Add(btnPlay, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnStop = new wxButton(this, ID_BUTTON3, _("Stop"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON3"));
    btnStop->Disable();
    FlexGridSizer8->Add(btnStop, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer8->Add(-1,-1,1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    StaticText6 = new wxStaticText(this, ID_STATICTEXT7, _("Additional delay (ms):"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT7"));
    FlexGridSizer8->Add(StaticText6, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edDelay = new wxTextCtrl(this, ID_TEXTCTRL2, _("10"), wxDefaultPosition, wxSize(61,21), 0, wxDefaultValidator, _T("ID_TEXTCTRL2"));
    FlexGridSizer8->Add(edDelay, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer8->Add(-1,-1,1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    btnClose = new wxButton(this, ID_BUTTON5, _("CLOSE"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON5"));
    FlexGridSizer8->Add(btnClose, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer1->Add(FlexGridSizer8, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer4 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer4->AddGrowableCol(0);
    FlexGridSizer5 = new wxFlexGridSizer(1, 1, 0, 0);
    FlexGridSizer5->AddGrowableCol(0);
    progressBar = new wxSlider(this, ID_SLIDER1, 0, 0, 100, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_SLIDER1"));
    FlexGridSizer5->Add(progressBar, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer4->Add(FlexGridSizer5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer7 = new wxFlexGridSizer(1, 4, 0, 0);
    FlexGridSizer7->AddGrowableCol(3);
    StaticText1 = new wxStaticText(this, ID_STATICTEXT1, _("Rawlog index:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
    FlexGridSizer7->Add(StaticText1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    edIndex = new wxSpinCtrl(this, ID_SPINCTRL1, _T("0"), wxDefaultPosition, wxDefaultSize, 0, 0, 100, 0, _T("ID_SPINCTRL1"));
    edIndex->SetValue(_T("0"));
    FlexGridSizer7->Add(edIndex, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnJump = new wxButton(this, ID_BUTTON1, _("Jump"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
    FlexGridSizer7->Add(btnJump, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    lbProgress = new wxStaticText(this, ID_STATICTEXT3, _("(Progress)"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
    FlexGridSizer7->Add(lbProgress, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    FlexGridSizer4->Add(FlexGridSizer7, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer1->Add(FlexGridSizer4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer3 = new wxFlexGridSizer(2, 3, 0, 0);
    FlexGridSizer3->AddGrowableCol(0);
    FlexGridSizer3->AddGrowableCol(1);
    FlexGridSizer3->AddGrowableCol(2);
    FlexGridSizer3->AddGrowableRow(1);
    FlexGridSizer10 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer10->AddGrowableCol(0);
    lbCam1 = new wxStaticText(this, ID_STATICTEXT5, _("Cam1\?\?"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT5"));
    FlexGridSizer10->Add(lbCam1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnSaveCam1 = new wxBitmapButton(this, ID_BITMAPBUTTON1, wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FILE_SAVE_AS")),wxART_BUTTON), wxDefaultPosition, wxDefaultSize, wxBU_AUTODRAW, wxDefaultValidator, _T("ID_BITMAPBUTTON1"));
    btnSaveCam1->SetDefault();
    FlexGridSizer10->Add(btnSaveCam1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer3->Add(FlexGridSizer10, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer11 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer11->AddGrowableCol(0);
    lbCam2 = new wxStaticText(this, ID_STATICTEXT4, _("Cam2\?\?"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT4"));
    FlexGridSizer11->Add(lbCam2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnSaveCam2 = new wxBitmapButton(this, ID_BITMAPBUTTON2, wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FILE_SAVE_AS")),wxART_BUTTON), wxDefaultPosition, wxDefaultSize, wxBU_AUTODRAW, wxDefaultValidator, _T("ID_BITMAPBUTTON2"));
    btnSaveCam2->SetDefault();
    FlexGridSizer11->Add(btnSaveCam2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer3->Add(FlexGridSizer11, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    FlexGridSizer12 = new wxFlexGridSizer(1, 2, 0, 0);
    FlexGridSizer12->AddGrowableCol(0);
    lbCam3 = new wxStaticText(this, ID_STATICTEXT2, _("Cam3\?\?"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
    FlexGridSizer12->Add(lbCam3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnSaveCam3 = new wxBitmapButton(this, ID_BITMAPBUTTON3, wxArtProvider::GetBitmap(wxART_MAKE_ART_ID_FROM_STR(_T("wxART_FILE_SAVE_AS")),wxART_BUTTON), wxDefaultPosition, wxDefaultSize, wxBU_AUTODRAW, wxDefaultValidator, _T("ID_BITMAPBUTTON3"));
    btnSaveCam3->SetDefault();
    FlexGridSizer12->Add(btnSaveCam3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
    FlexGridSizer3->Add(FlexGridSizer12, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    pnLeft = new wxPanel(this, ID_PANEL3, wxDefaultPosition, wxSize(320,240), wxSUNKEN_BORDER|wxTAB_TRAVERSAL, _T("ID_PANEL3"));
    pnLeft->SetMinSize(wxSize(-1,240));
    FlexGridSizer3->Add(pnLeft, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    pnRight = new wxPanel(this, ID_PANEL4, wxDefaultPosition, wxSize(320,240), wxSUNKEN_BORDER|wxTAB_TRAVERSAL, _T("ID_PANEL4"));
    pnRight->SetMinSize(wxSize(-1,240));
    FlexGridSizer3->Add(pnRight, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    pnRight2 = new wxPanel(this, ID_PANEL2, wxDefaultPosition, wxSize(320,240), wxSUNKEN_BORDER|wxTAB_TRAVERSAL, _T("ID_PANEL2"));
    pnRight2->SetMinSize(wxSize(-1,240));
    FlexGridSizer3->Add(pnRight2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 1);
    FlexGridSizer1->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
    SetSizer(FlexGridSizer1);
    FlexGridSizer1->Fit(this);
    FlexGridSizer1->SetSizeHints(this);
    Center();

    Connect(ID_RADIOBUTTON1,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&CFormPlayVideo::OnrbLoadedSelect);
    Connect(ID_RADIOBUTTON2,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&CFormPlayVideo::OnrbFileSelect);
    Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormPlayVideo::OnbtnPickClick);
    Connect(ID_COMBOBOX1,wxEVT_COMMAND_COMBOBOX_SELECTED,(wxObjectEventFunction)&CFormPlayVideo::OncbImageDirsSelect);
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormPlayVideo::OnbtnPlayClick);
    Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormPlayVideo::OnbtnStopClick);
    Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormPlayVideo::OnbtnCloseClick);
    Connect(ID_SLIDER1,wxEVT_SCROLL_THUMBTRACK,(wxObjectEventFunction)&CFormPlayVideo::OnprogressBarCmdScrollChanged);
    Connect(ID_SLIDER1,wxEVT_SCROLL_CHANGED,(wxObjectEventFunction)&CFormPlayVideo::OnprogressBarCmdScrollChanged);
    Connect(ID_BITMAPBUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormPlayVideo::OnbtnSaveCam1Click);
    Connect(ID_BITMAPBUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormPlayVideo::OnbtnSaveCam2Click);
    Connect(ID_BITMAPBUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CFormPlayVideo::OnbtnSaveCam3Click);
    Connect(wxID_ANY,wxEVT_INIT_DIALOG,(wxObjectEventFunction)&CFormPlayVideo::OnInit);
    //*)

	WX_END_TRY
}

CFormPlayVideo::~CFormPlayVideo()
{
    //(*Destroy(CFormPlayVideo)
    //*)
}

// Pick a file:
void CFormPlayVideo::OnbtnPickClick(wxCommandEvent& event)
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

        // Save the path
        try
        {
            iniFile->write(iniFileSect,"LastDir",std::string(filePath.mb_str()));
        }
        catch (std::exception &e)
        {
            wxMessageBox( wxString(e.what(),wxConvUTF8), wxT("Exception"), wxOK, this);
        }

        // Sets the file:
        edFile->ChangeValue( fileName );
    }
}

// On stop:
void CFormPlayVideo::OnbtnStopClick(wxCommandEvent& event)
{
    m_nowPlaying = false;
}

// On play:
void CFormPlayVideo::OnbtnPlayClick(wxCommandEvent& event)
{
    btnPlay->Enable(false);
    btnStop->Enable(true);
    m_nowPlaying = true;

    try
    {
        long delay_ms=0;
        edDelay->GetValue().ToLong( &delay_ms );

        CFileGZInputStream  *fil;


        if (rbFile->GetValue())
        {
            // Load from file:
            fil = new CFileGZInputStream ( std::string(edFile->GetValue().mb_str()) );
        }
        else
        {
            // Use the loaded rawlog:
            fil = NULL;
        }

        size_t      nImgs=0,count=0;

		// If we are playing from memory, continue:
		if (!fil)
		{
			count = edIndex->GetValue();
		}

        progressBar->SetRange(0, fil ? (int)fil->getTotalBytesCount() : (int)rawlog.size() );
        progressBar->SetValue(0);


        // Repeat until EOF exception or cancel.
        while (m_nowPlaying)
        {
            wxTheApp->Yield();
            CSerializablePtr obj;

            if (fil)
            {
                (*fil) >> obj;
            }
            else
            {
            	obj = rawlog.getAsGeneric(count);
            	m_idxInRawlog  = count;
            }

			bool doDelay = false;

            if (IS_CLASS(obj,CSensoryFrame))
            {
                doDelay = showSensoryFrame(obj.pointer(), nImgs);
            }
            else if (IS_DERIVED(obj,CObservation))
            {
            	CSensoryFrame	sf;
            	sf.insert( CObservationPtr(obj) );
                doDelay = showSensoryFrame( &sf, nImgs);
            }

            // Free the loaded object!
            if (fil) obj.clear();

            // Update UI
            if ((count++)%100==0)
            {
                progressBar->SetValue( fil ? (int)fil->getPosition():(int)count );
                wxString str;
                str.sprintf(_("Processed: %d images"),nImgs);
                lbProgress->SetLabel( str );
                if (!doDelay) wxTheApp->Yield();
            }

			//if (doDelay || (count % 100)==0)
			edIndex->SetValue(count);

            // End?
            if (!fil && count>=rawlog.size()) m_nowPlaying=false;

            // Time to process stop button press and redraw image.


			if (doDelay)
			{
				wxTheApp->Yield();
				wxMilliSleep(delay_ms);
			}
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
    catch (std::exception &e)
    {
        wxMessageBox( _U(e.what()), _("Exception"), wxOK, this);
    }

    btnPlay->Enable(true);
    btnStop->Enable(false);
}


void CFormPlayVideo::OnInit(wxInitDialogEvent& event)
{
    wxCommandEvent	dumm;
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
    } // end there is loaded rawlog

}

void CFormPlayVideo::OnrbLoadedSelect(wxCommandEvent& event)
{
    btnPickInput->Disable();
    edFile->Disable();

	progressBar->SetRange(0, (int)rawlog.size() );
	edIndex->SetRange(0, (int)rawlog.size() );
	progressBar->Enable();
	edIndex->Enable();
}

void CFormPlayVideo::OnrbFileSelect(wxCommandEvent& event)
{
    btnPickInput->Enable();
    edFile->Enable();
	progressBar->Disable();
	edIndex->Disable();
}

void CFormPlayVideo::OnbtnCloseClick(wxCommandEvent& event)
{
    m_nowPlaying = false;
    Close();
}

void CFormPlayVideo::drawHorzRules( mrpt::utils::CImage &img)
{
	if (!cbDrawStereoRules->IsChecked())
		return;
	
	img.forceLoad();
	const size_t Ay = edHorzRuleSpace->GetValue();
	const size_t h = img.getHeight();
	const size_t w = img.getWidth();

	for (size_t y=Ay;y<h;y+=Ay)
		img.line(0,y, w-1,y, mrpt::utils::TColor::white);
}


bool CFormPlayVideo::showSensoryFrame(void *SF, size_t &nImgs)
{
	WX_START_TRY

    ASSERT_(SF);
    CSensoryFrame *sf = (CSensoryFrame*)SF;

    bool 	doDelay=false;
    bool  	doReduceLargeImgs = cbReduceLarge->GetValue();
	bool 	orderByYaw = cbOrderByYaw->GetValue();
    bool  	orderByY   = cbOrderByY->GetValue();

    // Find an image to show:
    CImage  *imgShow = NULL;

    // Displayed images:
    //displayedImgs.resize(3);

	// unload current imgs:
	for (size_t i=0;i<displayedImgs.size();i++)
		if (displayedImgs[i])
			displayedImgs[i]->unload();


	CImage  auxImgForSubSampling;

	// Monocular images:
	{
		wxPanel *thePanel;
		wxStaticText *theLabel = NULL;

		for (int img_idx_sf=0;img_idx_sf<3;img_idx_sf++) // Up to 3 images maximum:
		{
			CObservationImagePtr obsImg = sf->getObservationByClass<CObservationImage>(img_idx_sf );
			if (!obsImg) break; // No more images, go on...

			// Onto which panel to draw??
			if (!orderByYaw && !orderByY)
			{
				// Sequentially
				switch(img_idx_sf)
				{
				case 0: thePanel = pnLeft; theLabel=lbCam1;   break;
				case 1: thePanel = pnRight; theLabel=lbCam2;  break;
				case 2: thePanel = pnRight2; theLabel=lbCam3;  break;
				default: ASSERT_(false);
				};
			}
			else
			{
				if (orderByY)
				{
					// By Y
					if (obsImg->cameraPose.y() <0 )
					{
						thePanel = pnRight;
						theLabel = lbCam2;
					}
					else if (obsImg->cameraPose.y() >0 )
					{
						thePanel = pnLeft;
						theLabel=lbCam1;
					}
					else
					{
						thePanel = pnRight2;
						theLabel=lbCam3;
					}
				}
				else
				{
					// By yaw angle:
					if (obsImg->cameraPose.yaw() < DEG2RAD( -3 ))
					{
						thePanel = pnRight2;
						theLabel=lbCam3;
					}
					else if (obsImg->cameraPose.yaw() > DEG2RAD( 3 ) )
					{
						thePanel = pnLeft;
						theLabel=lbCam1;
					}
					else
					{
						thePanel = pnRight;// Center
						theLabel=lbCam2;
					}
				}
			}

			nImgs++;

			imgShow = &obsImg->image;

			// Subsampling?
			if ( doReduceLargeImgs && imgShow->getWidth() > 650 )
			{
				auxImgForSubSampling = imgShow->scaleHalf();
				imgShow = &auxImgForSubSampling ;
			}

			if (firstFit)
			{
				pnLeft->SetMinSize( wxSize( imgShow->getWidth()+2,imgShow->getHeight()+2 ) );
				pnRight->SetMinSize( wxSize( imgShow->getWidth()+2,imgShow->getHeight()+2 ) );
				pnRight2->SetMinSize( wxSize( imgShow->getWidth()+2,imgShow->getHeight()+2 ) );
				Fit();
				firstFit = false;
			}


			// Draw image:
			drawHorzRules(*imgShow);
			wxImage *wxIMG = mrpt::gui::MRPTImage2wxImage( *imgShow );

			wxWindowDC  dc( thePanel );
			wxMemoryDC  tmpDc;
			tmpDc.SelectObjectAsSource(wxBitmap( *wxIMG ));
			dc.Blit(0,0,wxIMG->GetWidth(), wxIMG->GetHeight(), &tmpDc, 0, 0);
			delete wxIMG;

			// Set text label:
			if ( theLabel )
			{
				double t=0;
				if (obsImg->timestamp!=INVALID_TIMESTAMP)
						t = mrpt::system::timeDifference(rawlog_first_timestamp, obsImg->timestamp);

				string s = format("%s [t=%.03fsec]",obsImg->sensorLabel.c_str(), t);
				theLabel->SetLabel( _U(s.c_str()));
			}

			// save:
			displayedImgs[ thePanel==pnLeft ? 0 : (thePanel==pnRight ? 1:2) ] = obsImg;

			doDelay= true;

		} // end for 0,1,2
	}

	// Stereo images:
	{
        CObservationStereoImagesPtr obsImg2 = sf->getObservationByClass<CObservationStereoImages>();
        if (obsImg2)
        {
            nImgs++;

            // Left:
            {
                imgShow = &obsImg2->imageLeft;

				// Subsampling?
				if ( doReduceLargeImgs && imgShow->getWidth() > 650 )
				{
					auxImgForSubSampling = imgShow->scaleHalf();
					imgShow = &auxImgForSubSampling ;
				}

                if (firstFit)
                {
                    pnLeft->SetMinSize( wxSize( imgShow->getWidth()+2,imgShow->getHeight()+2 ) );
                    //Fit();
                    // firstFit=false; // Done in the right pane below...
                }

				drawHorzRules(*imgShow);
                wxImage *wxIMG = mrpt::gui::MRPTImage2wxImage( *imgShow );
                imgShow->unload();  // for delayed-loaded rawlogs, save lots of memory!

                wxWindowDC  dc( pnLeft );
                wxMemoryDC  tmpDc;
                tmpDc.SelectObjectAsSource(wxBitmap( *wxIMG ));
                dc.Blit(0,0,wxIMG->GetWidth(), wxIMG->GetHeight(), &tmpDc, 0, 0);
                delete wxIMG;

                lbCam1->SetLabel( _U( format( "%s - left", obsImg2->sensorLabel.c_str()).c_str() ));

				// save:
				displayedImgs[ 0 ] = obsImg2;

                doDelay= true;
            }

            // Right, if present:
            if (obsImg2->hasImageRight)
            {
                imgShow = &obsImg2->imageRight;

				// Subsampling?
				if ( doReduceLargeImgs && imgShow->getWidth() > 650 )
				{
					auxImgForSubSampling = imgShow->scaleHalf();
					imgShow = &auxImgForSubSampling ;
				}

                if (firstFit)
                {
                    pnRight->SetMinSize( wxSize( imgShow->getWidth()+2,imgShow->getHeight()+2 ) );
                    Fit();
                    firstFit=false;
                }

				drawHorzRules(*imgShow);
                wxImage *wxIMG = mrpt::gui::MRPTImage2wxImage( *imgShow );
                imgShow->unload();  // for delayed-loaded rawlogs, save lots of memory!

                wxWindowDC  dc( pnRight );
                wxMemoryDC  tmpDc;
                tmpDc.SelectObjectAsSource(wxBitmap( *wxIMG ));
                dc.Blit(0,0,wxIMG->GetWidth(), wxIMG->GetHeight(), &tmpDc, 0, 0);
                delete wxIMG;

                lbCam2->SetLabel( _U( format( "%s - right", obsImg2->sensorLabel.c_str()).c_str() ));

				// save:
				displayedImgs[ 1 ] = obsImg2;

                doDelay= true;
            }

            // Disparity, if present:
            if (obsImg2->hasImageDisparity)
            {
                imgShow = &obsImg2->imageDisparity;

				// Subsampling?
				if ( doReduceLargeImgs && imgShow->getWidth() > 650 )
				{
					auxImgForSubSampling = imgShow->scaleHalf();
					imgShow = &auxImgForSubSampling ;
				}

                if (firstFit)
                {
                    pnRight->SetMinSize( wxSize( imgShow->getWidth()+2,imgShow->getHeight()+2 ) );
                    Fit();
                    firstFit=false;
                }

				drawHorzRules(*imgShow);
                wxImage *wxIMG = mrpt::gui::MRPTImage2wxImage( *imgShow );
                imgShow->unload();  // for delayed-loaded rawlogs, save lots of memory!

                wxWindowDC  dc( pnRight );
                wxMemoryDC  tmpDc;
                tmpDc.SelectObjectAsSource(wxBitmap( *wxIMG ));
                dc.Blit(0,0,wxIMG->GetWidth(), wxIMG->GetHeight(), &tmpDc, 0, 0);
                delete wxIMG;

                lbCam2->SetLabel( _U( format( "%s - disparity", obsImg2->sensorLabel.c_str()).c_str() ));

				// save:
				displayedImgs[ 1 ] = obsImg2;

                doDelay= true;
            }

        }
	}

	// 3D range images:
	{
        CObservation3DRangeScanPtr obs3D = sf->getObservationByClass<CObservation3DRangeScan>();
        if (obs3D && obs3D->hasIntensityImage)
        {
            nImgs++;

			// Intensity channel
            {
                imgShow = &obs3D->intensityImage;

				// Subsampling?
				if ( doReduceLargeImgs && imgShow->getWidth() > 650 )
				{
					auxImgForSubSampling = imgShow->scaleHalf();
					imgShow = &auxImgForSubSampling ;
				}

                if (firstFit)
                {
                    pnLeft->SetMinSize( wxSize( imgShow->getWidth()+2,imgShow->getHeight()+2 ) );
                    //Fit();
                    // firstFit=false; // Done in the right pane below...
                }

				drawHorzRules(*imgShow);
                wxImage *wxIMG = mrpt::gui::MRPTImage2wxImage( *imgShow );
                imgShow->unload();  // for delayed-loaded rawlogs, save lots of memory!

                wxWindowDC  dc( pnLeft );
                wxMemoryDC  tmpDc;
                tmpDc.SelectObjectAsSource(wxBitmap( *wxIMG ));
                dc.Blit(0,0,wxIMG->GetWidth(), wxIMG->GetHeight(), &tmpDc, 0, 0);
                delete wxIMG;

                lbCam1->SetLabel( _U( format( "%s - Intensity", obs3D->sensorLabel.c_str()).c_str() ));

				// save:
				displayedImgs[ 0 ] = obs3D;

                doDelay= true;
            }
        }
	}

    return doDelay;

	WX_END_TRY
	return false;
}

void CFormPlayVideo::OnprogressBarCmdScrollChanged(wxScrollEvent& event)
{
	int idx = progressBar->GetValue();
	m_idxInRawlog = idx;
	if (idx>0 && idx<(int)rawlog.size())
	{
		if (rawlog.getType(idx)==CRawlog::etSensoryFrame)
		{
			size_t dummy = 0;

			CSensoryFramePtr sf = rawlog.getAsObservations(idx);
			showSensoryFrame(sf.pointer(), dummy);
			edIndex->SetValue(idx);
		}
		else if (rawlog.getType(idx)==CRawlog::etObservation)
		{
			size_t dummy = 0;

			CObservationPtr o = rawlog.getAsObservation(idx);

			CSensoryFrame sf;
			sf.insert(o);
			showSensoryFrame( &sf, dummy);

			edIndex->SetValue(idx);
		}
	}
}

void CFormPlayVideo::saveCamImage(int n)
{
	WX_START_TRY

	if (n<0 || n>=(int)displayedImgs.size() || !displayedImgs[n]) return;

	wxString caption = wxT("Save image...");
	wxString wildcard = wxT("Image files (*.png,*.jpg,*.bmp)|*.jpg;*.bmp;*.png|All files (*.*)|*.*");
	wxString defaultDir( _U( iniFile->read_string(iniFileSect,"LastDir",".").c_str() ) );


	if (IS_CLASS(displayedImgs[n],CObservationImage))
	{
		CObservationImagePtr o = CObservationImagePtr( displayedImgs[n]);

		wxString defaultFilename = _U( format( "%s_%i.jpg",o->sensorLabel.c_str(),m_idxInRawlog ).c_str() );
		wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

		if (dialog.ShowModal() != wxID_OK) return;

		string fil = string( dialog.GetPath().mb_str() );

		o->image.saveToFile( fil );
	}
	else if (IS_CLASS(displayedImgs[n],CObservationStereoImages))
	{
		CObservationStereoImagesPtr o = CObservationStereoImagesPtr( displayedImgs[n]);

		wxString defaultFilename;
		switch(n)
		{
			case 0: defaultFilename = _U( format( "%s_left_%i.jpg",o->sensorLabel.c_str(),m_idxInRawlog ).c_str() ); break;
			case 1: defaultFilename = _U( format( "%s_right_%i.jpg",o->sensorLabel.c_str(),m_idxInRawlog ).c_str() ); break;
			case 2: defaultFilename = _U( format( "%s_disp_%i.jpg",o->sensorLabel.c_str(),m_idxInRawlog ).c_str() ); break;
		}

		wxFileDialog dialog(this, caption, defaultDir, defaultFilename,wildcard, wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

		if (dialog.ShowModal() != wxID_OK) return;

		string fil = string( dialog.GetPath().mb_str() );

		CImage &im = (n==2 ? o->imageDisparity : (n==1 ? o->imageRight : o->imageLeft));
		im.saveToFile( fil );
	}

	WX_END_TRY
}

void CFormPlayVideo::OnbtnSaveCam1Click(wxCommandEvent& event)
{
	saveCamImage(0);
}
void CFormPlayVideo::OnbtnSaveCam2Click(wxCommandEvent& event)
{
	saveCamImage(1);
}
void CFormPlayVideo::OnbtnSaveCam3Click(wxCommandEvent& event)
{
	saveCamImage(2);
}

void CFormPlayVideo::OncbImageDirsSelect(wxCommandEvent& event)
{
	wxString dir = cbImageDirs->GetValue();
	string 	 dirc = string( dir.mb_str() );

	if ( mrpt::system::fileExists(dirc) )
	{
		CImage::IMAGES_PATH_BASE = dirc;
	}
	else
	{
		wxMessageBox( _("The directory:\n")+dir+_("does not exist.") , _("External images"));
	}

}
