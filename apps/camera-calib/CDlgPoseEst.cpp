/* +---------------------------------------------------------------------------+
|                     Mobile Robot Programming Toolkit (MRPT)               |
|                          http://www.mrpt.org/                             |
|                                                                           |
| Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
| See: http://www.mrpt.org/Authors - All rights reserved.                   |
| Released under BSD License. See details in http://www.mrpt.org/License    |
+---------------------------------------------------------------------------+ */

#include <fstream>
#include "CDlgPoseEst.h"
#include "camera_calib_guiMain.h"

#include <mrpt/vision/chessboard_find_corners.h>

//(*InternalHeaders(CDlgPoseEst)
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/intl.h>
#include <wx/font.h>
//*)
#include <mrpt/gui/wx28-fixes.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;


//(*IdInit(CDlgPoseEst)
const long CDlgPoseEst::ID_CUSTOM2 = wxNewId();
const long CDlgPoseEst::ID_STATICTEXT1 = wxNewId();
const long CDlgPoseEst::ID_SPINCTRL1 = wxNewId();
const long CDlgPoseEst::ID_STATICTEXT2 = wxNewId();
const long CDlgPoseEst::ID_SPINCTRL2 = wxNewId();
const long CDlgPoseEst::ID_RADIOBOX1 = wxNewId();
const long CDlgPoseEst::ID_STATICTEXT3 = wxNewId();
const long CDlgPoseEst::ID_TEXTCTRL1 = wxNewId();
const long CDlgPoseEst::ID_STATICTEXT4 = wxNewId();
const long CDlgPoseEst::ID_TEXTCTRL3 = wxNewId();
const long CDlgPoseEst::ID_CHECKBOX1 = wxNewId();
const long CDlgPoseEst::ID_STATICTEXT5 = wxNewId();
const long CDlgPoseEst::ID_SPINCTRL3 = wxNewId();
const long CDlgPoseEst::ID_STATICTEXT6 = wxNewId();
const long CDlgPoseEst::ID_STATICTEXT7 = wxNewId();
const long CDlgPoseEst::ID_TEXTCTRL2 = wxNewId();
const long CDlgPoseEst::ID_BUTTON1 = wxNewId();
const long CDlgPoseEst::ID_BUTTON2 = wxNewId();
const long CDlgPoseEst::ID_BUTTON3 = wxNewId();
const long CDlgPoseEst::ID_CUSTOM1 = wxNewId();
const long CDlgPoseEst::ID_TIMER1 = wxNewId();
const long CDlgPoseEst::ID_ALGOCHOICE =wxNewId();
const long CDlgPoseEst::ID_CAMPOSEVIEW =wxNewId();
const long CDlgPoseEst::ID_STATICTEXTALGO = wxNewId();
//*)

BEGIN_EVENT_TABLE(CDlgPoseEst,wxDialog)
//(*EventTable(CDlgPoseEst)
//*)
END_EVENT_TABLE()

CDlgPoseEst::CPNP_PTR pose_algos[9] = {
	&mrpt::vision::pnp::CPnP::epnp, &mrpt::vision::pnp::CPnP::dls, &mrpt::vision::pnp::CPnP::upnp, &mrpt::vision::pnp::CPnP::p3p, &mrpt::vision::pnp::CPnP::lhm, &mrpt::vision::pnp::CPnP::posit, &mrpt::vision::pnp::CPnP::ppnp, &mrpt::vision::pnp::CPnP::rpnp, &mrpt::vision::pnp::CPnP::so3 }
;

CDlgPoseEst::CDlgPoseEst(wxWindow* parent,wxWindowID id,const wxPoint& pos,const wxSize& size)
{
	m_threadMustClose = false;
	m_threadResultsComputed=false;
	m_threadIsClosed = true;


	//(*Initialize(CDlgPoseEst)
	wxStaticBoxSizer* StaticBoxSizer2;
	wxFlexGridSizer* FlexGridSizer2;
	wxStaticBoxSizer* StaticBoxSizer5;
	wxFlexGridSizer* FlexGridSizer17;
	wxFlexGridSizer* FlexGridSizer7;
	wxFlexGridSizer* FlexGridSizer4;
	wxStaticBoxSizer* StaticBoxSizer3;
	wxFlexGridSizer* FlexGridSizer6;
	wxFlexGridSizer* FlexGridSizer3;
	wxStaticBoxSizer* StaticBoxSizer4;
	wxFlexGridSizer* FlexGridSizer18;
	wxFlexGridSizer* FlexGridSizer5;
	wxStaticBoxSizer* StaticBoxSizer1;

	Create(parent, id, _("Grab chessboard calibration pattern"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE, _T("id"));
	SetClientSize(wxDefaultSize);
	Move(wxDefaultPosition);
	FlexGridSizer1 = new wxFlexGridSizer(0, 3, 0, 0);
	FlexGridSizer1->AddGrowableCol(0);
	FlexGridSizer1->AddGrowableRow(0);
	FlexGridSizer2 = new wxFlexGridSizer(0, 1, 0, 0);
	FlexGridSizer4 = new wxFlexGridSizer(0, 1, 0, 0);
	StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Select your camera: "));
	m_panelCamera = new mrpt::gui::CPanelCameraSelection(this,ID_CUSTOM2);
	StaticBoxSizer1->Add(m_panelCamera, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer4->Add(StaticBoxSizer1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer2->Add(FlexGridSizer4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	StaticBoxSizer3 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Checkerboard detection parameters"));
	FlexGridSizer6 = new wxFlexGridSizer(2, 2, 0, 0);
	FlexGridSizer6->AddGrowableCol(0);
	FlexGridSizer6->AddGrowableCol(1);
	StaticBoxSizer4 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Number of inner corners: "));
	FlexGridSizer17 = new wxFlexGridSizer(1, 4, 0, 0);
	StaticText1 = new wxStaticText(this, ID_STATICTEXT1, _("In X:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
	FlexGridSizer17->Add(StaticText1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	edSizeX = new wxSpinCtrl(this, ID_SPINCTRL1, _T("5"), wxDefaultPosition, wxSize(50,-1), 0, 1, 200, 5, _T("ID_SPINCTRL1"));
	edSizeX->SetValue(_T("9"));
	FlexGridSizer17->Add(edSizeX, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText2 = new wxStaticText(this, ID_STATICTEXT2, _("In Y:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
	FlexGridSizer17->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	edSizeY = new wxSpinCtrl(this, ID_SPINCTRL2, _T("8"), wxDefaultPosition, wxSize(50,-1), 0, 1, 200, 8, _T("ID_SPINCTRL2"));
	edSizeY->SetValue(_T("7"));
	FlexGridSizer17->Add(edSizeY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticBoxSizer4->Add(FlexGridSizer17, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer6->Add(StaticBoxSizer4, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 2);
	wxString __wxRadioBoxChoices_1[2] =
	{
		_("OpenCV\'s default"),
		_("Scaramuzza et al.\'s")
	};
	rbMethod = new wxRadioBox(this, ID_RADIOBOX1, _(" Detector method: "), wxDefaultPosition, wxDefaultSize, 2, __wxRadioBoxChoices_1, 1, 0, wxDefaultValidator, _T("ID_RADIOBOX1"));
	FlexGridSizer6->Add(rbMethod, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 2);
	StaticBoxSizer5 = new wxStaticBoxSizer(wxHORIZONTAL, this, _(" Size of quads (in mm): "));
	FlexGridSizer18 = new wxFlexGridSizer(1, 4, 0, 0);
	StaticText3 = new wxStaticText(this, ID_STATICTEXT3, _("In X:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
	FlexGridSizer18->Add(StaticText3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	edLengthX = new wxTextCtrl(this, ID_TEXTCTRL1, _("40.0"), wxDefaultPosition, wxSize(40,-1), 0, wxDefaultValidator, _T("ID_TEXTCTRL1"));
	FlexGridSizer18->Add(edLengthX, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText4 = new wxStaticText(this, ID_STATICTEXT4, _("In Y:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT4"));
	FlexGridSizer18->Add(StaticText4, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	edLengthY = new wxTextCtrl(this, ID_TEXTCTRL3, _("40.0"), wxDefaultPosition, wxSize(40,-1), 0, wxDefaultValidator, _T("ID_TEXTCTRL3"));
	FlexGridSizer18->Add(edLengthY, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticBoxSizer5->Add(FlexGridSizer18, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer6->Add(StaticBoxSizer5, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 2);
	cbNormalize = new wxCheckBox(this, ID_CHECKBOX1, _("Normalize image"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
	cbNormalize->SetValue(true);
	FlexGridSizer6->Add(cbNormalize, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticBoxSizer3->Add(FlexGridSizer6, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer2->Add(StaticBoxSizer3, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticBoxSizer2 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Real Time Pose Estimation"));
	FlexGridSizer7 = new wxFlexGridSizer(2, 2, 0, 0);
	FlexGridSizer7->AddGrowableCol(0);
	FlexGridSizer7->AddGrowableRow(0);
	StaticTextAlgo = new wxStaticText(this, ID_STATICTEXTALGO, _("Choose Algorithm"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXTALGO"));
	FlexGridSizer7->Add(StaticTextAlgo, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
	const wxString ch_names[] = { wxT("epnp"), wxT("dls"), wxT("upnp"), wxT("p3p"), wxT("lhm"), wxT("posit"), wxT("ppnp"), wxT("rpnp"), wxT("so3")};
	wxArrayString ch_wx = wxArrayString(9, ch_names);
	wxString ch_default(wxT("epnp"));
	pnpSelect = new wxChoice(this, ID_ALGOCHOICE, wxDefaultPosition, wxDefaultSize, ch_wx, 0, wxDefaultValidator, ch_default );
	FlexGridSizer7->Add(pnpSelect, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
	pnpSelect->SetSelection(0);
	m_3Dview_cam = new CMyGLCanvas(this,ID_CAMPOSEVIEW,wxDefaultPosition,wxSize(300,300),wxTAB_TRAVERSAL,_T("ID_CAMPOSEVIEW"));
	FlexGridSizer7->Add(m_3Dview_cam, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);

	StaticBoxSizer2->Add(FlexGridSizer7, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer2->Add(StaticBoxSizer2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	txtLog = new wxTextCtrl(this, ID_TEXTCTRL2, _("(debug output)"), wxDefaultPosition, wxSize(239,78), wxTE_MULTILINE|wxTE_READONLY|wxHSCROLL|wxVSCROLL, wxDefaultValidator, _T("ID_TEXTCTRL2"));
	wxFont txtLogFont = wxSystemSettings::GetFont(wxSYS_OEM_FIXED_FONT);
	if ( !txtLogFont.Ok() ) txtLogFont = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
	txtLogFont.SetPointSize(12);
	txtLog->SetFont(txtLogFont);
	FlexGridSizer2->Add(txtLog, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer1->Add(FlexGridSizer2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer3 = new wxFlexGridSizer(2, 1, 0, 0);
	FlexGridSizer3->AddGrowableCol(0);
	FlexGridSizer3->AddGrowableRow(1);
	FlexGridSizer5 = new wxFlexGridSizer(0, 3, 0, 0);
	FlexGridSizer5->AddGrowableCol(2);
	btnStart = new wxButton(this, ID_BUTTON1, _("Start"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
	wxFont btnStartFont(wxDEFAULT,wxDEFAULT,wxFONTSTYLE_NORMAL,wxFONTWEIGHT_BOLD,false,wxEmptyString,wxFONTENCODING_DEFAULT);
	btnStart->SetFont(btnStartFont);
	FlexGridSizer5->Add(btnStart, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	btnStop = new wxButton(this, ID_BUTTON2, _("Stop"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
	btnStop->Disable();
	FlexGridSizer5->Add(btnStop, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	btnClose = new wxButton(this, ID_BUTTON3, _("Close"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON3"));
	FlexGridSizer5->Add(btnClose, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer3->Add(FlexGridSizer5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	m_realtimeview = new mrpt::gui::wxMRPTImageControl(this,ID_CUSTOM1,wxDefaultPosition.x,wxDefaultPosition.y,wxSize(468,286).GetWidth(), wxSize(468,286).GetHeight() );
	FlexGridSizer3->Add(m_realtimeview, 1, wxALL|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer1->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	SetSizer(FlexGridSizer1);
	timCapture.SetOwner(this, ID_TIMER1);
	FlexGridSizer1->Fit(this);
	FlexGridSizer1->SetSizeHints(this);
	Center();

	Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CDlgPoseEst::OnbtnStartClick);
	Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CDlgPoseEst::OnbtnStopClick);
	Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CDlgPoseEst::OnbtnCloseClick);
	Connect(ID_TIMER1,wxEVT_TIMER,(wxObjectEventFunction)&CDlgPoseEst::OntimCaptureTrigger);
	cam_intrinsic = Eigen::MatrixXd::Zero(3,3);
	I3 = Eigen::MatrixXd::Identity(3,3);
	pose_mat = Eigen::MatrixXd::Zero(6,1);
	pose_mat << -25, 25, 100, -0.1, 0.25, 0.5;
	//*)

	scene = mrpt::opengl::COpenGLScene::Create();
	cor   = mrpt::opengl::stock_objects::CornerXYZ();
	cor1  = mrpt::opengl::stock_objects::CornerXYZ();

	const unsigned int  check_size_x = edSizeX->GetValue();
	const unsigned int  check_size_y = edSizeY->GetValue();
	const double        check_squares_length_X_meters = 0.005 * atof( string(edLengthX->GetValue().mb_str()).c_str() );
	const double        check_squares_length_Y_meters = 0.005 * atof( string(edLengthY->GetValue().mb_str()).c_str() );

	grid  = opengl::CGridPlaneXY::Create(0,check_size_y*check_squares_length_Y_meters, 0, check_size_x*check_squares_length_X_meters, 0, check_squares_length_Y_meters );
	scene->insert( grid );

	mrpt::poses::CPose3D pose1;
	cor->setName("Global Frame");
	cor->enableShowName(true);
	cor->setScale(0.5);
	cor->setPose(pose1);
	scene->insert( cor );

	this->showCamPose();
	redire = new CMyRedirector(txtLog, true, 10);

}

CDlgPoseEst::~CDlgPoseEst()
{
	delete redire;
	//(*Destroy(CDlgPoseEst)
	//*)
}


void CDlgPoseEst::OnbtnCloseClick(wxCommandEvent& event)
{
	wxCommandEvent ev;
	OnbtnStopClick(ev);

	EndModal(wxID_CANCEL);
}

void CDlgPoseEst::OnbtnStartClick(wxCommandEvent& event)
{

	ifstream file("intrinsic_matrix.txt");

	if(file.is_open())
	{
		double arr[9];

		for(int i=0; i<9 ; i++)
		file>>arr[i];

		for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
		cam_intrinsic(i,j) = arr[3*i+j];

		file.close();

	}
	else
	{
		int answer = wxMessageBox(wxT("Do calibration first and save camera parameters as intrinsic_matrix.txt"), wxT("Quit?"), wxYES_NO | wxCANCEL);

		if (answer == wxYES)
		EndModal(wxID_CANCEL);
		else
		return;
	}

	// Try to open the camera:
	if (m_video)
	m_video.clear();

	m_video = mrpt::hwdrivers::prepareVideoSourceFromPanel(m_panelCamera);
	if (!m_video) return;

	// Launch thread:
	// --------------------------
	m_threadImgToProcess.clear();
	m_threadMustClose = false;
	m_threadResults.clear();
	m_threadResultsComputed=true;  // To start a new detection
	m_threadIsClosed = false;

	m_calibFrames.clear();

	m_threadCorners = mrpt::system::createThreadFromObjectMethod( this, &CDlgPoseEst::threadProcessCorners );

	//lbProgress->SetLabel(_("0"));

	btnStart->Disable();
	btnStop->Enable();
	this->m_panelCamera->Disable();

	// start processing:
	timCapture.Start(2,true); // One shot

}

void CDlgPoseEst::OnbtnStopClick(wxCommandEvent& event)
{
	m_video.clear();

	btnStart->Enable();
	btnStop->Disable();
	this->m_panelCamera->Enable();

	m_threadMustClose = true;
	mrpt::system::joinThread( m_threadCorners );
}

void CDlgPoseEst::OntimCaptureTrigger(wxTimerEvent& event)
{
	static mrpt::system::TTimeStamp  last_valid = INVALID_TIMESTAMP;

	try
	{
		if (!btnStop->IsEnabled())
		{
			timCapture.Stop();
			return;
		}

		ASSERT_(m_video)

		m_check_size_x = this->edSizeX->GetValue();
		m_check_size_y = this->edSizeY->GetValue();

		m_normalize_image = this->cbNormalize->GetValue();
		m_useScaramuzzaAlternativeDetector = this->rbMethod->GetSelection() == 1;

		CObservationPtr obs = m_video->getNextFrame();
		ASSERT_(obs)
		ASSERT_(IS_CLASS(obs,CObservationImage) || IS_CLASS(obs,CObservation3DRangeScan) )

		// Convert to an image:
		if (IS_CLASS(obs,CObservation3DRangeScan))
		{
			CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);

			CObservationImagePtr obsImg = CObservationImage::Create();
			obsImg->timestamp = obs3D->timestamp;
			ASSERT_(obs3D->hasIntensityImage)
			obsImg->image = obs3D->intensityImage;

			// Ale hoop!
			obs = obsImg;
		}

		CImage  img_to_show;

		// get the observation:
		CObservationImagePtr  obs_img = CObservationImagePtr(obs);

		bool blankTime = (last_valid != INVALID_TIMESTAMP) && mrpt::system::timeDifference(last_valid,mrpt::system::now())<0.5;
		if (!blankTime && m_threadResultsComputed && !m_threadResults.empty() )
		{
			// Update last valid
			last_valid=mrpt::system::now();

			// Display now:
			m_threadImgToProcess->image.colorImage(img_to_show);

			// Draw the corners:
			img_to_show.drawChessboardCorners(m_threadResults,m_check_size_x, m_check_size_y);

			// Set flag to process pose estimation
			flag_pose_est=true;
		}
		else
		{
			// Show current image:
			img_to_show = obs_img->image;
		}

		// Process a new image to detect checkerboard:
		if (m_threadResultsComputed)
		{
			m_threadImgToProcess = obs_img;
			m_threadResultsComputed = false;
			//cout << "new image sent"<<endl;
		}

		m_realtimeview->AssignImage(img_to_show);
		m_realtimeview->Refresh(false);
		wxTheApp->Yield();


		// Resize the display area, if needed:
		if (std::abs( (int)(this->m_realtimeview->GetClientSize().GetWidth()) - int(img_to_show.getWidth()) )>30 )
		{
			this->m_realtimeview->SetSize( img_to_show.getWidth(), img_to_show.getHeight() );
			this->m_realtimeview->SetMinSize( wxSize(img_to_show.getWidth(), img_to_show.getHeight()) );
			this->FlexGridSizer1->RecalcSizes();
			this->Fit();
		}
		// Get on...
		timCapture.Start(5,true);
	}
	catch(std::exception &e)
	{
		try { wxCommandEvent  dum; this->OnbtnStopClick(dum); } catch(...) {}
		cerr << endl << e.what() << endl;
		wxMessageBox(_U(e.what()),_("Error"),wxICON_INFORMATION,this);
		return;
	}
}

// ---------------------------------------------------------------------------------------------------
// The thread for parallel detection of edges. This is needed since from OpenCV 1.1.0,
//  chessboard corner detection runs very slow, so it cannot be done in real-time properly:
// ---------------------------------------------------------------------------------------------------
void CDlgPoseEst::threadProcessCorners()
{
	CDlgPoseEst *obj = this;
	try
	{
		while (!obj->m_threadMustClose)
		{
			if (!obj->m_threadResultsComputed)
			{
				try
				{
					// Detect corners:
					bool foundCorners = mrpt::vision::findChessboardCorners(
						obj->m_threadImgToProcess->image,
						obj->m_threadResults, obj->m_check_size_x, obj->m_check_size_y,	obj->m_normalize_image, obj->m_useScaramuzzaAlternativeDetector );

						if (!foundCorners)
							obj->m_threadResults.clear();
						else
						{
							if(obj->flag_pose_est ==true)
							{
								const double  m_check_len_x = 0.1 * atof( string(edLengthX->GetValue().mb_str()).c_str() );
								const double  m_check_len_y = 0.1 * atof( string(edLengthY->GetValue().mb_str()).c_str() );

								obj_pts.resize(m_check_size_x*m_check_size_y,3);
								img_pts.resize(m_check_size_x*m_check_size_y,3);

								// Sort Object points in Eigen Matrix
								for(unsigned int j=0; j<m_check_size_y; j++)
										for(unsigned int i=0; i<m_check_size_x; i++)
												obj_pts.row(j*m_check_size_x+i) << (i)*m_check_len_x, (j)*m_check_len_y,  1;

								// Sort Image points in Eigen Matrix
								for(unsigned int i=0; i<m_check_size_x; i++)
									for(unsigned int j=0; j<m_check_size_y; j++)
										img_pts.row(i*m_check_size_y+j) << (obj->m_threadResults[i*m_check_size_y+j].x - cam_intrinsic(0,2))/cam_intrinsic(0,0)
										, (obj->m_threadResults[i*m_check_size_y+j].y- cam_intrinsic(1,2))/cam_intrinsic(1,1)
										, 1;

								int algo_idx = pnpSelect->GetSelection();
								(pnp_algos.*pose_algos[algo_idx])(obj_pts, img_pts, m_check_size_x*m_check_size_y, I3, pose_mat );

								obj->showCamPose();

								obj->flag_pose_est=false;
							}
						}

						obj->m_threadResultsComputed = true;
						//flog.close();
					}
					catch(std::exception &e)
					{
						cerr << e.what() << endl;
					}
					catch(...)
					{
					}
				}
				else
				{
					// Nothing to do:
					mrpt::system::sleep(5);
				}
			}

			// Signal we're done
			obj->m_threadIsClosed = true;
		}
		catch(std::exception &e)
		{
			cerr << e.what() << endl;
			// Signal we're done
			obj->m_threadIsClosed = true;
		}
		catch(...)
		{
			// Signal we're done
			obj->m_threadIsClosed = true;
		}
	}


	void CDlgPoseEst::showCamPose()
	{

		double q_norm = sqrt(1 - pose_mat(3,0)*pose_mat(3,0) -pose_mat(4,0)*pose_mat(4,0) -pose_mat(5,0)*pose_mat(5,0) );
		mrpt::poses::CPose3DQuat q_pose(0.005*pose_mat(0,0), 0.005*pose_mat(1,0),0.005*pose_mat(2,0), mrpt::math::CQuaternionDouble(q_norm, pose_mat(3,0), pose_mat(4,0), pose_mat(5,0)) );
		q_pose.inverse();

		cor1->setName("Camera Frame");
		cor1->enableShowName(true);
		cor1->setScale(0.5);
		cor1->setPose(q_pose);
		scene->insert( cor1 );

		this->m_3Dview_cam->m_openGLScene = scene;
		this->m_3Dview_cam->Refresh();
	}