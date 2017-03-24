/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "CScanAnimation.h"

//(*InternalHeaders(CScanAnimation)
#include <wx/intl.h>
#include <wx/string.h>
//*)

#include <wx/app.h>
#include <wx/log.h>
#include <wx/msgdlg.h>
#include <wx/progdlg.h>
#include <wx/busyinfo.h>

#include "xRawLogViewerMain.h"

//(*IdInit(CScanAnimation)
const long CScanAnimation::ID_RADIOBUTTON1 = wxNewId();
const long CScanAnimation::ID_RADIOBUTTON2 = wxNewId();
const long CScanAnimation::ID_STATICTEXT22 = wxNewId();
const long CScanAnimation::ID_TEXTCTRL11 = wxNewId();
const long CScanAnimation::ID_BUTTON5 = wxNewId();
const long CScanAnimation::ID_BUTTON1 = wxNewId();
const long CScanAnimation::ID_BUTTON2 = wxNewId();
const long CScanAnimation::ID_STATICTEXT4 = wxNewId();
const long CScanAnimation::ID_SPINCTRL2 = wxNewId();
const long CScanAnimation::ID_CHECKBOX1 = wxNewId();
const long CScanAnimation::ID_BUTTON3 = wxNewId();
const long CScanAnimation::ID_XY_GLCANVAS = wxNewId();
const long CScanAnimation::ID_SLIDER1 = wxNewId();
const long CScanAnimation::ID_STATICTEXT1 = wxNewId();
const long CScanAnimation::ID_SPINCTRL1 = wxNewId();
const long CScanAnimation::ID_BUTTON4 = wxNewId();
const long CScanAnimation::ID_STATICTEXT2 = wxNewId();
const long CScanAnimation::ID_STATICTEXT3 = wxNewId();
//*)

BEGIN_EVENT_TABLE(CScanAnimation,wxDialog)
	//(*EventTable(CScanAnimation)
	//*)
END_EVENT_TABLE()

#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CPlanarLaserScan.h> // in library mrpt-maps

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;


CScanAnimation::CScanAnimation(wxWindow* parent,wxWindowID id,const wxPoint& pos,const wxSize& size)
{
	//(*Initialize(CScanAnimation)
	wxFlexGridSizer* FlexGridSizer4;
	wxFlexGridSizer* FlexGridSizer3;
	wxFlexGridSizer* FlexGridSizer2;
	wxFlexGridSizer* FlexGridSizer7;
	wxFlexGridSizer* FlexGridSizer6;
	wxFlexGridSizer* FlexGridSizer1;
	
	Create(parent, wxID_ANY, _("Animate laser scans"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE|wxRESIZE_BORDER, _T("wxID_ANY"));
	FlexGridSizer1 = new wxFlexGridSizer(4, 1, 0, 0);
	FlexGridSizer1->AddGrowableCol(0);
	FlexGridSizer1->AddGrowableRow(2);
	StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Get data from:"));
	BoxSizer4 = new wxBoxSizer(wxVERTICAL);
	BoxSizer5 = new wxBoxSizer(wxHORIZONTAL);
	FlexGridSizer8 = new wxFlexGridSizer(2, 4, 0, 0);
	FlexGridSizer8->AddGrowableCol(2);
	rbLoaded = new wxRadioButton(this, ID_RADIOBUTTON1, _("Currently loaded rawlog"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_RADIOBUTTON1"));
	rbLoaded->SetValue(true);
	FlexGridSizer8->Add(rbLoaded, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer8->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer8->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer8->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	rbFile = new wxRadioButton(this, ID_RADIOBUTTON2, _("Rawlog in file:"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_RADIOBUTTON2"));
	FlexGridSizer8->Add(rbFile, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
	StaticText22 = new wxStaticText(this, ID_STATICTEXT22, _("Input file:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT22"));
	FlexGridSizer8->Add(StaticText22, 1, wxALL|wxALIGN_RIGHT|wxALIGN_CENTER_VERTICAL, 5);
	edFile = new wxTextCtrl(this, ID_TEXTCTRL11, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TEXTCTRL11"));
	FlexGridSizer8->Add(edFile, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	btnPickInput = new wxButton(this, ID_BUTTON5, _("Select..."), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON5"));
	FlexGridSizer8->Add(btnPickInput, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 0);
	BoxSizer5->Add(FlexGridSizer8, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	BoxSizer4->Add(BoxSizer5, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	StaticBoxSizer1->Add(BoxSizer4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer1->Add(StaticBoxSizer1, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer4 = new wxFlexGridSizer(1, 7, 0, 0);
	FlexGridSizer4->AddGrowableCol(5);
	FlexGridSizer4->AddGrowableRow(0);
	btnPlay = new wxButton(this, ID_BUTTON1, _("Start"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
	btnPlay->SetDefault();
	FlexGridSizer4->Add(btnPlay, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	btnStop = new wxButton(this, ID_BUTTON2, _("Stop"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
	btnStop->Disable();
	FlexGridSizer4->Add(btnStop, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	StaticText2 = new wxStaticText(this, ID_STATICTEXT4, _("Animation delay (ms):"), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT4"));
	FlexGridSizer4->Add(StaticText2, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	edDelay = new wxSpinCtrl(this, ID_SPINCTRL2, _T("5"), wxDefaultPosition, wxDefaultSize, 0, 0, 1000, 5, _T("ID_SPINCTRL2"));
	edDelay->SetValue(_T("5"));
	FlexGridSizer4->Add(edDelay, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	cbAllowMix = new wxCheckBox(this, ID_CHECKBOX1, _("Enable mixing of diff. lasers"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX1"));
	cbAllowMix->SetValue(true);
	FlexGridSizer4->Add(cbAllowMix, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer4->Add(-1,-1,1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	btnClose = new wxButton(this, ID_BUTTON3, _("Close"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON3"));
	FlexGridSizer4->Add(btnClose, 1, wxALL|wxALIGN_BOTTOM|wxALIGN_CENTER_HORIZONTAL, 5);
	FlexGridSizer1->Add(FlexGridSizer4, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer2 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer2->AddGrowableCol(0);
	FlexGridSizer2->AddGrowableRow(0);
	m_plot3D = new CMyGLCanvas(this,ID_XY_GLCANVAS,wxDefaultPosition,wxDefaultSize,wxTAB_TRAVERSAL,_T("ID_XY_GLCANVAS"));
	FlexGridSizer2->Add(m_plot3D, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer1->Add(FlexGridSizer2, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer3 = new wxFlexGridSizer(2, 1, 0, 0);
	FlexGridSizer3->AddGrowableCol(0);
	FlexGridSizer6 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer6->AddGrowableCol(0);
	slPos = new wxSlider(this, ID_SLIDER1, 0, 0, 100, wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_SLIDER1"));
	FlexGridSizer6->Add(slPos, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 5);
	FlexGridSizer3->Add(FlexGridSizer6, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer7 = new wxFlexGridSizer(2, 4, 0, 0);
	FlexGridSizer7->AddGrowableCol(3);
	StaticText1 = new wxStaticText(this, ID_STATICTEXT1, _("Rawlog index:"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT1"));
	FlexGridSizer7->Add(StaticText1, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	edIndex = new wxSpinCtrl(this, ID_SPINCTRL1, _T("0"), wxDefaultPosition, wxDefaultSize, 0, 0, 100, 0, _T("ID_SPINCTRL1"));
	edIndex->SetValue(_T("0"));
	FlexGridSizer7->Add(edIndex, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	btnJump = new wxButton(this, ID_BUTTON4, _("Jump"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON4"));
	FlexGridSizer7->Add(btnJump, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	lbNumScans = new wxStaticText(this, ID_STATICTEXT2, _("Number of laser scans: 0"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT2"));
	FlexGridSizer7->Add(lbNumScans, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer7->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer7->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer7->Add(-1,-1,1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
	lbNumPoints = new wxStaticText(this, ID_STATICTEXT3, _("Number of points: 0"), wxDefaultPosition, wxDefaultSize, 0, _T("ID_STATICTEXT3"));
	FlexGridSizer7->Add(lbNumPoints, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer3->Add(FlexGridSizer7, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	FlexGridSizer1->Add(FlexGridSizer3, 1, wxALL|wxEXPAND|wxALIGN_LEFT|wxALIGN_TOP, 0);
	SetSizer(FlexGridSizer1);
	FlexGridSizer1->Fit(this);
	FlexGridSizer1->SetSizeHints(this);
	Center();
	
	Connect(ID_RADIOBUTTON1,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&CScanAnimation::OnrbLoadedSelect);
	Connect(ID_RADIOBUTTON2,wxEVT_COMMAND_RADIOBUTTON_SELECTED,(wxObjectEventFunction)&CScanAnimation::OnrbFile);
	Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CScanAnimation::OnbtnPickInputClick);
	Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CScanAnimation::OnbtnPlayClick);
	Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CScanAnimation::OnbtnStopClick);
	Connect(ID_CHECKBOX1,wxEVT_COMMAND_CHECKBOX_CLICKED,(wxObjectEventFunction)&CScanAnimation::OncbAllowMixClick);
	Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CScanAnimation::OnbtnCloseClick);
	Connect(ID_SLIDER1,wxEVT_SCROLL_TOP|wxEVT_SCROLL_BOTTOM|wxEVT_SCROLL_LINEUP|wxEVT_SCROLL_LINEDOWN|wxEVT_SCROLL_PAGEUP|wxEVT_SCROLL_PAGEDOWN|wxEVT_SCROLL_THUMBTRACK|wxEVT_SCROLL_THUMBRELEASE|wxEVT_SCROLL_CHANGED,(wxObjectEventFunction)&CScanAnimation::OnslPosCmdScrollChanged);
	Connect(ID_SLIDER1,wxEVT_SCROLL_THUMBTRACK,(wxObjectEventFunction)&CScanAnimation::OnslPosCmdScrollChanged);
	Connect(ID_SLIDER1,wxEVT_SCROLL_CHANGED,(wxObjectEventFunction)&CScanAnimation::OnslPosCmdScrollChanged);
	Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CScanAnimation::OnbtnJumpClick);
	Connect(wxID_ANY,wxEVT_INIT_DIALOG,(wxObjectEventFunction)&CScanAnimation::OnInit);
	//*)

	// Initialize 3D view:
	m_plot3D->m_openGLScene->insert( mrpt::opengl::CGridPlaneXY::Create(-50,50, -50,50, 0 /* z */, 5 /* freq */) );
	m_plot3D->m_openGLScene->insert( mrpt::opengl::stock_objects::CornerXYZSimple(1.0 /*scale*/, 3.0 /*line width*/) );

	m_mixlasers = cbAllowMix->GetValue();

}

CScanAnimation::~CScanAnimation()
{
	//(*Destroy(CScanAnimation)
	//*)
}

// Get the rawlog entry (from cur. loaded rawlog), build and displays its map:
void CScanAnimation::RebuildMaps()
{
	WX_START_TRY

	int  idx = slPos->GetValue();

	if ( idx>=0 && idx<(int)rawlog.size() )
	{
		if (rawlog.getType(idx)==CRawlog::etSensoryFrame)
		{
			CSensoryFramePtr sf = rawlog.getAsObservations(idx);
			BuildMapAndRefresh(sf.pointer());
		}
		else
		if (rawlog.getType(idx)==CRawlog::etObservation)
		{
			CSensoryFramePtr sf = CSensoryFrame::Create();
			sf->insert( rawlog.getAsObservation(idx) );
			BuildMapAndRefresh(sf.pointer());
		}
	}

	WX_END_TRY
}


// This method is called in any case for displaying a laser scan.
//  We keep an internal list of recent scans so they don't vanish instantaneously.
void CScanAnimation::BuildMapAndRefresh(CSensoryFrame *sf)
{
	WX_START_TRY

	// Preprocess: make sure 3D observations are ready:
	std::vector<CObservation3DRangeScanPtr> obs3D_to_clear;
	for (CSensoryFrame::iterator it=sf->begin();it!=sf->end();++it)
	{
		(*it)->load();
		// force generate 3D point clouds:
		if (IS_CLASS(*it, CObservation3DRangeScan))
		{
			CObservation3DRangeScanPtr o= CObservation3DRangeScanPtr(*it);
			if (o->hasRangeImage && !o->hasPoints3D)
			{
				mrpt::obs::T3DPointsProjectionParams pp;
				pp.takeIntoAccountSensorPoseOnRobot = false,
				o->project3DPointsFromDepthImageInto(*o, pp);
				obs3D_to_clear.push_back(o);
			}
		}
	}

	// Mix?
	if (!m_mixlasers)
	{
		// if not, just clear all old objects:
		for (TListGlObjects::iterator it = m_gl_objects.begin();it!=m_gl_objects.end();++it)
		{
			TRenderObject &ro = it->second;
			m_plot3D->m_openGLScene->removeObject(ro.obj);  // Remove from the opengl viewport
		}
		m_gl_objects.clear();
	}

	// Insert new scans:
	mrpt::system::TTimeStamp  	tim_last = INVALID_TIMESTAMP;
	bool						wereScans = false;
	for (CSensoryFrame::iterator it=sf->begin();it!=sf->end();++it)
	{
		const std::string sNameInMap = std::string((*it)->GetRuntimeClass()->className) + (*it)->sensorLabel;
		if (IS_CLASS(*it,CObservation2DRangeScan))
		{
			CObservation2DRangeScanPtr obs = CObservation2DRangeScanPtr(*it);
			wereScans = true;
			if (tim_last==INVALID_TIMESTAMP || tim_last<obs->timestamp)
				tim_last = obs->timestamp;

			// Already in the map with the same sensor label?
			TListGlObjects::iterator it_gl = m_gl_objects.find(sNameInMap);
			if (it_gl!=m_gl_objects.end())
			{
				// Update existing object:
				TRenderObject &ro = it_gl->second;
				CPlanarLaserScanPtr(ro.obj)->setScan(*obs);
				ro.timestamp = obs->timestamp;
			}
			else
			{
				// Create object:
				CPlanarLaserScanPtr gl_obj = CPlanarLaserScan::Create();
				gl_obj->setScan(*obs);

				TRenderObject ro;
				ro.obj = gl_obj;
				ro.timestamp = obs->timestamp;
				m_gl_objects[sNameInMap]=ro;
				m_plot3D->m_openGLScene->insert( gl_obj );
			}
		}
		else
		if (IS_CLASS(*it,CObservation3DRangeScan))
		{
			CObservation3DRangeScanPtr obs = CObservation3DRangeScanPtr(*it);
			wereScans = true;
			if (tim_last==INVALID_TIMESTAMP || tim_last<obs->timestamp)
				tim_last = obs->timestamp;

			CColouredPointsMap  pointMap;
			pointMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
			pointMap.insertionOptions.minDistBetweenLaserPoints = 0;

			pointMap.insertObservation( obs.pointer() );

			// Already in the map with the same sensor label?
			TListGlObjects::iterator it_gl = m_gl_objects.find(sNameInMap);
			if (it_gl!=m_gl_objects.end())
			{
				// Update existing object:
				TRenderObject &ro = it_gl->second;
				CPointCloudColouredPtr gl_obj = CPointCloudColouredPtr(ro.obj);
				gl_obj->loadFromPointsMap(&pointMap);
				ro.timestamp = obs->timestamp;
			}
			else
			{
				// Create object:
				CPointCloudColouredPtr gl_obj = CPointCloudColoured::Create();
				gl_obj->setPointSize(3.0);
				gl_obj->loadFromPointsMap(&pointMap);

				TRenderObject ro;
				ro.obj = gl_obj;
				ro.timestamp = obs->timestamp;
				m_gl_objects[sNameInMap]=ro;
				m_plot3D->m_openGLScene->insert( gl_obj );
			}
			// Add to list:
//				m_lstScans[obs->sensorLabel] = obs;
		}
		else
		if (IS_CLASS(*it,CObservationVelodyneScan))
		{
			CObservationVelodyneScanPtr obs = CObservationVelodyneScanPtr(*it);
			wereScans = true;
			if (tim_last==INVALID_TIMESTAMP || tim_last<obs->timestamp)
				tim_last = obs->timestamp;

			obs->generatePointCloud();
			CColouredPointsMap pointMap;
			pointMap.loadFromVelodyneScan(*obs);
			obs->point_cloud.clear_deep();
			
			// Already in the map with the same sensor label?
			TListGlObjects::iterator it_gl = m_gl_objects.find(sNameInMap);
			if (it_gl!=m_gl_objects.end())
			{
				// Update existing object:
				TRenderObject &ro = it_gl->second;
				CPointCloudColouredPtr gl_obj = CPointCloudColouredPtr(ro.obj);
				gl_obj->loadFromPointsMap(&pointMap);
				ro.timestamp = obs->timestamp;
			}
			else
			{
				// Create object:
				CPointCloudColouredPtr gl_obj = CPointCloudColoured::Create();
				gl_obj->setPointSize(3.0);
				gl_obj->loadFromPointsMap(&pointMap);

				TRenderObject ro;
				ro.obj = gl_obj;
				ro.timestamp = obs->timestamp;
				m_gl_objects[sNameInMap]=ro;
				m_plot3D->m_openGLScene->insert( gl_obj );
			}
		}
	}

	// Check what observations are too old and must be deleted:
	const double largest_period = 0.2;
	vector_string lst_to_delete;
	for (TListGlObjects::iterator it=m_gl_objects.begin();it!=m_gl_objects.end();++it)
	{
		TRenderObject &ro = it->second;

		if ((tim_last==INVALID_TIMESTAMP && wereScans)  // Scans without timestamps
			||
			(tim_last!=INVALID_TIMESTAMP && fabs(mrpt::system::timeDifference( ro.timestamp, tim_last)) > largest_period ))
		{
			lst_to_delete.push_back(it->first);
		}
	}

	// Remove too old observations:
	for (vector_string::iterator s=lst_to_delete.begin();s!=lst_to_delete.end();++s)
	{
		TRenderObject &ro = m_gl_objects[*s];

		m_plot3D->m_openGLScene->removeObject(ro.obj);  // Remove from the opengl viewport
		m_gl_objects.erase(*s); // and from my list
	}

	// Force refresh view:
	m_plot3D->Refresh();

	// Post-process: unload 3D observations.
	for (CSensoryFrame::iterator it=sf->begin();it!=sf->end();++it)
		(*it)->unload();
	for (size_t i=0;i<obs3D_to_clear.size();i++)
	{
		obs3D_to_clear[i]->resizePoints3DVectors(0);
		obs3D_to_clear[i]->hasPoints3D = false;
	}


	WX_END_TRY
}

void CScanAnimation::OnbtnPlayClick(wxCommandEvent& event)
{
	//


	// Disable all but stop while playing:
	btnStop->Enable();
	btnPlay->Disable();
	btnClose->Disable();

	slPos->Disable();

	m_stop=false;

	try
	{
		wxBusyCursor	cursorBusy;
		int 		    delay_ms = edDelay->GetValue();

		while (!m_stop)
		{
			int idx = slPos->GetValue();
			if (idx>=((int)rawlog.size())-1) break;	// End!

			if (rawlog.getType(idx) != CRawlog::etActionCollection )
			{
				RebuildMaps();
				::wxMilliSleep( delay_ms );
			}

			idx++;
			slPos->SetValue(idx);
			edIndex->SetValue(idx);
			wxTheApp->Yield();
		}
	}
	catch(...)
	{
	}

	btnStop->Disable();
	btnPlay->Enable();
	btnClose->Enable();

	slPos->Enable();
}

void CScanAnimation::OnbtnStopClick(wxCommandEvent& event)
{
	m_stop=true;
}

void CScanAnimation::OnbtnCloseClick(wxCommandEvent& event)
{
	Close();
}

void CScanAnimation::OnslPosCmdScrollChanged(wxScrollEvent& event)
{
    edIndex->SetValue( slPos->GetValue() );
	RebuildMaps();
}

void CScanAnimation::OnbtnJumpClick(wxCommandEvent& event)
{
    slPos->SetValue( edIndex->GetValue() );
	RebuildMaps();
}

void CScanAnimation::OnslPosCmdScroll(wxScrollEvent& event)
{
	RebuildMaps();
}

void CScanAnimation::OnbtnPickInputClick(wxCommandEvent& event)
{

}

void CScanAnimation::OnInit(wxInitDialogEvent& event)
{
	slPos->SetValue(0);
	slPos->SetMin(0);
	slPos->SetMax( (int)rawlog.size() );

	edIndex->SetValue(0);
	edIndex->SetRange(0, (int)rawlog.size() );

	Fit();

	wxCommandEvent dummy;

	OnrbLoadedSelect( dummy );
}


void CScanAnimation::OnrbLoadedSelect(wxCommandEvent& event)
{
    edFile->Enable(false);
    btnPickInput->Enable(false);

    slPos->Enable(true);
    edIndex->Enable(true);
    btnJump->Enable(true);
}

void CScanAnimation::OnrbFile(wxCommandEvent& event)
{
    edFile->Enable(true);
    btnPickInput->Enable(true);

    slPos->Enable(false);
    edIndex->Enable(false);
    btnJump->Enable(false);
}


void CScanAnimation::OncbAllowMixClick(wxCommandEvent& event)
{
	m_mixlasers = cbAllowMix->GetValue();
}
