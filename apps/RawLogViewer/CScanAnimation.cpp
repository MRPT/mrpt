/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#include "CScanAnimation.h"

//(*InternalHeaders(CScanAnimation)
#include <wx/intl.h>
#include <wx/string.h>
//*)

#ifdef None	 // X header conflict...
#undef None
#endif

#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPlanarLaserScan.h>  // in library mrpt-maps
#include <mrpt/opengl/stock_objects.h>
#include <wx/app.h>
#include <wx/busyinfo.h>
#include <wx/log.h>
#include <wx/msgdlg.h>
#include <wx/progdlg.h>

#include "xRawLogViewerMain.h"

extern xRawLogViewerFrame* theMainWindow;

//(*IdInit(CScanAnimation)
const long CScanAnimation::ID_LIST_OBS_LABELS = wxNewId();
const long CScanAnimation::ID_RADIOBUTTON2 = wxNewId();
const long CScanAnimation::ID_STATICTEXT22 = wxNewId();
const long CScanAnimation::ID_TEXTCTRL11 = wxNewId();
const long CScanAnimation::ID_BUTTON5 = wxNewId();
const long CScanAnimation::ID_BUTTON6 = wxNewId();
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
const long CScanAnimation::ID_CHECKBOX2 = wxNewId();
const long CScanAnimation::ID_STATICTEXT3 = wxNewId();
const long CScanAnimation::ID_BUTTON7 = wxNewId();
//*)
const long CScanAnimation::ID_BUTTON_SAVE_SCENE = wxNewId();

BEGIN_EVENT_TABLE(CScanAnimation, wxDialog)
//(*EventTable(CScanAnimation)
//*)
END_EVENT_TABLE()

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace std;

CScanAnimation::CScanAnimation(
	wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size)
{
	//(*Initialize(CScanAnimation)
	wxFlexGridSizer* FlexGridSizer4;
	wxFlexGridSizer* FlexGridSizer4a;
	wxFlexGridSizer* FlexGridSizer4b;
	wxFlexGridSizer* FlexGridSizer3;
	wxFlexGridSizer* FlexGridSizer2;
	wxFlexGridSizer* FlexGridSizer7;
	wxFlexGridSizer* FlexGridSizer6;
	wxFlexGridSizer* FlexGridSizer1;

	Create(
		parent, wxID_ANY, _("Animate laser scans"), wxDefaultPosition,
		wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER,
		_T("wxID_ANY"));
	FlexGridSizer1 = new wxFlexGridSizer(3, 1, 0, 0);
	FlexGridSizer1->AddGrowableCol(0);
	FlexGridSizer1->AddGrowableRow(1);

	FlexGridSizer4 = new wxFlexGridSizer(1, 5, 0, 0);
	FlexGridSizer4->AddGrowableCol(4);
	FlexGridSizer4->AddGrowableRow(0);

	FlexGridSizer4a = new wxFlexGridSizer(2, 3, 0, 0);
	btnPlay = new wxButton(
		this, ID_BUTTON1, _("Start"), wxDefaultPosition, wxDefaultSize, 0,
		wxDefaultValidator, _T("ID_BUTTON1"));
	btnPlay->SetDefault();
	FlexGridSizer4a->Add(
		btnPlay, 1, wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL,
		5);
	btnStop = new wxButton(
		this, ID_BUTTON2, _("Stop"), wxDefaultPosition, wxDefaultSize, 0,
		wxDefaultValidator, _T("ID_BUTTON2"));
	btnStop->Disable();
	FlexGridSizer4a->Add(
		btnStop, 1, wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL,
		5);
	btnClose = new wxButton(
		this, ID_BUTTON3, _("Close"), wxDefaultPosition, wxDefaultSize, 0,
		wxDefaultValidator, _T("ID_BUTTON3"));
	FlexGridSizer4a->Add(
		btnClose, 1, wxALL | wxALIGN_BOTTOM | wxALIGN_CENTER_HORIZONTAL, 5);

	auto btnSaveScene =
		new wxButton(this, ID_BUTTON_SAVE_SCENE, _("Save 3Dscene..."));
	FlexGridSizer4a->Add(
		btnSaveScene, 1, wxALL | wxALIGN_BOTTOM | wxALIGN_CENTER_HORIZONTAL, 5);

	FlexGridSizer4->Add(
		FlexGridSizer4a, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP);

	FlexGridSizer4b = new wxFlexGridSizer(1, 2, 0, 0);

	StaticText2 = new wxStaticText(
		this, ID_STATICTEXT4, _("Animation delay (ms):"), wxDefaultPosition,
		wxDefaultSize, wxALIGN_CENTRE, _T("ID_STATICTEXT4"));
	FlexGridSizer4b->Add(
		StaticText2, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	edDelay = new wxSpinCtrl(
		this, ID_SPINCTRL2, _T("5"), wxDefaultPosition, wxDefaultSize, 0, 0,
		1000, 20, _T("ID_SPINCTRL2"));
	edDelay->SetValue(_T("20"));
	FlexGridSizer4b->Add(
		edDelay, 1, wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL,
		5);
	FlexGridSizer4->Add(
		FlexGridSizer4b, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP);

	btnVizOptions = new wxButton(
		this, ID_BUTTON6, _("Visual options..."), wxDefaultPosition,
		wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON6"));
	FlexGridSizer4->Add(
		btnVizOptions, 1, wxALL | wxALIGN_TOP | wxALIGN_CENTER_HORIZONTAL, 5);

	StaticText3 = new wxStaticText(
		this, ID_STATICTEXT3, _("Visible sensors:"), wxDefaultPosition,
		wxDefaultSize);
	FlexGridSizer4->Add(
		StaticText3, 1, wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_TOP, 5);
	lstObsLabels = new wxCheckListBox(
		this, ID_LIST_OBS_LABELS, wxDefaultPosition, wxDefaultSize, {});
	FlexGridSizer4->Add(
		lstObsLabels, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP);

	FlexGridSizer1->Add(
		FlexGridSizer4, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	FlexGridSizer2 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer2->AddGrowableCol(0);
	FlexGridSizer2->AddGrowableRow(0);
	m_plot3D = new CMyGLCanvas(
		this, ID_XY_GLCANVAS, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL,
		_T("ID_XY_GLCANVAS"));
	FlexGridSizer2->Add(
		m_plot3D, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	FlexGridSizer1->Add(
		FlexGridSizer2, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	FlexGridSizer3 = new wxFlexGridSizer(2, 1, 0, 0);
	FlexGridSizer3->AddGrowableCol(0);
	FlexGridSizer6 = new wxFlexGridSizer(1, 1, 0, 0);
	FlexGridSizer6->AddGrowableCol(0);
	slPos = new wxSlider(
		this, ID_SLIDER1, 0, 0, 100, wxDefaultPosition, wxDefaultSize, 0,
		wxDefaultValidator, _T("ID_SLIDER1"));
	FlexGridSizer6->Add(
		slPos, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 5);
	FlexGridSizer3->Add(
		FlexGridSizer6, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	FlexGridSizer7 = new wxFlexGridSizer(1, 5, 0, 0);
	FlexGridSizer7->AddGrowableCol(4);
	StaticText1 = new wxStaticText(
		this, ID_STATICTEXT1, _("Rawlog index:"), wxDefaultPosition,
		wxDefaultSize, 0, _T("ID_STATICTEXT1"));
	FlexGridSizer7->Add(
		StaticText1, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	edIndex = new wxSpinCtrl(
		this, ID_SPINCTRL1, _T("0"), wxDefaultPosition, wxDefaultSize, 0, 0,
		100, 0, _T("ID_SPINCTRL1"));
	edIndex->SetValue(_T("0"));
	FlexGridSizer7->Add(
		edIndex, 1, wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL,
		5);
	btnJump = new wxButton(
		this, ID_BUTTON4, _("Jump"), wxDefaultPosition, wxDefaultSize, 0,
		wxDefaultValidator, _T("ID_BUTTON4"));
	FlexGridSizer7->Add(
		btnJump, 1, wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL,
		5);
	cbViewOrtho = new wxCheckBox(
		this, ID_CHECKBOX2, _("Orthogonal projection"), wxDefaultPosition,
		wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX2"));
	cbViewOrtho->SetValue(false);
	FlexGridSizer7->Add(
		cbViewOrtho, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);

	edTimestamp = new wxTextCtrl(
		this, ID_TEXTCTRL11, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0,
		wxDefaultValidator, _T("ID_TEXTCTRL11"));
	edTimestamp->SetValue("Timestamp: ");
	edTimestamp->SetEditable(false);
	FlexGridSizer7->Add(edTimestamp, 1, wxALL | wxEXPAND, 5);

	FlexGridSizer3->Add(
		FlexGridSizer7, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	FlexGridSizer1->Add(
		FlexGridSizer3, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);
	SetSizer(FlexGridSizer1);
	FlexGridSizer1->Fit(this);
	FlexGridSizer1->SetSizeHints(this);
	Center();

	Bind(wxEVT_BUTTON, &CScanAnimation::OnbtnPickInputClick, this, ID_BUTTON5);
	Bind(wxEVT_BUTTON, &CScanAnimation::OnbtnPlayClick, this, ID_BUTTON1);
	Bind(wxEVT_BUTTON, &CScanAnimation::OnbtnStopClick, this, ID_BUTTON2);
	Bind(wxEVT_BUTTON, &CScanAnimation::OnbtnCloseClick, this, ID_BUTTON3);
	Bind(
		wxEVT_SLIDER, &CScanAnimation::OnslPosCmdScrollChanged, this,
		ID_SLIDER1);
	Bind(
		wxEVT_SCROLL_THUMBTRACK, &CScanAnimation::OnslPosCmdScrollChanged, this,
		ID_SLIDER1);
	Bind(
		wxEVT_SCROLL_CHANGED, &CScanAnimation::OnslPosCmdScrollChanged, this,
		ID_SLIDER1);
	Bind(wxEVT_BUTTON, &CScanAnimation::OnbtnJumpClick, this, ID_BUTTON4);
	Bind(wxEVT_INIT_DIALOG, &CScanAnimation::OnInit, this, wxID_ANY);
	//*)

	Bind(wxEVT_BUTTON, &CScanAnimation::OnbtnVizOptions, this, ID_BUTTON6);
	Bind(
		wxEVT_BUTTON, &CScanAnimation::OnbtnSave3DScene, this,
		ID_BUTTON_SAVE_SCENE);
	Bind(
		wxEVT_CHECKBOX, &CScanAnimation::OncbViewOrthoClick, this,
		ID_CHECKBOX2);

	Bind(
		wxEVT_CHECKLISTBOX, &CScanAnimation::OncbViewOrthoClick, this,
		ID_LIST_OBS_LABELS);

	// Initialize 3D view:
	auto openGLSceneRef = m_plot3D->getOpenGLSceneRef();
	openGLSceneRef->insert(mrpt::opengl::CGridPlaneXY::Create(
		-50, 50, -50, 50, 0 /* z */, 5 /* freq */));
	openGLSceneRef->insert(mrpt::opengl::stock_objects::CornerXYZSimple(
		1.0 /*scale*/, 3.0 /*line width*/));
}

CScanAnimation::~CScanAnimation()
{
	//(*Destroy(CScanAnimation)
	//*)
}

// Get the rawlog entry (from cur. loaded rawlog), build and displays its map:
bool CScanAnimation::rebuild_view(bool forceRefreshView)
{
	WX_START_TRY

	int idx = slPos->GetValue();

	if (idx >= 0 && idx < (int)rawlog.size())
	{
		// Show/hide:
		for (const auto& kv : m_visibleSensors)
		{
			if (m_gl_objects.find(kv.first) == m_gl_objects.end()) continue;
			m_gl_objects.at(kv.first).obj->setVisibility(kv.second);
		}

		if (rawlog.getType(idx) == CRawlog::etSensoryFrame)
		{
			CSensoryFrame::Ptr sf = rawlog.getAsObservations(idx);

			bool r = update_opengl_viz(*sf);
			forceRefreshView = forceRefreshView || r;
		}
		else if (rawlog.getType(idx) == CRawlog::etObservation)
		{
			CSensoryFrame sf;
			sf.insert(rawlog.getAsObservation(idx));
			bool r = update_opengl_viz(sf);
			forceRefreshView = forceRefreshView || r;
		}
	}

	if (forceRefreshView)
	{
		m_plot3D->setCameraProjective(!cbViewOrtho->IsChecked());
		m_plot3D->Refresh();
	}

	WX_END_TRY
	return forceRefreshView;
}

// This method is called in any case for displaying a laser scan.
//  We keep an internal list of recent scans so they don't vanish
//  instantaneously.
bool CScanAnimation::update_opengl_viz(const CSensoryFrame& sf)
{
	bool hasToRefreshViz = false;

	WX_START_TRY

	auto lmbdProcessSensorLabel = [&](const std::string& sNameInMap) {
		if (m_visibleSensors.find(sNameInMap) == m_visibleSensors.end())
		{
			m_visibleSensors[sNameInMap] = true;

			lstObsLabels->AppendString(sNameInMap);
			lstObsLabels->Check(lstObsLabels->GetCount() - 1, true);
		}
	};

	// Insert new scans:
	mrpt::system::TTimeStamp tim_last = INVALID_TIMESTAMP;
	for (auto& it : sf)
	{
		const std::string sNameInMap = it->sensorLabel +
			std::string(" [Type: ") + it->GetRuntimeClass()->className +
			std::string("]");

		lmbdProcessSensorLabel(sNameInMap);

		if (IS_CLASS(*it, CObservation2DRangeScan))
		{
			auto obs = std::dynamic_pointer_cast<CObservation2DRangeScan>(it);
			hasToRefreshViz = true;
			if (tim_last == INVALID_TIMESTAMP || tim_last < obs->timestamp)
				tim_last = obs->timestamp;

			CSetOfObjects::Ptr gl_objs;

			// Already in the map with the same sensor label?
			auto it_gl = m_gl_objects.find(sNameInMap);
			if (it_gl != m_gl_objects.end())
			{
				// Update existing object:
				TRenderObject& ro = it_gl->second;
				gl_objs = std::dynamic_pointer_cast<CSetOfObjects>(ro.obj);
				ro.timestamp = obs->timestamp;
			}
			else
			{
				// Create object:
				gl_objs = CSetOfObjects::Create();

				TRenderObject ro;
				ro.obj = gl_objs;
				ro.timestamp = obs->timestamp;
				m_gl_objects[sNameInMap] = ro;
				m_plot3D->getOpenGLSceneRef()->insert(gl_objs);
			}

			const auto& p = theMainWindow->getViewOptions()->m_params;

			// convert to viz object:
			obs2Dscan_to_viz(obs, p, *gl_objs);
		}
		else if (IS_CLASS(*it, CObservation3DRangeScan))
		{
			auto obs = std::dynamic_pointer_cast<CObservation3DRangeScan>(it);

			obs->load();
			hasToRefreshViz = true;
			if (tim_last == INVALID_TIMESTAMP || tim_last < obs->timestamp)
				tim_last = obs->timestamp;

			CSetOfObjects::Ptr gl_objs;

			auto it_gl = m_gl_objects.find(sNameInMap);
			if (it_gl != m_gl_objects.end())
			{
				// Update existing object:
				TRenderObject& ro = it_gl->second;
				gl_objs = std::dynamic_pointer_cast<CSetOfObjects>(ro.obj);
				ro.timestamp = obs->timestamp;
			}
			else
			{
				// Create object:
				gl_objs = CSetOfObjects::Create();

				TRenderObject ro;
				ro.obj = gl_objs;
				ro.timestamp = obs->timestamp;
				m_gl_objects[sNameInMap] = ro;
				m_plot3D->getOpenGLSceneRef()->insert(gl_objs);
			}

			const auto& p = theMainWindow->getViewOptions()->m_params;

			// convert to viz object:
			obs3Dscan_to_viz(obs, p, *gl_objs);

			obs->unload();
		}
		else if (IS_CLASS(*it, CObservationVelodyneScan))
		{
			CObservationVelodyneScan::Ptr obs =
				std::dynamic_pointer_cast<CObservationVelodyneScan>(it);
			obs->load();

			hasToRefreshViz = true;
			if (tim_last == INVALID_TIMESTAMP || tim_last < obs->timestamp)
				tim_last = obs->timestamp;

			obs->generatePointCloud();

			CSetOfObjects::Ptr gl_objs;

			CColouredPointsMap pointMap;
			pointMap.loadFromVelodyneScan(*obs);

			// Already in the map with the same sensor label?
			auto it_gl = m_gl_objects.find(sNameInMap);
			if (it_gl != m_gl_objects.end())
			{
				// Update existing object:
				TRenderObject& ro = it_gl->second;
				gl_objs = std::dynamic_pointer_cast<CSetOfObjects>(ro.obj);
				ro.timestamp = obs->timestamp;
			}
			else
			{
				// Create object:
				gl_objs = CSetOfObjects::Create();

				TRenderObject ro;
				ro.obj = gl_objs;
				ro.timestamp = obs->timestamp;
				m_gl_objects[sNameInMap] = ro;
				m_plot3D->getOpenGLSceneRef()->insert(gl_objs);
			}

			auto& p = theMainWindow->getViewOptions()->m_params;

			// convert to viz object:
			obsVelodyne_to_viz(obs, p, *gl_objs);

			obs->point_cloud.clear_deep();
			obs->unload();
		}
		else if (IS_CLASS(*it, CObservationPointCloud))
		{
			auto obs = std::dynamic_pointer_cast<CObservationPointCloud>(it);
			obs->load();

			hasToRefreshViz = true;
			if (tim_last == INVALID_TIMESTAMP || tim_last < obs->timestamp)
				tim_last = obs->timestamp;

			// Already in the map with the same sensor label?
			auto it_gl = m_gl_objects.find(sNameInMap);
			if (it_gl != m_gl_objects.end())
			{
				// Update existing object:
				TRenderObject& ro = it_gl->second;
				auto gl_obj =
					std::dynamic_pointer_cast<CPointCloudColoured>(ro.obj);
				gl_obj->loadFromPointsMap(obs->pointcloud.get());
				gl_obj->setPose(obs->sensorPose);
				ro.timestamp = obs->timestamp;
			}
			else
			{
				// Create object:
				auto gl_obj = std::make_shared<CPointCloudColoured>();
				gl_obj->setPointSize(3.0);
				gl_obj->loadFromPointsMap(obs->pointcloud.get());
				gl_obj->setPose(obs->sensorPose);

				TRenderObject ro;
				ro.obj = gl_obj;
				ro.timestamp = obs->timestamp;
				m_gl_objects[sNameInMap] = ro;
				m_plot3D->getOpenGLSceneRef()->insert(gl_obj);
			}
			obs->unload();
		}
	}

	// Check what observations are too old and must be deleted:
	const double largest_period = 1.0;
	std::vector<std::string> lst_to_delete;
	for (auto& o : m_gl_objects)
	{
		TRenderObject& ro = o.second;

		// Scans without timestamps
		if ((tim_last == INVALID_TIMESTAMP && hasToRefreshViz) ||
			(tim_last != INVALID_TIMESTAMP &&
			 fabs(mrpt::system::timeDifference(ro.timestamp, tim_last)) >
				 largest_period))
		{ lst_to_delete.push_back(o.first); }
	}

	// Remove too old observations:
	for (const auto& s : lst_to_delete)
	{
		TRenderObject& ro = m_gl_objects[s];
		// Remove from the opengl viewport
		m_plot3D->getOpenGLSceneRef()->removeObject(ro.obj);
		m_gl_objects.erase(s);	// and from my list
	}

	// Show timestamp:
	edTimestamp->SetValue(mrpt::format(
		"Timestamp (UTC): %s (%.06f)",
		mrpt::system::dateTimeToString(tim_last).c_str(),
		mrpt::Clock::toDouble(tim_last)));

	WX_END_TRY
	return hasToRefreshViz;
}

void CScanAnimation::OnbtnPlayClick(wxCommandEvent&)
{
	//

	// Disable all but stop while playing:
	btnStop->Enable();
	btnPlay->Disable();
	btnClose->Disable();

	slPos->Disable();

	m_stop = false;

	try
	{
		wxBusyCursor cursorBusy;
		int delay_ms = edDelay->GetValue();

		while (!m_stop)
		{
			int idx = slPos->GetValue();
			if (idx >= ((int)rawlog.size()) - 1) break;	 // End!

			if (rawlog.getType(idx) != CRawlog::etActionCollection)
			{
				bool refreshed = rebuild_view(false);
				if (refreshed) ::wxMilliSleep(delay_ms);
			}

			idx++;
			slPos->SetValue(idx);
			edIndex->SetValue(idx);
			wxTheApp->Yield();
		}
	}
	catch (...)
	{
	}

	btnStop->Disable();
	btnPlay->Enable();
	btnClose->Enable();

	slPos->Enable();
}

void CScanAnimation::OnbtnStopClick(wxCommandEvent&) { m_stop = true; }
void CScanAnimation::OnbtnCloseClick(wxCommandEvent&) { Close(); }
void CScanAnimation::OnslPosCmdScrollChanged(wxCommandEvent&)
{
	edIndex->SetValue(slPos->GetValue());
	rebuild_view(true);
}

void CScanAnimation::OnbtnJumpClick(wxCommandEvent&)
{
	slPos->SetValue(edIndex->GetValue());
	rebuild_view(true);
}

void CScanAnimation::OnslPosCmdScroll(wxScrollEvent&) { rebuild_view(true); }
void CScanAnimation::OnbtnPickInputClick(wxCommandEvent&) {}
void CScanAnimation::OnInit(wxInitDialogEvent&)
{
	slPos->SetValue(0);
	slPos->SetMin(0);
	slPos->SetMax((int)rawlog.size());

	edIndex->SetValue(0);
	edIndex->SetRange(0, (int)rawlog.size());

	Fit();

	slPos->Enable(true);
	edIndex->Enable(true);
	btnJump->Enable(true);
}

void CScanAnimation::OncbViewOrthoClick(wxCommandEvent&)
{
	for (size_t i = 0; i < lstObsLabels->GetCount(); i++)
	{
		bool visible = lstObsLabels->IsChecked(i);
		const auto str = lstObsLabels->GetString(i).ToStdString();
		m_visibleSensors.at(str) = visible;
	}

	rebuild_view(true);
}

wxDialog* dlgViz = nullptr;
static void dlgButton(wxCommandEvent&) { dlgViz->EndModal(0); }

void CScanAnimation::OnbtnVizOptions(wxCommandEvent&)
{
	wxDialog dlg(
		this, wxID_ANY, "Visualization options", wxDefaultPosition,
		wxDefaultSize, wxDEFAULT_DIALOG_STYLE, _T("wxID_ANY"));
	dlgViz = &dlg;

	auto sizer1 = new wxFlexGridSizer(2, 1, 0, 0);
	sizer1->AddGrowableCol(0);

	auto panel = new ViewOptions3DPoints(&dlg);
	sizer1->Add(panel, 1, wxALL | wxEXPAND | wxALIGN_LEFT | wxALIGN_TOP, 0);

	auto pn2 = new wxPanel(&dlg);
	sizer1->Add(pn2, 1, wxALL | wxEXPAND, 5);

	auto btnOk = new wxButton(
		&dlg, ID_BUTTON7, _("Close"), wxDefaultPosition, wxDefaultSize, 0,
		wxDefaultValidator, _T("ID_BUTTON7"));
	sizer1->Add(btnOk, 1, wxALL | wxEXPAND, 5);

	dlg.SetSizer(sizer1);
	sizer1->Fit(&dlg);
	sizer1->SetSizeHints(&dlg);
	dlg.Center();

	dlg.Bind(wxEVT_BUTTON, &dlgButton, ID_BUTTON7);

	auto& p = theMainWindow->getViewOptions()->m_params;
	p.to_UI(*panel);

	dlg.ShowModal();

	p.from_UI(*panel);

	rebuild_view(true);
}

void CScanAnimation::OnbtnSave3DScene(wxCommandEvent&)
{
	WX_START_TRY

	wxString defaultDir = iniFile->read_string(iniFileSect, "LastDir", ".");

	wxFileDialog dialog(
		this, "Save scene as...", defaultDir,
		mrpt::format("view_%06i.3Dscene", static_cast<int>(slPos->GetValue())),
		"3Dscene files (*.3Dscene)|*.3Dscene|All files (*.*)|*.*",
		wxFD_SAVE | wxFD_OVERWRITE_PROMPT);

	if (dialog.ShowModal() != wxID_OK) return;

	auto openGLSceneRef = m_plot3D->getOpenGLSceneRef();
	openGLSceneRef->saveToFile(dialog.GetPath().ToStdString());

	WX_END_TRY
}
