/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "ViewOptions3DPoints.h"

//(*InternalHeaders(ViewOptions3DPoints)
#include <wx/intl.h>
#include <wx/string.h>
//*)

//(*IdInit(ViewOptions3DPoints)
const long ViewOptions3DPoints::ID_CHECKBOX4 = wxNewId();
const long ViewOptions3DPoints::ID_STATICTEXT1 = wxNewId();
const long ViewOptions3DPoints::ID_TEXTCTRL1 = wxNewId();
const long ViewOptions3DPoints::ID_STATICTEXT2 = wxNewId();
const long ViewOptions3DPoints::ID_TEXTCTRL2 = wxNewId();
const long ViewOptions3DPoints::ID_STATICTEXT3 = wxNewId();
const long ViewOptions3DPoints::ID_TEXTCTRL3 = wxNewId();
const long ViewOptions3DPoints::ID_BUTTON1 = wxNewId();
const long ViewOptions3DPoints::ID_CHECKBOX1 = wxNewId();
const long ViewOptions3DPoints::ID_RADIOBOX1 = wxNewId();
const long ViewOptions3DPoints::ID_RADIOBOX2 = wxNewId();
const long ViewOptions3DPoints::ID_CHECKBOX2 = wxNewId();
const long ViewOptions3DPoints::ID_STATICTEXT4 = wxNewId();
const long ViewOptions3DPoints::ID_SPINCTRL1 = wxNewId();
const long ViewOptions3DPoints::ID_CHECKBOX3 = wxNewId();
const long ViewOptions3DPoints::ID_CHECKBOX5 = wxNewId();
const long ViewOptions3DPoints::ID_STATICTEXT5 = wxNewId();
const long ViewOptions3DPoints::ID_TEXTCTRL4 = wxNewId();
const long ViewOptions3DPoints::ID_CHECKBOX6 = wxNewId();
const long ViewOptions3DPoints::ID_COLOURPICKERCTRL1 = wxNewId();
const long ViewOptions3DPoints::ID_CHECKBOX7 = wxNewId();
const long ViewOptions3DPoints::ID_COLOURPICKERCTRL2 = wxNewId();
//*)

BEGIN_EVENT_TABLE(ViewOptions3DPoints, wxPanel)
//(*EventTable(ViewOptions3DPoints)
//*)
END_EVENT_TABLE()

#include <mrpt/config/CConfigFile.h>
#include <mrpt/gui/WxUtils.h>

#include <memory>

extern std::unique_ptr<mrpt::config::CConfigFile> iniFile;

ViewOptions3DPoints::ViewOptions3DPoints(wxWindow* parent, wxWindowID id)
{
	//(*Initialize(ViewOptions3DPoints)
	wxStaticBoxSizer* StaticBoxSizer2;
	wxFlexGridSizer* FlexGridSizer1;
	wxFlexGridSizer* FlexGridSizer2;
	wxFlexGridSizer* FlexGridSizer7;
	wxFlexGridSizer* FlexGridSizer4;
	wxStaticBoxSizer* StaticBoxSizer3;
	wxFlexGridSizer* FlexGridSizer6;
	wxFlexGridSizer* FlexGridSizer3;
	wxStaticBoxSizer* StaticBoxSizer4;
	wxFlexGridSizer* FlexGridSizer5;
	wxStaticBoxSizer* StaticBoxSizer1;

	Create(
		parent, id, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL,
		_T("id"));
	FlexGridSizer1 = new wxFlexGridSizer(1, 3, 0, 0);
	StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Axes"));
	FlexGridSizer2 = new wxFlexGridSizer(0, 2, 0, 0);
	cbShowAxes = new wxCheckBox(
		this, ID_CHECKBOX4, _("Show axes"), wxDefaultPosition, wxDefaultSize, 0,
		wxDefaultValidator, _T("ID_CHECKBOX4"));
	cbShowAxes->SetValue(true);
	FlexGridSizer2->Add(
		cbShowAxes, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer2->Add(
		0, 0, 1, wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL,
		5);
	StaticText1 = new wxStaticText(
		this, ID_STATICTEXT1, _("Tick interval [m]:"), wxDefaultPosition,
		wxDefaultSize, 0, _T("ID_STATICTEXT1"));
	FlexGridSizer2->Add(
		StaticText1, 1, wxALL | wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL, 5);
	edTickInterval = new wxTextCtrl(
		this, ID_TEXTCTRL1, _("1.0"), wxDefaultPosition, wxDefaultSize, 0,
		wxDefaultValidator, _T("ID_TEXTCTRL1"));
	FlexGridSizer2->Add(
		edTickInterval, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	StaticText2 = new wxStaticText(
		this, ID_STATICTEXT2, _("Limits [m]:"), wxDefaultPosition,
		wxDefaultSize, 0, _T("ID_STATICTEXT2"));
	FlexGridSizer2->Add(
		StaticText2, 1, wxALL | wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL, 5);
	edAxisLimits = new wxTextCtrl(
		this, ID_TEXTCTRL2, _("20.0"), wxDefaultPosition, wxDefaultSize, 0,
		wxDefaultValidator, _T("ID_TEXTCTRL2"));
	FlexGridSizer2->Add(
		edAxisLimits, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	StaticText3 = new wxStaticText(
		this, ID_STATICTEXT3, _("Tick text size [m]:"), wxDefaultPosition,
		wxDefaultSize, 0, _T("ID_STATICTEXT3"));
	FlexGridSizer2->Add(
		StaticText3, 1, wxALL | wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL, 5);
	edTickTextSize = new wxTextCtrl(
		this, ID_TEXTCTRL3, _("0.075"), wxDefaultPosition, wxDefaultSize, 0,
		wxDefaultValidator, _T("ID_TEXTCTRL3"));
	FlexGridSizer2->Add(
		edTickTextSize, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	btnApply = new wxButton(
		this, ID_BUTTON1, _("Apply"), wxDefaultPosition, wxDefaultSize, 0,
		wxDefaultValidator, _T("ID_BUTTON1"));
	FlexGridSizer2->Add(
		btnApply, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	StaticBoxSizer1->Add(FlexGridSizer2, 1, wxEXPAND, 5);
	FlexGridSizer1->Add(StaticBoxSizer1, 1, wxEXPAND, 5);
	StaticBoxSizer2 =
		new wxStaticBoxSizer(wxHORIZONTAL, this, _("Point cloud"));
	FlexGridSizer3 = new wxFlexGridSizer(0, 1, 0, 0);
	cbColorFromRGB = new wxCheckBox(
		this, ID_CHECKBOX1, _("Color from RGB/intensity (if available)"),
		wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator,
		_T("ID_CHECKBOX1"));
	cbColorFromRGB->SetValue(true);
	FlexGridSizer3->Add(
		cbColorFromRGB, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	wxString __wxRadioBoxChoices_1[4] = {_("x"), _("y"), _("z"), _("none")};

	cbOnlyPointsWithColor = new wxCheckBox(
		this, ID_CHECKBOX7, _("Only points with RGB"), wxDefaultPosition,
		wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX7"));
	cbOnlyPointsWithColor->SetValue(false);
	FlexGridSizer3->Add(
		cbOnlyPointsWithColor, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);

	rbColorByAxis = new wxRadioBox(
		this, ID_RADIOBOX1, _("Otherwise, color from: "), wxDefaultPosition,
		wxDefaultSize, 4, __wxRadioBoxChoices_1, 1, 0, wxDefaultValidator,
		_T("ID_RADIOBOX1"));
	FlexGridSizer3->Add(rbColorByAxis, 1, wxEXPAND, 5);
	wxString __wxRadioBoxChoices_2[3] = {
		_("cmGRAYSCALE"), _("cmJET"), _("cmHOT")};
	RadioBox1 = new wxRadioBox(
		this, ID_RADIOBOX2, _("Color map"), wxDefaultPosition, wxDefaultSize, 3,
		__wxRadioBoxChoices_2, 1, 0, wxDefaultValidator, _T("ID_RADIOBOX2"));
	FlexGridSizer3->Add(
		RadioBox1, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	cbInvertColormap = new wxCheckBox(
		this, ID_CHECKBOX2, _("Invert color map"), wxDefaultPosition,
		wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX2"));
	cbInvertColormap->SetValue(false);
	FlexGridSizer3->Add(
		cbInvertColormap, 1, wxALL | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer4 = new wxFlexGridSizer(0, 2, 0, 0);
	StaticText4 = new wxStaticText(
		this, ID_STATICTEXT4, _("Point size:"), wxDefaultPosition,
		wxDefaultSize, 0, _T("ID_STATICTEXT4"));
	FlexGridSizer4->Add(
		StaticText4, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	edPointSize = new wxSpinCtrl(
		this, ID_SPINCTRL1, _T("4"), wxDefaultPosition, wxDefaultSize, 0, 1,
		100, 4, _T("ID_SPINCTRL1"));
	edPointSize->SetValue(_T("4"));
	FlexGridSizer4->Add(
		edPointSize, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer3->Add(FlexGridSizer4, 1, wxEXPAND, 0);
	StaticBoxSizer2->Add(FlexGridSizer3, 1, wxEXPAND, 5);
	FlexGridSizer1->Add(StaticBoxSizer2, 1, wxEXPAND, 5);
	FlexGridSizer5 = new wxFlexGridSizer(0, 1, 0, 0);
	StaticBoxSizer3 =
		new wxStaticBoxSizer(wxHORIZONTAL, this, _("Sensor pose"));
	FlexGridSizer6 = new wxFlexGridSizer(0, 1, 0, 0);

	cbShowSensorPose = new wxCheckBox(
		this, ID_CHECKBOX3, _("Show sensor pose"), wxDefaultPosition,
		wxDefaultSize, 0, wxDefaultValidator, _T("ID_CHECKBOX3"));
	cbShowSensorPose->SetValue(false);
	FlexGridSizer6->Add(
		cbShowSensorPose, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);

	StaticText5 = new wxStaticText(
		this, ID_STATICTEXT5, _("XYZ corner scale [m]:"), wxDefaultPosition,
		wxDefaultSize, 0, _T("ID_STATICTEXT5"));
	FlexGridSizer6->Add(
		StaticText5, 1, wxALL | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL, 5);
	edSensorPoseScale = new wxTextCtrl(
		this, ID_TEXTCTRL4, _("0.1"), wxDefaultPosition, wxDefaultSize, 0,
		wxDefaultValidator, _T("ID_TEXTCTRL4"));
	FlexGridSizer6->Add(edSensorPoseScale, 1, wxEXPAND, 5);
	StaticBoxSizer3->Add(FlexGridSizer6, 1, wxEXPAND, 5);
	FlexGridSizer5->Add(StaticBoxSizer3, 1, wxEXPAND, 5);
	StaticBoxSizer4 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("2D lidars"));
	FlexGridSizer7 = new wxFlexGridSizer(0, 2, 0, 0);
	cb2DShowSurf = new wxCheckBox(
		this, ID_CHECKBOX5, _("Show surface"), wxDefaultPosition, wxDefaultSize,
		0, wxDefaultValidator, _T("ID_CHECKBOX5"));
	cb2DShowSurf->SetValue(true);
	FlexGridSizer7->Add(
		cb2DShowSurf, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	colorSurface = new wxColourPickerCtrl(
		this, ID_COLOURPICKERCTRL1, wxColour(0, 0, 0), wxDefaultPosition,
		wxDefaultSize, 0, wxDefaultValidator, _T("ID_COLOURPICKERCTRL1"));
	FlexGridSizer7->Add(
		colorSurface, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	cb2DShowPoints = new wxCheckBox(
		this, ID_CHECKBOX6, _("Show points"), wxDefaultPosition, wxDefaultSize,
		0, wxDefaultValidator, _T("ID_CHECKBOX6"));
	cb2DShowPoints->SetValue(true);
	FlexGridSizer7->Add(
		cb2DShowPoints, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	color2DPoints = new wxColourPickerCtrl(
		this, ID_COLOURPICKERCTRL2, wxColour(0, 0, 0), wxDefaultPosition,
		wxDefaultSize, 0, wxDefaultValidator, _T("ID_COLOURPICKERCTRL2"));
	FlexGridSizer7->Add(
		color2DPoints, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	StaticBoxSizer4->Add(
		FlexGridSizer7, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	FlexGridSizer5->Add(StaticBoxSizer4, 1, wxEXPAND, 5);
	FlexGridSizer1->Add(FlexGridSizer5, 1, wxEXPAND, 0);
	SetSizer(FlexGridSizer1);
	FlexGridSizer1->Fit(this);
	FlexGridSizer1->SetSizeHints(this);
	//*)

	using Me = ViewOptions3DPoints;

	Bind(wxEVT_BUTTON, &Me::OnbtnApplyClick, this, ID_BUTTON1);
	Bind(wxEVT_CHECKBOX, &Me::OnbtnApplyClick, this, ID_CHECKBOX1);
	Bind(wxEVT_CHECKBOX, &Me::OnbtnApplyClick, this, ID_CHECKBOX2);
	Bind(wxEVT_RADIOBOX, &Me::OnbtnApplyClick, this, ID_RADIOBOX1);
	Bind(wxEVT_RADIOBOX, &Me::OnbtnApplyClick, this, ID_RADIOBOX2);
	Bind(wxEVT_CHECKBOX, &Me::OnbtnApplyClick, this, ID_CHECKBOX3);
	Bind(wxEVT_SPINCTRL, &Me::OnbtnApplyClick, this, ID_SPINCTRL1);

	m_params.load_from_ini_file(*iniFile);
	m_params.to_UI(*this);
}

ViewOptions3DPoints::~ViewOptions3DPoints()
{
	//(*Destroy(ViewOptions3DPoints)
	//*)
	m_params.save_to_ini_file(*iniFile);
}

void ViewOptions3DPoints::OnbtnApplyClick(wxCommandEvent&)
{
	m_params.from_UI(*this);
}

// ---------------------------------------------------
// ParametersView3DPoints
// ---------------------------------------------------
void ParametersView3DPoints::to_UI(ViewOptions3DPoints& ui) const
{
	WX_START_TRY

	ui.edTickInterval->SetValue(wxString::Format("%.03f", axisTickFrequency));
	ui.edAxisLimits->SetValue(wxString::Format("%.03f", axisLimits));
	ui.edTickTextSize->SetValue(wxString::Format("%.03f", axisTickTextSize));

	ui.cbColorFromRGB->SetValue(colorFromRGBimage);
	ui.rbColorByAxis->SetSelection(colorizeByAxis);
	ui.cbInvertColormap->SetValue(invertColorMapping);

	ui.RadioBox1->SetSelection(static_cast<int>(colorMap));

	ui.edPointSize->SetValue(pointSize);

	ui.cbShowSensorPose->SetValue(drawSensorPose);
	ui.cbOnlyPointsWithColor->SetValue(onlyPointsWithColor);
	ui.edSensorPoseScale->SetValue(wxString::Format("%.03f", sensorPoseScale));

	ui.cbShowAxes->SetValue(showAxis);
	ui.cb2DShowPoints->SetValue(showPointsIn2Dscans);
	ui.cb2DShowSurf->SetValue(showSurfaceIn2Dscans);

	ui.colorSurface->SetColour(wxColour(surface2DscansColor));
	ui.color2DPoints->SetColour(wxColour(points2DscansColor));

	WX_END_TRY
}
void ParametersView3DPoints::from_UI(const ViewOptions3DPoints& ui)
{
	WX_START_TRY

	ui.edTickInterval->GetValue().ToCDouble(&axisTickFrequency);
	ui.edAxisLimits->GetValue().ToCDouble(&axisLimits);
	ui.edTickTextSize->GetValue().ToCDouble(&axisTickTextSize);

	colorFromRGBimage = ui.cbColorFromRGB->IsChecked();

	colorizeByAxis = ui.rbColorByAxis->GetSelection();

	invertColorMapping = ui.cbInvertColormap->IsChecked();

	colorMap = mrpt::typemeta::TEnumType<mrpt::img::TColormap>::name2value(
		ui.RadioBox1->GetStringSelection().ToStdString());

	pointSize = ui.edPointSize->GetValue();

	drawSensorPose = ui.cbShowSensorPose->IsChecked();
	onlyPointsWithColor = ui.cbOnlyPointsWithColor->IsChecked();
	ui.edSensorPoseScale->GetValue().ToCDouble(&sensorPoseScale);

	showAxis = ui.cbShowAxes->GetValue();
	showPointsIn2Dscans = ui.cb2DShowPoints->GetValue();
	showSurfaceIn2Dscans = ui.cb2DShowSurf->GetValue();

	surface2DscansColor =
		mrpt::img::TColor(ui.colorSurface->GetColour().GetRGBA());
	points2DscansColor =
		mrpt::img::TColor(ui.color2DPoints->GetColour().GetRGBA());

	WX_END_TRY
}
