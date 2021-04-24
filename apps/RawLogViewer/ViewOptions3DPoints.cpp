/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "ViewOptions3DPoints.h"

//(*InternalHeaders(ViewOptions3DPoints)
#include <wx/intl.h>
#include <wx/string.h>
//*)

//(*IdInit(ViewOptions3DPoints)
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
	wxFlexGridSizer* FlexGridSizer4;
	wxFlexGridSizer* FlexGridSizer3;
	wxStaticBoxSizer* StaticBoxSizer1;

	Create(
		parent, id, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL,
		_T("id"));
	FlexGridSizer1 = new wxFlexGridSizer(2, 2, 0, 0);
	FlexGridSizer1->AddGrowableCol(0);
	FlexGridSizer1->AddGrowableCol(1);
	FlexGridSizer1->AddGrowableRow(0);
	StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Axes"));
	FlexGridSizer2 = new wxFlexGridSizer(0, 2, 0, 0);
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
	StaticBoxSizer1->Add(FlexGridSizer2, 1, wxALL | wxEXPAND, 5);
	FlexGridSizer1->Add(StaticBoxSizer1, 1, wxALL | wxEXPAND, 5);
	StaticBoxSizer2 =
		new wxStaticBoxSizer(wxHORIZONTAL, this, _("Point cloud"));
	FlexGridSizer3 = new wxFlexGridSizer(0, 1, 0, 0);
	cbColorFromRGB = new wxCheckBox(
		this, ID_CHECKBOX1, _("Color from RGB (if available)"),
		wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator,
		_T("ID_CHECKBOX1"));
	cbColorFromRGB->SetValue(true);
	FlexGridSizer3->Add(
		cbColorFromRGB, 1,
		wxALL | wxALIGN_CENTER_HORIZONTAL | wxALIGN_CENTER_VERTICAL, 5);
	wxString __wxRadioBoxChoices_1[4] = {_("x"), _("y"), _("z"), _("none")};
	rbColorByAxis = new wxRadioBox(
		this, ID_RADIOBOX1, _("Otherwise, color from: "), wxDefaultPosition,
		wxDefaultSize, 4, __wxRadioBoxChoices_1, 1, 0, wxDefaultValidator,
		_T("ID_RADIOBOX1"));
	FlexGridSizer3->Add(rbColorByAxis, 1, wxALL | wxEXPAND, 5);
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
	FlexGridSizer3->Add(FlexGridSizer4, 1, wxALL | wxEXPAND, 0);
	StaticBoxSizer2->Add(FlexGridSizer3, 1, wxALL | wxEXPAND, 5);
	FlexGridSizer1->Add(StaticBoxSizer2, 1, wxALL | wxEXPAND, 5);
	SetSizer(FlexGridSizer1);
	FlexGridSizer1->Fit(this);
	FlexGridSizer1->SetSizeHints(this);

	//*)

	Bind(wxEVT_BUTTON, &ViewOptions3DPoints::OnbtnApplyClick, this, ID_BUTTON1);
	Bind(
		wxEVT_RADIOBOX, &ViewOptions3DPoints::OnbtnApplyClick, this,
		ID_RADIOBOX1);
	Bind(
		wxEVT_RADIOBOX, &ViewOptions3DPoints::OnbtnApplyClick, this,
		ID_RADIOBOX2);
	Bind(
		wxEVT_CHECKBOX, &ViewOptions3DPoints::OnbtnApplyClick, this,
		ID_CHECKBOX2);
	Bind(
		wxEVT_SPINCTRL, &ViewOptions3DPoints::OnbtnApplyClick, this,
		ID_SPINCTRL1);

	m_params.load_from_ini_file();
	m_params.to_UI(*this);
}

ViewOptions3DPoints::~ViewOptions3DPoints()
{
	//(*Destroy(ViewOptions3DPoints)
	//*)
	m_params.save_to_ini_file();
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

	WX_END_TRY
}

void ParametersView3DPoints::save_to_ini_file() const
{
	auto& c = *iniFile;
	const std::string s = "ParametersView3DPoints";

	MRPT_SAVE_CONFIG_VAR(axisTickFrequency, c, s);
	MRPT_SAVE_CONFIG_VAR(axisLimits, c, s);
	MRPT_SAVE_CONFIG_VAR(axisTickTextSize, c, s);
	MRPT_SAVE_CONFIG_VAR(colorFromRGBimage, c, s);
	MRPT_SAVE_CONFIG_VAR(colorizeByAxis, c, s);
	MRPT_SAVE_CONFIG_VAR(invertColorMapping, c, s);
	MRPT_SAVE_CONFIG_VAR(pointSize, c, s);
	MRPT_SAVE_CONFIG_VAR(colorMap, c, s);
}

void ParametersView3DPoints::load_from_ini_file()
{
	auto& c = *iniFile;
	const std::string s = "ParametersView3DPoints";

	MRPT_LOAD_CONFIG_VAR_CS(axisTickFrequency, double);
	MRPT_LOAD_CONFIG_VAR_CS(axisLimits, double);
	MRPT_LOAD_CONFIG_VAR_CS(axisTickTextSize, double);
	MRPT_LOAD_CONFIG_VAR_CS(colorFromRGBimage, bool);
	MRPT_LOAD_CONFIG_VAR_CS(colorizeByAxis, int);
	MRPT_LOAD_CONFIG_VAR_CS(invertColorMapping, bool);
	MRPT_LOAD_CONFIG_VAR_CS(pointSize, double);
	colorMap = c.read_enum(s, "colorMap", colorMap);
}
