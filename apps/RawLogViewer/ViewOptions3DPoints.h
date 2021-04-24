/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#ifndef VIEWOPTIONS3DPOINTS_H
#define VIEWOPTIONS3DPOINTS_H

//(*Headers(ViewOptions3DPoints)
#include <wx/button.h>
#include <wx/checkbox.h>
#include <wx/panel.h>
#include <wx/radiobox.h>
#include <wx/sizer.h>
#include <wx/spinctrl.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
//*)

#include <mrpt/img/color_maps.h>

class ViewOptions3DPoints;

struct ParametersView3DPoints
{
	ParametersView3DPoints() = default;

	double axisTickFrequency = 1.0;
	double axisLimits = 20.0;
	double axisTickTextSize = 0.075;
	bool colorFromRGBimage = true;
	int colorizeByAxis = 0;	 // 0:x,1:y,2:z, anything else = none.
	bool invertColorMapping = false;
	mrpt::img::TColormap colorMap = mrpt::img::cmJET;
	double pointSize = 4.0;

	void to_UI(ViewOptions3DPoints& ui) const;
	void from_UI(const ViewOptions3DPoints& ui);

	void save_to_ini_file() const;
	void load_from_ini_file();
};

class ViewOptions3DPoints : public wxPanel
{
   public:
	ViewOptions3DPoints(wxWindow* parent, wxWindowID id = wxID_ANY);
	virtual ~ViewOptions3DPoints();

	//(*Declarations(ViewOptions3DPoints)
	wxTextCtrl* edAxisLimits;
	wxStaticText* StaticText1;
	wxCheckBox* cbInvertColormap;
	wxTextCtrl* edTickInterval;
	wxStaticText* StaticText3;
	wxButton* btnApply;
	wxStaticText* StaticText4;
	wxStaticText* StaticText2;
	wxRadioBox* RadioBox1;
	wxCheckBox* cbColorFromRGB;
	wxRadioBox* rbColorByAxis;
	wxTextCtrl* edTickTextSize;
	wxSpinCtrl* edPointSize;
	//*)

	ParametersView3DPoints m_params;

   protected:
	//(*Identifiers(ViewOptions3DPoints)
	static const long ID_STATICTEXT1;
	static const long ID_TEXTCTRL1;
	static const long ID_STATICTEXT2;
	static const long ID_TEXTCTRL2;
	static const long ID_STATICTEXT3;
	static const long ID_TEXTCTRL3;
	static const long ID_BUTTON1;
	static const long ID_CHECKBOX1;
	static const long ID_RADIOBOX1;
	static const long ID_RADIOBOX2;
	static const long ID_CHECKBOX2;
	static const long ID_STATICTEXT4;
	static const long ID_SPINCTRL1;
	//*)

   private:
	//(*Handlers(ViewOptions3DPoints)
	void OnbtnApplyClick(wxCommandEvent& event);
	//*)

	DECLARE_EVENT_TABLE()
};

#endif
