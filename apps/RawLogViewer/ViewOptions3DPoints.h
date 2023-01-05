/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#ifndef VIEWOPTIONS3DPOINTS_H
#define VIEWOPTIONS3DPOINTS_H

//(*Headers(ViewOptions3DPoints)
#include <wx/button.h>
#include <wx/checkbox.h>
#include <wx/clrpicker.h>
#include <wx/panel.h>
#include <wx/radiobox.h>
#include <wx/sizer.h>
#include <wx/spinctrl.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
//*)

#include <mrpt/img/color_maps.h>

#include "ParametersView3DPoints.h"

class ViewOptions3DPoints : public wxPanel
{
   public:
	ViewOptions3DPoints(wxWindow* parent, wxWindowID id = wxID_ANY);
	virtual ~ViewOptions3DPoints();

	//(*Declarations(ViewOptions3DPoints)
	wxColourPickerCtrl* color2DPoints;
	wxCheckBox* cb2DShowSurf;
	wxTextCtrl* edAxisLimits;
	wxStaticText* StaticText1;
	wxCheckBox* cbInvertColormap;
	wxTextCtrl* edTickInterval;
	wxStaticText* StaticText3;
	wxTextCtrl* edSensorPoseScale;
	wxCheckBox* cbShowSensorPose;
	wxCheckBox* cbOnlyPointsWithColor;
	wxButton* btnApply;
	wxColourPickerCtrl* colorSurface;
	wxStaticText* StaticText4;
	wxStaticText* StaticText5;
	wxStaticText* StaticText2;
	wxCheckBox* cb2DShowPoints;
	wxRadioBox* RadioBox1;
	wxCheckBox* cbColorFromRGB;
	wxRadioBox* rbColorByAxis;
	wxTextCtrl* edTickTextSize;
	wxCheckBox* cbShowAxes;
	wxSpinCtrl* edPointSize;
	//*)

	ParametersView3DPoints m_params;

   protected:
	//(*Identifiers(ViewOptions3DPoints)
	static const long ID_CHECKBOX4;
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
	static const long ID_CHECKBOX3;
	static const long ID_STATICTEXT5;
	static const long ID_TEXTCTRL4;
	static const long ID_CHECKBOX5;
	static const long ID_COLOURPICKERCTRL1;
	static const long ID_CHECKBOX6;
	static const long ID_CHECKBOX7;
	static const long ID_COLOURPICKERCTRL2;
	//*)

   private:
	//(*Handlers(ViewOptions3DPoints)
	void OnbtnApplyClick(wxCommandEvent& event);
	//*)

	DECLARE_EVENT_TABLE()
};

#endif
