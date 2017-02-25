/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CDLGPLYOPTIONS_H
#define CDLGPLYOPTIONS_H

//(*Headers(CDlgPLYOptions)
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/radiobox.h>
#include <wx/textctrl.h>
#include <wx/checkbox.h>
#include <wx/panel.h>
#include <wx/choice.h>
#include <wx/button.h>
#include <wx/dialog.h>
//*)

class CDlgPLYOptions: public wxDialog
{
	public:

		CDlgPLYOptions(wxWindow* parent,wxWindowID id=wxID_ANY);
		virtual ~CDlgPLYOptions();

		//(*Declarations(CDlgPLYOptions)
		wxCheckBox* cbXYZ;
		wxCheckBox* cbXYGrid;
		wxStaticText* StaticText2;
		wxButton* btnCancel;
		wxRadioBox* rbIntFromXYZ;
		wxTextCtrl* edPitch;
		wxPanel* Panel1;
		wxStaticText* StaticText1;
		wxStaticText* StaticText3;
		wxButton* btnOK;
		wxRadioBox* rbClass;
		wxTextCtrl* edYaw;
		wxChoice* cbPointSize;
		wxStaticText* StaticText4;
		wxTextCtrl* edRoll;
		//*)

	protected:

		//(*Identifiers(CDlgPLYOptions)
		static const long ID_CHECKBOX1;
		static const long ID_CHECKBOX2;
		static const long ID_STATICTEXT1;
		static const long ID_CHOICE1;
		static const long ID_STATICTEXT2;
		static const long ID_TEXTCTRL1;
		static const long ID_STATICTEXT3;
		static const long ID_TEXTCTRL2;
		static const long ID_STATICTEXT4;
		static const long ID_TEXTCTRL3;
		static const long ID_RADIOBOX2;
		static const long ID_RADIOBOX1;
		static const long ID_PANEL1;
		static const long ID_BUTTON1;
		static const long ID_BUTTON2;
		//*)

	private:

		//(*Handlers(CDlgPLYOptions)
		void OnbtnCancelClick(wxCommandEvent& event);
		void OnbtnOKClick(wxCommandEvent& event);
		void OnrbClassSelect(wxCommandEvent& event);
		//*)

		DECLARE_EVENT_TABLE()
};

#endif
