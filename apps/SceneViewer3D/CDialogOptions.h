/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CDIALOGOPTIONS_H
#define CDIALOGOPTIONS_H

//(*Headers(CDialogOptions)
#include <wx/notebook.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/checkbox.h>
#include <wx/spinctrl.h>
#include <wx/panel.h>
#include <wx/button.h>
#include <wx/dialog.h>
//*)

class CDialogOptions: public wxDialog
{
	public:

		CDialogOptions(wxWindow* parent,wxWindowID id=wxID_ANY,const wxPoint& pos=wxDefaultPosition,const wxSize& size=wxDefaultSize);
		virtual ~CDialogOptions();

		//(*Declarations(CDialogOptions)
		wxStaticText* StaticText10;
		wxStaticText* StaticText9;
		wxCheckBox* cbFreeCamera;
		wxSpinCtrl* SpinCtrl4;
		wxButton* btnOk;
		wxNotebook* Notebook1;
		wxSpinCtrl* SpinCtrl1;
		wxSpinCtrl* edDelay;
		wxStaticText* StaticText2;
		wxButton* btnCancel;
		wxStaticText* StaticText6;
		wxStaticText* StaticText8;
		wxStaticText* StaticText11;
		wxSpinCtrl* SpinCtrl9;
		wxSpinCtrl* SpinCtrl7;
		wxPanel* Panel1;
		wxStaticText* StaticText1;
		wxStaticText* StaticText3;
		wxPanel* Panel3;
		wxCheckBox* cbFreeCameraNoAzimuth;
		wxSpinCtrl* SpinCtrl3;
		wxStaticText* StaticText5;
		wxStaticText* StaticText7;
		wxSpinCtrl* SpinCtrl2;
		wxSpinCtrl* SpinCtrl5;
		wxCheckBox* cbViewFileName;
		wxPanel* Panel2;
		wxStaticText* StaticText4;
		wxSpinCtrl* SpinCtrl6;
		wxSpinCtrl* SpinCtrl8;
		//*)

	protected:

		//(*Identifiers(CDialogOptions)
		static const long ID_STATICTEXT1;
		static const long ID_SPINCTRL1;
		static const long ID_CHECKBOX1;
		static const long ID_CHECKBOX2;
		static const long ID_CHECKBOX3;
		static const long ID_PANEL1;
		static const long ID_PANEL2;
		static const long ID_STATICTEXT6;
		static const long ID_SPINCTRL2;
		static const long ID_STATICTEXT3;
		static const long ID_SPINCTRL3;
		static const long ID_STATICTEXT7;
		static const long ID_SPINCTRL4;
		static const long ID_STATICTEXT2;
		static const long ID_SPINCTRL6;
		static const long ID_STATICTEXT5;
		static const long ID_SPINCTRL8;
		static const long ID_STATICTEXT4;
		static const long ID_SPINCTRL5;
		static const long ID_STATICTEXT10;
		static const long ID_SPINCTRL10;
		static const long ID_STATICTEXT11;
		static const long ID_SPINCTRL9;
		static const long ID_STATICTEXT8;
		static const long ID_STATICTEXT9;
		static const long ID_SPINCTRL7;
		static const long ID_PANEL3;
		static const long ID_NOTEBOOK1;
		static const long ID_BUTTON1;
		static const long ID_BUTTON2;
		//*)

	private:

		//(*Handlers(CDialogOptions)
		void OnbtnOkClick(wxCommandEvent& event);
		void OnbtnCancelClick(wxCommandEvent& event);
		//*)

		DECLARE_EVENT_TABLE()
};

#endif
