/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CDIALOGOPTIONS_H
#define CDIALOGOPTIONS_H

//(*Headers(CDialogOptions)
#include <wx/spinctrl.h>
#include <wx/checkbox.h>
#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/notebook.h>
#include <wx/button.h>
#include <wx/panel.h>
#include <wx/stattext.h>
//*)

class CDialogOptions: public wxDialog
{
	public:

		CDialogOptions(wxWindow* parent,wxWindowID id=wxID_ANY,const wxPoint& pos=wxDefaultPosition,const wxSize& size=wxDefaultSize);
		virtual ~CDialogOptions();

		//(*Declarations(CDialogOptions)
		wxCheckBox* cbFreeCameraNoAzimuth;
		wxPanel* Panel1;
		wxSpinCtrl* SpinCtrl4;
		wxSpinCtrl* SpinCtrl7;
		wxCheckBox* cbFreeCamera;
		wxStaticText* StaticText1;
		wxStaticText* StaticText10;
		wxPanel* Panel2;
		wxStaticText* StaticText3;
		wxButton* btnOk;
		wxSpinCtrl* SpinCtrl3;
		wxStaticText* StaticText8;
		wxSpinCtrl* SpinCtrl5;
		wxPanel* Panel3;
		wxStaticText* StaticText7;
		wxSpinCtrl* SpinCtrl2;
		wxStaticText* StaticText4;
		wxStaticText* StaticText5;
		wxStaticText* StaticText2;
		wxNotebook* Notebook1;
		wxStaticText* StaticText6;
		wxSpinCtrl* SpinCtrl1;
		wxButton* btnCancel;
		wxSpinCtrl* SpinCtrl8;
		wxSpinCtrl* edDelay;
		wxSpinCtrl* SpinCtrl6;
		wxCheckBox* cbViewFileName;
		wxSpinCtrl* SpinCtrl9;
		wxStaticText* StaticText9;
		wxStaticText* StaticText11;
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
