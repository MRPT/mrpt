/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CODOMETRYPARAMS_H
#define CODOMETRYPARAMS_H

//(*Headers(COdometryParams)
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/button.h>
#include <wx/dialog.h>
//*)

class COdometryParams: public wxDialog
{
	public:

		COdometryParams(wxWindow* parent,wxWindowID id=wxID_ANY,const wxPoint& pos=wxDefaultPosition,const wxSize& size=wxDefaultSize);
		virtual ~COdometryParams();

		//(*Declarations(COdometryParams)
		wxButton* btnOk;
		wxStaticText* StaticText2;
		wxTextCtrl* edKR;
		wxButton* btnCancel;
		wxStaticText* StaticText6;
		wxTextCtrl* edD;
		wxStaticText* StaticText1;
		wxStaticText* StaticText3;
		wxTextCtrl* edKL;
		wxStaticText* StaticText5;
		wxStaticText* StaticText7;
		wxStaticText* StaticText4;
		//*)

	protected:

		//(*Identifiers(COdometryParams)
		static const long ID_STATICTEXT1;
		static const long ID_STATICTEXT2;
		static const long ID_TEXTCTRL1;
		static const long ID_STATICTEXT3;
		static const long ID_STATICTEXT4;
		static const long ID_TEXTCTRL2;
		static const long ID_STATICTEXT5;
		static const long ID_STATICTEXT6;
		static const long ID_TEXTCTRL3;
		static const long ID_STATICTEXT7;
		static const long ID_BUTTON1;
		static const long ID_BUTTON2;
		//*)

	private:

		//(*Handlers(COdometryParams)
		void OnbtnOkClick(wxCommandEvent& event);
		void OnbtnCancelClick(wxCommandEvent& event);
		//*)

		DECLARE_EVENT_TABLE()
};

#endif
