/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
