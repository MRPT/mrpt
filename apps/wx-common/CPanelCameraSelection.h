/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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
#ifndef CPANELCAMERASELECTION_H
#define CPANELCAMERASELECTION_H

//(*Headers(CPanelCameraSelection)
#include <wx/notebook.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/radiobox.h>
#include <wx/textctrl.h>
#include <wx/checkbox.h>
#include <wx/spinctrl.h>
#include <wx/panel.h>
#include <wx/choice.h>
#include <wx/button.h>
//*)

class CPanelCameraSelection: public wxPanel
{
	public:

		CPanelCameraSelection(wxWindow* parent,wxWindowID id=wxID_ANY);
		virtual ~CPanelCameraSelection();

		//(*Declarations(CPanelCameraSelection)
		wxTextCtrl* edRawlogLabel;
		wxStaticText* StaticText10;
		wxStaticText* StaticText9;
		wxPanel* Panel5;
		wxButton* btnBrowseRawlogDir;
		wxRadioBox* rbBumblebeeSel;
		wxButton* btnBrowseVideo;
		wxStaticText* StaticText2;
		wxPanel* Panel4;
		wxCheckBox* cbKinect_3D;
		wxRadioBox* rbKinect_int;
		wxCheckBox* cbSR_chConf;
		wxStaticText* StaticText6;
		wxSpinCtrl* opencvCamIndex;
		wxTextCtrl* edIPcamURL;
		wxStaticText* StaticText8;
		wxStaticText* StaticText11;
		wxTextCtrl* edCustomCamConfig;
		wxTextCtrl* edSR_IP;
		wxPanel* Panel1;
		wxChoice* cbOpencvCamType;
		wxStaticText* StaticText1;
		wxStaticText* StaticText3;
		wxRadioBox* rbSR_usb;
		wxPanel* Panel6;
		wxButton* btnBrowseRawlog;
		wxPanel* Panel3;
		wxCheckBox* cbGrayscale;
		wxCheckBox* cbSR_chRange;
		wxStaticText* StaticText5;
		wxStaticText* StaticText7;
		wxPanel* pnKinect;
		wxTextCtrl* edVideoFile;
		wxCheckBox* cbBumblebeeRectif;
		wxCheckBox* cbKinect_Int;
		wxCheckBox* cbSR_chIntensity;
		wxCheckBox* cbKinect_Depth;
		wxNotebook* pagesCameras;
		wxPanel* pnSwissRanger;
		wxTextCtrl* edRawlogFile;
		wxTextCtrl* edRawlogImgDir;
		wxPanel* Panel2;
		wxCheckBox* cbSR_ch3D;
		wxStaticText* StaticText4;
		wxChoice* cbOpencvResolution;
		//*)

	protected:

		//(*Identifiers(CPanelCameraSelection)
		static const long ID_STATICTEXT1;
		static const long ID_SPINCTRL1;
		static const long ID_STATICTEXT3;
		static const long ID_CHOICE1;
		static const long ID_STATICTEXT6;
		static const long ID_CHOICE2;
		static const long ID_PANEL2;
		static const long ID_STATICTEXT7;
		static const long ID_TEXTCTRL1;
		static const long ID_PANEL3;
		static const long ID_TEXTCTRL6;
		static const long ID_PANEL4;
		static const long ID_STATICTEXT8;
		static const long ID_TEXTCTRL2;
		static const long ID_BUTTON7;
		static const long ID_PANEL5;
		static const long ID_STATICTEXT9;
		static const long ID_TEXTCTRL3;
		static const long ID_BUTTON8;
		static const long ID_STATICTEXT5;
		static const long ID_TEXTCTRL7;
		static const long ID_BUTTON9;
		static const long ID_STATICTEXT10;
		static const long ID_TEXTCTRL8;
		static const long ID_STATICTEXT11;
		static const long ID_PANEL6;
		static const long ID_RADIOBOX1;
		static const long ID_CHECKBOX1;
		static const long ID_STATICTEXT2;
		static const long ID_PANEL7;
		static const long ID_RADIOBOX2;
		static const long ID_STATICTEXT4;
		static const long ID_TEXTCTRL4;
		static const long ID_CHECKBOX3;
		static const long ID_CHECKBOX4;
		static const long ID_CHECKBOX5;
		static const long ID_CHECKBOX6;
		static const long ID_PANEL1;
		static const long ID_CHECKBOX7;
		static const long ID_CHECKBOX8;
		static const long ID_CHECKBOX9;
		static const long ID_RADIOBOX3;
		static const long ID_PANEL8;
		static const long ID_NOTEBOOK1;
		static const long ID_CHECKBOX2;
		//*)

	private:

		//(*Handlers(CPanelCameraSelection)
		void OnpagesCamerasPageChanged(wxNotebookEvent& event);
		//*)

		DECLARE_EVENT_TABLE()
};

#endif
