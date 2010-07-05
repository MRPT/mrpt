/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */
#ifndef CPANELCAMERASELECTION_H
#define CPANELCAMERASELECTION_H

//(*Headers(CPanelCameraSelection)
#include <wx/spinctrl.h>
#include <wx/checkbox.h>
#include <wx/sizer.h>
#include <wx/notebook.h>
#include <wx/button.h>
#include <wx/panel.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/choice.h>
#include <wx/radiobox.h>
//*)

class CPanelCameraSelection: public wxPanel
{
	public:

		CPanelCameraSelection(wxWindow* parent,wxWindowID id=wxID_ANY);
		virtual ~CPanelCameraSelection();

		//(*Declarations(CPanelCameraSelection)
		wxNotebook* pagesCameras;
		wxPanel* Panel1;
		wxPanel* Panel6;
		wxPanel* pnSwissRanger;
		wxTextCtrl* edCustomCamConfig;
		wxCheckBox* cbSR_chRange;
		wxCheckBox* cbSR_chIntensity;
		wxTextCtrl* edIPcamURL;
		wxCheckBox* cbSR_ch3D;
		wxRadioBox* rbSR_usb;
		wxStaticText* StaticText1;
		wxStaticText* StaticText10;
		wxButton* btnBrowseVideo;
		wxPanel* Panel2;
		wxStaticText* StaticText3;
		wxPanel* Panel4;
		wxTextCtrl* edRawlogFile;
		wxButton* btnBrowseRawlog;
		wxChoice* cbOpencvResolution;
		wxPanel* Panel5;
		wxStaticText* StaticText8;
		wxPanel* Panel3;
		wxStaticText* StaticText7;
		wxCheckBox* cbGrayscale;
		wxChoice* cbOpencvCamType;
		wxRadioBox* rbBumblebeeSel;
		wxTextCtrl* edRawlogLabel;
		wxStaticText* StaticText4;
		wxCheckBox* cbSR_chConf;
		wxStaticText* StaticText5;
		wxStaticText* StaticText2;
		wxTextCtrl* edSR_IP;
		wxStaticText* StaticText6;
		wxCheckBox* cbBumblebeeRectif;
		wxButton* btnBrowseRawlogDir;
		wxTextCtrl* edRawlogImgDir;
		wxSpinCtrl* opencvCamIndex;
		wxStaticText* StaticText9;
		wxStaticText* StaticText11;
		wxTextCtrl* edVideoFile;
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
		static const long ID_NOTEBOOK1;
		static const long ID_CHECKBOX2;
		//*)

	private:

		//(*Handlers(CPanelCameraSelection)
		//*)

		DECLARE_EVENT_TABLE()
};

#endif
