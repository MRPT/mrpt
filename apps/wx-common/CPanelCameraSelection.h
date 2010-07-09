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
		wxTextCtrl* edVideoFile;
		wxCheckBox* cbBumblebeeRectif;
		wxCheckBox* cbSR_chIntensity;
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
		static const long ID_NOTEBOOK1;
		static const long ID_CHECKBOX2;
		//*)

	private:

		//(*Handlers(CPanelCameraSelection)
		//*)

		DECLARE_EVENT_TABLE()
};

#endif
