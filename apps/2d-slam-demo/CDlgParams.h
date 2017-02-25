/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CDLGPARAMS_H
#define CDLGPARAMS_H

//(*Headers(CDlgParams)
#include <wx/spinctrl.h>
#include <wx/checkbox.h>
#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/button.h>
#include <wx/radiobut.h>
#include <wx/panel.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/radiobox.h>
//*)

class CDlgParams: public wxDialog
{
	public:

		CDlgParams(wxWindow* parent,wxWindowID id=wxID_ANY,const wxPoint& pos=wxDefaultPosition,const wxSize& size=wxDefaultSize);
		virtual ~CDlgParams();

		//(*Declarations(CDlgParams)
		wxStaticText* StaticText24;
		wxStaticText* StaticText22;
		wxSpinCtrl* edOverSensor;
		wxTextCtrl* edMapFile;
		wxTextCtrl* edPathStepSize;
		wxStaticText* StaticText21;
		wxRadioButton* rbMapRandom;
		wxStaticText* StaticText13;
		wxStaticText* StaticText14;
		wxCheckBox* cbJacobTran;
		wxStaticText* StaticText15;
		wxTextCtrl* edSpuriousStd;
		wxTextCtrl* edMinR;
		wxRadioButton* rbKFdavison;
		wxStaticText* StaticText17;
		wxRadioBox* rbDAMetric;
		wxStaticText* StaticText28;
		wxRadioButton* rbKFnaiv;
		wxSpinCtrl* edIKFiters;
		wxTextCtrl* edOdomStdXY;
		wxCheckBox* cbJacobObs;
		wxRadioButton* rbIKF;
		wxStaticText* StaticText20;
		wxStaticText* StaticText18;
		wxStaticText* StaticText1;
		wxStaticText* StaticText10;
		wxStaticText* StaticText16;
		wxSpinCtrl* edLMs;
		wxStaticText* StaticText3;
		wxButton* btnOk;
		wxStaticText* StaticText23;
		wxButton* btnBrowse;
		wxSpinCtrl* edSeed;
		wxRadioButton* rbIKFdavison;
		wxSpinCtrl* edOverOdom;
		wxPanel* panelDA;
		wxTextCtrl* edMaxR;
		wxStaticText* StaticText8;
		wxStaticText* StaticText12;
		wxTextCtrl* edPathLen;
		wxTextCtrl* edStdAngle;
		wxStaticText* StaticText7;
		wxStaticText* StaticText4;
		wxTextCtrl* edSenX;
		wxStaticText* StaticText5;
		wxStaticText* StaticText2;
		wxTextCtrl* edSpuriousMean;
		wxStaticText* StaticText27;
		wxTextCtrl* edChi2;
		wxRadioButton* rbMapFile;
		wxStaticText* StaticText26;
		wxStaticText* StaticText6;
		wxTextCtrl* edICMLrefDist;
		wxButton* btnCancel;
		wxRadioBox* rbDAMethod;
		wxTextCtrl* edStdRange;
		wxRadioBox* rbICmetric;
		wxTextCtrl* edStdOdomPhi;
		wxStaticText* StaticText19;
		wxTextCtrl* edSenY;
		wxTextCtrl* edSenPhi;
		wxCheckBox* cbSensorDistin;
		wxStaticText* StaticText9;
		wxTextCtrl* edFOV;
		wxStaticText* StaticText11;
		wxRadioButton* rbMapCorridor;
		wxStaticText* StaticText25;
		//*)

	protected:

		//(*Identifiers(CDlgParams)
		static const long ID_RADIOBUTTON1;
		static const long ID_RADIOBUTTON2;
		static const long ID_RADIOBUTTON3;
		static const long ID_STATICTEXT2;
		static const long ID_SPINCTRL1;
		static const long ID_RADIOBUTTON4;
		static const long ID_STATICTEXT25;
		static const long ID_CHECKBOX2;
		static const long ID_CHECKBOX3;
		static const long ID_STATICTEXT3;
		static const long ID_TEXTCTRL3;
		static const long ID_STATICTEXT4;
		static const long ID_TEXTCTRL4;
		static const long ID_STATICTEXT5;
		static const long ID_TEXTCTRL5;
		static const long ID_STATICTEXT6;
		static const long ID_TEXTCTRL6;
		static const long ID_RADIOBOX3;
		static const long ID_STATICTEXT17;
		static const long ID_TEXTCTRL13;
		static const long ID_STATICTEXT24;
		static const long ID_TEXTCTRL15;
		static const long ID_RADIOBOX1;
		static const long ID_RADIOBOX2;
		static const long ID_PANEL1;
		static const long ID_RADIOBUTTON5;
		static const long ID_STATICTEXT9;
		static const long ID_RADIOBUTTON6;
		static const long ID_STATICTEXT7;
		static const long ID_SPINCTRL2;
		static const long ID_STATICTEXT8;
		static const long ID_SPINCTRL3;
		static const long ID_RADIOBUTTON7;
		static const long ID_TEXTCTRL2;
		static const long ID_BUTTON3;
		static const long ID_CHECKBOX1;
		static const long ID_STATICTEXT11;
		static const long ID_TEXTCTRL7;
		static const long ID_STATICTEXT1;
		static const long ID_TEXTCTRL1;
		static const long ID_STATICTEXT12;
		static const long ID_TEXTCTRL8;
		static const long ID_STATICTEXT23;
		static const long ID_TEXTCTRL14;
		static const long ID_STATICTEXT10;
		static const long ID_TEXTCTRL12;
		static const long ID_STATICTEXT13;
		static const long ID_STATICTEXT14;
		static const long ID_STATICTEXT15;
		static const long ID_STATICTEXT16;
		static const long ID_TEXTCTRL9;
		static const long ID_TEXTCTRL10;
		static const long ID_TEXTCTRL11;
		static const long ID_STATICTEXT26;
		static const long ID_STATICTEXT27;
		static const long ID_STATICTEXT28;
		static const long ID_TEXTCTRL16;
		static const long ID_TEXTCTRL17;
		static const long ID_STATICTEXT18;
		static const long ID_SPINCTRL4;
		static const long ID_STATICTEXT19;
		static const long ID_STATICTEXT20;
		static const long ID_SPINCTRL5;
		static const long ID_STATICTEXT21;
		static const long ID_STATICTEXT22;
		static const long ID_BUTTON1;
		static const long ID_BUTTON2;
		//*)

	//private:
	public:

		//(*Handlers(CDlgParams)
		void OnbtnOkClick(wxCommandEvent& event);
		void OnbtnCancelClick(wxCommandEvent& event);
		void OnbtnBrowseClick(wxCommandEvent& event);
		void OnrbIKFdavisonSelect(wxCommandEvent& event);
		void OnUpdateControlsState(wxCommandEvent& event);
		//*)

		DECLARE_EVENT_TABLE()
};

#endif
