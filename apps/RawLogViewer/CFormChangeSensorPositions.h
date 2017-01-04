/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CFORMCHANGESENSORPOSITIONS_H
#define CFORMCHANGESENSORPOSITIONS_H

//(*Headers(CFormChangeSensorPositions)
#include <wx/notebook.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/radiobox.h>
#include <wx/textctrl.h>
#include <wx/checkbox.h>
#include <wx/spinctrl.h>
#include <wx/statline.h>
#include <wx/radiobut.h>
#include <wx/panel.h>
#include <wx/button.h>
#include <wx/dialog.h>
#include <wx/combobox.h>
//*)

#include <wx/combobox.h>

#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>



class CFormChangeSensorPositions: public wxDialog
{
	public:

		/** Used in executeOperationOnRawlogFiles
		  */
		typedef void (*TRawlogFilter)( mrpt::obs::CActionCollection *acts, mrpt::obs::CSensoryFrame *SF, int &changesCount  );

		/** This is the common function for all operations over a rawlog file ("filter" a rawlog file into a new one) or over the loaded rawlog (depending on the user selection in the GUI).
		  */
		void executeOperationOnRawlog( TRawlogFilter operation, const char *endMsg);

		CFormChangeSensorPositions(wxWindow* parent,wxWindowID id=wxID_ANY);
		virtual ~CFormChangeSensorPositions();


		//(*Declarations(CFormChangeSensorPositions)
		wxStaticText* StaticText10;
		wxStaticText* StaticText22;
		wxStaticText* StaticText9;
		wxButton* btnGetCurPose;
		wxStaticText* StaticText20;
		wxRadioButton* rbLoaded;
		wxRadioBox* rbApply;
		wxTextCtrl* edFX;
		wxNotebook* Notebook1;
		wxStaticText* StaticText13;
		wxStaticText* StaticText2;
		wxStaticText* StaticText14;
		wxStaticText* StaticText30;
		wxButton* btnCancel;
		wxStaticText* StaticText26;
		wxStaticText* StaticText6;
		wxButton* btnGetCurCamModel;
		wxStaticText* StaticText19;
		wxTextCtrl* edPitch;
		wxComboBox* edLabel;
		wxStaticText* StaticText8;
		wxStaticText* StaticText11;
		wxStaticText* StaticText18;
		wxTextCtrl* edX;
		wxPanel* Panel1;
		wxTextCtrl* edK2;
		wxStaticText* StaticText1;
		wxStaticText* StaticText27;
		wxStaticText* StaticText3;
		wxButton* btnOK;
		wxTextCtrl* edZ;
		wxSpinCtrl* scIndex;
		wxButton* btnPickInput;
		wxTextCtrl* edY;
		wxTextCtrl* edFY;
		wxButton* btnPickOut;
		wxTextCtrl* edP1;
		wxStaticText* StaticText21;
		wxStaticText* StaticText23;
		wxStaticText* StaticText24;
		wxButton* btnApplyCameraParams;
		wxStaticText* StaticText5;
		wxStaticText* StaticText7;
		wxTextCtrl* edYaw;
		wxStaticLine* StaticLine1;
		wxFlexGridSizer* FlexGridSizer14;
		wxRadioButton* rbFile;
		wxStaticText* StaticText28;
		wxStaticText* StaticText15;
		wxStaticText* StaticText12;
		wxTextCtrl* edFocalLen;
		wxPanel* Panel2;
		wxStaticText* StaticText25;
		wxStaticText* StaticText17;
		wxStaticText* StaticText4;
		wxTextCtrl* edCY;
		wxTextCtrl* edCX;
		wxTextCtrl* edP2;
		wxTextCtrl* edK1;
		wxTextCtrl* edRoll;
		wxCheckBox* cbOnlyXYZ;
		wxTextCtrl* txtInputFile;
		wxStaticText* StaticText16;
		wxTextCtrl* txtOutputFile;
		//*)

	protected:

		//(*Identifiers(CFormChangeSensorPositions)
		static const long ID_RADIOBUTTON1;
		static const long ID_RADIOBUTTON2;
		static const long ID_STATICTEXT27;
		static const long ID_TEXTCTRL16;
		static const long ID_BUTTON9;
		static const long ID_STATICTEXT28;
		static const long ID_TEXTCTRL17;
		static const long ID_BUTTON11;
		static const long ID_STATICLINE1;
		static const long ID_RADIOBOX1;
		static const long ID_STATICTEXT15;
		static const long ID_SPINCTRL1;
		static const long ID_STATICTEXT30;
		static const long ID_COMBOBOX1;
		static const long ID_STATICTEXT16;
		static const long ID_STATICTEXT1;
		static const long ID_STATICTEXT2;
		static const long ID_STATICTEXT3;
		static const long ID_TEXTCTRL1;
		static const long ID_STATICTEXT4;
		static const long ID_STATICTEXT5;
		static const long ID_TEXTCTRL2;
		static const long ID_STATICTEXT6;
		static const long ID_STATICTEXT7;
		static const long ID_TEXTCTRL3;
		static const long ID_STATICTEXT8;
		static const long ID_STATICTEXT9;
		static const long ID_TEXTCTRL4;
		static const long ID_STATICTEXT10;
		static const long ID_STATICTEXT11;
		static const long ID_TEXTCTRL5;
		static const long ID_STATICTEXT12;
		static const long ID_STATICTEXT13;
		static const long ID_TEXTCTRL6;
		static const long ID_STATICTEXT14;
		static const long ID_CHECKBOX1;
		static const long ID_BUTTON3;
		static const long ID_BUTTON1;
		static const long ID_PANEL1;
		static const long ID_STATICTEXT17;
		static const long ID_STATICTEXT18;
		static const long ID_TEXTCTRL7;
		static const long ID_STATICTEXT19;
		static const long ID_TEXTCTRL8;
		static const long ID_STATICTEXT20;
		static const long ID_TEXTCTRL9;
		static const long ID_STATICTEXT21;
		static const long ID_TEXTCTRL10;
		static const long ID_STATICTEXT23;
		static const long ID_TEXTCTRL12;
		static const long ID_STATICTEXT24;
		static const long ID_TEXTCTRL13;
		static const long ID_STATICTEXT25;
		static const long ID_TEXTCTRL14;
		static const long ID_STATICTEXT26;
		static const long ID_TEXTCTRL15;
		static const long ID_STATICTEXT22;
		static const long ID_TEXTCTRL11;
		static const long ID_BUTTON4;
		static const long ID_BUTTON5;
		static const long ID_PANEL2;
		static const long ID_NOTEBOOK1;
		static const long ID_BUTTON2;
		//*)

	private:

		//(*Handlers(CFormChangeSensorPositions)
		void OnbtnCancelClick(wxCommandEvent& event);
		void OnbtnOKClick(wxCommandEvent& event);
		void OnbtnGetCurPoseClick(wxCommandEvent& event);
		void OnbtnGetCurCamModelClick(wxCommandEvent& event);
		void OnbtnGetCurPoseClick1(wxCommandEvent& event);
		void OnbtnApplyCameraParamsClick(wxCommandEvent& event);
		void OnrbLoadedSelect(wxCommandEvent& event);
		void OnrbFileSelect(wxCommandEvent& event);
		void OnbtnPickInputClick(wxCommandEvent& event);
		void OnbtnPickOutClick(wxCommandEvent& event);
		void OnInit(wxInitDialogEvent& event);
		void OnrbApplySelect(wxCommandEvent& event);
		//*)

		DECLARE_EVENT_TABLE()
};

#endif
