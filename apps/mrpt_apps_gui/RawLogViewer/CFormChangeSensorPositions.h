/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#ifndef CFORMCHANGESENSORPOSITIONS_H
#define CFORMCHANGESENSORPOSITIONS_H

//(*Headers(CFormChangeSensorPositions)
#include <wx/button.h>
#include <wx/checkbox.h>
#include <wx/combobox.h>
#include <wx/dialog.h>
#include <wx/notebook.h>
#include <wx/panel.h>
#include <wx/radiobox.h>
#include <wx/radiobut.h>
#include <wx/sizer.h>
#include <wx/spinctrl.h>
#include <wx/statline.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
//*)

#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <wx/combobox.h>

class CFormChangeSensorPositions : public wxDialog
{
 public:
  /** Used in executeOperationOnRawlogFiles
   */
  using TRawlogFilter =
      void (*)(mrpt::obs::CActionCollection* acts, mrpt::obs::CSensoryFrame* SF, int& changesCount);

  /** This is the common function for all operations over a rawlog file
   * ("filter" a rawlog file into a new one) or over the loaded rawlog
   * (depending on the user selection in the GUI).
   */
  void executeOperationOnRawlog(TRawlogFilter operation, const char* endMsg);

  CFormChangeSensorPositions(wxWindow* parent, wxWindowID id = wxID_ANY);
  ~CFormChangeSensorPositions() override;

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
  static const wxWindowID ID_RADIOBUTTON1;
  static const wxWindowID ID_RADIOBUTTON2;
  static const wxWindowID ID_STATICTEXT27;
  static const wxWindowID ID_TEXTCTRL16;
  static const wxWindowID ID_BUTTON9;
  static const wxWindowID ID_STATICTEXT28;
  static const wxWindowID ID_TEXTCTRL17;
  static const wxWindowID ID_BUTTON11;
  static const wxWindowID ID_STATICLINE1;
  static const wxWindowID ID_RADIOBOX1;
  static const wxWindowID ID_STATICTEXT15;
  static const wxWindowID ID_SPINCTRL1;
  static const wxWindowID ID_STATICTEXT30;
  static const wxWindowID ID_COMBOBOX1;
  static const wxWindowID ID_STATICTEXT16;
  static const wxWindowID ID_STATICTEXT1;
  static const wxWindowID ID_STATICTEXT2;
  static const wxWindowID ID_STATICTEXT3;
  static const wxWindowID ID_TEXTCTRL1;
  static const wxWindowID ID_STATICTEXT4;
  static const wxWindowID ID_STATICTEXT5;
  static const wxWindowID ID_TEXTCTRL2;
  static const wxWindowID ID_STATICTEXT6;
  static const wxWindowID ID_STATICTEXT7;
  static const wxWindowID ID_TEXTCTRL3;
  static const wxWindowID ID_STATICTEXT8;
  static const wxWindowID ID_STATICTEXT9;
  static const wxWindowID ID_TEXTCTRL4;
  static const wxWindowID ID_STATICTEXT10;
  static const wxWindowID ID_STATICTEXT11;
  static const wxWindowID ID_TEXTCTRL5;
  static const wxWindowID ID_STATICTEXT12;
  static const wxWindowID ID_STATICTEXT13;
  static const wxWindowID ID_TEXTCTRL6;
  static const wxWindowID ID_STATICTEXT14;
  static const wxWindowID ID_CHECKBOX1;
  static const wxWindowID ID_BUTTON3;
  static const wxWindowID ID_BUTTON1;
  static const wxWindowID ID_PANEL1;
  static const wxWindowID ID_STATICTEXT17;
  static const wxWindowID ID_STATICTEXT18;
  static const wxWindowID ID_TEXTCTRL7;
  static const wxWindowID ID_STATICTEXT19;
  static const wxWindowID ID_TEXTCTRL8;
  static const wxWindowID ID_STATICTEXT20;
  static const wxWindowID ID_TEXTCTRL9;
  static const wxWindowID ID_STATICTEXT21;
  static const wxWindowID ID_TEXTCTRL10;
  static const wxWindowID ID_STATICTEXT23;
  static const wxWindowID ID_TEXTCTRL12;
  static const wxWindowID ID_STATICTEXT24;
  static const wxWindowID ID_TEXTCTRL13;
  static const wxWindowID ID_STATICTEXT25;
  static const wxWindowID ID_TEXTCTRL14;
  static const wxWindowID ID_STATICTEXT26;
  static const wxWindowID ID_TEXTCTRL15;
  static const wxWindowID ID_STATICTEXT22;
  static const wxWindowID ID_TEXTCTRL11;
  static const wxWindowID ID_BUTTON4;
  static const wxWindowID ID_BUTTON5;
  static const wxWindowID ID_PANEL2;
  static const wxWindowID ID_NOTEBOOK1;
  static const wxWindowID ID_BUTTON2;
  //*)

 private:
  //(*Handlers(CFormChangeSensorPositions)
  void OnbtnCancelClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnOKClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnGetCurPoseClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnGetCurCamModelClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnGetCurPoseClick1([[maybe_unused]] wxCommandEvent& event);
  void OnbtnApplyCameraParamsClick([[maybe_unused]] wxCommandEvent& event);
  void OnrbLoadedSelect([[maybe_unused]] wxCommandEvent& event);
  void OnrbFileSelect([[maybe_unused]] wxCommandEvent& event);
  void OnbtnPickInputClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnPickOutClick([[maybe_unused]] wxCommandEvent& event);
  void OnInit(wxInitDialogEvent& event);
  void OnrbApplySelect([[maybe_unused]] wxCommandEvent& event);
  //*)

  DECLARE_EVENT_TABLE()
};

#endif
