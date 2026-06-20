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
#ifndef CFORMEDIT_H
#define CFORMEDIT_H

#include <wx/listbox.h>

//(*Headers(CFormEdit)
#include <wx/button.h>
#include <wx/checkbox.h>
#include <wx/checklst.h>
#include <wx/dialog.h>
#include <wx/radiobut.h>
#include <wx/sizer.h>
#include <wx/slider.h>
#include <wx/spinctrl.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
//*)

#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <wx/combobox.h>

class CFormEdit : public wxDialog
{
 public:
  CFormEdit(wxWindow* parent, wxWindowID id = -1);
  ~CFormEdit() override;

  /** Used in executeOperationOnRawlogFiles
   */
  using TRawlogFilter =
      void (*)(mrpt::obs::CActionCollection* acts, mrpt::obs::CSensoryFrame* SF, int& changesCount);
  /** This is the common function for all operations over a rawlog file
   * ("filter" a rawlog file into a new one) or over the loaded rawlog
   * (depending on the user selection in the GUI).
   */
  void executeOperationOnRawlog(TRawlogFilter operation, const char* endMsg);

  //(*Identifiers(CFormEdit)
  static const wxWindowID ID_RADIOBUTTON1;
  static const wxWindowID ID_RADIOBUTTON2;
  static const wxWindowID ID_STATICTEXT22;
  static const wxWindowID ID_TEXTCTRL11;
  static const wxWindowID ID_BUTTON9;
  static const wxWindowID ID_STATICTEXT23;
  static const wxWindowID ID_TEXTCTRL12;
  static const wxWindowID ID_BUTTON11;
  static const wxWindowID ID_STATICTEXT1;
  static const wxWindowID ID_SLIDER1;
  static const wxWindowID ID_SPINCTRL1;
  static const wxWindowID ID_STATICTEXT3;
  static const wxWindowID ID_SLIDER2;
  static const wxWindowID ID_SPINCTRL2;
  static const wxWindowID ID_BUTTON1;
  static const wxWindowID ID_BUTTON2;
  static const wxWindowID ID_CHECKBOX1;
  static const wxWindowID ID_CHECKBOX2;
  static const wxWindowID ID_CHECKBOX3;
  static const wxWindowID ID_CHECKBOX4;
  static const wxWindowID ID_CHECKBOX5;
  static const wxWindowID ID_BUTTON4;
  static const wxWindowID ID_CHECKBOX6;
  static const wxWindowID ID_CHECKBOX7;
  static const wxWindowID ID_CHECKBOX8;
  static const wxWindowID ID_CHECKBOX9;
  static const wxWindowID ID_CHECKBOX10;
  static const wxWindowID ID_BUTTON5;
  static const wxWindowID ID_CHECKLISTBOX2;
  static const wxWindowID ID_BUTTON7;
  static const wxWindowID ID_BUTTON8;
  static const wxWindowID ID_CHECKLISTBOX1;
  static const wxWindowID ID_BUTTON10;
  static const wxWindowID ID_BUTTON12;
  static const wxWindowID ID_BUTTON13;
  static const wxWindowID ID_STATICTEXT2;
  static const wxWindowID ID_TEXTCTRL2;
  static const wxWindowID ID_BUTTON3;
  static const wxWindowID ID_BUTTON6;
  //*)

 public:
  //(*Handlers(CFormEdit)
  void OnbtnCloseClick([[maybe_unused]] wxCommandEvent& event);
  void OnslFirstCmdScrollChanged(wxScrollEvent& event);
  void OnslToCmdScrollChanged(wxScrollEvent& event);
  void OnbtnKeepClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnDeleteClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnDelObsIndxClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnRemoveObsClassClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnRemActsIndxClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnImgSwapClick([[maybe_unused]] wxCommandEvent& event);
  void OnInit(wxInitDialogEvent& event);
  void OnrbLoadedSelect([[maybe_unused]] wxCommandEvent& event);
  void OnrbFileSelect([[maybe_unused]] wxCommandEvent& event);
  void OnbtnRemoveAllButByClassClick1([[maybe_unused]] wxCommandEvent& event);
  void OnbtnPickInputClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnPickOutClick([[maybe_unused]] wxCommandEvent& event);
  void OnRemoveByLabel([[maybe_unused]] wxCommandEvent& event);
  void OnRemoveButLabel([[maybe_unused]] wxCommandEvent& event);
  void OnslFromCmdScroll(wxScrollEvent& event);
  void OnslFromCmdScroll1(wxScrollEvent& event);
  void OnbtnLeaveHorizScansClick([[maybe_unused]] wxCommandEvent& event);
  //*)

  //(*Declarations(CFormEdit)
  wxBoxSizer* BoxSizer4;
  wxStaticBoxSizer* StaticBoxSizer2;
  wxStaticText* StaticText22;
  wxFlexGridSizer* FlexGridSizer4;
  wxRadioButton* rbLoaded;
  wxButton* btnLeaveHorizScans;
  wxButton* btnRemoveAllButByClass;
  wxCheckBox* cbO4;
  wxBoxSizer* BoxSizer5;
  wxCheckBox* cbA1;
  wxSpinCtrl* spinFirst;
  wxStaticText* StaticText2;
  wxButton* btnClose;
  wxStaticBoxSizer* StaticBoxSizer4;
  wxFlexGridSizer* FlexGridSizer3;
  wxButton* btnRemByLabel;
  wxTextCtrl* edMaxPitch;
  wxButton* btnDelete;
  wxFlexGridSizer* FlexGridSizer9;
  wxCheckListBox* cbObsLabel;
  wxFlexGridSizer* FlexGridSizer2;
  wxStaticText* StaticText1;
  wxCheckBox* cbA4;
  wxBoxSizer* BoxSizer2;
  wxStaticText* StaticText3;
  wxButton* btnRemByLabelNon;
  wxFlexGridSizer* FlexGridSizer7;
  wxStaticBoxSizer* StaticBoxSizer7;
  wxButton* btnPickInput;
  wxButton* btnPickOut;
  wxSpinCtrl* spinLast;
  wxStaticText* StaticText23;
  wxCheckBox* cbO3;
  wxCheckBox* cbO0;
  wxStaticBoxSizer* StaticBoxSizer3;
  wxStaticBoxSizer* StaticBoxSizer6;
  wxFlexGridSizer* FlexGridSizer8;
  wxButton* btnDelObsIndx;
  wxCheckBox* cbO1;
  wxButton* btnImgSwap;
  wxRadioButton* rbFile;
  wxSlider* slFrom;
  wxSlider* slTo;
  wxBoxSizer* BoxSizer1;
  wxButton* btnRemActsIndx;
  wxCheckBox* cbA0;
  wxCheckBox* cbA3;
  wxFlexGridSizer* FlexGridSizer6;
  wxButton* btnRemoveObsClass;
  wxCheckBox* cbA2;
  wxStaticBoxSizer* StaticBoxSizer1;
  wxFlexGridSizer* FlexGridSizer1;
  wxCheckBox* cbO2;
  wxFlexGridSizer* FlexGridSizer11;
  wxButton* btnKeep;
  wxStaticBoxSizer* StaticBoxSizer5;
  wxTextCtrl* txtInputFile;
  wxCheckListBox* cbObsClass;
  wxTextCtrl* txtOutputFile;
  //*)

 private:
  DECLARE_EVENT_TABLE()
};

#endif
