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
#ifndef CFORMBATCHSENSORPOSE_H
#define CFORMBATCHSENSORPOSE_H

//(*Headers(CFormBatchSensorPose)
#include <wx/bmpbuttn.h>
#include <wx/button.h>
#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
//*)

class CFormBatchSensorPose : public wxDialog
{
 public:
  CFormBatchSensorPose(wxWindow* parent, wxWindowID id = wxID_ANY);
  ~CFormBatchSensorPose() override;

  //(*Declarations(CFormBatchSensorPose)
  wxStaticText* StaticText1;
  wxButton* btnApply;
  wxBitmapButton* btnOpen;
  wxButton* btnCancel;
  wxTextCtrl* edText;
  //*)

 protected:
  //(*Identifiers(CFormBatchSensorPose)
  static const wxWindowID ID_STATICTEXT1;
  static const wxWindowID ID_TEXTCTRL1;
  static const wxWindowID ID_BITMAPBUTTON1;
  static const wxWindowID ID_BUTTON1;
  static const wxWindowID ID_BUTTON2;
  //*)

 private:
  //(*Handlers(CFormBatchSensorPose)
  void OnbtnOpenClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnApplyClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnCancelClick([[maybe_unused]] wxCommandEvent& event);
  //*)

  DECLARE_EVENT_TABLE()
};

#endif
