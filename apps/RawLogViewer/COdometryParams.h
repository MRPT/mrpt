/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#ifndef CODOMETRYPARAMS_H
#define CODOMETRYPARAMS_H

//(*Headers(COdometryParams)
#include <wx/button.h>
#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
//*)

class COdometryParams : public wxDialog
{
 public:
  COdometryParams(
      wxWindow* parent,
      wxWindowID id = wxID_ANY,
      const wxPoint& pos = wxDefaultPosition,
      const wxSize& size = wxDefaultSize);
  ~COdometryParams() override;

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
