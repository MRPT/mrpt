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
#ifndef CINIEDITOR_H
#define CINIEDITOR_H

//(*Headers(CIniEditor)
#include <wx/button.h>
#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/textctrl.h>
//*)

class CIniEditor : public wxDialog
{
 public:
  CIniEditor(
      wxWindow* parent,
      wxWindowID id = wxID_ANY,
      const wxPoint& pos = wxDefaultPosition,
      const wxSize& size = wxDefaultSize);
  ~CIniEditor() override;

  //(*Declarations(CIniEditor)
  wxButton* btnCancel;
  wxTextCtrl* edText;
  wxButton* btnOK;
  //*)

 protected:
  //(*Identifiers(CIniEditor)
  static const wxWindowID ID_BUTTON1;
  static const wxWindowID ID_BUTTON2;
  static const wxWindowID ID_TEXTCTRL1;
  //*)

 private:
  //(*Handlers(CIniEditor)
  void OnbtnCancelClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnOKClick([[maybe_unused]] wxCommandEvent& event);
  //*)

  DECLARE_EVENT_TABLE()
};

#endif
