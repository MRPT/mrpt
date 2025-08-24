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

#ifndef CLOGVIEW_H
#define CLOGVIEW_H

//(*Headers(CLogView)
#include <wx/button.h>
#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/textctrl.h>
//*)

class CLogView : public wxDialog
{
 public:
  CLogView(
      wxWindow* parent,
      wxWindowID id = wxID_ANY,
      const wxPoint& pos = wxDefaultPosition,
      const wxSize& size = wxDefaultSize);
  ~CLogView() override;

  //(*Declarations(CLogView)
  wxButton* btnOk;
  wxTextCtrl* edLog;
  //*)

 protected:
  //(*Identifiers(CLogView)
  static const long ID_TEXTCTRL1;
  static const long ID_BUTTON1;
  //*)

 private:
  //(*Handlers(CLogView)
  void OnbtnOkClick(wxCommandEvent& event);
  //*)

  DECLARE_EVENT_TABLE()
};

#endif
