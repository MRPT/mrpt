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
#ifndef CSCANMATCHING_H
#define CSCANMATCHING_H

//(*Headers(CScanMatching)
#include <wx/bmpbuttn.h>
#include <wx/button.h>
#include <wx/checkbox.h>
#include <wx/dialog.h>
#include <wx/gauge.h>
#include <wx/notebook.h>
#include <wx/panel.h>
#include <wx/radiobut.h>
#include <wx/sizer.h>
#include <wx/splitter.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>

#include "MyGLCanvas.h"
//*)

#include <mrpt/viz/CSetOfObjects.h>

class CScanMatching : public wxDialog
{
 public:
  CScanMatching(wxWindow* parent, wxWindowID id = wxID_ANY);
  ~CScanMatching() override;

  //(*Declarations(CScanMatching)
  wxTextCtrl* edOptRefGrid;
  wxGauge* pbSteps;
  wxPanel* Panel5;
  wxNotebook* Notebook1;
  wxTextCtrl* edOptRefPnt;
  wxStaticText* StaticText2;
  wxPanel* Panel4;
  wxButton* btnClose;
  wxTextCtrl* edOptICP;
  wxStaticText* StaticText6;
  wxTextCtrl* edSecond;
  wxTextCtrl* edFirst;
  wxSplitterWindow* SplitterWindow2;
  wxCheckBox* cbAnimate;
  wxTextCtrl* edOptAlignMap;
  wxBitmapButton* btnHelp;
  wxPanel* Panel1;
  wxTextCtrl* txtLog;
  wxStaticText* StaticText3;
  wxRadioButton* rbGrid;
  wxPanel* Panel3;
  wxStaticText* StaticText5;
  wxButton* btnRunICP;
  wxRadioButton* rbPoint;
  wxPanel* Panel2;
  wxStaticText* txtStep;
  wxSplitterWindow* SplitterWindow1;
  wxStaticText* StaticText4;
  CMyGLCanvas* m_plot3D;
  //*)

 protected:
  //(*Identifiers(CScanMatching)
  static const wxWindowID ID_STATICTEXT2;
  static const wxWindowID ID_BITMAPBUTTON1;
  static const wxWindowID ID_STATICTEXT3;
  static const wxWindowID ID_TEXTCTRL2;
  static const wxWindowID ID_STATICTEXT4;
  static const wxWindowID ID_TEXTCTRL3;
  static const wxWindowID ID_TEXTCTRL6;
  static const wxWindowID ID_STATICTEXT5;
  static const wxWindowID ID_RADIOBUTTON1;
  static const wxWindowID ID_RADIOBUTTON2;
  static const wxWindowID ID_TEXTCTRL4;
  static const wxWindowID ID_TEXTCTRL9;
  static const wxWindowID ID_NOTEBOOK1;
  static const wxWindowID ID_TEXTCTRL7;
  static const wxWindowID ID_PANEL1;
  static const wxWindowID ID_STATICTEXT6;
  static const wxWindowID ID_XY_GLCANVAS;
  static const wxWindowID ID_BUTTON1;
  static const wxWindowID ID_CHECKBOX1;
  static const wxWindowID ID_BUTTON2;
  static const wxWindowID ID_STATICTEXT1;
  static const wxWindowID ID_GAUGE1;
  static const wxWindowID ID_PANEL5;
  static const wxWindowID ID_PANEL3;
  static const wxWindowID ID_TEXTCTRL1;
  static const wxWindowID ID_PANEL4;
  static const wxWindowID ID_SPLITTERWINDOW2;
  static const wxWindowID ID_PANEL2;
  static const wxWindowID ID_SPLITTERWINDOW1;
  //*)

 private:
  //(*Handlers(CScanMatching)
  void OnInit(wxInitDialogEvent& event);
  void OnbtnICPClick([[maybe_unused]] wxCommandEvent& event);
  void OncbAnimateClick([[maybe_unused]] wxCommandEvent& event);
  void OChangeSelectedMapType([[maybe_unused]] wxCommandEvent& event);
  void OnNotebook1PageChanging(wxNotebookEvent& event);
  void OnbtnCloseClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnHelpClick([[maybe_unused]] wxCommandEvent& event);
  //*)

  DECLARE_EVENT_TABLE()

  mrpt::viz::CSetOfObjects::Ptr m_gl_map_ref, m_gl_map_new;
};

#endif
