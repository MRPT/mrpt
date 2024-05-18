/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#ifndef CABOUTBOX_H
#define CABOUTBOX_H

//(*Headers(CAboutBox)
#include <wx/button.h>
#include <wx/dialog.h>
#include <wx/notebook.h>
#include <wx/sizer.h>
#include <wx/statbmp.h>
#include <wx/statline.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
//*)

class CAboutBox : public wxDialog
{
 public:
  CAboutBox(wxWindow* parent, wxWindowID id = -1);
  ~CAboutBox() override;

  //(*Identifiers(CAboutBox)
  static const long ID_STATICTEXT1;
  static const long ID_STATICTEXT2;
  static const long ID_STATICBITMAP1;
  static const long ID_STATICLINE1;
  static const long ID_TEXTCTRL4;
  static const long ID_TEXTCTRL1;
  static const long ID_TEXTCTRL2;
  static const long ID_TEXTCTRL3;
  static const long ID_NOTEBOOK1;
  static const long ID_BUTTON1;
  //*)

 protected:
  //(*Handlers(CAboutBox)
  void OnInit(wxInitDialogEvent& event);
  void OnButton1Click(wxCommandEvent& event);
  //*)

  //(*Declarations(CAboutBox)
  wxFlexGridSizer* FlexGridSizer4;
  wxNotebook* Notebook1;
  wxStaticText* lbProgName;
  wxButton* Button11;
  wxStaticBitmap* StaticBitmap1;
  wxTextCtrl* lbLicense;
  wxTextCtrl* lbInfo;
  wxStaticText* lbBuild;
  wxStaticLine* StaticLine1;
  wxTextCtrl* TextCtrl1;
  wxFlexGridSizer* FlexGridSizer1;
  //*)

 private:
  DECLARE_EVENT_TABLE()
};

#endif
