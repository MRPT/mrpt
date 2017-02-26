/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CABOUTBOX_H
#define CABOUTBOX_H

//(*Headers(CAboutBox)
#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/notebook.h>
#include <wx/button.h>
#include <wx/statline.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/statbmp.h>
//*)

class CAboutBox: public wxDialog
{
public:

    CAboutBox(wxWindow* parent,wxWindowID id = -1);
    virtual ~CAboutBox();

    //(*Identifiers(CAboutBox)
    static const long ID_STATICTEXT1;
    static const long ID_STATICTEXT2;
    static const long ID_STATICBITMAP1;
    static const long ID_STATICLINE1;
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
    void OnChar(wxKeyEvent& event);
    //*)

    //(*Declarations(CAboutBox)
    wxStaticText* lbProgName;
    wxFlexGridSizer* FlexGridSizer1;
    wxTextCtrl* lbLicense;
    wxButton* Button1;
    wxFlexGridSizer* FlexGridSizer4;
    wxStaticLine* StaticLine1;
    wxStaticText* lbBuild;
    wxTextCtrl* TextCtrl1;
    wxNotebook* Notebook1;
    wxTextCtrl* lbInfo;
    wxStaticBitmap* StaticBitmap1;
    //*)

private:

    DECLARE_EVENT_TABLE()
};

#endif
