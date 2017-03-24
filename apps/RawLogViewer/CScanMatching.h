/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CSCANMATCHING_H
#define CSCANMATCHING_H

//(*Headers(CScanMatching)
#include <wx/notebook.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/checkbox.h>
#include <wx/splitter.h>
#include <wx/radiobut.h>
#include <wx/panel.h>
#include <wx/bmpbuttn.h>
#include <wx/button.h>
#include <wx/dialog.h>
#include <wx/gauge.h>
//*)

// The "custom class" mpWindow, from the wxMathPlot libray by David Schalig
//  See http://sourceforge.net/projects/wxmathplot
#include <mrpt/otherlibs/mathplot/mathplot.h>


class CScanMatching: public wxDialog
{
public:

    CScanMatching(wxWindow* parent,wxWindowID id=wxID_ANY);
    virtual ~CScanMatching();

    //(*Declarations(CScanMatching)
    wxTextCtrl* edOptRefGrid;
    wxGauge* pbSteps;
    wxPanel* Panel5;
    wxNotebook* Notebook1;
    mpWindow* plotMaps;
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
    //*)

protected:

    //(*Identifiers(CScanMatching)
    static const long ID_STATICTEXT2;
    static const long ID_BITMAPBUTTON1;
    static const long ID_STATICTEXT3;
    static const long ID_TEXTCTRL2;
    static const long ID_STATICTEXT4;
    static const long ID_TEXTCTRL3;
    static const long ID_TEXTCTRL6;
    static const long ID_STATICTEXT5;
    static const long ID_RADIOBUTTON1;
    static const long ID_RADIOBUTTON2;
    static const long ID_TEXTCTRL4;
    static const long ID_TEXTCTRL9;
    static const long ID_NOTEBOOK1;
    static const long ID_TEXTCTRL7;
    static const long ID_PANEL1;
    static const long ID_STATICTEXT6;
    static const long ID_CUSTOM1;
    static const long ID_BUTTON1;
    static const long ID_CHECKBOX1;
    static const long ID_BUTTON2;
    static const long ID_STATICTEXT1;
    static const long ID_GAUGE1;
    static const long ID_PANEL5;
    static const long ID_PANEL3;
    static const long ID_TEXTCTRL1;
    static const long ID_PANEL4;
    static const long ID_SPLITTERWINDOW2;
    static const long ID_PANEL2;
    static const long ID_SPLITTERWINDOW1;
    //*)

private:

    //(*Handlers(CScanMatching)
    void OnInit(wxInitDialogEvent& event);
    void OnbtnICPClick(wxCommandEvent& event);
    void OncbAnimateClick(wxCommandEvent& event);
    void OChangeSelectedMapType(wxCommandEvent& event);
    void OnNotebook1PageChanging(wxNotebookEvent& event);
    void OnbtnCloseClick(wxCommandEvent& event);
    void OnbtnHelpClick(wxCommandEvent& event);
    //*)

    DECLARE_EVENT_TABLE()
};

#endif
