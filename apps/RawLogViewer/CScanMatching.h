/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef CSCANMATCHING_H
#define CSCANMATCHING_H

//(*Headers(CScanMatching)
#include <wx/gauge.h>
#include <wx/bmpbuttn.h>
#include <wx/checkbox.h>
#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/notebook.h>
#include <wx/button.h>
#include <wx/radiobut.h>
#include <wx/panel.h>
#include <wx/splitter.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
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
    wxCheckBox* cbAnimate;
    wxPanel* Panel1;
    wxButton* btnClose;
    wxTextCtrl* edOptAlignMap;
    mpWindow* plotMaps;
    wxTextCtrl* edOptRefPnt;
    wxRadioButton* rbGrid;
    wxBitmapButton* btnHelp;
    wxTextCtrl* edOptICP;
    wxPanel* Panel2;
    wxButton* btnRunICP;
    wxTextCtrl* edFirst;
    wxSplitterWindow* SplitterWindow1;
    wxGauge* pbSteps;
    wxStaticText* StaticText3;
    wxPanel* Panel4;
    wxSplitterWindow* SplitterWindow2;
    wxPanel* Panel5;
    wxPanel* Panel3;
    wxTextCtrl* txtLog;
    wxTextCtrl* edSecond;
    wxStaticText* StaticText4;
    wxStaticText* StaticText5;
    wxStaticText* StaticText2;
    wxNotebook* Notebook1;
    wxStaticText* StaticText6;
    wxRadioButton* rbPoint;
    wxTextCtrl* edOptRefGrid;
    wxStaticText* txtStep;
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
