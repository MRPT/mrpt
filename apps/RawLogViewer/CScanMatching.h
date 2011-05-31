/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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
