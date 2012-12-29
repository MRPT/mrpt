/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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
#ifndef CFORMRAWMAP_H
#define CFORMRAWMAP_H

//(*Headers(CFormRawMap)
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/spinctrl.h>
#include <wx/slider.h>
#include <wx/panel.h>
#include <wx/bmpbuttn.h>
#include <wx/button.h>
#include <wx/dialog.h>
//*)

// The "custom class" mpWindow, from the wxMathPlot libray by David Schalig
//  See http://sourceforge.net/projects/wxmathplot
#include <mrpt/otherlibs/mathplot/mathplot.h>

#include <mrpt/gui/CDisplayWindow3D.h>


class CFormRawMap: public wxDialog
{
public:

    CFormRawMap(wxWindow* parent,wxWindowID id = -1);
    virtual ~CFormRawMap();

    //(*Identifiers(CFormRawMap)
    static const long ID_STATICTEXT7;
    static const long ID_STATICTEXT6;
    static const long ID_STATICTEXT5;
    static const long ID_STATICTEXT1;
    static const long ID_SLIDER1;
    static const long ID_SPINCTRL1;
    static const long ID_STATICTEXT3;
    static const long ID_SLIDER2;
    static const long ID_SPINCTRL2;
    static const long ID_STATICTEXT10;
    static const long ID_SLIDER3;
    static const long ID_SPINCTRL3;
    static const long ID_BUTTON2;
    static const long ID_BUTTON6;
    static const long ID_BUTTON5;
    static const long ID_BUTTON1;
    static const long ID_BUTTON3;
    static const long ID_BUTTON7;
    static const long ID_BUTTON8;
    static const long ID_BUTTON9;
    static const long ID_STATICTEXT8;
    static const long ID_STATICTEXT2;
    static const long ID_BITMAPBUTTON1;
    static const long ID_BUTTON4;
    static const long ID_TEXTCTRL1;
    static const long ID_PANEL1;
    static const long ID_CUSTOM2;
    static const long ID_PANEL3;
    //*)

public:

    //(*Handlers(CFormRawMap)
    void OnslFromCmdScrollThumbTrack(wxScrollEvent& event);
    void OnslToCmdScrollThumbTrack(wxScrollEvent& event);
    void OnbtnGenerateClick(wxCommandEvent& event);
    void OnbtnSaveTxtClick(wxCommandEvent& event);
    void OnbtnSave3DClick(wxCommandEvent& event);
    void OnbtnCloseClick(wxCommandEvent& event);
    void OnslDecimateCmdScrollThumbTrack(wxScrollEvent& event);
    void OnslToCmdScroll(wxScrollEvent& event);
    void OnbtnGeneratePathsClick(wxCommandEvent& event);
    void OnGenerateFromRTK(wxCommandEvent& event);
    void OnbtnSavePathClick(wxCommandEvent& event);
    void OnbtnSaveObsPathClick(wxCommandEvent& event);
    void OnbtnView3DClick(wxCommandEvent& event);
    void OnbtnHelpClick(wxCommandEvent& event);
    //*)

    //(*Declarations(CFormRawMap)
    wxSlider* slDecimate;
    wxSpinCtrl* edDec;
    wxStaticText* StaticText2;
    wxStaticText* lbLength;
    wxButton* btnClose;
    wxSpinCtrl* edLast;
    wxSpinCtrl* edFirst;
    wxFlexGridSizer* FlexGridSizer3;
    wxStaticText* lbCount;
    wxStaticText* StaticText6;
    wxBitmapButton* btnHelp;
    wxButton* btnSavePath;
    wxFlexGridSizer* FlexGridSizer2;
    wxStaticText* StaticText1;
    wxStaticText* StaticText3;
    wxButton* btnGeneratePaths;
    wxButton* btnGenerate;
    wxButton* btnSave3D;
    wxPanel* Panel3;
    wxTextCtrl* edOpts;
    wxStaticText* StaticText5;
    mpWindow* plotMap;
    wxSlider* slFrom;
    wxSlider* slTo;
    wxBoxSizer* BoxSizer1;
    wxButton* btnView3D;
    wxButton* btnSaveObsPath;
    wxPanel* Panel2;
    wxButton* btnSaveTxt;
    wxFlexGridSizer* FlexGridSizer1;
    wxStaticText* StaticText4;
    wxButton* btnGenerateRTK;
    //*)

private:


	mrpt::gui::CDisplayWindow3DPtr	win3Dmap;


    DECLARE_EVENT_TABLE()
};

#endif
