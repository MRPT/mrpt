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
#ifndef CFORMPLAYVIDEO_H
#define CFORMPLAYVIDEO_H

//(*Headers(CFormPlayVideo)
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/checkbox.h>
#include <wx/spinctrl.h>
#include <wx/radiobut.h>
#include <wx/slider.h>
#include <wx/panel.h>
#include <wx/bmpbuttn.h>
#include <wx/button.h>
#include <wx/dialog.h>
#include <wx/combobox.h>
//*)

class CFormPlayVideo: public wxDialog
{
public:

    CFormPlayVideo(wxWindow* parent,wxWindowID id = -1);
    virtual ~CFormPlayVideo();

    // Set to true to enable play, false to stop.
    bool        m_nowPlaying;

    wxComboBox* getImageDirsCombo() { return cbImageDirs;}

    //(*Identifiers(CFormPlayVideo)
    static const long ID_RADIOBUTTON1;
    static const long ID_RADIOBUTTON2;
    static const long ID_STATICTEXT22;
    static const long ID_TEXTCTRL11;
    static const long ID_BUTTON4;
    static const long ID_CHECKBOX1;
    static const long ID_CHECKBOX2;
    static const long ID_CHECKBOX3;
    static const long ID_STATICTEXT6;
    static const long ID_COMBOBOX1;
    static const long ID_BUTTON2;
    static const long ID_BUTTON3;
    static const long ID_STATICTEXT7;
    static const long ID_TEXTCTRL2;
    static const long ID_BUTTON5;
    static const long ID_PANEL1;
    static const long ID_STATICTEXT5;
    static const long ID_BITMAPBUTTON1;
    static const long ID_STATICTEXT4;
    static const long ID_BITMAPBUTTON2;
    static const long ID_STATICTEXT2;
    static const long ID_BITMAPBUTTON3;
    static const long ID_PANEL3;
    static const long ID_PANEL4;
    static const long ID_PANEL2;
    static const long ID_SLIDER1;
    static const long ID_STATICTEXT1;
    static const long ID_SPINCTRL1;
    static const long ID_BUTTON1;
    static const long ID_STATICTEXT3;
    //*)

protected:

    //(*Handlers(CFormPlayVideo)
    void OnbtnPickClick(wxCommandEvent& event);
    void OnbtnStopClick(wxCommandEvent& event);
    void OnbtnPlayClick(wxCommandEvent& event);
    void OnInit(wxInitDialogEvent& event);
    void OnrbLoadedSelect(wxCommandEvent& event);
    void OnrbFileSelect(wxCommandEvent& event);
    void OnbtnCloseClick(wxCommandEvent& event);
    void OnprogressBarCmdScrollChanged(wxScrollEvent& event);
    void OnbtnSaveCam1Click(wxCommandEvent& event);
    void OnbtnSaveCam2Click(wxCommandEvent& event);
    void OnbtnSaveCam3Click(wxCommandEvent& event);
    void OncbImageDirsSelect(wxCommandEvent& event);
    //*)

    bool showSensoryFrame(void *SF, size_t &nImgs);

    //(*Declarations(CFormPlayVideo)
    wxBoxSizer* BoxSizer4;
    wxStaticText* StaticText22;
    wxButton* btnStop;
    wxRadioButton* rbLoaded;
    wxBitmapButton* btnSaveCam2;
    wxStaticText* lbProgress;
    wxBoxSizer* BoxSizer5;
    wxStaticText* StaticText2;
    wxButton* btnClose;
    wxPanel* pnRight;
    wxStaticText* StaticText6;
    wxSlider* progressBar;
    wxPanel* pnRight2;
    wxPanel* Panel1;
    wxStaticText* StaticText1;
    wxCheckBox* cbOrderByY;
    wxButton* btnJump;
    wxBitmapButton* btnSaveCam1;
    wxButton* btnPickInput;
    wxStaticText* lbCam1;
    wxComboBox* cbImageDirs;
    wxSpinCtrl* edIndex;
    wxBitmapButton* btnSaveCam3;
    wxFlexGridSizer* FlexGridSizer8;
    wxRadioButton* rbFile;
    wxStaticText* lbCam3;
    wxFlexGridSizer* FlexGridSizer6;
    wxStaticBoxSizer* StaticBoxSizer1;
    wxFlexGridSizer* FlexGridSizer1;
    wxCheckBox* cbReduceLarge;
    wxPanel* pnLeft;
    wxCheckBox* cbOrderByYaw;
    wxTextCtrl* edDelay;
    wxStaticText* lbCam2;
    wxButton* btnPlay;
    wxTextCtrl* edFile;
    //*)

private:

	bool		firstFit;
	void 		saveCamImage(int nPanel);
	int 		m_idxInRawlog;

    DECLARE_EVENT_TABLE()
};

#endif
