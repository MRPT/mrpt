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
#ifndef CFORMPLAYVIDEO_H
#define CFORMPLAYVIDEO_H

//(*Headers(CFormPlayVideo)
#include <wx/bmpbuttn.h>
#include <wx/spinctrl.h>
#include <wx/combobox.h>
#include <wx/checkbox.h>
#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/button.h>
#include <wx/radiobut.h>
#include <wx/panel.h>
#include <wx/slider.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
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
    wxStaticText* lbCam2;
    wxCheckBox* cbReduceLarge;
    wxStaticText* StaticText22;
    wxStaticText* lbProgress;
    wxFlexGridSizer* FlexGridSizer8;
    wxFlexGridSizer* FlexGridSizer1;
    wxBitmapButton* btnSaveCam3;
    wxPanel* Panel1;
    wxButton* btnClose;
    wxButton* btnPickInput;
    wxTextCtrl* edFile;
    wxComboBox* cbImageDirs;
    wxButton* btnPlay;
    wxSpinCtrl* edIndex;
    wxStaticText* StaticText1;
    wxRadioButton* rbFile;
    wxButton* btnJump;
    wxFlexGridSizer* FlexGridSizer6;
    wxButton* btnStop;
    wxStaticText* lbCam3;
    wxBoxSizer* BoxSizer4;
    wxStaticText* lbCam1;
    wxTextCtrl* edDelay;
    wxPanel* pnLeft;
    wxBitmapButton* btnSaveCam1;
    wxPanel* pnRight;
    wxRadioButton* rbLoaded;
    wxStaticText* StaticText2;
    wxBitmapButton* btnSaveCam2;
    wxStaticText* StaticText6;
    wxCheckBox* cbOrderByY;
    wxSlider* progressBar;
    wxPanel* pnRight2;
    wxCheckBox* cbOrderByYaw;
    wxBoxSizer* BoxSizer5;
    wxStaticBoxSizer* StaticBoxSizer1;
    //*)

private:

	bool		firstFit;
	void 		saveCamImage(int nPanel);
	int 		m_idxInRawlog;

    DECLARE_EVENT_TABLE()
};

#endif
