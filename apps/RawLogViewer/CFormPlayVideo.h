/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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

#include <mrpt/utils/CImage.h>

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
    static const long ID_CHECKBOX4;
    static const long ID_STATICTEXT8;
    static const long ID_SPINCTRL2;
    static const long ID_STATICTEXT6;
    static const long ID_COMBOBOX1;
    static const long ID_BUTTON2;
    static const long ID_BUTTON3;
    static const long ID_STATICTEXT7;
    static const long ID_TEXTCTRL2;
    static const long ID_BUTTON5;
    static const long ID_SLIDER1;
    static const long ID_STATICTEXT1;
    static const long ID_SPINCTRL1;
    static const long ID_BUTTON1;
    static const long ID_STATICTEXT3;
    static const long ID_STATICTEXT5;
    static const long ID_BITMAPBUTTON1;
    static const long ID_STATICTEXT4;
    static const long ID_BITMAPBUTTON2;
    static const long ID_STATICTEXT2;
    static const long ID_BITMAPBUTTON3;
    static const long ID_PANEL3;
    static const long ID_PANEL4;
    static const long ID_PANEL2;
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

	void drawHorzRules( mrpt::utils::CImage &img);

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
    wxSpinCtrl* edHorzRuleSpace;
    wxStaticText* StaticText6;
    wxSlider* progressBar;
    wxPanel* pnRight2;
    wxCheckBox* cbDrawStereoRules;
    wxStaticText* StaticText1;
    wxCheckBox* cbOrderByY;
    wxBitmapButton* btnSaveCam1;
    wxButton* btnJump;
    wxStaticText* StaticText3;
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
