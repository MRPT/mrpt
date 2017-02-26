/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CFORMMOTIONMODEL_H
#define CFORMMOTIONMODEL_H

//(*Headers(CFormMotionModel)
#include <wx/notebook.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/checkbox.h>
#include <wx/radiobut.h>
#include <wx/panel.h>
#include <wx/hyperlink.h>
#include <wx/button.h>
#include <wx/dialog.h>
//*)

// The "custom class" mpWindow, from the wxMathPlot libray by David Schalig
//  See http://sourceforge.net/projects/wxmathplot
#include <mrpt/otherlibs/mathplot/mathplot.h>

class CFormMotionModel: public wxDialog
{
public:

    CFormMotionModel(wxWindow* parent,wxWindowID id = -1);
    virtual ~CFormMotionModel();

    //(*Identifiers(CFormMotionModel)
    static const long ID_STATICTEXT1;
    static const long ID_BUTTON1;
    static const long ID_HYPERLINKCTRL1;
    static const long ID_STATICTEXT4;
    static const long ID_TEXTCTRL1;
    static const long ID_STATICTEXT2;
    static const long ID_STATICTEXT3;
    static const long ID_TEXTCTRL2;
    static const long ID_STATICTEXT5;
    static const long ID_STATICTEXT6;
    static const long ID_TEXTCTRL3;
    static const long ID_STATICTEXT7;
    static const long ID_STATICTEXT26;
    static const long ID_TEXTCTRL15;
    static const long ID_STATICTEXT27;
    static const long ID_STATICTEXT8;
    static const long ID_TEXTCTRL4;
    static const long ID_STATICTEXT9;
    static const long ID_STATICTEXT10;
    static const long ID_TEXTCTRL5;
    static const long ID_STATICTEXT11;
    static const long ID_BUTTON10;
    static const long ID_BUTTON2;
    static const long ID_BUTTON8;
    static const long ID_PANEL2;
    static const long ID_STATICTEXT12;
    static const long ID_TEXTCTRL6;
    static const long ID_STATICTEXT13;
    static const long ID_STATICTEXT14;
    static const long ID_TEXTCTRL7;
    static const long ID_STATICTEXT15;
    static const long ID_STATICTEXT16;
    static const long ID_TEXTCTRL8;
    static const long ID_STATICTEXT17;
    static const long ID_STATICTEXT18;
    static const long ID_TEXTCTRL9;
    static const long ID_STATICTEXT19;
    static const long ID_STATICTEXT20;
    static const long ID_TEXTCTRL10;
    static const long ID_STATICTEXT21;
    static const long ID_STATICTEXT29;
    static const long ID_TEXTCTRL17;
    static const long ID_STATICTEXT30;
    static const long ID_STATICTEXT31;
    static const long ID_TEXTCTRL18;
    static const long ID_STATICTEXT32;
    static const long ID_BUTTON11;
    static const long ID_BUTTON3;
    static const long ID_BUTTON9;
    static const long ID_PANEL3;
    static const long ID_NOTEBOOK1;
    static const long ID_RADIOBUTTON1;
    static const long ID_BUTTON6;
    static const long ID_CHECKBOX1;
    static const long ID_STATICTEXT34;
    static const long ID_TEXTCTRL19;
    static const long ID_STATICTEXT33;
    static const long ID_TEXTCTRL20;
    static const long ID_RADIOBUTTON2;
    static const long ID_STATICTEXT22;
    static const long ID_TEXTCTRL11;
    static const long ID_BUTTON4;
    static const long ID_BUTTON7;
    static const long ID_STATICTEXT23;
    static const long ID_TEXTCTRL12;
    static const long ID_BUTTON5;
    static const long ID_STATICTEXT25;
    static const long ID_TEXTCTRL13;
    static const long ID_TEXTCTRL14;
    static const long ID_TEXTCTRL16;
    static const long ID_PANEL1;
    static const long ID_STATICTEXT24;
    static const long ID_CUSTOM1;
    static const long ID_STATICTEXT28;
    static const long ID_CUSTOM2;
    //*)

protected:

    //(*Handlers(CFormMotionModel)
    void OnbtnOkClick(wxCommandEvent& event);
    void OnbtnGaussOKClick(wxCommandEvent& event);
    void OnbtnThrunOkClick(wxCommandEvent& event);
    void OnInit(wxInitDialogEvent& event);
    void OnrbFileSelect(wxCommandEvent& event);
    void OnrbLoadedSelect(wxCommandEvent& event);
    void OnbtnGetFromCurrentClick(wxCommandEvent& event);
    void OnbtnSimulateClick(wxCommandEvent& event);
    void OnbtnSimulateThrunClick(wxCommandEvent& event);
    void OnbtnResetGaussClick(wxCommandEvent& event);
    void OnButton1Click(wxCommandEvent& event);
    void OnbtnResetThrunClick(wxCommandEvent& event);
    void OnbtnPickInputClick(wxCommandEvent& event);
    void OnbtnPickOutClick(wxCommandEvent& event);
    void OnbtnGetFromFileClick(wxCommandEvent& event);
    void OncbAllClick(wxCommandEvent& event);
    //*)

public:

    //(*Declarations(CFormMotionModel)
    wxStaticText* StaticText10;
    wxBoxSizer* BoxSizer4;
    wxTextCtrl* txtAphi;
    wxStaticText* StaticText22;
    wxStaticText* StaticText9;
    wxTextCtrl* edAddPhi;
    wxRadioButton* rbLoaded;
    wxStaticText* StaticText20;
    wxFlexGridSizer* FlexGridSizer4;
    wxButton* btnOk;
    wxButton* btnThrunOk;
    wxBoxSizer* BoxSizer5;
    wxStaticText* StaticText29;
    wxButton* btnGaussOK;
    wxButton* btnSimulate;
    wxTextCtrl* txtAx;
    wxStaticText* StaticText33;
    wxStaticText* StaticText13;
    wxTextCtrl* edA2;
    wxStaticText* StaticText2;
    wxStaticText* StaticText30;
    wxStaticText* StaticText14;
    wxButton* btnGetFromFile;
    wxStaticText* StaticText26;
    wxStaticText* StaticText6;
    wxNotebook* PageControl1;
    wxTextCtrl* edA1;
    wxFlexGridSizer* FlexGridSizer5;
    wxStaticText* StaticText32;
    wxStaticText* StaticText19;
    wxButton* btnSimulateThrun;
    wxTextCtrl* edMinStdXY;
    wxStaticText* StaticText8;
    wxStaticText* StaticText11;
    mpWindow* plotSamplesXY;
    wxStaticText* StaticText18;
    wxTextCtrl* edA4;
    wxPanel* Panel1;
    wxStaticText* StaticText31;
    wxFlexGridSizer* FlexGridSizer2;
    wxStaticText* StaticText1;
    wxStaticText* StaticText27;
    wxStaticText* StaticText3;
    mpWindow* plotSamplesPHI;
    wxTextCtrl* edA3;
    wxHyperlinkCtrl* HyperlinkCtrl1;
    wxButton* btnGetFromCurrent;
    wxFlexGridSizer* FlexGridSizer7;
    wxTextCtrl* edG_A3;
    wxButton* btnPickInput;
    wxTextCtrl* edRangeFrom;
    wxButton* btnPickOut;
    wxButton* btnResetGauss;
    wxPanel* Panel3;
    wxStaticText* StaticText21;
    wxStaticText* StaticText23;
    wxStaticText* StaticText24;
    wxTextCtrl* edG_A2;
    wxStaticText* StaticText34;
    wxStaticText* StaticText5;
    wxStaticText* StaticText7;
    wxFlexGridSizer* FlexGridSizer8;
    wxTextCtrl* txtAy;
    wxRadioButton* rbFile;
    wxTextCtrl* edRangeTo;
    wxTextCtrl* edAddXY;
    wxStaticText* StaticText28;
    wxStaticText* StaticText15;
    wxStaticText* StaticText12;
    wxCheckBox* cbAll;
    wxTextCtrl* edNumParts;
    wxFlexGridSizer* FlexGridSizer6;
    wxPanel* Panel2;
    wxStaticBoxSizer* StaticBoxSizer1;
    wxFlexGridSizer* FlexGridSizer1;
    wxStaticText* StaticText25;
    wxFlexGridSizer* FlexGridSizer11;
    wxBoxSizer* BoxSizer3;
    wxTextCtrl* edG_A4;
    wxStaticText* StaticText17;
    wxStaticText* StaticText4;
    wxTextCtrl* txtInputFile;
    wxTextCtrl* edMinStdPHI;
    wxTextCtrl* edG_A1;
    wxStaticText* StaticText16;
    wxTextCtrl* txtOutputFile;
    wxButton* btnResetThrun;
    //*)

    // Layers for the 2D graphs:
    mpFXYVector     *lyAction2D_XY;
    mpFXYVector     *lyAction2D_PHI;

private:
    void loadFromGaussian();
    void loadFromThrun();

    void applyToLoadedRawlog();
    void applyToRawlogFile();

    void drawRandomSamples();
    void showOptionsInDialog();

    DECLARE_EVENT_TABLE()
};

#endif
