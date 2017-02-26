/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CFORMEDIT_H
#define CFORMEDIT_H

#include <wx/listbox.h>

//(*Headers(CFormEdit)
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/checklst.h>
#include <wx/checkbox.h>
#include <wx/spinctrl.h>
#include <wx/radiobut.h>
#include <wx/slider.h>
#include <wx/button.h>
#include <wx/dialog.h>
//*)


#include <wx/combobox.h>


#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>


class CFormEdit: public wxDialog
{
public:

    CFormEdit(wxWindow* parent,wxWindowID id = -1);
    virtual ~CFormEdit();

	/** Used in executeOperationOnRawlogFiles
	  */
	typedef void (*TRawlogFilter)( mrpt::obs::CActionCollection *acts, mrpt::obs::CSensoryFrame *SF, int &changesCount  );
    /** This is the common function for all operations over a rawlog file ("filter" a rawlog file into a new one) or over the loaded rawlog (depending on the user selection in the GUI).
      */
    void executeOperationOnRawlog( TRawlogFilter operation, const char *endMsg);


    //(*Identifiers(CFormEdit)
    static const long ID_RADIOBUTTON1;
    static const long ID_RADIOBUTTON2;
    static const long ID_STATICTEXT22;
    static const long ID_TEXTCTRL11;
    static const long ID_BUTTON9;
    static const long ID_STATICTEXT23;
    static const long ID_TEXTCTRL12;
    static const long ID_BUTTON11;
    static const long ID_STATICTEXT1;
    static const long ID_SLIDER1;
    static const long ID_SPINCTRL1;
    static const long ID_STATICTEXT3;
    static const long ID_SLIDER2;
    static const long ID_SPINCTRL2;
    static const long ID_BUTTON1;
    static const long ID_BUTTON2;
    static const long ID_CHECKBOX1;
    static const long ID_CHECKBOX2;
    static const long ID_CHECKBOX3;
    static const long ID_CHECKBOX4;
    static const long ID_CHECKBOX5;
    static const long ID_BUTTON4;
    static const long ID_CHECKBOX6;
    static const long ID_CHECKBOX7;
    static const long ID_CHECKBOX8;
    static const long ID_CHECKBOX9;
    static const long ID_CHECKBOX10;
    static const long ID_BUTTON5;
    static const long ID_CHECKLISTBOX2;
    static const long ID_BUTTON7;
    static const long ID_BUTTON8;
    static const long ID_CHECKLISTBOX1;
    static const long ID_BUTTON10;
    static const long ID_BUTTON12;
    static const long ID_BUTTON13;
    static const long ID_STATICTEXT2;
    static const long ID_TEXTCTRL2;
    static const long ID_BUTTON3;
    static const long ID_BUTTON6;
    //*)

public:

    //(*Handlers(CFormEdit)
    void OnbtnCloseClick(wxCommandEvent& event);
    void OnslFirstCmdScrollChanged(wxScrollEvent& event);
    void OnslToCmdScrollChanged(wxScrollEvent& event);
    void OnbtnKeepClick(wxCommandEvent& event);
    void OnbtnDeleteClick(wxCommandEvent& event);
    void OnbtnDelObsIndxClick(wxCommandEvent& event);
    void OnbtnRemoveObsClassClick(wxCommandEvent& event);
    void OnbtnRemActsIndxClick(wxCommandEvent& event);
    void OnbtnImgSwapClick(wxCommandEvent& event);
    void OnInit(wxInitDialogEvent& event);
    void OnrbLoadedSelect(wxCommandEvent& event);
    void OnrbFileSelect(wxCommandEvent& event);
    void OnbtnRemoveAllButByClassClick1(wxCommandEvent& event);
    void OnbtnPickInputClick(wxCommandEvent& event);
    void OnbtnPickOutClick(wxCommandEvent& event);
    void OnRemoveByLabel(wxCommandEvent& event);
    void OnRemoveButLabel(wxCommandEvent& event);
    void OnslFromCmdScroll(wxScrollEvent& event);
    void OnslFromCmdScroll1(wxScrollEvent& event);
    void OnbtnLeaveHorizScansClick(wxCommandEvent& event);
    //*)

    //(*Declarations(CFormEdit)
    wxBoxSizer* BoxSizer4;
    wxStaticBoxSizer* StaticBoxSizer2;
    wxStaticText* StaticText22;
    wxFlexGridSizer* FlexGridSizer4;
    wxRadioButton* rbLoaded;
    wxButton* btnLeaveHorizScans;
    wxButton* btnRemoveAllButByClass;
    wxCheckBox* cbO4;
    wxBoxSizer* BoxSizer5;
    wxCheckBox* cbA1;
    wxSpinCtrl* spinFirst;
    wxStaticText* StaticText2;
    wxButton* btnClose;
    wxStaticBoxSizer* StaticBoxSizer4;
    wxFlexGridSizer* FlexGridSizer3;
    wxButton* btnRemByLabel;
    wxTextCtrl* edMaxPitch;
    wxButton* btnDelete;
    wxFlexGridSizer* FlexGridSizer9;
    wxCheckListBox* cbObsLabel;
    wxFlexGridSizer* FlexGridSizer2;
    wxStaticText* StaticText1;
    wxCheckBox* cbA4;
    wxBoxSizer* BoxSizer2;
    wxStaticText* StaticText3;
    wxButton* btnRemByLabelNon;
    wxFlexGridSizer* FlexGridSizer7;
    wxStaticBoxSizer* StaticBoxSizer7;
    wxButton* btnPickInput;
    wxButton* btnPickOut;
    wxSpinCtrl* spinLast;
    wxStaticText* StaticText23;
    wxCheckBox* cbO3;
    wxCheckBox* cbO0;
    wxStaticBoxSizer* StaticBoxSizer3;
    wxStaticBoxSizer* StaticBoxSizer6;
    wxFlexGridSizer* FlexGridSizer8;
    wxButton* btnDelObsIndx;
    wxCheckBox* cbO1;
    wxButton* btnImgSwap;
    wxRadioButton* rbFile;
    wxSlider* slFrom;
    wxSlider* slTo;
    wxBoxSizer* BoxSizer1;
    wxButton* btnRemActsIndx;
    wxCheckBox* cbA0;
    wxCheckBox* cbA3;
    wxFlexGridSizer* FlexGridSizer6;
    wxButton* btnRemoveObsClass;
    wxCheckBox* cbA2;
    wxStaticBoxSizer* StaticBoxSizer1;
    wxFlexGridSizer* FlexGridSizer1;
    wxCheckBox* cbO2;
    wxFlexGridSizer* FlexGridSizer11;
    wxButton* btnKeep;
    wxStaticBoxSizer* StaticBoxSizer5;
    wxTextCtrl* txtInputFile;
    wxCheckListBox* cbObsClass;
    wxTextCtrl* txtOutputFile;
    //*)

private:

    DECLARE_EVENT_TABLE()
};

#endif
