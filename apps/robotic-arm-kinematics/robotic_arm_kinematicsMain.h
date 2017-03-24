/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef robotic_arm_KINEMATICSMAIN_H
#define robotic_arm_KINEMATICSMAIN_H

//(*Headers(robotic_arm_kinematicsFrame)
#include <wx/sizer.h>
#include <wx/notebook.h>
#include <wx/htmllbox.h>
#include <wx/button.h>
#include <wx/menu.h>
#include <wx/panel.h>
#include <wx/slider.h>
#include <wx/statline.h>
#include <wx/frame.h>
#include "MyGLCanvas.h"
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/radiobox.h>
#include <wx/listbox.h>
//*)

#include "PanelDOF.h"

#include <mrpt/kinematics/CKinematicChain.h>

class robotic_arm_kinematicsFrame: public wxFrame
{
    public:

        robotic_arm_kinematicsFrame(wxWindow* parent,wxWindowID id = -1);
        virtual ~robotic_arm_kinematicsFrame();


		void OnSliderDOFScroll(wxScrollEvent& event);

    private:

        //(*Handlers(robotic_arm_kinematicsFrame)
        void OnQuit(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);
        void OnlistLinksSelect(wxCommandEvent& event);
        void OnSliderScroll(wxScrollEvent& event);
        void OnButtonSaveFromEdit(wxCommandEvent& event);
        void OnbtnClearClick(wxCommandEvent& event);
        void OnbtnAddLinkClick(wxCommandEvent& event);
        void OnLoadBinary(wxCommandEvent& event);
        void OnSaveBinary(wxCommandEvent& event);
        void OnrbTypeSelect(wxCommandEvent& event);
        void OnbtnDeleteClick(wxCommandEvent& event);
        void OnlbXYZsSelect(wxCommandEvent& event);
        void On1stXYZSelect(wxCommandEvent& event);
        //*)

        //(*Identifiers(robotic_arm_kinematicsFrame)
        static const long ID_RADIOBOX2;
        static const long ID_STATICTEXT1;
        static const long ID_SIMPLEHTMLLISTBOX1;
        static const long ID_BUTTON5;
        static const long ID_BUTTON6;
        static const long ID_BUTTON7;
        static const long ID_RADIOBOX1;
        static const long ID_STATICLINE4;
        static const long ID_SIMPLEHTMLLISTBOX2;
        static const long ID_TEXTCTRL1;
        static const long ID_STATICTEXT3;
        static const long ID_BUTTON1;
        static const long ID_SLIDER1;
        static const long ID_STATICLINE1;
        static const long ID_SIMPLEHTMLLISTBOX3;
        static const long ID_TEXTCTRL2;
        static const long ID_STATICTEXT5;
        static const long ID_BUTTON2;
        static const long ID_SLIDER2;
        static const long ID_STATICLINE2;
        static const long ID_SIMPLEHTMLLISTBOX4;
        static const long ID_TEXTCTRL3;
        static const long ID_STATICTEXT7;
        static const long ID_BUTTON3;
        static const long ID_SLIDER3;
        static const long ID_STATICLINE3;
        static const long ID_SIMPLEHTMLLISTBOX5;
        static const long ID_TEXTCTRL4;
        static const long ID_STATICTEXT9;
        static const long ID_BUTTON4;
        static const long ID_SLIDER4;
        static const long ID_PANEL1;
        static const long ID_XY_GLCANVAS;
        static const long ID_STATICTEXT10;
        static const long ID_PANEL2;
        static const long ID_STATICTEXT2;
        static const long ID_LISTBOX1;
        static const long ID_TEXTCTRL5;
        static const long ID_PANEL3;
        static const long ID_NOTEBOOK1;
        static const long ID_MENUITEM3;
        static const long ID_MENUITEM1;
        static const long ID_MENUITEM2;
        static const long idMenuQuit;
        static const long idMenuAbout;
        //*)

        //(*Declarations(robotic_arm_kinematicsFrame)
        wxButton* btnDelete;
        wxSlider* slD;
        wxPanel* pnDOFs;
        wxPanel* Panel1;
        wxButton* btnA;
        wxStaticLine* StaticLine2;
        wxSimpleHtmlListBox* SimpleHtmlListBox2;
        wxButton* btnClear;
        wxTextCtrl* edTheta;
        wxSimpleHtmlListBox* SimpleHtmlListBox3;
        wxTextCtrl* edD;
        wxRadioBox* rbType;
        wxSlider* slTheta;
        wxStaticText* StaticText1;
        wxStaticText* StaticText10;
        CMyGLCanvas* m_plot3D;
        wxStaticText* StaticText3;
        wxTextCtrl* edA;
        wxButton* btnAddLink;
        wxMenuItem* MenuItem3;
        wxStaticLine* StaticLine1;
        wxPanel* panelProperties;
        wxTextCtrl* edMatrix;
        wxStaticLine* StaticLine3;
        wxSlider* slA;
        wxStaticText* StaticText7;
        wxListBox* lbXYZs;
        wxMenuItem* MenuItem5;
        wxButton* btnD;
        wxStaticText* StaticText5;
        wxStaticText* StaticText2;
        wxSimpleHtmlListBox* SimpleHtmlListBox4;
        wxButton* btnAlpha;
        wxNotebook* Notebook1;
        wxSimpleHtmlListBox* SimpleHtmlListBox5;
        wxRadioBox* RadioBox1;
        wxMenuItem* MenuItem4;
        wxButton* btnTh;
        wxStaticLine* StaticLine4;
        wxSimpleHtmlListBox* listLinks;
        wxSlider* slAlpha;
        wxBoxSizer* boxSizerDOFs;
        wxStaticText* StaticText9;
        wxTextCtrl* edAlpha;
        //*)

        DECLARE_EVENT_TABLE()


	protected:
		mrpt::kinematics::CKinematicChain  m_robot;

		std::vector<PanelDOF*>             m_dof_panels;

		mrpt::opengl::CSetOfObjectsPtr     m_gl_robot;

		mrpt::aligned_containers<mrpt::poses::CPose3D>::vector_t m_all_poses;

		/** Regenerate the left list from m_robot */
		void UpdateListLinks();

		/** Regenerate the bottom controls from m_robot */
		void RegenerateDOFPanels();

		/** Just update the DOF panel status from m_robot */
		void UpdateDOFPanels();

		void Regenerate3DView();

		void OnListSelectionChange();

		void UpdateMatrixView();

};

extern robotic_arm_kinematicsFrame *the_win;


#endif // robotic_arm_KINEMATICSMAIN_H
