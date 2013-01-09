/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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
        //*)

        //(*Identifiers(robotic_arm_kinematicsFrame)
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
