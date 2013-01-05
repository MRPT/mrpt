/***************************************************************
 * Name:      robotic_arm_kinematicsMain.h
 * Purpose:   Defines Application Frame
 * Author:     ()
 * Created:   2013-01-05
 * Copyright:  ()
 * License:
 **************************************************************/

#ifndef robotic_arm_KINEMATICSMAIN_H
#define robotic_arm_KINEMATICSMAIN_H

//(*Headers(robotic_arm_kinematicsFrame)
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/radiobox.h>
#include <wx/menu.h>
#include <wx/textctrl.h>
#include <wx/htmllbox.h>
#include "MyGLCanvas.h"
#include <wx/slider.h>
#include <wx/frame.h>
#include <wx/statusbr.h>
//*)

class robotic_arm_kinematicsFrame: public wxFrame
{
    public:

        robotic_arm_kinematicsFrame(wxWindow* parent,wxWindowID id = -1);
        virtual ~robotic_arm_kinematicsFrame();

    private:

        //(*Handlers(robotic_arm_kinematicsFrame)
        void OnQuit(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);
        //*)

        //(*Identifiers(robotic_arm_kinematicsFrame)
        static const long ID_STATICTEXT1;
        static const long ID_SIMPLEHTMLLISTBOX1;
        static const long ID_RADIOBOX1;
        static const long ID_SIMPLEHTMLLISTBOX2;
        static const long ID_STATICTEXT2;
        static const long ID_TEXTCTRL1;
        static const long ID_STATICTEXT3;
        static const long ID_SLIDER1;
        static const long ID_XY_GLCANVAS;
        static const long idMenuQuit;
        static const long idMenuAbout;
        static const long ID_STATUSBAR1;
        //*)

        //(*Declarations(robotic_arm_kinematicsFrame)
        wxStaticText* StaticText2;
        wxStaticText* StaticText1;
        wxStaticText* StaticText3;
        wxStatusBar* StatusBar1;
        wxRadioBox* RadioBox1;
        wxSlider* slTheta;
        wxSimpleHtmlListBox* SimpleHtmlListBox1;
        wxSimpleHtmlListBox* SimpleHtmlListBox2;
        wxTextCtrl* edTheta;
        CMyGLCanvas* m_plot3D;
        //*)

        DECLARE_EVENT_TABLE()
};

#endif // robotic_arm_KINEMATICSMAIN_H
