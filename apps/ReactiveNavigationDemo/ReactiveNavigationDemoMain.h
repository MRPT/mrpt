/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef REACTIVENAVIGATIONDEMOMAIN_H
#define REACTIVENAVIGATIONDEMOMAIN_H

//(*Headers(ReactiveNavigationDemoFrame)
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/checkbox.h>
#include <wx/panel.h>
#include <wx/button.h>
#include <wx/frame.h>
#include <wx/timer.h>
#include <wx/statusbr.h>
//*)

#include <mrpt/otherlibs/mathplot/mathplot.h>
#include "../wx-common/CMyRedirector.h"


class ReactiveNavigationDemoFrame: public wxFrame
{
    public:

        ReactiveNavigationDemoFrame(wxWindow* parent,wxWindowID id = -1);
        virtual ~ReactiveNavigationDemoFrame();

 //   private:

        //(*Handlers(ReactiveNavigationDemoFrame)
        void OnQuit(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);
        void OnbtnStartClick(wxCommandEvent& event);
        void OnbtnPauseClick(wxCommandEvent& event);
        void OnbtnExitClick(wxCommandEvent& event);
        void OnbtnNavigateClick(wxCommandEvent& event);
        void OnplotMouseMove(wxMouseEvent& event);
        void OntimSimulateTrigger(wxTimerEvent& event);
        void OnbtnResetClick(wxCommandEvent& event);
        void OnbtnEditNavParamsClick(wxCommandEvent& event);
        void OnrbExtMapSelect(wxCommandEvent& event);
        void OncbInternalParamsClick(wxCommandEvent& event);
        void OncbInternalParamsClick1(wxCommandEvent& event);
        //*)

		void OnreactivenavTargetMenu(wxCommandEvent& event);

        //(*Identifiers(ReactiveNavigationDemoFrame)
        static const long ID_BUTTON1;
        static const long ID_BUTTON2;
        static const long ID_CHECKBOX3;
        static const long ID_BUTTON3;
        static const long ID_CHECKBOX1;
        static const long ID_STATICTEXT1;
        static const long ID_TEXTCTRL2;
        static const long ID_BUTTON7;
        static const long ID_CHECKBOX2;
        static const long ID_STATICTEXT6;
        static const long ID_TEXTCTRL6;
        static const long ID_STATICTEXT2;
        static const long ID_TEXTCTRL3;
        static const long ID_STATICTEXT4;
        static const long ID_STATICTEXT3;
        static const long ID_TEXTCTRL4;
        static const long ID_BUTTON4;
        static const long ID_PANEL1;
        static const long ID_CUSTOM1;
        static const long ID_TEXTCTRL1;
        static const long ID_STATUSBAR1;
        static const long ID_TIMER1;
        //*)

		static const long ID_MENUITEM_SET_reactivenav_TARGET;

        //(*Declarations(ReactiveNavigationDemoFrame)
        wxStaticText* StaticText2;
        wxButton* btnStart;
        wxTextCtrl* edMapFile;
        wxButton* btnEditNavParams;
        wxStaticText* StaticText6;
        wxButton* btnNavigate;
        wxTextCtrl* edX;
        wxPanel* Panel1;
        wxStaticText* StaticText3;
        wxCheckBox* cbExtMap;
        wxTextCtrl* edY;
        wxButton* btnExit;
        wxTextCtrl* edLog;
        wxButton* btnPause;
        wxStatusBar* StatusBar1;
        wxCheckBox* cbLog;
        wxTimer timSimulate;
        wxTextCtrl* edNavCfgFile;
        wxStaticText* StaticText4;
        mpWindow* plot;
        wxCheckBox* cbInternalParams;
        //*)


		CMyRedirector   *myRedirector;


        bool reloadMap();
		void reloadRobotShape();
		void tryConstructReactiveNavigator();


        DECLARE_EVENT_TABLE()

	public:
        mpBitmapLayer   *lyGridmap;
        mpPolygon       *lyVehicle, *lyTarget, *lyLaserPoints;

};

#endif // REACTIVENAVIGATIONDEMOMAIN_H
