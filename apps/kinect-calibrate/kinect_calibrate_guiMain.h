/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
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

/*
  App      : kinect-calibrate
  Web page : http://www.mrpt.org/Kinect_and_MRPT

  Usage    : Run and follow on-screen instructions
*/

#ifndef KINECT_CALIBRATE_GUIMAIN_H
#define KINECT_CALIBRATE_GUIMAIN_H

//(*Headers(kinect_calibrate_guiDialog)
#include <wx/spinctrl.h>
#include <wx/checkbox.h>
#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/notebook.h>
#include <wx/button.h>
#include <wx/panel.h>
#include <mrpt/gui/WxUtils.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/timer.h>
//*)

#include "../wx-common/CMyRedirector.h"


#include <mrpt/utils.h>
#include <mrpt/obs.h>
#include <mrpt/system/threads.h>

// Thread for grabbing: Do this is another thread so we divide rendering and grabbing
//   and exploit multicore CPUs.
struct TThreadParam
{
	TThreadParam() : quit(false), terminated(false), command(0), tilt_ang_deg(0), Hz(0) { }

	volatile bool   quit;
	volatile bool   terminated;
	volatile int    command;
	volatile double tilt_ang_deg;
	volatile double Hz;

	mrpt::synch::CThreadSafeVariable<mrpt::slam::CObservation3DRangeScanPtr> new_obs;     // RGB+D (+3D points)
};


class kinect_calibrate_guiDialog: public wxDialog
{
    public:

        kinect_calibrate_guiDialog(wxWindow* parent,wxWindowID id = -1);
        virtual ~kinect_calibrate_guiDialog();

    private:

		CMyRedirector  *m_my_redirector;

		mrpt::system::TThreadHandle  m_cap_thread;
		TThreadParam                 m_cap_thread_data;

		mrpt::slam::CObservation3DRangeScanPtr  m_last_obs;

		void thread_grabbing();
		void ProcessNewGrabbedObs();

        //(*Handlers(kinect_calibrate_guiDialog)
        void OnQuit(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);
        void OnNotebook1PageChanging(wxNotebookEvent& event);
        void OnbtnNext1Click(wxCommandEvent& event);
        void OnbtnQuitClick(wxCommandEvent& event);
        void OnbtnConnectClick(wxCommandEvent& event);
        void OntimConsoleDumpTrigger(wxTimerEvent& event);
        void OntimMiscTrigger(wxTimerEvent& event);
        void OnClose(wxCloseEvent& event);
        //*)

        //(*Identifiers(kinect_calibrate_guiDialog)
        static const long ID_CUSTOM2;
        static const long ID_STATICTEXT6;
        static const long ID_STATICTEXT7;
        static const long ID_STATICTEXT8;
        static const long ID_STATICTEXT5;
        static const long ID_BUTTON3;
        static const long ID_TEXTCTRL2;
        static const long ID_PANEL1;
        static const long ID_CUSTOM3;
        static const long ID_STATICTEXT11;
        static const long ID_BUTTON5;
        static const long ID_STATICTEXT9;
        static const long ID_BUTTON4;
        static const long ID_STATICTEXT10;
        static const long ID_TEXTCTRL4;
        static const long ID_PANEL3;
        static const long ID_CUSTOM1;
        static const long ID_STATICTEXT1;
        static const long ID_SPINCTRL1;
        static const long ID_STATICTEXT2;
        static const long ID_SPINCTRL2;
        static const long ID_STATICTEXT3;
        static const long ID_TEXTCTRL1;
        static const long ID_STATICTEXT4;
        static const long ID_TEXTCTRL3;
        static const long ID_CHECKBOX1;
        static const long ID_PANEL7;
        static const long ID_PANEL4;
        static const long ID_PANEL5;
        static const long ID_PANEL6;
        static const long ID_NOTEBOOK1;
        static const long ID_BUTTON1;
        static const long ID_BUTTON2;
        static const long ID_PANEL2;
        static const long ID_TIMER1;
        static const long ID_TIMER2;
        //*)

        //(*Declarations(kinect_calibrate_guiDialog)
        wxButton* btnQuit;
        mrpt::gui::wxMRPTImageControl* m_realtimeview_cap;
        wxTextCtrl* edLengthY;
        wxPanel* Panel6;
        wxPanel* Panel1;
        wxPanel* Panel7;
        wxCheckBox* cbNormalize;
        mrpt::gui::wxMRPTImageControl* m_imgStaticKinect;
        wxButton* btnConnect;
        wxButton* bntAbout;
        wxTimer timMisc;
        wxSpinCtrl* edSizeY;
        wxStaticText* StaticText1;
        wxStaticText* StaticText10;
        wxTimer timConsoleDump;
        wxPanel* Panel2;
        wxStaticText* StaticText3;
        wxPanel* Panel4;
        wxPanel* Panel5;
        wxStaticText* StaticText8;
        wxSpinCtrl* edSizeX;
        wxPanel* Panel3;
        wxStaticText* StaticText7;
        wxButton* btnNext1;
        wxTextCtrl* TextCtrl1;
        wxStaticText* StaticText4;
        wxStaticText* StaticText5;
        wxStaticText* StaticText2;
        wxNotebook* Notebook1;
        wxTextCtrl* edLengthX;
        wxStaticText* StaticText6;
        wxButton* btnNext2;
        mrpt::gui::wxMRPTImageControl* m_realtimeview_test;
        wxTextCtrl* edLogTest;
        wxStaticText* StaticText9;
        wxStaticText* StaticText11;
        //*)

        DECLARE_EVENT_TABLE()


};

#endif // KINECT_CALIBRATE_GUIMAIN_H
