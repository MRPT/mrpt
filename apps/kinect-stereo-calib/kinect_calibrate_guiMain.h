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

#include "wx/defs.h"

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
#include <wx/radiobox.h>
#include <wx/timer.h>
#include <wx/listbox.h>
//*)

#include <wx/config.h>  // wxConfig

#include "../wx-common/CMyRedirector.h"

#include <mrpt/utils.h>
#include <mrpt/obs.h>
#include <mrpt/system/threads.h>
#include <mrpt/vision/chessboard_stereo_camera_calib.h>

// Thread for grabbing: Do this is another thread so we divide rendering and grabbing
//   and exploit multicore CPUs.
struct TThreadParam
{
	TThreadParam() : quit(false), terminated(false), tilt_ang_deg(0), Hz(0),select_IR_channel(false) { }

	volatile bool   quit;
	volatile bool   terminated;
	volatile double tilt_ang_deg;
	volatile double Hz;
	volatile bool   select_IR_channel;

	mrpt::synch::CThreadSafeVariable<mrpt::slam::CObservation3DRangeScanPtr> new_obs;     // RGB+D (+3D points)
};

// Thread for (async) live detecting corners: Do in another thread so the GUI doesn't freeze.
struct TThreadDetectCornerParam
{
	TThreadDetectCornerParam() : quit(false), terminated(false), ready_for_new_images(true), image_timestamp(INVALID_TIMESTAMP), detected_corners_done(false) {}

	volatile bool   quit;
	volatile bool   terminated;

	volatile bool ready_for_new_images;
	mrpt::utils::CImage  image;
	volatile mrpt::system::TTimeStamp  image_timestamp;
	std::vector<mrpt::utils::TPixelCoordf>  detected_corners;
	volatile bool  detected_corners_done;
};

enum TGrabState
{
	gsIdle = 0,
	gsSwitchingRGB,
	gsCapturingRGB,
	gsSwitchingIR,
	gsCapturingIR
};


class kinect_calibrate_guiDialog: public wxDialog
{
    public:

        kinect_calibrate_guiDialog(wxWindow* parent,wxWindowID id = -1);
        virtual ~kinect_calibrate_guiDialog();

    private:

		wxConfig      m_config;

		CMyRedirector  *m_my_redirector;

		mrpt::system::TThreadHandle  m_cap_thread;
		TThreadParam                 m_cap_thread_data;

		mrpt::system::TThreadHandle  m_findcorners_thread;
		TThreadDetectCornerParam     m_findcorners_thread_data;

		mrpt::slam::CObservation3DRangeScanPtr  m_last_obs;

		TGrabState                                 m_grabstate;

		mrpt::vision::TCalibrationStereoImageList  m_calib_images;
		mrpt::vision::TStereoCalibResults          m_calib_result;
		mrpt::vision::TStereoCalibParams           m_calib_params;

		void thread_grabbing();
		void thread_find_corners();
		void ProcessNewGrabbedObs();

		void StopLiveGrabThreads();

		void UpdateListOfImages();
		void ProcessNewSelectedImageListBox();


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
        void OnrbChannelSwitchSelect(wxCommandEvent& event);
        void OnbtnCaptureClick(wxCommandEvent& event);
        void OnbtnNextCalibClick(wxCommandEvent& event);
        void OnedTiltChange(wxSpinEvent& event);
        void OnlbImagePairsSelect(wxCommandEvent& event);
        void OnbtnRunCalibClick(wxCommandEvent& event);
        void OnbtnSaveCalibClick(wxCommandEvent& event);
        void OnbtnOpCalibKinectClick(wxCommandEvent& event);
        void OnbtnOpCalibStereoGenericClick(wxCommandEvent& event);
        void OnbtnOpTestKinectClick(wxCommandEvent& event);
        void OnbtnDisconnectClick(wxCommandEvent& event);
        void OnbtnListLoadClick(wxCommandEvent& event);
        void OnbtnListSaveClick(wxCommandEvent& event);
        void OnbtnListRemoveSelectedClick(wxCommandEvent& event);
        void OnbtnLoadImageListClick(wxCommandEvent& event);
        //*)

        //(*Identifiers(kinect_calibrate_guiDialog)
        static const long ID_STATICTEXT27;
        static const long ID_BUTTON15;
        static const long ID_BUTTON16;
        static const long ID_BUTTON17;
        static const long ID_PANEL8;
        static const long ID_STATICTEXT28;
        static const long ID_STATICTEXT29;
        static const long ID_CUSTOM2;
        static const long ID_STATICTEXT30;
        static const long ID_TEXTCTRL10;
        static const long ID_STATICTEXT31;
        static const long ID_BUTTON3;
        static const long ID_PANEL9;
        static const long ID_CUSTOM3;
        static const long ID_STATICTEXT11;
        static const long ID_BUTTON5;
        static const long ID_STATICTEXT9;
        static const long ID_BUTTON4;
        static const long ID_STATICTEXT5;
        static const long ID_BUTTON8;
        static const long ID_PANEL3;
        static const long ID_CUSTOM1;
        static const long ID_TEXTCTRL5;
        static const long ID_STATICTEXT1;
        static const long ID_SPINCTRL1;
        static const long ID_STATICTEXT2;
        static const long ID_SPINCTRL2;
        static const long ID_RADIOBOX1;
        static const long ID_CHECKBOX1;
        static const long ID_STATICTEXT22;
        static const long ID_SPINCTRL7;
        static const long ID_BUTTON6;
        static const long ID_STATICTEXT12;
        static const long ID_BUTTON7;
        static const long ID_PANEL7;
        static const long ID_PANEL4;
        static const long ID_STATICTEXT23;
        static const long ID_LISTBOX1;
        static const long ID_BUTTON9;
        static const long ID_BUTTON10;
        static const long ID_BUTTON11;
        static const long ID_BUTTON12;
        static const long ID_STATICTEXT19;
        static const long ID_CUSTOM4;
        static const long ID_STATICTEXT20;
        static const long ID_CUSTOM5;
        static const long ID_STATICTEXT25;
        static const long ID_STATICTEXT13;
        static const long ID_SPINCTRL3;
        static const long ID_STATICTEXT14;
        static const long ID_SPINCTRL4;
        static const long ID_STATICTEXT15;
        static const long ID_TEXTCTRL6;
        static const long ID_STATICTEXT16;
        static const long ID_TEXTCTRL7;
        static const long ID_CHECKBOX2;
        static const long ID_CHECKBOX4;
        static const long ID_CHECKBOX5;
        static const long ID_CHECKBOX6;
        static const long ID_CHECKBOX7;
        static const long ID_CHECKBOX8;
        static const long ID_STATICTEXT26;
        static const long ID_SPINCTRL8;
        static const long ID_BUTTON14;
        static const long ID_STATICTEXT24;
        static const long ID_BUTTON13;
        static const long ID_TEXTCTRL8;
        static const long ID_PANEL6;
        static const long ID_STATICTEXT3;
        static const long ID_SPINCTRL5;
        static const long ID_STATICTEXT4;
        static const long ID_SPINCTRL6;
        static const long ID_STATICTEXT17;
        static const long ID_TEXTCTRL1;
        static const long ID_STATICTEXT18;
        static const long ID_TEXTCTRL3;
        static const long ID_CHECKBOX3;
        static const long ID_PANEL5;
        static const long ID_NOTEBOOK1;
        static const long ID_STATICTEXT10;
        static const long ID_TEXTCTRL4;
        static const long ID_BUTTON1;
        static const long ID_BUTTON2;
        static const long ID_PANEL2;
        static const long ID_TIMER1;
        static const long ID_TIMER2;
        //*)

        //(*Declarations(kinect_calibrate_guiDialog)
        wxTextCtrl* edCalibSizeX;
        wxStaticText* StaticText24;
        wxStaticText* StaticText22;
        wxCheckBox* cbOptT1;
        wxButton* btnQuit;
        mrpt::gui::wxMRPTImageControl* m_realtimeview_cap;
        wxTextCtrl* TextCtrl3;
        wxPanel* Panel6;
        wxStaticText* StaticText21;
        wxStaticText* StaticText13;
        wxStaticText* StaticText14;
        wxStaticText* StaticText15;
        wxPanel* Panel7;
        wxButton* btnOpCalibKinect;
        wxStaticText* StaticText17;
        wxStaticText* lbNumCaptured;
        wxStaticText* StaticText28;
        wxCheckBox* cbNormalize;
        wxListBox* lbImagePairs;
        wxSpinCtrl* SpinCtrl4;
        mrpt::gui::wxMRPTImageControl* m_imgStaticKinect;
        wxButton* btnDisconnect;
        wxButton* btnConnect;
        wxButton* bntAbout;
        wxButton* btnSaveCalib;
        wxButton* btnListLoad;
        wxButton* btnRunCalib;
        wxTimer timMisc;
        wxButton* btnOpTestKinect;
        wxCheckBox* cbCalibNormalize;
        wxSpinCtrl* edSizeY;
        wxPanel* Panel9;
        wxPanel* Panel8;
        wxCheckBox* cbOptK2;
        wxStaticText* StaticText18;
        wxSpinCtrl* edTilt;
        wxStaticText* StaticText1;
        wxStaticText* StaticText10;
        wxStaticText* StaticText16;
        wxTimer timConsoleDump;
        wxButton* btnLoadImageList;
        wxPanel* Panel2;
        wxTextCtrl* TextCtrl5;
        wxStaticText* StaticText3;
        wxPanel* Panel4;
        wxTextCtrl* edCalibSizeY;
        wxStaticText* StaticText23;
        wxCheckBox* cbOptK3;
        wxPanel* Panel5;
        wxSpinCtrl* SpinCtrl3;
        wxCheckBox* cbOptK1;
        wxCheckBox* CheckBox2;
        wxStaticText* StaticText12;
        wxSpinCtrl* edSizeX;
        wxSpinCtrl* edCalibCheckY;
        wxRadioBox* rbChannelSwitch;
        wxTextCtrl* TextCtrl6;
        wxPanel* Panel3;
        wxButton* btnNext1;
        wxButton* btnListSave;
        wxSpinCtrl* edCalibCheckX;
        wxSpinCtrl* edMaxIters;
        wxStaticText* StaticText4;
        wxStaticText* StaticText5;
        wxButton* btnNextCalib;
        wxStaticText* StaticText2;
        wxStaticText* StaticText30;
        wxStaticText* StaticText27;
        wxNotebook* Notebook1;
        mrpt::gui::wxMRPTImageControl* m_view_right;
        wxButton* btnOpCalibStereoGeneric;
        wxStaticText* StaticText26;
        wxCheckBox* cbOptT2;
        wxButton* btnCapture;
        wxButton* btnNext2;
        wxStaticText* StaticText19;
        wxStaticText* StaticText29;
        mrpt::gui::wxMRPTImageControl* m_realtimeview_test;
        wxTextCtrl* edLogTest;
        wxStaticText* StaticText9;
        wxTextCtrl* edLogCalibResult;
        wxTextCtrl* TextCtrl2;
        wxStaticText* StaticText11;
        wxStaticText* StaticText25;
        mrpt::gui::wxMRPTImageControl* m_view_left;
        wxButton* btnListRemoveSelected;
        //*)

        DECLARE_EVENT_TABLE()


};

#endif // KINECT_CALIBRATE_GUIMAIN_H
