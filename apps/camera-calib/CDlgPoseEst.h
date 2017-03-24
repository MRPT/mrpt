/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#ifndef CDLGPOSEEST_H
#define CDLGPOSEEST_H

//(*Headers(CDlgCalibWizardOnline)
#include <wx/spinctrl.h>
#include <wx/checkbox.h>
#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/button.h>
#include <mrpt/gui/WxUtils.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/radiobox.h>
#include <wx/timer.h>

#include <wx/choice.h>
//*)

#include <mrpt/vision/chessboard_camera_calib.h>
#include <mrpt/hwdrivers/CCameraSensor.h>
#include <mrpt/gui/CMyRedirector.h>

#include <mrpt/gui/CDisplayWindow3D.h>

#include "MyGLCanvas.h"

#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/vision/pnp_algos.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/math/CQuaternion.h>
#include <fstream>

#define CAMERA_CALIB_GUI_VERSION  "2.0"

class CDlgPoseEst: public wxDialog
{
	public:

		CDlgPoseEst(wxWindow* parent,wxWindowID id=wxID_ANY,const wxPoint& pos=wxDefaultPosition,const wxSize& size=wxDefaultSize);
		virtual ~CDlgPoseEst();

		CMyRedirector	*redire;

		//(*Declarations(CDlgCalibWizardOnline)
		wxStaticText* lbProgress;
		wxFlexGridSizer* FlexGridSizer1;
		wxTextCtrl* edLengthY;
		wxButton* btnClose;
		wxCheckBox* cbNormalize;
		wxRadioBox* rbMethod;
		mrpt::gui::wxMRPTImageControl* m_realtimeview;
		wxSpinCtrl* edSizeY;
		wxStaticText* StaticText1;
		mrpt::gui::CPanelCameraSelection* m_panelCamera;
		wxStaticText* StaticText3;
		wxButton* btnStop;
		wxTimer timCapture;
		wxSpinCtrl* edSizeX;
		wxTextCtrl* txtLog;
		wxStaticText* StaticText4;
		wxStaticText* StaticText5;
		wxStaticText* StaticText2;
		wxSpinCtrl* edNumCapture;
		wxStaticText* StaticText6;
		wxTextCtrl* edLengthX;
		wxButton* btnStart;
		wxChoice* pnpSelect;
		wxStaticText* StaticTextAlgo;
		//*)

	protected:

		//(*Identifiers(CDlgCalibWizardOnline)
		static const long ID_CUSTOM2;
		static const long ID_STATICTEXT1;
		static const long ID_SPINCTRL1;
		static const long ID_STATICTEXT2;
		static const long ID_SPINCTRL2;
		static const long ID_RADIOBOX1;
		static const long ID_STATICTEXT3;
		static const long ID_TEXTCTRL1;
		static const long ID_STATICTEXT4;
		static const long ID_TEXTCTRL3;
		static const long ID_CHECKBOX1;
		static const long ID_STATICTEXT5;
		static const long ID_SPINCTRL3;
		static const long ID_STATICTEXT6;
		static const long ID_STATICTEXT7;
		static const long ID_TEXTCTRL2;
		static const long ID_BUTTON1;
		static const long ID_BUTTON2;
		static const long ID_BUTTON3;
		static const long ID_CUSTOM1;
		static const long ID_TIMER1;
		static const long ID_ALGOCHOICE;
		static const long ID_CAMPOSEVIEW;
		static const long ID_STATICTEXTALGO;
		//*)

	private:

		//(*Handlers(CDlgCalibWizardOnline)
		void OnbtnCloseClick(wxCommandEvent& event);
		void OnbtnStartClick(wxCommandEvent& event);
		void OnbtnStopClick(wxCommandEvent& event);
		void OntimCaptureTrigger(wxTimerEvent& event);
		//*)

		DECLARE_EVENT_TABLE()

		void threadProcessCorners();

		mrpt::system::TThreadHandle		m_threadCorners;	//!< The thread for corner detection.
		mrpt::obs::CObservationImagePtr  m_threadImgToProcess;  //!< Input for the thread, null if nothing pending
		bool m_threadMustClose;  //!< Close signal
		std::vector<mrpt::utils::TPixelCoordf>	m_threadResults;    //!< The detected corners, if threadResultsComputed=true
		bool m_threadResultsComputed; //!< Put to true by the thread when done with an image
		bool m_threadIsClosed;

		unsigned int m_check_size_x;
		unsigned int m_check_size_y;
		bool m_normalize_image;
		bool m_useScaramuzzaAlternativeDetector;
		mrpt::hwdrivers::CCameraSensorPtr m_video;
		CMyGLCanvas* m_3Dview_cam;
		mrpt::vision::pnp::CPnP pnp_algos;
		Eigen::MatrixXd obj_pts, img_pts, pose_mat, cam_intrinsic, I3;

		mrpt::opengl::COpenGLScenePtr	scene;
		mrpt::opengl::CSetOfObjectsPtr	cor, cor1;
		mrpt::opengl::CGridPlaneXYPtr	grid;

	public:
		typedef  bool (mrpt::vision::pnp::CPnP::*CPNP_PTR) (const Eigen::Ref<Eigen::MatrixXd> obj_pts, const Eigen::Ref<Eigen::MatrixXd> img_pts, int n, const Eigen::Ref<Eigen::MatrixXd> cam_intrinsic, Eigen::Ref<Eigen::MatrixXd> pose_mat);

		/** The list of selected frames to use in camera calibration */
		mrpt::vision::TCalibrationImageList	  m_calibFrames;

		void showCamPose();

		bool flag_pose_est;
};

#endif