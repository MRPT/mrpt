/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#ifndef CDLGCALIBWIZARDONLINE_H
#define CDLGCALIBWIZARDONLINE_H

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


class CDlgCalibWizardOnline: public wxDialog
{
	public:

		CDlgCalibWizardOnline(wxWindow* parent,wxWindowID id=wxID_ANY,const wxPoint& pos=wxDefaultPosition,const wxSize& size=wxDefaultSize);
		virtual ~CDlgCalibWizardOnline();

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
		static const long ID_CHOICE1;
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
		bool 							m_threadMustClose;  //!< Close signal
		std::vector<mrpt::utils::TPixelCoordf>	m_threadResults;    //!< The detected corners, if threadResultsComputed=true
		bool							m_threadResultsComputed; //!< Put to true by the thread when done with an image
		bool							m_threadIsClosed;

		unsigned int  m_check_size_x;
		unsigned int  m_check_size_y;
		bool		  m_normalize_image;
		bool		  m_useScaramuzzaAlternativeDetector;


		mrpt::hwdrivers::CCameraSensorPtr  m_video;

	public:
		/** The list of selected frames to use in camera calibration */
		mrpt::vision::TCalibrationImageList	  m_calibFrames;

};

#endif
