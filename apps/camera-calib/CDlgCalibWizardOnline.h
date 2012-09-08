/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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
//*)

#include <mrpt/vision.h>
#include <mrpt/hwdrivers/CCameraSensor.h>
#include "../wx-common/CMyRedirector.h"


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
		mrpt::slam::CObservationImagePtr  m_threadImgToProcess;  //!< Input for the thread, null if nothing pending
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
