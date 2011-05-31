/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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


#ifndef CDLGCALIBWIZARDONLINE_H
#define CDLGCALIBWIZARDONLINE_H

//(*Headers(CDlgCalibWizardOnline)
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/radiobox.h>
#include <wx/textctrl.h>
#include <wx/checkbox.h>
#include <wx/spinctrl.h>
#include <wx/button.h>
#include <wx/dialog.h>
#include <mrpt/gui/WxUtils.h>
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
		wxButton* btnStop;
		wxCheckBox* cbNormalize;
		wxTextCtrl* edLengthX;
		mrpt::gui::wxMRPTImageControl* m_realtimeview;
		wxStaticText* lbProgress;
		wxRadioBox* rbMethod;
		wxStaticText* StaticText2;
		wxButton* btnStart;
		wxButton* btnClose;
		wxStaticText* StaticText6;
		wxSpinCtrl* edNumCapture;
		wxTextCtrl* edLengthY;
		wxSpinCtrl* edSizeY;
		wxStaticText* StaticText1;
		wxTextCtrl* txtLog;
		wxStaticText* StaticText3;
		wxStaticText* StaticText5;
		wxSpinCtrl* edSizeX;
		wxFlexGridSizer* FlexGridSizer1;
		wxStaticText* StaticText4;
		wxTimer timCapture;
		mrpt::gui::CPanelCameraSelection* m_panelCamera;
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
