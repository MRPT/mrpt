/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
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
#ifndef CSCANANIMATION_H
#define CSCANANIMATION_H

//(*Headers(CScanAnimation)
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/checkbox.h>
#include <wx/spinctrl.h>
#include <wx/radiobut.h>
#include <wx/slider.h>
#include <wx/button.h>
#include <wx/dialog.h>
//*)

// The "custom class" mpWindow, from the wxMathPlot libray by David Schalig
//  See http://sourceforge.net/projects/wxmathplot
#include <mrpt/otherlibs/mathplot/mathplot.h>

#include <mrpt/slam/CSensoryFrame.h>

class CScanAnimation: public wxDialog
{
	public:

		CScanAnimation(wxWindow* parent,wxWindowID id=wxID_ANY,const wxPoint& pos=wxDefaultPosition,const wxSize& size=wxDefaultSize);
		virtual ~CScanAnimation();

		//(*Declarations(CScanAnimation)
		wxBoxSizer* BoxSizer4;
		wxStaticText* StaticText22;
		wxButton* btnStop;
		wxRadioButton* rbLoaded;
		wxSlider* slPos;
		wxBoxSizer* BoxSizer5;
		wxSpinCtrl* edDelay;
		wxStaticText* StaticText2;
		wxButton* btnClose;
		wxStaticText* StaticText1;
		wxButton* btnJump;
		wxStaticText* lbNumScans;
		wxButton* btnPickInput;
		wxSpinCtrl* edIndex;
		wxFlexGridSizer* FlexGridSizer8;
		mpWindow* plotMap;
		wxCheckBox* cbAllowMix;
		wxRadioButton* rbFile;
		wxStaticBoxSizer* StaticBoxSizer1;
		wxStaticText* lbNumPoints;
		wxButton* btnPlay;
		wxTextCtrl* edFile;
		//*)

	protected:

		//(*Identifiers(CScanAnimation)
		static const long ID_RADIOBUTTON1;
		static const long ID_RADIOBUTTON2;
		static const long ID_STATICTEXT22;
		static const long ID_TEXTCTRL11;
		static const long ID_BUTTON5;
		static const long ID_BUTTON1;
		static const long ID_BUTTON2;
		static const long ID_STATICTEXT4;
		static const long ID_SPINCTRL2;
		static const long ID_CHECKBOX1;
		static const long ID_BUTTON3;
		static const long ID_CUSTOM2;
		static const long ID_SLIDER1;
		static const long ID_STATICTEXT1;
		static const long ID_SPINCTRL1;
		static const long ID_BUTTON4;
		static const long ID_STATICTEXT2;
		static const long ID_STATICTEXT3;
		//*)

	private:

		//(*Handlers(CScanAnimation)
		void OnbtnPlayClick(wxCommandEvent& event);
		void OnbtnStopClick(wxCommandEvent& event);
		void OnbtnCloseClick(wxCommandEvent& event);
		void OnslPosCmdScrollChanged(wxScrollEvent& event);
		void OnbtnJumpClick(wxCommandEvent& event);
		void OnslPosCmdScroll(wxScrollEvent& event);
		void OnbtnPickInputClick(wxCommandEvent& event);
		void OnInit(wxInitDialogEvent& event);
		void OnrbLoadedSelect(wxCommandEvent& event);
		void OnrbFile(wxCommandEvent& event);
		void OncbAllowMixClick(wxCommandEvent& event);
		//*)

		DECLARE_EVENT_TABLE()


		mpFXYVector		*m_lyMapPoints;

		bool			m_stop;

		bool			m_mixlasers;


		void RebuildMaps();
		void BuildMapAndRefresh( mrpt::slam::CSensoryFrame *sf);

};

#endif
