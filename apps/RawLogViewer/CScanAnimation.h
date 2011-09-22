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
#ifndef CSCANANIMATION_H
#define CSCANANIMATION_H

//(*Headers(CScanAnimation)
#include <wx/spinctrl.h>
#include <wx/checkbox.h>
#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/button.h>
#include <wx/radiobut.h>
#include <wx/slider.h>
#include "MyGLCanvas.h"
#include <wx/stattext.h>
#include <wx/textctrl.h>
//*)

// The "custom class" mpWindow, from the wxMathPlot libray by David Schalig
//  See http://sourceforge.net/projects/wxmathplot
#include <mrpt/otherlibs/mathplot/mathplot.h>

#include <mrpt/slam/CSensoryFrame.h>
#include <mrpt/opengl/CPointCloudColoured.h>

class CScanAnimation: public wxDialog
{
	public:

		CScanAnimation(wxWindow* parent,wxWindowID id=wxID_ANY,const wxPoint& pos=wxDefaultPosition,const wxSize& size=wxDefaultSize);
		virtual ~CScanAnimation();

		//(*Declarations(CScanAnimation)
		wxStaticText* StaticText22;
		wxCheckBox* cbAllowMix;
		wxFlexGridSizer* FlexGridSizer8;
		wxButton* btnClose;
		wxSlider* slPos;
		wxButton* btnPickInput;
		wxTextCtrl* edFile;
		wxButton* btnPlay;
		wxSpinCtrl* edIndex;
		wxStaticText* StaticText1;
		wxRadioButton* rbFile;
		wxButton* btnJump;
		wxStaticText* lbNumPoints;
		CMyGLCanvas* m_plot3D;
		wxStaticText* lbNumScans;
		wxButton* btnStop;
		wxBoxSizer* BoxSizer4;
		wxRadioButton* rbLoaded;
		wxStaticText* StaticText2;
		wxSpinCtrl* edDelay;
		wxBoxSizer* BoxSizer5;
		wxStaticBoxSizer* StaticBoxSizer1;
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
		static const long ID_XY_GLCANVAS;
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

		bool			m_stop;
		bool			m_mixlasers;

		struct TRenderObject
		{
			mrpt::system::TTimeStamp       timestamp;
			mrpt::opengl::CRenderizablePtr obj;
		};
		typedef std::map<std::string,TRenderObject> TListGlObjects;
		TListGlObjects  m_gl_objects;  //!< All the observations added to the map.

		void RebuildMaps();
		void BuildMapAndRefresh( mrpt::slam::CSensoryFrame *sf);

};

#endif
