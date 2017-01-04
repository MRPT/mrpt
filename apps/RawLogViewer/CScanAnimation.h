/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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
#include "MyGLCanvas.h"
#include <wx/slider.h>
#include <wx/button.h>
#include <wx/dialog.h>
//*)

// The "custom class" mpWindow, from the wxMathPlot libray by David Schalig
//  See http://sourceforge.net/projects/wxmathplot
#include <mrpt/otherlibs/mathplot/mathplot.h>

#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/opengl/CPointCloudColoured.h>

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
		wxCheckBox* cbAllowMix;
		wxRadioButton* rbFile;
		wxStaticBoxSizer* StaticBoxSizer1;
		wxStaticText* lbNumPoints;
		wxButton* btnPlay;
		CMyGLCanvas* m_plot3D;
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
		void BuildMapAndRefresh( mrpt::obs::CSensoryFrame *sf);

};

#endif
