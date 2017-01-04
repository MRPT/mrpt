/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#ifndef NAVLOG_VIEWER_GUI_DESIGNMAIN_H
#define NAVLOG_VIEWER_GUI_DESIGNMAIN_H

//(*Headers(navlog_viewer_GUI_designDialog)
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/radiobox.h>
#include <wx/menu.h>
#include <wx/textctrl.h>
#include <wx/checkbox.h>
#include <wx/things/toggle.h>
#include <wx/slider.h>
#include <wx/panel.h>
#include <wx/button.h>
#include <wx/dialog.h>
#include <wx/timer.h>
//*)
#include <wx/frame.h>

#include <mrpt/gui.h>
#include <mrpt/nav.h>

class navlog_viewer_GUI_designDialog: public wxFrame //wxDialog
{
	private:
		// App data:
		std::vector<mrpt::utils::CSerializablePtr>  m_logdata;
		std::vector<mrpt::nav::CParameterizedTrajectoryGeneratorPtr>  m_logdata_ptg_paths; // Retrieved from the first entry in m_logdata when loading

		std::map<std::string, mrpt::gui::CDisplayWindowPlotsPtr> m_mywins;
		std::map<std::string, mrpt::gui::CDisplayWindow3DPtr>    m_mywins3D;

		mrpt::system::TTimeStamp m_log_first_tim, m_log_last_tim;


	public:
		navlog_viewer_GUI_designDialog(wxWindow* parent,wxWindowID id = -1);
		virtual ~navlog_viewer_GUI_designDialog();

	private:

		//(*Handlers(navlog_viewer_GUI_designDialog)
		void OnbtnLoadClick(wxCommandEvent& event);
		void OnbtnHelpClick(wxCommandEvent& event);
		void OnbtnQuitClick(wxCommandEvent& event);
		void OnslidLogCmdScroll(wxScrollEvent& event);
		void OnbtnPlayClick(wxCommandEvent& event);
		void OnbtnStopClick(wxCommandEvent& event);
		void OntimPlayTrigger(wxTimerEvent& event);
		void OntimAutoloadTrigger(wxTimerEvent& event);
		void OnbtnMoreOpsClick(wxCommandEvent& event);
		void OnmnuMatlabPlotsSelected(wxCommandEvent& event);
		void OnmnuSeePTGParamsSelected(wxCommandEvent& event);
		void OncbGlobalFrameClick(wxCommandEvent& event);
		void OnmnuSaveScoreMatrixSelected(wxCommandEvent& event);
		void OncbShowXYClick(wxCommandEvent& event);
		void OnrbPerPTGPlotsSelect(wxCommandEvent& event);
		//*)
		void OntimMouseXY(wxTimerEvent& event);

		//(*Identifiers(navlog_viewer_GUI_designDialog)
		static const long ID_BUTTON1;
		static const long ID_STATICTEXT1;
		static const long ID_TEXTCTRL1;
		static const long ID_BUTTON2;
		static const long ID_BUTTON3;
		static const long ID_SLIDER1;
		static const long ID_BUTTON4;
		static const long ID_BUTTON5;
		static const long ID_STATICTEXT9;
		static const long ID_TEXTCTRL3;
		static const long ID_PANEL2;
		static const long ID_STATICTEXT2;
		static const long ID_STATICTEXT3;
		static const long ID_STATICTEXT4;
		static const long ID_STATICTEXT5;
		static const long ID_STATICTEXT6;
		static const long ID_STATICTEXT7;
		static const long ID_PANEL3;
		static const long ID_RADIOBOX1;
		static const long ID_CHECKBOX1;
		static const long ID_CHECKBOX2;
		static const long ID_CHECKBOX3;
		static const long ID_CHECKBOX4;
		static const long ID_CHECKBOX5;
		static const long ID_STATICTEXT8;
		static const long ID_TEXTCTRL2;
		static const long ID_BUTTON6;
		static const long ID_PANEL1;
		static const long ID_TIMER1;
		static const long ID_TIMER2;
		static const long ID_MENUITEM2;
		static const long ID_MENUITEM1;
		static const long ID_MENUITEM3;
		//*)
		static const long ID_TIMER3;

		//(*Declarations(navlog_viewer_GUI_designDialog)
		wxButton* btnStop;
		wxTextCtrl* edShapeMinDist;
		wxButton* btnMoreOps;
		wxMenuItem* mnuSeePTGParams;
		wxPanel* Panel_AUX;
		wxStaticText* txtLogDuration;
		wxStaticText* StaticText2;
		wxCheckBox* cbDrawShapePath;
		wxRadioBox* rbPerPTGPlots;
		wxStaticText* StaticText6;
		wxTimer timAutoload;
		wxFlexGridSizer* FlexGridSizer9;
		wxCustomButton* btnQuit;
		wxPanel* Panel1;
		wxStaticText* StaticText1;
		wxMenu mnuMoreOps;
		wxStaticText* StaticText3;
		wxCheckBox* cbShowAllDebugEntries;
		wxPanel* Panel3;
		wxSlider* slidLog;
		wxTimer timPlay;
		wxStaticText* StaticText5;
		wxCheckBox* cbGlobalFrame;
		wxTextCtrl* edAnimDelayMS;
		wxStaticText* txtLogEntries;
		wxMenuItem* mnuMatlabPlots;
		wxFlexGridSizer* flexGridRightHand;
		wxCheckBox* cbShowRelPoses;
		wxCustomButton* btnHelp;
		wxTextCtrl* edLogFile;
		wxCheckBox* cbShowXY;
		wxCustomButton* btnLoad;
		wxStaticText* StaticText4;
		wxStaticText* txtSelectedPTG;
		wxButton* btnPlay;
		//*)
		wxTimer timMouseXY;

		DECLARE_EVENT_TABLE()

		void loadLogfile(const std::string &filName);
		void UpdateInfoFromLoadedLog();
};


#ifdef wxUSE_UNICODE
#define _U(x) wxString((x),wxConvUTF8)
#define _UU(x,y) wxString((x),y)
#else
#define _U(x) (x)
#define _UU(x,y) (x)
#endif


#define WX_START_TRY \
    try \
    {


#define WX_END_TRY \
    } \
	catch(std::exception &e) \
    { \
        wxMessageBox( wxString(e.what(),wxConvUTF8), wxT("Exception"), wxOK, this); \
    } \
    catch(...) \
    { \
        wxMessageBox( _("Untyped exception!"), _("Exception"), wxOK, this); \
    }



#endif // NAVLOG_VIEWER_GUI_DESIGNMAIN_H
