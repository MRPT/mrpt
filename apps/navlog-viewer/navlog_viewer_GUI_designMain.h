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


#ifndef NAVLOG_VIEWER_GUI_DESIGNMAIN_H
#define NAVLOG_VIEWER_GUI_DESIGNMAIN_H

//(*Headers(navlog_viewer_GUI_designDialog)
#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/button.h>
#include <wx/menu.h>
#include <wx/panel.h>
#include <wx/slider.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/timer.h>
#include <wx/things/toggle.h>
//*)
#include <wx/frame.h>

#include <mrpt/gui.h>
#include <mrpt/reactivenav.h>

class navlog_viewer_GUI_designDialog: public wxFrame //wxDialog
{
	private:
		// App data:
		std::vector<mrpt::utils::CSerializablePtr>  m_logdata;
		std::map<std::string, mrpt::gui::CDisplayWindowPlotsPtr> m_mywins;

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
        //*)

        //(*Identifiers(navlog_viewer_GUI_designDialog)
        static const long ID_BUTTON1;
        static const long ID_STATICTEXT1;
        static const long ID_TEXTCTRL1;
        static const long ID_BUTTON2;
        static const long ID_BUTTON3;
        static const long ID_SLIDER1;
        static const long ID_BUTTON4;
        static const long ID_BUTTON5;
        static const long ID_PANEL2;
        static const long ID_STATICTEXT2;
        static const long ID_STATICTEXT3;
        static const long ID_STATICTEXT4;
        static const long ID_STATICTEXT5;
        static const long ID_PANEL3;
        static const long ID_BUTTON6;
        static const long ID_PANEL1;
        static const long ID_TIMER1;
        static const long ID_TIMER2;
        static const long ID_MENUITEM1;
        //*)

        //(*Declarations(navlog_viewer_GUI_designDialog)
        wxMenu mnuMoreOps;
        wxPanel* Panel1;
        wxTextCtrl* edLogFile;
        wxButton* btnPlay;
        wxStaticText* StaticText1;
        wxFlexGridSizer* FlexGridSizer9;
        wxStaticText* StaticText3;
        wxButton* btnStop;
        wxButton* btnMoreOps;
        wxTimer timPlay;
        wxCustomButton* btnLoad;
        wxPanel* Panel3;
        wxStaticText* StaticText2;
        wxStaticText* txtLogDuration;
        wxTimer timAutoload;
        wxSlider* slidLog;
        wxMenuItem* mnuMatlabPlots;
        wxPanel* Panel_AUX;
        wxStaticText* txtLogEntries;
        wxFlexGridSizer* flexGridRightHand;
        wxCustomButton* btnHelp;
        wxCustomButton* btnQuit;
        //*)

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


#define NAVLOGVIEWER_VERSION  "1.0"


#endif // NAVLOG_VIEWER_GUI_DESIGNMAIN_H
