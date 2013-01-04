/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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


#ifndef NAVLOG_VIEWER_GUI_DESIGNMAIN_H
#define NAVLOG_VIEWER_GUI_DESIGNMAIN_H

//(*Headers(navlog_viewer_GUI_designDialog)
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/menu.h>
#include <wx/textctrl.h>
#include <wx/things/toggle.h>
#include <wx/slider.h>
#include <wx/panel.h>
#include <wx/button.h>
#include <wx/dialog.h>
#include <wx/timer.h>
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
        static const long ID_STATICTEXT6;
        static const long ID_STATICTEXT7;
        static const long ID_PANEL3;
        static const long ID_BUTTON6;
        static const long ID_PANEL1;
        static const long ID_TIMER1;
        static const long ID_TIMER2;
        static const long ID_MENUITEM1;
        //*)

        //(*Declarations(navlog_viewer_GUI_designDialog)
        wxButton* btnStop;
        wxButton* btnMoreOps;
        wxPanel* Panel_AUX;
        wxStaticText* txtLogDuration;
        wxStaticText* StaticText2;
        wxTimer timAutoload;
        wxFlexGridSizer* FlexGridSizer9;
        wxCustomButton* btnQuit;
        wxPanel* Panel1;
        wxStaticText* StaticText1;
        wxMenu mnuMoreOps;
        wxStaticText* StaticText3;
        wxPanel* Panel3;
        wxSlider* slidLog;
        wxTimer timPlay;
        wxStaticText* txtLogEntries;
        wxMenuItem* mnuMatlabPlots;
        wxFlexGridSizer* flexGridRightHand;
        wxCustomButton* btnHelp;
        wxTextCtrl* edLogFile;
        wxCustomButton* btnLoad;
        wxStaticText* StaticText4;
        wxStaticText* txtSelectedPTG;
        wxButton* btnPlay;
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
