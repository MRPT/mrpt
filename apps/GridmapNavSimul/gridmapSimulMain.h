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

#ifndef gridmapSimulMAIN_H
#define gridmapSimulMAIN_H

//(*Headers(gridmapSimulFrame)
#include <wx/spinctrl.h>
#include <wx/checkbox.h>
#include <wx/sizer.h>
#include <wx/button.h>
#include <wx/menu.h>
#include <wx/panel.h>
#include <wx/splitter.h>
#include <wx/frame.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/timer.h>
//*)

class CMyGLCanvas;


#define GRIDMAPSIMUL_VERSION  "1.0"

class gridmapSimulFrame: public wxFrame
{
	public:
        gridmapSimulFrame(wxWindow* parent,wxWindowID id = -1);
        virtual ~gridmapSimulFrame();

    private:

        //(*Handlers(gridmapSimulFrame)
        void OnbtnQuitClick(wxCommandEvent& event);
        void OntimRunTrigger(wxTimerEvent& event);
        void OnbtnStartClick(wxCommandEvent& event);
        void OnbtnEndClick(wxCommandEvent& event);
        void OnbtnExploreClick(wxCommandEvent& event);
        void OnbtnSetLaserClick(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);
        void OnbtnResimulateClick(wxCommandEvent& event);
        //*)
		void OnMenuLoadMap(wxCommandEvent& event);

		void update_grid_map_3d();

        //(*Identifiers(gridmapSimulFrame)
        static const long ID_STATICTEXT1;
        static const long ID_TEXTCTRL1;
        static const long ID_STATICTEXT2;
        static const long ID_TEXTCTRL2;
        static const long ID_STATICTEXT3;
        static const long ID_TEXTCTRL3;
        static const long ID_STATICTEXT8;
        static const long ID_TEXTCTRL5;
        static const long ID_BUTTON5;
        static const long ID_STATICTEXT16;
        static const long ID_SPINCTRL1;
        static const long ID_STATICTEXT9;
        static const long ID_TEXTCTRL6;
        static const long ID_STATICTEXT13;
        static const long ID_TEXTCTRL7;
        static const long ID_STATICTEXT10;
        static const long ID_TEXTCTRL8;
        static const long ID_STATICTEXT11;
        static const long ID_TEXTCTRL9;
        static const long ID_STATICTEXT12;
        static const long ID_TEXTCTRL10;
        static const long ID_STATICTEXT14;
        static const long ID_TEXTCTRL11;
        static const long ID_STATICTEXT4;
        static const long ID_BUTTON4;
        static const long ID_CHECKBOX1;
        static const long ID_CHECKBOX_RAWLOG_FORMAT;
        static const long ID_STATICTEXT6;
        static const long ID_STATICTEXT7;
        static const long ID_BUTTON6;
        static const long ID_PANEL3;
        static const long ID_BUTTON1;
        static const long ID_BUTTON2;
        static const long ID_STATICTEXT5;
        static const long ID_TEXTCTRL4;
        static const long ID_BUTTON3;
        static const long ID_STATICTEXT15;
        static const long ID_TEXTCTRL12;
        static const long ID_PANEL6;
        static const long ID_PANEL4;
        static const long ID_PANEL2;
        static const long ID_PANEL1;
        static const long ID_PANEL5;
        static const long ID_SPLITTERWINDOW1;
        static const long ID_TIMER1;
        static const long ID_MENUITEM1;
        static const long ID_MENUITEM_LOADMAP;
        static const long ID_MENUITEM2;
        static const long ID_MENUITEM3;
        //*)
		static const long ID_TEXTCTRL_INPUT;

        //(*Declarations(gridmapSimulFrame)
        wxTextCtrl* edSpan;
        wxMenuItem* MenuItem1;
        wxMenuItem* MenuItemLoadMap;
        wxTextCtrl* edOutGT;
        wxButton* btnExplore;
        wxSpinCtrl* edDecimate;
        wxPanel* Panel1;
        wxStaticText* StaticText13;
        wxStaticText* StaticText14;
        wxStaticText* StaticText15;
        wxMenu* Menu1;
        wxButton* btnSetLaser;
        wxCheckBox* cbJoy;
        wxCheckBox* cbRawlogSFformat;
        wxTextCtrl* edAys;
        wxStaticText* StaticText1;
        wxStaticText* StaticText10;
        wxStaticText* StaticText16;
        wxPanel* Panel2;
        wxSplitterWindow* SplitterWindow1;
        wxStaticText* StaticText3;
        wxPanel* Panel4;
        wxButton* btnExit;
        wxTextCtrl* edAyb;
        wxTextCtrl* edStdNoiseAng;
        wxButton* btnResimulate;
        wxPanel* Panel5;
        wxStaticText* StaticText8;
        wxTextCtrl* edOutFile;
        wxStaticText* StaticText12;
        wxTextCtrl* edCount;
        wxTimer timRun;
        wxPanel* Panel3;
        wxStaticText* StaticText7;
        wxTextCtrl* edStdNoise;
        wxStaticText* StaticText4;
        wxTextCtrl* edAxb;
        wxButton* btnEnd;
        wxStaticText* StaticText5;
        wxMenuBar* MenuBar1;
        wxStaticText* StaticText2;
        wxTextCtrl* edApb;
        wxPanel* panelGL;
        wxStaticText* StaticText6;
        wxMenu* Menu2;
        wxTextCtrl* edAps;
        wxTextCtrl* edAxs;
        wxButton* btnStart;
        wxStaticText* StaticText9;
        wxStaticText* StaticText11;
        //*)

        DECLARE_EVENT_TABLE()


        CMyGLCanvas		*m_canvas;
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



#endif // gridmapSimulMAIN_H
