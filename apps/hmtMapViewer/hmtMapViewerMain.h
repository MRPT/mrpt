/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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

#ifndef HMTMAPVIEWERMAIN_H
#define HMTMAPVIEWERMAIN_H

//(*Headers(hmtMapViewerFrame)
#include <wx/toolbar.h>
#include <wx/sizer.h>
#include <wx/menu.h>
#include <wx/panel.h>
#include <wx/splitter.h>
#include <wx/statusbr.h>
#include <wx/frame.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/choice.h>
#include <wx/treectrl.h>
//*)
#include <wx/timer.h>

/* Jerome Monceaux : 2011/03/08
 * Include <string> needed under snow leopard
 */
#include <string>

class CMyGLCanvas;


class hmtMapViewerFrame: public wxFrame
{
    public:

        hmtMapViewerFrame(wxWindow* parent,wxWindowID id = -1);
        virtual ~hmtMapViewerFrame();


        bool AskForOpenHMTMap( std::string &fil );

        // Returns true if OK.
        bool loadHTMSLAMFromFile( const std::string &filePath );

        // Rebuilds the tree view on the left panel.
        void rebuildTreeView();

        void updateGlobalMapView();
        void updateLocalMapView();


        std::string     m_curFileOpen;      //!< File loaded now.

    private:

        //(*Handlers(hmtMapViewerFrame)
        void OnQuit(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);
        void OnMenuLoad(wxCommandEvent& event);
        void OnMenuOverlapBtw2(wxCommandEvent& event);
        void OnMenuTranslationBtw2(wxCommandEvent& event);
        void OntreeViewSelectionChanged(wxTreeEvent& event);
        void OnmenuExportLocalMapsSelected(wxCommandEvent& event);
        void OnTopologicalModel_Gridmap(wxCommandEvent& event);
        void OnTopologicalModel_Fabmap(wxCommandEvent& event);
        //*)

        //(*Identifiers(hmtMapViewerFrame)
        static const long ID_STATICTEXT1;
        static const long ID_STATICTEXT2;
        static const long ID_CHOICE1;
        static const long ID_PANEL5;
        static const long ID_TREECTRL1;
        static const long ID_PANEL9;
        static const long ID_TEXTCTRL1;
        static const long ID_PANEL8;
        static const long ID_SPLITTERWINDOW3;
        static const long ID_PANEL1;
        static const long ID_STATICTEXT3;
        static const long ID_PANEL6;
        static const long ID_PANEL3;
        static const long ID_STATICTEXT4;
        static const long ID_PANEL7;
        static const long ID_PANEL4;
        static const long ID_SPLITTERWINDOW2;
        static const long ID_PANEL2;
        static const long ID_SPLITTERWINDOW1;
        static const long ID_MENUITEM1;
        static const long ID_MENUITEM4;
        static const long idMenuQuit;
        static const long ID_MENUITEM2;
        static const long ID_MENUITEM3;
        static const long ID_MENUITEM6;
        static const long ID_MENUITEM7;
        static const long ID_MENUITEM5;
        static const long idMenuAbout;
        static const long ID_STATUSBAR1;
        static const long ID_TOOLBARITEM1;
        static const long ID_TOOLBARITEM2;
        static const long ID_TOOLBAR1;
        //*)

		static const long ID_TIMER1;

        //(*Declarations(hmtMapViewerFrame)
        wxPanel* Panel6;
        wxPanel* Panel1;
        wxPanel* Panel7;
        wxStatusBar* StatusBar1;
        wxMenuItem* menuExportLocalMaps;
        wxChoice* cbHypos;
        wxMenu* Menu3;
        wxPanel* Panel8;
        wxPanel* Panel9;
        wxStaticText* StaticText1;
        wxPanel* Panel2;
        wxToolBarToolBase* ToolBarItem2;
        wxSplitterWindow* SplitterWindow1;
        wxStaticText* StaticText3;
        wxMenu* MenuItem6;
        wxPanel* Panel4;
        wxSplitterWindow* SplitterWindow2;
        wxMenuItem* MenuItem3;
        wxPanel* Panel5;
        wxSplitterWindow* SplitterWindow3;
        wxToolBar* ToolBar1;
        wxPanel* Panel3;
        wxTextCtrl* edLog;
        wxMenuItem* MenuItem5;
        wxStaticText* StaticText4;
        wxToolBarToolBase* ToolBarItem1;
        wxStaticText* StaticText2;
        wxMenuItem* MenuItem7;
        wxMenuItem* MenuItem4;
        wxTreeCtrl* treeView;
        wxMenuItem* MenuItem8;
        //*)

        DECLARE_EVENT_TABLE()


        CMyGLCanvas		*m_canvas_HMAP;
        CMyGLCanvas		*m_canvas_LMH;

		wxTimer timAutoLoad;
		void OntimAutoLoadTrigger(wxTimerEvent& event);
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



#endif // HMTMAPVIEWERMAIN_H
