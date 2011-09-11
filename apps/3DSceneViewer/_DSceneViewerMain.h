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

#ifndef _DSCENEVIEWERMAIN_H
#define _DSCENEVIEWERMAIN_H

#include <wx/menu.h>
#include <wx/toolbar.h>
#include <wx/frame.h>
#include <wx/timer.h>
#include <wx/statusbr.h>
#include <wx/msgdlg.h>
#include <wx/artprov.h>
#include <wx/bitmap.h>
#include <wx/intl.h>
#include <wx/image.h>
#include <wx/string.h>


class CDlgCamTracking;

#include <mrpt/gui/CMyGLCanvasBase.h>


class CMyGLCanvas : public mrpt::gui::CMyGLCanvasBase
{
public:
    CMyGLCanvas( wxWindow *parent, wxWindowID id = wxID_ANY,
                 const wxPoint& pos = wxDefaultPosition,
                 const wxSize& size = wxDefaultSize,
                 long style = 0, const wxString& name = _T("CMyGLCanvasBase") )
		: CMyGLCanvasBase(parent,id,pos,size,style,name)
	{
	}


	void OnCharCustom( wxKeyEvent& event );

	void OnPreRender();
	void OnPostRender();
	void OnPostRenderSwapBuffers(double At, wxPaintDC &dc);
	void OnRenderError( const wxString &str );

};



class _DSceneViewerFrame: public wxFrame
{
	friend class CMyGLCanvas;
	friend class CDlgCamTracking;

    public:

        _DSceneViewerFrame(wxWindow* parent,wxWindowID id = -1);
        virtual ~_DSceneViewerFrame();

    private:

        //(*Handlers(_DSceneViewerFrame)
        void OnQuit(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);
        void OnNewScene(wxCommandEvent& event);
        void OnOpenFile(wxCommandEvent& event);
        void OntimLoadFileCmdLineTrigger(wxTimerEvent& event);
        void OnbtnAutoplayClicked(wxCommandEvent& event);
        void OnMenuBackColor(wxCommandEvent& event);
        void OnMenuOptions(wxCommandEvent& event);
        void OnPrevious(wxCommandEvent& event);
        void OnNext(wxCommandEvent& event);
        void OnClose(wxCloseEvent& event);
        void OnBtnRecordClicked(wxCommandEvent& event);
        void OnbtnOrthoClicked(wxCommandEvent& event);
        void OnReload(wxCommandEvent& event);
        void OnInsert3DS(wxCommandEvent& event);
        void OnMenuSave(wxCommandEvent& event);
        void OnChangeCameraPose(wxCommandEvent& event);
        void OnTravellingTrigger(wxTimerEvent& event);
        void OnStartCameraTravelling(wxCommandEvent& event);
        void OnClose1(wxCloseEvent& event);
        void OnMenuAddSICK(wxCommandEvent& event);
        void OnMenuDeleteAll(wxCommandEvent& event);
        void OnMenuItem14Selected(wxCommandEvent& event);
        void OnMenuCameraTrackingArbitrary(wxCommandEvent& event);
        void OnmnuItemShowCloudOctreesSelected(wxCommandEvent& event);
        void OnmnuItemChangeMaxPointsPerOctreeNodeSelected(wxCommandEvent& event);
        void OnmnuSceneStatsSelected(wxCommandEvent& event);
        void OnMenuItemImportPLYPointCloud(wxCommandEvent& event);
        void OnMenuItemExportPointsPLY(wxCommandEvent& event);
        //*)

		void OntimAutoplay(wxTimerEvent& event);

        //(*Identifiers(_DSceneViewerFrame)
        static const long ID_MENUITEM1;
        static const long ID_MENUITEM2;
        static const long ID_MENUITEM5;
        static const long ID_MENUITEM7;
        static const long ID_MENUITEM6;
        static const long ID_MENUITEM20;
        static const long ID_MENUITEM19;
        static const long ID_MENUITEM22;
        static const long ID_MENUITEM21;
        static const long ID_MENUITEM12;
        static const long ID_MENUITEM18;
        static const long idMenuQuit;
        static const long ID_MENUITEM4;
        static const long ID_MENUITEM3;
        static const long ID_MENUITEM15;
        static const long ID_MENUITEM17;
        static const long ID_MENUITEM16;
        static const long ID_MENUITEM11;
        static const long ID_MENUITEM9;
        static const long ID_MENUITEM8;
        static const long ID_MENUITEM10;
        static const long ID_MENUITEM14;
        static const long ID_MENUITEM13;
        static const long idMenuAbout;
        static const long ID_STATUSBAR1;
        static const long ID_TOOLBARITEM7;
        static const long ID_TOOLBARITEM1;
        static const long ID_TOOLBARITEM5;
        static const long ID_TOOLBARITEM6;
        static const long ID_TOOLBARITEM11;
        static const long ID_TOOLBARITEM2;
        static const long ID_TOOLBARITEM10;
        static const long ID_TOOLBARITEM8;
        static const long ID_TOOLBARITEM9;
        static const long ID_TOOLBARITEM3;
        static const long ID_TOOLBARITEM4;
        static const long ID_TOOLBAR1;
        static const long ID_TIMER1;
        //*)

        static const long ID_TRAVELLING_TIMER;
        static const long ID_TIMER_AUTOPLAY;

        //(*Declarations(_DSceneViewerFrame)
        wxMenu* MenuItem11;
        wxMenu* MenuItem18;
        wxToolBarToolBase* ToolBarItem5;
        wxStatusBar* StatusBar1;
        wxMenuItem* mnuSceneStats;
        wxMenuItem* MenuItem16;
        wxToolBarToolBase* ToolBarItem11;
        wxToolBarToolBase* ToolBarItem6;
        wxMenu* Menu3;
        wxMenuItem* MenuItem19;
        wxToolBarToolBase* btnAutoplay;
        wxTimer timLoadFileCmdLine;
        wxToolBarToolBase* ToolBarItem2;
        wxMenuItem* MenuItem21;
        wxMenuItem* mnuItemShowCloudOctrees;
        wxMenuItem* MenuItem9;
        wxToolBarToolBase* ToolBarItem10;
        wxToolBarToolBase* btnOrtho;
        wxToolBar* ToolBar1;
        wxMenuItem* mnuItemChangeMaxPointsPerOctreeNode;
        wxToolBarToolBase* ToolBarItem4;
        wxMenuItem* MenuItem5;
        wxToolBarToolBase* ToolBarItem1;
        wxMenu* MenuItem17;
        wxMenuItem* MenuItem10;
        wxToolBarToolBase* btnRecord;
        wxToolBarToolBase* ToolBarItem3;
        wxMenuItem* MenuItem6;
        wxMenuItem* MenuItem7;
        wxMenuItem* MenuItem8;
        wxMenuItem* MenuItem14;
        wxMenu* MenuItem20;
        //*)

		CMyGLCanvas	*m_canvas;
		wxTimer		*m_autoplayTimer;

        wxTimer 	m_tTravelling;
        bool		m_travelling_is_arbitrary;
        mrpt::system::TTimeStamp m_travelling_start_time;


		int m_nTicksNumber;
		double m_nCurrentAzimuth;
		int maxv;
		void loadFromFile( const std::string &fil, bool isInASequence = false );
		void updateTitle();
		void clear_all_octrees_in_scene();


		CDlgCamTracking *m_dlg_tracking;


        DECLARE_EVENT_TABLE()
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



#endif // _DSCENEVIEWERMAIN_H
