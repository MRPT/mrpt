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
#include <wx/statline.h>

#include <wx/things/toggle.h>

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
        void OnMenuItemHighResRender(wxCommandEvent& event);
        void OnmnuSelectionDeleteSelected(wxCommandEvent& event);
        void OnmnuSelectionScaleSelected(wxCommandEvent& event);
        void OnmnuSelectByClassSelected(wxCommandEvent& event);
        void OnmnuSelectNoneSelected(wxCommandEvent& event);
        //*)

		void OntimAutoplay(wxTimerEvent& event);

        //(*Identifiers(_DSceneViewerFrame)
        static const long ID_BUTTON1;
        static const long ID_BUTTON2;
        static const long ID_STATICLINE1;
        static const long ID_BUTTON3;
        static const long ID_BUTTON4;
        static const long ID_BUTTON5;
        static const long ID_STATICLINE2;
        static const long ID_BUTTON6;
        static const long ID_BUTTON7;
        static const long ID_BUTTON8;
        static const long ID_BUTTON9;
        static const long ID_STATICLINE3;
        static const long ID_BUTTON10;
        static const long ID_BUTTON11;
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
        static const long ID_MENUITEM23;
        static const long ID_MENUITEM18;
        static const long idMenuQuit;
        static const long ID_MENUITEM24;
        static const long ID_MENUITEM26;
        static const long ID_MENUITEM27;
        static const long ID_MENUITEM28;
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
        static const long ID_TIMER1;
        //*)

        static const long ID_TRAVELLING_TIMER;
        static const long ID_TIMER_AUTOPLAY;

        //(*Declarations(_DSceneViewerFrame)
        wxMenuItem* MenuItem8;
        wxMenuItem* mnuSelectionDelete;
        wxMenuItem* MenuItem7;
        wxCustomButton* btnCapture;
        wxCustomButton* btnNew;
        wxCustomButton* btnNext;
        wxMenuItem* MenuItem5;
        wxMenu* Menu3;
        wxMenu* MenuItem20;
        wxMenuItem* MenuItem14;
        wxCustomButton* btnToolbarOpen;
        wxMenuItem* mnuItemChangeMaxPointsPerOctreeNode;
        wxCustomButton* btnQuit;
        wxCustomButton* btnAutoplay;
        wxMenuItem* MenuItem22;
        wxMenuItem* MenuItem10;
        wxStaticLine* StaticLine2;
        wxCustomButton* btnOptions;
        wxMenuItem* mnuSelectNone;
        wxMenuItem* mnuSceneStats;
        wxMenuItem* mnuSelectionScale;
        wxCustomButton* btnOrtho;
        wxStatusBar* StatusBar1;
        wxMenuItem* MenuItem6;
        wxStaticLine* StaticLine3;
        wxStaticLine* StaticLine1;
        wxTimer timLoadFileCmdLine;
        wxCustomButton* btnAbout;
        wxMenuItem* MenuItem21;
        wxMenuItem* mnuSelectByClass;
        wxMenuItem* MenuItem16;
        wxMenuItem* MenuItem9;
        wxMenu* MenuItem18;
        wxMenuItem* mnuItemShowCloudOctrees;
        wxCustomButton* btnReload;
        wxCustomButton* btnPrev;
        wxMenu* Menu4;
        wxMenuItem* MenuItem19;
        wxMenu* MenuItem11;
        wxMenu* MenuItem17;
        //*)

		CMyGLCanvas	*m_canvas;
		wxTimer		*m_autoplayTimer;

		std::vector<mrpt::opengl::CRenderizablePtr> m_selected_gl_objects; //!< The list of currently selected objects

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
