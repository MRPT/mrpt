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


#ifndef HMT_SLAM_GUIMAIN_H
#define HMT_SLAM_GUIMAIN_H

#include <wx/log.h>
#include "CDlgLog.h"


//(*Headers(hmt_slam_guiFrame)
#include <wx/sizer.h>
#include <wx/notebook.h>
#include <wx/button.h>
#include <wx/menu.h>
#include <wx/panel.h>
#include <wx/splitter.h>
#include <wx/statusbr.h>
#include <wx/statline.h>
#include <wx/frame.h>
#include "MyGLCanvas.h"
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/choice.h>
#include <wx/treectrl.h>
#include <wx/things/toggle.h>
//*)



#include <mrpt/slam.h>
#include <mrpt/hmtslam.h>


// JLBC: Unix X headers have these funny things...
#ifdef Button1
#	undef Button1
#	undef Button2
#	undef Button3
#endif

class CDlgLog;


class hmt_slam_guiFrame: public wxFrame
{
	friend class CDlgLog;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    public:

        hmt_slam_guiFrame(wxWindow* parent,wxWindowID id = -1);
        virtual ~hmt_slam_guiFrame();

    private:

        //(*Handlers(hmt_slam_guiFrame)
        void OnQuit(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);
        void OnNotebook2PageChanged(wxNotebookEvent& event);
        void OnbtnResetClick(wxCommandEvent& event);
        void OnbtnLoadClick(wxCommandEvent& event);
        void OnbtnSaveClick(wxCommandEvent& event);
        void OnbtnStartClick(wxCommandEvent& event);
        void OnbtnPauseClick(wxCommandEvent& event);
        void OnMenuSetSLAMParameter(wxCommandEvent& event);
        void OnbtnPickRawlogClick(wxCommandEvent& event);
        void OnbtnShowLogWinClick(wxCommandEvent& event);
        //*)

        //(*Identifiers(hmt_slam_guiFrame)
        static const long ID_BUTTON1;
        static const long ID_STATICLINE3;
        static const long ID_BUTTON2;
        static const long ID_BUTTON3;
        static const long ID_STATICLINE1;
        static const long ID_BUTTON4;
        static const long ID_BUTTON6;
        static const long ID_STATICLINE2;
        static const long ID_BUTTON12;
        static const long ID_BUTTON10;
        static const long ID_BUTTON5;
        static const long ID_PANEL1;
        static const long ID_STATICTEXT1;
        static const long ID_TEXTCTRL1;
        static const long ID_BUTTON11;
        static const long ID_STATICTEXT6;
        static const long ID_TEXTCTRL2;
        static const long ID_PANEL3;
        static const long ID_STATICTEXT2;
        static const long ID_CHOICE1;
        static const long ID_TREECTRL1;
        static const long ID_PANEL15;
        static const long ID_PANEL17;
        static const long ID_PANEL16;
        static const long ID_NOTEBOOK2;
        static const long ID_STATICTEXT5;
        static const long ID_BUTTON7;
        static const long ID_BUTTON8;
        static const long ID_BUTTON9;
        static const long ID_PANEL14;
        static const long ID_PANEL8;
        static const long ID_PANEL5;
        static const long ID_STATICTEXT3;
        static const long ID_XY_GLCANVAS;
        static const long ID_PANEL11;
        static const long ID_PANEL10;
        static const long ID_STATICTEXT4;
        static const long ID_CUSTOM1;
        static const long ID_PANEL13;
        static const long ID_PANEL12;
        static const long ID_SPLITTERWINDOW2;
        static const long ID_PANEL7;
        static const long ID_SPLITTERWINDOW1;
        static const long ID_PANEL4;
        static const long ID_NOTEBOOK1;
        static const long ID_PANEL2;
        static const long ID_MENUITEM1;
        static const long ID_MENUITEM2;
        static const long ID_MENUITEM3;
        static const long idMenuQuit;
        static const long ID_MENUITEM6;
        static const long ID_MENUITEM4;
        static const long ID_MENUITEM5;
        static const long idMenuAbout;
        static const long ID_STATUSBAR1;
        //*)

        //(*Declarations(hmt_slam_guiFrame)
        wxCustomButton* btnShowLogWin;
        wxButton* btnPickRawlog;
        wxPanel* Panel11;
        wxPanel* Panel6;
        wxPanel* Panel1;
        wxTextCtrl* edRestParams;
        wxPanel* Panel7;
        wxStaticLine* StaticLine2;
        wxCustomButton* btnReset;
        wxStatusBar* StatusBar1;
        wxChoice* cbHypos;
        wxMenu* Menu3;
        wxPanel* Panel12;
        wxCustomButton* btnSave;
        wxCustomButton* btnAbout;
        wxPanel* Panel8;
        wxStaticText* StaticText1;
        wxPanel* Panel10;
        wxCustomButton* btnAddNode;
        wxPanel* Panel2;
        wxSplitterWindow* SplitterWindow1;
        wxTextCtrl* edInputRawlog;
        wxStaticText* StaticText3;
        wxPanel* Panel4;
        wxSplitterWindow* SplitterWindow2;
        wxMenuItem* MenuItem3;
        wxCustomButton* btnAddArc;
        wxStaticLine* StaticLine1;
        wxPanel* panTreeView;
        wxCustomButton* btnStart;
        wxPanel* panMapView;
        wxStaticLine* StaticLine3;
        wxCustomButton* btnLoad;
        wxPanel* Panel3;
        wxPanel* Panel15;
        wxCustomButton* btnPause;
        wxCustomButton* btnImportArea;
        wxPanel* panConfig;
        wxNotebook* Notebook2;
        wxPanel* Panel14;
        wxStaticText* StaticText4;
        wxStaticText* StaticText5;
        wxStaticText* StaticText2;
        wxNotebook* Notebook1;
        wxMenuItem* MenuItem7;
        wxMenuItem* MenuItem4;
        wxStaticText* StaticText6;
        wxTreeCtrl* treeView;
        CMyGLCanvas* m_glLocalArea;
        CMyGLCanvas* m_glGlobalHMTMap;
        wxCustomButton* btnQuit;
        //*)

        DECLARE_EVENT_TABLE()

        // DATA =============================
		CDlgLog  *m_logWin;

		mrpt::hmtslam::CHMTSLAM *m_hmtslam;   //!< The main HMT-SLAM object, keeps the HMT map and does HMT SLAM.


		// DATA ABOUT THREAD OF HMT-SLAM ---------------|
		mrpt::system::TThreadHandle  m_hThreadHMTSLAM; //!< Handle of HMT-SLAM thread

		void thread_HMTSLAM();

		enum THREAD_OPCODE
		{
			OP_QUIT_THREAD  = 0,
			OP_START_SLAM,
			OP_PAUSE_SLAM
		};

		struct TThreadMsg
		{
			TThreadMsg(THREAD_OPCODE op) : opcode(op)
			{}

			THREAD_OPCODE opcode;
		};

		mrpt::utils::CThreadSafeQueue<TThreadMsg>  m_thread_in_queue, m_thread_out_queue;

		// ---------------------------------------------|


		std::string		m_curFileOpen;


		// METHODS:
		bool AskForOpenHMTMap( std::string &fil );
		bool loadHTMSLAMFromFile( const std::string &filePath );

		void loadHMTConfigFromSettings();

		void rebuildTreeView();
		void updateGlobalMapView();
		void updateLocalMapView();

		void updateAllMapViews()
		{
			rebuildTreeView();
			updateGlobalMapView();
			updateLocalMapView();
		}


};


// Auxiliary class for the tree data
class CItemData : public wxTreeItemData
{
public:
	mrpt::utils::CSerializablePtr m_ptr;
	size_t          m_itemIndex;

	CItemData( mrpt::utils::CSerializablePtr ptr, size_t itemIndex) : m_ptr(ptr), m_itemIndex(itemIndex)
	{
	}
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



#endif // HMT_SLAM_GUIMAIN_H
