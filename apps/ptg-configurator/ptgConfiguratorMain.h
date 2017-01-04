/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

//(*Headers(ptgConfiguratorframe)
#include <wx/notebook.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/menu.h>
#include <wx/textctrl.h>
#include <wx/checkbox.h>
#include <wx/spinctrl.h>
#include "MyGLCanvas.h"
#include <wx/slider.h>
#include <wx/panel.h>
#include <wx/choice.h>
#include <wx/button.h>
#include <wx/frame.h>
#include <wx/statusbr.h>
//*)

#include <mrpt/otherlibs/mathplot/mathplot.h>

#include <mrpt/gui/CMyRedirector.h>

#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/COpenGLViewport.h>
#include <mrpt/opengl/CAxis.h>
#include <mrpt/opengl/CMesh.h>

// JLBC: Unix X headers have these funny things...
#ifdef Button1
#	undef Button1
#	undef Button2
#	undef Button3
#	undef Button4
#	undef Button5
#	undef Button6
#	undef Button7
#endif
// To avoid conflicts between Eigen & X11 headers
#ifdef Success 
#	undef Success 
#endif

class ptgConfiguratorframe: public wxFrame
{
    public:

        ptgConfiguratorframe(wxWindow* parent,wxWindowID id = -1);
        virtual ~ptgConfiguratorframe();

    private:

        //(*Handlers(ptgConfiguratorframe)
        void OnAbout(wxCommandEvent& event);
        void OnQuit(wxCommandEvent& event);
        void OnbtnReloadParamsClick(wxCommandEvent& event);
        void OncbPTGClassSelect(wxCommandEvent& event);
        void OnedPTGIndexChange(wxSpinEvent& event);
        void OncbDrawShapePathClick(wxCommandEvent& event);
        void OncbBuildTPObsClick(wxCommandEvent& event);
        void OnbtnRebuildTPObsClick(wxCommandEvent& event);
        void OnbtnPlaceObsClick(wxCommandEvent& event);
        void OnbtnPlaceTargetClick(wxCommandEvent& event);
        void OnslidPathHighlightCmdScroll(wxScrollEvent& event);
        void OncbHighlightOnePathClick(wxCommandEvent& event);
        void OnedIndexHighlightPathChange(wxSpinEvent& event);
        void OnedCfgText(wxCommandEvent& event);
        void OnButton1Click(wxCommandEvent& event);
        void OnrbShowTPSelectSelect(wxCommandEvent& event);
        //*)

        //(*Identifiers(ptgConfiguratorframe)
        static const long ID_STATICTEXT1;
        static const long ID_CHOICE1;
        static const long ID_STATICTEXT5;
        static const long ID_STATICTEXT2;
        static const long ID_SPINCTRL1;
        static const long ID_BUTTON1;
        static const long ID_CHECKBOX1;
        static const long ID_STATICTEXT4;
        static const long ID_TEXTCTRL5;
        static const long ID_CHECKBOX3;
        static const long ID_SLIDER1;
        static const long ID_SPINCTRL2;
        static const long ID_CHECKBOX4;
        static const long ID_CHECKBOX2;
        static const long ID_TEXTCTRL3;
        static const long ID_STATICTEXT3;
        static const long ID_TEXTCTRL4;
        static const long ID_BUTTON3;
        static const long ID_BUTTON2;
        static const long ID_STATICTEXT6;
        static const long ID_TEXTCTRL6;
        static const long ID_STATICTEXT7;
        static const long ID_TEXTCTRL7;
        static const long ID_BUTTON4;
        static const long ID_TEXTCTRL1;
        static const long ID_BUTTON5;
        static const long ID_PANEL1;
        static const long ID_XY_GLCANVAS;
        static const long ID_CHECKBOX5;
        static const long ID_CHECKBOX6;
        static const long ID_CUSTOM2;
        static const long ID_PANEL2;
        static const long ID_CUSTOM1;
        static const long ID_PANEL3;
        static const long ID_STATICTEXT8;
        static const long ID_CUSTOM3;
        static const long ID_STATICTEXT9;
        static const long ID_CUSTOM4;
        static const long ID_PANEL4;
        static const long ID_STATICTEXT10;
        static const long ID_CUSTOM5;
        static const long ID_STATICTEXT11;
        static const long ID_CUSTOM6;
        static const long ID_STATICTEXT12;
        static const long ID_CUSTOM7;
        static const long ID_PANEL5;
        static const long ID_STATICTEXT13;
        static const long ID_CUSTOM8;
        static const long ID_STATICTEXT14;
        static const long ID_CUSTOM9;
        static const long ID_STATICTEXT15;
        static const long ID_CUSTOM10;
        static const long ID_PANEL6;
        static const long ID_NOTEBOOK1;
        static const long ID_TEXTCTRL2;
        static const long idMenuQuit;
        static const long idMenuAbout;
        static const long ID_STATUSBAR1;
        //*)

        //(*Declarations(ptgConfiguratorframe)
        wxStaticText* StaticText10;
        mpWindow* m_plotPathW;
        wxStaticText* StaticText9;
        wxTextCtrl* edObsY;
        wxButton* btnLoadPlugin;
        wxPanel* Panel5;
        mpWindow* m_plotHeadAngAll;
        mpWindow* m_plotPathY;
        wxNotebook* Notebook1;
        CMyGLCanvas* m_plotTPSpace;
        wxStaticText* StaticText13;
        wxStaticText* StaticText2;
        wxPanel* Panel4;
        wxStaticText* StaticText14;
        wxCheckBox* cbDrawShapePath;
        wxCheckBox* cbShowOnlySelectedTraj;
        wxSpinCtrl* edPTGIndex;
        wxStaticText* StaticText6;
        wxCheckBox* cbBuildTPObs;
        mpWindow* m_plotPathPhi;
        wxSpinCtrl* edIndexHighlightPath;
        wxButton* btnPlaceObs;
        wxStaticText* StaticText8;
        wxButton* btnRebuildTPObs;
        wxStaticText* StaticText11;
        wxPanel* Panel1;
        wxStaticText* StaticText1;
        wxCheckBox* cbShowTPObs;
        wxStaticText* StaticText3;
        wxChoice* cbPTGClass;
        wxPanel* Panel6;
        wxPanel* Panel3;
        CMyGLCanvas* m_plot;
        wxButton* btnPlaceTarget;
        mpWindow* m_plotPathYp;
        wxTextCtrl* edTargetX;
        wxCheckBox* cbHighlightOnePath;
        wxTextCtrl* edLog;
        wxStaticText* StaticText5;
        mpWindow* m_plotPathXp;
        mpWindow* m_plotHeadAngIndiv;
        wxStaticText* StaticText7;
        wxStatusBar* StatusBar1;
        mpWindow* m_plotVelCmds;
        wxTextCtrl* edObsX;
        wxButton* btnReloadParams;
        wxStaticText* StaticText15;
        wxStaticText* StaticText12;
        mpWindow* m_plotPathX;
        wxTextCtrl* edCfg;
        wxPanel* Panel2;
        wxTextCtrl* edMinDistBtwShapes;
        wxStaticText* StaticText4;
        wxCheckBox* cbShowClearance;
        wxTextCtrl* edTargetY;
        wxSlider* slidPathHighlight;
        //*)

        DECLARE_EVENT_TABLE()

		void Onplot3DMouseMove(wxMouseEvent& event);
		void Onplot3DMouseClick(wxMouseEvent& event);

		void rebuild3Dview();
		void loadPlugin();
		void dumpPTGcfgToTextBox();

		/* Methods: */
		CMyRedirector *m_myRedirector;

		/**  The state of the cursor onto the 3D view */
		enum TCursorPickState
		{
			cpsNone = 0,
			cpsPickObstacle,
			cpsPickTarget
		};

		mrpt::math::TPoint2D             m_curCursorPos; //!< Of the cursor on the 3D view (in world coordinates at Z=0)
		TCursorPickState                 m_cursorPickState;   //!< The state of the cursor onto the 3D view:

		// ========= Opengl View =======
		mrpt::opengl::COpenGLViewportPtr  gl_view_WS, gl_view_TPSpace;
		mrpt::opengl::CSetOfObjectsPtr    gl_TPSpace_TP_obstacles;
		mrpt::opengl::CMeshPtr            gl_TPSpace_clearance;
		mrpt::opengl::CCameraPtr          gl_view_TPSpace_cam;
		mrpt::opengl::CAxisPtr            gl_axis_WS, gl_axis_TPS;
		mrpt::opengl::CSetOfLinesPtr      gl_robot_ptg_prediction, gl_robot_ptg_prediction_highlight, gl_tp_obstacles;
		mrpt::opengl::CPointCloudPtr      gl_WS_obs;
		mrpt::opengl::CPointCloudPtr      gl_WS_target,gl_TP_target;

		// 2D plot views:
		mpFXYVector  *m_graph_head_all, *m_graph_head_indiv;
		mpFXYVector  *m_graph_path_x, *m_graph_path_y, *m_graph_path_phi;
		mpFXYVector  *m_graph_path_vx, *m_graph_path_vy, *m_graph_path_omega;

		static void prepareRobotPathPlot(mpWindow *plot, mpFXYVector  **graph, const std::string &name);
};

