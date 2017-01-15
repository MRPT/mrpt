/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef HOLONOMIC_NAVIGATOR_DEMOMAIN_H
#define HOLONOMIC_NAVIGATOR_DEMOMAIN_H

//(*Headers(reactive_navigator_demoframe)
#include <wx/notebook.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/radiobox.h>
#include <wx/menu.h>
#include <wx/textctrl.h>
#include <wx/checkbox.h>
#include <wx/things/toggle.h>
#include <wx/splitter.h>
#include "MyGLCanvas.h"
#include <wx/panel.h>
#include <wx/choice.h>
#include <wx/frame.h>
#include <wx/timer.h>
#include <wx/statusbr.h>
//*)

#include <mrpt/nav.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CDisk.h>
#include <mrpt/opengl/CSimpleLine.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CPlanarLaserScan.h>			// It's in the lib mrpt-maps
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/utils/CFileOutputStream.h>

#include <memory> // unique_ptr<>

#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>
#include <mrpt/gui/CMyRedirector.h>

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


class reactive_navigator_demoframe: public wxFrame
{
    public:

        reactive_navigator_demoframe(wxWindow* parent,wxWindowID id = -1);
        virtual ~reactive_navigator_demoframe();

    private:

        //(*Handlers(reactive_navigator_demoframe)
        void OnQuit(wxCommandEvent& event);
        void OnAbout(wxCommandEvent& event);
        void OnbtnPlaceRobotClick(wxCommandEvent& event);
        void OnbtnPlaceTargetClick(wxCommandEvent& event);
        void OnbtnStartClick(wxCommandEvent& event);
        void OnbtnStopClick(wxCommandEvent& event);
        void OntimRunSimulTrigger(wxTimerEvent& event);
        void OnMenuItemChangeVisibleStuff(wxCommandEvent& event);
        void OnMenuItemClearRobotPath(wxCommandEvent& event);
        void OnbtnLoadMapClick(wxCommandEvent& event);
        void OnNotebook1PageChanged(wxNotebookEvent& event);
        void OnNotebook1PageChanged1(wxNotebookEvent& event);
        void OnedManualKinRampsText(wxCommandEvent& event);
        void OnbtnQuitClick(wxCommandEvent& event);
        void OnrbKinTypeSelect(wxCommandEvent& event);
        void OnbtnDrawMapObsClick(wxCommandEvent& event);
        void OnbtnEmptyMapClick(wxCommandEvent& event);
        void OnbtnSaveMapClick(wxCommandEvent& event);
        void OnbtnDrawEmptyClick(wxCommandEvent& event);
        void OnbtnSetWaypointSeqClick(wxCommandEvent& event);
        //*)

        //(*Identifiers(reactive_navigator_demoframe)
        static const long ID_BUTTON4;
        static const long ID_BUTTON5;
        static const long ID_BUTTON7;
        static const long ID_BUTTON12;
        static const long ID_BUTTON6;
        static const long ID_BUTTON1;
        static const long ID_BUTTON9;
        static const long ID_BUTTON8;
        static const long ID_BUTTON11;
        static const long ID_BUTTON10;
        static const long ID_BUTTON2;
        static const long ID_BUTTON3;
        static const long ID_RADIOBOX2;
        static const long ID_CHECKBOX1;
        static const long ID_CHECKBOX2;
        static const long ID_CHECKBOX3;
        static const long ID_CHECKBOX4;
        static const long ID_CHECKBOX5;
        static const long ID_RADIOBOX1;
        static const long ID_PANEL6;
        static const long ID_TEXTCTRL1;
        static const long ID_PANEL2;
        static const long ID_TEXTCTRL4;
        static const long ID_PANEL4;
        static const long ID_TEXTCTRL3;
        static const long ID_PANEL3;
        static const long ID_NOTEBOOK1;
        static const long ID_PANEL1;
        static const long ID_STATICTEXT3;
        static const long ID_TEXTCTRL5;
        static const long ID_PANEL7;
        static const long ID_STATICTEXT2;
        static const long ID_STATICTEXT1;
        static const long ID_XY_GLCANVAS;
        static const long ID_STATICTEXT4;
        static const long ID_CHOICE1;
        static const long ID_CUSTOM1;
        static const long ID_TEXTCTRL2;
        static const long ID_PANEL8;
        static const long ID_SPLITTERWINDOW2;
        static const long ID_PANEL5;
        static const long ID_SPLITTERWINDOW1;
        static const long ID_MENUITEM4;
        static const long idMenuQuit;
        static const long ID_MENUITEM1;
        static const long ID_MENUITEM2;
        static const long ID_MENUITEM3;
        static const long idMenuAbout;
        static const long ID_STATUSBAR1;
        static const long ID_TIMER1;
        //*)

        //(*Declarations(reactive_navigator_demoframe)
        wxTimer timRunSimul;
        wxTextCtrl* edInfoLocalView;
        wxPanel* Panel5;
        wxCheckBox* cbWaypointsAllowSkip;
        CMyGLCanvas* m_plotLocalView;
        wxTextCtrl* edParamsReactive;
        wxCustomButton* btnStop;
        wxNotebook* Notebook1;
        wxChoice* cbSelPTG;
        wxMenuItem* MenuItem5;
        wxTextCtrl* edManualSeqs;
        wxStaticText* StaticText2;
        wxTextCtrl* edParamsGeneral;
        wxRadioBox* rbKinType;
        wxCustomButton* btnStart;
        wxPanel* pnParamsReactive;
        wxCheckBox* cbDrawShapePath;
        wxRadioBox* rbNavMode;
        wxMenu* Menu3;
        wxSplitterWindow* SplitterWindow2;
        wxCustomButton* btnLoadMap;
        wxCustomButton* btnQuit;
        wxPanel* Panel1;
        wxStaticText* StaticText1;
        wxCustomButton* btnSetWaypointSeq;
        wxStaticText* StaticText3;
        wxPanel* pnParamsGeneral;
        wxCustomButton* btnDrawMapObs;
        wxCustomButton* btnEmptyMap;
        wxPanel* Panel3;
        wxCheckBox* cbNavLog;
        wxMenuItem* MenuItem3;
        wxMenuItem* mnuViewRobotPath;
        wxCustomButton* btnDrawEmpty;
        wxTextCtrl* edLog;
        wxCheckBox* cbShowPredictedPTG;
        wxStatusBar* StatusBar1;
        wxPanel* pnParamsPreprog;
        wxCustomButton* btnHelp;
        wxCustomButton* btnPlaceRobot;
        wxCustomButton* btnSaveMap;
        wxPanel* Panel2;
        wxSplitterWindow* SplitterWindow1;
        wxMenuItem* mnuViewMaxRange;
        wxStaticText* StaticText4;
        wxCustomButton* btnPlaceTarget;
        wxPanel* pnNavSelButtons;
        wxCheckBox* cbEnableLog;
        CMyGLCanvas* m_plot3D;
        //*)

        DECLARE_EVENT_TABLE()

		/* Methods: */
		void updateMap3DView();
		bool reinitSimulator();  // Create navigator object & load params from GUI. Return false on error
		void simulateOneStep(double time_step);
		void updateViewsDynamicObjects(); // Update 3D object positions and refresh views.

		void updateButtonsEnableState(bool is_running);

		void Onplot3DMouseClick(wxMouseEvent& event);
		void Onplot3DMouseMove(wxMouseEvent& event);

		/* Vars: */
		struct TOptions : public mrpt::utils::CLoadableOptions
		{
			double MAX_SENSOR_RADIUS;
			double SENSOR_FOV;
			uint64_t SENSOR_NUM_RANGES;
			double SENSOR_RANGE_NOISE_STD;
			double SENSOR_RATE;
			double NAVIGATION_RATE;


			TOptions();
			void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section) MRPT_OVERRIDE; // See base docs
			void saveToConfigFile(mrpt::utils::CConfigFileBase &source,const std::string &section) const MRPT_OVERRIDE; // See base docs
		};

		TOptions m_simul_options;

		/**  The state of the cursor onto the 3D view */
		enum TCursorPickState
		{
			cpsNone = 0,
			cpsPickTarget,
			cpsPlaceRobot,
			cpsDrawObstacles,
			cpsDrawClear,
			cpsPickWaypoints
		};

		mrpt::maps::COccupancyGridMap2D  m_gridMap;
		mrpt::math::TPoint2D             m_targetPoint;
		bool                             m_is_running; //!< is simulator running or paused?
		mrpt::maps::CSimplePointsMap     m_latest_obstacles;
		mrpt::nav::TWaypointSequence     m_waypoints_clicked;

		struct MyNavIFBase
		{
			MyNavIFBase(mrpt::maps::CSimplePointsMap &ref_latest_obstacles ) : latest_obstacles(ref_latest_obstacles) 
			{
			}
			mrpt::maps::CSimplePointsMap & latest_obstacles;
		};

		class MyRobot2NavInterface_Diff : public mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven, public MyNavIFBase
		{
		public:
			MyRobot2NavInterface_Diff(mrpt::kinematics::CVehicleSimul_DiffDriven &simul,mrpt::maps::CSimplePointsMap &ref_latest_obstacles) : 
				mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven(simul),
				MyNavIFBase(ref_latest_obstacles)
			{}
			bool senseObstacles(mrpt::maps::CSimplePointsMap &obstacles, mrpt::system::TTimeStamp &timestamp) MRPT_OVERRIDE {
				obstacles = latest_obstacles;
				timestamp = mrpt::system::now();
				return true;
			}
			bool changeSpeedsNOP() MRPT_OVERRIDE { return true; }
		};
		class MyRobot2NavInterface_Holo : public mrpt::nav::CRobot2NavInterfaceForSimulator_Holo, public MyNavIFBase
		{
		public:
			MyRobot2NavInterface_Holo(mrpt::kinematics::CVehicleSimul_Holo &simul,mrpt::maps::CSimplePointsMap &ref_latest_obstacles) :  
				mrpt::nav::CRobot2NavInterfaceForSimulator_Holo(simul),
				MyNavIFBase(ref_latest_obstacles)
			{}
			bool senseObstacles( mrpt::maps::CSimplePointsMap &obstacles, mrpt::system::TTimeStamp &timestamp ) MRPT_OVERRIDE {
				obstacles = latest_obstacles;
				timestamp = mrpt::system::now();
				return true;
			}
			bool changeSpeedsNOP() MRPT_OVERRIDE { return true; }
		};

		std::unique_ptr<mrpt::nav::CAbstractNavigator>  m_navMethod;

		std::unique_ptr<mrpt::nav::CRobot2NavInterface> m_robotSimul2NavInterface;
		std::unique_ptr<mrpt::kinematics::CVehicleSimulVirtualBase> m_robotSimul;

		mrpt::utils::CTicTac             m_runtime; // just for animations, this is not robot time
		mrpt::math::TPoint2D             m_curCursorPos; //!< Of the cursor on the 3D view (in world coordinates at Z=0)
		TCursorPickState                 m_cursorPickState;   //!< The state of the cursor onto the 3D view:

		mrpt::utils::CFileOutputStream   m_log_trajectory_file;

		CMyRedirector *m_myRedirector;

		// ========= Opengl View: Map & robot  =======
		mrpt::opengl::CSetOfObjectsPtr		gl_grid;
		mrpt::opengl::CSetOfObjectsPtr		gl_robot,gl_robot_local, gl_target;
		mrpt::opengl::CSetOfObjectsPtr		m_gl_placing_nav_target, m_gl_placing_robot, m_gl_drawing_obs;
		mrpt::opengl::CDiskPtr		        gl_robot_sensor_range;
		mrpt::opengl::CSetOfLinesPtr        gl_robot_path;
		mrpt::opengl::CPlanarLaserScanPtr   gl_scan3D;
		mrpt::opengl::CSetOfLinesPtr        gl_robot_ptg_prediction;
		mrpt::opengl::CSetOfObjectsPtr      gl_waypoints_clicking, gl_waypoints_status;

		// ========= Opengl View: Local view (holonomic)  =======
		mrpt::opengl::CSimpleLinePtr        gl_line_direction;
		mrpt::opengl::CPointCloudPtr        gl_rel_target, gl_rel_robot;
		mrpt::opengl::CSetOfLinesPtr        gl_nd_gaps, gl_tp_obstacles;

};

#endif // HOLONOMIC_NAVIGATOR_DEMOMAIN_H
