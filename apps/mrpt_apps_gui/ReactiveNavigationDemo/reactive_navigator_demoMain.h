/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#ifndef HOLONOMIC_NAVIGATOR_DEMOMAIN_H
#define HOLONOMIC_NAVIGATOR_DEMOMAIN_H

//(*Headers(reactive_navigator_demoframe)
#include <wx/button.h>
#include <wx/checkbox.h>
#include <wx/choice.h>
#include <wx/frame.h>
#include <wx/menu.h>
#include <wx/notebook.h>
#include <wx/panel.h>
#include <wx/radiobox.h>
#include <wx/sizer.h>
#include <wx/splitter.h>
#include <wx/stattext.h>
#include <wx/statusbr.h>
#include <wx/textctrl.h>
#include <wx/things/toggle.h>
#include <wx/timer.h>

#include "MyGLCanvas.h"
//*)

#include <mrpt/gui/CMyRedirector.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/nav.h>
#include <mrpt/viz/CDisk.h>
#include <mrpt/viz/CPlanarLaserScan.h>  // It's in the lib mrpt-maps
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CSimpleLine.h>

#include <memory>  // unique_ptr<>

// JLBC: Unix X headers have these funny things...
#ifdef Button1
#undef Button1
#undef Button2
#undef Button3
#undef Button4
#undef Button5
#undef Button6
#undef Button7
#endif
// To avoid conflicts between Eigen & X11 headers
#ifdef Success
#undef Success
#endif

class reactive_navigator_demoframe : public wxFrame
{
 public:
  reactive_navigator_demoframe(wxWindow* parent, wxWindowID id = -1);
  ~reactive_navigator_demoframe() override;

 private:
  //(*Handlers(reactive_navigator_demoframe)
  void OnQuit([[maybe_unused]] wxCommandEvent& event);
  void OnAbout([[maybe_unused]] wxCommandEvent& event);
  void OnbtnPlaceRobotClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnPlaceTargetClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnStartClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnStopClick([[maybe_unused]] wxCommandEvent& event);
  void OntimRunSimulTrigger(wxTimerEvent& event);
  void OnMenuItemChangeVisibleStuff([[maybe_unused]] wxCommandEvent& event);
  void OnMenuItemClearRobotPath([[maybe_unused]] wxCommandEvent& event);
  void OnbtnLoadMapClick([[maybe_unused]] wxCommandEvent& event);
  void OnNotebook1PageChanged(wxNotebookEvent& event);
  void OnNotebook1PageChanged1(wxNotebookEvent& event);
  void OnedManualKinRampsText([[maybe_unused]] wxCommandEvent& event);
  void OnbtnQuitClick([[maybe_unused]] wxCommandEvent& event);
  void OnrbKinTypeSelect([[maybe_unused]] wxCommandEvent& event);
  void OnbtnDrawMapObsClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnEmptyMapClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnSaveMapClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnDrawEmptyClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnSetWaypointSeqClick([[maybe_unused]] wxCommandEvent& event);
  void OnbtnGenerateTemplateClick([[maybe_unused]] wxCommandEvent& event);
  //*)

  //(*Identifiers(reactive_navigator_demoframe)
  static const wxWindowID ID_BUTTON4;
  static const wxWindowID ID_BUTTON5;
  static const wxWindowID ID_BUTTON7;
  static const wxWindowID ID_BUTTON12;
  static const wxWindowID ID_STATICTEXT10;
  static const wxWindowID ID_STATICTEXT9;
  static const wxWindowID ID_STATICTEXT8;
  static const wxWindowID ID_TEXTCTRL6;
  static const wxWindowID ID_BUTTON6;
  static const wxWindowID ID_BUTTON8;
  static const wxWindowID ID_BUTTON11;
  static const wxWindowID ID_BUTTON1;
  static const wxWindowID ID_BUTTON9;
  static const wxWindowID ID_BUTTON10;
  static const wxWindowID ID_BUTTON2;
  static const wxWindowID ID_BUTTON3;
  static const wxWindowID ID_RADIOBOX2;
  static const wxWindowID ID_CHECKBOX1;
  static const wxWindowID ID_CHECKBOX2;
  static const wxWindowID ID_CHECKBOX3;
  static const wxWindowID ID_CHECKBOX4;
  static const wxWindowID ID_CHECKBOX5;
  static const wxWindowID ID_RADIOBOX1;
  static const wxWindowID ID_BUTTON13;
  static const wxWindowID ID_PANEL6;
  static const wxWindowID ID_TEXTCTRL1;
  static const wxWindowID ID_PANEL2;
  static const wxWindowID ID_TEXTCTRL4;
  static const wxWindowID ID_PANEL4;
  static const wxWindowID ID_TEXTCTRL3;
  static const wxWindowID ID_PANEL3;
  static const wxWindowID ID_NOTEBOOK1;
  static const wxWindowID ID_PANEL1;
  static const wxWindowID ID_STATICTEXT3;
  static const wxWindowID ID_TEXTCTRL5;
  static const wxWindowID ID_PANEL7;
  static const wxWindowID ID_STATICTEXT2;
  static const wxWindowID ID_STATICTEXT1;
  static const wxWindowID ID_XY_GLCANVAS;
  static const wxWindowID ID_STATICTEXT4;
  static const wxWindowID ID_CHOICE1;
  static const wxWindowID ID_CUSTOM1;
  static const wxWindowID ID_TEXTCTRL2;
  static const wxWindowID ID_PANEL8;
  static const wxWindowID ID_SPLITTERWINDOW2;
  static const wxWindowID ID_PANEL5;
  static const wxWindowID ID_SPLITTERWINDOW1;
  static const wxWindowID ID_MENUITEM4;
  static const long idMenuQuit;
  static const wxWindowID ID_MENUITEM1;
  static const wxWindowID ID_MENUITEM2;
  static const wxWindowID ID_MENUITEM3;
  static const long idMenuAbout;
  static const wxWindowID ID_STATUSBAR1;
  static const wxWindowID ID_TIMER1;
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
  wxStaticText* StaticText6;
  wxSplitterWindow* SplitterWindow2;
  wxButton* btnGenerateTemplate;
  wxCustomButton* btnLoadMap;
  wxStaticText* StaticText8;
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
  wxStaticText* StaticText7;
  wxCheckBox* cbShowPredictedPTG;
  wxStatusBar* StatusBar1;
  wxPanel* pnParamsPreprog;
  wxCustomButton* btnHelp;
  wxCustomButton* btnPlaceRobot;
  wxCustomButton* btnSaveMap;
  wxPanel* Panel2;
  wxSplitterWindow* SplitterWindow1;
  wxTextCtrl* edWayPtHeading;
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
  bool reinitSimulator();  // Create navigator object & load params from GUI.
  // Return false on error
  void simulateOneStep(double time_step);
  void updateViewsDynamicObjects();  // Update 3D object positions and refresh
  // views.

  void updateButtonsEnableState(bool is_running);

  void Onplot3DMouseClick(wxMouseEvent& event);
  void Onplot3DMouseMove(wxMouseEvent& event);

  /* Vars: */
  struct TOptions : public mrpt::config::CLoadableOptions
  {
    double MAX_SENSOR_RADIUS{10.0};
    double SENSOR_FOV{M_PI * 2.0};
    uint64_t SENSOR_NUM_RANGES{181};
    double SENSOR_RANGE_NOISE_STD{0.02};
    double SENSOR_RATE{10.0};
    double NAVIGATION_RATE{4.0};

    TOptions() = default;
    void loadFromConfigFile(
        const mrpt::config::CConfigFileBase& source,
        const std::string& section) override;  // See base docs
    void saveToConfigFile(
        mrpt::config::CConfigFileBase& source,
        const std::string& section) const override;  // See base docs
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

  mrpt::maps::COccupancyGridMap2D m_gridMap;
  mrpt::math::TPoint2D m_targetPoint;
  /** is simulator running or paused? */
  bool m_is_running;
  mrpt::maps::CSimplePointsMap m_latest_obstacles;
  mrpt::nav::TWaypointSequence m_waypoints_clicked;

  struct MyNavIFBase
  {
    MyNavIFBase(mrpt::maps::CSimplePointsMap& ref_latest_obstacles) :
        latest_obstacles(ref_latest_obstacles)
    {
    }
    mrpt::maps::CSimplePointsMap& latest_obstacles;
  };

  class MyRobot2NavInterface_Diff :
      public mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven,
      public MyNavIFBase
  {
   public:
    MyRobot2NavInterface_Diff(
        mrpt::kinematics::CVehicleSimul_DiffDriven& simul,
        mrpt::maps::CSimplePointsMap& ref_latest_obstacles) :
        mrpt::nav::CRobot2NavInterfaceForSimulator_DiffDriven(simul),
        MyNavIFBase(ref_latest_obstacles)
    {
    }
    bool senseObstacles(
        mrpt::maps::CSimplePointsMap& obstacles, mrpt::system::TTimeStamp& timestamp) override
    {
      obstacles = latest_obstacles;
      timestamp = mrpt::Clock::now();
      return true;
    }
    bool changeSpeedsNOP() override { return true; }
  };
  class MyRobot2NavInterface_Holo :
      public mrpt::nav::CRobot2NavInterfaceForSimulator_Holo,
      public MyNavIFBase
  {
   public:
    MyRobot2NavInterface_Holo(
        mrpt::kinematics::CVehicleSimul_Holo& simul,
        mrpt::maps::CSimplePointsMap& ref_latest_obstacles) :
        mrpt::nav::CRobot2NavInterfaceForSimulator_Holo(simul), MyNavIFBase(ref_latest_obstacles)
    {
    }
    bool senseObstacles(
        mrpt::maps::CSimplePointsMap& obstacles, mrpt::system::TTimeStamp& timestamp) override
    {
      obstacles = latest_obstacles;
      timestamp = mrpt::Clock::now();
      return true;
    }
    bool changeSpeedsNOP() override { return true; }
  };

  std::unique_ptr<mrpt::nav::CAbstractNavigator> m_navMethod;

  std::unique_ptr<mrpt::nav::CRobot2NavInterface> m_robotSimul2NavInterface;
  std::unique_ptr<mrpt::kinematics::CVehicleSimulVirtualBase> m_robotSimul;

  mrpt::system::CTicTac m_runtime;  // just for animations, this is not robot time
  /** Of the cursor on the 3D view (in world coordinates at Z=0) */
  mrpt::math::TPoint2D m_curCursorPos;
  /** The state of the cursor onto the 3D view: */
  TCursorPickState m_cursorPickState;

  mrpt::io::CFileOutputStream m_log_trajectory_file;

  std::unique_ptr<CMyRedirector> m_myRedirector;

  // ========= Opengl View: Map & robot  =======
  mrpt::viz::CSetOfObjects::Ptr gl_grid;
  mrpt::viz::CSetOfObjects::Ptr gl_robot, gl_robot_local, gl_target;
  mrpt::viz::CSetOfObjects::Ptr m_gl_placing_nav_target, m_gl_placing_robot, m_gl_drawing_obs;
  mrpt::viz::CDisk::Ptr gl_robot_sensor_range;
  mrpt::viz::CSetOfLines::Ptr gl_robot_path;
  mrpt::viz::CPlanarLaserScan::Ptr gl_scan3D;
  mrpt::viz::CSetOfLines::Ptr gl_robot_ptg_prediction;
  mrpt::viz::CSetOfObjects::Ptr gl_waypoints_clicking, gl_waypoints_status;

  // ========= Opengl View: Local view (holonomic)  =======
  mrpt::viz::CSimpleLine::Ptr gl_line_direction;
  mrpt::viz::CPointCloud::Ptr gl_rel_target, gl_rel_robot;
  mrpt::viz::CSetOfLines::Ptr gl_nd_gaps, gl_tp_obstacles;
};

#endif  // HOLONOMIC_NAVIGATOR_DEMOMAIN_H
